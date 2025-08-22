// ESP32 WROOM-DA Power Backup System
// Implements requirements from "Power_system_master_code_instruction.txt"
// Toolchain: Arduino ESP32 core
//
// Multi-core, thread-safe tasks for:
// - Button debounce & power switch relay "refresh"
// - Power-outage input monitor (pin 19)
// - UART2 poller for measurements (RX=21, TX=22) using "GET\n" and manual JSON parsing
// - Battery arbiter + two per-battery relay controllers with dwell/rest timing, SOC rules
// - NVS persistence for global state & counters
//
// -------- Pin Map --------
//  18: Momentary pushbutton (debounced) toggles global "power switch" state
//  19: Power-good sense (HIGH = normal; LOW = outage) -> global "powerOutage" state
//  23: Power switch relay output mirrors global power switch state; auto "refresh"
//  21: UART2 RX
//  22: UART2 TX
//  25: Battery 1 relay control (switches between CHARGER / INVERTER)
//  26: Battery 2 relay control (switches between CHARGER / INVERTER)
//   4: (reserved for CAN)  -- not used
//   5: (reserved for CAN)  -- not used
//
// -------- Behavioral Notes --------
// - Only one battery may be connected to CHARGER at any time (unless power outage).
// - At least one battery must be connected to INVERTER at all times.
// - On power outage (pin 19 LOW), immediately connect BOTH batteries to INVERTER.
// - "Start charging the lower-voltage battery" (by dc_v1/dc_v2). Stay charging it
//   until the OTHER battery's SOC <= chargeSwitchingSOC (default 0%).
// - If a battery becomes "offline/damage" (V < 10.0V), switch THAT battery to CHARGER,
//   preparing to charge it (while keeping at least one on INVERTER).
// - If both go offline, the one detected later stays on INVERTER, the other goes to CHARGER.
// - Charger rest time between disconnecting one battery and connecting any battery is 7s.
// - Minimum dwell time while a battery is CHARGER-connected is 10s.
// - "Battery full" when |dc_i| < FULL_CURRENT_THRESH for FULL_STABLE_MS while that battery is charging.
//   Each time a battery becomes full, increment its counter (persisted).
//
// -------- CLI (USB Serial, 115200) --------
//   S=<n>    -> set chargeSwitchingSOC to n (0,25,50,75,100)
//   STATE    -> print concise state
//
// ------------------------------------------

#include <Arduino.h>
#include <Preferences.h>

// ===== Pins =====
static const int PIN_BTN          = 18;
static const int PIN_PWR_GOOD     = 19; // HIGH = normal, LOW = outage
static const int PIN_PWR_RELAY    = 23; // mirrors global powerSwitchState

// UART2
static const int PIN_UART2_RX     = 21;
static const int PIN_UART2_TX     = 22;

// Battery relays
static const int PIN_BAT1_RELAY   = 25; // Battery 1: charger/inverter switch
static const int PIN_BAT2_RELAY   = 26; // Battery 2: charger/inverter switch

// Reserved CAN pins (unused, left as inputs)
static const int PIN_CAN_RX       = 4;
static const int PIN_CAN_TX       = 5;

// ===== Relay polarity assumptions =====
// Adjust these if your relays are active-low.
// Here: HIGH -> connect to CHARGER, LOW -> connect to INVERTER
static const bool RELAY_ACTIVE_HIGH = true;

// ===== Timing constants =====
static const uint32_t BTN_SAMPLE_MS       = 10;    // debounce sampling
static const uint32_t BTN_DEBOUNCE_MS     = 50;    // stable press time
static const uint32_t UART_POLL_MS        = 100;   // "GET\n" cadence
static const uint32_t CHARGER_REST_MS     = 7000;  // 7 sec rest between connections
static const uint32_t CHARGE_MIN_DWELL_MS = 10000; // 10 sec min dwell on charger
static const uint32_t FULL_STABLE_MS      = 5000;  // current below threshold for this long => full

// ===== Full detection threshold (Amps) =====
static const float FULL_CURRENT_THRESH    = 0.5f;  // tweak as needed

// ===== SOC thresholds (Volts) =====
static const float V_OFFLINE              = 10.0f;
static const float V_SOC0_MIN             = 10.0f;  // >=10.0 and <12.0 => 0%
static const float V_SOC25_MIN            = 12.0f;  // >=12.0 and <13.15 => 25%
static const float V_SOC50_MIN            = 13.15f; // >=13.15 and <13.20 => 50%
static const float V_SOC75_MIN            = 13.20f; // >=13.20 and <13.33 => 75%
static const float V_SOC100_MIN           = 13.33f; // >=13.33 => 100%

// ===== Task cores =====
static const BaseType_t CORE_IO           = 1;
static const BaseType_t CORE_UART         = 0;
static const BaseType_t CORE_BATT         = 1;

// ===== NVS keys =====
Preferences prefs;
static const char* NVS_NS                = "pwr-backup";
static const char* KEY_PWR_SW            = "pwrSw";
static const char* KEY_CHG_SOC           = "chgSoc";
static const char* KEY_CNT0              = "cnt0";
static const char* KEY_CNT1              = "cnt1";
static const char* KEY_LAST_CHARGING     = "lastChg"; // -1, 0, or 1

// ===== Measurements =====
struct Measurements {
  float ac_v1 = 0, ac_v2 = 0, ac_i1 = 0, ac_i2 = 0;
  float dc_v1 = 0, dc_v2 = 0, dc_i  = 0; // shared "dc_i"
  uint32_t lastUpdateMs = 0;
};

// Forward-declare parse helpers (from your working code)
struct Measurements;
bool extractFloat(const String& json, const char* key, float& out);
bool parseMeasurements(const String& json, Measurements& m);

// ===== Shared state & concurrency =====
Measurements g_meas;
SemaphoreHandle_t g_stateMutex;

#define LOCK()   xSemaphoreTake(g_stateMutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(g_stateMutex)

// Global power switch state (toggled by button) & power outage state (pin19)
volatile bool g_powerSwitchState = false; // persisted
volatile bool g_powerOutage      = false; // HIGH=normal(false), LOW=outage(true)
volatile bool g_requestTurnOff   = false; // <<< ADDED: set on outage, cleared on power return

// Battery connection enum
enum Conn : uint8_t { TO_INVERTER = 0, TO_CHARGER = 1 };

// Per-battery state
struct BatteryState {
  Conn desired = TO_INVERTER;   // set by arbiter
  Conn actual  = TO_INVERTER;   // commanded to hardware

  uint32_t lastSwitchMs = 0;    // last relay toggle (any direction)
  uint32_t chargeStartMs = 0;   // when connected to CHARGER
  uint32_t dcILowStartMs = 0;   // for "full" detection timing

  bool offline = false;         // V < 10.0
  uint32_t offlineDetectedMs = 0;

  uint8_t soc = 0;              // 0,25,50,75,100
  bool fullFlag = false;        // per-battery "was full"

  uint32_t fullCount = 0;       // persisted counter
};

// Two batteries
BatteryState g_batt[2];

// Which battery is connected to CHARGER (-1 if none)
int g_chargingBattery = -1;

// Global "battery full" state (true if whichever is charging has just qualified)
volatile bool g_globalBatteryFull = false;

// Charger rest window; next time when it's legal to connect ANY battery to CHARGER
uint32_t g_chargerAvailableAt = 0;

// Charge-switching SOC (debuggable). Default 0%.
uint8_t g_chargeSwitchingSOC = 0;

// ========= Helpers =========

uint8_t socFromVoltage(float v) {
  if (v < V_OFFLINE) return 255; // special marker for offline/damage
  if (v >= V_SOC100_MIN) return 100;
  if (v >= V_SOC75_MIN)  return 75;
  if (v >= V_SOC50_MIN)  return 50;
  if (v >= V_SOC25_MIN)  return 25;
  if (v >= V_SOC0_MIN)   return 0;
  return 0; // below 10.0 already handled as offline
}

void persistCountsAndSettings() {
  prefs.putBool(KEY_PWR_SW, g_powerSwitchState);
  prefs.putUChar(KEY_CHG_SOC, g_chargeSwitchingSOC);
  prefs.putULong(KEY_CNT0, g_batt[0].fullCount);
  prefs.putULong(KEY_CNT1, g_batt[1].fullCount);
  prefs.putChar(KEY_LAST_CHARGING, (char)g_chargingBattery);
}

void loadPersisted() {
  g_powerSwitchState = prefs.getBool(KEY_PWR_SW, false);
  g_chargeSwitchingSOC = prefs.getUChar(KEY_CHG_SOC, 0);
  g_batt[0].fullCount = prefs.getULong(KEY_CNT0, 0);
  g_batt[1].fullCount = prefs.getULong(KEY_CNT1, 0);
  g_chargingBattery = (int8_t)prefs.getChar(KEY_LAST_CHARGING, -1);
}

// Relay drive
void setRelayPin(int pin, Conn c) {
  bool level = (c == TO_CHARGER) ? RELAY_ACTIVE_HIGH : !RELAY_ACTIVE_HIGH;
  digitalWrite(pin, level ? HIGH : LOW);
}

// Return current millis() safely
uint32_t nowMs() { return (uint32_t)millis(); }

// Debug dump
void printStateBrief() {
  LOCK();
  Serial.printf("\n--- STATE --- @%lu ms\n", nowMs());
  Serial.printf("PwrSwitch=%s  PwrOutage=%s  requestTurnOff=%s  chgSOC=%u  chargerAvailAt=%lu\n", // <<< MODIFIED
                g_powerSwitchState ? "ON" : "OFF",
                g_powerOutage ? "OUTAGE" : "NORMAL",
                g_requestTurnOff ? "YES" : "NO", // <<< ADDED
                g_chargeSwitchingSOC, g_chargerAvailableAt);
  Serial.printf("Meas: dc_v1=%.2f  dc_v2=%.2f  dc_i=%.2f  (t=%lu)\n",
                g_meas.dc_v1, g_meas.dc_v2, g_meas.dc_i, g_meas.lastUpdateMs);
  for (int i = 0; i < 2; ++i) {
    const BatteryState &b = g_batt[i];
    Serial.printf("B%d: desired=%s actual=%s soc=%s%s full=%s cnt=%lu lastSw=%lu chgStart=%lu\n",
      i+1,
      b.desired==TO_CHARGER?"CHG":"INV",
      b.actual==TO_CHARGER?"CHG":"INV",
      (b.soc==255?"OFFLINE":String(b.soc).c_str()),
      (b.offline?"/OFFLINE":""), b.fullFlag?"Y":"N", b.fullCount,
      b.lastSwitchMs, b.chargeStartMs
    );
  }
  Serial.printf("ChargingBattery=%d  GlobalFull=%s\n",
                g_chargingBattery, g_globalBatteryFull?"Y":"N");
  UNLOCK();
}

// ========= UART2 JSON parsing (your working code) =========
bool extractFloat(const String& json, const char* key, float& out) {
  String pat = String("\"") + key + String("\":");
  int i = json.indexOf(pat);
  if (i < 0) return false;
  i += pat.length();
  while (i < (int)json.length() && json[i] == ' ') i++;
  int j = i;
  while (j < (int)json.length() && json[j] != ',' && json[j] != '}') j++;
  if (j <= i) return false;
  out = json.substring(i, j).toFloat();
  return true;
}

bool parseMeasurements(const String& json, Measurements& m) {
  if (json.length() < 10 || json[0] != '{' || json[json.length()-1] != '}') return false;
  bool ok = true;
  ok &= extractFloat(json, "ac_v1", m.ac_v1);
  ok &= extractFloat(json, "ac_v2", m.ac_v2);
  ok &= extractFloat(json, "ac_i1", m.ac_i1);
  ok &= extractFloat(json, "ac_i2", m.ac_i2);
  ok &= extractFloat(json, "dc_v1", m.dc_v1);
  ok &= extractFloat(json, "dc_v2", m.dc_v2);
  ok &= extractFloat(json, "dc_i",  m.dc_i);
  return ok;
}

// --- Debug helper to print the two full-charge counters ---
void printChargeCounters() {                                  // <<< ADDED
  LOCK();                                                     // <<< ADDED
  Serial.printf("[COUNTS] Battery1=%lu  Battery2=%lu\n",      // <<< ADDED
                g_batt[0].fullCount, g_batt[1].fullCount);   // <<< ADDED
  UNLOCK();                                                   // <<< ADDED
}                                                             // <<< ADDED

// ========= Tasks =========

// --- Task: Button debounce + Power relay refresh + CLI ---
void TaskIO(void* arg) {
  pinMode(PIN_BTN, INPUT_PULLUP); // assuming button pulls to GND when pressed
  pinMode(PIN_PWR_GOOD, INPUT);
  pinMode(PIN_PWR_RELAY, OUTPUT);

  // Init power relay to current persisted state
  digitalWrite(PIN_PWR_RELAY, g_powerSwitchState ? HIGH : LOW);

  // Button debounce state
  bool lastStable = digitalRead(PIN_BTN);
  bool lastRaw = lastStable;
  uint32_t lastChangeMs = nowMs();

  String cliBuf;

  for (;;) {

    // ----- Power good sense & outage transition handling -----
    bool pg = digitalRead(PIN_PWR_GOOD);
    bool outage = (pg == LOW);
    if (outage != g_powerOutage) {
      g_powerOutage = outage;
      if (g_powerOutage) {
        g_requestTurnOff = true;                 // <<< ADDED
        pinMode(PIN_BTN, INPUT);                 // <<< ADDED: disable pullup / input handling
      } else {
        g_requestTurnOff = false;                // <<< ADDED
        pinMode(PIN_BTN, INPUT_PULLUP);          // <<< ADDED: re-enable button
        // Re-sync debounce state after re-enabling input      // <<< ADDED
        lastStable = digitalRead(PIN_BTN);       // <<< ADDED
        lastRaw    = lastStable;                 // <<< ADDED
        lastChangeMs = nowMs();                  // <<< ADDED
      }
      Serial.printf("[PWR] Power %s\n", g_powerOutage ? "OUTAGE (LOW)" : "NORMAL (HIGH)");
    }

    // ----- Button sampling (skip entirely during outage) -----
    if (!g_powerOutage) {                        // <<< ADDED
      bool raw = digitalRead(PIN_BTN);
      if (raw != lastRaw) {
        lastRaw = raw;
        lastChangeMs = nowMs();
      } else {
        if ((nowMs() - lastChangeMs) >= BTN_DEBOUNCE_MS) {
          if (raw != lastStable) {
            lastStable = raw;
            // Active LOW press?
            if (lastStable == LOW) {
              g_powerSwitchState = !g_powerSwitchState;
              digitalWrite(PIN_PWR_RELAY, g_powerSwitchState ? HIGH : LOW);
              persistCountsAndSettings();
              Serial.printf("[BTN] PowerSwitch toggled -> %s\n",
                            g_powerSwitchState ? "ON" : "OFF");
            }
          }
        }
      }
    } // <<< ADDED

    // "Refresh" power relay to mirror global state
    int want = g_powerSwitchState ? HIGH : LOW;
    if (digitalRead(PIN_PWR_RELAY) != want) {
      digitalWrite(PIN_PWR_RELAY, want);
      Serial.println("[PWR] Relay refreshed to match switch state.");
    }

    // ----- Simple CLI from USB Serial -----
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (cliBuf.length()) {
          if (cliBuf.startsWith("S=") || cliBuf.startsWith("s=")) {
            int val = cliBuf.substring(2).toInt();
            if (val==0 || val==25 || val==50 || val==75 || val==100) {
              LOCK();
              g_chargeSwitchingSOC = (uint8_t)val;
              persistCountsAndSettings();
              UNLOCK();
              Serial.printf("[CLI] chargeSwitchingSOC set to %d\n", val);
            } else {
              Serial.println("[CLI] S must be one of 0,25,50,75,100");
            }
          } else if (cliBuf.equalsIgnoreCase("STATE")) {
            printStateBrief(); // now includes requestTurnOff  // <<< MODIFIED (implicitly shows new state)
          } else if (cliBuf.equalsIgnoreCase("COUNTS")) {      // <<< ADDED
              printChargeCounters();                           // <<< ADDED
          } else {
            Serial.println("[CLI] Unknown. Use: S=<0|25|50|75|100>  STATE  COUNTS"); // <<< MODIFIED
          }
          cliBuf = "";
        }
      } else {
        cliBuf += c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(BTN_SAMPLE_MS));
  }
}


// --- Task: UART2 poller + JSON parser ---
void TaskUART(void* arg) {
  Serial2.begin(115200, SERIAL_8N1, PIN_UART2_RX, PIN_UART2_TX);

  String line;
  uint32_t lastPoll = 0;

  for (;;) {
    uint32_t t = nowMs();

    // Periodically send GET\n
    if (t - lastPoll >= UART_POLL_MS) {
      Serial2.print("GET\n");
      lastPoll = t;
    }

    // Read response lines until '}'
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (c == '\n' || c == '\r') continue;
      line += c;
      if (c == '}') {
        // Attempt parse
        Measurements m;
        if (parseMeasurements(line, m)) {
          m.lastUpdateMs = nowMs();
          LOCK();
          g_meas = m;
          UNLOCK();

          // Debug print
          //Serial.printf("[MEAS] ac_v1=%.2f ac_v2=%.2f ac_i1=%.2f ac_i2=%.2f  dc_v1=%.2f dc_v2=%.2f dc_i=%.2f\n",
          //              m.ac_v1, m.ac_v2, m.ac_i1, m.ac_i2, m.dc_v1, m.dc_v2, m.dc_i);
        } else {
          Serial.printf("[MEAS] Parse failed: %s\n", line.c_str());
        }
        line = "";
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// --- Task: Battery arbiter ---
// Decides desired connections based on SOC, rules, outage override, dwell/rest.
void TaskArbiter(void* arg) {
  for (;;) {
    LOCK();

    // Update SOC/offline flags from current measurements
    float v1 = g_meas.dc_v1;
    float v2 = g_meas.dc_v2;

    uint8_t s1 = socFromVoltage(v1);
    uint8_t s2 = socFromVoltage(v2);

    auto updBatt = [&](int idx, float v, uint8_t s) {
      BatteryState &b = g_batt[idx];
      bool wasOffline = b.offline;
      b.offline = (s == 255);
      b.soc = (s == 255) ? 0 : s;

      if (b.offline && !wasOffline) {
        b.offlineDetectedMs = nowMs();
      }
      if (!b.offline && wasOffline) {
        // Came back from offline; clear timestamp
        b.offlineDetectedMs = 0;
      }
    };

    updBatt(0, v1, s1);
    updBatt(1, v2, s2);

    // Outage override
    if (g_powerOutage) {
      g_batt[0].desired = TO_INVERTER;
      g_batt[1].desired = TO_INVERTER;
      g_chargingBattery = -1; // nobody is charging during outage
      // No need to touch rest/dwell here; battery tasks will force to inverter immediately.
      UNLOCK();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // If either is offline, prepare it for charging (per 11)
    bool b0_off = g_batt[0].offline;
    bool b1_off = g_batt[1].offline;

    if (b0_off || b1_off) {
      if (b0_off && b1_off) {
        // Both offline: one detected later stays INVERTER (per 12)
        if (g_batt[0].offlineDetectedMs > g_batt[1].offlineDetectedMs) {
          g_batt[0].desired = TO_INVERTER;
          g_batt[1].desired = TO_CHARGER;
          g_chargingBattery = 1;
        } else {
          g_batt[0].desired = TO_CHARGER;
          g_batt[1].desired = TO_INVERTER;
          g_chargingBattery = 0;
        }
      } else if (b0_off) {
        // B0 offline => prepare to CHARGER, keep at least one on INVERTER
        g_batt[0].desired = TO_CHARGER;
        g_batt[1].desired = TO_INVERTER;
        g_chargingBattery = 0;
      } else { // b1_off
        g_batt[0].desired = TO_INVERTER;
        g_batt[1].desired = TO_CHARGER;
        g_chargingBattery = 1;
      }
      // Persist "last charging" for recovery
      persistCountsAndSettings();
      UNLOCK();
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Normal case: no outage, neither offline.
    // Rule (10): Start charging lower voltage battery (regardless SOC) and
    // stay charging it until the OTHER battery's SOC <= chargeSwitchingSOC.
    // Also enforce: only one on CHARGER; at least one on INVERTER.
    float dv1 = g_meas.dc_v1;
    float dv2 = g_meas.dc_v2;

    if (g_chargingBattery == -1) {
      // Choose lower-V to charge
      int pick = (dv1 <= dv2) ? 0 : 1;
      g_chargingBattery = pick;
      g_batt[pick].desired = TO_CHARGER;
      g_batt[1 - pick].desired = TO_INVERTER;
      persistCountsAndSettings();
    } else {
      int chg = g_chargingBattery;
      int oth = 1 - chg;

      // Check switch condition: if OTHER battery's SOC <= chargeSwitchingSOC, switch charger to OTHER
      if (g_batt[oth].soc <= g_chargeSwitchingSOC) {
        // set desired: other->CHARGER, current->INVERTER
        g_batt[oth].desired = TO_CHARGER;
        g_batt[chg].desired = TO_INVERTER;
        g_chargingBattery = oth;
        persistCountsAndSettings();
      } else {
        // Maintain current roles
        g_batt[chg].desired = TO_CHARGER;
        g_batt[oth].desired = TO_INVERTER;
      }
    }

    UNLOCK();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- Common per-battery enforcement (relay control with dwell/rest/full detect) ---
void handleBatteryRelay(int idx, int pin) {
  LOCK();
  BatteryState &b = g_batt[idx];
  BatteryState &o = g_batt[1 - idx];
  Conn want = b.desired;
  Conn have = b.actual;
  uint32_t t = nowMs();

  // If outage => force inverter immediately
  if (g_powerOutage) {
    if (have != TO_INVERTER) {
      setRelayPin(pin, TO_INVERTER);
      b.actual = TO_INVERTER;
      b.lastSwitchMs = t;
      Serial.printf("[B%d] OUTAGE -> set INVERTER\n", idx+1);
    }
    UNLOCK();
    return;
  }

  // Enforce "only one on CHARGER"
  if (want == TO_CHARGER && o.actual == TO_CHARGER) {
    // Another battery is currently charger-connected; defer
    UNLOCK();
    return;
  }

  // If moving FROM CHARGER to INVERTER: respect min dwell time unless override condition requires immediate (offline/outage handled earlier).
  if (have == TO_CHARGER && want == TO_INVERTER) {
    if ((t - b.chargeStartMs) < CHARGE_MIN_DWELL_MS) {
      // Not yet allowed to leave charger
      UNLOCK();
      return;
    }
    // Disconnect from charger, start global charger rest timer
    setRelayPin(pin, TO_INVERTER);
    b.actual = TO_INVERTER;
    b.lastSwitchMs = t;

    // Start rest window (applies to connecting ANY battery to charger next)
    g_chargerAvailableAt = t + CHARGER_REST_MS;
    Serial.printf("[B%d] -> INVERTER (leaving CHARGER, dwell ok). Rest until %lu\n", idx+1, g_chargerAvailableAt);
    UNLOCK();
    return;
  }

  // If moving TO CHARGER: ensure rest window elapsed, and the other is on INVERTER.
  if (have == TO_INVERTER && want == TO_CHARGER) {
    if (t < g_chargerAvailableAt) {
      // Wait for charger rest
      UNLOCK();
      return;
    }
    if (o.actual != TO_INVERTER) {
      // Ensure counterpart stays inverter
      UNLOCK();
      return;
    }
    // Connect to charger
    setRelayPin(pin, TO_CHARGER);
    b.actual = TO_CHARGER;
    b.lastSwitchMs = t;
    b.chargeStartMs = t;
    b.dcILowStartMs = 0; // reset full-detect timing
    b.fullFlag = false;   // <<< FIX: allow a new "full" to be counted this session
    g_globalBatteryFull = false; // reset global flag when new charging cycle starts
    Serial.printf("[B%d] -> CHARGER (rest ok)\n", idx+1);
    UNLOCK();
    return;
  }

  // Maintaining current state: check "full" detection if on charger using shared dc_i
  if (have == TO_CHARGER) {
    float dc_i = g_meas.dc_i;
    // Consider magnitude in case of sign variations
    if (fabsf(dc_i) < FULL_CURRENT_THRESH) {
      if (b.dcILowStartMs == 0) b.dcILowStartMs = t;
      if ((t - b.dcILowStartMs) >= FULL_STABLE_MS && !b.fullFlag) {
        b.fullFlag = true;
        b.dcILowStartMs = t;
        b.fullCount++;
        g_globalBatteryFull = true;
        persistCountsAndSettings();
        Serial.printf("[B%d] FULL detected (dc_i=%.2f). Count=%lu\n", idx+1, dc_i, b.fullCount);
      }
    } else {
      b.dcILowStartMs = 0;
    }
  }

  UNLOCK();
}

// --- Task: Battery 1 control ---
void TaskBatt1(void* arg) {
  pinMode(PIN_BAT1_RELAY, OUTPUT);
  setRelayPin(PIN_BAT1_RELAY, TO_INVERTER); // safe default
  g_batt[0].actual = TO_INVERTER;

  for (;;) {
    handleBatteryRelay(0, PIN_BAT1_RELAY);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --- Task: Battery 2 control ---
void TaskBatt2(void* arg) {
  pinMode(PIN_BAT2_RELAY, OUTPUT);
  setRelayPin(PIN_BAT2_RELAY, TO_INVERTER); // safe default
  g_batt[1].actual = TO_INVERTER;

  for (;;) {
    handleBatteryRelay(1, PIN_BAT2_RELAY);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ========= Setup / Loop =========
void setup() {
  Serial.begin(115200);
  delay(100);

  // Reserved pins
  pinMode(PIN_CAN_RX, INPUT);
  pinMode(PIN_CAN_TX, INPUT);

  // NVS
  prefs.begin(NVS_NS, false);
  loadPersisted();

  // MutexF
  g_stateMutex = xSemaphoreCreateMutex();

  // Show persisted
  Serial.printf("Boot: powerSwitch=%s  chargeSwitchingSOC=%u  cnt0=%lu  cnt1=%lu  lastCharging=%d\n",
                g_powerSwitchState?"ON":"OFF", g_chargeSwitchingSOC, g_batt[0].fullCount,
                g_batt[1].fullCount, g_chargingBattery);

  // Tasks
  xTaskCreatePinnedToCore(TaskIO,     "TaskIO",     4096, nullptr, 2, nullptr, CORE_IO);
  xTaskCreatePinnedToCore(TaskUART,   "TaskUART",   4096, nullptr, 3, nullptr, CORE_UART);
  xTaskCreatePinnedToCore(TaskArbiter,"TaskArbiter",4096, nullptr, 2, nullptr, CORE_BATT);
  xTaskCreatePinnedToCore(TaskBatt1,  "TaskBatt1",  4096, nullptr, 2, nullptr, CORE_BATT);
  xTaskCreatePinnedToCore(TaskBatt2,  "TaskBatt2",  4096, nullptr, 2, nullptr, CORE_BATT);
}

void loop() {
  // Nothing here; all work is done in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
