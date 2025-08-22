/*
  ESP32-WROOM-DA Power Monitor
  Core 0: High-rate sampling + RMS/avg computation
  Core 1: OLED display + UART command handling

  Sensors (7 total):
    AC Voltage (RMS):  ZMPT101B  -> GPIO32, GPIO33
    AC Current (RMS):  ACS712-20A -> GPIO34, GPIO35
    DC Voltage (avg):  5:1 divider -> GPIO36 (VP), GPIO39 (VN)
    DC Current (avg):  ACS712-20A -> GPIO25

  OLED: Adafruit_SSD1305, 128x32, SPI
    MOSI 23, CLK 18, DC 16, CS 5, RST 17

  UART: Send "GET" or "R" to receive one-line JSON with all 7 values.
*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1305.h>


#include <Power_System.h>

// =================== STATE (shared between cores) ===================
typedef struct {
  // computed values
  float ac_v_rms_1;
  float ac_v_rms_2;
  float ac_i_rms_1;
  float ac_i_rms_2;
  float dc_v_1;
  float dc_v_2;
  float dc_i;       // average (A)

  // raw offsets (learned or fixed)
  float zmpt_mean_v1;
  float zmpt_mean_v2;
  float acs_mean_i1;
  float acs_mean_i2;

  // timing/diag
  uint32_t last_update_ms;
} Measurements;

volatile Measurements g_meas = {0};

// Protect shared state
SemaphoreHandle_t g_measMutex;

// =================== OLED ===================
Adafruit_SSD1305 display(OLED_W, OLED_H, &SPI, OLED_DC, OLED_RST, OLED_CS);

// =================== UTILS ===================
static inline void lockMeas()   { xSemaphoreTake(g_measMutex, portMAX_DELAY); }
static inline void unlockMeas() { xSemaphoreGive(g_measMutex); }

// Running mean (for AC offset tracking)
struct RunningMean {
  float mean = 0.0f;
  float alpha;  // 0..1, closer to 0 = slower
  RunningMean(float alpha_=0.001f) : alpha(alpha_) {}
  inline float update(float x) { mean += alpha * (x - mean); return mean; }
};

// =================== TASKS ===================
TaskHandle_t samplerTaskHandle = nullptr;
TaskHandle_t uiTaskHandle = nullptr;

// Core 0: high-rate sampling + RMS/avg calculations
void samplerTask(void* pv) {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_DC_I, ADC_11db);

  for (int i=0; i<100; ++i) { (void)analogRead(PIN_AC_I1); (void)analogRead(PIN_AC_I2); vTaskDelay(pdMS_TO_TICKS(2)); }

  RunningMean v1_mean(0.0005f), v2_mean(0.0005f), i1_mean(0.0005f), i2_mean(0.0005f);

  const uint32_t windowSamples = (uint32_t)(SAMPLE_RATE_HZ * RMS_WINDOW_SEC);
  const TickType_t tick = pdMS_TO_TICKS(1);                   // 1 ms cadence
  const uint32_t SAMPLES_PER_TICK = max<uint32_t>(1, SAMPLE_RATE_HZ / 1000);

  uint32_t n = 0, sinceYield = 0;
  double sumsq_v1=0, sumsq_v2=0, sumsq_i1=0, sumsq_i2=0;
  double sum_dc_v1=0, sum_dc_v2=0, sum_dc_i=0;

  TickType_t nextWake = xTaskGetTickCount();

  for (;;) {
    // --- take a small batch each millisecond ---
    for (uint32_t k=0; k<SAMPLES_PER_TICK; ++k) {
      uint16_t r_v1 = analogRead(PIN_AC_V1);
      uint16_t r_v2 = analogRead(PIN_AC_V2);
      uint16_t r_i1 = analogRead(PIN_AC_I1);
      uint16_t r_i2 = analogRead(PIN_AC_I2);
      uint16_t r_dv1= analogRead(PIN_DC_V1);
      uint16_t r_dv2= analogRead(PIN_DC_V2);
      uint16_t r_di = analogRead(PIN_DC_I);

      float v_v1 = countsToVolts(r_v1);
      float v_v2 = countsToVolts(r_v2);
      float v_i1 = countsToVolts(r_i1);
      float v_i2 = countsToVolts(r_i2);
      float v_dv1= countsToVolts(r_dv1);
      float v_dv2= countsToVolts(r_dv2);
      float v_di = countsToVolts(r_di);

      float mu_v1 = v1_mean.update(v_v1);
      float mu_v2 = v2_mean.update(v_v2);
      float mu_i1 = i1_mean.update(v_i1);
      float mu_i2 = i2_mean.update(v_i2);

      float ac_v1 = v_v1 - mu_v1;
      float ac_v2 = v_v2 - mu_v2;
      float ac_i1 = v_i1 - mu_i1;
      float ac_i2 = v_i2 - mu_i2;

      sumsq_v1 += (double)ac_v1 * (double)ac_v1;
      sumsq_v2 += (double)ac_v2 * (double)ac_v2;
      sumsq_i1 += (double)ac_i1 * (double)ac_i1;
      sumsq_i2 += (double)ac_i2 * (double)ac_i2;

      sum_dc_v1 += v_dv1;
      sum_dc_v2 += v_dv2;
      sum_dc_i  += v_di;

      n++; sinceYield++;
    }

    // publish when window is full
    if (n >= windowSamples) {
      float vrms1_pin = sqrtf((float)(sumsq_v1 / n));
      float vrms2_pin = sqrtf((float)(sumsq_v2 / n));
      float irms1_pin = sqrtf((float)(sumsq_i1 / n));
      float irms2_pin = sqrtf((float)(sumsq_i2 / n));

      float ac_v_rms_1 = vrms1_pin * ZMPT_V_PER_V1;
      float V1_adj = (V1_NOMINAL - ac_v_rms_1) * V1_adj_k;
      ac_v_rms_1 = vrms1_pin * (ZMPT_V_PER_V1 - V1_adj);

      float ac_v_rms_2 = vrms2_pin * ZMPT_V_PER_V2;
      float V2_adj = (V2_NOMINAL - ac_v_rms_2) * V2_adj_k;
      ac_v_rms_2 = vrms2_pin * (ZMPT_V_PER_V2 - V2_adj);

      float ac_i_rms_1 = irms1_pin / ACS712_SENS_V_PER_A;
      float ac_i_rms_2 = irms2_pin / ACS712_SENS_V_PER_A;

      float dc_v_1 = (float)(sum_dc_v1 / n) * DIVIDER_GAIN1;
      float dc_v_2 = (float)(sum_dc_v2 / n) * DIVIDER_GAIN2;

      float di_avg_pin = (float)(sum_dc_i / n);
      float dc_i = (di_avg_pin - ACS_MID_V_DC) / ACS712_SENS_V_PER_ADC;

      lockMeas();
      g_meas.ac_v_rms_1 = ac_v_rms_1;
      g_meas.ac_v_rms_2 = ac_v_rms_2;
      g_meas.ac_i_rms_1 = ac_i_rms_1;
      g_meas.ac_i_rms_2 = ac_i_rms_2;
      g_meas.dc_v_1     = dc_v_1;
      g_meas.dc_v_2     = dc_v_2;
      g_meas.dc_i       = dc_i;
      g_meas.last_update_ms = millis();
      unlockMeas();

      n = 0;
      sumsq_v1 = sumsq_v2 = sumsq_i1 = sumsq_i2 = 0.0;
      sum_dc_v1 = sum_dc_v2 = sum_dc_i = 0.0;
    }

    // --- guaranteed yield point every 1 ms ---
    vTaskDelayUntil(&nextWake, tick);

    // safety: if we ever loop extremely fast, still yield every ~250 samples
    if (sinceYield >= 250) { sinceYield = 0; vTaskDelay(1); }
  }
}


// Core 1: OLED + UART
void uiTask(void* pv) {
  uint32_t lastOLED = 0;
  uint32_t lastTxMs = 0;
  String cmd;

  for (;;) {
    // UART command (simple line reader)
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (c == '\n' || c == '\r') {
        // handle
        if (cmd.length() > 0) {
          if (cmd.equalsIgnoreCase("GET") || cmd.equalsIgnoreCase("R")) {
            uint32_t now = millis();
            if (now - lastTxMs >= JSON_RATE_LIMIT_MS) {
              Measurements snap;
              lockMeas(); memcpy((void*)&snap, (const void*)&g_meas, sizeof(snap)); unlockMeas();
              // JSON line (compact)
              Serial2.print("{\"ac_v1\":"); Serial2.print(snap.ac_v_rms_1, 3);
              Serial2.print(",\"ac_v2\":"); Serial2.print(snap.ac_v_rms_2, 3);
              Serial2.print(",\"ac_i1\":"); Serial2.print(snap.ac_i_rms_1, 3);
              Serial2.print(",\"ac_i2\":"); Serial2.print(snap.ac_i_rms_2, 3);
              Serial2.print(",\"dc_v1\":"); Serial2.print(snap.dc_v_1, 3);
              Serial2.print(",\"dc_v2\":"); Serial2.print(snap.dc_v_2, 3);
              Serial2.print(",\"dc_i\":");  Serial2.print(snap.dc_i, 3);
              Serial2.print("}\n");
              lastTxMs = now;
            }
          }
          cmd = "";
        }
      } else {
        if (cmd.length() < 64) cmd += c;
      }
    }

    // OLED update
    uint32_t now = millis();
    if (now - lastOLED >= OLED_UPDATE_MS) {
      Measurements snap;
      lockMeas(); memcpy((void*)&snap, (const void*)&g_meas, sizeof(snap)); unlockMeas();

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);

      // 4 lines max on 32px height; pack 2 items per line
      // L1: AC V
      display.setCursor(0, 0);
      display.print("Vac1:");
      display.print(snap.ac_v_rms_1, 0);
      display.print("  Vac2:");
      display.print(snap.ac_v_rms_2, 0);

      // L2: AC I
      display.setCursor(0, 8);
      display.print("Iac1:");
      display.print(snap.ac_i_rms_1, 2);
      display.print("  Iac2:");
      display.print(snap.ac_i_rms_2, 2);

      // L3: DC V
      display.setCursor(0, 16);
      display.print("Vdc1:");
      display.print(snap.dc_v_1, 2);
      display.print(" Vdc2:");
      display.print(snap.dc_v_2, 2);

      // L4: DC I
      display.setCursor(0, 24);
      display.print("Idc:");
      display.print(snap.dc_i, 2);
      display.print("A");

      display.display();
      lastOLED = now;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// =================== SETUP/LOOP ===================
void setup() {
  // No WiFi needed (keep ADC2 free for GPIO25)
  esp_bt_controller_disable();
  esp_wifi_stop();
  esp_wifi_deinit();

  // Serial2/UART2
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART2_RX, UART2_TX);
  delay(50);

  // OLED SPI pins
  SPI.begin(OLED_CLK, /*MISO*/ -1, OLED_MOSI, OLED_CS);
  if (!display.begin(0x3C)) { // param ignored for SPI, but begin() must be called
    // Some Adafruit_SSD1305 variants require begin() without address; keep as-is
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Power Monitor");
  display.setCursor(0, 10);
  display.println("Init...");
  display.display();

  delay(1000);
  
  // Prepare shared mutex
  g_measMutex = xSemaphoreCreateMutex();

  // Start tasks
  xTaskCreatePinnedToCore(samplerTask, "sampler", 8192, nullptr, 3, &samplerTaskHandle, 0); // Core 0
  xTaskCreatePinnedToCore(uiTask,       "ui_uart", 8192, nullptr, 2, &uiTaskHandle,       1); // Core 1
}

void loop() {
  // not used; everything is in FreeRTOS tasks
}
