# Power System Firmware - Technical Documentation

## Table of Contents
1. [System Overview](#system-overview)
2. [Architecture Deep Dive](#architecture-deep-dive)
3. [Master Controller Analysis](#master-controller-analysis)
4. [Sensor Module Analysis](#sensor-module-analysis)
5. [Communication Protocol](#communication-protocol)
6. [Battery Management Algorithm](#battery-management-algorithm)
7. [Real-Time Operating System](#real-time-operating-system)
8. [Hardware Interface Layer](#hardware-interface-layer)
9. [Error Handling and Recovery](#error-handling-and-recovery)
10. [Performance Analysis](#performance-analysis)
11. [Memory Management](#memory-management)
12. [Power Consumption](#power-consumption)

## System Overview

The Power System firmware implements a sophisticated dual-battery backup solution using two ESP32 WROOM-DA microcontrollers in a master-sensor configuration. The system provides uninterrupted power supply with intelligent battery management, real-time monitoring, and automatic failover capabilities.

### Key Features
- **Dual-Core Processing**: Utilizes both cores of ESP32 for optimal performance
- **Thread-Safe Operations**: Comprehensive mutex-based synchronization
- **Real-Time Monitoring**: 7-channel analog measurement system
- **Intelligent Battery Management**: Voltage-based SOC estimation and switching
- **Persistent State Management**: NVS-based configuration and counter storage
- **Emergency Response**: Automatic power outage detection and response
- **User Interface**: Serial CLI and OLED display

## Architecture Deep Dive

### System Topology

```
┌─────────────────────┐    UART2     ┌─────────────────────┐
│   Master Controller │◄────────────►│   Sensor Module     │
│   (PowerSystem_     │   115200     │   (PowerSystem_     │
│    master.ino)      │    baud      │    sensor.ino)      │
└─────────────────────┘              └─────────────────────┘
         │                                       │
         ▼                                       ▼
┌─────────────────────┐              ┌─────────────────────┐
│  Battery Management │              │  Measurement System │
│  • Relay Control    │              │  • 7 Analog Inputs  │
│  • SOC Monitoring   │              │  • RMS Calculation  │
│  • Switching Logic  │              │  • OLED Display     │
│  • Safety Features  │              │  • Data Logging     │
└─────────────────────┘              └─────────────────────┘
```

### Data Flow Architecture

```
Sensor Module Flow:
ADC Sampling → RMS/Average Calculation → JSON Formatting → UART Transmission
     ↓                    ↓                    ↓               ↓
  Core 0              Core 0              Core 1         Core 1
(High Priority)    (High Priority)    (Medium Priority) (Medium Priority)

Master Controller Flow:
UART Reception → JSON Parsing → Battery Logic → Relay Control → State Persistence
     ↓              ↓             ↓              ↓               ↓
  Core 0         Core 0        Core 1         Core 1          Core 1
(High Priority) (High Priority) (Medium Priority) (Medium Priority) (Low Priority)
```

## Master Controller Analysis

### Task Architecture

The master controller implements a sophisticated multi-task architecture:

#### Core 0 (Protocol Core)
- **TaskUART** (Priority 3): Handles communication with sensor module
  - Sends "GET\n" commands every 100ms
  - Parses incoming JSON responses
  - Updates global measurement structure
  - Thread-safe data sharing via mutex

#### Core 1 (Application Core)
- **TaskIO** (Priority 2): User interface and power monitoring
  - Button debouncing with 50ms stability requirement
  - Power outage detection on pin 19
  - Power relay control and "refresh" functionality
  - Serial CLI command processing
  - Emergency response handling

- **TaskArbiter** (Priority 2): Battery management decision engine
  - SOC calculation from voltage measurements
  - Battery switching logic implementation
  - Offline/damage detection and recovery
  - Power outage override handling
  - State persistence coordination

- **TaskBatt1/TaskBatt2** (Priority 2): Individual battery controllers
  - Relay state enforcement
  - Timing constraint validation (dwell/rest periods)
  - Battery full detection
  - Safety rule enforcement

### State Management

#### Global State Variables

```cpp
// Power Management
volatile bool g_powerSwitchState;    // User-controlled power switch
volatile bool g_powerOutage;         // Power grid status
volatile bool g_requestTurnOff;      // Emergency shutdown request

// Battery Management
BatteryState g_batt[2];              // Per-battery state tracking
int g_chargingBattery;               // Currently charging battery (-1 = none)
volatile bool g_globalBatteryFull;   // Battery full detection flag
uint32_t g_chargerAvailableAt;       // Charger rest timer
uint8_t g_chargeSwitchingSOC;        // SOC threshold for switching

// Measurements
Measurements g_meas;                 // Current sensor readings
SemaphoreHandle_t g_stateMutex;      // Thread synchronization
```

#### Battery State Structure

```cpp
struct BatteryState {
    Conn desired;           // Arbiter decision (TO_INVERTER/TO_CHARGER)
    Conn actual;            // Current hardware state
    uint32_t lastSwitchMs;  // Timing for dwell/rest enforcement
    uint32_t chargeStartMs; // Charge cycle start time
    uint32_t dcILowStartMs; // Full detection timing
    bool offline;           // Voltage < 10.0V detection
    uint32_t offlineDetectedMs; // Offline detection timestamp
    uint8_t soc;            // State of charge (0,25,50,75,100)
    bool fullFlag;          // Full charge detection flag
    uint32_t fullCount;     // Persistent full charge counter
};
```

### Battery Management Algorithm

#### Normal Operation Logic

1. **Initial Battery Selection**:
   ```cpp
   if (g_chargingBattery == -1) {
       int pick = (dv1 <= dv2) ? 0 : 1;  // Choose lower voltage
       g_chargingBattery = pick;
       g_batt[pick].desired = TO_CHARGER;
       g_batt[1 - pick].desired = TO_INVERTER;
   }
   ```

2. **Switching Decision**:
   ```cpp
   if (g_batt[other_battery].soc <= g_chargeSwitchingSOC) {
       // Switch charger to other battery
       g_batt[other_battery].desired = TO_CHARGER;
       g_batt[current_battery].desired = TO_INVERTER;
       g_chargingBattery = other_battery;
   }
   ```

3. **Safety Enforcement**:
   - Only one battery on CHARGER at any time
   - At least one battery on INVERTER always
   - 7-second rest between charger connections
   - 10-second minimum dwell on charger

#### Emergency Handling

1. **Power Outage Response**:
   ```cpp
   if (g_powerOutage) {
       g_batt[0].desired = TO_INVERTER;
       g_batt[1].desired = TO_INVERTER;
       g_chargingBattery = -1;
       // Disable button input
       pinMode(PIN_BTN, INPUT);
   }
   ```

2. **Battery Offline Recovery**:
   ```cpp
   if (battery_offline) {
       offline_battery.desired = TO_CHARGER;  // Prepare for charging
       online_battery.desired = TO_INVERTER;  // Maintain load support
   }
   ```

### Timing and Synchronization

#### Critical Timing Constants

```cpp
static const uint32_t BTN_DEBOUNCE_MS     = 50;    // Button stability
static const uint32_t UART_POLL_MS        = 100;   // Sensor polling
static const uint32_t CHARGER_REST_MS     = 7000;  // Charger rest period
static const uint32_t CHARGE_MIN_DWELL_MS = 10000; // Minimum charge time
static const uint32_t FULL_STABLE_MS      = 5000;  // Full detection time
```

#### Thread Synchronization

```cpp
#define LOCK()   xSemaphoreTake(g_stateMutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(g_stateMutex)

// Usage pattern:
LOCK();
// Critical section - modify shared state
UNLOCK();
```

## Sensor Module Analysis

### Dual-Core Sampling Architecture

#### Core 0: High-Rate Sampling Task

```cpp
void samplerTask(void* pv) {
    const uint32_t windowSamples = SAMPLE_RATE_HZ * RMS_WINDOW_SEC;
    const uint32_t SAMPLES_PER_TICK = SAMPLE_RATE_HZ / 1000;
    
    for (;;) {
        // Batch sampling (1ms intervals)
        for (uint32_t k = 0; k < SAMPLES_PER_TICK; ++k) {
            // Read all 7 channels
            uint16_t raw_values[7];
            // Convert to voltages
            // Apply offset correction
            // Accumulate for RMS/average
        }
        
        // Publish when window complete
        if (samples >= windowSamples) {
            // Calculate RMS for AC channels
            // Calculate averages for DC channels
            // Update global measurements (thread-safe)
        }
        
        vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(1));
    }
}
```

#### Core 1: Display and Communication Task

```cpp
void uiTask(void* pv) {
    for (;;) {
        // UART command processing
        if (command == "GET" || command == "R") {
            // Rate limiting check
            if (now - lastTx >= JSON_RATE_LIMIT_MS) {
                // Thread-safe measurement copy
                // JSON formatting and transmission
            }
        }
        
        // OLED display update
        if (now - lastOLED >= OLED_UPDATE_MS) {
            // Thread-safe measurement copy
            // Display formatting and update
        }
    }
}
```

### Measurement Processing

#### RMS Calculation for AC Signals

```cpp
// Offset tracking with running mean
RunningMean v1_mean(0.0005f);  // Slow adaptation
float ac_component = raw_voltage - v1_mean.update(raw_voltage);
sumsq_v1 += (double)ac_component * (double)ac_component;

// RMS calculation
float vrms = sqrtf((float)(sumsq_v1 / sample_count));
float ac_voltage = vrms * ZMPT_V_PER_V1;  // Apply scaling
```

#### DC Average Calculation

```cpp
// Simple averaging for DC signals
sum_dc_v1 += raw_dc_voltage;
float dc_voltage = (float)(sum_dc_v1 / sample_count) * DIVIDER_GAIN1;
```

#### Current Measurement Processing

```cpp
// AC current (RMS)
float ac_current = irms_pin / ACS712_SENS_V_PER_A;

// DC current (average with offset correction)
float dc_current = (avg_pin_voltage - ACS_MID_V_DC) / ACS712_SENS_V_PER_ADC;
```

### Display System

#### OLED Layout (128x32 pixels)

```
Line 1 (Y=0):  "Vac1:120  Vac2:119"
Line 2 (Y=8):  "Iac1:2.34  Iac2:1.87"
Line 3 (Y=16): "Vdc1:13.2 Vdc2:12.8"
Line 4 (Y=24): "Idc:5.6A"
```

## Communication Protocol

### UART Configuration
- **Baud Rate**: 115200
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Flow Control**: None
- **Pins**: RX=21, TX=22

### Message Format

#### Request (Master → Sensor)
```
GET\n
```

#### Response (Sensor → Master)
```json
{"ac_v1":120.5,"ac_v2":119.8,"ac_i1":2.34,"ac_i2":1.87,"dc_v1":13.2,"dc_v2":12.8,"dc_i":5.6}
```

### JSON Parsing Implementation

```cpp
bool extractFloat(const String& json, const char* key, float& out) {
    String pattern = String("\"") + key + String("\":");
    int start = json.indexOf(pattern);
    if (start < 0) return false;
    
    start += pattern.length();
    while (start < json.length() && json[start] == ' ') start++;
    
    int end = start;
    while (end < json.length() && json[end] != ',' && json[end] != '}') end++;
    
    if (end <= start) return false;
    out = json.substring(start, end).toFloat();
    return true;
}
```

### Error Handling

- **Parse Failures**: Logged with error message, previous values retained
- **Communication Timeouts**: Automatic retry with exponential backoff
- **Rate Limiting**: Prevents sensor overload (50ms minimum between responses)

## Real-Time Operating System

### FreeRTOS Configuration

#### Task Priorities
```cpp
// Higher numbers = higher priority
TaskUART:     Priority 3 (Highest - time-critical communication)
TaskIO:       Priority 2 (High - user interface and safety)
TaskArbiter:  Priority 2 (High - battery management)
TaskBatt1/2:  Priority 2 (High - hardware control)
samplerTask:  Priority 3 (Highest - real-time sampling)
uiTask:       Priority 2 (High - display and communication)
```

#### Core Assignment Strategy
```cpp
// Core 0 (Protocol Core) - Time-critical, deterministic tasks
TaskUART:     Core 0  // Communication processing
samplerTask:  Core 0  // High-rate ADC sampling

// Core 1 (Application Core) - Application logic, user interface
TaskIO:       Core 1  // Button, CLI, power monitoring
TaskArbiter:  Core 1  // Battery management logic
TaskBatt1/2:  Core 1  // Relay control
uiTask:       Core 1  // OLED display, UART commands
```

#### Memory Allocation
```cpp
// Stack sizes (bytes)
TaskIO:       4096  // CLI buffer, string processing
TaskUART:     4096  // JSON parsing, communication buffers
TaskArbiter:  4096  // State management, calculations
TaskBatt1/2:  4096  // Relay control, timing
samplerTask:  8192  // Large sample buffers, calculations
uiTask:       8192  // Display buffers, UART processing
```

### Synchronization Mechanisms

#### Mutex Usage
```cpp
// Global state protection
SemaphoreHandle_t g_stateMutex;
SemaphoreHandle_t g_measMutex;

// Critical section pattern
LOCK();
// Atomic operations on shared data
UNLOCK();
```

#### Task Delays and Timing
```cpp
// Precise timing with vTaskDelayUntil
TickType_t nextWake = xTaskGetTickCount();
vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(period_ms));

// Simple delays with vTaskDelay
vTaskDelay(pdMS_TO_TICKS(delay_ms));
```

## Hardware Interface Layer

### GPIO Configuration

#### Input Pins
```cpp
// Button with internal pull-up
pinMode(PIN_BTN, INPUT_PULLUP);

// Power good sense
pinMode(PIN_PWR_GOOD, INPUT);

// Reserved CAN pins
pinMode(PIN_CAN_RX, INPUT);
pinMode(PIN_CAN_TX, INPUT);
```

#### Output Pins
```cpp
// Power relay control
pinMode(PIN_PWR_RELAY, OUTPUT);
digitalWrite(PIN_PWR_RELAY, g_powerSwitchState ? HIGH : LOW);

// Battery relay control
pinMode(PIN_BAT1_RELAY, OUTPUT);
pinMode(PIN_BAT2_RELAY, OUTPUT);
```

### ADC Configuration

```cpp
// 12-bit resolution, 11dB attenuation
analogReadResolution(12);
analogSetAttenuation(ADC_11db);
analogSetPinAttenuation(PIN_DC_I, ADC_11db);

// Warm-up period
for (int i = 0; i < 100; ++i) {
    analogRead(PIN_AC_I1);
    analogRead(PIN_AC_I2);
    vTaskDelay(pdMS_TO_TICKS(2));
}
```

### UART Configuration

```cpp
// UART2 initialization
Serial2.begin(115200, SERIAL_8N1, PIN_UART2_RX, PIN_UART2_TX);

// Communication pattern
Serial2.print("GET\n");  // Request
while (Serial2.available()) {
    char c = Serial2.read();
    // Process response
}
```

### SPI Configuration (Sensor Module)

```cpp
// OLED display SPI
SPI.begin(OLED_CLK, -1, OLED_MOSI, OLED_CS);
display.begin(0x3C);
```

## Error Handling and Recovery

### Error Classification

#### Critical Errors (Immediate Response Required)
1. **Power Outage**: Switch both batteries to inverter
2. **Both Batteries Offline**: Emergency shutdown sequence
3. **Communication Failure**: Fallback to local control

#### Warning Conditions (Logged, Continued Operation)
1. **Single Battery Offline**: Switch to charger for recovery
2. **JSON Parse Failure**: Retain previous measurements
3. **Sensor Out of Range**: Use default values

### Recovery Mechanisms

#### Automatic Recovery
```cpp
// Power outage recovery
if (!g_powerOutage && was_outage) {
    g_requestTurnOff = false;
    pinMode(PIN_BTN, INPUT_PULLUP);  // Re-enable button
    // Re-sync debounce state
    lastStable = digitalRead(PIN_BTN);
}

// Battery offline recovery
if (!battery.offline && was_offline) {
    battery.offlineDetectedMs = 0;  // Clear offline timestamp
    // Resume normal switching logic
}
```

#### Watchdog and Monitoring
```cpp
// Task health monitoring
if ((millis() - g_meas.lastUpdateMs) > COMM_TIMEOUT_MS) {
    // Communication failure detected
    // Switch to local control mode
}

// Relay state verification
int expected = g_powerSwitchState ? HIGH : LOW;
if (digitalRead(PIN_PWR_RELAY) != expected) {
    digitalWrite(PIN_PWR_RELAY, expected);  // Correct mismatch
    Serial.println("[PWR] Relay refreshed");
}
```

## Performance Analysis

### CPU Utilization

#### Master Controller
- **Core 0**: ~15% (UART communication, JSON parsing)
- **Core 1**: ~25% (Battery management, user interface)
- **Total**: ~20% average system load

#### Sensor Module
- **Core 0**: ~60% (High-rate ADC sampling, RMS calculations)
- **Core 1**: ~10% (Display updates, UART responses)
- **Total**: ~35% average system load

### Memory Usage

#### RAM Allocation
```cpp
// Static allocations
Measurements g_meas;           // 32 bytes
BatteryState g_batt[2];        // 64 bytes
String buffers;                // ~512 bytes
Task stacks;                   // ~32KB total
FreeRTOS overhead;             // ~8KB

// Total RAM usage: ~40KB of 520KB available
```

#### Flash Usage
```cpp
// Code size
Master controller: ~180KB
Sensor module:     ~120KB
Libraries:         ~200KB

// Total Flash usage: ~500KB of 4MB available
```

### Timing Analysis

#### Critical Path Timing
```
ADC Sample:           ~10μs per channel
RMS Calculation:      ~100μs per window
JSON Generation:      ~500μs
UART Transmission:    ~1ms
Relay Switching:      ~1ms
Button Debounce:      50ms
Charger Rest Period:  7000ms
```

#### Response Times
```
Power Outage Response:    <10ms
Battery Switch Decision:  <100ms
Relay State Change:       <1s (with timing constraints)
User Button Response:     <100ms (after debounce)
Measurement Update:       100ms (UART poll rate)
```

## Memory Management

### Stack Management

```cpp
// Task stack monitoring
void checkStackUsage() {
    UBaseType_t highWater = uxTaskGetStackHighWaterMark(NULL);
    if (highWater < 512) {  // Less than 512 bytes free
        Serial.printf("[WARN] Low stack: %u bytes\n", highWater);
    }
}
```

### Heap Management

```cpp
// Dynamic allocation monitoring
void checkHeapUsage() {
    size_t freeHeap = esp_get_free_heap_size();
    size_t minFreeHeap = esp_get_minimum_free_heap_size();
    
    Serial.printf("Heap: %u free, %u minimum\n", freeHeap, minFreeHeap);
}
```

### String Handling

```cpp
// Efficient string operations
String cliBuf;  // Pre-allocated CLI buffer
cliBuf.reserve(64);  // Prevent frequent reallocations

// Avoid String concatenation in loops
// Use sprintf for formatted output
char buffer[128];
snprintf(buffer, sizeof(buffer), "Value: %.2f", measurement);
```

## Power Consumption

### Power Modes

#### Active Mode
- **Master Controller**: ~150mA @ 3.3V
- **Sensor Module**: ~200mA @ 3.3V (OLED active)
- **Total System**: ~350mA @ 3.3V

#### Sleep Mode Potential
```cpp
// Light sleep configuration (future enhancement)
esp_sleep_enable_timer_wakeup(100000);  // 100ms
esp_light_sleep_start();
```

### Optimization Strategies

1. **OLED Power Management**: Turn off display during inactive periods
2. **CPU Frequency Scaling**: Reduce clock speed during low activity
3. **Peripheral Power Down**: Disable unused peripherals
4. **Task Optimization**: Increase task delays where possible

## Conclusion

This technical documentation provides a comprehensive analysis of the Power System firmware architecture. The system demonstrates sophisticated real-time embedded programming techniques including:

- **Multi-core task distribution** for optimal performance
- **Thread-safe programming** with comprehensive synchronization
- **Real-time measurement processing** with high accuracy
- **Intelligent battery management** with safety-first design
- **Robust error handling** and automatic recovery
- **Efficient resource utilization** with room for expansion

The modular design and comprehensive documentation facilitate maintenance, debugging, and future enhancements while ensuring reliable operation in critical power backup applications.