#ifndef POWER_SYSTEM_H
#define POWER_SYSTEM_H

#include <Arduino.h>

// =================== SENSOR MODULE CONFIGURATION ===================

// ===== Pin Definitions =====
// AC Voltage Sensors (ZMPT101B)
#define PIN_AC_V1           32
#define PIN_AC_V2           33

// AC Current Sensors (ACS712-20A)
#define PIN_AC_I1           34
#define PIN_AC_I2           35

// DC Voltage Sensors (5:1 Divider)
#define PIN_DC_V1           36  // VP
#define PIN_DC_V2           39  // VN

// DC Current Sensor (ACS712-20A)
#define PIN_DC_I            25

// OLED Display (SSD1305, 128x32, SPI)
#define OLED_MOSI           23
#define OLED_CLK            18
#define OLED_DC             16
#define OLED_CS             5
#define OLED_RST            17
#define OLED_W              128
#define OLED_H              32

// UART2 Communication
#define UART2_RX            21
#define UART2_TX            22
#define UART_BAUD           115200

// ===== Sensor Calibration Constants =====

// ADC Configuration
#define ADC_RESOLUTION      12      // 12-bit ADC (0-4095)
#define ADC_VREF            3.3f    // Reference voltage
#define ADC_MAX_COUNT       4095.0f // Maximum ADC count

// ADC to Voltage Conversion
#define countsToVolts(counts) ((float)(counts) * ADC_VREF / ADC_MAX_COUNT)

// ZMPT101B AC Voltage Sensors
#define ZMPT_V_PER_V1       100.0f  // Voltage scaling factor for channel 1
#define ZMPT_V_PER_V2       100.0f  // Voltage scaling factor for channel 2
#define V1_NOMINAL          120.0f  // Nominal voltage for calibration
#define V2_NOMINAL          120.0f  // Nominal voltage for calibration
#define V1_adj_k            0.01f   // Adjustment factor for V1
#define V2_adj_k            0.01f   // Adjustment factor for V2

// ACS712-20A Current Sensors
#define ACS712_SENS_V_PER_A     0.1f    // Sensitivity: 100mV/A for 20A version
#define ACS712_SENS_V_PER_ADC   0.1f    // Sensitivity for ADC version
#define ACS_MID_V_DC            1.65f   // Midpoint voltage for DC current sensor

// DC Voltage Dividers (5:1 ratio)
#define DIVIDER_GAIN1       5.0f    // Voltage divider gain for battery 1
#define DIVIDER_GAIN2       5.0f    // Voltage divider gain for battery 2

// ===== Sampling Configuration =====
#define SAMPLE_RATE_HZ      1000    // ADC sampling rate (Hz)
#define RMS_WINDOW_SEC      0.1f    // RMS calculation window (seconds)

// ===== Timing Configuration =====
#define OLED_UPDATE_MS      200     // OLED display update interval
#define JSON_RATE_LIMIT_MS  50      // Minimum time between JSON responses

// ===== Battery Management Constants =====

// SOC Voltage Thresholds (from master controller)
#define V_OFFLINE           10.0f   // Below this = offline/damaged
#define V_SOC0_MIN          10.0f   // 0% SOC minimum voltage
#define V_SOC25_MIN         12.0f   // 25% SOC minimum voltage
#define V_SOC50_MIN         13.15f  // 50% SOC minimum voltage
#define V_SOC75_MIN         13.20f  // 75% SOC minimum voltage
#define V_SOC100_MIN        13.33f  // 100% SOC minimum voltage

// Battery Full Detection
#define FULL_CURRENT_THRESH 0.5f    // Current threshold for "full" detection (A)
#define FULL_STABLE_MS      5000    // Time current must be below threshold (ms)

// Timing Constants
#define CHARGER_REST_MS     7000    // Rest time between charger connections (ms)
#define CHARGE_MIN_DWELL_MS 10000   // Minimum time on charger (ms)

// ===== Communication Protocol =====

// JSON Field Names
#define JSON_AC_V1          "ac_v1"
#define JSON_AC_V2          "ac_v2"
#define JSON_AC_I1          "ac_i1"
#define JSON_AC_I2          "ac_i2"
#define JSON_DC_V1          "dc_v1"
#define JSON_DC_V2          "dc_v2"
#define JSON_DC_I           "dc_i"

// Command Strings
#define CMD_GET             "GET"
#define CMD_READ            "R"

// ===== Utility Macros =====

// Math helpers
#define ABS(x)              ((x) < 0 ? -(x) : (x))
#define MIN(a, b)           ((a) < (b) ? (a) : (b))
#define MAX(a, b)           ((a) > (b) ? (a) : (b))
#define CONSTRAIN(x, a, b)  ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

// Voltage to SOC conversion
inline uint8_t voltageToSOC(float voltage) {
    if (voltage < V_OFFLINE) return 255;        // Offline marker
    if (voltage >= V_SOC100_MIN) return 100;
    if (voltage >= V_SOC75_MIN) return 75;
    if (voltage >= V_SOC50_MIN) return 50;
    if (voltage >= V_SOC25_MIN) return 25;
    if (voltage >= V_SOC0_MIN) return 0;
    return 0;
}

// SOC to string conversion
inline const char* socToString(uint8_t soc) {
    switch (soc) {
        case 255: return "OFFLINE";
        case 100: return "100%";
        case 75:  return "75%";
        case 50:  return "50%";
        case 25:  return "25%";
        case 0:   return "0%";
        default:  return "UNKNOWN";
    }
}

// ===== Debug Configuration =====
#ifdef DEBUG_POWER_SYSTEM
    #define DEBUG_PRINT(x)      Serial.print(x)
    #define DEBUG_PRINTLN(x)    Serial.println(x)
    #define DEBUG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif

// ===== Error Codes =====
enum PowerSystemError {
    PS_OK = 0,
    PS_ERROR_SENSOR_INIT,
    PS_ERROR_OLED_INIT,
    PS_ERROR_UART_INIT,
    PS_ERROR_JSON_PARSE,
    PS_ERROR_ADC_READ,
    PS_ERROR_COMMUNICATION,
    PS_ERROR_BATTERY_OFFLINE,
    PS_ERROR_RELAY_FAULT,
    PS_ERROR_POWER_OUTAGE
};

// ===== System States =====
enum SystemState {
    STATE_INIT = 0,
    STATE_NORMAL_OPERATION,
    STATE_POWER_OUTAGE,
    STATE_BATTERY_FAULT,
    STATE_COMMUNICATION_FAULT,
    STATE_EMERGENCY_SHUTDOWN
};

// ===== Battery Connection States =====
enum BatteryConnection {
    CONN_INVERTER = 0,
    CONN_CHARGER = 1
};

// ===== Function Prototypes =====

// Sensor functions
float readACVoltage(uint8_t channel);
float readACCurrent(uint8_t channel);
float readDCVoltage(uint8_t channel);
float readDCCurrent(void);

// Calibration functions
void calibrateSensors(void);
float applyCalibration(float rawValue, uint8_t sensorType, uint8_t channel);

// Communication functions
bool sendMeasurements(void);
bool parseMeasurementRequest(const String& request);

// Display functions
void updateOLEDDisplay(void);
void displaySystemStatus(void);
void displayError(PowerSystemError error);

// Utility functions
float calculateRMS(float* samples, uint16_t count);
float calculateAverage(float* samples, uint16_t count);
bool isValidMeasurement(float value, float minVal, float maxVal);

#endif // POWER_SYSTEM_H