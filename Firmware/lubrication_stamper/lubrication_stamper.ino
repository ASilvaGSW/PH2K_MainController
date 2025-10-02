/*
 * lubrication_stamper.ino
 * ----------------------
 * Firmware for lubrication stamper system with dual-CAN (Controller Area Network) based communication using ESP32.
 * This system includes 3-level liquid monitoring using float switches, 3 liquid flow meters, and 3 solenoid valves
 * for comprehensive fluid management, monitoring, and control.
 *
 * FEATURES:
 *   - Single CAN bus support: ESP32 TWAI (integrated) for communication with the network.
 *   - Implements FreeRTOS for multitasking, using queues for instruction dispatch.
 *   - Robust command processing with status reporting and error handling via CAN.
 *   - 3-level liquid monitoring system using Elecall ES10010 float switches (Low, Medium, High levels).
 *   - 3 liquid flow meters with pulse counting and real-time flow rate calculation (L/min).
 *   - 3 solenoid valves for precise liquid flow control with state tracking and emergency shutdown.
 *   - Interrupt-driven flow measurement for accurate pulse counting.
 *   - Linked process architecture: Each process has 1 level sensor + 1 flow meter + 1 solenoid valve.
 *   - Modular design ready for adding custom components.
 *
 * ARCHITECTURE OVERVIEW:
 *   - setup(): Initializes peripherals, CAN buses, level sensors, flow meters, solenoid valves, and FreeRTOS tasks.
 *   - loop(): Main event loop, processes queued CAN instructions.
 *   - twai_listener_task(): Listens for incoming CAN instructions and queues them for processing.
 *   - process_instruction(): Decodes and executes commands from CAN, including level sensor, flow meter, and solenoid dispatch.
 *   - Helper functions for CAN reply waiting, CAN response sending, level sensor reading, flow rate calculation, and solenoid control.
 *
 * COMMANDS (CAN):
 *   - 0x01: Reset microcontroller
 *   - 0x02: Heartbeat
 *   - 0x30: Read individual level sensor (data[1] = sensor number 1-3)
 *   - 0x31: Read all level sensors (returns status of all 3 sensors)
 *   - 0x32: Read flow meter rate (data[1] = meter number 1-3, returns float L/min)
 *   - 0x33: Read flow meter total pulses (data[1] = meter number 1-3, returns unsigned long)
 *   - 0x34: Reset flow meter (data[1] = meter number 1-3)
 *   - 0x35: Read all flow meter rates (returns all 3 rates as 16-bit integers * 100)
 *   - 0x36: Set solenoid valve state (data[1] = solenoid number 1-3, data[2] = state 0/1)
 *   - 0x37: Get solenoid valve state (data[1] = solenoid number 1-3)
 *   - 0x38: Toggle solenoid valve (data[1] = solenoid number 1-3)
 *   - 0x39: Get all solenoid states (returns status of all 3 solenoids)
 *   - 0x3A: Emergency close all solenoids (safety command)
 *   - 0xFF: Power off command
 *
 * LEVEL SENSORS:
 *   - Sensor 1 (GPIO13): Tank 1 liquid detection (individual tank)
 *   - Sensor 2 (GPIO14): Tank 2 liquid detection (individual tank)
 *   - Sensor 3 (GPIO16): Tank 3 liquid detection (individual tank)
 *   - Float switches: Elecall ES10010 (110V, 4" wire length, stainless steel)
 *   - Each sensor monitors a separate tank for liquid presence (true = liquid present, false = empty)
 *
 * FLOW METERS:
 *   - Flow Meter 1 (GPIO21): Liquid flow measurement with pulse output
 *   - Flow Meter 2 (GPIO22): Liquid flow measurement with pulse output
 *   - Flow Meter 3 (GPIO23): Liquid flow measurement with pulse output
 *   - Type: Plastic 1/2" NPS Threaded flow meters
 *   - Calibration: 450 pulses per liter (adjustable via FLOW_CALIBRATION_FACTOR)
 *   - Real-time flow rate calculation in L/min with interrupt-driven pulse counting
 *
 * SOLENOID VALVES:
 *   - Solenoid 1 (GPIO17): Process 1 liquid flow control (linked to Level sensor 1 + Flow meter 1)
 *   - Solenoid 2 (GPIO18): Process 2 liquid flow control (linked to Level sensor 2 + Flow meter 2)
 *   - Solenoid 3 (GPIO19): Process 3 liquid flow control (linked to Level sensor 3 + Flow meter 3)
 *   - Control: HIGH = Open, LOW = Closed (configurable based on valve type)
 *   - State tracking with last toggle time for timing control
 *   - Emergency close all function for safety shutdown
 *
 * DEPENDENCIES:
 *   - ESP32-TWAI-CAN
 *   - SPI
 *   - FreeRTOS (ESP32)
 *
 * AUTHOR: Alan Silva
 * DATE: 2025-01-27
 * CONTACT: asilva@gswiring.com
 * DETAILED DOCUMENTATION:
 *   - Each function and major logic block is documented below for clarity and maintainability.
 *   - For protocol details and troubleshooting, see the project README and hardware wiring diagrams.
 */

#include <ESP32-TWAI-CAN.hpp>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Global variables
byte replyData[8];  // Buffer for CAN replies

// Flow meter variables
volatile unsigned long flowMeter1PulseCount = 0;
volatile unsigned long flowMeter2PulseCount = 0;
volatile unsigned long flowMeter3PulseCount = 0;
volatile unsigned long lastFlowMeter1Time = 0;
volatile unsigned long lastFlowMeter2Time = 0;
volatile unsigned long lastFlowMeter3Time = 0;
float flowMeter1Rate = 0.0;  // Flow rate in L/min
float flowMeter2Rate = 0.0;  // Flow rate in L/min
float flowMeter3Rate = 0.0;  // Flow rate in L/min
unsigned long lastFlowCalculationTime = 0;

// Flow meter calibration factor (pulses per liter) - adjust based on your specific flow meter
#define FLOW_CALIBRATION_FACTOR 450.0  // Typical value for 1/2" flow meters

// Solenoid valve state variables
bool solenoid1State = false;  // false = closed, true = open
bool solenoid2State = false;  // false = closed, true = open
bool solenoid3State = false;  // false = closed, true = open
unsigned long solenoid1LastToggleTime = 0;  // For timing control if needed
unsigned long solenoid2LastToggleTime = 0;  // For timing control if needed
unsigned long solenoid3LastToggleTime = 0;  // For timing control if needed

// CAN pins for ESP32 TWAI
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Level sensor pins (Float switches - Elecall ES10010)
#define LEVEL_SENSOR_1_PIN 13  // GPIO13 - Level sensor 1 (Tank 1)
#define LEVEL_SENSOR_2_PIN 14  // GPIO14 - Level sensor 2 (Tank 2)
#define LEVEL_SENSOR_3_PIN 16  // GPIO16 - Level sensor 3 (Tank 3)

// Flow meter pins (Plastic 1/2" NPS Threaded Flow Meters)
#define FLOW_METER_1_PIN 21  // GPIO21 - Flow meter 1 pulse input
#define FLOW_METER_2_PIN 22  // GPIO22 - Flow meter 2 pulse input
#define FLOW_METER_3_PIN 23  // GPIO23 - Flow meter 3 pulse input

// Solenoid valve pins (Linked to corresponding level sensors and flow meters)
#define SOLENOID_1_PIN 17  // GPIO17 - Solenoid valve 1 (Process 1: Level sensor 1 + Flow meter 1)
#define SOLENOID_2_PIN 18  // GPIO18 - Solenoid valve 2 (Process 2: Level sensor 2 + Flow meter 2)
#define SOLENOID_3_PIN 19  // GPIO19 - Solenoid valve 3 (Process 3: Level sensor 3 + Flow meter 3)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x192

// CAN instances
CanFrame rxFrame, txFrame;

// FreeRTOS queue for CAN instructions
QueueHandle_t instructionQueue;

// Structure for CAN instructions
struct CANInstruction {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
};

// Function prototypes
void twai_listener_task(void *parameter);
void process_instruction(CANInstruction instruction);
void send_twai_response(byte response[8]);
bool wait_for_twai_reply(unsigned long timeout_ms = 5000);
bool read_level_sensor(int sensor_number);
void get_all_level_sensors(bool levels[3]);
void IRAM_ATTR flowMeter1ISR();
void IRAM_ATTR flowMeter2ISR();
void IRAM_ATTR flowMeter3ISR();
void calculate_flow_rates();
float get_flow_rate(int meter_number);
unsigned long get_total_pulses(int meter_number);
void reset_flow_meter(int meter_number);
void set_solenoid_state(int solenoid_number, bool state);
bool get_solenoid_state(int solenoid_number);
void toggle_solenoid(int solenoid_number);
void close_all_solenoids();

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Clean Template System...");

  // Initialize CAN0 (ESP32 TWAI)
  ESP32Can.setPins(CAN0_TX, CAN0_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
  if (ESP32Can.begin()) {
    Serial.println("CAN0 (TWAI) initialized successfully");
  } else {
    Serial.println("CAN0 (TWAI) initialization failed");
  }

  // Create instruction queue
  instructionQueue = xQueueCreate(10, sizeof(CANInstruction));
  if (instructionQueue == NULL) {
    Serial.println("Failed to create instruction queue");
    return;
  }

  // Create TWAI listener task
  xTaskCreate(
    twai_listener_task,
    "TWAI Listener",
    4096,
    NULL,
    1,
    NULL
  );

  // Initialize level sensors (Float switches)
  pinMode(LEVEL_SENSOR_1_PIN, INPUT_PULLUP);
  pinMode(LEVEL_SENSOR_2_PIN, INPUT_PULLUP);
  pinMode(LEVEL_SENSOR_3_PIN, INPUT_PULLUP);
  Serial.println("Level sensors initialized");

  // Initialize flow meters
  pinMode(FLOW_METER_1_PIN, INPUT);
  pinMode(FLOW_METER_2_PIN, INPUT);
  pinMode(FLOW_METER_3_PIN, INPUT);
  
  // Attach interrupts for flow meters (rising edge detection)
  attachInterrupt(digitalPinToInterrupt(FLOW_METER_1_PIN), flowMeter1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_METER_2_PIN), flowMeter2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_METER_3_PIN), flowMeter3ISR, RISING);
  
  // Initialize flow calculation timer
  lastFlowCalculationTime = millis();
  Serial.println("Flow meters initialized with interrupts");

  // Initialize solenoid valves (start with all closed)
  pinMode(SOLENOID_1_PIN, OUTPUT);
  pinMode(SOLENOID_2_PIN, OUTPUT);
  pinMode(SOLENOID_3_PIN, OUTPUT);
  
  // Set all solenoids to closed state (LOW = closed for most solenoid valves)
  digitalWrite(SOLENOID_1_PIN, LOW);
  digitalWrite(SOLENOID_2_PIN, LOW);
  digitalWrite(SOLENOID_3_PIN, LOW);
  
  // Initialize state variables
  solenoid1State = false;
  solenoid2State = false;
  solenoid3State = false;
  Serial.println("Solenoid valves initialized (all closed)");

  Serial.println("System initialization complete");
}

void loop() {
  CANInstruction instruction;
  
  // Process queued instructions
  if (xQueueReceive(instructionQueue, &instruction, pdMS_TO_TICKS(10)) == pdTRUE) {
    process_instruction(instruction);
  }
  
  // Add your main loop logic here
  delay(1);
}

void twai_listener_task(void *parameter) {
  CANInstruction instruction;
  
  while (true) {
    // Check for TWAI messages
    if (ESP32Can.readFrame(rxFrame, 0)) {
      if (rxFrame.identifier == DEVICE_CAN_ID) {
        instruction.id = rxFrame.identifier;
        instruction.len = rxFrame.data_length_code;
        memcpy(instruction.data, rxFrame.data, rxFrame.data_length_code);
        
        if (xQueueSend(instructionQueue, &instruction, pdMS_TO_TICKS(100)) != pdTRUE) {
          Serial.println("Failed to queue TWAI instruction");
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void process_instruction(CANInstruction instruction)
{
  byte command = instruction.data[0];
  
  Serial.print("Processing command: 0x");
  Serial.println(command, HEX);
  
  // Clear reply buffer
  memset(replyData, 0, sizeof(replyData));
  replyData[0] = command; // Echo command in response
  
  switch (command) {
    case 0x01: // Reset microcontroller
      Serial.println("Reset command received");
      replyData[1] = 0x01; // OK
      send_twai_response(replyData);
      delay(100);
      ESP.restart();
      break;
      
    case 0x02: // Heartbeat
      Serial.println("Heartbeat received");
      replyData[1] = 0x01; // OK - System alive
      send_twai_response(replyData);
      break;
      
    case 0xFF: // Power off
      Serial.println("Power off command received");
      replyData[1] = 0x01; // OK
      send_twai_response(replyData);
      // Add your power off logic here
      break;
      
    // Level sensor commands
    case 0x03: // Read individual level sensor
      {
        byte sensor_number = instruction.data[1];
        Serial.print("Reading level sensor ");
        Serial.println(sensor_number);
        
        if (sensor_number >= 1 && sensor_number <= 3) {
          bool level_detected = read_level_sensor(sensor_number);
          replyData[1] = 0x01; // OK
          replyData[2] = sensor_number;
          replyData[3] = level_detected ? 0x01 : 0x00; // 1 = liquid detected, 0 = no liquid
          Serial.print("Sensor ");
          Serial.print(sensor_number);
          Serial.print(" level: ");
          Serial.println(level_detected ? "DETECTED" : "NOT DETECTED");
        } else {
          replyData[1] = 0x02; // FAIL - Invalid sensor number
          Serial.println("Invalid sensor number");
        }
        send_twai_response(replyData);
      }
      break;
      
    case 0x04: // Read all level sensors
      {
        Serial.println("Reading all level sensors");
        bool levels[3];
        get_all_level_sensors(levels);
        
        replyData[1] = 0x01; // OK
        replyData[2] = levels[0] ? 0x01 : 0x00; // Tank 1 sensor
        replyData[3] = levels[1] ? 0x01 : 0x00; // Tank 2 sensor
        replyData[4] = levels[2] ? 0x01 : 0x00; // Tank 3 sensor
        
        Serial.print("Level sensors - Tank 1: ");
        Serial.print(levels[0] ? "DETECTED" : "NOT DETECTED");
        Serial.print(", Tank 2: ");
        Serial.print(levels[1] ? "DETECTED" : "NOT DETECTED");
        Serial.print(", Tank 3: ");
        Serial.println(levels[2] ? "DETECTED" : "NOT DETECTED");
        
        send_twai_response(replyData);
      }
      break;
      
    case 0x05: // Read flow meter rate (L/min)
      {
        byte meter_number = instruction.data[1];
        Serial.print("Reading flow meter ");
        Serial.print(meter_number);
        Serial.println(" rate");
        
        if (meter_number >= 1 && meter_number <= 3) {
          float flow_rate = get_flow_rate(meter_number);
          
          replyData[1] = 0x01; // OK
          replyData[2] = meter_number;
          
          // Convert float to bytes (IEEE 754 format)
          union {
            float f;
            byte b[4];
          } floatBytes;
          floatBytes.f = flow_rate;
          
          replyData[3] = floatBytes.b[0];
          replyData[4] = floatBytes.b[1];
          replyData[5] = floatBytes.b[2];
          replyData[6] = floatBytes.b[3];
          
          Serial.print("Flow rate: ");
          Serial.print(flow_rate);
          Serial.println(" L/min");
        } else {
          replyData[1] = 0x02; // FAIL - Invalid meter number
          Serial.println("Invalid flow meter number");
        }
        send_twai_response(replyData);
      }
      break;
      
    case 0x06: // Read flow meter total pulses
      {
        byte meter_number = instruction.data[1];
        Serial.print("Reading flow meter ");
        Serial.print(meter_number);
        Serial.println(" total pulses");
        
        if (meter_number >= 1 && meter_number <= 3) {
          unsigned long total_pulses = get_total_pulses(meter_number);
          
          replyData[1] = 0x01; // OK
          replyData[2] = meter_number;
          
          // Convert unsigned long to bytes
          replyData[3] = (total_pulses >> 24) & 0xFF;
          replyData[4] = (total_pulses >> 16) & 0xFF;
          replyData[5] = (total_pulses >> 8) & 0xFF;
          replyData[6] = total_pulses & 0xFF;
          
          Serial.print("Total pulses: ");
          Serial.println(total_pulses);
        } else {
          replyData[1] = 0x02; // FAIL - Invalid meter number
          Serial.println("Invalid flow meter number");
        }
        send_twai_response(replyData);
      }
      break;
      
    case 0x07: // Reset flow meter
      {
        byte meter_number = instruction.data[1];
        Serial.print("Resetting flow meter ");
        Serial.println(meter_number);
        
        if (meter_number >= 1 && meter_number <= 3) {
          reset_flow_meter(meter_number);
          
          replyData[1] = 0x01; // OK
          replyData[2] = meter_number;
          
          Serial.print("Flow meter ");
          Serial.print(meter_number);
          Serial.println(" reset successfully");
        } else {
          replyData[1] = 0x02; // FAIL - Invalid meter number
          Serial.println("Invalid flow meter number");
        }
        send_twai_response(replyData);
      }
      break;
      
    case 0x08: // Read all flow meter rates
      {
        Serial.println("Reading all flow meter rates");
        
        float rate1 = get_flow_rate(1);
        float rate2 = get_flow_rate(2);
        float rate3 = get_flow_rate(3);
        
        replyData[1] = 0x01; // OK
        
        // Pack flow rates as 16-bit integers (rate * 100 for 2 decimal precision)
        uint16_t rate1_int = (uint16_t)(rate1 * 100);
        uint16_t rate2_int = (uint16_t)(rate2 * 100);
        uint16_t rate3_int = (uint16_t)(rate3 * 100);
        
        replyData[2] = (rate1_int >> 8) & 0xFF;
        replyData[3] = rate1_int & 0xFF;
        replyData[4] = (rate2_int >> 8) & 0xFF;
        replyData[5] = rate2_int & 0xFF;
        replyData[6] = (rate3_int >> 8) & 0xFF;
        replyData[7] = rate3_int & 0xFF;
        
        Serial.print("Flow rates - Meter 1: ");
        Serial.print(rate1);
        Serial.print(" L/min, Meter 2: ");
        Serial.print(rate2);
        Serial.print(" L/min, Meter 3: ");
        Serial.print(rate3);
        Serial.println(" L/min");
        
        send_twai_response(replyData);
      }
      break;
      
    case 0x09: // Set solenoid valve state
      {
        byte solenoid_number = instruction.data[1];
        byte state = instruction.data[2]; // 0 = close, 1 = open
        Serial.print("Setting solenoid ");
        Serial.print(solenoid_number);
        Serial.print(" to ");
        Serial.println(state ? "OPEN" : "CLOSED");
        
        if (solenoid_number >= 1 && solenoid_number <= 3) {
          set_solenoid_state(solenoid_number, state == 1);
          replyData[1] = 0x01; // OK
          replyData[2] = solenoid_number;
          replyData[3] = state;
          Serial.print("Solenoid ");
          Serial.print(solenoid_number);
          Serial.print(" set to ");
          Serial.println(state ? "OPEN" : "CLOSED");
        } else {
          replyData[1] = 0x02; // FAIL - Invalid solenoid number
          Serial.println("Invalid solenoid number");
        }
        send_twai_response(replyData);
      }
      break;
      
    case 0x0A: // Get solenoid valve state
      {
        byte solenoid_number = instruction.data[1];
        Serial.print("Reading solenoid ");
        Serial.print(solenoid_number);
        Serial.println(" state");
        
        if (solenoid_number >= 1 && solenoid_number <= 3) {
          bool state = get_solenoid_state(solenoid_number);
          replyData[1] = 0x01; // OK
          replyData[2] = solenoid_number;
          replyData[3] = state ? 0x01 : 0x00; // 1 = open, 0 = closed
          Serial.print("Solenoid ");
          Serial.print(solenoid_number);
          Serial.print(" state: ");
          Serial.println(state ? "OPEN" : "CLOSED");
        } else {
          replyData[1] = 0x02; // FAIL - Invalid solenoid number
          Serial.println("Invalid solenoid number");
        }
        send_twai_response(replyData);
      }
      break;
      
    case 0x0B: // Toggle solenoid valve
      {
        byte solenoid_number = instruction.data[1];
        Serial.print("Toggling solenoid ");
        Serial.println(solenoid_number);
        
        if (solenoid_number >= 1 && solenoid_number <= 3) {
          bool old_state = get_solenoid_state(solenoid_number);
          toggle_solenoid(solenoid_number);
          bool new_state = get_solenoid_state(solenoid_number);
          
          replyData[1] = 0x01; // OK
          replyData[2] = solenoid_number;
          replyData[3] = new_state ? 0x01 : 0x00; // New state
          Serial.print("Solenoid ");
          Serial.print(solenoid_number);
          Serial.print(" toggled from ");
          Serial.print(old_state ? "OPEN" : "CLOSED");
          Serial.print(" to ");
          Serial.println(new_state ? "OPEN" : "CLOSED");
        } else {
          replyData[1] = 0x02; // FAIL - Invalid solenoid number
          Serial.println("Invalid solenoid number");
        }
        send_twai_response(replyData);
      }
      break;
      
    case 0x0C: // Get all solenoid states
      {
        Serial.println("Reading all solenoid states");
        
        bool state1 = get_solenoid_state(1);
        bool state2 = get_solenoid_state(2);
        bool state3 = get_solenoid_state(3);
        
        replyData[1] = 0x01; // OK
        replyData[2] = state1 ? 0x01 : 0x00; // Solenoid 1 state
        replyData[3] = state2 ? 0x01 : 0x00; // Solenoid 2 state
        replyData[4] = state3 ? 0x01 : 0x00; // Solenoid 3 state
        
        Serial.print("Solenoid states - 1: ");
        Serial.print(state1 ? "OPEN" : "CLOSED");
        Serial.print(", 2: ");
        Serial.print(state2 ? "OPEN" : "CLOSED");
        Serial.print(", 3: ");
        Serial.println(state3 ? "OPEN" : "CLOSED");
        
        send_twai_response(replyData);
      }
      break;
      
    case 0x0D: // Emergency close all solenoids
      {
        Serial.println("Emergency close all solenoids");
        close_all_solenoids();
        
        replyData[1] = 0x01; // OK
        replyData[2] = 0x00; // All closed
        replyData[3] = 0x00; // All closed
        replyData[4] = 0x00; // All closed
        
        Serial.println("All solenoids closed");
        send_twai_response(replyData);
      }
      break;
      
    // Add your custom command cases here
    // Example:
    // case 0x03: // Your custom command
    //   Serial.println("Custom command received");
    //   // Add your logic here
    //   replyData[1] = 0x01; // OK
    //   send_twai_response(replyData);
    //   break;
      
    default:
      Serial.print("Unknown command: 0x");
      Serial.println(command, HEX);
      replyData[1] = 0x02; // FAIL - Unknown command
      send_twai_response(replyData);
      break;
  }
}

void send_twai_response(byte response[8]) {
  txFrame.identifier = DEVICE_CAN_ID;
  txFrame.data_length_code = 8;
  memcpy(txFrame.data, response, 8);
  
  if (ESP32Can.writeFrame(txFrame, 1000)) {
    Serial.println("Response sent successfully");
  } else {
    Serial.println("Failed to send response");
  }
}

bool wait_for_twai_reply(unsigned long timeout_ms) {
  unsigned long start_time = millis();
  
  while (millis() - start_time < timeout_ms) {
    if (ESP32Can.readFrame(rxFrame, 10)) {
      if (rxFrame.identifier == DEVICE_CAN_ID) {
        return true;
      }
    }
    delay(1);
  }
  
  return false;
}

// Level sensor functions
bool read_level_sensor(int sensor_number) {
  int pin;
  
  switch (sensor_number) {
    case 1:
      pin = LEVEL_SENSOR_1_PIN;
      break;
    case 2:
      pin = LEVEL_SENSOR_2_PIN;
      break;
    case 3:
      pin = LEVEL_SENSOR_3_PIN;
      break;
    default:
      return false; // Invalid sensor number
  }
  
  // Float switches are normally open, close when liquid level is reached
  // With INPUT_PULLUP, LOW means liquid detected, HIGH means no liquid
  return !digitalRead(pin); // Return true if liquid detected
}

void get_all_level_sensors(bool levels[3]) {
  levels[0] = read_level_sensor(1); // Low level sensor
  levels[1] = read_level_sensor(2); // Medium level sensor
  levels[2] = read_level_sensor(3); // High level sensor
}

// Flow meter interrupt service routines
void IRAM_ATTR flowMeter1ISR() {
  flowMeter1PulseCount++;
  lastFlowMeter1Time = millis();
}

void IRAM_ATTR flowMeter2ISR() {
  flowMeter2PulseCount++;
  lastFlowMeter2Time = millis();
}

void IRAM_ATTR flowMeter3ISR() {
  flowMeter3PulseCount++;
  lastFlowMeter3Time = millis();
}

// Calculate flow rates for all meters
void calculate_flow_rates() {
  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - lastFlowCalculationTime;
  
  if (timeDiff >= 1000) { // Calculate every second
    // Calculate flow rate in L/min
    flowMeter1Rate = (flowMeter1PulseCount / FLOW_CALIBRATION_FACTOR) * (60000.0 / timeDiff);
    flowMeter2Rate = (flowMeter2PulseCount / FLOW_CALIBRATION_FACTOR) * (60000.0 / timeDiff);
    flowMeter3Rate = (flowMeter3PulseCount / FLOW_CALIBRATION_FACTOR) * (60000.0 / timeDiff);
    
    // Reset pulse counts for next calculation
    flowMeter1PulseCount = 0;
    flowMeter2PulseCount = 0;
    flowMeter3PulseCount = 0;
    
    lastFlowCalculationTime = currentTime;
  }
}

// Get flow rate for specific meter
float get_flow_rate(int meter_number) {
  calculate_flow_rates(); // Update rates before returning
  
  switch (meter_number) {
    case 1:
      return flowMeter1Rate;
    case 2:
      return flowMeter2Rate;
    case 3:
      return flowMeter3Rate;
    default:
      return -1.0; // Invalid meter number
  }
}

// Get total pulse count for specific meter
unsigned long get_total_pulses(int meter_number) {
  switch (meter_number) {
    case 1:
      return flowMeter1PulseCount;
    case 2:
      return flowMeter2PulseCount;
    case 3:
      return flowMeter3PulseCount;
    default:
      return 0; // Invalid meter number
  }
}

// Reset flow meter pulse count
void reset_flow_meter(int meter_number) {
  switch (meter_number) {
    case 1:
      flowMeter1PulseCount = 0;
      flowMeter1Rate = 0.0;
      break;
    case 2:
      flowMeter2PulseCount = 0;
      flowMeter2Rate = 0.0;
      break;
    case 3:
      flowMeter3PulseCount = 0;
      flowMeter3Rate = 0.0;
      break;
  }
}

// Set solenoid valve state (open/close)
void set_solenoid_state(int solenoid_number, bool state) {
  switch (solenoid_number) {
    case 1:
      digitalWrite(SOLENOID_1_PIN, state ? HIGH : LOW);
      solenoid1State = state;
      solenoid1LastToggleTime = millis();
      break;
    case 2:
      digitalWrite(SOLENOID_2_PIN, state ? HIGH : LOW);
      solenoid2State = state;
      solenoid2LastToggleTime = millis();
      break;
    case 3:
      digitalWrite(SOLENOID_3_PIN, state ? HIGH : LOW);
      solenoid3State = state;
      solenoid3LastToggleTime = millis();
      break;
  }
}

// Get solenoid valve state
bool get_solenoid_state(int solenoid_number) {
  switch (solenoid_number) {
    case 1:
      return solenoid1State;
    case 2:
      return solenoid2State;
    case 3:
      return solenoid3State;
    default:
      return false; // Invalid solenoid number
  }
}

// Toggle solenoid valve state
void toggle_solenoid(int solenoid_number) {
  bool current_state = get_solenoid_state(solenoid_number);
  set_solenoid_state(solenoid_number, !current_state);
}

// Close all solenoid valves (emergency function)
void close_all_solenoids() {
  set_solenoid_state(1, false);
  set_solenoid_state(2, false);
  set_solenoid_state(3, false);
}