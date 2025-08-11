/*
 * template.ino
 * -----------
 * Generic template for dual-CAN (Controller Area Network) based ESP32 projects.
 * 
 * FEATURES:
 *   - Dual CAN bus support: ESP32 TWAI (integrated) and MCP2515 (external)
 *   - FreeRTOS for multitasking
 *   - EEPROM for persistent storage of counters and configuration
 *   - Robust CAN communication with error handling
 *   - Modular design for easy extension
 *
 * DEPENDENCIES:
 *   - ESP32-TWAI-CAN
 *   - mcp_can
 *   - SPI
 *   - FreeRTOS (ESP32)
 *   - EEPROM (ESP32)
 */

#include <ESP32-TWAI-CAN.hpp>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage
#include <ESP32Servo.h>

// EEPROM Configuration
#define EEPROM_SIZE 36  // Size in bytes for EEPROM storage (4 bytes per counter * 9 counters)
#define COUNTER_ADDR 0  // Starting address for main counter

// Servo counter addresses in EEPROM
#define SLIDER_JOINT_COUNTER_ADDR 4
#define CLAMP_JOINT_COUNTER_ADDR 8
#define HOLDER_HOSE_J_COUNTER_ADDR 12
#define SLIDER_NOZZLE_COUNTER_ADDR 16
#define CLAMP_NOZZLE_COUNTER_ADDR 20
#define HOLDER_HOSE_N_COUNTER_ADDR 24
#define CUTTER_COUNTER_ADDR 28

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x002
#define RESPONSE_CAN_ID 0x402  // Response ID (DEVICE_CAN_ID + 0x400)

// Status codes
#define STATUS_OK 0x01
#define STATUS_FAIL 0x02
#define STATUS_TIMEOUT 0x03
#define STATUS_NO_NETWORK 0x04

// Global variables
unsigned long operationCounter = 0;  // Example counter stored in EEPROM
byte replyData[8] = {0};  // Buffer for CAN replies

// Servo movement counters
unsigned long sliderJointCounter = 0;
unsigned long clampJointCounter = 0;
unsigned long holderHoseJCounter = 0;
unsigned long sliderNozzleCounter = 0;
unsigned long clampNozzleCounter = 0;
unsigned long holderHoseNCounter = 0;
unsigned long cutterCounter = 0;

// Ports for Servos
#define SLIDER_JOINT 18 
#define CLAMP_JOINT 17
#define HOLDER_HOSE_J 16

#define SLIDER_NOZZLE 15
#define CLAMP_NOZZLE 14
#define HOLDER_HOSE_N 21

#define CUTTER 19


// Defined Angles

#define SLIDER_JOINT_MIN 0
#define SLIDER_JOINT_MAX 160

#define CLAMP_JOINT_MIN 20
#define CLAMP_JOINT_MAX 110

#define HOLDER_HOSE_J_MIN 0
#define HOLDER_HOSE_J_MAX 125

#define SLIDER_NOZZLE_MIN 125
#define SLIDER_NOZZLE_MAX 0

#define CLAMP_NOZZLE_MIN 40
#define CLAMP_NOZZLE_MAX 140

#define HOLDER_HOSE_N_MIN 0
#define HOLDER_HOSE_N_MAX 120

#define CUTTER_MIN 0
#define CUTTER_MAX 84


// Instance for the Servos

Servo SliderJoint;
Servo ClampJoint;
Servo HolderHoseJ;

Servo SliderNozzle;
Servo ClampNozzle;
Servo HolderHoseN;

Servo Cutter;

// Queue to store instructions received from the main CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Function declarations
void process_instruction(CanFrame instruction);
void send_twai_response(const byte response_data[8]);
void saveCounter();
void loadCounter();
void incrementCounter();

// Servo counter functions
void saveServoCounter(unsigned long counter, int address);
void loadServoCounters();
void incrementServoCounter(int servoId);
void resetServoCounter(int servoId);

// Function to save counter to EEPROM
void saveCounter()
{
  EEPROM.writeULong(COUNTER_ADDR, operationCounter);
  EEPROM.commit();
  Serial.print("Counter saved: ");
  Serial.println(operationCounter);
}

// Function to load counter from EEPROM
void loadCounter()
{
  operationCounter = EEPROM.readULong(COUNTER_ADDR);
  if (operationCounter == 0xFFFFFFFF) {  // First time or EEPROM not initialized
    operationCounter = 0;
    saveCounter();
  }
  Serial.print("Loaded counter from EEPROM: ");
  Serial.println(operationCounter);
}

// Function to increment and save the counter
void incrementCounter()
{
  operationCounter++;
  saveCounter();
}

// Function to save a servo counter to EEPROM
void saveServoCounter(unsigned long counter, int address)
{
  EEPROM.writeULong(address, counter);
  EEPROM.commit();
}

// Function to load all servo counters from EEPROM
void loadServoCounters()
{
  sliderJointCounter = EEPROM.readULong(SLIDER_JOINT_COUNTER_ADDR);
  if (sliderJointCounter == 0xFFFFFFFF) {
    sliderJointCounter = 0;
    saveServoCounter(sliderJointCounter, SLIDER_JOINT_COUNTER_ADDR);
  }
  
  clampJointCounter = EEPROM.readULong(CLAMP_JOINT_COUNTER_ADDR);
  if (clampJointCounter == 0xFFFFFFFF) {
    clampJointCounter = 0;
    saveServoCounter(clampJointCounter, CLAMP_JOINT_COUNTER_ADDR);
  }
  
  holderHoseJCounter = EEPROM.readULong(HOLDER_HOSE_J_COUNTER_ADDR);
  if (holderHoseJCounter == 0xFFFFFFFF) {
    holderHoseJCounter = 0;
    saveServoCounter(holderHoseJCounter, HOLDER_HOSE_J_COUNTER_ADDR);
  }
  
  sliderNozzleCounter = EEPROM.readULong(SLIDER_NOZZLE_COUNTER_ADDR);
  if (sliderNozzleCounter == 0xFFFFFFFF) {
    sliderNozzleCounter = 0;
    saveServoCounter(sliderNozzleCounter, SLIDER_NOZZLE_COUNTER_ADDR);
  }
  
  clampNozzleCounter = EEPROM.readULong(CLAMP_NOZZLE_COUNTER_ADDR);
  if (clampNozzleCounter == 0xFFFFFFFF) {
    clampNozzleCounter = 0;
    saveServoCounter(clampNozzleCounter, CLAMP_NOZZLE_COUNTER_ADDR);
  }
  
  holderHoseNCounter = EEPROM.readULong(HOLDER_HOSE_N_COUNTER_ADDR);
  if (holderHoseNCounter == 0xFFFFFFFF) {
    holderHoseNCounter = 0;
    saveServoCounter(holderHoseNCounter, HOLDER_HOSE_N_COUNTER_ADDR);
  }
  
  cutterCounter = EEPROM.readULong(CUTTER_COUNTER_ADDR);
  if (cutterCounter == 0xFFFFFFFF) {
    cutterCounter = 0;
    saveServoCounter(cutterCounter, CUTTER_COUNTER_ADDR);
  }
  
  Serial.println("Loaded all servo counters from EEPROM");
}

// Function to increment a specific servo counter
void incrementServoCounter(int servoId)
{
  switch(servoId) {
    case 1: // Slider Joint
      sliderJointCounter++;
      saveServoCounter(sliderJointCounter, SLIDER_JOINT_COUNTER_ADDR);
      break;
    case 2: // Clamp Joint
      clampJointCounter++;
      saveServoCounter(clampJointCounter, CLAMP_JOINT_COUNTER_ADDR);
      break;
    case 3: // Holder Hose Joint
      holderHoseJCounter++;
      saveServoCounter(holderHoseJCounter, HOLDER_HOSE_J_COUNTER_ADDR);
      break;
    case 4: // Slider Nozzle
      sliderNozzleCounter++;
      saveServoCounter(sliderNozzleCounter, SLIDER_NOZZLE_COUNTER_ADDR);
      break;
    case 5: // Clamp Nozzle
      clampNozzleCounter++;
      saveServoCounter(clampNozzleCounter, CLAMP_NOZZLE_COUNTER_ADDR);
      break;
    case 6: // Holder Hose Nozzle
      holderHoseNCounter++;
      saveServoCounter(holderHoseNCounter, HOLDER_HOSE_N_COUNTER_ADDR);
      break;
    case 7: // Cutter
      cutterCounter++;
      saveServoCounter(cutterCounter, CUTTER_COUNTER_ADDR);
      break;
  }
}

// Function to reset a specific servo counter
void resetServoCounter(int servoId)
{
  switch(servoId) {
    case 1: // Slider Joint
      sliderJointCounter = 0;
      saveServoCounter(sliderJointCounter, SLIDER_JOINT_COUNTER_ADDR);
      break;
    case 2: // Clamp Joint
      clampJointCounter = 0;
      saveServoCounter(clampJointCounter, CLAMP_JOINT_COUNTER_ADDR);
      break;
    case 3: // Holder Hose Joint
      holderHoseJCounter = 0;
      saveServoCounter(holderHoseJCounter, HOLDER_HOSE_J_COUNTER_ADDR);
      break;
    case 4: // Slider Nozzle
      sliderNozzleCounter = 0;
      saveServoCounter(sliderNozzleCounter, SLIDER_NOZZLE_COUNTER_ADDR);
      break;
    case 5: // Clamp Nozzle
      clampNozzleCounter = 0;
      saveServoCounter(clampNozzleCounter, CLAMP_NOZZLE_COUNTER_ADDR);
      break;
    case 6: // Holder Hose Nozzle
      holderHoseNCounter = 0;
      saveServoCounter(holderHoseNCounter, HOLDER_HOSE_N_COUNTER_ADDR);
      break;
    case 7: // Cutter
      cutterCounter = 0;
      saveServoCounter(cutterCounter, CUTTER_COUNTER_ADDR);
      break;
  }
}

// Seting up servos
void setup_servos()
{
  SliderJoint.attach(SLIDER_JOINT,500,2500);
  ClampJoint.attach(CLAMP_JOINT,500,2500);
  HolderHoseJ.attach(HOLDER_HOSE_J,500,2500);
  SliderNozzle.attach(SLIDER_NOZZLE,500,2500);
  ClampNozzle.attach(CLAMP_NOZZLE,500,2500);
  HolderHoseN.attach(HOLDER_HOSE_N,500,2500);
  Cutter.attach(CUTTER,500,2500);

  SliderJoint.write(SLIDER_JOINT_MIN);
  ClampJoint.write(CLAMP_JOINT_MIN);
  HolderHoseJ.write(HOLDER_HOSE_J_MIN);
  SliderNozzle.write(SLIDER_NOZZLE_MIN);
  ClampNozzle.write(CLAMP_NOZZLE_MIN);
  HolderHoseN.write(HOLDER_HOSE_N_MIN);
  Cutter.write(CUTTER_MIN);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadCounter();
  loadServoCounters();


  setup_servos();


  // Create the instruction queue
  instruction_queue = xQueueCreate(10, sizeof(CanFrame));
  if (instruction_queue == NULL) {
    Serial.println("Error creating instruction queue");
    while(1);
  }

  // Initialize the first CAN bus (TWAI - General Network)
  Serial.println("Initializing CAN0 (TWAI)...");
  ESP32Can.setPins(CAN0_TX, CAN0_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(125));
  if (ESP32Can.begin()) {
    Serial.println("CAN0 (TWAI) initialized successfully");
  } else {
    Serial.println("Error initializing CAN0 (TWAI)");
    while (1);
  }


  // Create the task to listen on the TWAI bus on core 0
  xTaskCreatePinnedToCore(
      twai_listener_task,   /* Task function */
      "TWAICanListener",    /* Task name */
      4096,                 /* Stack size */
      NULL,                 /* Task parameters */
      1,                    /* Task priority */
      NULL,                 /* Task handle */
      0);                   /* Core to run on (0) */

  Serial.println("CAN system ready. Main loop running on core 1.");
}

/**
 * Task running on core 0 to listen to the TWAI bus
 * Continuously listens for CAN frames and queues them for processing
 */
void twai_listener_task(void *pvParameters)
{
  Serial.println("TWAI listener task started on core 0");
  CanFrame rxFrame;
  
  for (;;) {
    if (ESP32Can.readFrame(rxFrame)) {
      // Only process frames with our device ID
      if (rxFrame.identifier == DEVICE_CAN_ID) {
        Serial.print("CAN0 (TWAI) - Instruction received ID: 0x");
        Serial.println(rxFrame.identifier, HEX);
        
        // Try to send to queue with 10ms timeout
        if (xQueueSend(instruction_queue, &rxFrame, pdMS_TO_TICKS(10)) != pdPASS) {
          Serial.println("Error: Instruction queue is full");
          
          // Send a failure response if queue is full
          byte response[8] = {rxFrame.data[0], STATUS_FAIL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          send_twai_response(response);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to avoid saturating the CPU
  }
}

/**
 * Main loop running on core 1
 * Processes instructions from the queue and handles other non-blocking tasks
 */
void loop()
{
  CanFrame instruction;
  
  // Process any pending instructions with a 10ms timeout
  if (xQueueReceive(instruction_queue, &instruction, pdMS_TO_TICKS(10)) == pdPASS) {
    Serial.print("Processing command: 0x");
    Serial.println(instruction.data[0], HEX);
    process_instruction(instruction);
  }
    
  vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent watchdog triggers
}

void process_instruction(CanFrame instruction)
{
  Serial.print("Processing command: 0x");
  Serial.println(instruction.data[0], HEX);

  byte response_data[8] = {0};
  bool success = false;

  // The timeout for the local network reply can be dynamic
  // We'll use data[1] as a multiplier for 100ms. Default to 1s if 0.
  unsigned int timeout = instruction.data[1] > 0 ? instruction.data[1] * 100 : 1000;

  switch (instruction.data[0])
  {
    case 0x01: // Restart Microcontroller
      {
        Serial.println("Case 0x01: Restart Microcontroller");
        byte response[8] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(response);
        ESP.restart();
      }
      break;

    case 0x02: // Hearhtbeat
      {
        Serial.println("Case 0x02: Hearthbeat");
        byte response[8] = {0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      
      }
      break;

    case 0x03: // Move Slider Joint
      {
        Serial.println("Case 0x03: Moving Slider Joint");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        SliderJoint.write(angle);
        incrementServoCounter(1); // Increment Slider Joint counter
        
        byte response[8] = {0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x04: // Move Clamp Joint
      {
        Serial.println("Case 0x04: Moving Clamp Joint");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        ClampJoint.write(angle);
        incrementServoCounter(2); // Increment Clamp Joint counter
        
        byte response[8] = {0x04,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x05: // Move Holder Hose Joint
      {
        Serial.println("Case 0x05: Moving Holder Hose Joint");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        HolderHoseJ.write(angle);
        incrementServoCounter(3); // Increment Holder Hose Joint counter
        
        byte response[8] = {0x05,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x06: // Move Slider Nozzle
      {
        Serial.println("Case 0x06: Moving Slider Nozzle");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        SliderNozzle.write(angle);
        incrementServoCounter(4); // Increment Slider Nozzle counter
        
        byte response[8] = {0x06,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x07: // Move Clamp Nozzle
      {
        Serial.println("Case 0x07: Moving Clamp Nozzle");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        ClampNozzle.write(angle);
        incrementServoCounter(5); // Increment Clamp Nozzle counter
        
        byte response[8] = {0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x08: // Move Holder Hose Nozzle
      {
        Serial.println("Case 0x08: Moving Holder Hose Nozzle");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        HolderHoseN.write(angle);
        incrementServoCounter(6); // Increment Holder Hose Nozzle counter
        
        byte response[8] = {0x08,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x09: // Move Cutter
      {
        Serial.println("Case 0x09: Moving Cutter");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        Cutter.write(angle);
        incrementServoCounter(7); // Increment Cutter counter
        
        byte response[8] = {0x09,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;

    case 0x0A: // Read Servo Counter
      {
        Serial.println("Case 0x0A: Reading Servo Counter");
        byte servoId = instruction.data[1];
        unsigned long counter = 0;
        
        switch(servoId) {
          case 1: counter = sliderJointCounter; break;
          case 2: counter = clampJointCounter; break;
          case 3: counter = holderHoseJCounter; break;
          case 4: counter = sliderNozzleCounter; break;
          case 5: counter = clampNozzleCounter; break;
          case 6: counter = holderHoseNCounter; break;
          case 7: counter = cutterCounter; break;
          default: counter = 0xFFFFFFFF; // Invalid servo ID
        }
        
        byte response[8] = {
          0x0A, 
          STATUS_OK, 
          (byte)((counter >> 24) & 0xFF),  // Most significant byte first
          (byte)((counter >> 16) & 0xFF),
          (byte)((counter >> 8) & 0xFF),
          (byte)(counter & 0xFF),          // Least significant byte last
          0x00,
          0x00
        };
        send_twai_response(response);
      }
      break;
      
    case 0x0B: // Reset Servo Counter
      {
        Serial.println("Case 0x0B: Resetting Servo Counter");
        byte servoId = instruction.data[1];
        
        if (servoId >= 1 && servoId <= 7) {
          resetServoCounter(servoId);
          byte response[8] = {0x0B, STATUS_OK, servoId, 0x00, 0x00, 0x00, 0x00, 0x00};
          send_twai_response(response);
        } else {
          byte response[8] = {0x0B, STATUS_FAIL, servoId, 0x00, 0x00, 0x00, 0x00, 0x00};
          send_twai_response(response);
        }
      }
      break;

    default:
      Serial.println("Unknown command.");
      byte response[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Unknown command error code
      break;
  }
}

void send_twai_response(const byte response_data[8])
{
  CanFrame response_frame = {0};
  response_frame.identifier = RESPONSE_CAN_ID; // Use the device ID for responses
  response_frame.extd = 0;
  response_frame.data_length_code = 8;
  memcpy(response_frame.data, response_data, 8);
  
  ESP32Can.writeFrame(response_frame);
  Serial.println("Response sent to the main bus (TWAI).");
}