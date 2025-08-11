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
#define EEPROM_SIZE 8  // Size in bytes for EEPROM storage
#define COUNTER_ADDR 0  // Starting address for counters

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
        
        byte response[8] = {0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x04: // Move Clamp Joint
      {
        Serial.println("Case 0x04: Moving Clamp Joint");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        ClampJoint.write(angle);
        
        byte response[8] = {0x04,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x05: // Move Holder Hose Joint
      {
        Serial.println("Case 0x05: Moving Holder Hose Joint");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        HolderHoseJ.write(angle);
        
        byte response[8] = {0x05,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x06: // Move Slider Nozzle
      {
        Serial.println("Case 0x06: Moving Slider Nozzle");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        SliderNozzle.write(angle);
        
        byte response[8] = {0x06,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x07: // Move Clamp Nozzle
      {
        Serial.println("Case 0x07: Moving Clamp Nozzle");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        ClampNozzle.write(angle);
        
        byte response[8] = {0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x08: // Move Holder Hose Nozzle
      {
        Serial.println("Case 0x08: Moving Holder Hose Nozzle");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        HolderHoseN.write(angle);
        
        byte response[8] = {0x08,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
      }
      break;
    
    case 0x09: // Move Cutter
      {
        Serial.println("Case 0x09: Moving Cutter");
        int angle = (instruction.data[1] << 8) | instruction.data[2];
        Cutter.write(angle);
        
        byte response[8] = {0x09,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
        send_twai_response(response);
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