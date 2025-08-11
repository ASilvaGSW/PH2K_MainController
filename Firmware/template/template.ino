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
#include <mcp_can.h>
#include <SPI.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 8  // Size in bytes for EEPROM storage
#define COUNTER_ADDR 0  // Starting address for counters

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 16  // CS for MCP2515 (Local Network)
#define CAN1_INT 17  // INT for MCP2515 (Local Network)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x001
#define RESPONSE_CAN_ID 0x401  // Response ID (DEVICE_CAN_ID + 0x400)

// Status codes
#define STATUS_OK 0x01
#define STATUS_FAIL 0x02
#define STATUS_TIMEOUT 0x03
#define STATUS_NO_NETWORK 0x04

// Global variables
unsigned long operationCounter = 0;  // Example counter stored in EEPROM
byte replyData[8] = {0};  // Buffer for CAN replies

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Queue to store instructions received from the main CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Function declarations
void process_instruction(CanFrame instruction);
uint8_t waitForCanReply(uint16_t expectedId);
void send_twai_response(const byte response_data[8]);
void saveCounter();
void loadCounter();
void incrementCounter();

// Function to save counter to EEPROM
void saveCounter() {
  EEPROM.writeULong(COUNTER_ADDR, operationCounter);
  EEPROM.commit();
  Serial.print("Counter saved: ");
  Serial.println(operationCounter);
}

// Function to load counter from EEPROM
void loadCounter() {
  operationCounter = EEPROM.readULong(COUNTER_ADDR);
  if (operationCounter == 0xFFFFFFFF) {  // First time or EEPROM not initialized
    operationCounter = 0;
    saveCounter();
  }
  Serial.print("Loaded counter from EEPROM: ");
  Serial.println(operationCounter);
}

// Function to increment and save the counter
void incrementCounter() {
  operationCounter++;
  saveCounter();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadCounter();

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

  // Initialize the second CAN bus (MCP2515 - Local Network)
  Serial.println("Initializing CAN1 (MCP2515)...");
  if (CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 initialized successfully");
    CAN1.setMode(MCP_NORMAL);
  } else {
    Serial.println("Error initializing MCP2515");
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
void twai_listener_task(void *pvParameters) {
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
void loop() {
  CanFrame instruction;
  
  // Process any pending instructions with a 10ms timeout
  if (xQueueReceive(instruction_queue, &instruction, pdMS_TO_TICKS(10)) == pdPASS) {
    Serial.print("Processing command: 0x");
    Serial.println(instruction.data[0], HEX);
    process_instruction(instruction);
  }
  
  // Add other non-blocking tasks here
  // Example: Read sensors, update displays, etc.
  
  vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent watchdog triggers
}

void process_instruction(CanFrame instruction) {
  Serial.print("Processing command: 0x");
  Serial.println(instruction.data[0], HEX);

  byte response_data[8] = {0};
  bool success = false;

  // The timeout for the local network reply can be dynamic
  // We'll use data[1] as a multiplier for 100ms. Default to 1s if 0.
  unsigned int timeout = instruction.data[1] > 0 ? instruction.data[1] * 100 : 1000;

  switch (instruction.data[0]) {
    case 0x01: // Example instruction: PING
      {
        Serial.println("Case 0x01: PING");
        byte temp[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        memcpy(response_data, temp, sizeof(temp));
      }
      break;

    case 0x02: // Example instruction: CALL Slave
      {
        Serial.println("Case 0x02: Sending motor speed command to local network");
        byte request_to_local[] = {0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        
        if (CAN1.sendMsgBuf(0x002, 0, 8, request_to_local) != CAN_OK) {
          Serial.println("Error sending via CAN1");
          response_data[0] = 0x02; // Función
          response_data[1] = 0x02; // FAIL
          break;
        }

        // Wait for a response from the local network with dynamic timeout
        Serial.print("Waiting for reply with timeout: ");
        Serial.print(timeout);
        Serial.println("ms");
        uint8_t status = waitForCanReply(0x402, response_data, timeout);
        response_data[0] = 0x02;
        response_data[1] = status;
      }
      break;

    default:
      Serial.println("Unknown command.");
      response_data[0] = 0xFD; // Unknown command error code
      break;
  }

  // Respond to the main bus (TWAI)
  CanFrame response_frame = {0};
  response_frame.identifier = instruction.identifier + 0x400; // Reply with the same ID as received
  response_frame.extd = 0;
  response_frame.data_length_code = 8;
  memcpy(response_frame.data, response_data, 8);
  
  ESP32Can.writeFrame(response_frame);
  Serial.println("Response sent to the main bus (TWAI).");
}

// Devuelve 0x01 si nos respondieron con 1 en el segundo byte,
// 0x02 si nos respondieron con 2,
// 0x03 si hubo timeout.
bool allResponsesReceived(bool* received, int numIds) {
    for (int i = 0; i < numIds; i++) {
        if (!received[i]) {
            return false;
        }
    }
    return true;
}

uint8_t waitForCanReply(unsigned long* expectedIds, int numIds, byte* replyData, unsigned int timeoutMillis) {
    unsigned long startTime = millis();
    bool received[numIds];
    for (int i = 0; i < numIds; i++) {
        received[i] = false;
    }

    while (millis() - startTime < timeoutMillis) {
        if (CAN_MSGAVAIL == CAN1.checkReceive()) {
            unsigned char len = 0;
            unsigned long canId;
            CAN1.readMsgBuf(&canId, &len, replyData);

            for (int i = 0; i < numIds; i++) {
                if (canId == expectedIds[i] && !received[i]) {
                    Serial.print("Valid response received from ID: ");
                    Serial.println(canId, HEX);
                    received[i] = true;
                    if (replyData[1] == 0x02) return 0x02; // FAIL, return immediately
                    break; // Exit the for loop once ID is found
                }
            }

            if (allResponsesReceived(received, numIds)) {
                Serial.println("All responses received successfully.");
                return 0x01; // OK
            }
        }
    }

    Serial.println("Timeout: Not all responses received from local network.");
    return 0x03; // Timeout
}

// Overloaded function for a single ID to maintain backward compatibility
uint8_t waitForCanReply(unsigned long expectedId, byte* replyData, unsigned int timeoutMillis) {
    unsigned long ids[] = {expectedId};
    return waitForCanReply(ids, 1, replyData, timeoutMillis);
}