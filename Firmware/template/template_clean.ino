/*
 * template_clean.ino
 * -----------------
 * Clean template firmware for dual-CAN (Controller Area Network) based system using ESP32.
 * This template provides basic CAN bus functionality and command structure for adding new components.
 *
 * FEATURES:
 *   - Dual CAN bus support: ESP32 TWAI (integrated) and MCP2515 (external) for communication with general and local networks.
 *   - Implements FreeRTOS for multitasking, using queues for instruction dispatch.
 *   - Robust command processing with status reporting and error handling via CAN.
 *   - Modular design ready for adding custom components.
 *
 * ARCHITECTURE OVERVIEW:
 *   - setup(): Initializes peripherals, CAN buses, and FreeRTOS tasks.
 *   - loop(): Main event loop, processes queued CAN instructions.
 *   - twai_listener_task(): Listens for incoming CAN instructions and queues them for processing.
 *   - process_instruction(): Decodes and executes commands from CAN, ready for component dispatch.
 *   - Helper functions for CAN reply waiting and CAN response sending.
 *
 * COMMANDS (CAN):
 *   - 0x01: Reset microcontroller
 *   - 0x02: Heartbeat
 *   - 0xFF: Power off command
 *   - Add your custom commands here...
 *
 * DEPENDENCIES:
 *   - ESP32-TWAI-CAN
 *   - mcp_can
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
#include <mcp_can.h>
#include <SPI.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Global variables
byte replyData[8];  // Buffer for CAN replies

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 26   // GPIO26 - CS pin for MCP2515
#define CAN1_INT 25  // GPIO25 - INT pin for MCP2515

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x192

// CAN instances
CanFrame rxFrame, txFrame;
MCP_CAN CAN1(CAN1_CS);

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

void setup() {
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

  // Initialize CAN1 (MCP2515)
  if (CAN1.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN1 (MCP2515) initialized successfully");
    CAN1.setMode(MCP_NORMAL);
  } else {
    Serial.println("CAN1 (MCP2515) initialization failed");
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
    
    // Check for MCP2515 messages
    if (!digitalRead(CAN1_INT)) {
      long unsigned int rxId;
      unsigned char len = 0;
      unsigned char rxBuf[8];
      
      CAN1.readMsgBuf(&rxId, &len, rxBuf);
      
      if (rxId == DEVICE_CAN_ID) {
        instruction.id = rxId;
        instruction.len = len;
        memcpy(instruction.data, rxBuf, len);
        
        if (xQueueSend(instructionQueue, &instruction, pdMS_TO_TICKS(100)) != pdTRUE) {
          Serial.println("Failed to queue MCP2515 instruction");
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void process_instruction(CANInstruction instruction) {
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