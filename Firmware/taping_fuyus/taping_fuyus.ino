/*
 * taping_fuyus.ino
 * -----------------
 * Firmware for a dual-CAN (Controller Area Network) based taping robotic system using ESP32.
 *
 * FEATURES:
 *   - Dual CAN bus support: ESP32 TWAI (integrated) and MCP2515 (external) for communication with general and local networks.
 *   - Controls two linear actuators (Y and Z axes) for taping operations.
 *   - Implements FreeRTOS for multitasking, using queues for instruction dispatch.
 *   - Stores and manages actuator movement counters in non-volatile EEPROM memory.
 *   - Robust command processing with status reporting and error handling via CAN.
 *   - Modular design using custom classes for actuators.
 *
 * ARCHITECTURE OVERVIEW:
 *   - setup(): Initializes peripherals, CAN buses, EEPROM, and FreeRTOS tasks.
 *   - loop(): Main event loop, processes queued CAN instructions.
 *   - twai_listener_task(): Listens for incoming CAN instructions and queues them for processing.
 *   - process_instruction(): Decodes and executes commands from CAN, dispatches to actuators as needed.
 *   - Helper functions for EEPROM, CAN reply waiting, and CAN response sending.
 *
 * COMMANDS (CAN):
 *   - 0x01: Reset microcontroller
 *   - 0x02: Heartbeat
 *   - 0x03: Home actuators (Y and Z)
 *   - 0x04: Move Y actuator to absolute position
 *   - 0x05: Move Z actuator to absolute position
 *   - 0x06: Read Z actuator movement counter
 *   - 0x07: Read Y actuator movement counter
 *   - 0x08: Reset Y movement counter
 *   - 0x09: Reset Z movement counter
 *   - 0x13: Home Y axis using go_home function
 *   - 0x14: Home Z axis using go_home function
 *   - 0x15: Move Y actuator to absolute position with speed control
 *   - 0xFF: Power off, home all axes
 *
 * DEPENDENCIES:
 *   - ESP32-TWAI-CAN
 *   - mcp_can
 *   - SPI
 *   - FreeRTOS (ESP32)
 *   - EEPROM (ESP32)
 *   - Custom: src/linear_actuator.h
 *
 * AUTHOR: Alan Silva
 * DATE: 2025-01-25
 * CONTACT : asilva@gswiring.com
 * DETAILED DOCUMENTATION:
 *   - Each function and major logic block is documented below for clarity and maintainability.
 *   - For protocol details and troubleshooting, see the project README and hardware wiring diagrams.
 */

#include <ESP32-TWAI-CAN.hpp>
#include <mcp_can.h>
#include <SPI.h>
#include "src/linear_actuator.h"

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 16  // 8 bytes for actuator counters (Y and Z)
#define ACTUATOR_COUNTER_ADDR 4  // Address in EEPROM to store the Y actuator counter
#define ACTUATOR_COUNTER_ADDR2 8 // Address in EEPROM to store the Z actuator counter

// Global variables
unsigned long actuatorMoveCounterY = 0;
unsigned long actuatorMoveCounterZ = 0;
byte replyData[8];  // Buffer for CAN replies

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 26   // GPIO26 - CS pin for MCP2515
#define CAN1_INT 25  // GPIO25 - INT pin for MCP2515 (FIXED: was GPIO4, conflicted with TWAI TX)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x007
#define RESPONSE_CAN_ID 0x407

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Instance of LinearActuator
LinearActuator y_axis(0x001);
LinearActuator z_axis(0x002);

const int ACTUATOR_HOME_POSITION = 0;

// Queue to store instructions received from the main CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Status :
// 1 -> OK
// 2 -> FAIL
// 3 -> TIMEOUT
// 4 -> NO LOCAL NETWORK

// Function declarations
void process_instruction(CanFrame instruction);
uint8_t waitForCanReply(uint16_t expectedId);
void send_twai_response(const byte response_data[8]);

// Function to save actuator counter to EEPROM
void saveActuatorCounterY()
{
  EEPROM.writeULong(ACTUATOR_COUNTER_ADDR, actuatorMoveCounterY);
  EEPROM.commit();
  Serial.print("Y Actuator counter saved: ");
  Serial.println(actuatorMoveCounterY);
}

void saveActuatorCounterZ()
{
  EEPROM.writeULong(ACTUATOR_COUNTER_ADDR2, actuatorMoveCounterZ);
  EEPROM.commit();
  Serial.print("Z Actuator counter saved: ");
  Serial.println(actuatorMoveCounterZ);
}

// Function to increment and save the actuator counter
void incrementActuatorCounterY()
{ 
  actuatorMoveCounterY++;
  saveActuatorCounterY();
}

void incrementActuatorCounterZ()
{ 
  actuatorMoveCounterZ++;
  saveActuatorCounterZ();
}

/*
 * setup()
 * -------
 * Initializes all peripherals, communication buses, memory, and FreeRTOS tasks.
 * - Sets up serial communication for debugging.
 * - Loads actuator movement counters from EEPROM (persistent storage).
 * - Creates a FreeRTOS queue for CAN instruction handling.
 * - Initializes both CAN buses (TWAI for general, MCP2515 for local network).
 * - Creates a dedicated FreeRTOS task for listening to the TWAI bus.
 * - Sends a startup message over CAN to indicate readiness.
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN taping system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  actuatorMoveCounterY = EEPROM.readULong(ACTUATOR_COUNTER_ADDR);
  actuatorMoveCounterZ = EEPROM.readULong(ACTUATOR_COUNTER_ADDR2);
  Serial.print("Loaded Y actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterY);
  Serial.print("Loaded Z actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterZ);

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
  
  // // Try 16MHz crystal first (most common)
  // Serial.println("Trying MCP2515 with 16MHz crystal...");
  // if (CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ) == CAN_OK) {
  //   Serial.println("MCP2515 initialized successfully with 16MHz crystal");
  //   CAN1.setMode(MCP_NORMAL);
  // } else {
  Serial.println("16MHz failed, trying 8MHz crystal...");
  if (CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 initialized successfully with 8MHz crystal");
    CAN1.setMode(MCP_NORMAL);
  } else {
    Serial.println("MCP2515 initialization failed with both 16MHz and 8MHz crystals");
    Serial.println("Check hardware connections:");
    Serial.println("- CS pin connected to GPIO26");
    Serial.println("- INT pin connected to GPIO25");
    Serial.println("- MOSI, MISO, SCK connected to ESP32 SPI pins");
    Serial.println("- VCC and GND properly connected");
    Serial.println("- Crystal frequency on MCP2515 module");
    // Don't halt execution, continue without MCP2515
    Serial.println("Continuing without MCP2515 (Local Network disabled)");
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

  Serial.println("CAN taping system ready. Main loop running on core 1.");

  // Send startup message
  byte startup_msg[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_twai_response(startup_msg);
  delay(100); // Give some time for the message to be sent
}

// Task running on core 0 to listen to the TWAI bus
/*
 * twai_listener_task(void *pvParameters)
 * --------------------------------------
 * FreeRTOS task running on core 0. Continuously listens for new CAN frames on the TWAI bus.
 * - Accepts only frames matching DEVICE_CAN_ID.
 * - Places valid instructions in the instruction_queue for processing by the main loop.
 * - Ignores frames not addressed to this device.
 * - Uses a small delay to avoid CPU saturation.
 */
void twai_listener_task(void *pvParameters)
{
  Serial.println("TWAI listener task started on core 0");
  CanFrame rxFrame;
  for (;;) {
    if (ESP32Can.readFrame(rxFrame)) {
      Serial.print("CAN0 (TWAI) - Instruction received ID: 0x");
      Serial.println(rxFrame.identifier, HEX);
      // Only accept instructions for this device
      if (rxFrame.identifier == DEVICE_CAN_ID) {
        if (xQueueSend(instruction_queue, &rxFrame, (TickType_t)10) != pdPASS) {
          Serial.println("Error: Instruction queue is full.");
        }
      } else {
        Serial.println("Ignored frame: ID does not match DEVICE_CAN_ID");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid saturating the CPU
  }
}

// The main loop runs on core 1 by default
/*
 * loop()
 * -------
 * Main event loop running on core 1.
 * - Checks for new instructions in the instruction_queue and processes them.
 * - Includes a small delay for stability.
 * - Additional non-blocking logic can be inserted here if needed.
 */
void loop()
{
  CanFrame instruction;
  // Is there any instruction in the queue?
  if (xQueueReceive(instruction_queue, &instruction, (TickType_t)0) == pdPASS)
  {
    Serial.println("Processing new instruction from queue...");
    process_instruction(instruction);
  }
  
  // Other non-blocking logic can be added here if needed
  delay(10);
}

/*
 * process_instruction(CanFrame instruction)
 * ----------------------------------------
 * Decodes and executes CAN instructions received from the queue.
 * - Each case in the switch statement corresponds to a command (see command list above).
 * - Handles actuator moves, counter reads/resets, and power-off.
 * - Sends appropriate CAN responses for each command, including error/status codes.
 * - Uses helper functions for actuator actions and CAN communication.
 *
 * PARAMS:
 *   instruction: CanFrame containing the received CAN message, including identifier and data payload.
 *
 * NOTES:
 *   - Many actions are non-blocking; responses may be sent before motion completes.
 *   - Error handling is robust: failures in CAN communication or timeouts are reported via response codes.
 *   - For actuator details, see respective class documentation in src/.
 */
void process_instruction(CanFrame instruction)
{
  Serial.print("Processing instruction with command: 0x");
  Serial.println(instruction.data[0], HEX);

  switch (instruction.data[0])
  {
    // ***************************** CASE 0x01 ***************************** //
    // Reset microcontroller
    case 0x01:
    {
      Serial.println("Case 0x01: Reset microcontroller");
      byte response[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
      delay(500);
      ESP.restart();
    }
    break;

    // ***************************** CASE 0x02 ***************************** //
    // Send Heartbeat
    case 0x02:
    {
      Serial.println("Case 0x02: Send Heartbeat");
      byte response[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x03 ***************************** //
    // Home actuators
    case 0x03:
    {
      Serial.println("Case 0x03: Homing all axes");
      
      uint8_t overall_status = 0x01; // OK by default


      // Home Z actuator using go_home method
      uint8_t z_payload[2] = {0};
      z_axis.go_home(z_payload);
      flushCanBuffer();
      
      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 2, z_payload) == CAN_OK) {
        uint8_t z_status = waitForCanReply(z_axis.motor_id);
        if (z_status != 0x01) {
          overall_status = (z_status == 0x03) ? 0x03 : 0x02; // TIMEOUT or FAIL
        }
      } else {
        overall_status = 0x04; // NO LOCAL NETWORK
      }
      
      // Home Y actuator using go_home method
      uint8_t y_payload[2] = {0};
      y_axis.go_home(y_payload);
      flushCanBuffer();
      
      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 2, y_payload) == CAN_OK) {
        uint8_t y_status = waitForCanReply(y_axis.motor_id);
        if (y_status != 0x01) {
          overall_status = (y_status == 0x03) ? 0x03 : 0x02; // TIMEOUT or FAIL
        }
      } else {
        overall_status = 0x04; // NO LOCAL NETWORK
      }
      
      
      
      if (overall_status == 0x01) {
        Serial.println("Both actuators homed successfully");
      } else if (overall_status == 0x03) {
        Serial.println("Timeout during homing");
      } else {
        Serial.println("Failed to home actuators");
      }
      
      byte response[] = {0x03, overall_status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Move Y actuator to absolute position
    case 0x04:
    {
      Serial.println("Case 0x04: Moving actuator Y to absolute position");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      y_axis.abs_mode(angle, payload);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(y_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterY();
      }

      // Send response with status
      byte statusResponse[] = {0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x05 ***************************** //
    // Move Z actuator to absolute position
    case 0x05:
    {
      Serial.println("Case 0x05: Moving actuator Z to absolute position");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      z_axis.abs_mode(angle, payload);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x05, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(z_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }   

      // Send response with status
      byte statusResponse[] = {0x05, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x06 ***************************** //
    // Read Z actuator movement counter
    case 0x06:
    {
      Serial.println("Case 0x06: Reading actuator movement counter Z");
      
      // Send counter value as 4-byte little-endian response
      byte response[8];
      response[0] = 0x06;  // Command ID
      response[1] = 0x01;  // Status: OK
      response[2] = actuatorMoveCounterZ & 0xFF;         // Counter byte 0 (LSB)
      response[3] = (actuatorMoveCounterZ >> 8) & 0xFF;  // Counter byte 1
      response[4] = (actuatorMoveCounterZ >> 16) & 0xFF; // Counter byte 2
      response[5] = (actuatorMoveCounterZ >> 24) & 0xFF; // Counter byte 3 (MSB)
      response[6] = 0x00;
      response[7] = 0x00;
      
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x07 ***************************** //
    // Read Y actuator movement counter
    case 0x07:
    {
      Serial.println("Case 0x07: Reading actuator movement counter Y");
      
      // Send counter value as 4-byte little-endian response
      byte response[8];
      response[0] = 0x07;  // Command ID
      response[1] = 0x01;  // Status: OK
      response[2] = actuatorMoveCounterY & 0xFF;         // Counter byte 0 (LSB)
      response[3] = (actuatorMoveCounterY >> 8) & 0xFF;  // Counter byte 1
      response[4] = (actuatorMoveCounterY >> 16) & 0xFF; // Counter byte 2
      response[5] = (actuatorMoveCounterY >> 24) & 0xFF; // Counter byte 3 (MSB)
      response[6] = 0x00;
      response[7] = 0x00;
      
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Reset Y actuator movement counter
    case 0x08:
    {
      Serial.println("Case 0x08: Resetting actuator movement counter Y");
      actuatorMoveCounterY = 0;
      saveActuatorCounterY();
      delay(500);
      byte response[] = {0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // OK
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x09 ***************************** //
    // Reset Z actuator movement counter
    case 0x09:
    {
      Serial.println("Case 0x09: Resetting actuator movement counter Z");
      actuatorMoveCounterZ = 0;
      saveActuatorCounterZ();
      delay(500);
      byte response[] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // OK
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x13 ***************************** //
    // Home Y axis using go_home function
    case 0x13:
    {
      Serial.println("Case 0x13: Homing Y axis using go_home");
      
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      y_axis.go_home(payload);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 2, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(y_axis.motor_id);
      
      // Send response with status
      byte statusResponse[] = {0x13, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //
    // Home Z axis using go_home function
    case 0x14:
    {
      Serial.println("Case 0x14: Homing Z axis using go_home");
      
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      z_axis.go_home(payload);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 2, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x14, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(z_axis.motor_id);
      
      // Send response with status
      byte statusResponse[] = {0x14, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x15 ***************************** //
    // Move Y actuator to absolute position with speed control
    case 0x15:
    {
      Serial.println("Case 0x15: Moving actuator Y to absolute position with speed control");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];
      // English: Speed is 2 bytes (high/low). Español: Velocidad en 2 bytes. 日本語: 速度は2バイト
      uint8_t speed_high = instruction.data[4];
      uint8_t speed_low  = instruction.data[5];
      uint16_t speed = (static_cast<uint16_t>(speed_high) << 8) | static_cast<uint16_t>(speed_low);

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      y_axis.abs_mode_with_speed_control(angle, speed, payload);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x15, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(y_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterY();
      }

      // Send response with status
      byte statusResponse[] = {0x15, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Move Z actuator to absolute position with speed control
    case 0x16:
    {
      Serial.println("Case 0x16: Moving actuator Z to absolute position with speed control");
      
      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];
      // English: Speed is 2 bytes (high/low). Español: Velocidad en 2 bytes. 日本語: 速度は2バイト
      uint8_t speed_high = instruction.data[4];
      uint8_t speed_low  = instruction.data[5];
      uint16_t speed = (static_cast<uint16_t>(speed_high) << 8) | static_cast<uint16_t>(speed_low);

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      z_axis.abs_mode_with_speed_control(angle, speed, payload);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x16, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(z_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }

      // Send response with status
      byte statusResponse[] = {0x16, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
      Serial.println("Case 0xFF: Powering off - Moving all to home position");
      
      uint8_t overall_status = 0x01; // OK by default
      
      // Home Y actuator using go_home method
      uint8_t y_payload[8] = {0};
      y_axis.go_home(y_payload);
      flushCanBuffer();
      
      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 2, y_payload) == CAN_OK) {
        uint8_t y_status = waitForCanReply(y_axis.motor_id);
        if (y_status != 0x01) {
          overall_status = (y_status == 0x03) ? 0x03 : 0x02; // TIMEOUT or FAIL
        }
      } else {
        overall_status = 0x04; // NO LOCAL NETWORK
      }
      
      // Home Z actuator using go_home method
      uint8_t z_payload[8] = {0};
      z_axis.go_home(z_payload);
      flushCanBuffer();
      
      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 2, z_payload) == CAN_OK) {
        uint8_t z_status = waitForCanReply(z_axis.motor_id);
        if (z_status != 0x01) {
          overall_status = (z_status == 0x03) ? 0x03 : 0x02; // TIMEOUT or FAIL
        }
      } else {
        overall_status = 0x04; // NO LOCAL NETWORK
      }
      
      if (overall_status == 0x01) {
         Serial.println("All actuators moved to home position successfully");
       } else if (overall_status == 0x03) {
         Serial.println("Timeout during power off homing");
       } else {
         Serial.println("Failed to move actuators to home position");
       }
      
      byte response[] = {0xFF, overall_status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE DEFAULT ***************************** //
    default:
      Serial.println("Unknown command.");
      byte errorResponse[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      send_twai_response(errorResponse);
      break;
  }
}

// Helper function to flush CAN buffer
void flushCanBuffer() {
  byte tempData[8];
  unsigned long tempId;
  byte tempLen;
  
  // Read and discard all pending messages
  while (CAN1.checkReceive() == CAN_MSGAVAIL) {
    CAN1.readMsgBuf(&tempId, &tempLen, tempData);
    Serial.println("Flushed old CAN message from buffer");
  }
}

// Helper function to wait for CAN reply
/*
 * waitForCanReply(uint16_t expectedId)
 * -----------------------------------
 * Waits for a reply message on the CAN1 (MCP2515) bus with the specified expected CAN ID.
 * - Returns 0x01 on success (reply received), 0x02 on timeout (10 seconds).
 * - Used after sending actuator commands to confirm action completion/status.
 * - replyData buffer is cleared before waiting.
 * - Uses a small delay to avoid busy-waiting.
 *
 * PARAMS:
 *   expectedId: The CAN ID to wait for in the reply.
 * RETURNS:
 *   0x01 if reply received, 0x03 if timeout.
 */
uint8_t waitForCanReply(uint16_t expectedId) {
  memset(replyData, 0, sizeof(replyData)); // Clear the buffer before waiting
  unsigned long startTime = millis();
  const unsigned long timeout = 10000;  // 10 second timeout
  
  while (millis() - startTime < timeout) {
    if (CAN1.checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      byte len = 0;
      CAN1.readMsgBuf(&canId, &len, replyData);
      
      if (canId == expectedId && (replyData[1] == 0x02 || replyData[1] == 0x03))
      {
        return 0x01;  // Success
      }
    }
    vTaskDelay(1);  // Small delay to prevent busy-waiting
  }
  return 0x03;  // Timeout
}

// Helper function to send response via TWAI
/*
 * send_twai_response(const byte response_data[8])
 * -----------------------------------------------
 * Sends a response frame on the TWAI (CAN0) bus with the RESPONSE_CAN_ID.
 * - Used to acknowledge commands, report status, or send data back to the master controller.
 * - The response_data buffer must be 8 bytes.
 *
 * PARAMS:
 *   response_data: 8-byte array containing the response payload.
 */
void send_twai_response(const byte response_data[8]) {
  CanFrame tx_frame;
  tx_frame.identifier = RESPONSE_CAN_ID;  // Response ID
  tx_frame.extd = 0;
  tx_frame.data_length_code = 8;
  memcpy(tx_frame.data, response_data, 8);
  ESP32Can.writeFrame(tx_frame);
}
