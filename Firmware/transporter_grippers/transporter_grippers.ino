/*
 * transporter_grippers.ino
 * -----------------------
 * Firmware for a single-CAN (Controller Area Network) based triple gripper robotic system using ESP32.
 *
 * FEATURES:
 *   - Single CAN bus support: ESP32 TWAI (integrated) for communication with general and local networks.
 *   - Controls 3 digital grippers that operate in parallel.
 *   - Implements FreeRTOS for multitasking, using queues for instruction dispatch.
 *   - Stores and manages gripper movement counters in non-volatile EEPROM memory.
 *   - Robust command processing with status reporting and error handling via CAN.
 *   - Modular design using custom classes for grippers.
 *
 * ARCHITECTURE OVERVIEW:
 *   - setup(): Initializes peripherals, CAN buses, EEPROM, and FreeRTOS tasks.
 *   - loop(): Main event loop, processes queued CAN instructions and manages grippers.
 *   - twai_listener_task(): Listens for incoming CAN instructions and queues them for processing.
 *   - process_instruction(): Decodes and executes commands from CAN, dispatches to grippers as needed.
 *   - Helper functions for EEPROM, CAN reply waiting, and CAN response sending.
 *
 * COMMANDS (CAN):
 *   - 0x01: Reset microcontroller
 *   - 0x02: Heartbeat
 *   - 0x0D: Open all grippers
 *   - 0x0E: Close all grippers
 *   - 0x0F: Set all grippers force
 *   - 0x10: Read gripper 1 movement counter
 *   - 0x11: Read gripper 2 movement counter
 *   - 0x12: Read gripper 3 movement counter
 *   - 0x13: Reset gripper 1 movement counter
 *   - 0x14: Reset gripper 2 movement counter
 *   - 0x15: Reset gripper 3 movement counter
 *   - 0x16: Open gripper 1
 *   - 0x17: Close gripper 1
 *   - 0x18: Open gripper 2
 *   - 0x19: Close gripper 2
 *   - 0x1A: Open gripper 3
 *   - 0x1B: Close gripper 3
 *   - 0xFF: Power off, open all grippers
 *
 * DEPENDENCIES:
 *   - ESP32-TWAI-CAN
 *   - FreeRTOS (ESP32)
 *   - EEPROM (ESP32)
 *   - Custom: src/gripper_digital.h
 *
 * AUTHOR: Alan Silva
 * DATE: 2025-01-15
 * CONTACT: asilva@gswiring.com
 * DETAILED DOCUMENTATION:
 *   - Each function and major logic block is documented below for clarity and maintainability.
 *   - For protocol details and troubleshooting, see the project README and hardware wiring diagrams.
 */

#include <ESP32-TWAI-CAN.hpp>
#include "src/gripper_digital.h"

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 12  // 4 bytes for each gripper counter
#define GRIPPER1_COUNTER_ADDR 0 // Address in EEPROM to store gripper 1 counter
#define GRIPPER2_COUNTER_ADDR 4 // Address in EEPROM to store gripper 2 counter
#define GRIPPER3_COUNTER_ADDR 8 // Address in EEPROM to store gripper 3 counter

// Global variables
unsigned long gripper1MoveCounter = 0;
unsigned long gripper2MoveCounter = 0;
unsigned long gripper3MoveCounter = 0;
byte replyData[8];  // Buffer for CAN replies

// Pins for the CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Gripper Pin Definitions
// Gripper 1 pins (all inputs)
#define GRIPPER1_IN0_PIN 14    // GPIO14 - Input 0 for gripper 1
#define GRIPPER1_IN1_PIN 15    // GPIO15 - Input 1 for gripper 1
#define GRIPPER1_IN2_PIN 16    // GPIO16 - Input 2 for gripper 1

// Gripper 2 pins (all inputs)
#define GRIPPER2_IN0_PIN 17    // GPIO17 - Input 0 for gripper 2
#define GRIPPER2_IN1_PIN 18    // GPIO18 - Input 1 for gripper 2
#define GRIPPER2_IN2_PIN 19    // GPIO19 - Input 2 for gripper 2

// Gripper 3 pins (all inputs)
#define GRIPPER3_IN0_PIN 21    // GPIO21 - Input 0 for gripper 3
#define GRIPPER3_IN1_PIN 22    // GPIO22 - Input 1 for gripper 3
#define GRIPPER3_IN2_PIN 23    // GPIO23 - Input 2 for gripper 3

// Instances of GripperDigital
GripperDigital gripper1(GRIPPER1_IN0_PIN, GRIPPER1_IN1_PIN, GRIPPER1_IN2_PIN);
GripperDigital gripper2(GRIPPER2_IN0_PIN, GRIPPER2_IN1_PIN, GRIPPER2_IN2_PIN);
GripperDigital gripper3(GRIPPER3_IN0_PIN, GRIPPER3_IN1_PIN, GRIPPER3_IN2_PIN);

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x020
#define RESPONSE_CAN_ID 0x420

// Queue to store instructions received from the main CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Status :
// 1 -> OK
// 2 -> FAIL
// 3 -> TIMEOUT

// Function declarations
void process_instruction(CanFrame instruction);
void send_twai_response(const byte response_data[8]);

// EEPROM helper functions
void saveGripper1Counter()
{
  EEPROM.writeULong(GRIPPER1_COUNTER_ADDR, gripper1MoveCounter);
  EEPROM.commit();
  Serial.print("Gripper 1 counter saved: ");
  Serial.println(gripper1MoveCounter);
}

void saveGripper2Counter()
{
  EEPROM.writeULong(GRIPPER2_COUNTER_ADDR, gripper2MoveCounter);
  EEPROM.commit();
  Serial.print("Gripper 2 counter saved: ");
  Serial.println(gripper2MoveCounter);
}

void saveGripper3Counter()
{
  EEPROM.writeULong(GRIPPER3_COUNTER_ADDR, gripper3MoveCounter);
  EEPROM.commit();
  Serial.print("Gripper 3 counter saved: ");
  Serial.println(gripper3MoveCounter);
}

void incrementGripper1Counter()
{
  gripper1MoveCounter++;
  saveGripper1Counter();
}

void incrementGripper2Counter()
{
  gripper2MoveCounter++;
  saveGripper2Counter();
}

void incrementGripper3Counter()
{
  gripper3MoveCounter++;
  saveGripper3Counter();
}

/*
 * setup()
 * -------
 * Initializes all peripherals, communication buses, memory, and FreeRTOS tasks.
 * - Sets up serial communication for debugging.
 * - Initializes all three grippers with their respective pins.
 * - Loads gripper movement counters from EEPROM (persistent storage).
 * - Creates a FreeRTOS queue for CAN instruction handling.
 * - Initializes CAN bus (TWAI for general network).
 * - Creates a dedicated FreeRTOS task for listening to the TWAI bus.
 * - Sends a startup message over CAN to indicate readiness.
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing triple gripper CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  gripper1MoveCounter = EEPROM.readULong(GRIPPER1_COUNTER_ADDR);
  gripper2MoveCounter = EEPROM.readULong(GRIPPER2_COUNTER_ADDR);
  gripper3MoveCounter = EEPROM.readULong(GRIPPER3_COUNTER_ADDR);
  Serial.print("Loaded gripper 1 move counter from EEPROM: ");
  Serial.println(gripper1MoveCounter);
  Serial.print("Loaded gripper 2 move counter from EEPROM: ");
  Serial.println(gripper2MoveCounter);
  Serial.print("Loaded gripper 3 move counter from EEPROM: ");
  Serial.println(gripper3MoveCounter);

  // Create the instruction queue
  instruction_queue = xQueueCreate(10, sizeof(CanFrame));
  if (instruction_queue == NULL) {
    Serial.println("Error creating instruction queue");
    while(1);
  }

  // Initialize the CAN bus (TWAI - General Network)
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
 * - Handles gripper control, counter reads/resets, and power-off.
 * - Sends appropriate CAN responses for each command, including error/status codes.
 * - Uses helper functions for gripper actions and CAN communication.
 *
 * PARAMS:
 *   instruction: CanFrame containing the received CAN message, including identifier and data payload.
 *
 * NOTES:
 *   - Many actions are non-blocking; responses may be sent before motion completes.
 *   - Error handling is robust: failures in CAN communication or timeouts are reported via response codes.
 *   - For gripper details, see respective class documentation in src/.
 */
void process_instruction(CanFrame instruction)
{
  Serial.print("Processing command: 0x");
  Serial.println(instruction.data[0], HEX);

  byte response_data[8] = {0};
  bool success = false;

  switch (instruction.data[0])
  {
    // ***************************** CASE 0x01 ***************************** //
    // Reset microcontroller
    case 0x01: 
    {
      Serial.println("Case 0x01: Reset microcontroller");
      byte response[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
      delay(100);
      ESP.restart();
    }
    break;

    // ***************************** CASE 0x02 ***************************** //
    // Send heartbeat
    case 0x02:
    {
      Serial.println("Case 0x02: Send Heartbeat");
      byte response[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0D ***************************** //
    // Open all grippers
    case 0x0D:
    {
      Serial.println("Case 0x0D: Opening all grippers");
      
      gripper1.open();
      gripper2.open();
      gripper3.open();

      incrementGripper1Counter();
      incrementGripper2Counter();
      incrementGripper3Counter();
      
      delay(500);
      byte response[] = {0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0E ***************************** //
    // Close all grippers
    case 0x0E:
    {
      Serial.println("Case 0x0E: Closing all grippers");

      gripper1.close();
      gripper2.close();
      gripper3.close();

      incrementGripper1Counter();
      incrementGripper2Counter();
      incrementGripper3Counter();
      
      delay(500);
      byte response[] = {0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0F ***************************** //
    // Set all grippers force
    case 0x0F:
    {
      Serial.println("Case 0x0F: Setting all grippers force");
      
      gripper1.setFuerza(instruction.data[1]);
      gripper2.setFuerza(instruction.data[1]);
      gripper3.setFuerza(instruction.data[1]);
      
      byte response[] = {0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x10 ***************************** //
    // Read gripper 1 movement counter
    case 0x10:
    {
      Serial.println("Case 0x10: Reading gripper 1 movement counter");
      byte counter_high = (gripper1MoveCounter >> 8) & 0xFF;
      byte counter_low = gripper1MoveCounter & 0xFF;
      byte response[] = {0x10, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x11 ***************************** //
    // Read gripper 2 movement counter
    case 0x11:
    {
      Serial.println("Case 0x11: Reading gripper 2 movement counter");
      byte counter_high = (gripper2MoveCounter >> 8) & 0xFF;
      byte counter_low = gripper2MoveCounter & 0xFF;
      byte response[] = {0x11, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x12 ***************************** //
    // Read gripper 3 movement counter
    case 0x12:
    {
      Serial.println("Case 0x12: Reading gripper 3 movement counter");
      byte counter_high = (gripper3MoveCounter >> 8) & 0xFF;
      byte counter_low = gripper3MoveCounter & 0xFF;
      byte response[] = {0x12, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x13 ***************************** //
    // Reset gripper 1 movement counter
    case 0x13:
    {
      Serial.println("Case 0x13: Resetting gripper 1 movement counter");
      gripper1MoveCounter = 0;
      saveGripper1Counter();
      delay(500);
      byte response[] = {0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //
    // Reset gripper 2 movement counter
    case 0x14:
    {
      Serial.println("Case 0x14: Resetting gripper 2 movement counter");
      gripper2MoveCounter = 0;
      saveGripper2Counter();
      delay(500);
      byte response[] = {0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x15 ***************************** //
    // Reset gripper 3 movement counter
    case 0x15:
    {
      Serial.println("Case 0x15: Resetting gripper 3 movement counter");
      gripper3MoveCounter = 0;
      saveGripper3Counter();
      delay(500);
      byte response[] = {0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Open gripper 1
    case 0x16:
    {
      Serial.println("Case 0x16: Opening gripper 1");
      
      gripper1.open();
      incrementGripper1Counter();
      
      delay(500);
      byte response[] = {0x16, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x17 ***************************** //
    // Close gripper 1
    case 0x17:
    {
      Serial.println("Case 0x17: Closing gripper 1");

      gripper1.close();
      incrementGripper1Counter();
      
      delay(500);
      byte response[] = {0x17, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x18 ***************************** //
    // Open gripper 2
    case 0x18:
    {
      Serial.println("Case 0x18: Opening gripper 2");
      
      gripper2.open();
      incrementGripper2Counter();
      
      delay(500);
      byte response[] = {0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x19 ***************************** //
    // Close gripper 2
    case 0x19:
    {
      Serial.println("Case 0x19: Closing gripper 2");

      gripper2.close();
      incrementGripper2Counter();
      
      delay(500);
      byte response[] = {0x19, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x1A ***************************** //
    // Open gripper 3
    case 0x1A:
    {
      Serial.println("Case 0x1A: Opening gripper 3");
      
      gripper3.open();
      incrementGripper3Counter();
      
      delay(500);
      byte response[] = {0x1A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x1B ***************************** //
    // Close gripper 3
    case 0x1B:
    {
      Serial.println("Case 0x1B: Closing gripper 3");

      gripper3.close();
      incrementGripper3Counter();
      
      delay(500);
      byte response[] = {0x1B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Open all grippers
    case 0xFF:
    {
      Serial.println("Case 0xFF: Powering off - Opening all grippers");

      // Open all grippers
      gripper1.open();
      gripper2.open();
      gripper3.open();

      delay(500);
      // Send response with status
      byte statusResponse[] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
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