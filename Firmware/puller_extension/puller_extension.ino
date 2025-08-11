/*
 * puller_extension.ino
 * -----------------
 * Firmware for a single-CAN (Controller Area Network) based hose pulling robotic system using ESP32.
 *
 * FEATURES:
 *   - Single CAN bus support: ESP32 TWAI (integrated) for communication with general and local networks.
 *   - Controls a stepper motor for additional axis movement.
 *   - Implements FreeRTOS for multitasking, using queues for instruction dispatch.
 *   - Stores and manages actuator movement counters in  non-volatile EEPROM memory.
 *   - Robust command processing with status reporting and error handling via CAN.
 *   - Modular design using custom classes for actuators and gripper.
 *
 * ARCHITECTURE OVERVIEW:
 *   - setup(): Initializes peripherals, CAN buses, EEPROM, and FreeRTOS tasks.
 *   - loop(): Main event loop, processes queued CAN instructions and manages stepper.
 *   - twai_listener_task(): Listens for incoming CAN instructions and queues them for processing.
 *   - process_instruction(): Decodes and executes commands from CAN, dispatches to actuators, stepper, or gripper as needed.
 *   - Helper functions for EEPROM, CAN reply waiting, and CAN response sending.
 *
 * COMMANDS (CAN):
 *   - 0x01: Reset microcontroller
 *   - 0x02: Heartbeat
 *   - 0x0A: Read stepper movement counter
 *   - 0x0B: Move stepper to position
 *   - 0x0C: Home stepper
 *   - 0x0D: Open gripper
 *   - 0x0E: Close gripper
 *   - 0x0F: Set gripper force
 *   - 0x10: Read gripper movement counter
 *   - 0x11: Reset gripper movement counter
 *   - 0x12: Reset stepper movement counter
 *   - 0x13: Move servo to position
 *   - 0xFF: Power off, home all axes
 *
 * DEPENDENCIES:
 *   - ESP32-TWAI-CAN
 *   - ESP32Servo
 *   - AccelStepper
 *   - FreeRTOS (ESP32)
 *   - EEPROM (ESP32)
 *   - Custom: src/gripper_digital.h
 *
 * AUTHOR: Alan Silva
 * DATE: 2025-07-25
 * CONTACT : asilva@gswiring.com
 * DETAILED DOCUMENTATION:
 *   - Each function and major logic block is documented below for clarity and maintainability.
 *   - For protocol details and troubleshooting, see the project README and hardware wiring diagrams.
 */

#include <ESP32-TWAI-CAN.hpp>
#include "src/gripper_digital.h"
#include <AccelStepper.h>
#include <ESP32Servo.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 12  // 4 bytes for each counter (servos + actuator + servo)
#define STEPPER_COUNTER_ADDR 4 // Address in EEPROM to store the stepper counter
#define GRIPPER_COUNTER_ADDR 8 // Address in EEPROM to store the gripper counter
#define SERVO_COUNTER_ADDR 0 // Address in EEPROM to store the servo counter

// Global variables
unsigned long stepperMoveCounter = 0;
unsigned long gripperMoveCounter = 0;
unsigned long servoMoveCounter = 0;
byte replyData[8];  // Buffer for CAN replies

// Servo motor configuration
#define SERVO_PIN 18  // GPIO18 for servo control
Servo servoMotor;

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Stepper Motor Pins (using available GPIO pins)
#define STEPPER_STEP_PIN 26  // GPIO26 - Step pin
#define STEPPER_DIR_PIN 27   // GPIO27 - Direction pin
#define STEPPER_ENABLE_PIN 25 // GPIO25 - Enable pin

// Instance of GripperDigital
GripperDigital gripper(14, 16, 17);

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x193
#define RESPONSE_CAN_ID 0x593

// Stepper Motor Configuration
#define STEPS_PER_REVOLUTION 200  // Number of steps per revolution for your stepper motor
#define MAX_SPEED 1500000  // Maximum speed in steps per second
#define ACCELERATION 1500000  // Acceleration in steps per second squared

// Stepper motor instance
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

// Queue to store instructions received from the main CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Status :
// 1 -> OK
// 2 -> FAIL
// 3 -> TIMEOUT

// Function declarations
void process_instruction(CanFrame instruction);
void send_twai_response(const byte response_data[8]);


void saveStepperCounter()
{
  EEPROM.writeULong(STEPPER_COUNTER_ADDR, stepperMoveCounter);
  EEPROM.commit();
  Serial.print("Stepper counter saved: ");
  Serial.println(stepperMoveCounter);
}

void saveGripperCounter()
{
  EEPROM.writeULong(GRIPPER_COUNTER_ADDR, gripperMoveCounter);
  EEPROM.commit();
  Serial.print("Gripper counter saved: ");
  Serial.println(gripperMoveCounter);
}

void saveServoCounter()
{
  EEPROM.writeULong(SERVO_COUNTER_ADDR, servoMoveCounter);
  EEPROM.commit();
  Serial.print("Servo counter saved: ");
  Serial.println(servoMoveCounter);
}

void incrementStepperCounter()
{
  stepperMoveCounter++;
  saveStepperCounter();
}

void incrementGripperCounter()
{
  gripperMoveCounter++;
  saveGripperCounter();
}

void incrementServoCounter()
{
  servoMoveCounter++;
  saveServoCounter();
}

/*
 * setup()
 * -------
 * Initializes all peripherals, communication buses, memory, and FreeRTOS tasks.
 * - Sets up serial communication for debugging.
 * - Initializes stepper motor with max speed, acceleration, and enable pin.
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
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize servo motor
  servoMotor.setPeriodHertz(50); // Standard 50Hz servo
  servoMotor.attach(14, 900, 2100);
  servoMotor.writeMicroseconds(900);  // Start at neutral position (90 degrees)

  // Initialize stepper motor
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.setEnablePin(STEPPER_ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);  // Invert enable pin
  stepper.enableOutputs();
  stepper.moveTo(0);  // Set initial position

  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, LOW);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  stepperMoveCounter = EEPROM.readULong(STEPPER_COUNTER_ADDR);
  gripperMoveCounter = EEPROM.readULong(GRIPPER_COUNTER_ADDR);
  servoMoveCounter = EEPROM.readULong(SERVO_COUNTER_ADDR);
  Serial.print("Loaded stepper move counter from EEPROM: ");
  Serial.println(stepperMoveCounter);
  Serial.print("Loaded gripper move counter from EEPROM: ");
  Serial.println(gripperMoveCounter);
  Serial.print("Loaded servo move counter from EEPROM: ");
  Serial.println(servoMoveCounter);

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
 * - Calls stepper.run() to update stepper position (non-blocking).
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
  
  // No longer needed since we handle stepper movement directly in the cases
  
  // Other non-blocking logic can be added here if needed
  delay(10);
}

/*
 * process_instruction(CanFrame instruction)
 * ----------------------------------------
 * Decodes and executes CAN instructions received from the queue.
 * - Each case in the switch statement corresponds to a command (see command list above).
 * - Handles actuator moves, stepper moves, gripper control, counter reads/resets, and power-off.
 * - Sends appropriate CAN responses for each command, including error/status codes.
 * - Uses helper functions for actuator/gripper actions and CAN communication.
 *
 * PARAMS:
 *   instruction: CanFrame containing the received CAN message, including identifier and data payload.
 *
 * NOTES:
 *   - Many actions are non-blocking; responses may be sent before motion completes.
 *   - Error handling is robust: failures in CAN communication or timeouts are reported via response codes.
 *   - For actuator/stepper/gripper details, see respective class documentation in src/.
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

    // ***************************** CASE 0x0A ***************************** //
    // Read stepper movement counter
    case 0x0A:
    {
      Serial.println("Case 0x0A: Reading stepper movement counter");
      byte counter_high = (stepperMoveCounter >> 8) & 0xFF;
      byte counter_low = stepperMoveCounter & 0xFF;
      byte response[] = {0x0A, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

      // ***************************** CASE 0x0B ***************************** //
    // Move stepper motor to position
    case 0x0B:
    {
      Serial.println("Case 0x0B: Moving stepper to position");
      // Extract target position from the CAN message (4 bytes, little-endian)
      long targetPosition = ((uint8_t)instruction.data[1] << 16) |
                           ((uint8_t)instruction.data[2] << 8) |
                           ((uint8_t)instruction.data[3]);
      

      // Get direction from byte 54 (0 = negative, 1 = positive, other = use sign of position)
      int direction = instruction.data[4];
      
      // Apply direction
      if (direction == 1) {
        targetPosition = -targetPosition;
      }
      
      Serial.print("Case 0x0B: Moving stepper to position: ");
      Serial.print(targetPosition);
      Serial.print(", Direction: ");
      Serial.println((direction > 0) ? "Forward" : "Backward");
      
      // Set target position and run the movement directly
      stepper.moveTo(targetPosition);
      stepper.runToPosition();

      delay(200);

      incrementStepperCounter();
      
      // Send response after movement is complete
      byte response[] = {0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0C ***************************** //
    // Home stepper motor
    case 0x0C:
    {
      Serial.println("Case 0x0C: Homing stepper");
      // Set target position and run the movement directly
      stepper.moveTo(0);
      stepper.runToPosition();
      
      incrementStepperCounter();
      
      delay(200);
      // Send response after movement is complete
      byte response[] = {0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0D ***************************** //
    // Open gripper
    case 0x0D:
    {
      Serial.println("Case 0x0D: Opening gripper");
      
      gripper.open();

      incrementGripperCounter();
      
      delay(500);
      byte response[] = {0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

  
    // ***************************** CASE 0x0E ***************************** //
    // Close gripper
    case 0x0E:
    {
      Serial.println("Case 0x0E: Closing gripper");

      gripper.close();

      incrementGripperCounter();
      
      delay(500);
      byte response[] = {0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0F ***************************** //
    // Set gripper force
    case 0x0F:
    {
      Serial.println("Case 0x0F: Setting gripper force");
      
      gripper.setFuerza(instruction.data[1]);
      byte response[] = {0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
      
    }
    break;  

    // ***************************** CASE 0x13 ***************************** //
    // Move servo to position
    case 0x13:
    {
      Serial.println("Case 0x13: Moving servo to position");
      
        int position = instruction.data[1];
        if (position == 1)
        {
          servoMotor.writeMicroseconds(2100);
          incrementServoCounter();
          delay(1000);
          servoMotor.writeMicroseconds(900);
          byte response[] = {0x13, 0x01, position, 0x00, 0x00, 0x00, 0x00, 0x00};
          send_twai_response(response);
        }
        else
        {
          servoMotor.writeMicroseconds(900);
          byte response[] = {0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x10 ***************************** //
    // Read gripper movement counter
    case 0x10:
    {
      Serial.println("Case 0x10: Reading gripper movement counter");

      byte counter_high = (gripperMoveCounter >> 8) & 0xFF;
      byte counter_low = gripperMoveCounter & 0xFF;
      byte response[] = {0x10, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x11 ***************************** //
    // Reset gripper movement counter
    case 0x11:
    {
      Serial.println("Case 0x11: Resetting gripper movement counter");
      gripperMoveCounter = 0;
      saveGripperCounter();
      delay(500);
      byte response[] = {0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

  
    // ***************************** CASE 0x12 ***************************** //
    // Reset stepper movement counter
    case 0x12:
    {
      Serial.println("Case 0x12: Resetting stepper movement counter");
      stepperMoveCounter = 0;
      saveStepperCounter();
      delay(500);
      byte response[] = {0x12, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
      Serial.println("Case 0xFF: Powering off - Moving all to home position");
  

      // Homing Stepper
      stepper.moveTo(0);

      // Open gripper
      gripper.open();

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