/*
 * hose_puller.ino
 * -----------------
 * Firmware for a dual-CAN (Controller Area Network) based hose pulling robotic system using ESP32.
 *
 * FEATURES:
 *   - Dual CAN bus support: ESP32 TWAI (integrated) and MCP2515 (external) for communication with general and local networks.
 *   - Controls two linear actuators (Y and Z axes) and a digital gripper for hose manipulation.
 *   - Controls a stepper motor for additional axis movement.
 *   - Hose presence sensor on pin 31 (HIGH = hose present, LOW = no hose).
 *   - Implements FreeRTOS for multitasking, using queues for instruction dispatch.
 *   - Stores and manages actuator movement counters in non-volatile EEPROM memory.
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
 *   - 0x03: Home actuators (Y and Z)
 *   - 0x04: Move Y actuator to absolute position
 *   - 0x05: Move Z actuator to absolute position
 *   - 0x06: Read Z actuator movement counter
 *   - 0x07: Read Y actuator movement counter
 *   - 0x08: Reset Y movement counter
 *   - 0x09: Reset Z movement counter
 *   - 0x0A: Read stepper movement counter
 *   - 0x0B: Move stepper to position
 *   - 0x0C: Home stepper
 *   - 0x0D: Open gripper
 *   - 0x0E: Close gripper
 *   - 0x0F: Set gripper force
 *   - 0x10: Read gripper movement counter
 *   - 0x11: Reset gripper movement counter
 *   - 0x12: Reset stepper movement counter
 *   - 0x13: Home Y axis using go_home
 *   - 0x14: Home Z axis using go_home
 *   - 0x15: Move Y actuator to absolute position with speed control
 *   - 0x16: Move Y axis with speed mode until hose presence sensor detects no hose
 *   - 0x17: Move Y actuator to relative position with speed and acceleration control
 *   - 0xFF: Power off, home all axes
 *
 * DEPENDENCIES:
 *   - ESP32-TWAI-CAN
 *   - mcp_can
 *   - SPI
 *   - ESP32Servo
 *   - AccelStepper
 *   - FreeRTOS (ESP32)
 *   - EEPROM (ESP32)
 *   - Custom: src/linear_actuator.h, src/gripper_digital.h
 *
 * AUTHOR: Alan Silva
 * DATE: 2025-07-25
 * CONTACT : asilva@gswiring.com
 * DETAILED DOCUMENTATION:
 *   - Each function and major logic block is documented below for clarity and maintainability.
 *   - For protocol details and troubleshooting, see the project README and hardware wiring diagrams.
 */

#include <ESP32-TWAI-CAN.hpp>
#include <mcp_can.h>
#include <SPI.h>
#include "src/linear_actuator.h"
#include "src/gripper_digital.h"
#include <ESP32Servo.h>
#include <AccelStepper.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 20  // 4 bytes for each counter (servos + actuator)
#define ACTUATOR_COUNTER_ADDR 4  // Address in EEPROM to store the actuator counter
#define ACTUATOR_COUNTER_ADDR2 8 // Address in EEPROM to store the actuator counter
#define STEPPER_COUNTER_ADDR 12 // Address in EEPROM to store the stepper counter
#define GRIPPER_COUNTER_ADDR 16 // Address in EEPROM to store the gripper counter

// Global variables
unsigned long actuatorMoveCounterY = 0;
unsigned long actuatorMoveCounterZ = 0;
unsigned long stepperMoveCounter = 0;
unsigned long gripperMoveCounter = 0;
byte replyData[8];  // Buffer for CAN replies

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 26  // Changed from 15 to 26 to avoid upload issues
#define CAN1_INT 25  // Changed from 2 to 25 to avoid boot issues

// Stepper Motor Pins (using available GPIO pins)
#define STEPPER_STEP_PIN 22  // GPIO22 - Step pin
#define STEPPER_DIR_PIN 21   // GPIO21 - Direction pin
#define STEPPER_ENABLE_PIN 20 // GPIO20 - Enable pin

// Hose Presence Sensor Pin
#define HOSE_PRESENCE_SENSOR_PIN 32  // GPIO31 - Hose presence sensor (HIGH = hose present, LOW = no hose)

// Instance of GripperDigital
GripperDigital gripper(35, 33, 14);

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x192
#define RESPONSE_CAN_ID 0x592

// Stepper Motor Configuration
#define STEPS_PER_REVOLUTION 200  // Number of steps per revolution for your stepper motor
#define MAX_SPEED 1000  // Maximum speed in steps per second
#define ACCELERATION 500  // Acceleration in steps per second squared

// Stepper motor instance
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Instance of LinearActuator
LinearActuator y_axis(0x2CB);
LinearActuator z_axis(0x2CC);

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
  Serial.print("Actuator counter saved: ");
  Serial.println(actuatorMoveCounterY);
}

void saveActuatorCounterZ()
{
  EEPROM.writeULong(ACTUATOR_COUNTER_ADDR2, actuatorMoveCounterZ);
  EEPROM.commit();
  Serial.print("Actuator counter saved: ");
  Serial.println(actuatorMoveCounterZ);
}

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

  // Initialize stepper motor
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.setEnablePin(STEPPER_ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);  // Invert enable pin
  stepper.enableOutputs();
  stepper.moveTo(0);  // Set initial position

  // Initialize hose presence sensor
  pinMode(HOSE_PRESENCE_SENSOR_PIN, INPUT);
  Serial.println("Hose presence sensor initialized on pin 31");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  actuatorMoveCounterY = EEPROM.readULong(ACTUATOR_COUNTER_ADDR);
  actuatorMoveCounterZ = EEPROM.readULong(ACTUATOR_COUNTER_ADDR2);
  stepperMoveCounter = EEPROM.readULong(STEPPER_COUNTER_ADDR);
  gripperMoveCounter = EEPROM.readULong(GRIPPER_COUNTER_ADDR);
  Serial.print("Loaded actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterY);
  Serial.print("Loaded actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterZ);
  Serial.print("Loaded stepper move counter from EEPROM: ");
  Serial.println(stepperMoveCounter);
  Serial.print("Loaded gripper move counter from EEPROM: ");
  Serial.println(gripperMoveCounter);

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
  
  // Run stepper motor
  stepper.run();
  
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

    // ***************************** CASE 0x03 ***************************** //
    // Home actuator
    case 0x03:
    { 
      Serial.println("Case 0x03: Homing actuator");

      // Homing Z axis

      uint8_t payload2[8] = {0};  // Initialize buffer for CAN message
      z_axis.abs_mode(ACTUATOR_HOME_POSITION, payload2);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 8, payload2) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x03, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(z_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }

      // Homing Y axis

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      y_axis.abs_mode(ACTUATOR_HOME_POSITION, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x03, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      status = waitForCanReply(y_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterY();
      }
      

      // Send response with status
      byte statusResponse[] = {0x03, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Move actuator Y to absolute position
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
    // Move actuator Z to absolute position
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
        byte errorResponse[] = {0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
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
    // Read actuator movement counter Z
    case 0x06:
    {
      Serial.println("Case 0x06: Reading actuator movement counter Z");
      byte counter_high = (actuatorMoveCounterZ >> 8) & 0xFF;
      byte counter_low = actuatorMoveCounterZ & 0xFF;
      byte response[] = {0x06, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;
          
    // ***************************** CASE 0x07 ***************************** //
    // Read actuator movement counter Y
    case 0x07:
    {
      Serial.println("Case 0x07: Reading actuator movement counter Y");
      byte counter_high = (actuatorMoveCounterY >> 8) & 0xFF;
      byte counter_low = actuatorMoveCounterY & 0xFF;
      byte response[] = {0x07, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Reset actuator movement counter Y
    case 0x08:
    {
      Serial.println("Case 0x08: Resetting actuator movement counter Y");
      actuatorMoveCounterY = 0;
      saveActuatorCounterY();
      byte response[] = {0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x09 ***************************** //
    // Reset actuator movement counter Z
    case 0x09:
    {
      Serial.println("Case 0x09: Resetting actuator movement counter Z");
      actuatorMoveCounterZ = 0;
      saveActuatorCounterZ();
      byte response[] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
      long targetPosition = ((uint8_t)instruction.data[4] << 24) |
                           ((uint8_t)instruction.data[3] << 16) |
                           ((uint8_t)instruction.data[2] << 8) |
                           ((uint8_t)instruction.data[1]);
      

      // Get direction from byte 5 (0 = negative, 1 = positive, other = use sign of position)
      int direction = instruction.data[5];
      
      // Apply direction
      if (direction == 1) {
        targetPosition = -targetPosition;
      }
      
      Serial.print("Case 0x0B: Moving stepper to position: ");
      Serial.print(targetPosition);
      Serial.print(", Direction: ");
      Serial.println((direction > 0) ? "Forward" : "Backward");
      
      // Set target position (non-blocking)
      stepper.moveTo(targetPosition);

      incrementStepperCounter();
      
      delay(500);
      // Send immediate response (movement happens asynchronously in loop())
      byte response[] = {0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0C ***************************** //
    // Home stepper motor
    case 0x0C:
    {
      Serial.println("Case 0x0C: Homing stepper");
      // Set target position (non-blocking)
      stepper.moveTo(0);

      incrementStepperCounter();
      
      delay(500);
      // Send immediate response (movement happens asynchronously in loop())
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
      
      // Send immediate response
      byte response[] = {0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
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

    // ***************************** CASE 0x13 ***************************** //
    // Home Y axis using go_home function
    case 0x13:
    {
      Serial.println("Case 0x13: Homing Y axis using go_home");
      
      uint8_t payload[2] = {0};  // Initialize buffer for CAN message (2 bytes)
      y_axis.go_home(payload);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 2, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(y_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterY();
      }
      
      // Send response with status
      byte statusResponse[] = {0x13, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //

    case 0x15:
    {
      Serial.println("Case 0x15: Moving actuator Y to absolute position with speed control");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];
      uint8_t speed_high = instruction.data[4];
      uint8_t speed_low = instruction.data[5];

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint16_t local_speed = (speed_high << 8) | speed_low;

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      y_axis.abs_mode_with_speed_control(angle, local_speed, payload);  // Generate the CAN message

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

    // ***************************** CASE 0x17 ***************************** //
    // Move actuator Y to relative position with speed and acceleration control
    case 0x17:
    {
      Serial.println("Case 0x17: Moving actuator Y to relative position with speed control");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3]; // 0 = positivo, 1 = negativo
      uint8_t speed_high = instruction.data[4];
      uint8_t speed_low = instruction.data[5];
      uint8_t acceleration = instruction.data[6]; // 0-255

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint16_t local_speed = (speed_high << 8) | speed_low;

      // Debug de parámetros recibidos
      Serial.print("Ángulo relativo (grados): ");
      Serial.println(angle);
      Serial.print("Velocidad (RPM): ");
      Serial.println(local_speed);
      Serial.print("Aceleración: ");
      Serial.println(acceleration);

      uint8_t payload[8] = {0};  // Buffer para mensaje CAN
      y_axis.relative_move_with_speed_control(angle, local_speed, acceleration, payload);  // Generar mensaje CAN F4

      flushCanBuffer();

      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x17, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Esperar respuesta del servo y obtener estado
      uint8_t status = waitForCanReply(y_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterY();
      }

      // Respuesta con estado
      byte statusResponse[] = {0x17, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //
    // Home Z axis using go_home function
    case 0x14:
    {
      Serial.println("Case 0x14: Homing Z axis using go_home");
      
      uint8_t payload[2] = {0};  // Initialize buffer for CAN message (2 bytes)
      z_axis.go_home(payload);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 2, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x14, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(z_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }
      
      // Send response with status
      byte statusResponse[] = {0x14, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Move Y axis with speed mode until hose presence sensor detects no hose
    case 0x16:
    {
      Serial.println("Case 0x16: Moving Y axis with speed mode until no hose detected");

      uint8_t speed_high = instruction.data[1];
      uint8_t speed_low = instruction.data[2];
      uint8_t direction = instruction.data[3]; // 0 = positive, 1 = negative
      uint8_t acceleration = instruction.data[4]; // Acceleration parameter

      uint16_t local_speed = (speed_high << 8) | speed_low;
      bool dir = (direction == 1); // Convert to boolean for speed_mode
      
      // Debug: Print received speed value
      Serial.print("Velocidad recibida desde interfaz: ");
      Serial.println(local_speed);
      Serial.print("Direccion: ");
      Serial.println(direction);
      Serial.print("Aceleracion: ");
      Serial.println(acceleration);
      
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      y_axis.speed_mode(dir, local_speed, acceleration, payload);  // Use speed mode directly

      flushCanBuffer();

      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 5, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x16, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Monitor hose presence sensor while moving
      bool hose_detected = true;
      unsigned long start_time = millis();
      const unsigned long timeout = 30000; // 30 second timeout

      while (hose_detected && (millis() - start_time) < timeout)
      {
        // Read sensor (LOW = no hose, HIGH = hose present)
        hose_detected = digitalRead(HOSE_PRESENCE_SENSOR_PIN);
        
        if (!hose_detected) {
          Serial.println("No hose detected - stopping Y axis movement");
          
          // Stop the actuator by sending speed mode with 0 speed
          uint8_t stop_payload[8] = {0};
          y_axis.speed_mode(false, 0, acceleration, stop_payload);
          
          if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 5, stop_payload) == CAN_OK) {
            Serial.println("Y axis stopped successfully");
          }
          break;
        }
        
        delay(50); // Check sensor every 50ms
      }

      if (millis() - start_time >= timeout) {
        Serial.println("Timeout reached - stopping Y axis movement");
        // Send stop command on timeout
        uint8_t stop_payload[8] = {0};
        y_axis.speed_mode(false, 0, acceleration, stop_payload);
        CAN1.sendMsgBuf(y_axis.motor_id, 0, 5, stop_payload);
      }

      // Wait for final reply and get status
      uint8_t status = waitForCanReply(y_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterY();
      }

      // Send response with status and sensor state
      byte statusResponse[] = {0x16, status, (byte)(!hose_detected), 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
      Serial.println("Case 0xFF: Powering off - Moving all to home position");
      
      // Move linear actuator to home position (assuming position 0 is home)

      // Homing Z

      uint8_t payload2[8] = {0};  // Initialize buffer for CAN message
      z_axis.abs_mode(0, payload2);  // Generate the CAN message

      flushCanBuffer();

      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 8, payload2) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(z_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }
      else 
      {
        byte errorResponse[] = {0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }


      // Homing Y
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      y_axis.abs_mode(0, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(y_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      status = waitForCanReply(y_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterY();
      }
      else 
      {
        byte errorResponse[] = {0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      } 

      // Homing Stepper
      stepper.moveTo(0);

      // Open gripper
      gripper.open();


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

// Helper function to wait for CAN reply
/*
 * waitForCanReply(uint16_t expectedId)
 * -----------------------------------
 * Waits for a reply message on the CAN1 (MCP2515) bus with the specified expected CAN ID.
 * - Returns 0x01 on success (reply received), 0x02 on timeout (1 second).
 * - Used after sending actuator commands to confirm action completion/status.
 * - replyData buffer is cleared before waiting.
 * - Uses a small delay to avoid busy-waiting.
 *
 * PARAMS:
 *   expectedId: The CAN ID to wait for in the reply.
 * RETURNS:
 *   0x01 if reply received, 0x02 if timeout.
 */
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


uint8_t waitForCanReply(uint16_t expectedId) {
  memset(replyData, 0, sizeof(replyData)); // Clear the buffer before waiting
  unsigned long startTime = millis();
  const unsigned long timeout = 10000;  // 1 second timeout
  
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