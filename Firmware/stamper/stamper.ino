/*
 * stamper.ino
 * -----------------
 * Firmware for a dual-CAN (Controller Area Network) based stamping robotic system using ESP32.
 *
 * FEATURES:
 *   - Dual CAN bus support: ESP32 TWAI (integrated) and MCP2515 (external) for communication with general and local networks.
 *   - Controls two linear actuators (Y and Z axes) for stamping operations.
 *   - Controls three pump linear actuators (pump1, pump2, pump3) with speed mode and timer functionality.
 *   - Controls two servo motors (servo_cover, servo_stamper) with angle positioning and movement tracking.
 *   - Controls two optical sensors (optical_sensor_1, optical_sensor_2) for position detection and activation counting.
 *   - Controls Y-axis and Z-axis stepper motors for additional positioning.
 *   - Stepper movement with optical sensor feedback for precise positioning until sensor activation.
 *   - Implements FreeRTOS for multitasking, using queues for instruction dispatch.
 *   - Stores and manages actuator movement counters in non-volatile EEPROM memory.
 *   - Robust command processing with status reporting and error handling via CAN.
 *   - Modular design using custom classes for actuators.
 *
 * ARCHITECTURE OVERVIEW:
 *   - setup(): Initializes peripherals, CAN buses, EEPROM, and FreeRTOS tasks.
 *   - loop(): Main event loop, processes queued CAN instructions and manages Y-axis and Z-axis steppers.
 *   - twai_listener_task(): Listens for incoming CAN instructions and queues them for processing.
 *   - process_instruction(): Decodes and executes commands from CAN, dispatches to actuators, Y-axis stepper, Z-axis stepper, pumps, servos, or optical sensors as needed.
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
 *   - 0x0A: Read Y-axis stepper movement counter
 *   - 0x0B: Move Y-axis stepper to position
 *   - 0x0C: Home Y-axis stepper
 *   - 0x12: Reset Y-axis stepper movement counter
 *   - 0x13: Home Y axis using go_home function
 *   - 0x14: Home Z axis using go_home function
 *   - 0x15: Move Y actuator to absolute position with speed control
 *   - 0x16: Read Z-axis stepper movement counter
 *   - 0x17: Move Z-axis stepper to position
 *   - 0x18: Home Z-axis stepper
 *   - 0x19: Reset Z-axis stepper movement counter
 *   - 0x1A: Control pump speed mode with timer (pump selection, direction, speed, time, acceleration)
 *   - 0x1B: Read pump movement counter (pump selection)
 *   - 0x1C: Reset pump movement counter (pump selection)
 *   - 0x1D: Control servo motors with angle positioning (servo selection, angle)
 *   - 0x1E: Read servo movement counter (servo selection)
 *   - 0x1F: Reset servo movement counter (servo selection)
 *   - 0x20: Move stepper until optical sensor activated (stepper selection, sensor selection, direction, max steps)
 *   - 0x21: Read optical sensor counter (sensor selection)
 *   - 0x22: Reset optical sensor counter (sensor selection)
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
 *   - Custom: src/linear_actuator.h
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
#include <ESP32Servo.h>
#include <AccelStepper.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 48  // 4 bytes for each counter (servos + actuator + steppers + pumps + optical sensors)
#define ACTUATOR_COUNTER_ADDR 4  // Address in EEPROM to store the actuator counter
#define ACTUATOR_COUNTER_ADDR2 8 // Address in EEPROM to store the actuator counter
#define STEPPER_COUNTER_ADDR 12 // Address in EEPROM to store the Y-axis stepper counter
#define Z_STEPPER_COUNTER_ADDR 16 // Address in EEPROM to store the Z-axis stepper counter
#define PUMP1_COUNTER_ADDR 20 // Address in EEPROM to store the pump1 counter
#define PUMP2_COUNTER_ADDR 24 // Address in EEPROM to store the pump2 counter
#define PUMP3_COUNTER_ADDR 28 // Address in EEPROM to store the pump3 counter
#define SERVO_COVER_COUNTER_ADDR 32 // Address in EEPROM to store the servo cover counter
#define SERVO_STAMPER_COUNTER_ADDR 36 // Address in EEPROM to store the servo stamper counter
#define OPTICAL_SENSOR_1_COUNTER_ADDR 40 // Address in EEPROM to store the optical sensor 1 counter
#define OPTICAL_SENSOR_2_COUNTER_ADDR 44 // Address in EEPROM to store the optical sensor 2 counter

// Global variables
unsigned long actuatorMoveCounterY = 0;
unsigned long actuatorMoveCounterZ = 0;
unsigned long stepperMoveCounter = 0;  // Y-axis stepper counter
unsigned long zStepperMoveCounter = 0; // Z-axis stepper counter
unsigned long pump1MoveCounter = 0;    // Pump1 counter
unsigned long pump2MoveCounter = 0;    // Pump2 counter
unsigned long pump3MoveCounter = 0;    // Pump3 counter
unsigned long servoCoverMoveCounter = 0;    // Servo cover counter
unsigned long servoStamperMoveCounter = 0;  // Servo stamper counter
unsigned long opticalSensor1Counter = 0;    // Optical sensor 1 activation counter
unsigned long opticalSensor2Counter = 0;    // Optical sensor 2 activation counter
byte replyData[8];  // Buffer for CAN replies

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 26   // GPIO26 - CS pin for MCP2515
#define CAN1_INT 25  // GPIO25 - INT pin for MCP2515

// Y-axis Stepper Motor Pins
#define Y_AXIS_STEPPER_STEP_PIN 27  // GPIO27 - Step pin for Y-axis stepper (Swapped with Z)
#define Y_AXIS_STEPPER_DIR_PIN 32   // GPIO32 - Direction pin for Y-axis stepper (Swapped with Z)
#define Y_AXIS_STEPPER_ENABLE_PIN 22 // GPIO22 - Enable pin for Y-axis stepper (Swapped with Z)

// Z-axis Stepper Motor Pins
#define Z_AXIS_STEPPER_STEP_PIN 16  // GPIO16 - Step pin for Z-axis stepper (Swapped with Y)
#define Z_AXIS_STEPPER_DIR_PIN 17   // GPIO17 - Direction pin for Z-axis stepper (Swapped with Y)
#define Z_AXIS_STEPPER_ENABLE_PIN 21 // GPIO21 - Enable pin for Z-axis stepper (Swapped with Y)

// Servo Motor Pins
#define SERVO_COVER_PIN 15    // GPIO15 - PWM pin for servo cover
#define SERVO_STAMPER_PIN 14  // GPIO14 - PWM pin for servo stamper

// Optical Sensor Pins
#define OPTICAL_SENSOR_1_PIN 33  // GPIO33 - Digital input for optical sensor 1 (Y-axis) (Swapped with Sensor 2)
#define OPTICAL_SENSOR_2_PIN 13  // GPIO13 - Digital input for optical sensor 2 (Z-axis) (Swapped with Sensor 1)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x005
#define RESPONSE_CAN_ID 0x405

// Stepper Motor Configuration
#define STEPS_PER_REVOLUTION 200  // Number of steps per revolution for your stepper motor
#define MAX_SPEED 1500000  // Maximum speed in steps per second
#define ACCELERATION 1500000  // Acceleration in steps per second squared

// Y-axis stepper motor instance
AccelStepper y_axis_stepper(AccelStepper::DRIVER, Y_AXIS_STEPPER_STEP_PIN, Y_AXIS_STEPPER_DIR_PIN);

// Z-axis stepper motor instance
AccelStepper z_axis_stepper(AccelStepper::DRIVER, Z_AXIS_STEPPER_STEP_PIN, Z_AXIS_STEPPER_DIR_PIN);

// Servo motor instances
Servo servo_cover;
Servo servo_stamper;


// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Instance of LinearActuator
LinearActuator y_axis(0x001);
LinearActuator z_axis(0x002);

// Pump LinearActuator instances
LinearActuator pump1(0x003);
LinearActuator pump2(0x004);
LinearActuator pump3(0x005);

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
  Serial.print("Y-axis stepper counter saved: ");
  Serial.println(stepperMoveCounter);
}

void saveZStepperCounter()
{
  EEPROM.writeULong(Z_STEPPER_COUNTER_ADDR, zStepperMoveCounter);
  EEPROM.commit();
  Serial.print("Z-axis stepper counter saved: ");
  Serial.println(zStepperMoveCounter);
}

void savePump1Counter()
{
  EEPROM.writeULong(PUMP1_COUNTER_ADDR, pump1MoveCounter);
  EEPROM.commit();
  Serial.print("Pump1 counter saved: ");
  Serial.println(pump1MoveCounter);
}

void savePump2Counter()
{
  EEPROM.writeULong(PUMP2_COUNTER_ADDR, pump2MoveCounter);
  EEPROM.commit();
  Serial.print("Pump2 counter saved: ");
  Serial.println(pump2MoveCounter);
}

void savePump3Counter()
{
  EEPROM.writeULong(PUMP3_COUNTER_ADDR, pump3MoveCounter);
  EEPROM.commit();
  Serial.print("Pump3 counter saved: ");
  Serial.println(pump3MoveCounter);
}

// Function to save servo cover counter to EEPROM
void saveServoCoverCounter()
{
  EEPROM.writeULong(SERVO_COVER_COUNTER_ADDR, servoCoverMoveCounter);
  EEPROM.commit();
  Serial.print("Servo cover counter saved: ");
  Serial.println(servoCoverMoveCounter);
}

// Function to save servo stamper counter to EEPROM
void saveServoStamperCounter()
{
  EEPROM.writeULong(SERVO_STAMPER_COUNTER_ADDR, servoStamperMoveCounter);
  EEPROM.commit();
  Serial.print("Servo stamper counter saved: ");
  Serial.println(servoStamperMoveCounter);
}

// Function to save optical sensor 1 counter to EEPROM
void saveOpticalSensor1Counter()
{
  EEPROM.writeULong(OPTICAL_SENSOR_1_COUNTER_ADDR, opticalSensor1Counter);
  EEPROM.commit();
  Serial.print("Optical sensor 1 counter saved: ");
  Serial.println(opticalSensor1Counter);
}

// Function to save optical sensor 2 counter to EEPROM
void saveOpticalSensor2Counter()
{
  EEPROM.writeULong(OPTICAL_SENSOR_2_COUNTER_ADDR, opticalSensor2Counter);
  EEPROM.commit();
  Serial.print("Optical sensor 2 counter saved: ");
  Serial.println(opticalSensor2Counter);
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

void incrementZStepperCounter()
{
  zStepperMoveCounter++;
  saveZStepperCounter();
}

void incrementPump1Counter()
{
  pump1MoveCounter++;
  savePump1Counter();
}

void incrementPump2Counter()
{
  pump2MoveCounter++;
  savePump2Counter();
}

void incrementPump3Counter()
{
  pump3MoveCounter++;
  savePump3Counter();
}

void incrementServoCoverCounter()
{
  servoCoverMoveCounter++;
  saveServoCoverCounter();
}

void incrementServoStamperCounter()
{
  servoStamperMoveCounter++;
  saveServoStamperCounter();
}

void incrementOpticalSensor1Counter()
{
  opticalSensor1Counter++;
  saveOpticalSensor1Counter();
}

void incrementOpticalSensor2Counter()
{
  opticalSensor2Counter++;
  saveOpticalSensor2Counter();
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
  Serial.println("Initializing dual CAN stamper system with FreeRTOS");

  // Initialize Y-axis stepper motor
  y_axis_stepper.setMaxSpeed(MAX_SPEED);
  y_axis_stepper.setAcceleration(ACCELERATION);
  y_axis_stepper.setEnablePin(Y_AXIS_STEPPER_ENABLE_PIN);
  y_axis_stepper.enableOutputs();
  y_axis_stepper.moveTo(0);  // Set initial position

  // Initialize Z-axis stepper motor
  z_axis_stepper.setMaxSpeed(MAX_SPEED);
  z_axis_stepper.setAcceleration(ACCELERATION);
  z_axis_stepper.setEnablePin(Z_AXIS_STEPPER_ENABLE_PIN);
  z_axis_stepper.enableOutputs();
  z_axis_stepper.moveTo(0);  // Set initial position

  // Set enable pins as outputs and enable steppers
  pinMode(Y_AXIS_STEPPER_ENABLE_PIN, OUTPUT);
  pinMode(Z_AXIS_STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(Y_AXIS_STEPPER_ENABLE_PIN, LOW);
  digitalWrite(Z_AXIS_STEPPER_ENABLE_PIN, LOW);

  // Initialize servo motors
  servo_cover.attach(SERVO_COVER_PIN, 500, 2500);
  servo_stamper.attach(SERVO_STAMPER_PIN, 500, 2500);
  
  // Set initial positions
  servo_cover.write(105);   // servoCover_home from main.cpp
  servo_stamper.write(102); // servoStamp_home from main.cpp
  
  Serial.println("Servo motors initialized");

  // Initialize optical sensors
  pinMode(OPTICAL_SENSOR_1_PIN, INPUT_PULLUP);
  pinMode(OPTICAL_SENSOR_2_PIN, INPUT_PULLUP);
  Serial.println("Optical sensors initialized");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  actuatorMoveCounterY = EEPROM.readULong(ACTUATOR_COUNTER_ADDR);
  actuatorMoveCounterZ = EEPROM.readULong(ACTUATOR_COUNTER_ADDR2);
  stepperMoveCounter = EEPROM.readULong(STEPPER_COUNTER_ADDR);
  zStepperMoveCounter = EEPROM.readULong(Z_STEPPER_COUNTER_ADDR);
  pump1MoveCounter = EEPROM.readULong(PUMP1_COUNTER_ADDR);
  pump2MoveCounter = EEPROM.readULong(PUMP2_COUNTER_ADDR);
  pump3MoveCounter = EEPROM.readULong(PUMP3_COUNTER_ADDR);
  servoCoverMoveCounter = EEPROM.readULong(SERVO_COVER_COUNTER_ADDR);
  servoStamperMoveCounter = EEPROM.readULong(SERVO_STAMPER_COUNTER_ADDR);
  opticalSensor1Counter = EEPROM.readULong(OPTICAL_SENSOR_1_COUNTER_ADDR);
  opticalSensor2Counter = EEPROM.readULong(OPTICAL_SENSOR_2_COUNTER_ADDR);
  Serial.print("Loaded actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterY);
  Serial.print("Loaded actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterZ);
  Serial.print("Loaded Y-axis stepper move counter from EEPROM: ");
  Serial.println(stepperMoveCounter);
  Serial.print("Loaded Z-axis stepper move counter from EEPROM: ");
  Serial.println(zStepperMoveCounter);
  Serial.print("Loaded pump1 move counter from EEPROM: ");
  Serial.println(pump1MoveCounter);
  Serial.print("Loaded pump2 move counter from EEPROM: ");
  Serial.println(pump2MoveCounter);
  Serial.print("Loaded pump3 move counter from EEPROM: ");
  Serial.println(pump3MoveCounter);
  Serial.print("Loaded servo cover move counter from EEPROM: ");
  Serial.println(servoCoverMoveCounter);
  Serial.print("Loaded servo stamper move counter from EEPROM: ");
  Serial.println(servoStamperMoveCounter);
  Serial.print("Loaded optical sensor 1 counter from EEPROM: ");
  Serial.println(opticalSensor1Counter);
  Serial.print("Loaded optical sensor 2 counter from EEPROM: ");
  Serial.println(opticalSensor2Counter);

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

  Serial.println("CAN stamper system ready. Main loop running on core 1.");

  
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
 * - Calls y_axis_stepper.run() and z_axis_stepper.run() to update stepper positions (non-blocking).
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
  
  // Run Y-axis stepper motor
  y_axis_stepper.run();
  
  // Run Z-axis stepper motor
  z_axis_stepper.run();
  
  // Other non-blocking logic can be added here if needed
  delay(10);
}

/*
 * process_instruction(CanFrame instruction)
 * ----------------------------------------
 * Decodes and executes CAN instructions received from the queue.
 * - Each case in the switch statement corresponds to a command (see command list above).
 * - Handles actuator moves, stepper moves, pump control, servo control, optical sensor operations, counter reads/resets, and power-off.
 * - Sends appropriate CAN responses for each command, including error/status codes.
 * - Uses helper functions for actuator actions and CAN communication.
 *
 * PARAMS:
 *   instruction: CanFrame containing the received CAN message, including identifier and data payload.
 *
 * NOTES:
 *   - Many actions are non-blocking; responses may be sent before motion completes.
 *   - Error handling is robust: failures in CAN communication or timeouts are reported via response codes.
 *   - For actuator/stepper details, see respective class documentation in src/.
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
    // Read Y-axis stepper movement counter
    case 0x0A:
    {
      Serial.println("Case 0x0A: Reading Y-axis stepper movement counter");
      byte counter_high = (stepperMoveCounter >> 8) & 0xFF;
      byte counter_low = stepperMoveCounter & 0xFF;
      byte response[] = {0x0A, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

      // ***************************** CASE 0x0B ***************************** //
    // Move Y-axis stepper motor to position
    case 0x0B:
    {
      Serial.println("Case 0x0B: Moving Y-axis stepper to position");
      // Extract target position from the CAN message (4 bytes, big-endian: byte 1 is MSB)
      long targetPosition = ((uint8_t)instruction.data[1] << 24) |
                           ((uint8_t)instruction.data[2] << 16) |
                           ((uint8_t)instruction.data[3] << 8) |
                           ((uint8_t)instruction.data[4]);

      // Get direction from byte 5 (0 = negative, 1 = positive, other = use sign of position)
      int direction = instruction.data[5];
      // Apply direction
      if (direction == 1) { targetPosition = -targetPosition;}
      
      Serial.print("Case 0x0B: Moving Y-axis stepper to position: ");
      Serial.print(targetPosition);
      Serial.print(", Direction: ");
      Serial.println((direction > 0) ? "Forward" : "Backward");
      
      // Move to target position (blocking)
      y_axis_stepper.moveTo(targetPosition);
      y_axis_stepper.runToPosition();

      incrementStepperCounter();
      // Send response after movement completes
      byte response[] = {0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0C ***************************** //
    // Home Y-axis stepper motor
    case 0x0C:
    {
      Serial.println("Case 0x0C: Homing Y-axis stepper");
      // Move to home (blocking)
      y_axis_stepper.moveTo(0);
      y_axis_stepper.runToPosition();

      incrementStepperCounter();

      // Send response after movement completes
      byte response[] = {0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

  
    // ***************************** CASE 0x12 ***************************** //
    // Reset Y-axis stepper movement counter
    case 0x12:
    {
      Serial.println("Case 0x12: Resetting Y-axis stepper movement counter");
      stepperMoveCounter = 0;
      saveStepperCounter();
      delay(500);
      byte response[] = {0x12, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Read Z-axis stepper movement counter
    case 0x16:
    {
      Serial.println("Case 0x16: Reading Z-axis stepper movement counter");
      byte counter_high = (zStepperMoveCounter >> 8) & 0xFF;
      byte counter_low = zStepperMoveCounter & 0xFF;
      byte response[] = {0x16, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x17 ***************************** //
    // Move Z-axis stepper motor to position
    case 0x17:
    {
      Serial.println("Case 0x17: Moving Z-axis stepper to position");
      // Extract target position from the CAN message (4 bytes, big-endian: byte 1 is MSB)
      long targetPosition = ((uint8_t)instruction.data[1] << 24) |
                           ((uint8_t)instruction.data[2] << 16) |
                           ((uint8_t)instruction.data[3] << 8) |
                           ((uint8_t)instruction.data[4]);
      

      // Get direction from byte 5 (0 = negative, 1 = positive, other = use sign of position)
      int direction = instruction.data[5];
      
      // Apply direction
      if (direction == 1) {
        targetPosition = -targetPosition;
      }
      
      Serial.print("Case 0x17: Moving Z-axis stepper to position: ");
      Serial.print(targetPosition);
      Serial.print(", Direction: ");
      Serial.println((direction > 0) ? "Forward" : "Backward");
      
      // Move to target position (blocking)
      z_axis_stepper.moveTo(targetPosition);
      z_axis_stepper.runToPosition();

      incrementZStepperCounter();

      // Send response after movement completes
      byte response[] = {0x17, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x18 ***************************** //
    // Home Z-axis stepper motor
    case 0x18:
    {
      Serial.println("Case 0x18: Homing Z-axis stepper");
      // Move to home (blocking)
      z_axis_stepper.moveTo(0);
      z_axis_stepper.runToPosition();

      incrementZStepperCounter();

      // Send response after movement completes
      byte response[] = {0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x19 ***************************** //
    // Reset Z-axis stepper movement counter
    case 0x19:
    {
      Serial.println("Case 0x19: Resetting Z-axis stepper movement counter");
      zStepperMoveCounter = 0;
      saveZStepperCounter();
      delay(500);
      byte response[] = {0x19, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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

      // Homing Y-axis Stepper
       y_axis_stepper.moveTo(0);

      // Homing Z-axis Stepper
       z_axis_stepper.moveTo(0);

      // Send response with status
      byte statusResponse[] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
      

    }
    break;

    // ***************************** CASE 0x1A ***************************** //
    // Pump speed mode control with timer
    case 0x1A:
    {
      Serial.println("Case 0x1A: Pump speed mode control with timer");
      
      // Extract pump selection from CAN data (byte 0: 1=pump1, 2=pump2, 3=pump3)
      uint8_t pumpSelection = instruction.data[0];
      // Extract direction from CAN data (byte 1: 0=reverse, 1=forward)
      bool direction = instruction.data[1] > 0;
      // Extract speed from CAN data (bytes 2-3, big-endian: byte 2 is MSB)
      uint16_t speed = (instruction.data[2] << 8) | instruction.data[3];
      // Extract time in seconds from CAN data (bytes 4-5, big-endian: byte 4 is MSB)
      uint16_t timeSeconds = (instruction.data[4] << 8) | instruction.data[5];
      // Extract acceleration from CAN data (byte 6)
      uint8_t acceleration = instruction.data[6];
      
      Serial.print("Pump: "); Serial.print(pumpSelection);
      Serial.print(", Direction: "); Serial.print(direction);
      Serial.print(", Speed: "); Serial.print(speed);
      Serial.print(", Time: "); Serial.print(timeSeconds);
      Serial.print("s, Acceleration: "); Serial.println(acceleration);
      
      LinearActuator* selectedPump = nullptr;
      void (*incrementCounter)() = nullptr;
      
      // Select the appropriate pump and counter function
      switch(pumpSelection) {
        case 1:
          selectedPump = &pump1;
          incrementCounter = incrementPump1Counter;
          break;
        case 2:
          selectedPump = &pump2;
          incrementCounter = incrementPump2Counter;
          break;
        case 3:
          selectedPump = &pump3;
          incrementCounter = incrementPump3Counter;
          break;
        default:
          Serial.println("Invalid pump selection");
          byte errorResponse[] = {0x1A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
          send_twai_response(errorResponse);
          break;
      }
      
      if (selectedPump != nullptr) {
        // Start pump with specified speed and direction
        uint8_t payload[8] = {0};
        selectedPump->speed_mode(direction, speed, acceleration, payload);
        
        flushCanBuffer();
        
        if (CAN1.sendMsgBuf(selectedPump->motor_id, 0, 8, payload) != CAN_OK) {
          Serial.println("Error sending pump start command");
          byte errorResponse[] = {0x1A, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
          send_twai_response(errorResponse);
          break;
        }
        
        // Wait for reply and get status
        uint8_t status = waitForCanReply(selectedPump->motor_id);
        
        if (status == 0x01) {
          // Pump started successfully, now wait for specified time
          delay(timeSeconds * 1000);  // Convert seconds to milliseconds
          
          // Stop pump by sending speed 0
          uint8_t stopPayload[8] = {0};
          selectedPump->speed_mode(direction, 0, acceleration, stopPayload);
          
          flushCanBuffer();
          
          if (CAN1.sendMsgBuf(selectedPump->motor_id, 0, 8, stopPayload) != CAN_OK) {
            Serial.println("Error sending pump stop command");
            byte errorResponse[] = {0x1A, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
            send_twai_response(errorResponse);
            break;
          }
          
          // Wait for stop confirmation
          uint8_t stopStatus = waitForCanReply(selectedPump->motor_id);
          
          if (stopStatus == 0x01) {
            incrementCounter();  // Increment the appropriate counter
            byte response[] = {0x1A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // OK
            send_twai_response(response);
          } else {
            byte errorResponse[] = {0x1A, stopStatus, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            send_twai_response(errorResponse);
          }
        } else {
          byte errorResponse[] = {0x1A, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          send_twai_response(errorResponse);
        }
      }
    }
    break;

    // ***************************** CASE 0x1B ***************************** //
    // Read pump movement counters
    case 0x1B:
    {
      Serial.println("Case 0x1B: Reading pump movement counters");
      
      // Extract pump selection from CAN data (byte 0: 1=pump1, 2=pump2, 3=pump3)
      uint8_t pumpSelection = instruction.data[0];
      
      unsigned long counterValue = 0;
      bool validSelection = true;
      
      // Select the appropriate pump counter
      switch(pumpSelection) {
        case 1:
          counterValue = pump1MoveCounter;
          Serial.print("Pump1 movement counter: ");
          break;
        case 2:
          counterValue = pump2MoveCounter;
          Serial.print("Pump2 movement counter: ");
          break;
        case 3:
          counterValue = pump3MoveCounter;
          Serial.print("Pump3 movement counter: ");
          break;
        default:
          Serial.println("Invalid pump selection");
          validSelection = false;
          byte errorResponse[] = {0x1B, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
          send_twai_response(errorResponse);
          break;
      }
      
      if (validSelection) {
        Serial.println(counterValue);
        
        // Send response with counter value (4 bytes, big-endian)
        byte response[] = {
          0x1B, 0x01,  // Command and status (OK)
          (byte)((counterValue >> 24) & 0xFF),
          (byte)((counterValue >> 16) & 0xFF),
          (byte)((counterValue >> 8) & 0xFF),
          (byte)(counterValue & 0xFF),
          0x00, 0x00
        };
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x1C ***************************** //
    // Reset pump movement counters
    case 0x1C:
    {
      Serial.println("Case 0x1C: Resetting pump movement counters");
      
      // Extract pump selection from CAN data (byte 0: 1=pump1, 2=pump2, 3=pump3)
      uint8_t pumpSelection = instruction.data[0];
      
      bool validSelection = true;
      void (*saveCounter)() = nullptr;
      
      // Select the appropriate pump counter and save function
      switch(pumpSelection) {
        case 1:
          pump1MoveCounter = 0;
          saveCounter = savePump1Counter;
          Serial.println("Pump1 movement counter reset to 0");
          break;
        case 2:
          pump2MoveCounter = 0;
          saveCounter = savePump2Counter;
          Serial.println("Pump2 movement counter reset to 0");
          break;
        case 3:
          pump3MoveCounter = 0;
          saveCounter = savePump3Counter;
          Serial.println("Pump3 movement counter reset to 0");
          break;
        default:
          Serial.println("Invalid pump selection");
          validSelection = false;
          byte errorResponse[] = {0x1C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
          send_twai_response(errorResponse);
          break;
      }
      
      if (validSelection && saveCounter != nullptr) {
        saveCounter();  // Save the reset counter to EEPROM
        delay(500);
        byte response[] = {0x1C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // OK
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x1D ***************************** //
    // Control servo motors with selection
    case 0x1D:
    {
      Serial.println("Case 0x1D: Controlling servo motors");
      
      // Extract servo selection and angle from CAN data
      // Byte 1: servo selection (1=servo_cover, 2=servo_stamper)
      // Bytes 2: angle (0-180 degrees, little-endian)
      uint8_t servoSelection = instruction.data[1];
      uint8_t angle = instruction.data[2];
      
      // Validate angle range (0-180 degrees)
      if (angle > 180) {
        Serial.println("Invalid angle: must be 0-180 degrees");
        byte errorResponse[] = {0x1D, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
        send_twai_response(errorResponse);
        break;
      }
      
      bool validSelection = true;
      void (*incrementCounter)() = nullptr;
      
      // Select the appropriate servo and increment function
      switch(servoSelection) {
        case 1:
          servo_cover.write(angle);
          incrementCounter = incrementServoCoverCounter;
          Serial.print("Servo cover moved to angle: ");
          Serial.println(angle);
          break;
        case 2:
          servo_stamper.write(angle);
          incrementCounter = incrementServoStamperCounter;
          Serial.print("Servo stamper moved to angle: ");
          Serial.println(angle);
          break;
        default:
          Serial.println("Invalid servo selection");
          validSelection = false;
          byte errorResponse[] = {0x1D, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
          send_twai_response(errorResponse);
          break;
      }
      
      if (validSelection && incrementCounter != nullptr) {
        incrementCounter();  // Increment and save the counter
        delay(500);  // Allow time for servo to move
        byte response[] = {0x1D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // OK
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x1E ***************************** //
    // Read servo movement counters
    case 0x1E:
    {
      Serial.println("Case 0x1E: Reading servo movement counters");
      
      // Extract servo selection from CAN data (byte 0: 1=servo_cover, 2=servo_stamper)
      uint8_t servoSelection = instruction.data[0];
      
      uint32_t counterValue = 0;
      bool validSelection = true;
      
      // Select the appropriate servo counter
      switch(servoSelection) {
        case 1:
          counterValue = servoCoverMoveCounter;
          Serial.print("Servo cover movement counter: ");
          Serial.println(counterValue);
          break;
        case 2:
          counterValue = servoStamperMoveCounter;
          Serial.print("Servo stamper movement counter: ");
          Serial.println(counterValue);
          break;
        default:
          Serial.println("Invalid servo selection");
          validSelection = false;
          byte errorResponse[] = {0x1E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
          send_twai_response(errorResponse);
          break;
      }
      
      if (validSelection) {
        // Send counter value as 4-byte big-endian response
        byte response[] = {
          0x1E, 0x01,  // Command and status
          (byte)((counterValue >> 24) & 0xFF),
          (byte)((counterValue >> 16) & 0xFF),
          (byte)((counterValue >> 8) & 0xFF),
          (byte)(counterValue & 0xFF),
          0x00, 0x00
        };
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x1F ***************************** //
    // Reset servo movement counters
    case 0x1F:
    {
      Serial.println("Case 0x1F: Resetting servo movement counters");
      
      // Extract servo selection from CAN data (byte 0: 1=servo_cover, 2=servo_stamper)
      uint8_t servoSelection = instruction.data[0];
      
      bool validSelection = true;
      void (*saveCounter)() = nullptr;
      
      // Select the appropriate servo counter and save function
      switch(servoSelection) {
        case 1:
          servoCoverMoveCounter = 0;
          saveCounter = saveServoCoverCounter;
          Serial.println("Servo cover movement counter reset to 0");
          break;
        case 2:
          servoStamperMoveCounter = 0;
          saveCounter = saveServoStamperCounter;
          Serial.println("Servo stamper movement counter reset to 0");
          break;
        default:
          Serial.println("Invalid servo selection");
          validSelection = false;
          byte errorResponse[] = {0x1F, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
          send_twai_response(errorResponse);
          break;
      }
      
      if (validSelection && saveCounter != nullptr) {
        saveCounter();  // Save the reset counter to EEPROM
        delay(500);
        byte response[] = {0x1F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // OK
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x20 ***************************** //
    // Move stepper until optical sensor is activated
    case 0x20:
    {
      Serial.println("Case 0x20: Move stepper until optical sensor activated");
      
      // Extract parameters from CAN data
      uint8_t stepperSelection = instruction.data[0];  // 1=X-axis, 2=Y-axis, 3=Z-axis
      uint8_t sensorSelection = instruction.data[1];   // 1=optical_sensor_1, 2=optical_sensor_2
      uint8_t direction = instruction.data[2];         // 0=negative, 1=positive
      uint16_t maxSteps = (instruction.data[3] << 8) | instruction.data[4];  // Maximum steps to move
      
      bool validParameters = true;
      AccelStepper* selectedStepper = nullptr;
      int sensorPin = 0;
      void (*incrementSensorCounter)() = nullptr;
      
      // Select the appropriate stepper motor
      switch(stepperSelection) {
        case 1:
          selectedStepper = &y_axis_stepper;
          Serial.println("Selected Y-axis stepper");
          break;
        case 2:
          selectedStepper = &z_axis_stepper;
          Serial.println("Selected Z-axis stepper");
          break;
        default:
          Serial.println("Invalid stepper selection");
          validParameters = false;
          break;
      }
      
      // Select the appropriate optical sensor
      switch(sensorSelection) {
        case 1:
          sensorPin = OPTICAL_SENSOR_1_PIN;
          incrementSensorCounter = incrementOpticalSensor1Counter;
          Serial.println("Selected optical sensor 1");
          break;
        case 2:
          sensorPin = OPTICAL_SENSOR_2_PIN;
          incrementSensorCounter = incrementOpticalSensor2Counter;
          Serial.println("Selected optical sensor 2");
          break;
        default:
          Serial.println("Invalid sensor selection");
          validParameters = false;
          break;
      }
      
      if (!validParameters) {
        byte errorResponse[] = {0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
        send_twai_response(errorResponse);
        break;
      }
      
      // Move stepper until sensor is activated or max steps reached
      long currentPosition = selectedStepper->currentPosition();
      long targetPosition = currentPosition + (direction == 1 ? maxSteps : -maxSteps);
      selectedStepper->moveTo(targetPosition);
      
      uint16_t stepsMoved = 0;
      bool sensorActivated = false;
      
      Serial.print("Moving stepper ");
      Serial.print(direction == 1 ? "forward" : "backward");
      Serial.print(" until sensor activated or ");
      Serial.print(maxSteps);
      Serial.println(" steps reached");
      
      while (selectedStepper->distanceToGo() != 0 && stepsMoved < maxSteps) {
        selectedStepper->run();
        
        // Check if optical sensor is activated (LOW = activated with pullup)
        if (digitalRead(sensorPin) == LOW) {
          selectedStepper->stop();
          sensorActivated = true;
          incrementSensorCounter();  // Increment the sensor counter
          Serial.println("Optical sensor activated!");
          break;
        }
        
        // Update steps moved counter
        if (abs(selectedStepper->currentPosition() - currentPosition) > stepsMoved) {
          stepsMoved = abs(selectedStepper->currentPosition() - currentPosition);
        }
        
        delay(1);  // Small delay to prevent overwhelming the system
      }
      
      // Prepare response with steps moved and sensor status
      byte response[8];
      response[0] = 0x20;  // Command ID
      response[1] = sensorActivated ? 0x01 : 0x00;  // Status: 1=sensor activated, 0=max steps reached
      response[2] = (stepsMoved >> 8) & 0xFF; // Steps moved (high byte)
      response[3] = stepsMoved & 0xFF;        // Steps moved (low byte)
      response[4] = 0x00;
      response[5] = 0x00;
      response[6] = 0x00;
      response[7] = 0x00;
      
      send_twai_response(response);
      
      Serial.print("Movement completed. Steps moved: ");
      Serial.print(stepsMoved);
      Serial.print(", Sensor activated: ");
      Serial.println(sensorActivated ? "Yes" : "No");
    }
    break;

    // ***************************** CASE 0x21 ***************************** //
    // Read optical sensor counters
    case 0x21:
    {
      Serial.println("Case 0x21: Reading optical sensor counters");
      
      // Extract sensor selection from CAN data (byte 0: 1=sensor_1, 2=sensor_2)
      uint8_t sensorSelection = instruction.data[0];
      
      unsigned long counterValue = 0;
      bool validSelection = true;
      
      // Select the appropriate sensor counter
      switch(sensorSelection) {
        case 1:
          counterValue = opticalSensor1Counter;
          Serial.print("Optical sensor 1 counter: ");
          Serial.println(counterValue);
          break;
        case 2:
          counterValue = opticalSensor2Counter;
          Serial.print("Optical sensor 2 counter: ");
          Serial.println(counterValue);
          break;
        default:
          Serial.println("Invalid sensor selection");
          validSelection = false;
          byte errorResponse[] = {0x21, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
          send_twai_response(errorResponse);
          break;
      }
      
      if (validSelection) {
        // Send counter value as 4-byte big-endian response
        byte response[8];
        response[0] = 0x21;  // Command ID
        response[1] = 0x01;  // Status: OK
        response[2] = (counterValue >> 24) & 0xFF; // Counter byte 3 (MSB)
        response[3] = (counterValue >> 16) & 0xFF; // Counter byte 2
        response[4] = (counterValue >> 8) & 0xFF;  // Counter byte 1
        response[5] = counterValue & 0xFF;         // Counter byte 0 (LSB)
        response[6] = 0x00;
        response[7] = 0x00;
        
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x22 ***************************** //
    // Reset optical sensor counters
    case 0x22:
    {
      Serial.println("Case 0x22: Resetting optical sensor counters");
      
      // Extract sensor selection from CAN data (byte 0: 1=sensor_1, 2=sensor_2)
      uint8_t sensorSelection = instruction.data[0];
      
      bool validSelection = true;
      void (*saveCounter)() = nullptr;
      
      // Select the appropriate sensor counter and save function
      switch(sensorSelection) {
        case 1:
          opticalSensor1Counter = 0;
          saveCounter = saveOpticalSensor1Counter;
          Serial.println("Optical sensor 1 counter reset to 0");
          break;
        case 2:
          opticalSensor2Counter = 0;
          saveCounter = saveOpticalSensor2Counter;
          Serial.println("Optical sensor 2 counter reset to 0");
          break;
        default:
          Serial.println("Invalid sensor selection");
          validSelection = false;
          byte errorResponse[] = {0x22, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // FAIL
          send_twai_response(errorResponse);
          break;
      }
      
      if (validSelection && saveCounter != nullptr) {
        saveCounter();  // Save the reset counter to EEPROM
        delay(500);
        byte response[] = {0x22, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // OK
        send_twai_response(response);
      }
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
