/*
 * Command Documentation for process_instruction function
 * 
 * The process_instruction function processes CAN bus commands for the pick and place system.
 * Each command is identified by the first byte of the CAN message (instruction.data[0]).
 * 
 * Command Structure:
 * - Each command has a 1-byte command code (0x01-0xFF)
 * - Commands may have additional data bytes as parameters
 * - Each command returns an 8-byte response
 * 
 * Command List:
 * 
 * 0x01 - Reset Microcontroller
 *   Description: Resets the microcontroller
 *   Response: [0x01, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x02 - Send Heartbeat
 *   Description: Responds with a heartbeat message
 *   Response: [0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x03 - Home Actuators
 *   Description: Homes both X and Z axis actuators
 *   Response: [0x03, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x04 - Move Actuator X to Absolute Position
 *   Parameters: [pos_high, pos_low, orientation]
 *   Response: [0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x05 - Move Actuator Z to Absolute Position
 *   Parameters: [pos_high, pos_low, orientation]
 *   Response: [0x05, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x06 - Read Actuator Z Movement Counter
 *   Response: [0x06, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x07 - Read Actuator X Movement Counter
 *   Response: [0x07, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x08 - Reset Actuator X Movement Counter
 *   Response: [0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x09 - Reset Actuator Z Movement Counter
 *   Response: [0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x0A - Move Left Conveyor in Speed Mode
 *   Parameters: [direction, speed_high, speed_low, acceleration]
 *   Response: [0x0A, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x0B - Move Right Conveyor in Speed Mode
 *   Parameters: [direction, speed_high, speed_low, acceleration]
 *   Response: [0x0B, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x0D - Open Gripper
 *   Response: [0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x0E - Close Gripper
 *   Response: [0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x0F - Set Gripper Force
 *   Parameters: [force_value]
 *   Response: [0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x10 - Read Gripper Movement Counter
 *   Response: [0x10, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x11 - Reset Gripper Movement Counter
 *   Response: [0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x12 - Read Left Conveyor Movement Counter
 *   Response: [0x12, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x13 - Reset Left Conveyor Movement Counter
 *   Response: [0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x14 - Read Right Conveyor Movement Counter
 *   Response: [0x14, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x15 - Reset Right Conveyor Movement Counter
 *   Response: [0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x18 - Move Left Conveyor Until IR Sensor Activation
 *   Parameters: [direction, speed_high, speed_low, acceleration]
 *   Response: [0x18, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x19 - Move Right Conveyor Until IR Sensor Activation
 *   Parameters: [direction, speed_high, speed_low, acceleration]
 *   Response: [0x19, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x1A - Check Left IR Sensor Status
 *   Response: [0x1A, 0x01, sensor_status, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0x1B - Check Right IR Sensor Status
 *   Response: [0x1B, 0x01, sensor_status, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * 0xFF - Power Off (Move All to Home Position)
 *   Description: Moves all actuators to home position and opens gripper
 *   Response: [0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 * 
 * Status Codes:
 * 0x01 - Success
 * 0x02 - Failure
 * 0x03 - Timeout
 * 0x04 - Network/Communication Error
 * 0xFF - Unknown Command
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
#define EEPROM_SIZE 24  // 4 bytes for each counter (servos + actuator)
#define ACTUATOR_COUNTER_ADDR 4  // Address in EEPROM to store the actuator counter
#define ACTUATOR_COUNTER_ADDR2 8 // Address in EEPROM to store the actuator counter
#define CONVEYOR_COUNTER_L_ADDR 12 // Address in EEPROM to store the conveyor counter
#define CONVEYOR_COUNTER_R_ADDR 16 // Address in EEPROM to store the conveyor counter
#define GRIPPER_COUNTER_ADDR 20 // Address in EEPROM to store the gripper counter

// Global variables 
unsigned long actuatorMoveCounterX = 0;
unsigned long actuatorMoveCounterZ = 0;
unsigned long conveyorMoveCounterL = 0;
unsigned long conveyorMoveCounterR = 0;
unsigned long gripperMoveCounter = 0;

byte replyData[8];  // Buffer for CAN replies

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 26  // Changed from 15 to 26 to avoid upload issues
#define CAN1_INT 25  // Changed from 2 to 25 to avoid boot issues

// IR Sensor pins
#define IR_SENSOR_LEFT_PIN 21   // GPIO21 - Left side IR sensor
#define IR_SENSOR_RIGHT_PIN 22  // GPIO22 - Right side IR sensor

// Instance of GripperDigital
GripperDigital gripper(14, 16, 17);

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x191
#define RESPONSE_CAN_ID 0x591

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Constants for actuator positions
#define ACTUATOR_HOME_POSITION 0

// Instance of LinearActuator
LinearActuator x_axis(0x001);
LinearActuator z_axis(0x002);
LinearActuator conveyor_left(0x003);
LinearActuator conveyor_right(0x004);


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
void saveActuatorCounterX()
{
  EEPROM.writeULong(ACTUATOR_COUNTER_ADDR, actuatorMoveCounterX);
  EEPROM.commit();
  Serial.print("Actuator counter saved: ");
  Serial.println(actuatorMoveCounterX);
}

void saveActuatorCounterZ()
{
  EEPROM.writeULong(ACTUATOR_COUNTER_ADDR2, actuatorMoveCounterZ);
  EEPROM.commit();
  Serial.print("Actuator counter saved: ");
  Serial.println(actuatorMoveCounterZ);
}

void saveGripperCounter()
{
  EEPROM.writeULong(GRIPPER_COUNTER_ADDR, gripperMoveCounter);
  EEPROM.commit();
  Serial.print("Gripper counter saved: ");
  Serial.println(gripperMoveCounter);
}

void saveConveyorCounterL()
{
  EEPROM.writeULong(CONVEYOR_COUNTER_L_ADDR, conveyorMoveCounterL);
  EEPROM.commit();
  Serial.print("Conveyor counter saved: ");
  Serial.println(conveyorMoveCounterL);  
}

void saveConveyorCounterR()
{
  EEPROM.writeULong(CONVEYOR_COUNTER_R_ADDR, conveyorMoveCounterR);
  EEPROM.commit();
  Serial.print("Conveyor counter saved: ");
  Serial.println(conveyorMoveCounterR);  
}

// Function to increment and save the actuator counter
void incrementActuatorCounterX()
{ 
  actuatorMoveCounterX++;
  saveActuatorCounterX();
}

void incrementActuatorCounterZ()
{ 
  actuatorMoveCounterZ++;
  saveActuatorCounterZ();
}

void incrementGripperCounter()
{
  gripperMoveCounter++;
  saveGripperCounter();
}

void incrementConveyorCounterL()
{
  conveyorMoveCounterL++;
  saveConveyorCounterL();
}

void incrementConveyorCounterR()
{
  conveyorMoveCounterR++;
  saveConveyorCounterR();
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  actuatorMoveCounterX = EEPROM.readULong(ACTUATOR_COUNTER_ADDR);
  actuatorMoveCounterZ = EEPROM.readULong(ACTUATOR_COUNTER_ADDR2);
  gripperMoveCounter = EEPROM.readULong(GRIPPER_COUNTER_ADDR);
  Serial.print("Loaded actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterX);
  Serial.print("Loaded actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterZ);
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

  // Initialize IR sensors
  pinMode(IR_SENSOR_LEFT_PIN, INPUT_PULLUP);
  pinMode(IR_SENSOR_RIGHT_PIN, INPUT_PULLUP);
  Serial.println("IR sensors initialized on pins 21 (left) and 22 (right)");

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

      // Homing X axis

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode(ACTUATOR_HOME_POSITION, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x03, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterX();
      }      

      // Send response with status
      byte statusResponse[] = {0x03, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Move actuator X to absolute position
    case 0x04:
    {
      Serial.println("Case 0x04: Moving actuator X to absolute position");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode(angle, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterX();
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
    // Read actuator movement counter X
    case 0x07:
    {
      Serial.println("Case 0x07: Reading actuator movement counter X");
      byte counter_high = (actuatorMoveCounterX >> 8) & 0xFF;
      byte counter_low = actuatorMoveCounterX & 0xFF;
      byte response[] = {0x07, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Reset actuator movement counter X
    case 0x08:
    {
      Serial.println("Case 0x08: Resetting actuator movement counter Y");
      actuatorMoveCounterX = 0;
      saveActuatorCounterX();
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
    // Move conveyor left in speed mode
    case 0x0A:
    {
      Serial.println("Case 0x0A: Moving conveyor left in speed mode");
      
      uint8_t dir = instruction.data[1];
      uint16_t speed = (instruction.data[2] << 8) | instruction.data[3];
      uint8_t acc = instruction.data[4];
      

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      conveyor_left.speed_mode(dir, speed, acc, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(conveyor_left.motor_id, 0, 5, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x0A, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReplyConveyor(conveyor_left.motor_id);

      if (status == 0x01) {
        incrementConveyorCounterL();
      }   
      
      delay(300);
      byte response[] = {0x0A, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;
    
    // ***************************** CASE 0x0B ***************************** //
    // Move conveyor right in speed mode
    case 0x0B:
    {
      Serial.println("Case 0x0B: Moving conveyor right in speed mode");
      
      uint8_t dir = instruction.data[1];
      uint16_t speed = (instruction.data[2] << 8) | instruction.data[3];
      uint8_t acc = instruction.data[4];
      

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      conveyor_right.speed_mode(dir, speed, acc, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(conveyor_right.motor_id, 0, 5, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x0B, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReplyConveyor(conveyor_right.motor_id);

      if (status == 0x01) {
        incrementConveyorCounterR();
      }   
      
      delay(300);
      byte response[] = {0x0B, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
      
      delay(300);
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
      
      delay(300);
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
    // Read left conveyor movement counter
    case 0x12:
    {
      Serial.println("Case 0x12: Reading conveyor movement counter");

      byte counter_high = (conveyorMoveCounterL >> 8) & 0xFF;
      byte counter_low = conveyorMoveCounterL & 0xFF;
      byte response[] = {0x12, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x13 ***************************** //
    // Reset left conveyor movement counter
    case 0x13:
    {
      Serial.println("Case 0x13: Resetting conveyor movement counter");
      conveyorMoveCounterL = 0;
      saveConveyorCounterL();
      delay(500);
      byte response[] = {0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //
    // Read right conveyor movement counter
    case 0x14:
    {
      Serial.println("Case 0x14: Reading conveyor movement counter");

      byte counter_high = (conveyorMoveCounterR >> 8) & 0xFF;
      byte counter_low = conveyorMoveCounterR & 0xFF;
      byte response[] = {0x14, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x15 ***************************** //
    // Reset right conveyor movement counter
    case 0x15:
    {
      Serial.println("Case 0x15: Resetting conveyor movement counter");
      conveyorMoveCounterR = 0;
      saveConveyorCounterR();
      delay(500);
      byte response[] = {0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Home X axis using go_home function
    case 0x16:
    {
      Serial.println("Case 0x16: Homing X axis using go_home");
      
      uint8_t payload[2] = {0};  // Initialize buffer for CAN message (2 bytes)
      x_axis.go_home(payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 2, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x16, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterX();
      }
      
      // Send response with status
      byte statusResponse[] = {0x16, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x17 ***************************** //
    // Home Z axis using go_home function
    case 0x17:
    {
      Serial.println("Case 0x17: Homing Z axis using go_home");
      
      uint8_t payload[2] = {0};  // Initialize buffer for CAN message (2 bytes)
      z_axis.go_home(payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 2, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x17, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(z_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }
      
      // Send response with status
      byte statusResponse[] = {0x17, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x18 ***************************** //
    // Move actuator X to absolute position with speed control
    case 0x1C:
    {
      Serial.println("Case 0x18: Moving actuator X to absolute position with speed control");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];
      uint16_t speed = instruction.data[4] << 8 | instruction.data[5]; // Speed parameter

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode_with_speed_control(angle, speed, payload);  // Generate the CAN message with speed control

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x18, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterX();
      }

      // Send response with status
      byte statusResponse[] = {0x18, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x19 ***************************** //
    // Move actuator Z to absolute position with speed control
    case 0x1D:
    {
      Serial.println("Case 0x19: Moving actuator Z to absolute position with speed control");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];
      uint16_t speed = instruction.data[4] << 8 | instruction.data[5]; // Speed parameter

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      z_axis.abs_mode_with_speed_control(angle, speed, payload);  // Generate the CAN message with speed control

      if (CAN1.sendMsgBuf(z_axis.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x19, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(z_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }   

      // Send response with status
      byte statusResponse[] = {0x19, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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


      // Homing X
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode(0, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterX();
      }
      else 
      {
        byte errorResponse[] = {0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      } 

      // Open gripper
      gripper.open();


      // Send response with status
      byte statusResponse[] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
      

    }
    break;

    // ***************************** CASE 0x18 ***************************** //
    // Move left conveyor until IR sensor is activated
    case 0x18:
    {
      Serial.println("Case 0x18: Moving left conveyor until IR sensor activation");
      
      uint8_t dir = instruction.data[1];
      uint16_t speed = (instruction.data[2] << 8) | instruction.data[3];
      uint8_t acc = instruction.data[4];
      
      // Start conveyor movement
      uint8_t payload[8] = {0};
      conveyor_left.speed_mode(dir, speed, acc, payload);
      
      if (CAN1.sendMsgBuf(conveyor_left.motor_id, 0, 5, payload) != CAN_OK) {
        Serial.println("Error sending conveyor command");
        byte errorResponse[] = {0x18, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(errorResponse);
        break;
      }
      
      // Monitor IR sensor with timeout (30 seconds)
      unsigned long startTime = millis();
      const unsigned long timeout = 30000;
      uint8_t sensorStatus = 0x03; // Default to timeout
      
      while (millis() - startTime < timeout) {
        if (digitalRead(IR_SENSOR_LEFT_PIN) == LOW) { // Object detected
          // Stop conveyor
          uint8_t stopPayload[8] = {0};
          conveyor_left.speed_mode(0, 0, 0, stopPayload);
          CAN1.sendMsgBuf(conveyor_left.motor_id, 0, 5, stopPayload);
          
          sensorStatus = 0x01; // Success
          incrementConveyorCounterL();
          Serial.println("Left IR sensor activated - conveyor stopped");
          break;
        }
        delay(10);
      }
      
      if (sensorStatus == 0x03) {
        // Timeout - stop conveyor
        uint8_t stopPayload[8] = {0};
        conveyor_left.speed_mode(0, 0, 0, stopPayload);
        CAN1.sendMsgBuf(conveyor_left.motor_id, 0, 5, stopPayload);
        Serial.println("Timeout - left conveyor stopped");
      }
      
      byte response[] = {0x18, sensorStatus, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;
    
    // ***************************** CASE 0x19 ***************************** //
    // Move right conveyor until IR sensor is activated
    case 0x19:
    {
      Serial.println("Case 0x19: Moving right conveyor until IR sensor activation");
      
      uint8_t dir = instruction.data[1];
      uint16_t speed = (instruction.data[2] << 8) | instruction.data[3];
      uint8_t acc = instruction.data[4];
      
      // Start conveyor movement
      uint8_t payload[8] = {0};
      conveyor_right.speed_mode(dir, speed, acc, payload);
      
      if (CAN1.sendMsgBuf(conveyor_right.motor_id, 0, 5, payload) != CAN_OK) {
        Serial.println("Error sending conveyor command");
        byte errorResponse[] = {0x19, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(errorResponse);
        break;
      }
      
      // Monitor IR sensor with timeout (30 seconds)
      unsigned long startTime = millis();
      const unsigned long timeout = 30000;
      uint8_t sensorStatus = 0x03; // Default to timeout
      
      while (millis() - startTime < timeout) {
        if (digitalRead(IR_SENSOR_RIGHT_PIN) == LOW) { // Object detected
          // Stop conveyor
          uint8_t stopPayload[8] = {0};
          conveyor_right.speed_mode(0, 0, 0, stopPayload);
          CAN1.sendMsgBuf(conveyor_right.motor_id, 0, 5, stopPayload);
          
          sensorStatus = 0x01; // Success
          incrementConveyorCounterR();
          Serial.println("Right IR sensor activated - conveyor stopped");
          break;
        }
        delay(10);
      }
      
      if (sensorStatus == 0x03) {
        // Timeout - stop conveyor
        uint8_t stopPayload[8] = {0};
        conveyor_right.speed_mode(0, 0, 0, stopPayload);
        CAN1.sendMsgBuf(conveyor_right.motor_id, 0, 5, stopPayload);
        Serial.println("Timeout - right conveyor stopped");
      }
      
      byte response[] = {0x19, sensorStatus, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x1A ***************************** //
    // Check left IR sensor status
    case 0x1A:
    {
      Serial.println("Case 0x1A: Check left IR sensor status");
      
      uint8_t sensorStatus = digitalRead(IR_SENSOR_LEFT_PIN) == LOW ? 0x01 : 0x00;
      
      byte response[] = {0x1A, 0x01, sensorStatus, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x1B ***************************** //
    // Check right IR sensor status
    case 0x1B:
    {
      Serial.println("Case 0x1B: Check right IR sensor status");
      
      uint8_t sensorStatus = digitalRead(IR_SENSOR_RIGHT_PIN) == LOW ? 0x01 : 0x00;
      
      byte response[] = {0x1B, 0x01, sensorStatus, 0x00, 0x00, 0x00, 0x00, 0x00};
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


// Helper function to wait for CAN reply
uint8_t waitForCanReply(uint16_t expectedId)
{
  memset(replyData, 0, sizeof(replyData)); // Clear the buffer before waiting
  unsigned long startTime = millis();
  const unsigned long timeout = 6000;  // 1 second timeout
  
  while (millis() - startTime < timeout) {
    if (CAN1.checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      byte len = 0;
      CAN1.readMsgBuf(&canId, &len, replyData);
      
      if (canId == expectedId && (replyData[1] == 0x02 || replyData[1] == 0x03))
      {
        return 0x01;  // Success
      }
      else if (canId == expectedId && replyData[1] == 0x00)
      {
        return 0x02;  // Failure
      }
    }
    vTaskDelay(1);  // Small delay to prevent busy-waiting
  }
  return 0x03;  // Timeout
}

uint8_t waitForCanReplyConveyor(uint16_t expectedId)
{
  memset(replyData, 0, sizeof(replyData)); // Clear the buffer before waiting
  unsigned long startTime = millis();
  const unsigned long timeout = 6000;  // 1 second timeout
  
  while (millis() - startTime < timeout) {
    if (CAN1.checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      byte len = 0;
      CAN1.readMsgBuf(&canId, &len, replyData);
      
      if (canId == expectedId && (replyData[1] == 0x01))
      {
        return 0x01;  // Success
      }
      else if (canId == expectedId && replyData[1] == 0x00)
      {
        return 0x02;  // Failure
      }
    }
    vTaskDelay(1);  // Small delay to prevent busy-waiting
  }
  return 0x03;  // Timeout
}

// Helper function to send response via TWAI
void send_twai_response(const byte response_data[8])
{
  CanFrame tx_frame;
  tx_frame.identifier = RESPONSE_CAN_ID;  // Response ID
  tx_frame.extd = 0;
  tx_frame.data_length_code = 8;
  memcpy(tx_frame.data, response_data, 8);
  ESP32Can.writeFrame(tx_frame);
}

