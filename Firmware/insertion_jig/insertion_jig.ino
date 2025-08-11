/**
 * Insertion Jig Control System
 * 
 * This program controls a multi-axis insertion jig system using CAN bus communication.
 * It supports both TWAI (ESP32 Integrated CAN) and MCP2515-based CAN interfaces.
 * 
 * Instruction Set (Case ID - Description):
 * 
 * 0x01 - Reset Microcontroller
 *   - Resets the microcontroller and sends an acknowledgment before restarting.
 *   - Response: [0x01, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x02 - Heartbeat
 *   - Acknowledges that the system is alive and responsive.
 *   - Response: [0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x03 - Home X Axis
 *   - Moves the X-axis actuator to its home position (0).
 *   - Response: [0x03, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x04 - Move X Axis to Absolute Position
 *   - Moves the X-axis to a specified absolute position.
 *   - Data: [0x04, pos_high, pos_low, orientation, 0x00, 0x00, 0x00, 0x00]
 *   - Response: [0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x05 - Home Z Axis
 *   - Moves all Z-axis actuators to their home positions (0).
 *   - Response: [0x05, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x06 - Move Z Axis to Absolute Position
 *   - Moves all Z-axis actuators to a specified absolute position.
 *   - Data: [0x06, pos_high, pos_low, orientation, 0x00, 0x00, 0x00, 0x00]
 *   - Response: [0x06, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x08 - Get X Movement Counter
 *   - Returns the number of movements made by the X-axis actuator.
 *   - Response: [0x08, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x09 - Reset X Movement Counter
 *   - Resets the X-axis movement counter to zero.
 *   - Response: [0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x0A - Get Z Movement Counter
 *   - Returns the number of movements made by the Z-axis actuators.
 *   - Response: [0x0A, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x0B - Reset Z Movement Counter
 *   - Resets the Z-axis movement counter to zero.
 *   - Response: [0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0x0C - Move to Predefined Position
 *   - Moves the X-axis to a predefined position based on the position ID.
 *   - Data: [0x0C, position_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *   - Response: [0x0C, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * 0xFF - Power Off
 *   - Moves all axes to home position and prepares for power off.
 *   - Response: [0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 *
 * Status Codes:
 * 0x01 - OK
 * 0x02 - FAIL
 * 0x03 - TIMEOUT
 * 0x04 - NO LOCAL NETWORK
 */

#include <ESP32-TWAI-CAN.hpp>
#include <mcp_can.h>
#include <SPI.h>
#include "src/linear_actuator.h"
#include <ESP32Servo.h>


// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 16 // 4 bytes for each counter (servos + actuator)
#define ACTUATOR_COUNTER_ADDR_X 4  // Address in EEPROM to store the actuator counter
#define ACTUATOR_COUNTER_ADDR_Z 8 // Address in EEPROM to store the actuator counter
#define ACTUATOR_INSERTION_POSITION_Z_ADDR 12
// Global variables
unsigned long actuatorMoveCounterX = 0;
unsigned long actuatorMoveCounterZ = 0;

byte replyData[8];  // Buffer for CAN replies

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 26  // Changed from 15 to 26 to avoid upload issues
#define CAN1_INT 25  // Changed from 2 to 25 to avoid boot issues
//The rest keep standard MCP2515 configuration
// MOSI = 23
// MISO = 19
// SCK = 18
// CS = 26
// INT = 25

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x0C9
#define RESPONSE_CAN_ID 0x4C9

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Instance of LinearActuator
LinearActuator x_axis(0x001);
LinearActuator z1_axis(0x002);
LinearActuator z2_axis(0x003);
LinearActuator z3_axis(0x004);


// Actuator positions on X axis
int ACTUATOR_HOME_POSITION_X = 0;
int ACTUATOR_LUB_NOZZLE = 0;
int ACTUATOR_INS_NOZZLE = 100;
int ACTUATOR_LUB_JOINT = 120;
int ACTUATOR_INS_JOINT = 120;

// Actuator positions on Z axis
int ACTUATOR_HOME_POSITION_Z = 0;
int ACTUATOR_INSERTION_POSITION_Z = 100;


void saveConfigValues()
{
  EEPROM.writeUShort(ACTUATOR_INSERTION_POSITION_Z_ADDR, ACTUATOR_INSERTION_POSITION_Z);
  EEPROM.commit();
}

void loadConfigValues()
{
  ACTUATOR_INSERTION_POSITION_Z = EEPROM.readUShort(ACTUATOR_INSERTION_POSITION_Z_ADDR);
}

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


// Function to save actuator counter x to EEPROM
void saveActuatorCounterX()
{
  EEPROM.writeULong(ACTUATOR_COUNTER_ADDR_X, actuatorMoveCounterX);
  EEPROM.commit();
  Serial.print("Actuator counter saved: ");
  Serial.println(actuatorMoveCounterX);
}

// Function to save actuator counter z to EEPROM
void saveActuatorCounterZ()
{
  EEPROM.writeULong(ACTUATOR_COUNTER_ADDR_Z, actuatorMoveCounterZ);
  EEPROM.commit();
  Serial.print("Actuator counter saved: ");
  Serial.println(actuatorMoveCounterZ);
}

// Function to increment and save the actuator counter x
void incrementActuatorCounterX()
{
  actuatorMoveCounterX++;
  saveActuatorCounterX();
}

// Function to increment and save the actuator counter z
void incrementActuatorCounterZ()
{
  actuatorMoveCounterZ++;
  saveActuatorCounterZ();
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  actuatorMoveCounterX = EEPROM.readULong(ACTUATOR_COUNTER_ADDR_X);
  actuatorMoveCounterZ = EEPROM.readULong(ACTUATOR_COUNTER_ADDR_Z);
  Serial.print("Loaded actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounterX);

  // Load persistent config values
  loadConfigValues();
  Serial.print("Loaded ACTUATOR_INSERTION_POSITION_Z: "); Serial.println(ACTUATOR_INSERTION_POSITION_Z);

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
      Serial.println("Case 0x01: Read X axis position");
      byte response[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
      delay(100);
      ESP.restart();
    }
    break;

    // ***************************** CASE 0x02 ***************************** //
    // Send Heartbeat
    case 0x02:
    {
      Serial.println("Case 0x02: Heartbeat");
      byte response[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x03 ***************************** //
    // Move actuator X to home position
    case 0x03:
    { 
      Serial.println("Case 0x03: Homing actuator X");

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode(ACTUATOR_HOME_POSITION_X, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

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

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
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
    // Move actuator Z to home position
    case 0x05:
    {
      Serial.println("Case 0x05: Moving actuator Z to home position");

      uint8_t payload1[8] = {0};  // Initialize buffer for CAN message
      uint8_t payload2[8] = {0};  // Initialize buffer for CAN message
      uint8_t payload3[8] = {0};  // Initialize buffer for CAN message

      z1_axis.abs_mode(ACTUATOR_HOME_POSITION_Z, payload1);  // Generate the CAN message
      z2_axis.abs_mode(ACTUATOR_HOME_POSITION_Z, payload2);  // Generate the CAN message
      z3_axis.abs_mode(ACTUATOR_HOME_POSITION_Z, payload3);  // Generate the CAN message
      
      if (CAN1.sendMsgBuf(z1_axis.motor_id, 0, 8, payload1) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x05, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      if (CAN1.sendMsgBuf(z2_axis.motor_id, 0, 8, payload2) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x05, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      if (CAN1.sendMsgBuf(z3_axis.motor_id, 0, 8, payload3) != CAN_OK)
      {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x05, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReplyMultiple(z1_axis.motor_id, z2_axis.motor_id, z3_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }

      // Send response with status
      byte statusResponse[] = {0x05, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
 
    // ***************************** CASE 0x06 ***************************** //
    // Move actuator Z to absolute position
    case 0x06:
    {
      Serial.println("Case 0x06: Moving actuator Z to absolute position");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      z1_axis.abs_mode(angle, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(z1_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      uint8_t payload2[8] = {0};  // Initialize buffer for CAN message
      z2_axis.abs_mode(angle, payload2);  // Generate the CAN message

      if (CAN1.sendMsgBuf(z2_axis.motor_id, 0, 8, payload2) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      uint8_t payload3[8] = {0};  // Initialize buffer for CAN message
      z3_axis.abs_mode(angle, payload3);  // Generate the CAN message

      if (CAN1.sendMsgBuf(z3_axis.motor_id, 0, 8, payload3) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      

      // Wait for reply and get status
      uint8_t status = waitForCanReplyMultiple(z1_axis.motor_id, z2_axis.motor_id, z3_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterZ();
      }

      // Send response with status
      byte statusResponse[] = {0x06, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Get X movement counter
    case 0x08:
    {
      Serial.println("Case 0x08: Reading X movement counter");
      byte counter_high = (actuatorMoveCounterX >> 8) & 0xFF;
      byte counter_low = actuatorMoveCounterX & 0xFF;
      byte response[] = {0x08, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x09 ***************************** //
    // Reset X movement counter
    case 0x09:
    {
      Serial.println("Case 0x09: Resetting X movement counter");
      actuatorMoveCounterX = 0;
      saveActuatorCounterX();
      byte response[] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0A ***************************** //
    // Get Z movement counter
    case 0x0A:
    {
      Serial.println("Case 0x0A: Reading Z movement counter");
      byte counter_high = (actuatorMoveCounterZ >> 8) & 0xFF;
      byte counter_low = actuatorMoveCounterZ & 0xFF;
      byte response[] = {0x0A, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0B ***************************** //
    // Reset Z movement counter
    case 0x0B:
    {
      Serial.println("Case 0x0B: Resetting Z movement counter");
      actuatorMoveCounterZ = 0;
      saveActuatorCounterZ();
      byte response[] = {0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;


    // ***************************** CASE 0x0C ***************************** //
    //Move to Desire Position
    case 0x0C:
    {
      Serial.println("Case 0x0C: Moving X actuator established position");

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      int position_id = instruction.data[1];
      int position_list[] = {ACTUATOR_LUB_NOZZLE, ACTUATOR_INS_NOZZLE, ACTUATOR_LUB_JOINT, ACTUATOR_INS_JOINT};

      x_axis.abs_mode(position_list[position_id], payload);  // Generate the CAN message 
      
     
      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x0C, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounterX();
      }

      // Send response with status
      byte statusResponse[] = {0x0C, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;
    
    
    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
      Serial.println("Case 0xFF: Powering off - Moving all to home position");
      
      // Move linear actuator to home position (assuming position 0 is home)

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode(0, payload);  // Generate the CAN message
      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      // Send response with status
      byte statusResponse[] = {0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
      
      delay(1000); // Wait for movements to complete
      byte finalResponse[] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(finalResponse);
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
uint8_t waitForCanReply(uint16_t expectedId) {
  memset(replyData, 0, sizeof(replyData)); // Clear the buffer before waiting
  unsigned long startTime = millis();
  const unsigned long timeout = 6000;  // 1 second timeout
  
  while (millis() - startTime < timeout) {
    if (CAN1.checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      byte len = 0;
      CAN1.readMsgBuf(&canId, &len, replyData);
      
      if (canId == expectedId and (replyData[1] == 0x02 || replyData[1] == 0x03)) {
        return 0x01;  // Success
      }
    }
    vTaskDelay(1);  // Small delay to prevent busy-waiting
  }
  return 0x02;  // Timeout
}

// Helper function to wait for multiple CAN replies
uint8_t waitForCanReplyMultiple(uint16_t expectedId1, uint16_t expectedId2, uint16_t expectedId3)
{

  bool received1 = false;
  bool received2 = false;
  bool received3 = false;

  memset(replyData, 0, sizeof(replyData)); // Clear the buffer before waiting
  unsigned long startTime = millis();
  const unsigned long timeout = 6000;  // 6 second timeout
  
  while (millis() - startTime < timeout)
  {
    if (CAN1.checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      byte len = 0;
      CAN1.readMsgBuf(&canId, &len, replyData);

      // replyData[1] = 0x02;
      
      if (canId == expectedId1 && (replyData[1] == 0x02 || replyData[1] == 0x03))
      {
        received1 = true;
      }
      else if (canId == expectedId1 && replyData[1] == 0x00)
      {
        return 0x02;  // FAIL
      }
      
      else if (canId == expectedId2 && (replyData[1] == 0x02 || replyData[1] == 0x03))
      {
        received2 = true;
      }
      else if (canId == expectedId2 && replyData[1] == 0x00)
      {
        return 0x02;  // FAIL
      }
      else if (canId == expectedId3 && (replyData[1] == 0x02 || replyData[1] == 0x03))
      {
        received3 = true;
      }
      else if (canId == expectedId3 && replyData[1] == 0x00)
      {
        return 0x02;  // FAIL
      }
      
      if (received1 && received2 && received3)
      {
        return 0x01;  // Success
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