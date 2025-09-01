// CAN Command Cases (input/output summary):
// 0x01: Reset microcontroller
//   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
// 0x02: Send Heartbeat
//   IN: None | OUT: [0x02, 0x01, ...]
// 0x03: Move Individual Actuator
//   IN: [actuator_id, angle_H, angle_L, orientation] | OUT: [0x03, status, ...]
// 0x04: Move Gantry XYZ
//   IN: [x_H, x_L, y_H, y_L, z_H, z_L, orientation] | OUT: [0x04, status, ...]
// 0x05: Move Elevator Z
//   IN: [angle_H, angle_L, orientation] | OUT: [0x05, status, ...]
// 0x06: Move Gripper
//   IN: [action: 0x01=open, 0x02=close] | OUT: [0x06, 0x01, ...]
// 0x07: Set gripper force
//   IN: [force (4-7)] | OUT: [0x07, 0x01, ...] if valid
// 0x08: Get gripper force
//   IN: None | OUT: [0x08, gripperForce, ...]
// 0x09: Get gripper counter
//   IN: None | OUT: [0x09, counter_H, counter_L, ...]
// 0x0A: Reset gripper counter
//   IN: None | OUT: [0x0A, 0x01, ...]
// 0x0B: Get Actuator Counter
//   IN: [actuator_id] | OUT: [0x0B, counter_H, counter_L, ...]
// 0x0C: Reset Actuator Counter
//   IN: [actuator_id] | OUT: [0x0C, 0x01, ...]
// 0x10: Home Individual Actuator
//   IN: [actuator_id] | OUT: [0x10, status, ...]
// 0x11: Home All Gantry Axes
//   IN: None | OUT: [0x11, status, ...]
// 0x12: Home All Axes (Gantry + Elevator)
//   IN: None | OUT: [0x12, status, ...]
// 0x13: Move Elevator in Speed Mode until IR Sensor
//   IN: [direction, speed_H, speed_L, acceleration] | OUT: [0x13, status, ...]
// 0xFF: Power off - Move all to home position
  //   IN: None | OUT: None (moves all to home)


/*
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
#include "src/gripper_digital.h"
#include <ESP32Servo.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 28 // 4 bytes for each counter (servos + actuator)
#define GANTRY_COUNTER_X_ADDR 4  // Address in EEPROM to store the actuator counter
#define GANTRY_COUNTER_Y_ADDR 8 // Address in EEPROM to store the actuator counter
#define GANTRY_COUNTER_Z_ADDR 12 // Address in EEPROM to store the actuator counter
#define ELEVATOR_COUNTER_Z_ADDR 16 // Address in EEPROM to store the actuator counter
#define GRIPPER_COUNTER_ADDR 20 // Address in EEPROM to store the gripper counter
#define GRIPPER_FORCE_ADDR 24 // Address in EEPROM to store the gripper force
// Global variables
unsigned long counter_gantryX = 0;
unsigned long counter_gantryY = 0;
unsigned long counter_gantryZ = 0;
unsigned long counter_elevatorZ = 0;
unsigned long counter_gripper = 0;
int gripperForce = 4;

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

// IR Sensor Pin
#define IR_SENSOR_PIN 21  // GPIO21 - IR Sensor input

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x189
#define RESPONSE_CAN_ID 0x589

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Instance of LinearActuator
LinearActuator gantryX(0x001);
LinearActuator gantryY(0x002);
LinearActuator gantryY2(0x003);
LinearActuator gantryZ(0x004);
LinearActuator elevatorZ(0x005);

GripperDigital gripper(14, 16, 17);

void loadConfigValues()
{
  counter_gantryX = EEPROM.readULong(GANTRY_COUNTER_X_ADDR);
  counter_gantryY = EEPROM.readULong(GANTRY_COUNTER_Y_ADDR);
  counter_gantryZ = EEPROM.readULong(GANTRY_COUNTER_Z_ADDR);
  counter_elevatorZ = EEPROM.readULong(ELEVATOR_COUNTER_Z_ADDR);
  counter_gripper = EEPROM.readULong(GRIPPER_COUNTER_ADDR);
  gripperForce = EEPROM.readUShort(GRIPPER_FORCE_ADDR);
}

// Queue to store instructions received from the main CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Function declarations
void process_instruction(CanFrame instruction);
uint8_t waitForCanReply(uint16_t expectedId);
void send_twai_response(const byte response_data[8]);


// Generic function to save counter to EEPROM
void saveCounter(const char* name, uint32_t address, unsigned long counter)
{
  counter++;
  EEPROM.writeULong(address, counter);
  EEPROM.commit();
  Serial.print(name);
  Serial.print(" counter saved: ");
  Serial.println(counter);
}

// Wrapper functions for each counter
void saveGantryXCounter() { saveCounter("Gantry X", GANTRY_COUNTER_X_ADDR, counter_gantryX); }
void saveGantryYCounter() { saveCounter("Gantry Y", GANTRY_COUNTER_Y_ADDR, counter_gantryY); }
void saveGantryZCounter() { saveCounter("Gantry Z", GANTRY_COUNTER_Z_ADDR, counter_gantryZ); }
void saveElevatorZCounter() { saveCounter("Elevator Z", ELEVATOR_COUNTER_Z_ADDR, counter_elevatorZ); }
void saveGripperCounter() { saveCounter("Gripper", GRIPPER_COUNTER_ADDR, counter_gripper); }

void saveGripperForce()
{ 
  EEPROM.writeUShort(GRIPPER_FORCE_ADDR, gripperForce);
  EEPROM.commit();
  Serial.print("Gripper Force saved: ");
  Serial.println(gripperForce);
}

void SaveActuatorCounter(int id)
{
  switch (id)
  {
    case 1: saveGantryXCounter(); break;
    case 2: saveGantryYCounter(); break;
    case 3: saveGantryZCounter(); break;
    case 4: saveElevatorZCounter(); break;
    default: break;
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load persistent config values
  loadConfigValues();

  Serial.print("Loaded Gantry X counter from EEPROM: "); Serial.println(counter_gantryX);
  Serial.print("Loaded Gantry Y counter from EEPROM: "); Serial.println(counter_gantryY);
  Serial.print("Loaded Gantry Z counter from EEPROM: "); Serial.println(counter_gantryZ);
  Serial.print("Loaded Elevator Z counter from EEPROM: "); Serial.println(counter_elevatorZ);
  Serial.print("Loaded Gripper counter from EEPROM: "); Serial.println(counter_gripper);
  
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

  gripper.setFuerza(gripperForce);

  // Initialize IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);
  Serial.println("IR sensor on pin 21 initialized");

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

  flushCanBuffer();

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
    // Move Individual Actuator
    case 0x03:
    { 
      Serial.println("Case 0x03: Move Individual Axis");

      uint8_t actuator_id = instruction.data[1];
      int16_t angle = instruction.data[2] << 8 | instruction.data[3];
      uint8_t orientation = instruction.data[4];

      uint8_t status = moveIndividualActuator(actuator_id, angle, orientation);

      // Send response with status
      byte statusResponse[] = {0x03, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Move Gantry XYZ
    case 0x04:
    {
      Serial.println("Case 0x04: Move Gantry XYZ");

      uint16_t x_angle = instruction.data[1] << 8 | instruction.data[2];
      uint16_t y_angle = instruction.data[3] << 8 | instruction.data[4];
      uint16_t z_angle = instruction.data[5] << 8 | instruction.data[6];
      uint8_t orientation = instruction.data[7];
      
      int ori1 = 0;
      int ori2 = 0;
      int ori3 = 0;
      
      if (orientation & 0x04) ori1 = 1;
      if (orientation & 0x02) ori2 = 1;
      if (orientation & 0x01) ori3 = 1;

      uint8_t status = moveIndividualActuator(gantryX.motor_id, x_angle, ori1);
      if (status != 0x01)
      {
        byte errorResponse[] = {0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      status = moveIndividualActuator(gantryY.motor_id, y_angle, ori2);
      if (status != 0x01)
      {
        byte errorResponse[] = {0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      status = moveIndividualActuator(gantryZ.motor_id, z_angle, ori3);
      if (status != 0x01)
      {
        byte errorResponse[] = {0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      byte statusResponse[] = {0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x05 ***************************** //
    // Move Elevator Z
    case 0x05:
    {
      Serial.println("Case 0x05: Move Elevator Z");

      uint16_t angle = instruction.data[1] << 8 | instruction.data[2];
      uint8_t orientation = instruction.data[3];
      

      uint8_t status = moveIndividualActuator(elevatorZ.motor_id, angle, orientation);
      
      byte statusResponse[] = {0x05, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;
    
    // ***************************** CASE 0x06 ***************************** //
    // Move Gripper
    case 0x06:
    {
      int action = instruction.data[1];
      
      if (action == 0x01)
      {
        gripper.open();
      }
      else if (action == 0x02)
      {
        gripper.close();
      }

      saveGripperCounter();

      delay(500);
      byte statusResponse[] = {0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x07 ***************************** //
    // Set gripper force
    case 0x07:
    {
      Serial.println("Case 0x07: Set gripper force");
      
      int newGripperForce = instruction.data[1];

      if (newGripperForce >= 4 || newGripperForce <= 7)
      {
        gripperForce = newGripperForce;      
        gripper.setFuerza(gripperForce);
        saveGripperForce();
        byte statusResponse[] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(statusResponse);
      }
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Get gripper force
    case 0x08:
    {
      Serial.println("Case 0x08: Get gripper force");
      byte response[] = {0x08, gripperForce, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;
    
    // ***************************** CASE 0x09 ***************************** //
    // Get gripper counter
    case 0x09:
    {
      Serial.println("Case 0x09: Get gripper counter");

      uint8_t counter_high = counter_gripper >> 8;
      uint8_t counter_low = counter_gripper & 0xFF;

      byte response[] = {0x09, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0A ***************************** //
    // Reset gripper counter
    case 0x0A:
    {
      Serial.println("Case 0x0A: Reset gripper counter");
      counter_gripper = 0;
      saveGripperCounter();
      byte statusResponse[] = {0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x0B ***************************** //
    // Get Actuator Counter
    case 0x0B:
    {
      Serial.println("Case 0x0B: Get Actuator Counter");

      uint8_t actuator_id = instruction.data[1];

      uint16_t counter = 0;

      switch (actuator_id)
      {
        case 1: counter = counter_gantryX; break;
        case 2: counter = counter_gantryY; break;
        case 3: counter = counter_gantryZ; break;
        case 4: counter = counter_elevatorZ; break;
        default: break;
      }

      uint8_t counter_high = counter >> 8;
      uint8_t counter_low = counter & 0xFF;

      byte response[] = {0x0B, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0C ***************************** //
    // Reset Actuator Counter
    case 0x0C:
    {
      Serial.println("Case 0x0C: Reset Actuator Counter");

      uint8_t actuator_id = instruction.data[1];

      switch (actuator_id)
      {
        case 1: counter_gantryX = 0; break;
        case 2: counter_gantryY = 0; break;
        case 3: counter_gantryZ = 0; break;
        case 4: counter_elevatorZ = 0; break;
        default: break;
      }

      SaveActuatorCounter(actuator_id);
      
      byte statusResponse[] = {0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x10 ***************************** //
    // Home Individual Actuator
    case 0x10:
    {
      Serial.println("Case 0x10: Home Individual Actuator");
      
      uint8_t actuator_id = instruction.data[1];
      uint8_t payload[2] = {0};
      uint8_t payload2[2] = {0};
      uint8_t status = 0x01;
      
      switch (actuator_id)
      {
        case 1: // Gantry X
          gantryX.go_home(payload);
          if (CAN1.sendMsgBuf(gantryX.motor_id, 0, 2, payload) != CAN_OK) {
            status = 0x04; // NO LOCAL NETWORK
          } else {
            status = waitForCanReply(gantryX.motor_id);
          }
          break;
          
        case 2: // Gantry Y (dual axis)
          gantryY.go_home(payload);
          gantryY2.go_home(payload2);
          if (CAN1.sendMsgBuf(gantryY.motor_id, 0, 2, payload) != CAN_OK ||
              CAN1.sendMsgBuf(gantryY2.motor_id, 0, 2, payload2) != CAN_OK) {
            status = 0x04; // NO LOCAL NETWORK
          } else {
            status = waitForCanReplyMultiple(gantryY.motor_id, gantryY2.motor_id);
          }
          break;
          
        case 3: // Gantry Z
          gantryZ.go_home(payload);
          if (CAN1.sendMsgBuf(gantryZ.motor_id, 0, 2, payload) != CAN_OK) {
            status = 0x04; // NO LOCAL NETWORK
          } else {
            status = waitForCanReply(gantryZ.motor_id);
          }
          break;
          
        case 4: // Elevator Z
          elevatorZ.go_home(payload);
          if (CAN1.sendMsgBuf(elevatorZ.motor_id, 0, 2, payload) != CAN_OK) {
            status = 0x04; // NO LOCAL NETWORK
          } else {
            status = waitForCanReply(elevatorZ.motor_id);
          }
          break;
          
        default:
          status = 0x02; // FAIL - Invalid actuator ID
          break;
      }
      
      byte statusResponse[] = {0x10, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x11 ***************************** //
    // Home All Gantry Axes
    case 0x11:
    {
      Serial.println("Case 0x11: Home All Gantry Axes");
      
      uint8_t payload[2] = {0};
      uint8_t payload2[2] = {0};
      uint8_t payload3[2] = {0};
      uint8_t payload4[2] = {0};
      uint8_t status = 0x01;
      
      // Generate home commands for all gantry axes
      gantryX.go_home(payload);
      gantryY.go_home(payload2);
      gantryY2.go_home(payload3);
      gantryZ.go_home(payload4);
      
      // Send commands to all gantry actuators
      if (CAN1.sendMsgBuf(gantryX.motor_id, 0, 2, payload) != CAN_OK ||
          CAN1.sendMsgBuf(gantryY.motor_id, 0, 2, payload2) != CAN_OK ||
          CAN1.sendMsgBuf(gantryY2.motor_id, 0, 2, payload3) != CAN_OK ||
          CAN1.sendMsgBuf(gantryZ.motor_id, 0, 2, payload4) != CAN_OK) {
        status = 0x04; // NO LOCAL NETWORK
      } else {
        // Wait for all replies (simplified - checking key actuators)
        uint8_t statusX = waitForCanReply(gantryX.motor_id);
        uint8_t statusY = waitForCanReplyMultiple(gantryY.motor_id, gantryY2.motor_id);
        uint8_t statusZ = waitForCanReply(gantryZ.motor_id);
        
        if (statusX != 0x01 || statusY != 0x01 || statusZ != 0x01) {
          status = (statusX == 0x03 || statusY == 0x03 || statusZ == 0x03) ? 0x03 : 0x02;
        }
      }
      
      byte statusResponse[] = {0x11, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x12 ***************************** //
    // Home All Axes (Gantry + Elevator)
    case 0x12:
    {
      Serial.println("Case 0x12: Home All Axes");
      
      uint8_t payload[2] = {0};
      uint8_t payload2[2] = {0};
      uint8_t payload3[2] = {0};
      uint8_t payload4[2] = {0};
      uint8_t payload5[2] = {0};
      uint8_t status = 0x01;
      
      // Generate home commands for all axes
      gantryX.go_home(payload);
      gantryY.go_home(payload2);
      gantryY2.go_home(payload3);
      gantryZ.go_home(payload4);
      elevatorZ.go_home(payload5);
      
      // Send commands to all actuators
      if (CAN1.sendMsgBuf(gantryX.motor_id, 0, 2, payload) != CAN_OK ||
          CAN1.sendMsgBuf(gantryY.motor_id, 0, 2, payload2) != CAN_OK ||
          CAN1.sendMsgBuf(gantryY2.motor_id, 0, 2, payload3) != CAN_OK ||
          CAN1.sendMsgBuf(gantryZ.motor_id, 0, 2, payload4) != CAN_OK ||
          CAN1.sendMsgBuf(elevatorZ.motor_id, 0, 2, payload5) != CAN_OK) {
        status = 0x04; // NO LOCAL NETWORK
      } else {
        // Wait for all replies
        uint8_t statusX = waitForCanReply(gantryX.motor_id);
        uint8_t statusY = waitForCanReplyMultiple(gantryY.motor_id, gantryY2.motor_id);
        uint8_t statusZ = waitForCanReply(gantryZ.motor_id);
        uint8_t statusE = waitForCanReply(elevatorZ.motor_id);
        
        if (statusX != 0x01 || statusY != 0x01 || statusZ != 0x01 || statusE != 0x01) {
          status = (statusX == 0x03 || statusY == 0x03 || statusZ == 0x03 || statusE == 0x03) ? 0x03 : 0x02;
        }
      }
      
      byte statusResponse[] = {0x12, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
        Serial.println("Case 0xFF: Powering off - Moving all to home position");
        
    }
    break;

    // ***************************** CASE 0x13 ***************************** //
    // Move Elevator in Speed Mode until IR Sensor is activated
    case 0x13:
    {
      Serial.println("Case 0x13: Move Elevator in Speed Mode until IR Sensor");
      
      uint8_t direction = instruction.data[1];  // 0 = down, 1 = up
      uint16_t speed = instruction.data[2] << 8 | instruction.data[3];  // Speed value
      uint8_t acceleration = instruction.data[4];  // Acceleration value
      
      uint8_t payload[5] = {0};
      uint8_t status = 0x01;
      
      // Start elevator in speed mode
      elevatorZ.speed_mode(direction, speed, acceleration, payload);
      
      if (CAN1.sendMsgBuf(elevatorZ.motor_id, 0, 5, payload) != CAN_OK) {
        status = 0x04; // NO LOCAL NETWORK
      } else {
        // Monitor IR sensor while moving
        unsigned long startTime = millis();
        const unsigned long maxTimeout = 30000; // 30 second maximum timeout
        bool sensorActivated = false;
        
        while (millis() - startTime < maxTimeout && !sensorActivated) {
          // Check IR sensor status
          if (digitalRead(IR_SENSOR_PIN) == LOW) { // Assuming LOW means object detected
            sensorActivated = true;
            Serial.println("IR sensor activated - stopping elevator");
            
            // Stop the elevator by sending a stop command (speed = 0)
            uint8_t stopPayload[8] = {0};
            elevatorZ.speed_mode(direction, 0, acceleration, stopPayload);
            CAN1.sendMsgBuf(elevatorZ.motor_id, 0, 8, stopPayload);
            
            status = 0x01; // Success
            break;
          }
          
          delay(10); // Small delay to avoid overwhelming the sensor
        }
        
        if (!sensorActivated) {
          // Timeout occurred - stop the elevator
          uint8_t stopPayload[8] = {0};
          elevatorZ.speed_mode(direction, 0, acceleration, stopPayload);
          CAN1.sendMsgBuf(elevatorZ.motor_id, 0, 8, stopPayload);
          status = 0x03; // TIMEOUT
          Serial.println("Timeout - IR sensor not activated");
        }
      }
      
      saveElevatorZCounter();
      
      byte statusResponse[] = {0x13, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
uint8_t waitForCanReply(uint16_t expectedId)
{
  memset(replyData, 0, sizeof(replyData)); // Clear the buffer before waiting
  unsigned long startTime = millis();
  const unsigned long timeout = 10000;  // 1 second timeout
  
  while (millis() - startTime < timeout) {
    if (CAN1.checkReceive() == CAN_MSGAVAIL) {
      unsigned long canId;
      byte len = 0;
      CAN1.readMsgBuf(&canId, &len, replyData);
      
      if (canId == expectedId and (replyData[1] == 0x02 || replyData[1] == 0x03))
      {
        return 0x01;  // Success
      }
      else if (canId == expectedId and replyData[1] == 0x00)
      {
        return 0x02;  // FAIL
      }
    }
    vTaskDelay(1);  // Small delay to prevent busy-waiting
  }
  return 0x03;  // Timeout
}

// Helper function to wait for multiple CAN replies
uint8_t waitForCanReplyMultiple(uint16_t expectedId1, uint16_t expectedId2)
{

  bool received1 = false;
  bool received2 = false;

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
      
      if (received1 && received2)
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

uint8_t moveIndividualActuator(uint8_t actuator_id, int16_t angle, uint8_t orientation)
{
  if (orientation == 1)
  {
    angle = -angle;
  }

  uint8_t payload[8] = {0};  // Initialize buffer for CAN message
  uint8_t payload2[8] = {0};

  bool dual_axis = false;

  if (actuator_id == 1)
  {
    gantryX.abs_mode(angle, payload);  // Generate the CAN message
  }
  else if (actuator_id == 2)
  {
    gantryY.abs_mode(angle, payload);  // Generate the CAN message
    gantryY2.abs_mode(angle, payload2);  // Generate the CAN message
    dual_axis = true;
  }
  else if (actuator_id == 4)
  {
    gantryZ.abs_mode(angle, payload);  // Generate the CAN message
  }
  else if (actuator_id == 5)
  {
    elevatorZ.abs_mode(angle, payload);  // Generate the CAN message
  }


  if (CAN1.sendMsgBuf(actuator_id, 0, 8, payload) != CAN_OK)
  {
    Serial.println("Error sending actuator command");
    return 0x04;
  }

  if(actuator_id == 2)
  {
    if (CAN1.sendMsgBuf(gantryY2.motor_id, 0, 8, payload2) != CAN_OK)
    {
      Serial.println("Error sending actuator command");
      return 0x04;
    }
  }

  uint8_t status = 0;

  // Wait for reply and get status
  if(dual_axis)
  {
    status = waitForCanReplyMultiple(gantryY.motor_id, gantryY2.motor_id);
  }
  else
  {
    status = waitForCanReply(actuator_id);
  }
  

  if (status == 0x01) { SaveActuatorCounter(actuator_id); }

  return status;
}