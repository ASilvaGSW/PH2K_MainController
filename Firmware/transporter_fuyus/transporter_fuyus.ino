// CAN Command Cases (input/output summary):
// 0x01: Reset microcontroller
//   IN: None | OUT: [0x01, 0x01, ...], then MCU restarts
// 0x02: Send Heartbeat
//   IN: None | OUT: [0x02, 0x01, ...]
// 0x04: Move X Axis (dual motors)
//   IN: [x_H, x_L, orientation] | OUT: [0x04, status, ...]
// 0x05: Get Actuator Counter
//   IN: [actuator_id] | OUT: [0x05, counter_H, counter_L, ...] (ID: 1=X_axis, 2=stepper1, 3=stepper2, 4=stepper3)
// 0x06: Reset Actuator Counter
//   IN: [actuator_id] | OUT: [0x06, 0x01, ...] (ID: 1=X_axis, 2=stepper1, 3=stepper2, 4=stepper3)
// 0x07: Move All Steppers to Same Position
//   IN: [pos_H, pos_L, direction] | OUT: [0x07, status, ...]
// 0x11: Home X Axis
//   IN: None | OUT: [0x11, status, ...]
// 0xFF: Power off - Move X axis to home position
//   IN: None | OUT: None (moves X axis to home)

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
#include <AccelStepper.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 20 // 4 bytes for each counter (X actuator + 3 steppers)
#define FUYU_X_COUNTER_ADDR 0  // Address in EEPROM to store the X actuator counter
#define STEPPER1_COUNTER_ADDR 4 // Address for stepper 1 counter
#define STEPPER2_COUNTER_ADDR 8 // Address for stepper 2 counter
#define STEPPER3_COUNTER_ADDR 12 // Address for stepper 3 counter

// Global variables
unsigned long counter_fuyuX = 0;

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

// Stepper Motor Pins
// Stepper 1
#define STEPPER1_STEP_PIN 13
#define STEPPER1_DIR_PIN 14
#define STEPPER1_ENABLE_PIN 21

// Stepper 2
#define STEPPER2_STEP_PIN 16
#define STEPPER2_DIR_PIN 17
#define STEPPER2_ENABLE_PIN 22

// Stepper 3
#define STEPPER3_STEP_PIN 27
#define STEPPER3_DIR_PIN 32
#define STEPPER3_ENABLE_PIN 33

// Stepper Motor Configuration
#define STEPS_PER_REVOLUTION 200  // Number of steps per revolution
#define MAX_SPEED 1500000  // Maximum speed in steps per second
#define ACCELERATION 1500000  // Acceleration in steps per second squared

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x021
#define RESPONSE_CAN_ID 0x421

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Instance of LinearActuator for X axis (dual motors)
LinearActuator fuyuX(0x001);   // X axis actuator 1
LinearActuator fuyuX2(0x002);  // X axis actuator 2

// Stepper motor instances
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, STEPPER3_STEP_PIN, STEPPER3_DIR_PIN);

// Stepper movement counters
unsigned long stepper1MoveCounter = 0;
unsigned long stepper2MoveCounter = 0;
unsigned long stepper3MoveCounter = 0;

void loadConfigValues()
{
  counter_fuyuX = EEPROM.readULong(FUYU_X_COUNTER_ADDR);
  stepper1MoveCounter = EEPROM.readULong(STEPPER1_COUNTER_ADDR);
  stepper2MoveCounter = EEPROM.readULong(STEPPER2_COUNTER_ADDR);
  stepper3MoveCounter = EEPROM.readULong(STEPPER3_COUNTER_ADDR);
}

// Queue to store instructions received from the main CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Function declarations
void process_instruction(CanFrame instruction);
uint8_t waitForCanReply(uint16_t expectedId);
uint8_t waitForCanReplyMultiple(uint16_t expectedId1, uint16_t expectedId2);
void send_twai_response(const byte response_data[8]);
void flushCanBuffer();
uint8_t moveIndividualActuator(uint16_t motor_id, int16_t angle, uint8_t orientation);

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
void saveFuyuXCounter() { saveCounter("Fuyu X", FUYU_X_COUNTER_ADDR, counter_fuyuX); }

// Stepper counter save functions
void saveStepper1Counter()
{
  stepper1MoveCounter++;
  EEPROM.writeULong(STEPPER1_COUNTER_ADDR, stepper1MoveCounter);
  EEPROM.commit();
  Serial.print("Stepper 1 counter saved: ");
  Serial.println(stepper1MoveCounter);
}

void saveStepper2Counter()
{
  stepper2MoveCounter++;
  EEPROM.writeULong(STEPPER2_COUNTER_ADDR, stepper2MoveCounter);
  EEPROM.commit();
  Serial.print("Stepper 2 counter saved: ");
  Serial.println(stepper2MoveCounter);
}

void saveStepper3Counter()
{
  stepper3MoveCounter++;
  EEPROM.writeULong(STEPPER3_COUNTER_ADDR, stepper3MoveCounter);
  EEPROM.commit();
  Serial.print("Stepper 3 counter saved: ");
  Serial.println(stepper3MoveCounter);
}

void incrementStepperCounters()
{
  saveStepper1Counter();
  saveStepper2Counter();
  saveStepper3Counter();
}

void SaveActuatorCounter(int id)
{
  switch (id)
  {
    case 1: saveFuyuXCounter(); break;
    default: break;
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing Transporter Fuyus dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load persistent config values
  loadConfigValues();

  Serial.print("Loaded Fuyu X counter from EEPROM: "); Serial.println(counter_fuyuX);
  Serial.print("Loaded Stepper 1 counter from EEPROM: "); Serial.println(stepper1MoveCounter);
  Serial.print("Loaded Stepper 2 counter from EEPROM: "); Serial.println(stepper2MoveCounter);
  Serial.print("Loaded Stepper 3 counter from EEPROM: "); Serial.println(stepper3MoveCounter);

  // Initialize stepper motors
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setAcceleration(ACCELERATION);
  stepper1.setEnablePin(STEPPER1_ENABLE_PIN);
  stepper1.setPinsInverted(false, false, true);  // Invert enable pin
  stepper1.enableOutputs();
  stepper1.moveTo(0);

  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setAcceleration(ACCELERATION);
  stepper2.setEnablePin(STEPPER2_ENABLE_PIN);
  stepper2.setPinsInverted(false, false, true);  // Invert enable pin
  stepper2.enableOutputs();
  stepper2.moveTo(0);

  stepper3.setMaxSpeed(MAX_SPEED);
  stepper3.setAcceleration(ACCELERATION);
  stepper3.setEnablePin(STEPPER3_ENABLE_PIN);
  stepper3.setPinsInverted(false, false, true);  // Invert enable pin
  stepper3.enableOutputs();
  stepper3.moveTo(0);

  // Set enable pins as outputs and enable steppers
  pinMode(STEPPER1_ENABLE_PIN, OUTPUT);
  pinMode(STEPPER2_ENABLE_PIN, OUTPUT);
  pinMode(STEPPER3_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER1_ENABLE_PIN, LOW);
  digitalWrite(STEPPER2_ENABLE_PIN, LOW);
  digitalWrite(STEPPER3_ENABLE_PIN, LOW);

  Serial.println("Stepper motors initialized successfully");
  
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

  Serial.println("Transporter Fuyus CAN system ready. Main loop running on core 1.");

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
      Serial.println("Case 0x01: Reset microcontroller");
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



    // ***************************** CASE 0x04 ***************************** //
    // Move X Axis
    case 0x04:
    {
      Serial.println("Case 0x04: Move X Axis");

      uint16_t x_angle = instruction.data[1] << 8 | instruction.data[2];
      uint8_t orientation = instruction.data[3];
      
      int ori1 = 0;
      
      if (orientation & 0x01) ori1 = 1;

      // Move X axis (dual motors)
      uint8_t payload[8] = {0};
      uint8_t payload2[8] = {0};
      
      fuyuX.abs_mode(x_angle * (ori1 ? -1 : 1), payload);
      fuyuX2.abs_mode(x_angle * (ori1 ? -1 : 1), payload2);
      
      if (CAN1.sendMsgBuf(fuyuX.motor_id, 0, 8, payload) != CAN_OK ||
          CAN1.sendMsgBuf(fuyuX2.motor_id, 0, 8, payload2) != CAN_OK) {
        byte errorResponse[] = {0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }
      
      uint8_t status = waitForCanReplyMultiple(fuyuX.motor_id, fuyuX2.motor_id);
      if (status != 0x01)
      {
        byte errorResponse[] = {0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(errorResponse);
        break;
      }
      
      SaveActuatorCounter(1);

      byte statusResponse[] = {0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
        case 1: counter = counter_fuyuX; break;
        case 2: counter = stepper1MoveCounter; break;
        case 3: counter = stepper2MoveCounter; break;
        case 4: counter = stepper3MoveCounter; break;
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
        case 1: 
          counter_fuyuX = 0;
          EEPROM.writeULong(FUYU_X_COUNTER_ADDR, counter_fuyuX);
          EEPROM.commit();
          break;
        case 2: 
          stepper1MoveCounter = 0;
          EEPROM.writeULong(STEPPER1_COUNTER_ADDR, stepper1MoveCounter);
          EEPROM.commit();
          break;
        case 3: 
          stepper2MoveCounter = 0;
          EEPROM.writeULong(STEPPER2_COUNTER_ADDR, stepper2MoveCounter);
          EEPROM.commit();
          break;
        case 4: 
          stepper3MoveCounter = 0;
          EEPROM.writeULong(STEPPER3_COUNTER_ADDR, stepper3MoveCounter);
          EEPROM.commit();
          break;
        default: break;
      }
      
      byte statusResponse[] = {0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x07 ***************************** //
    // Move All Steppers to Same Position
    case 0x07:
    {
      Serial.println("Case 0x07: Move All Steppers to Same Position");
      
      // Extract target position from the CAN message (only use first position for all steppers)
      long targetPosition = ((uint8_t)instruction.data[1] << 8) | ((uint8_t)instruction.data[2]);
      
      // Get direction from byte 3 (0 = negative, 1 = positive)
      int direction = instruction.data[3];
      
      // Apply direction
      if (direction == 1) {
        targetPosition = -targetPosition;
      }
      
      Serial.print("Moving all steppers to position: ");
      Serial.print(targetPosition);
      Serial.print(", Direction: ");
      Serial.println((direction > 0) ? "Forward" : "Backward");
      
      // Set same target position for all steppers
      stepper1.moveTo(targetPosition);
      stepper2.moveTo(targetPosition);
      stepper3.moveTo(targetPosition);
      
      // Move all steppers to their positions simultaneously
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      }
      
      delay(200);
      
      // Increment all stepper counters
      incrementStepperCounters();
      
      // Send response after movement is complete
      byte response[] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x11 ***************************** //
    // Home X Axis
    case 0x11:
    {
      Serial.println("Case 0x11: Home X Axis");
      
      uint8_t payload[2] = {0};
      uint8_t payload2[2] = {0};
      uint8_t status = 0x01;
      
      // Generate home commands for X axis motors
      fuyuX.go_home(payload);
      fuyuX2.go_home(payload2);
      
      // Send commands to X axis actuators
      if (CAN1.sendMsgBuf(fuyuX.motor_id, 0, 2, payload) != CAN_OK ||
          CAN1.sendMsgBuf(fuyuX2.motor_id, 0, 2, payload2) != CAN_OK) {
        status = 0x04; // NO LOCAL NETWORK
      } else {
        // Wait for X axis replies
        uint8_t statusX = waitForCanReplyMultiple(fuyuX.motor_id, fuyuX2.motor_id);
        
        if (statusX != 0x01) {
          status = statusX;
        }
      }
      
      byte statusResponse[] = {0x11, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
      Serial.println("Case 0xFF: Power off - Moving all to home");
      
      uint8_t payload[2] = {0};
      uint8_t payload2[2] = {0};
      
      // Generate home commands for X axis motors
      fuyuX.go_home(payload);
      fuyuX2.go_home(payload2);
      
      // Send commands to X axis actuators
      CAN1.sendMsgBuf(fuyuX.motor_id, 0, 2, payload);
      CAN1.sendMsgBuf(fuyuX2.motor_id, 0, 2, payload2);
      
      // Home all steppers
      stepper1.moveTo(0);
      stepper2.moveTo(0);
      stepper3.moveTo(0);
      
      // Move all steppers to home simultaneously
      while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
      }
      
      Serial.println("All steppers homed");
      
      // No response for power off command
    }
    break;

    default:
      Serial.println("Unknown command");
      break;
  }
}



uint8_t waitForCanReply(uint16_t expectedId)
{
  unsigned long startTime = millis();
  const unsigned long timeout = 5000; // 5 second timeout
  
  while (millis() - startTime < timeout) {
    if (CAN1.checkReceive() == CAN_MSGAVAIL) {
      unsigned char len = 0;
      unsigned char buf[8];
      unsigned long canId;
      
      CAN1.readMsgBuf(&canId, &len, buf);
      
      if (canId == expectedId) {
        Serial.print("Received reply from ID: 0x");
        Serial.println(canId, HEX);
        if (buf[0] == 0x01) return 0x01; // SUCCESS
        if (buf[0] == 0x02) return 0x02; // FAIL
        if (buf[0] == 0x03) return 0x03; // TIMEOUT
        return 0x01; // Default OK for any other response
      }
    }
    delay(10);
  }
  
  Serial.println("Timeout waiting for CAN reply");
  return 0x03; // TIMEOUT
}

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
    delay(10);
  }
  return 0x03;  // Timeout
}



void send_twai_response(const byte response_data[8])
{
  CanFrame txFrame;
  txFrame.identifier = RESPONSE_CAN_ID;
  txFrame.extd = 0;
  txFrame.data_length_code = 8;
  
  for (int i = 0; i < 8; i++) {
    txFrame.data[i] = response_data[i];
  }
  
  if (ESP32Can.writeFrame(txFrame)) {
    Serial.print("Response sent on TWAI: ");
    for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      Serial.print(response_data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Error sending response on TWAI");
  }
}

void flushCanBuffer()
{
  while (CAN1.checkReceive() == CAN_MSGAVAIL) {
    unsigned char len = 0;
    unsigned char buf[8];
    unsigned long canId;
    CAN1.readMsgBuf(&canId, &len, buf);
    Serial.print("Flushed message from ID: 0x");
    Serial.println(canId, HEX);
  }
}