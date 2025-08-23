/*
 * Status Codes:
 * 0x01 - Success
 * 0x02 - Failure
 * 0x03 - Timeout
 * 0x04 - Network/Communication Error
 * 0xFF - Unknown Command
 *
 * CAN Command Reference (Case -> Action):
 * 0x01 - Reset Micro (System health check)
 * 0x02 - Heartbeat (Verify system responsiveness)
 * 0x03 - Activate Valve 1 (Primary lubrication valve)
 * 0x04 - Activate Valve 2 (Secondary lubrication valve)
 * 0x05 - Move Feeder (Speed mode with direction & acceleration)
 * 0x06 - Move Feeder to Position (Position mode)
 * 0x07 - Move Pre-feeder to Position (Position mode)
 * 0x08 - Get Feeder Position (Current position reading)
 * 0x09 - Get Pre-feeder Position (Current position reading)
 * 0x0A - Read Valve 1 Counter (Usage counter)
 * 0x0B - Read Valve 2 Counter (Usage counter)
 * 0x0C - Reset Valve 1 Counter (Clear usage counter)
 * 0x0D - Reset Valve 2 Counter (Clear usage counter)
 * 0x10 - Read Feeder Counter (Movement counter)
 * 0x11 - Read Pre-feeder Counter (Movement counter)
 * 0x12 - Reset Feeder Counter (Clear movement counter)
 * 0x13 - Move Pre-feeder (Speed mode with direction & acceleration)
 * 0x14 - Conditional Dual Movement (Feeder + conditional pre-feeder based on potentiometer)
 * 0x15 - Check Hose Position (IR break beam sensor status)
 * 0x16 - Check Alcohol Level (Tank level using 2 digital sensors)
 * 0x17 - Open Hose Holder (Servo to 90 degrees)
 * 0x18 - Close Hose Holder (Servo to 0 degrees)
 * 0x19 - Attach Electromagnet (Activate electromagnet)
 * 0x1A - Detach Electromagnet (Deactivate electromagnet)
 * 0x1B - Read Servo Counter (Hose holder servo usage counter)
 * 0x1C - Reset Servo Counter (Clear servo usage counter)
 * 0x1D - Acknowledge Message (Send acknowledgment response)
 * 0xFF - Emergency Stop (Stop all actuators immediately)
 *
 * Note: All commands send acknowledgment message (0xAA + function_code) before execution
 *       when enableAckMessages = true (can be disabled for debugging)
 */


#include <ESP32-TWAI-CAN.hpp>
#include <mcp_can.h>
#include <SPI.h>
#include "src/linear_actuator.h"
#include <ESP32Servo.h>  // For servo control

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage


// EEPROM Configuration
#define EEPROM_SIZE 20  // 4 bytes for each counter (2 actuators + 2 valve counters + 1 servo counter)
#define FEEDER_COUNTER_ADDR 4  // Address in EEPROM to store the feeder counter
#define PRE_FEEDER_COUNTER_ADDR 8 // Address in EEPROM to store the pre-feeder counter
#define VALVE1_COUNTER_ADDR 12 // Address in EEPROM to store the valve 1 counter
#define VALVE2_COUNTER_ADDR 16 // Address in EEPROM to store the valve 2 counter
#define SERVO_COUNTER_ADDR 20 // Address in EEPROM to store the hose holder servo counter

// Global variables 
unsigned long feederMoveCounter = 0;
unsigned long preFeederMoveCounter = 0;
unsigned long valve1Counter = 0;
unsigned long valve2Counter = 0;
unsigned long servoCounter = 0;

// Acknowledgment control - set to false to disable ack messages during debugging
bool enableAckMessages = false;

byte replyData[8];  // Buffer for CAN replies

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 26  // Changed from 15 to 26 to avoid upload issues
#define CAN1_INT 25  // Changed from 2 to 25 to avoid boot issues

// Lubrication feeder system - no gripper needed

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x019
#define RESPONSE_CAN_ID 0x419

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Constants for actuator positions
#define ACTUATOR_HOME_POSITION 0

// Instance of LinearActuator
LinearActuator feeder(0x001);        // Main feeder actuator
LinearActuator pre_feeder(0x002);    // Pre-feeder actuator

// Valve control pins
#define VALVE_1 27  // Pin for valve 1 control
#define VALVE_2 32  // Pin for valve 2 control

// Linear potentiometer (SoftPot) pin
#define LINEAR_POT_PIN 33  // Analog pin for linear potentiometer

// IR Break Beam Sensor pin (Optical Sensor)
#define IR_SENSOR_PIN 16   // Digital pin for IR break beam sensor (hose position detection)

// Alcohol Level Sensors pins
#define LEVEL_SENSOR_1_PIN 39  // Digital pin for alcohol level sensor 1 (2/3 tank level)
#define LEVEL_SENSOR_2_PIN 36  // Digital pin for alcohol level sensor 2 (1/3 tank level)

// Hose Holder Servo pin
#define HOSE_HOLDER_SERVO_PIN 14  // PWM pin for hose holder servo

// Electromagnet control pin
#define ELECTROMAGNET_PIN 17  // Digital pin for electromagnet control via MOSFET

// Digital level sensors - Active HIGH (sensor triggered when liquid present)
// Sensor 1 at 2/3 tank level, Sensor 2 at 1/3 tank level
// Tank states: FULL (both HIGH), MEDIUM (only sensor 2 HIGH), EMPTY (both LOW)

// Servo object for hose holder
Servo hoseHolderServo;


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
void saveFeederCounter()
{
  EEPROM.writeULong(FEEDER_COUNTER_ADDR, feederMoveCounter);
  EEPROM.commit();
  Serial.print("Feeder counter saved: ");
  Serial.println(feederMoveCounter);
}

void savePreFeederCounter()
{
  EEPROM.writeULong(PRE_FEEDER_COUNTER_ADDR, preFeederMoveCounter);
  EEPROM.commit();
  Serial.print("Pre-feeder counter saved: ");
  Serial.println(preFeederMoveCounter);
}

// Function to increment and save the actuator counter
void incrementFeederCounter()
{ 
  feederMoveCounter++;
  saveFeederCounter();
}

void incrementPreFeederCounter() {
  preFeederMoveCounter++;
  savePreFeederCounter();
}

// Valve counter management functions
void saveValve1Counter() {
  EEPROM.put(VALVE1_COUNTER_ADDR, valve1Counter);
  EEPROM.commit();
}

void saveValve2Counter() {
  EEPROM.put(VALVE2_COUNTER_ADDR, valve2Counter);
  EEPROM.commit();
}

void incrementValve1Counter() {
  valve1Counter++;
  saveValve1Counter();
}

void incrementValve2Counter() {
  valve2Counter++;
  saveValve2Counter();
}

// Servo counter management functions
void saveServoCounter() {
  EEPROM.put(SERVO_COUNTER_ADDR, servoCounter);
  EEPROM.commit();
}

void incrementServoCounter() {
  servoCounter++;
  saveServoCounter();
}

// Valve activation function
void activateValve(int valvePin, int duration) {
  digitalWrite(valvePin, HIGH);
  delay(duration);
  digitalWrite(valvePin, LOW);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing lubrication feeder CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  feederMoveCounter = EEPROM.readULong(FEEDER_COUNTER_ADDR);
  preFeederMoveCounter = EEPROM.readULong(PRE_FEEDER_COUNTER_ADDR);
  EEPROM.get(VALVE1_COUNTER_ADDR, valve1Counter);
  EEPROM.get(VALVE2_COUNTER_ADDR, valve2Counter);
  servoCounter = EEPROM.readULong(SERVO_COUNTER_ADDR);
  
  // Initialize valve pins
  pinMode(VALVE_1, OUTPUT);
  pinMode(VALVE_2, OUTPUT);
  digitalWrite(VALVE_1, LOW);
  digitalWrite(VALVE_2, LOW);
  
  // Initialize linear potentiometer pin
  pinMode(LINEAR_POT_PIN, INPUT);
  
  // Initialize IR break beam sensor pin
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
  
  // Initialize alcohol level sensor pins
  pinMode(LEVEL_SENSOR_1_PIN, INPUT);  // Digital input for high level sensor
  pinMode(LEVEL_SENSOR_2_PIN, INPUT);  // Digital input for low level sensor
  
  // Initialize hose holder servo
  hoseHolderServo.attach(HOSE_HOLDER_SERVO_PIN);
  hoseHolderServo.write(0);  // Start at closed position (0 degrees)
  
  // Initialize electromagnet pin
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  digitalWrite(ELECTROMAGNET_PIN, LOW);  // Start with electromagnet off
  
  Serial.print("Loaded feeder move counter from EEPROM: ");
  Serial.println(feederMoveCounter);
  Serial.print("Loaded pre-feeder move counter from EEPROM: ");
  Serial.println(preFeederMoveCounter);
  Serial.print("Valve 1 Counter: "); Serial.println(valve1Counter);
  Serial.print("Valve 2 Counter: "); Serial.println(valve2Counter);
  Serial.print("Servo Counter: "); Serial.println(servoCounter);

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

  flushCanBuffer();

  switch (instruction.data[0])
  {

    // ***************************** CASE 0x01 ***************************** //
    // Reset microcontroller
    case 0x01: 
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
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
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x02: Send Heartbeat");
      byte response[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x03 ***************************** //
    // Activate Valve 1
    case 0x03:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x03: Activating Valve 1");
      
      // Get duration from instruction data (default 1000ms if not specified)
      uint16_t duration = 1000;
      if (instruction.data[1] != 0 || instruction.data[2] != 0) {
        duration = (instruction.data[1] << 8) | instruction.data[2];
      }
      
      // Activate valve 1
      activateValve(VALVE_1, duration);
      incrementValve1Counter();
      
      Serial.print("Valve 1 activated for ");
      Serial.print(duration);
      Serial.println(" ms");
      
      // Send success response
      byte response[] = {0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Activate Valve 2
    case 0x04:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x04: Activating Valve 2");
      
      // Get duration from instruction data (default 1000ms if not specified)
      uint16_t duration = 1000;
      if (instruction.data[1] != 0 || instruction.data[2] != 0) {
        duration = (instruction.data[1] << 8) | instruction.data[2];
      }
      
      // Activate valve 2
      activateValve(VALVE_2, duration);
      incrementValve2Counter();
      
      Serial.print("Valve 2 activated for ");
      Serial.print(duration);
      Serial.println(" ms");
      
      // Send success response
      byte response[] = {0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

     // ***************************** CASE 0x05 ***************************** //
    // Move feeder with speed mode
    case 0x05:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x05: Moving feeder with speed mode");

      uint8_t speed_high = instruction.data[1];
      uint8_t speed_low = instruction.data[2];
      uint8_t direction = instruction.data[3];
      uint8_t acceleration = instruction.data[4];  // Get acceleration from data[4]

      uint16_t speed = (speed_high << 8) | speed_low;
      bool dir = (direction == 1);  // Convert to boolean
      
      // Use default acceleration if not provided
      if (acceleration == 0) {
        acceleration = 50;  // Default acceleration value
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      feeder.speed_mode(dir, speed, acceleration, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(feeder.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending feeder speed command");
        byte errorResponse[] = {0x05, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReplySpeed(feeder.motor_id);

      if (status == 0x01) {
        incrementFeederCounter();
      }   

      // Send response with status
      byte statusResponse[] = {0x05, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x06 ***************************** //
    // Read pre-feeder movement counter
    case 0x06:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x06: Reading pre-feeder movement counter");
      byte counter_high = (preFeederMoveCounter >> 8) & 0xFF;
      byte counter_low = preFeederMoveCounter & 0xFF;
      byte response[] = {0x06, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;
          
    // ***************************** CASE 0x07 ***************************** //
    // Read feeder movement counter
    case 0x07:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x07: Reading feeder movement counter");
      byte counter_high = (feederMoveCounter >> 8) & 0xFF;
      byte counter_low = feederMoveCounter & 0xFF;
      byte response[] = {0x07, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Reset feeder movement counter
    case 0x08:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x08: Resetting feeder movement counter");
      feederMoveCounter = 0;
      saveFeederCounter();
      byte response[] = {0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x09 ***************************** //
    // Reset pre-feeder movement counter
    case 0x09:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x09: Resetting pre-feeder movement counter");
      preFeederMoveCounter = 0;
      savePreFeederCounter();
      byte response[] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0A ***************************** //
    // Read valve 1 counter
    case 0x0A:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x0A: Reading valve 1 counter");
      byte counter_high = (valve1Counter >> 8) & 0xFF;
      byte counter_low = valve1Counter & 0xFF;
      byte response[] = {0x0A, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0B ***************************** //
    // Read valve 2 counter
    case 0x0B:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x0B: Reading valve 2 counter");
      byte counter_high = (valve2Counter >> 8) & 0xFF;
      byte counter_low = valve2Counter & 0xFF;
      byte response[] = {0x0B, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0C ***************************** //
    // Reset valve 1 counter
    case 0x0C:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x0C: Resetting valve 1 counter");
      valve1Counter = 0;
      saveValve1Counter();
      byte response[] = {0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0D ***************************** //
    // Reset valve 2 counter
    case 0x0D:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x0D: Resetting valve 2 counter");
      valve2Counter = 0;
      saveValve2Counter();
      byte response[] = {0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x10 ***************************** //
    // Move feeder to absolute position
    case 0x10:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x10: Moving feeder to absolute position");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      feeder.abs_mode(angle, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(feeder.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending feeder command");
        byte errorResponse[] = {0x10, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(feeder.motor_id);

      if (status == 0x01) {
        incrementFeederCounter();
      }

      // Send response with status
      byte statusResponse[] = {0x10, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x11 ***************************** //
    // Move pre-feeder to absolute position
    case 0x11:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x11: Moving pre-feeder to absolute position");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      pre_feeder.abs_mode(angle, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(pre_feeder.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending pre-feeder command");
        byte errorResponse[] = {0x11, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(pre_feeder.motor_id);

      if (status == 0x01) {
        incrementPreFeederCounter();
      }

      // Send response with status
      byte statusResponse[] = {0x11, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x12 ***************************** //
    // Home actuators
    case 0x12:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x12: Homing actuators");

      // Homing pre-feeder actuator
      uint8_t payload2[8] = {0};  // Initialize buffer for CAN message
      pre_feeder.abs_mode(ACTUATOR_HOME_POSITION, payload2);  // Generate the CAN message
      
      if (CAN1.sendMsgBuf(pre_feeder.motor_id, 0, 8, payload2) != CAN_OK) {
        Serial.println("Error sending pre-feeder command");
        byte errorResponse[] = {0x12, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(pre_feeder.motor_id);

      if (status == 0x01) {
        incrementPreFeederCounter();
      }

      // Homing main feeder actuator
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      feeder.abs_mode(ACTUATOR_HOME_POSITION, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(feeder.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending feeder command");
        byte errorResponse[] = {0x12, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      status = waitForCanReply(feeder.motor_id);

      if (status == 0x01) {
        incrementFeederCounter();
      }      

      // Send response with status
      byte statusResponse[] = {0x12, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x13 ***************************** //
    // Move pre-feeder with speed mode
    case 0x13:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x13: Moving pre-feeder with speed mode");

      uint8_t speed_high = instruction.data[1];
      uint8_t speed_low = instruction.data[2];
      uint8_t direction = instruction.data[3];
      uint8_t acceleration = instruction.data[4];  // Get acceleration from data[4]

      uint16_t speed = (speed_high << 8) | speed_low;
      bool dir = (direction == 1);  // Convert to boolean
      
      // Use default acceleration if not provided
      if (acceleration == 0) {
        acceleration = 50;  // Default acceleration value
      }

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      pre_feeder.speed_mode(dir, speed, acceleration, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(pre_feeder.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending pre-feeder speed command");
        byte errorResponse[] = {0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReplySpeed(pre_feeder.motor_id);

      if (status == 0x01) {
        incrementPreFeederCounter();
      }   

      // Send response with status
      byte statusResponse[] = {0x13, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //
    // Move feeder with speed mode, and pre-feeder if potentiometer is activated
    case 0x14:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x14: Moving feeder with speed mode (conditional pre-feeder)");

      uint8_t speed_high = instruction.data[1];
      uint8_t speed_low = instruction.data[2];
      uint8_t direction = instruction.data[3];
      uint8_t acceleration = instruction.data[4];  // Get acceleration from data[4]

      uint16_t speed = (speed_high << 8) | speed_low;
      bool dir = (direction == 1);  // Convert to boolean
      
      // Use default acceleration if not provided
      if (acceleration == 0) {
        acceleration = 50;  // Default acceleration value
      }

      // Always move the main feeder
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      feeder.speed_mode(dir, speed, acceleration, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(feeder.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending feeder speed command");
        byte errorResponse[] = {0x14, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReplySpeed(feeder.motor_id);

      if (status == 0x01) {
        incrementFeederCounter();
      }

      // Check if linear potentiometer is activated (threshold > 100 for activation)
      int potValue = analogRead(LINEAR_POT_PIN);
      Serial.print("Linear potentiometer value: ");
      Serial.println(potValue);
      
      if (potValue > 100) {
        Serial.println("Potentiometer activated - also moving pre-feeder");
        
        // Determine pre-feeder direction based on potentiometer value
        // Higher pot values (>512) = forward, lower values (100-512) = reverse
        bool preFeederDir = (potValue > 512) ? false : true;  // false = forward, true = reverse
        
        Serial.print("Pre-feeder direction based on pot value ");
        Serial.print(potValue);
        Serial.print(": ");
        Serial.println(preFeederDir ? "reverse" : "forward");
        
        // Move pre-feeder with potentiometer-based direction
        uint8_t payload2[8] = {0};  // Initialize buffer for CAN message
        pre_feeder.speed_mode(preFeederDir, speed, acceleration, payload2);  // Generate the CAN message

        if (CAN1.sendMsgBuf(pre_feeder.motor_id, 0, 8, payload2) != CAN_OK)
        {
          Serial.println("Error sending pre-feeder speed command");
          byte errorResponse[] = {0x14, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
          send_twai_response(errorResponse);
          break;
        }

        // Wait for reply and get status
        uint8_t preStatus = waitForCanReplySpeed(pre_feeder.motor_id);

        if (preStatus == 0x01) {
          incrementPreFeederCounter();
        }
        
        // Send response with both statuses (main status in byte 1, pre-feeder status in byte 2)
        byte statusResponse[] = {0x14, status, preStatus, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(statusResponse);
      } else {
        Serial.println("Potentiometer not activated - only feeder moved");
        // Send response with only main feeder status
        byte statusResponse[] = {0x14, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(statusResponse);
      }
    }
    break;

    // ***************************** CASE 0x15 ***************************** //
    // Check hose position using IR break beam sensor
    case 0x15:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x15: Checking hose position with IR break beam sensor");
      
      // Read IR sensor state (LOW = beam broken = hose in position)
      bool hoseInPosition = !digitalRead(IR_SENSOR_PIN);  // Invert because sensor is active LOW
      
      Serial.print("IR Sensor reading: ");
      Serial.println(digitalRead(IR_SENSOR_PIN));
      Serial.print("Hose in position: ");
      Serial.println(hoseInPosition ? "YES" : "NO");
      
      // Prepare response
      byte response[8] = {0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      response[2] = hoseInPosition ? 0x01 : 0x00;  // 0x01 = hose in position, 0x00 = hose not in position
      
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Check alcohol level using digital level sensors
    case 0x16:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x16: Checking alcohol level with digital level sensors");
      
      // Read digital values from both level sensors (Active HIGH)
      bool sensor1State = digitalRead(LEVEL_SENSOR_1_PIN);  // 2/3 level sensor
      bool sensor2State = digitalRead(LEVEL_SENSOR_2_PIN);  // 1/3 level sensor
      
      Serial.print("Level Sensor 1 (2/3 tank) state: ");
      Serial.println(sensor1State ? "HIGH (liquid present)" : "LOW (no liquid)");
      Serial.print("Level Sensor 2 (1/3 tank) state: ");
      Serial.println(sensor2State ? "HIGH (liquid present)" : "LOW (no liquid)");
      
      // Determine alcohol level status based on digital sensor states
      uint8_t levelStatus = 0x00;  // Default: Empty
      
      // Both sensors HIGH = tank is full (above 2/3 level)
      if (sensor1State && sensor2State) {
        Serial.println("Alcohol level: FULL");
        levelStatus = 0x03;  // Full level
      }
      // Sensor 1 LOW, Sensor 2 HIGH = medium level (between 1/3 and 2/3)
      else if (!sensor1State && sensor2State) {
        Serial.println("Alcohol level: MEDIUM");
        levelStatus = 0x02;  // Medium level
      }
      // Both sensors LOW = tank is empty (below 1/3 level)
      else if (!sensor1State && !sensor2State) {
        Serial.println("Alcohol level: EMPTY");
        levelStatus = 0x00;  // Empty
      }
      // Sensor 1 HIGH, Sensor 2 LOW = error condition (impossible state)
      else {
        Serial.println("Alcohol level: ERROR - Invalid sensor state");
        levelStatus = 0xFF;  // Error code
      }
      
      // Prepare response with level status and sensor states
      byte response[8] = {0x16, 0x01, levelStatus, 0x00, 0x00, 0x00, 0x00, 0x00};
      
      // Include sensor states in response
      response[3] = sensor1State ? 0x01 : 0x00;  // 2/3 level sensor state
      response[4] = sensor2State ? 0x01 : 0x00;  // 1/3 level sensor state
      
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x17 ***************************** //
    // Open hose holder servo
    case 0x17:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x17: Opening hose holder servo");
      
      // Move servo to open position (90 degrees)
      hoseHolderServo.write(0);
      incrementServoCounter();
      
      Serial.println("Hose holder opened to 90 degrees");
      
      // Send success response
      byte response[] = {0x17, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x18 ***************************** //
    // Close hose holder servo
    case 0x18:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x18: Closing hose holder servo");
      
      // Move servo to closed position (0 degrees)
      hoseHolderServo.write(90);
      incrementServoCounter();
      
      Serial.println("Hose holder closed to 0 degrees");
      
      // Send success response
      byte response[] = {0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x19 ***************************** //
    // Attach electromagnet (HIGH)
    case 0x19:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x19: Attaching electromagnet (HIGH)");
      
      digitalWrite(ELECTROMAGNET_PIN, HIGH);
      
      Serial.println("Electromagnet activated (HIGH)");
      
      // Send success response
      byte response[] = {0x19, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x1A ***************************** //
    // Detach electromagnet (LOW)
    case 0x1A:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x1A: Detaching electromagnet (LOW)");
      
      digitalWrite(ELECTROMAGNET_PIN, LOW);
      
      Serial.println("Electromagnet deactivated (LOW)");
      
      // Send success response
      byte response[] = {0x1A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x1B ***************************** //
    // Read servo counter
    case 0x1B:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x1B: Reading servo counter");
      
      Serial.print("Current servo counter: ");
      Serial.println(servoCounter);
      
      // Send response with counter value (16-bit split into high and low bytes)
      byte response[] = {0x1B, 0x01, (byte)(servoCounter >> 8), (byte)(servoCounter & 0xFF), 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x1C ***************************** //
    // Reset servo counter
    case 0x1C:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x1C: Resetting servo counter");
      
      servoCounter = 0;
      saveServoCounter();
      
      Serial.println("Servo counter reset to 0");
      
      // Send success response
      byte response[] = {0x1C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x1D ***************************** //
    // Acknowledge message - responds with 0xAA and received function code
    case 0x1D:
    {
      // This is the ACK message
      if (enableAckMessages) {
        byte ack[] = {0x1D, 0xAC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }

      Serial.println("Case 0x1D: Acknowledge message");
      
      // Get the original function code from the instruction
      byte function_code = instruction.data[0];
      
      Serial.print("Acknowledging function code: 0x");
      Serial.println(function_code, HEX);
      
      // Send acknowledge response with 0xAA in first byte and function code in second byte
      if (enableAckMessages) {
        byte response[] = {0xAA, function_code, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(response);
      }
    }
    break;


    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all actuators to home position
    case 0xFF:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0xFF: Powering off - Moving all actuators to home position");
      
      // Homing pre-feeder actuator
      uint8_t payload2[8] = {0};  // Initialize buffer for CAN message
      pre_feeder.abs_mode(0, payload2);  // Generate the CAN message

      if (CAN1.sendMsgBuf(pre_feeder.motor_id, 0, 8, payload2) != CAN_OK) {
        Serial.println("Error sending pre-feeder command");
        byte errorResponse[] = {0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(pre_feeder.motor_id);

      if (status == 0x01) {
        incrementPreFeederCounter();
      }
      else 
      {
        byte errorResponse[] = {0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Homing main feeder actuator
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      feeder.abs_mode(0, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(feeder.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending feeder command");
        byte errorResponse[] = {0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      status = waitForCanReply(feeder.motor_id);

      if (status == 0x01) {
        incrementFeederCounter();
      }
      else 
      {
        byte errorResponse[] = {0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      } 

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

uint8_t waitForCanReplySpeed(uint16_t expectedId)
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

