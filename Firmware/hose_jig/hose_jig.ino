/*
 * CASE DESCRIPTIONS:
 * 
 * 0x01: Reset microcontroller
 * 0x02: Send ping
 * 0x03: Home actuator
 * 0x04: Move actuator to absolute position
 * 0x05: Move servos to open position
 * 0x06: Move servos to close position
 * 0x08: Get servo movement counter
 * 0x09: Reset servo movement counter
 * 0x0A: Move servo to absolute position
 * 0x0B: Move actuator to insertion position
 * 0x0C: Move actuator to deliver position
 * 0x0D: Read actuator movement counter
 * 0x0E: Reset actuator movement counter
 * 0x10: Update SERVO_OPEN_ANGLE
 * 0x11: Update SERVO_CLOSE_ANGLE
 * 0x12: Update ACTUATOR_DELIVER_POSITION
 * 0x13: Update ACTUATOR_INSERTION_POSITION
 * 0x14: Read SERVO_OPEN_ANGLE
 * 0x15: Read SERVO_CLOSE_ANGLE
 * 0x16: Read ACTUATOR_DELIVER_POSITION
 * 0x17: Read ACTUATOR_INSERTION_POSITION
 * 0x18: Home actuator using go_home function
 * 0xFF: Power off - Move all to home position
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
#define EEPROM_SIZE 32  // 8 bytes for counters + 24 bytes for config values
#define COUNTER_ADDR 0
#define ACTUATOR_COUNTER_ADDR 4
#define SERVO_OPEN_ANGLE_ADDR 8
#define SERVO_CLOSE_ANGLE_ADDR 10
#define ACTUATOR_DELIVER_POSITION_ADDR 12
#define ACTUATOR_INSERTION_POSITION_ADDR 14


// Global variables
unsigned long servoMoveCounter = 0;
unsigned long actuatorMoveCounter = 0;
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
#define DEVICE_CAN_ID 0x0CA
#define RESPONSE_CAN_ID 0x4CA

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Instance of LinearActuator
LinearActuator x_axis(0x2CE, 100, 236);

// Servo objects
ESP32PWM pwm;
Servo servo_left;
Servo servo_center;
Servo servo_right;

// Servo pins (moved to avoid conflicts with MCP2515)
const int SERVO_LEFT_PIN = 14;
const int SERVO_CENTER_PIN = 15;
const int SERVO_RIGHT_PIN = 16;

// Inductive sensors
const int SENSOR_LEFT_PIN = 27;
const int SENSOR_CENTER_PIN = 32;
const int SENSOR_RIGHT_PIN = 33;

int SERVO_OPEN_ANGLE = 4;
int SERVO_CLOSE_ANGLE = 70;

const int ACTUATOR_HOME_POSITION = 0;
int ACTUATOR_DELIVER_POSITION = 100;
int ACTUATOR_INSERTION_POSITION = 100;

void saveConfigValues() {
  // Verify addresses are within bounds
  if (SERVO_OPEN_ANGLE_ADDR + 2 > EEPROM_SIZE ||
      SERVO_CLOSE_ANGLE_ADDR + 2 > EEPROM_SIZE ||
      ACTUATOR_DELIVER_POSITION_ADDR + 2 > EEPROM_SIZE ||
      ACTUATOR_INSERTION_POSITION_ADDR + 2 > EEPROM_SIZE) {
    Serial.println("Error: EEPROM address out of bounds");
    return;
  }
  
  // Write all config values
  EEPROM.writeUShort(SERVO_OPEN_ANGLE_ADDR, SERVO_OPEN_ANGLE);
  EEPROM.writeUShort(SERVO_CLOSE_ANGLE_ADDR, SERVO_CLOSE_ANGLE);
  EEPROM.writeUShort(ACTUATOR_DELIVER_POSITION_ADDR, ACTUATOR_DELIVER_POSITION);
  EEPROM.writeUShort(ACTUATOR_INSERTION_POSITION_ADDR, ACTUATOR_INSERTION_POSITION);
  
  // Commit changes to EEPROM
  if (!EEPROM.commit()) {
    Serial.println("Error: EEPROM commit failed");
  } else {
    Serial.println("Config values saved to EEPROM");
  }
}

void loadConfigValues() {
  // Initialize with default values first
  uint16_t defaultOpenAngle = 4;
  uint16_t defaultCloseAngle = 70;
  uint16_t defaultDeliverPos = 0;
  uint16_t defaultInsertPos = 1000;
  
  // Try to read from EEPROM
  SERVO_OPEN_ANGLE = EEPROM.readUShort(SERVO_OPEN_ANGLE_ADDR);
  SERVO_CLOSE_ANGLE = EEPROM.readUShort(SERVO_CLOSE_ANGLE_ADDR);
  ACTUATOR_DELIVER_POSITION = EEPROM.readUShort(ACTUATOR_DELIVER_POSITION_ADDR);
  ACTUATOR_INSERTION_POSITION = EEPROM.readUShort(ACTUATOR_INSERTION_POSITION_ADDR);
  
  // Check if values are uninitialized (0xFFFF)
  bool needsSave = false;
  
  if (SERVO_OPEN_ANGLE == 0xFFFF) {
    SERVO_OPEN_ANGLE = defaultOpenAngle;
    needsSave = true;
  }
  if (SERVO_CLOSE_ANGLE == 0xFFFF) {
    SERVO_CLOSE_ANGLE = defaultCloseAngle;
    needsSave = true;
  }
  if (ACTUATOR_DELIVER_POSITION == 0xFFFF) {
    ACTUATOR_DELIVER_POSITION = defaultDeliverPos;
    needsSave = true;
  }
  if (ACTUATOR_INSERTION_POSITION == 0xFFFF) {
    ACTUATOR_INSERTION_POSITION = defaultInsertPos;
    needsSave = true;
  }
  
  // Save default values if any were uninitialized
  if (needsSave) {
    saveConfigValues();
    Serial.println("Initialized EEPROM with default values");
  }
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

// Function to save servo counter to EEPROM
void saveCounter() {
  EEPROM.writeULong(COUNTER_ADDR, servoMoveCounter);
  EEPROM.commit();
  Serial.print("Servo counter saved: ");
  Serial.println(servoMoveCounter);
}

// Function to save actuator counter to EEPROM
void saveActuatorCounter() {
  EEPROM.writeULong(ACTUATOR_COUNTER_ADDR, actuatorMoveCounter);
  EEPROM.commit();
  Serial.print("Actuator counter saved: ");
  Serial.println(actuatorMoveCounter);
}

// Function to increment and save the servo counter
void incrementCounter() {
  servoMoveCounter++;
  saveCounter();
}

// Function to increment and save the actuator counter
void incrementActuatorCounter() {
  actuatorMoveCounter++;
  saveActuatorCounter();
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM
  servoMoveCounter = EEPROM.readULong(COUNTER_ADDR);
  actuatorMoveCounter = EEPROM.readULong(ACTUATOR_COUNTER_ADDR);
  Serial.print("Loaded servo move counter from EEPROM: ");
  Serial.println(servoMoveCounter);
  Serial.print("Loaded actuator move counter from EEPROM: ");
  Serial.println(actuatorMoveCounter);

  // Load persistent config values
  loadConfigValues();
  Serial.print("Loaded SERVO_OPEN_ANGLE: "); Serial.println(SERVO_OPEN_ANGLE);
  Serial.print("Loaded SERVO_CLOSE_ANGLE: "); Serial.println(SERVO_CLOSE_ANGLE);
  Serial.print("Loaded ACTUATOR_DELIVER_POSITION: "); Serial.println(ACTUATOR_DELIVER_POSITION);
  Serial.print("Loaded ACTUATOR_INSERTION_POSITION: "); Serial.println(ACTUATOR_INSERTION_POSITION);

  // Configure PWM for servos
  ESP32PWM::timerCount[0] = 0; // Reset timer count
  pwm.attachPin(0, 10000, 12); // 10kHz, 12-bit resolution
  
  // Attach Servo motors with ESP32Servo
  servo_left.setPeriodHertz(50);    // Standard 50hz servo
  servo_left.attach(SERVO_LEFT_PIN, 500, 2500); // Min/max pulse width in microseconds
  
  servo_center.setPeriodHertz(50);
  servo_center.attach(SERVO_CENTER_PIN, 500, 2500);
  
  servo_right.setPeriodHertz(50);
  servo_right.attach(SERVO_RIGHT_PIN, 500, 2500);

  // Configure inductive sensors as inputs with pullup
  pinMode(SENSOR_LEFT_PIN, INPUT_PULLUP);
  pinMode(SENSOR_CENTER_PIN, INPUT_PULLUP);
  pinMode(SENSOR_RIGHT_PIN, INPUT_PULLUP);

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

  //Servos to Home Position 0
  servo_left.write(SERVO_OPEN_ANGLE);
  servo_center.write(SERVO_OPEN_ANGLE);
  servo_right.write(SERVO_OPEN_ANGLE);
  
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
    // Send ping
    case 0x02:
    {
      Serial.println("Case 0x02: Read X axis status");
      byte response[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x03 ***************************** //
    // Home actuator
    case 0x03:
    { 
      Serial.println("Case 0x03: Homing actuator");

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode(ACTUATOR_HOME_POSITION, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounter();
      }

      // Send response with status
      byte statusResponse[] = {0x03, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Move actuator to absolute position
    case 0x04:
    {
      Serial.println("Case 0x04: Moving actuator to absolute position");

      uint8_t pos_high = instruction.data[1];
      uint8_t pos_low = instruction.data[2];
      uint8_t orientation = instruction.data[3];

      int16_t angle = (pos_high << 8) | pos_low;
      if (orientation == 1)
      {
        angle = angle * -1;
      }
      Serial.print("Angle after conversion: ");
      Serial.println(angle);

      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode(angle, payload);  // Generate the CAN message

      // Debug: Print the CAN message being sent
      Serial.print("Sending CAN message to ID: 0x");
      Serial.print(x_axis.motor_id, HEX);
      Serial.print(" Data: ");
      for(int i = 0; i < 8; i++) {
        if(payload[i] < 0x10) Serial.print("0");
        Serial.print(payload[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      // Check CAN bus status before sending
      if(CAN1.checkReceive() == CAN_MSGAVAIL) {
        Serial.println("Warning: CAN buffer has pending messages before send!");
      }

      // Try to send the message
      byte sendStatus = CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload);
      if (sendStatus != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounter();
      }

      // Send response with status
      byte statusResponse[] = {0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x05 ***************************** //
    // Move Servos to Open Position
    case 0x05:
    {
      Serial.println("Case 0x05: Moving servos to open position");
      servo_left.write(SERVO_OPEN_ANGLE);
      servo_center.write(SERVO_OPEN_ANGLE);
      servo_right.write(SERVO_OPEN_ANGLE);
      incrementCounter();
      delay(1000);
      byte response[] = {0x05, 0x01, SERVO_OPEN_ANGLE, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;
 
    // ***************************** CASE 0x06 ***************************** //
    // Move Servos to Close Position
    case 0x06:
    {
      Serial.println("Case 0x06: Moving servos to close position");
      servo_left.write(SERVO_CLOSE_ANGLE);
      servo_center.write(SERVO_CLOSE_ANGLE);
      servo_right.write(SERVO_CLOSE_ANGLE);
      incrementCounter();
      delay(500);
      byte response[] = {0x06, 0x01, SERVO_CLOSE_ANGLE, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Get servo movement counter
    case 0x08:
    {
      Serial.println("Case 0x08: Reading servo movement counter");
      byte counter_high = (servoMoveCounter >> 8) & 0xFF;
      byte counter_low = servoMoveCounter & 0xFF;
      byte response[] = {0x08, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x09 ***************************** //
    // Reset servo movement counter
    case 0x09:
    {
      Serial.println("Case 0x09: Resetting servo movement counter");
      servoMoveCounter = 0;
      saveCounter();
      byte response[] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    
    // ***************************** CASE 0x0A ***************************** //
    // Move Servo to Absolute Position
    case 0x0A:
    {
      Serial.println("Case 0x0A: Moving servos to absolute position");

      uint8_t position = instruction.data[1];

      servo_left.write(position);
      servo_center.write(position);
      servo_right.write(position);
      incrementCounter();

      delay(1000);
      byte response[] = {0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
      send_twai_response(response);
    }
    break;


     // ***************************** CASE 0x0B ***************************** //
    // Move Actuator to Insertion Position
    case 0x0B:
    {
      Serial.println("Case 0x0B: Moving actuator to insertion position");
      
      uint8_t payload[8] = {0};  // Initialize buffer for CAN message
      x_axis.abs_mode(ACTUATOR_INSERTION_POSITION, payload);  // Generate the CAN message

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x0B, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounter();
      }

      // Send response with status
      byte statusResponse[] = {0x0B, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x0C ***************************** //
    // Move Actuator to Deliver Position
    case 0x0C:
    {
      Serial.println("Case 0x0C: Moving actuator to deliver position");
      
      uint8_t payload[8] = {0};
      x_axis.abs_mode(ACTUATOR_DELIVER_POSITION, payload);

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 8, payload) != CAN_OK) {
        Serial.println("Error sending actuator command");
        byte errorResponse[] = {0x0C, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(errorResponse);
        break;
      }

      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounter();
      }

      byte statusResponse[] = {0x0C, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    
    // ***************************** CASE 0x0D ***************************** //
    // Read actuator movement counter
    case 0x0D:
    {
      Serial.println("Case 0x0D: Reading actuator movement counter");
      byte counter_high = (actuatorMoveCounter >> 8) & 0xFF;
      byte counter_low = actuatorMoveCounter & 0xFF;
      byte response[] = {0x0D, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0E ***************************** //
    // Reset actuator movement counter
    case 0x0E:
    {
      Serial.println("Case 0x0E: Resetting actuator movement counter");
      actuatorMoveCounter = 0;
      saveActuatorCounter();
      byte response[] = {0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;


    // ***************************** CASE 0x10 ***************************** //
    // Update SERVO_OPEN_ANGLE
    case 0x10:
    {
      uint16_t value = (instruction.data[1] << 8) | instruction.data[2];
      SERVO_OPEN_ANGLE = value;
      saveConfigValues();
      Serial.print("Updated SERVO_OPEN_ANGLE to: "); Serial.println(SERVO_OPEN_ANGLE);
      byte response[] = {0x10, 0x01, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF), 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x11 ***************************** //
    // Update SERVO_CLOSE_ANGLE
    case 0x11:
    {
      uint16_t value = (instruction.data[1] << 8) | instruction.data[2];
      SERVO_CLOSE_ANGLE = value;
      saveConfigValues();
      Serial.print("Updated SERVO_CLOSE_ANGLE to: "); Serial.println(SERVO_CLOSE_ANGLE);
      byte response[] = {0x11, 0x01, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF), 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x12 ***************************** //
    // Update ACTUATOR_DELIVER_POSITION
    case 0x12:
    {
      uint16_t value = (instruction.data[1] << 8) | instruction.data[2];
      ACTUATOR_DELIVER_POSITION = value;
      saveConfigValues();
      Serial.print("Updated ACTUATOR_DELIVER_POSITION to: "); Serial.println(ACTUATOR_DELIVER_POSITION);
      byte response[] = {0x12, 0x01, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF), 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x13 ***************************** //
    // Update ACTUATOR_INSERTION_POSITION
    case 0x13:
    {
      uint16_t value = (instruction.data[1] << 8) | instruction.data[2];
      ACTUATOR_INSERTION_POSITION = value;
      saveConfigValues();
      Serial.print("Updated ACTUATOR_INSERTION_POSITION to: "); Serial.println(ACTUATOR_INSERTION_POSITION);
      byte response[] = {0x13, 0x01, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF), 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //
    // Read SERVO_OPEN_ANGLE
    case 0x14:
    {
      Serial.println("Case 0x14: Reading SERVO_OPEN_ANGLE");
      byte high = (SERVO_OPEN_ANGLE >> 8) & 0xFF;
      byte low = SERVO_OPEN_ANGLE & 0xFF;
      byte response[] = {0x14, 0x01, high, low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x15 ***************************** //
    // Read SERVO_CLOSE_ANGLE
    case 0x15:
    {
      Serial.println("Case 0x15: Reading SERVO_CLOSE_ANGLE");
      byte high = (SERVO_CLOSE_ANGLE >> 8) & 0xFF;
      byte low = SERVO_CLOSE_ANGLE & 0xFF;
      byte response[] = {0x15, 0x01, high, low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Read ACTUATOR_DELIVER_POSITION
    case 0x16:
    {
      Serial.println("Case 0x16: Reading ACTUATOR_DELIVER_POSITION");
      byte high = (ACTUATOR_DELIVER_POSITION >> 8) & 0xFF;
      byte low = ACTUATOR_DELIVER_POSITION & 0xFF;
      byte response[] = {0x16, 0x01, high, low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x17 ***************************** //
    // Read ACTUATOR_INSERTION_POSITION
    case 0x17:
    {
      Serial.println("Case 0x17: Reading ACTUATOR_INSERTION_POSITION");
      byte high = (ACTUATOR_INSERTION_POSITION >> 8) & 0xFF;
      byte low = ACTUATOR_INSERTION_POSITION & 0xFF;
      byte response[] = {0x17, 0x01, high, low, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;    

    // ***************************** CASE 0x18 ***************************** //
    // Home actuator using go_home function
    case 0x18:
    {
      Serial.println("Case 0x18: Homing actuator using go_home function");

      uint8_t payload[2] = {0};  // Initialize buffer for CAN message
      x_axis.go_home(payload);  // Generate the go_home CAN message

      if (CAN1.sendMsgBuf(x_axis.motor_id, 0, 2, payload) != CAN_OK) {
        Serial.println("Error sending go_home command");
        byte errorResponse[] = {0x18, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // NO LOCAL NETWORK
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply and get status
      uint8_t status = waitForCanReply(x_axis.motor_id);

      if (status == 0x01) {
        incrementActuatorCounter();
      }

      // Send response with status
      byte statusResponse[] = {0x18, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
      Serial.println("Case 0xFF: Powering off - Moving all to home position");
      
      // Move servos to home position
      servo_left.write(0);
      servo_center.write(0);
      servo_right.write(0);
      incrementCounter();
      
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
      
      if (canId == expectedId && (replyData[1] == 0x02 || replyData[1] == 0x03)) {
        return 0x01;  // Success
      }
    }
    vTaskDelay(1);  // Small delay to prevent busy-waiting
  }
  return 0x03;  // Timeout
}

// Helper function to send response via TWAI
void send_twai_response(const byte response_data[8]) {
  CanFrame tx_frame;
  tx_frame.identifier = RESPONSE_CAN_ID;  // Response ID
  tx_frame.extd = 0;
  tx_frame.data_length_code = 8;
  memcpy(tx_frame.data, response_data, 8);
  ESP32Can.writeFrame(tx_frame);
}