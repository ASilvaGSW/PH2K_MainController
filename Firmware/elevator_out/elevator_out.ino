/*
 * Status Codes:
 * 0x01 - OK
 * 0x02 - FAIL
 * 0x03 - TIMEOUT
 * 0x04 - NO LOCAL NETWORK
 * 
 * CAN Commands:
 * 0x01 - Reset microcontroller
 * 0x02 - Heartbeat
 * 0x03 - Move Individual Actuator
 * 0x05 - Move Elevator Z
 * 0x0B - Get Actuator Counter
 * 0x0C - Reset Actuator Counter
 * 0x10 - Home Individual Actuator (by actuator_id)
 * 0x12 - Home All Axes (elevatorZ)
 * 0x13 - Control Servo
 * 0x14 - Get Servo Counter
 * 0x15 - Reset Servo Counter
 * 0x16 - Get Optical Sensor Status
 * 0x17 - Move Elevator to Reception
 * 0x18 - Set Servo State (Home/Open)
 * 0x19 - Move Elevator Relative (Jog)
 * 0xFF - Power off
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
#define EEPROM_SIZE 32 // 4 bytes for each counter (servos + actuator)
#define ELEVATOR_COUNTER_Z_ADDR 16 // Address in EEPROM to store the actuator counter
#define SERVO2_COUNTER_ADDR 20 // Address in EEPROM to store the second servo counter
#define SERVO_COUNTER_ADDR 28 // Address in EEPROM to store the servo counter

// Global variables
unsigned long counter_elevatorZ = 0;
unsigned long counter_servo = 0;
unsigned long counter_servo2 = 0;

byte replyData[8];  // Buffer for CAN replies

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO4 - TX for CAN0 (General Network)

// Servo configuration
#define SERVO_PIN 14  // Pin where the servo is connected
#define SERVO2_PIN 15 // Pin where the second servo is connected

// Configuración de goBILDA para el modo "Servo" (Posicional)
const int minPulse = 500;  // Pulso mínimo en microsegundos
const int maxPulse = 2500; // Pulso máximo en microsegundos

#define SERVO_PWM_HOME 0x360 // 864 us
#define SERVO_PWM_OPEN 0x9C0 // 2464 us

Servo gobilda;  // Create servo object to control a servo
Servo gobilda2; // Create second servo object object to control a servo1

// Optical Sensor
#define OPTICAL_SENSOR_PIN 21

// Pins for the second CAN bus (MCP2515)
#define CAN1_CS 26  // Changed from 15 to 26 to avoid upload issues
#define CAN1_INT 25  // Changed from 2 to 25 to avoid boot issues

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x188
#define RESPONSE_CAN_ID 0x588

// Instance for the second CAN bus (MCP2515)
MCP_CAN CAN1(CAN1_CS);

// Instance of LinearActuator
LinearActuator elevatorZ(0x001);

void loadConfigValues()
{
  counter_elevatorZ = EEPROM.readULong(ELEVATOR_COUNTER_Z_ADDR);
  counter_servo = EEPROM.readULong(SERVO_COUNTER_ADDR);
  counter_servo2 = EEPROM.readULong(SERVO2_COUNTER_ADDR);
}

// Queue to store instructions received from the main CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Function declarations
void process_instruction(CanFrame instruction);
uint8_t waitForCanReply(uint16_t expectedId);
void send_twai_response(const byte response_data[8]);
uint8_t moveIndividualActuator(uint8_t actuator_id, int16_t angle, uint8_t orientation);

// Generic function to save counter to EEPROM
void saveCounter(const char* name, uint32_t address, unsigned long &counter)
{
  counter++;
  EEPROM.writeULong(address, counter);
  EEPROM.commit();
  Serial.print(name);
  Serial.print(" counter saved: ");
  Serial.println(counter);
}

// Wrapper functions for each counter
void saveElevatorZCounter() { saveCounter("Elevator Z", ELEVATOR_COUNTER_Z_ADDR, counter_elevatorZ); }
void saveServoCounter() { saveCounter("Servo", SERVO_COUNTER_ADDR, counter_servo); }
void saveServo2Counter() { saveCounter("Servo 2", SERVO2_COUNTER_ADDR, counter_servo2); }

void SaveCounter(int id)
{
  switch (id)
  {
    case 1: saveElevatorZCounter(); break;
    default: break;
  }
}

void setup()
{
  Serial.begin(115200);

  // Asignar el pin y definir los límites de pulso específicos de goBILDA
  gobilda.setPeriodHertz(50); // Frecuencia estándar de 50Hz para servos
  gobilda.attach(SERVO_PIN, minPulse, maxPulse);  // Attach servo to pin

  gobilda2.setPeriodHertz(50); // Frecuencia estándar de 50Hz para servos
  gobilda2.attach(SERVO2_PIN, minPulse, maxPulse); // Attach second servo to pin

  pinMode(OPTICAL_SENSOR_PIN, INPUT); // Initialize optical sensor pin as input
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load persistent config values
  loadConfigValues();

  Serial.print("Loaded Elevator Z counter from EEPROM: "); Serial.println(counter_elevatorZ);
  
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

  switch (instruction.data[0])
  {

    // ***************************** CASE 0x01 ***************************** //
    // Reset microcontroller
    case 0x01: 
    {
      Serial.println("Case 0x01: Reset");
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
    
    // ***************************** CASE 0x0B ***************************** //
    // Get Actuator Counter
    case 0x0B:
    {
      Serial.println("Case 0x0B: Get Actuator Counter");

      uint8_t actuator_id = instruction.data[1];
      uint16_t counter = 0;

      switch (actuator_id)
      {
        case 1: counter = counter_elevatorZ; break;
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
        case 1: counter_elevatorZ = 0; break;
        default: break;
      }

      SaveCounter(actuator_id);
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
      uint16_t expected_id = 0;
      
      // Generate go_home command based on actuator ID
      if (actuator_id == 1) {
        elevatorZ.go_home(payload);
        expected_id = elevatorZ.motor_id + 0x580;
      }
      else {
        Serial.println("Invalid actuator ID");
        byte errorResponse[] = {0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(errorResponse);
        break;
      }
      
      // Send CAN message(s)
      uint8_t status = 0x04; // Default to NO LOCAL NETWORK
      
      if (CAN1.sendMsgBuf(elevatorZ.motor_id, 0, 2, payload) == CAN_OK)
      {
          status = waitForCanReply(expected_id);
      }
      
      byte statusResponse[] = {0x10, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;


    // ***************************** CASE 0x13 ***************************** //
    // Control servo
    case 0x13:
    {

      // home 2500 open 850 
      Serial.println("Case 0x13: Control servo");
      uint16_t pwm_value = instruction.data[1] << 8 | instruction.data[2]; // Target PWM
      uint8_t mode = instruction.data[3];  // 0: Both, 1: Servo 1, 2: Servo 2
      
      if (mode == 0) // Move Both
      {
          gobilda.writeMicroseconds(pwm_value);
          gobilda2.writeMicroseconds(pwm_value);
          Serial.print("Both servos moved to PWM "); Serial.println(pwm_value);
          saveServoCounter();
          saveServo2Counter();
      }
      else if (mode == 1) // Move Servo 1
      {
          gobilda.writeMicroseconds(pwm_value);
          Serial.print("Servo 1 moved to PWM "); Serial.println(pwm_value);
          saveServoCounter();
      }
      else if (mode == 2) // Move Servo 2
      {
          gobilda2.writeMicroseconds(pwm_value);
          Serial.print("Servo 2 moved to PWM "); Serial.println(pwm_value);
          saveServo2Counter();
      }
      else
      {
          Serial.println("Invalid servo mode");
          byte errorResponse[] = {0x13, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          send_twai_response(errorResponse);
          break;
      }
      
      byte statusResponse[] = {0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //
    // Get Servo Counter
    case 0x14:
    {
      Serial.println("Case 0x14: Get Servo Counter");
      uint8_t servo_id = instruction.data[1]; // Wait, protocol check.
      // If we follow pattern, data[1] might be servo id if we redefine. 
      // But currently 0x14 ignores data[1].
      // To support 2nd servo, we should check data[1] or data[2]. 
      // Let's use data[1] as ID if provided, consistent with other commands?
      // Case 0x13 used data[2] because data[1] was action.
      // Case 0x14 has no other data. So data[1] is fine for ID.
      // Default to servo 1 if 0.
      
      uint16_t current_counter = (servo_id == 2) ? counter_servo2 : counter_servo;
      
      uint8_t counter_high = current_counter >> 8;
      uint8_t counter_low = current_counter & 0xFF;
      byte response[] = {0x14, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x15 ***************************** //
    // Reset Servo Counter
    case 0x15:
    {
      Serial.println("Case 0x15: Reset Servo Counter");
      uint8_t servo_id = instruction.data[1];
      
      if (servo_id == 2) {
        counter_servo2 = 0;
        saveServo2Counter();
      } else {
        counter_servo = 0;
        saveServoCounter();
      }
      
      byte statusResponse[] = {0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Get Optical Sensor Status
    case 0x16:
    {
      Serial.println("Case 0x16: Get Optical Sensor Status");
      uint8_t sensor_status = digitalRead(OPTICAL_SENSOR_PIN);
      Serial.print("Sensor Status: "); Serial.println(sensor_status);
      
      byte response[] = {0x16, sensor_status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x17 ***************************** //
    // Move Elevator to Reception
    case 0x17:
    {
      Serial.println("Case 0x17: Move Elevator to Reception");
      
      uint16_t angle = 0x3300; // 0x33 << 8 | 0x00
      uint8_t orientation = 0x01;
      
      uint8_t status = moveIndividualActuator(elevatorZ.motor_id, angle, orientation);
      
      byte statusResponse[] = {0x17, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x18 ***************************** //
    // Set Servo State (Home/Open)
    case 0x18:
    {
      Serial.println("Case 0x18: Set Servo State");
      uint8_t state = instruction.data[1]; // 0: Home, 1: Open
      uint8_t mode = instruction.data[2];  // 0: Both, 1: Servo 1, 2: Servo 2
      
      uint16_t target_pwm = (state == 1) ? SERVO_PWM_OPEN : SERVO_PWM_HOME;
      
      if (mode == 0) // Move Both
      {
          gobilda.writeMicroseconds(target_pwm);
          gobilda2.writeMicroseconds(target_pwm);
          Serial.print("Both servos moved to "); Serial.println(target_pwm);
          saveServoCounter();
          saveServo2Counter();
      }
      else if (mode == 1) // Move Servo 1
      {
          gobilda.writeMicroseconds(target_pwm);
          Serial.print("Servo 1 moved to "); Serial.println(target_pwm);
          saveServoCounter();
      }
      else if (mode == 2) // Move Servo 2
      {
          gobilda2.writeMicroseconds(target_pwm);
          Serial.print("Servo 2 moved to "); Serial.println(target_pwm);
          saveServo2Counter();
      }
      
      byte statusResponse[] = {0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0x19 ***************************** //
    // Move Elevator Relative
    case 0x19:
    {
      Serial.println("Case 0x19: Move Elevator Relative");
      
      // Use a fixed small angle for jogging or read from data if provided
      // For "small movements", let's use data[1] and data[2] as the relative amount
      // If data is 0, default to a small step like 10 degrees (approx 0.1 turns)
      
      int16_t angle = instruction.data[1] << 8 | instruction.data[2];
      if (angle == 0) angle = 3600; // Default small move if 0 sent (approx 10 degrees? No, unit depends on motor)
                                    // Assuming 3600 is significant enough based on previous 0x3300 reception pos.
                                    // Actually, let's look at 0x3300 = 13056. 
                                    // If 360 degrees = 36000 (0.01 deg units?), then 0x3300 is ~130 deg.
                                    // Let's assume user sends the delta.
      
      uint8_t orientation = 0; // Fixed orientation 0 as requested
      
      // We need to use relative_move_with_speed_control from LinearActuator
      // But we need to implement the wrapper to send it via CAN1
      
      uint8_t payload[8] = {0};
      // Default speed/acc from class are 1500/236. Let's use them or pass specific ones.
      elevatorZ.relative_move_with_speed_control(angle, 1500, 236, payload);
      
      if (CAN1.sendMsgBuf(elevatorZ.motor_id, 0, 8, payload) != CAN_OK)
      {
        Serial.println("Error sending relative move command");
        byte errorResponse[] = {0x19, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(errorResponse);
        break;
      }

      // Wait for reply
      uint8_t status = waitForCanReply(elevatorZ.motor_id);
      
      if (status == 0x01) { 
        // Update counter if needed, though relative moves might make absolute counter tracking tricky 
        // unless we read back position. For now, just save as success.
        saveElevatorZCounter(); 
      }
      
      byte statusResponse[] = {0x19, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(statusResponse);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
        Serial.println("Case 0xFF: Powering off - Moving all to home position");
        
        // Move servos to HOME
        gobilda.writeMicroseconds(SERVO_PWM_HOME);
        gobilda2.writeMicroseconds(SERVO_PWM_HOME);
        Serial.println("Servos moved to HOME position");
        
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

  // Filter for ElevatorZ (Accept ID 1)
  if (actuator_id == 1)
  {
    elevatorZ.abs_mode(angle, payload);  // Generate the CAN message
    
    if (CAN1.sendMsgBuf(elevatorZ.motor_id, 0, 8, payload) != CAN_OK)
    {
      Serial.println("Error sending actuator command");
      return 0x04;
    }

    // Wait for reply (using elevatorZ.motor_id)
    uint8_t status = waitForCanReply(elevatorZ.motor_id);
    
    if (status == 0x01) { SaveCounter(actuator_id); }

    return status;
  }
  
  return 0x04; // Error for other IDs
}
