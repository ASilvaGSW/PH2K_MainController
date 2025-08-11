/*
 * insertion_servos.ino
 * -----------
 * Firmware for a single-CAN (Controller Area Network) based ESP32 project controlling 7 servos.
 * 
 * FEATURES:
 *   - Single CAN bus support: ESP32 TWAI (integrated)
 *   - FreeRTOS for multitasking
 *   - EEPROM for persistent storage of individual servo counters
 *   - Robust CAN communication with error handling
 *   - Controls 7 servos for various insertion operations
 *
 * DEPENDENCIES:
 *   - ESP32-TWAI-CAN
 *   - ESP32Servo
 *   - FreeRTOS (ESP32)
 *   - EEPROM (ESP32)
 */

#include <ESP32-TWAI-CAN.hpp>
#include <ESP32Servo.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration
#define EEPROM_SIZE 28  // Size in bytes for EEPROM storage (4 bytes per servo counter)
#define SERVO_COUNTER_BASE_ADDR 0  // Base address for servo counters

// Pins for the CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x002
#define RESPONSE_CAN_ID 0x402  // Response ID (DEVICE_CAN_ID + 0x400)

// Status codes
#define STATUS_OK 0x01
#define STATUS_FAIL 0x02
#define STATUS_TIMEOUT 0x03
#define STATUS_NO_NETWORK 0x04

// Global variables
byte replyData[8] = {0};  // Buffer for CAN replies

// Queue to store instructions received from the CAN bus
QueueHandle_t instruction_queue;

// Definition of servos with names and limits
struct ServoConfig
{
  String id;         // Servo ID
  String action;     // Action name
  int pin;           // Assigned pin
  int minAngle;      // Minimum angle
  int maxAngle;      // Maximum angle
  Servo servo;       // Servo object
  bool atOrigin;     // Tracks if the servo is at its origin
  uint8_t canCommandID; // Unique CAN command ID for this servo
  unsigned long counter; // Individual operation counter
};

// Servo configuration
ServoConfig servos[] =
{
  // Format: {id, action, pin, min, max, Servo(), atOrigin, canCommandID, counter}
  {"s3", "Slider Joint", 18, 0, 160, Servo(), true, 0x10, 0},
  {"s4", "Clamp Joint", 17, 20, 110, Servo(), true, 0x11, 0},
  {"s8", "Holder Hose (Nozzle side)", 21, 0, 120, Servo(), true, 0x12, 0},
  {"s5", "Slider nozzle", 15, 125, 0, Servo(), true, 0x13, 0},
  {"s6", "Clamp nozzle", 14, 40, 140, Servo(), true, 0x14, 0},
  {"s7", "Holder hose (Joint side)", 16, 0, 125, Servo(), true, 0x15, 0},
  {"s9", "Cutter", 19, 0, 84, Servo(), true, 0x16, 0}
};

const int servoCount = sizeof(servos) / sizeof(servos[0]);

// Function declarations
void process_instruction(CanFrame instruction);
void send_twai_response(byte response_data[8]);
void saveServoCounter(int servoIndex);
void loadCounters();
void incrementServoCounter(int servoIndex);
void showConfiguration();
void moveAllToOrigin();
int findServoIndexByID(String id);
int findServoIndexByID(uint8_t id);
void setup_servos_and_serial();
void setup_can();
void twai_listener_task(void *pvParameters);

// Function to save individual servo counter to EEPROM
void saveServoCounter(int servoIndex) {
  if (servoIndex >= 0 && servoIndex < servoCount) {
    uint32_t address = SERVO_COUNTER_BASE_ADDR + (servoIndex * 4);
    EEPROM.writeULong(address, servos[servoIndex].counter);
    EEPROM.commit();
    Serial.print("Servo ");
    Serial.print(servos[servoIndex].id);
    Serial.print(" counter saved: ");
    Serial.println(servos[servoIndex].counter);
  }
}

// Function to load all counters from EEPROM
void loadCounters() {
  // Load individual servo counters
  for (int i = 0; i < servoCount; i++) {
    uint32_t address = SERVO_COUNTER_BASE_ADDR + (i * 4);
    servos[i].counter = EEPROM.readULong(address);
    if (servos[i].counter == 0xFFFFFFFF) {  // First time or EEPROM not initialized
      servos[i].counter = 0;
      saveServoCounter(i);
    }
    Serial.print("Loaded ");
    Serial.print(servos[i].id);
    Serial.print(" counter from EEPROM: ");
    Serial.println(servos[i].counter);
  }
}

// Function to increment and save an individual servo counter
void incrementServoCounter(int servoIndex) {
  if (servoIndex >= 0 && servoIndex < servoCount) {
    servos[servoIndex].counter++;
    saveServoCounter(servoIndex);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing Insertion Servos Controller...");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadCounters();

  setup_servos_and_serial();
  setup_can();

  Serial.println("System ready.");
}

void setup_can() {
  // Create the instruction queue
  instruction_queue = xQueueCreate(10, sizeof(CanFrame));
  if (instruction_queue == NULL) {
    Serial.println("Error creating instruction queue");
    while(1);
  }

  // Initialize the CAN bus (TWAI)
  Serial.println("Initializing CAN (TWAI)...");
  ESP32Can.setPins(CAN0_TX, CAN0_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(125)); // Set to 125 kbps
  if (ESP32Can.begin()) {
    Serial.println("CAN (TWAI) initialized successfully");
  } else {
    Serial.println("Error initializing CAN (TWAI)");
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
}

void setup_servos_and_serial()
{
  Serial.println("Servo control system started.");
  Serial.print("Available Servo IDs: ");
  for (int i = 0; i < servoCount; i++) {
    Serial.print(servos[i].id);
    if (i < servoCount - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();

  // Configure servos
  for (int i = 0; i < servoCount; i++)
  {
    servos[i].servo.setPeriodHertz(50);
    servos[i].servo.attach(servos[i].pin, 500, 2500);
    servos[i].servo.write(servos[i].minAngle); // Initial position
  }
 
  showConfiguration();
}
 
void loop() {
  // Process any pending CAN instructions with a 10ms timeout
  CanFrame can_instruction;
  if (xQueueReceive(instruction_queue, &can_instruction, pdMS_TO_TICKS(10)) == pdPASS) {
    Serial.println("Processing CAN instruction...");
    process_instruction(can_instruction);
  }
}
 
// Displays the initial servo configuration
void showConfiguration()
{
  Serial.println("\nServo Configuration:");
  Serial.println("---------------------------------");
  for (int i = 0; i < servoCount; i++)
  {
    Serial.print(servos[i].id);
    Serial.print(" (" + servos[i].action + ") | Min: ");
    Serial.print(servos[i].minAngle);
    Serial.print("° | Max: ");
    Serial.print(servos[i].maxAngle);
    Serial.print("° | Counter: ");
    Serial.println(servos[i].counter);
  }
  Serial.println("---------------------------------");
}
 
// Moves all servos to their origin position
void moveAllToOrigin()
{
  Serial.println("Moving all servos to their origin position...");
  for (int i = 0; i < servoCount; i++)
  {
    servos[i].servo.write(servos[i].minAngle);
    servos[i].atOrigin = true;
    incrementServoCounter(i);
    delay(100); // Small delay between servos
  }
}
 

 

 


// Find servo index by ID (String version)
int findServoIndexByID(String id)
{
  for (int i = 0; i < servoCount; i++)
  {
    if (servos[i].id.equalsIgnoreCase(id))
    {
      return i;
    }
  }
  Serial.print("Error: Servo with ID ");
  Serial.print(id);
  Serial.println(" not found.\n");
  return -1;
}

// Find servo index by numeric ID
int findServoIndexByID(uint8_t id)
{
  for (int i = 0; i < servoCount; i++)
  {
    if (servos[i].id.toInt() == id)
    {
      return i;
    }
  }
  Serial.print("Error: Servo with ID ");
  Serial.print(id);
  Serial.println(" not found.\n");
  return -1;
}

// Task running on core 0 to listen to the TWAI bus
void twai_listener_task(void *pvParameters) {
  Serial.println("CAN listener task started on core 0");
  CanFrame rxFrame;
  
  for (;;) {
    if (ESP32Can.readFrame(rxFrame)) {
      // Only process frames with our device ID
      if (rxFrame.identifier == DEVICE_CAN_ID) {
        Serial.print("CAN Instruction received ID: 0x");
        Serial.println(rxFrame.identifier, HEX);
        
        // Try to send to queue with 10ms timeout
        if (xQueueSend(instruction_queue, &rxFrame, pdMS_TO_TICKS(10)) != pdPASS) {
          Serial.println("Error: Instruction queue is full");
          byte response[8] = {rxFrame.data[0], STATUS_FAIL, 0, 0, 0, 0, 0, 0};
          send_twai_response(response);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to avoid saturating the CPU
  }
}

/**
 * @brief Processes an instruction received from the CAN bus.
 * 
 * This function is called when a new CAN frame is received in the instruction queue.
 * It parses the frame to identify the command and executes it accordingly.
 * 
 * @param instruction The CanFrame object received from the bus.
 * 
 * CAN Frame Data Structure:
 * - data[0]: Command ID
 *   - 0x01: Reset microcontroller
 *   - 0x02: Heartbeat
 *   - 0x05: Read individual servo counter
 *   - 0x06: Reset individual servo counter
 *   - 0x07: Move servo by ID and angle
 * 
 * For command 0x07 (Move servo by ID and angle):
 * - data[1]: Servo ID (0-6)
 * - data[2]: High byte of the angle value
 * - data[3]: Low byte of the angle value
 * - data[4-7]: Not used
 * 
 * For commands 0x05 and 0x06 (Counter operations):
 * - data[1]: Servo index (0-6)
 * - data[2-7]: Not used
 */
void process_instruction(CanFrame instruction)
{
  uint8_t command = instruction.data[0];
  Serial.print("Processing CAN command: 0x");
  Serial.println(command, HEX);

  // Handle commands using switch-case structure
  switch (command)
  {
    case 0x01: // Reset microcontroller
      Serial.println("Resetting microcontroller...");
      {
        byte response[8] = {0x01, STATUS_OK, 0, 0, 0, 0, 0, 0};
        send_twai_response(response);
      }
      delay(100); // Give time for the response to be sent
      ESP.restart();
      break;
      
    case 0x02: // Heartbeat
      Serial.println("Heartbeat request received");
      {
        byte response[8] = {0x02, STATUS_OK, 0, 0, 0, 0, 0, 0};
        send_twai_response(response);
      }
      break;
    
    case 0x05: // Read individual servo counter
      {
        uint8_t servoIndex = instruction.data[1];
        if (servoIndex < servoCount) {
          unsigned long servoCounter = servos[servoIndex].counter;
          Serial.print("Reading counter for servo ");
          Serial.print(servos[servoIndex].id);
          Serial.print(": ");
          Serial.println(servoCounter);
          
          // Prepare response with counter value
          byte response[8] = {0x05, STATUS_OK, 
                              (byte)servoIndex, // Servo index in byte 2
                              (byte)((servoCounter >> 8) & 0xFF), // Highest byte (byte 3)
                              (byte)(servoCounter & 0xFF), // Lowest byte (byte 4)
                              0x00, 0x00, 0x00};
          
          send_twai_response(response);
          Serial.println("Servo counter value sent via CAN.");
        } else {
          byte response[8] = {0x05, STATUS_FAIL, 0, 0, 0, 0, 0, 0};
          send_twai_response(response);
        }
      }
      break;

    case 0x06: // Reset individual servo counter
      {
        uint8_t servoIndex = instruction.data[1];
        if (servoIndex < servoCount) {
          Serial.print("Resetting counter for servo ");
          Serial.println(servos[servoIndex].id);
          servos[servoIndex].counter = 0;
          saveServoCounter(servoIndex);
          byte response[8] = {0x06, STATUS_OK, 0, 0, 0, 0, 0, 0};
          send_twai_response(response);
        } else {
          byte response[8] = {0x06, STATUS_FAIL, 0, 0, 0, 0, 0, 0};
          send_twai_response(response);
        }
      }
      break;
      
    case 0x07: // Move servo by ID and angle
      {
        uint8_t servoID = instruction.data[1]; // Servo ID
        int angle = (instruction.data[2] << 8) | instruction.data[3]; // Angle from high and low bytes
        
        // Find the servo index by ID
        int servoIndex = findServoIndexByID(servoID);
        
        if (servoIndex != -1)
        {
          Serial.print("Moving servo ");
          Serial.print(servoID);
          Serial.print(" to angle ");
          Serial.println(angle);
          
          // Check if angle is within bounds
          int lowerBound = min(servos[servoIndex].minAngle, servos[servoIndex].maxAngle);
          int upperBound = max(servos[servoIndex].minAngle, servos[servoIndex].maxAngle);
          
          if (angle >= lowerBound && angle <= upperBound) {
            servos[servoIndex].servo.write(angle);
            incrementServoCounter(servoIndex);
            byte response[8] = {0x07, STATUS_OK, 0, 0, 0, 0, 0, 0};
            send_twai_response(response);
          } else {
            Serial.println("Error: Angle out of range.");
            byte response[8] = {0x07, STATUS_FAIL, 0, 0, 0, 0, 0, 0};
            send_twai_response(response);
          }
        } else {
          Serial.println("Error: Invalid servo ID.");
          byte response[8] = {0x07, STATUS_FAIL, 0, 0, 0, 0, 0, 0};
          send_twai_response(response);
        }
      }
      break;

    default:
      Serial.println("Unknown command.");
      byte response[8] = {0xFF, 0xFF, 0, 0, 0, 0, 0, 0};
      send_twai_response(response);
      break;
  }
}

// Sends a response frame over the CAN bus
void send_twai_response(byte response_data[8])
{
  CanFrame response_frame;
  response_frame.identifier = RESPONSE_CAN_ID;
  response_frame.extd = 0;
  response_frame.data_length_code = 8; // Always send 8 bytes
  memcpy(response_frame.data, response_data, 8);
  
  ESP32Can.writeFrame(response_frame);
  Serial.println("CAN response sent.");
}