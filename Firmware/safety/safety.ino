/*
 * Safety System with CAN Bus Integration
 * 
 * CAN Command Reference:
 * 0x01 - Reset Micro (System health check)
 * 0x02 - Heartbeat (Verify system responsiveness)
 * 0x03 - Get Door Status (Returns door open/closed states)
 * 0x04 - Control Andon Tower Light (Red, Yellow, Green, Buzzer)
 * 0x05 - Get Locking Touch Sensor Status
 * 0x06 - Control Attachment Mechanisms
 * 0x07 - Get Master Lock Status (All sensors locked)
 * 0xFF - Emergency Stop (Deactivate all outputs)
 */

#include <ESP32-TWAI-CAN.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// CAN Bus Configuration
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x020
#define RESPONSE_CAN_ID 0x420

// Queue to store instructions received from the CAN bus
QueueHandle_t instruction_queue;

// Acknowledgment control
bool enableAckMessages = false;

// Door sensor pins (6 digital inputs)
const int doorSensor1Pin = 32;
const int doorSensor2Pin = 33;
const int doorSensor3Pin = 34;
const int doorSensor4Pin = 35;
const int doorSensor5Pin = 39;
const int doorSensor6Pin = 36;

// Andon tower light pins (4 outputs)
const int redLightPin = 12;
const int yellowLightPin = 13;
const int greenLightPin = 26;
const int buzzerPin = 27;

// Locking touch sensor pins (4 inputs)
const int lockingTouchSensor1Pin = 16;
const int lockingTouchSensor2Pin = 18;
const int lockingTouchSensor3Pin = 21;
const int lockingTouchSensor4Pin = 23;

// Attachment mechanism pins (4 outputs)
const int attachmentMech1Pin = 17;
const int attachmentMech2Pin = 19;
const int attachmentMech3Pin = 22;
const int attachmentMech4Pin = 25;

// Master lock sensor (all sensors locked)
const int masterLockSensorPin = 14;

// Door sensor states
bool doorSensor1Open = false;
bool doorSensor2Open = false;
bool doorSensor3Open = false;
bool doorSensor4Open = false;
bool doorSensor5Open = false;
bool doorSensor6Open = false;

// Locking touch sensor states
bool lockingTouchSensor1Active = false;
bool lockingTouchSensor2Active = false;
bool lockingTouchSensor3Active = false;
bool lockingTouchSensor4Active = false;

// Attachment mechanism states
bool attachmentMech1Active = false;
bool attachmentMech2Active = false;
bool attachmentMech3Active = false;
bool attachmentMech4Active = false;

// Master lock status
bool masterLockActive = false;

void setup()
{
  Serial.begin(115200);
  Serial.println("Safety System Starting...");
  
  // Initialize door sensor pins as inputs with pull-up
  pinMode(doorSensor1Pin, INPUT_PULLUP);
  pinMode(doorSensor2Pin, INPUT_PULLUP);
  pinMode(doorSensor3Pin, INPUT_PULLUP);
  pinMode(doorSensor4Pin, INPUT_PULLUP);
  pinMode(doorSensor5Pin, INPUT_PULLUP);
  pinMode(doorSensor6Pin, INPUT_PULLUP);
  
  // Initialize andon tower light pins as outputs
  pinMode(redLightPin, OUTPUT);
  pinMode(yellowLightPin, OUTPUT);
  pinMode(greenLightPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  
  // Initialize locking touch sensor pins as inputs with pull-up
  pinMode(lockingTouchSensor1Pin, INPUT_PULLUP);
  pinMode(lockingTouchSensor2Pin, INPUT_PULLUP);
  pinMode(lockingTouchSensor3Pin, INPUT_PULLUP);
  pinMode(lockingTouchSensor4Pin, INPUT_PULLUP);
  
  // Initialize attachment mechanism pins as outputs
  pinMode(attachmentMech1Pin, OUTPUT);
  pinMode(attachmentMech2Pin, OUTPUT);
  pinMode(attachmentMech3Pin, OUTPUT);
  pinMode(attachmentMech4Pin, OUTPUT);
  
  // Initialize master lock sensor pin as input with pull-up
  pinMode(masterLockSensorPin, INPUT_PULLUP);
  
  // Set initial states - all outputs OFF
  digitalWrite(redLightPin, LOW);
  digitalWrite(yellowLightPin, LOW);
  digitalWrite(greenLightPin, LOW);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(attachmentMech1Pin, LOW);
  digitalWrite(attachmentMech2Pin, LOW);
  digitalWrite(attachmentMech3Pin, LOW);
  digitalWrite(attachmentMech4Pin, LOW);
  
  // Initialize CAN bus
  ESP32Can.setPins(CAN0_TX, CAN0_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
  if (ESP32Can.begin()) {
    Serial.println("CAN bus initialized successfully");
  } else {
    Serial.println("CAN bus initialization failed");
  }
  
  // Create instruction queue
  instruction_queue = xQueueCreate(10, sizeof(CanFrame));
  if (instruction_queue == NULL) {
    Serial.println("Error: Could not create instruction queue.");
  }
  
  // Create TWAI listener task on core 0
  xTaskCreatePinnedToCore(
    twai_listener_task,
    "TWAI Listener",
    4096,
    NULL,
    1,
    NULL,
    0
  );
  
  Serial.println("\n=== CAN Command Reference ===");
  Serial.println("0x01 - Reset Micro");
  Serial.println("0x02 - Heartbeat");
  Serial.println("0x03 - Get Door Status");
  Serial.println("0x04 - Control Andon Tower Light");
  Serial.println("0x05 - Get Locking Touch Sensor Status");
  Serial.println("0x06 - Control Attachment Mechanisms");
  Serial.println("0x07 - Get Master Lock Status");
  Serial.println("0xFF - Emergency Stop");
  Serial.println("==============================\n");
  
  Serial.println("Safety System Ready");
}

void loop()
{
  // Process CAN instructions
  CanFrame instruction;
  if (xQueueReceive(instruction_queue, &instruction, (TickType_t)0) == pdPASS) {
    process_can_instruction(instruction);
  }
  
  // Read door sensors (LOW = door open, HIGH = door closed)
  int door1State = digitalRead(doorSensor1Pin);
  int door2State = digitalRead(doorSensor2Pin);
  int door3State = digitalRead(doorSensor3Pin);
  int door4State = digitalRead(doorSensor4Pin);
  int door5State = digitalRead(doorSensor5Pin);
  int door6State = digitalRead(doorSensor6Pin);
  
  // Update door sensor states
  if (door1State == LOW && !doorSensor1Open) {
    Serial.println("Door 1 opened");
    doorSensor1Open = true;
  } else if (door1State == HIGH && doorSensor1Open) {
    Serial.println("Door 1 closed");
    doorSensor1Open = false;
  }
  
  if (door2State == LOW && !doorSensor2Open) {
    Serial.println("Door 2 opened");
    doorSensor2Open = true;
  } else if (door2State == HIGH && doorSensor2Open) {
    Serial.println("Door 2 closed");
    doorSensor2Open = false;
  }
  
  if (door3State == LOW && !doorSensor3Open) {
    Serial.println("Door 3 opened");
    doorSensor3Open = true;
  } else if (door3State == HIGH && doorSensor3Open) {
    Serial.println("Door 3 closed");
    doorSensor3Open = false;
  }
  
  if (door4State == LOW && !doorSensor4Open) {
    Serial.println("Door 4 opened");
    doorSensor4Open = true;
  } else if (door4State == HIGH && doorSensor4Open) {
    Serial.println("Door 4 closed");
    doorSensor4Open = false;
  }
  
  if (door5State == LOW && !doorSensor5Open) {
    Serial.println("Door 5 opened");
    doorSensor5Open = true;
  } else if (door5State == HIGH && doorSensor5Open) {
    Serial.println("Door 5 closed");
    doorSensor5Open = false;
  }
  
  if (door6State == LOW && !doorSensor6Open) {
    Serial.println("Door 6 opened");
    doorSensor6Open = true;
  } else if (door6State == HIGH && doorSensor6Open) {
    Serial.println("Door 6 closed");
    doorSensor6Open = false;
  }
  
  // Read locking touch sensors (HIGH = touching, LOW = not touching)
  int lockTouch1State = digitalRead(lockingTouchSensor1Pin);
  int lockTouch2State = digitalRead(lockingTouchSensor2Pin);
  int lockTouch3State = digitalRead(lockingTouchSensor3Pin);
  int lockTouch4State = digitalRead(lockingTouchSensor4Pin);
  
  // Update locking touch sensor states
  if (lockTouch1State == HIGH && !lockingTouchSensor1Active) {
    Serial.println("Locking Touch Sensor 1 activated");
    lockingTouchSensor1Active = true;
  } else if (lockTouch1State == LOW && lockingTouchSensor1Active) {
    Serial.println("Locking Touch Sensor 1 deactivated");
    lockingTouchSensor1Active = false;
  }
  
  if (lockTouch2State == HIGH && !lockingTouchSensor2Active) {
    Serial.println("Locking Touch Sensor 2 activated");
    lockingTouchSensor2Active = true;
  } else if (lockTouch2State == LOW && lockingTouchSensor2Active) {
    Serial.println("Locking Touch Sensor 2 deactivated");
    lockingTouchSensor2Active = false;
  }
  
  if (lockTouch3State == HIGH && !lockingTouchSensor3Active) {
    Serial.println("Locking Touch Sensor 3 activated");
    lockingTouchSensor3Active = true;
  } else if (lockTouch3State == LOW && lockingTouchSensor3Active) {
    Serial.println("Locking Touch Sensor 3 deactivated");
    lockingTouchSensor3Active = false;
  }
  
  if (lockTouch4State == HIGH && !lockingTouchSensor4Active) {
    Serial.println("Locking Touch Sensor 4 activated");
    lockingTouchSensor4Active = true;
  } else if (lockTouch4State == LOW && lockingTouchSensor4Active) {
    Serial.println("Locking Touch Sensor 4 deactivated");
    lockingTouchSensor4Active = false;
  }
  
  // Read master lock sensor (HIGH = everything locked)
  int masterLockState = digitalRead(masterLockSensorPin);
  if (masterLockState == HIGH && !masterLockActive) {
    Serial.println("Master Lock: All sensors locked");
    masterLockActive = true;
  } else if (masterLockState == LOW && masterLockActive) {
    Serial.println("Master Lock: Not all sensors locked");
    masterLockActive = false;
  }
  
  delay(50); // Small delay to avoid excessive processing
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

// Process CAN instructions
void process_can_instruction(CanFrame instruction)
{
  Serial.print("Processing CAN command: 0x");
  Serial.println(instruction.data[0], HEX);

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
    // Get Door Status
    case 0x03:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x03: Get Door Status");
      byte door_states = 0;
      if (doorSensor1Open) door_states |= 0x01;
      if (doorSensor2Open) door_states |= 0x02;
      if (doorSensor3Open) door_states |= 0x04;
      if (doorSensor4Open) door_states |= 0x08;
      if (doorSensor5Open) door_states |= 0x10;
      if (doorSensor6Open) door_states |= 0x20;
      
      byte response[] = {0x03, 0x01, door_states, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Control Andon Tower Light
    case 0x04:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x04: Control Andon Tower Light");
      
      // instruction.data[1] contains the light control byte
      // Bit 0: Red Light, Bit 1: Yellow Light, Bit 2: Green Light, Bit 3: Buzzer
      digitalWrite(redLightPin, (instruction.data[1] & 0x01) ? HIGH : LOW);
      digitalWrite(yellowLightPin, (instruction.data[1] & 0x02) ? HIGH : LOW);
      digitalWrite(greenLightPin, (instruction.data[1] & 0x04) ? HIGH : LOW);
      digitalWrite(buzzerPin, (instruction.data[1] & 0x08) ? HIGH : LOW);
      
      byte response[] = {0x04, 0x01, instruction.data[1], 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x05 ***************************** //
    // Get Locking Touch Sensor Status
    case 0x05:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x05: Get Locking Touch Sensor Status");
      byte touch_states = 0;
      if (lockingTouchSensor1Active) touch_states |= 0x01;
      if (lockingTouchSensor2Active) touch_states |= 0x02;
      if (lockingTouchSensor3Active) touch_states |= 0x04;
      if (lockingTouchSensor4Active) touch_states |= 0x08;
      
      byte response[] = {0x05, 0x01, touch_states, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x06 ***************************** //
    // Control Attachment Mechanisms
    case 0x06:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x06: Control Attachment Mechanisms");
      
      // instruction.data[1] contains the attachment control byte
      // Bit 0: Attachment 1, Bit 1: Attachment 2, Bit 2: Attachment 3, Bit 3: Attachment 4
      digitalWrite(attachmentMech1Pin, (instruction.data[1] & 0x01) ? HIGH : LOW);
      digitalWrite(attachmentMech2Pin, (instruction.data[1] & 0x02) ? HIGH : LOW);
      digitalWrite(attachmentMech3Pin, (instruction.data[1] & 0x04) ? HIGH : LOW);
      digitalWrite(attachmentMech4Pin, (instruction.data[1] & 0x08) ? HIGH : LOW);
      
      // Update attachment states
      attachmentMech1Active = (instruction.data[1] & 0x01) ? true : false;
      attachmentMech2Active = (instruction.data[1] & 0x02) ? true : false;
      attachmentMech3Active = (instruction.data[1] & 0x04) ? true : false;
      attachmentMech4Active = (instruction.data[1] & 0x08) ? true : false;
      
      byte response[] = {0x06, 0x01, instruction.data[1], 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x07 ***************************** //
    // Get Master Lock Status
    case 0x07:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x07: Get Master Lock Status");
      byte master_lock_state = masterLockActive ? 0x01 : 0x00;
      
      byte response[] = {0x07, 0x01, master_lock_state, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Emergency Stop - Deactivate all outputs
    case 0xFF:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0xFF: Emergency Stop - Deactivating all outputs");
      
      // Turn off all andon tower lights and buzzer
      digitalWrite(redLightPin, LOW);
      digitalWrite(yellowLightPin, LOW);
      digitalWrite(greenLightPin, LOW);
      digitalWrite(buzzerPin, LOW);
      
      // Turn off all attachment mechanisms
      digitalWrite(attachmentMech1Pin, LOW);
      digitalWrite(attachmentMech2Pin, LOW);
      digitalWrite(attachmentMech3Pin, LOW);
      digitalWrite(attachmentMech4Pin, LOW);
      
      // Update attachment states
      attachmentMech1Active = false;
      attachmentMech2Active = false;
      attachmentMech3Active = false;
      attachmentMech4Active = false;
      
      byte response[] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    default:
    {
      Serial.print("Unknown command: 0x");
      Serial.println(instruction.data[0], HEX);
      
      // Send error response
      byte response[] = {instruction.data[0], 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;
  }
}

// Send response via TWAI
void send_twai_response(byte data[])
{
  CanFrame txFrame;
  txFrame.identifier = RESPONSE_CAN_ID;
  txFrame.extd = 0;
  txFrame.data_length_code = 8;
  
  for (int i = 0; i < 8; i++) {
    txFrame.data[i] = data[i];
  }
  
  if (ESP32Can.writeFrame(txFrame)) {
    Serial.print("Response sent: ");
    for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      Serial.print(txFrame.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Failed to send response");
  }
}
 