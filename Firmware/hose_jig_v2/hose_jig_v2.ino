/*
 * CASE DESCRIPTIONS (Servos-only version):
 * 
 * 0x01: Reset microcontroller
 * 0x02: Send ping
 * 0x05: Move servos to open position
 * 0x06: Move servos to close position
 * 0x08: Get servo movement counter
 * 0x09: Reset servo movement counter
 * 0x0A: Move servos to absolute position
 * 0x10: Update SERVO_OPEN_ANGLE
 * 0x11: Update SERVO_CLOSE_ANGLE
 * 0x14: Read SERVO_OPEN_ANGLE
 * 0x15: Read SERVO_CLOSE_ANGLE
 * 0x20: Move Hose Holder servos (Pins 21, 22)
 * 0xFF: Power off - Move all servos to home position
 */

#include <ESP32-TWAI-CAN.hpp>
#include <ESP32Servo.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <EEPROM.h>  // For flash memory storage

// EEPROM Configuration (dual jig)
#define EEPROM_SIZE 24  // Storage for two counters and two jig angle pairs
#define COUNTER_JIG1_ADDR 0
#define COUNTER_JIG2_ADDR 4
#define SERVO_OPEN_ANGLE_JIG1_ADDR 8
#define SERVO_CLOSE_ANGLE_JIG1_ADDR 10
#define SERVO_OPEN_ANGLE_JIG2_ADDR 12
#define SERVO_CLOSE_ANGLE_JIG2_ADDR 14

// 78 hex open 0 close 

// Global variables
unsigned long servoMoveCounterJig[2] = {0, 0};

// Pins for the first CAN bus (TWAI - ESP32 Integrated)
#define CAN0_TX 4  // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5  // GPIO5 - RX for CAN0 (General Network)

// (MCP2515 removed in servos-only version)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x0CB
#define RESPONSE_CAN_ID 0x4CB

// (MCP2515 and LinearActuator removed)

// Servo objects (dual jig: 2 x 3 servos)
ESP32PWM pwm;
Servo jig1_servos[3];
Servo jig2_servos[3];
Servo hose_holder_servos[2];

// Servo pins (Jig 1 and Jig 2)
const int JIG1_SERVO_PINS[3] = {14, 15, 16};
const int JIG2_SERVO_PINS[3] = {17, 18, 19};
const int HOSE_HOLDER_SERVO_PINS[2] = {21, 22};

// Inductive sensors (Jig 1 and Jig 2)
const int JIG1_SENSOR_PINS[3] = {23, 25, 26};
const int JIG2_SENSOR_PINS[3] = {27, 32, 33};

// Per-jig configurable angles (default values)
int SERVO_OPEN_ANGLE_JIG[2] = {4, 4};
int SERVO_CLOSE_ANGLE_JIG[2] = {70, 70};

void saveConfigValues() {
  // Verify addresses are within bounds
  if (SERVO_OPEN_ANGLE_JIG1_ADDR + 2 > EEPROM_SIZE ||
      SERVO_CLOSE_ANGLE_JIG1_ADDR + 2 > EEPROM_SIZE ||
      SERVO_OPEN_ANGLE_JIG2_ADDR + 2 > EEPROM_SIZE ||
      SERVO_CLOSE_ANGLE_JIG2_ADDR + 2 > EEPROM_SIZE) {
    Serial.println("Error: EEPROM address out of bounds");
    return;
  }
  // Write servo config values for both jigs
  EEPROM.writeUShort(SERVO_OPEN_ANGLE_JIG1_ADDR, SERVO_OPEN_ANGLE_JIG[0]);
  EEPROM.writeUShort(SERVO_CLOSE_ANGLE_JIG1_ADDR, SERVO_CLOSE_ANGLE_JIG[0]);
  EEPROM.writeUShort(SERVO_OPEN_ANGLE_JIG2_ADDR, SERVO_OPEN_ANGLE_JIG[1]);
  EEPROM.writeUShort(SERVO_CLOSE_ANGLE_JIG2_ADDR, SERVO_CLOSE_ANGLE_JIG[1]);
  // Commit changes to EEPROM
  if (!EEPROM.commit()) {
    Serial.println("Error: EEPROM commit failed");
  } else {
    Serial.println("Servo config values saved to EEPROM (both jigs)");
  }
}

void loadConfigValues() {
  // Initialize with default values
  uint16_t defaultOpenAngle = 4;
  uint16_t defaultCloseAngle = 70;
  
  // Try to read from EEPROM
  SERVO_OPEN_ANGLE_JIG[0] = EEPROM.readUShort(SERVO_OPEN_ANGLE_JIG1_ADDR);
  SERVO_CLOSE_ANGLE_JIG[0] = EEPROM.readUShort(SERVO_CLOSE_ANGLE_JIG1_ADDR);
  SERVO_OPEN_ANGLE_JIG[1] = EEPROM.readUShort(SERVO_OPEN_ANGLE_JIG2_ADDR);
  SERVO_CLOSE_ANGLE_JIG[1] = EEPROM.readUShort(SERVO_CLOSE_ANGLE_JIG2_ADDR);
  
  // Check if values are uninitialized (0xFFFF)
  bool needsSave = false;
  for (int j = 0; j < 2; j++) {
    if (SERVO_OPEN_ANGLE_JIG[j] == 0xFFFF) {
      SERVO_OPEN_ANGLE_JIG[j] = defaultOpenAngle;
      needsSave = true;
    }
    if (SERVO_CLOSE_ANGLE_JIG[j] == 0xFFFF) {
      SERVO_CLOSE_ANGLE_JIG[j] = defaultCloseAngle;
      needsSave = true;
    }
  }
  // Save default values if any were uninitialized
  if (needsSave) {
    saveConfigValues();
    Serial.println("Initialized EEPROM with default servo values for both jigs");
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

// Function to save servo counter to EEPROM (per jig)
void saveCounter(uint8_t jig) {
  if (jig == 0) {
    EEPROM.writeULong(COUNTER_JIG1_ADDR, servoMoveCounterJig[0]);
  } else if (jig == 1) {
    EEPROM.writeULong(COUNTER_JIG2_ADDR, servoMoveCounterJig[1]);
  }
  EEPROM.commit();
  Serial.print("Servo counter saved (jig "); Serial.print(jig+1); Serial.print("): ");
  Serial.println(jig == 0 ? servoMoveCounterJig[0] : servoMoveCounterJig[1]);
}

// Function to save actuator counter to EEPROM (servos-only: not supported)
void saveActuatorCounter() {
  Serial.println("Actuator counter not supported in v2 (servos-only)");
}

// Function to increment and save the servo counter (per jig)
void incrementCounter(uint8_t jig) {
  if (jig > 1) return;
  servoMoveCounterJig[jig]++;
  saveCounter(jig);
}

// Function to increment and save the actuator counter (servos-only: no-op)
void incrementActuatorCounter() {
  // no-op
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing dual CAN system with FreeRTOS");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load counters from EEPROM (per jig)
  servoMoveCounterJig[0] = EEPROM.readULong(COUNTER_JIG1_ADDR);
  servoMoveCounterJig[1] = EEPROM.readULong(COUNTER_JIG2_ADDR);
  Serial.print("Loaded servo move counter (Jig 1): "); Serial.println(servoMoveCounterJig[0]);
  Serial.print("Loaded servo move counter (Jig 2): "); Serial.println(servoMoveCounterJig[1]);

  // Load persistent config values
  loadConfigValues();
  Serial.print("Loaded SERVO_OPEN_ANGLE J1: "); Serial.println(SERVO_OPEN_ANGLE_JIG[0]);
  Serial.print("Loaded SERVO_CLOSE_ANGLE J1: "); Serial.println(SERVO_CLOSE_ANGLE_JIG[0]);
  Serial.print("Loaded SERVO_OPEN_ANGLE J2: "); Serial.println(SERVO_OPEN_ANGLE_JIG[1]);
  Serial.print("Loaded SERVO_CLOSE_ANGLE J2: "); Serial.println(SERVO_CLOSE_ANGLE_JIG[1]);
  // Servos-only: actuator positions removed

  // Configure PWM for servos
  ESP32PWM::timerCount[0] = 0; // Reset timer count
  pwm.attachPin(0, 10000, 12); // 10kHz, 12-bit resolution
  
  // Attach Servo motors for both jigs with ESP32Servo
  for (int i = 0; i < 3; i++) {
    jig1_servos[i].setPeriodHertz(50);
    jig1_servos[i].attach(JIG1_SERVO_PINS[i], 500, 2500);
    jig2_servos[i].setPeriodHertz(50);
    jig2_servos[i].attach(JIG2_SERVO_PINS[i], 500, 2500);
  }

  // Attach Hose Holder servos (Pins 21, 22)
  for (int i = 0; i < 2; i++) {
    hose_holder_servos[i].setPeriodHertz(50);
    hose_holder_servos[i].attach(HOSE_HOLDER_SERVO_PINS[i], 500, 2500);
  }

  // Configure inductive sensors as inputs with pullup for both jigs
  for (int i = 0; i < 3; i++) {
    pinMode(JIG1_SENSOR_PINS[i], INPUT_PULLUP);
    pinMode(JIG2_SENSOR_PINS[i], INPUT_PULLUP);
  }

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

  // Servos-only: MCP2515 initialization removed

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

  // Servos to initial Open Position (per jig)
  for (int i = 0; i < 3; i++) {
    jig1_servos[i].write(SERVO_OPEN_ANGLE_JIG[0]);
    jig2_servos[i].write(SERVO_OPEN_ANGLE_JIG[1]);
  }
  
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
      Serial.println("Case 0x01: Reset microcontroller / Reiniciar microcontrolador / マイコンをリセット");
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
      Serial.println("Case 0x02: Ping / Señal / ピング");
      byte response[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x03 ***************************** //
    // Home actuator (not supported in v2)
    case 0x03:
    {
      Serial.println("Case 0x03: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Move actuator to absolute position (not supported)
    case 0x04:
    {
      Serial.println("Case 0x04: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x05 ***************************** //
    // Move Servos to Open Position
    case 0x05:
    {
      // data[1]: jig selector (0=ambos, 1=jig1, 2=jig2). Si falta, se asume ambos.
      uint8_t jigSel = instruction.data[1];
      Serial.println("Case 0x05: Moving servos to open position / Abrir / オープン");
      if (jigSel == 1) {
        for (int i = 0; i < 3; i++) jig1_servos[i].write(SERVO_OPEN_ANGLE_JIG[0]);
        incrementCounter(0);
      } else if (jigSel == 2) {
        for (int i = 0; i < 3; i++) jig2_servos[i].write(SERVO_OPEN_ANGLE_JIG[1]);
        incrementCounter(1);
      } else {
        for (int i = 0; i < 3; i++) {
          jig1_servos[i].write(SERVO_OPEN_ANGLE_JIG[0]);
          jig2_servos[i].write(SERVO_OPEN_ANGLE_JIG[1]);
        }
        incrementCounter(0);
        incrementCounter(1);
      }
      delay(1000);
      byte response[] = {0x05, 0x01, jigSel, (uint8_t)SERVO_OPEN_ANGLE_JIG[jigSel == 2 ? 1 : 0], 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;
 
    // ***************************** CASE 0x06 ***************************** //
    // Move Servos to Close Position
    case 0x06:
    {
      // data[1]: jig selector (0=ambos, 1=jig1, 2=jig2). Si falta, se asume ambos.
      uint8_t jigSel = instruction.data[1];
      Serial.println("Case 0x06: Moving servos to close position / Cerrar / クローズ");
      if (jigSel == 1) {
        for (int i = 0; i < 3; i++) jig1_servos[i].write(SERVO_CLOSE_ANGLE_JIG[0]);
        incrementCounter(0);
      } else if (jigSel == 2) {
        for (int i = 0; i < 3; i++) jig2_servos[i].write(SERVO_CLOSE_ANGLE_JIG[1]);
        incrementCounter(1);
      } else {
        for (int i = 0; i < 3; i++) {
          jig1_servos[i].write(SERVO_CLOSE_ANGLE_JIG[0]);
          jig2_servos[i].write(SERVO_CLOSE_ANGLE_JIG[1]);
        }
        incrementCounter(0);
        incrementCounter(1);
      }
      delay(500);
      byte response[] = {0x06, 0x01, jigSel, (uint8_t)SERVO_CLOSE_ANGLE_JIG[jigSel == 2 ? 1 : 0], 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Get servo movement counter
    case 0x08:
    {
      Serial.println("Case 0x08: Reading servo movement counter / Leer contador / カウンタを読む");
      uint8_t jigSel = instruction.data[1];
      uint32_t cnt = 0;
      if (jigSel == 1) cnt = servoMoveCounterJig[0];
      else if (jigSel == 2) cnt = servoMoveCounterJig[1];
      else cnt = servoMoveCounterJig[0] + servoMoveCounterJig[1];
      byte counter_high = (cnt >> 8) & 0xFF;
      byte counter_low = cnt & 0xFF;
      byte response[] = {0x08, 0x01, jigSel, counter_high, counter_low, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x09 ***************************** //
    // Reset servo movement counter
    case 0x09:
    {
      Serial.println("Case 0x09: Resetting servo movement counter / Reiniciar contador / カウンタをリセット");
      uint8_t jigSel = instruction.data[1];
      if (jigSel == 1) {
        servoMoveCounterJig[0] = 0; saveCounter(0);
      } else if (jigSel == 2) {
        servoMoveCounterJig[1] = 0; saveCounter(1);
      } else {
        servoMoveCounterJig[0] = 0; saveCounter(0);
        servoMoveCounterJig[1] = 0; saveCounter(1);
      }
      byte response[] = {0x09, 0x01, jigSel, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    
    // ***************************** CASE 0x0A ***************************** //
    // Move Servo to Absolute Position
    case 0x1A:
    {

      Serial.println("Case 0x0A: Moving servos to absolute position / Posición absoluta / 絶対位置へ移動");
      // Nuevo formato: data[1]=jig (0 ambos,1 j1,2 j2), data[2]=posición 0..180
      // Compat: si data[2]==0 y data[1] en 0..180, aplica ambos con posición=data[1]
      uint8_t jigSel = instruction.data[1];
      uint8_t position = instruction.data[2];
      if (position == 0 && jigSel <= 180) {
        position = jigSel; // compat con firma antigua
        jigSel = 0;        // aplicar ambos
      }
      position = constrain(position, 0, 180);
      if (jigSel == 1) {
        for (int i = 0; i < 3; i++) jig1_servos[i].write(position);
        incrementCounter(0);
      } else if (jigSel == 2) {
        for (int i = 0; i < 3; i++) jig2_servos[i].write(position);
        incrementCounter(1);
      } else {
        for (int i = 0; i < 3; i++) { jig1_servos[i].write(position); jig2_servos[i].write(position); }
        incrementCounter(0);
        incrementCounter(1);
      }

      delay(500);
      byte response[] = {0x1A, 0x01, jigSel, position, 0x00, 0x00, 0x00, 0x00}; 
      send_twai_response(response);
    }
    break;


     // ***************************** CASE 0x0B ***************************** //
    // Move Actuator to Insertion Position (not supported)
    case 0x0B:
    {
      Serial.println("Case 0x0B: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x0B, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0C ***************************** //
    // Move Actuator to Deliver Position (not supported)
    case 0x0C:
    {
      Serial.println("Case 0x0C: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x0C, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    
    // ***************************** CASE 0x0D ***************************** //
    // Read actuator movement counter (not supported)
    case 0x0D:
    {
      Serial.println("Case 0x0D: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x0D, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0E ***************************** //
    // Reset actuator movement counter (not supported)
    case 0x0E:
    {
      Serial.println("Case 0x0E: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x0E, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;


    // ***************************** CASE 0x10 ***************************** //
    // Update SERVO_OPEN_ANGLE
    case 0x10:
    {
      // Mantiene firma: value en data[1..2]; jig opcional en data[3] (1 o 2)
      uint16_t value = (instruction.data[1] << 8) | instruction.data[2];
      value = constrain(value, 0, 180);
      uint8_t jigSel = instruction.data[3];
      if (jigSel == 1) {
        SERVO_OPEN_ANGLE_JIG[0] = value;
      } else if (jigSel == 2) {
        SERVO_OPEN_ANGLE_JIG[1] = value;
      } else {
        SERVO_OPEN_ANGLE_JIG[0] = value;
        SERVO_OPEN_ANGLE_JIG[1] = value;
      }
      saveConfigValues();
      Serial.print("Updated SERVO_OPEN_ANGLE (jig "); Serial.print(jigSel == 0 ? 0 : jigSel); Serial.print(") to: "); Serial.println(value);
      byte response[] = {0x10, 0x01, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF), jigSel, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x11 ***************************** //
    // Update SERVO_CLOSE_ANGLE
    case 0x11:
    {
      // Mantiene firma: value en data[1..2]; jig opcional en data[3] (1 o 2)
      uint16_t value = (instruction.data[1] << 8) | instruction.data[2];
      value = constrain(value, 0, 180);
      uint8_t jigSel = instruction.data[3];
      if (jigSel == 1) {
        SERVO_CLOSE_ANGLE_JIG[0] = value;
      } else if (jigSel == 2) {
        SERVO_CLOSE_ANGLE_JIG[1] = value;
      } else {
        SERVO_CLOSE_ANGLE_JIG[0] = value;
        SERVO_CLOSE_ANGLE_JIG[1] = value;
      }
      saveConfigValues();
      Serial.print("Updated SERVO_CLOSE_ANGLE (jig "); Serial.print(jigSel == 0 ? 0 : jigSel); Serial.print(") to: "); Serial.println(value);
      byte response[] = {0x11, 0x01, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF), jigSel, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x12 ***************************** //
    // Update ACTUATOR_DELIVER_POSITION (not supported)
    case 0x12:
    {
      Serial.println("Case 0x12: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x12, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x13 ***************************** //
    // Update ACTUATOR_INSERTION_POSITION (not supported)
    case 0x13:
    {
      Serial.println("Case 0x13: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x14 ***************************** //
    // Read SERVO_OPEN_ANGLE
    case 0x14:
    {
      Serial.println("Case 0x14: Reading SERVO_OPEN_ANGLE / Leer / オープン角");
      uint8_t jigSel = instruction.data[1];
      int val = SERVO_OPEN_ANGLE_JIG[(jigSel == 2) ? 1 : 0];
      byte high = (val >> 8) & 0xFF;
      byte low = val & 0xFF;
      byte response[] = {0x14, 0x01, jigSel, high, low, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x15 ***************************** //
    // Read SERVO_CLOSE_ANGLE
    case 0x15:
    {
      Serial.println("Case 0x15: Reading SERVO_CLOSE_ANGLE / Leer / クローズ角");
      uint8_t jigSel = instruction.data[1];
      int val = SERVO_CLOSE_ANGLE_JIG[(jigSel == 2) ? 1 : 0];
      byte high = (val >> 8) & 0xFF;
      byte low = val & 0xFF;
      byte response[] = {0x15, 0x01, jigSel, high, low, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x16 ***************************** //
    // Read ACTUATOR_DELIVER_POSITION (not supported)
    case 0x16:
    {
      Serial.println("Case 0x16: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x16, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x17 ***************************** //
    // Read ACTUATOR_INSERTION_POSITION (not supported)
    case 0x17:
    {
      Serial.println("Case 0x17: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x17, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;    

    // ***************************** CASE 0x18 ***************************** //
    // Home actuator using go_home function (not supported)
    case 0x18:
    {
      Serial.println("Case 0x18: Actuator not supported in v2 / Actuador no soportado / アクチュエータ非対応");
      byte response[] = {0x18, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x20 ***************************** //
    // Move Hose Holder Servos (Pins 21, 22)
    case 0x20:
    {
      Serial.println("Case 0x20: Move Hose Holder Servos / Mover Hose Holder");
      uint8_t selector = instruction.data[1]; // 0=Both, 1=Servo1(21), 2=Servo2(22)
      uint8_t angle = instruction.data[2];
      angle = constrain(angle, 0, 180);

      if (selector == 1) {
        hose_holder_servos[0].write(angle);
      } else if (selector == 2) {
        hose_holder_servos[1].write(angle);
      } else {
        // Default to both
        hose_holder_servos[0].write(angle);
        hose_holder_servos[1].write(angle);
      }
      
      delay(200); // Small delay for movement
      byte response[] = {0x20, 0x01, selector, angle, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Power off - Move all to home position
    case 0xFF:
    {
      Serial.println("Case 0xFF: Power off - Move servos home / Apagar - Servos a home / 電源OFF - サーボをホームへ");
      // Move all servos (both jigs) to home position (0)
      for (int i = 0; i < 3; i++) { jig1_servos[i].write(0); jig2_servos[i].write(0); }
      incrementCounter(0);
      incrementCounter(1);
      delay(1000);
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

// Helper function to flush CAN buffer (servos-only: MCP2515 removed)
void flushCanBuffer() {
  // No-op in servos-only version
}

// Helper function to wait for CAN reply (servos-only: MCP2515 removed)
uint8_t waitForCanReply(uint16_t expectedId) {
  // Not used in servos-only version; return NO LOCAL NETWORK
  return 0x04;
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