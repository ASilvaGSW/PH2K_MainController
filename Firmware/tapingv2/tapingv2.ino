/*
 * TapingV2 - CAN (TWAI) minimal integration
 * English: Initializes ESP32 TWAI CAN, listens for instructions, and responds. Implements default cases 0x01 (reset) and 0x02 (heartbeat).
 * Español: Inicializa CAN TWAI en ESP32, escucha instrucciones y responde. Implementa los casos por defecto 0x01 (reset) y 0x02 (heartbeat).
 * 日本語: ESP32 の TWAI CAN を初期化し、指示を受信・返信します。デフォルトの 0x01（リセット）と 0x02（ハートビート）を実装します。
 */

#include <ESP32-TWAI-CAN.hpp>
#include <ESP32Servo.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/////////////////////
// CAN Bus TWAI Pins//
/////////////////////
#define CAN0_TX 4   // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5   // GPIO5 - RX for CAN0 (General Network)

// Device CAN IDs (keep consistent with taping.ino)
#define DEVICE_CAN_ID 0x00A   // Taping device CAN ID
#define RESPONSE_CAN_ID 0x40A // Response CAN ID

// English: Define pins and create servo objects for feeder, wrapper, cutter, holder, gripper, elevator.
const int FEEDER_PIN   = 2;
const int WRAPPER_PIN  = 25;
const int CUTTER_PIN   = 15;
const int HOLDER_PIN   = 14;
const int GRIPPER_PIN  = 19;
const int ELEVATOR_PIN = 17;

Servo feederServo;
Servo wrapperServo;
Servo cutterServo;
Servo holderServo;
Servo gripperServo;
Servo elevatorServo;

// Home positions (microseconds)
// English: Define home microsecond values for each servo
const int FEEDER_HOME_US   = 1500;
const int WRAPPER_HOME_US  = 1500;
const int CUTTER_HOME_US   = 500;
const int HOLDER_HOME_US   = 1500;
const int GRIPPER_HOME_US  = 2000;
const int ELEVATOR_HOME_US = 700;
// Track last commanded elevator position to avoid redundant moves
int elevator_current_us = ELEVATOR_HOME_US; // English: state; Español: estado; 日本語: 状態

// Queue to store instructions received from the CAN bus (TWAI)
QueueHandle_t instruction_queue;

// Reply buffer (8 bytes)
byte replyData[8];

// Forward declarations
void twai_listener_task(void *pvParameters);
void process_instruction(CanFrame instruction);
void send_twai_response(const byte response_data[8]);

void setup()
{

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Initializing TapingV2 CAN (TWAI) core...");

  // Attach servos to their respective pins
  feederServo.attach(FEEDER_PIN);
  wrapperServo.attach(WRAPPER_PIN);
  cutterServo.attach(CUTTER_PIN);
  holderServo.attach(HOLDER_PIN);
  gripperServo.attach(GRIPPER_PIN);
  elevatorServo.attach(ELEVATOR_PIN);
  Serial.println("Servos initialized: feeder, wrapper, cutter, holder, gripper, elevator");

  // Move servos to HOME positions
  feederServo.writeMicroseconds(FEEDER_HOME_US);
  wrapperServo.writeMicroseconds(WRAPPER_HOME_US);
  cutterServo.writeMicroseconds(CUTTER_HOME_US);
  holderServo.writeMicroseconds(HOLDER_HOME_US);
  gripperServo.writeMicroseconds(GRIPPER_HOME_US);
  elevatorServo.writeMicroseconds(ELEVATOR_HOME_US);
  // Keep elevator state in sync with commanded HOME
  elevator_current_us = ELEVATOR_HOME_US;
  Serial.println("Servos moved to HOME positions (feeder 1500, wrapper 1500, cutter 500, holder 1500, gripper 2050, elevator 700)");

  // Create the instruction queue
  instruction_queue = xQueueCreate(10, sizeof(CanFrame));
  if (instruction_queue == NULL) {
    Serial.println("Error creating instruction queue");
    while(1) { delay(1000); }
  }

  // Initialize CAN0 (TWAI)
  Serial.println("Initializing CAN0 (TWAI)...");
  ESP32Can.setPins(CAN0_TX, CAN0_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(125)); // 125 kbps
  if (ESP32Can.begin()) {
    Serial.println("CAN0 (TWAI) initialized successfully");
  } else {
    Serial.println("Error initializing CAN0 (TWAI)");
    while (1) { delay(1000); }
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

  Serial.println("TapingV2 CAN ready. Main loop running on core 1.");


  // Send startup message (0x01, status OK = 0x01)
  byte startup_msg[8] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_twai_response(startup_msg);
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
        // Ignored frame: not for this device
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid saturating the CPU
  }
}

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

// Helper: send response via TWAI
void send_twai_response(const byte response_data[8])
{
  CanFrame txFrame;
  txFrame.identifier = RESPONSE_CAN_ID;
  txFrame.extd = 0;
  txFrame.data_length_code = 8;
  for (int i = 0; i < 8; i++) {
    txFrame.data[i] = response_data[i];
  }
  ESP32Can.writeFrame(txFrame);
}

// Process instruction: implement default cases 0x01 and 0x02
void process_instruction(CanFrame instruction)
{
  uint8_t command = instruction.data[0];
  Serial.print("Command: 0x");
  Serial.println(command, HEX);

  switch (command)
  {
    case 0x01: // Reset microcontroller
    {
      byte resp[8] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 OK
      send_twai_response(resp);
      Serial.println("Reset command received. Restarting...");
      delay(50);
      ESP.restart();
      break;
    }

    case 0x02: // Heartbeat
    {
      byte resp[8] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 OK
      send_twai_response(resp);
      Serial.println("Heartbeat processed.");
      break;
    }

    //Step 1
    case 0x03: // Activate feeder (continuous rotation)
    {
      // English: Run feeder at 1390us for 3s, then stop at 1500us and confirm via CAN
      feederServo.writeMicroseconds(1390);
      Serial.println("Feeder activated at 1390us for 3 seconds");
      delay(3000);
      feederServo.writeMicroseconds(1500);
      Serial.println("Feeder stopped at 1500us");
      byte resp[8] = {0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x01 OK
      send_twai_response(resp);
      break;
    }

    //Step 2
    case 0x04: // Wrapper CCW (1200us for 1s, then stop)
    {
      wrapperServo.writeMicroseconds(1200);
      Serial.println("Wrapper activated at 1200us for 1 second");
      delay(1000);
      wrapperServo.writeMicroseconds(1500);
      Serial.println("Wrapper stopped at 1500us");
      byte resp[8] = {0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 3
    case 0x05: // Wrapper CW (1650us for 1s, then stop)
    {
      wrapperServo.writeMicroseconds(1650);
      Serial.println("Wrapper activated at 1650us for 1 second");
      delay(1000);
      wrapperServo.writeMicroseconds(1500);
      Serial.println("Wrapper stopped at 1500us");
      byte resp[8] = {0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 4
    case 0x06: // Cutter sequence: 500us -> wait 300ms -> 1380us -> wait 1s -> 500us
    {
      // English: Move cutter to 500us, wait 300ms, move to 1380us, wait 1s, then back to 500us
      cutterServo.writeMicroseconds(500);
      Serial.println("Cutter moved to 500us");
      delay(300);
      cutterServo.writeMicroseconds(1600);
      Serial.println("Cutter moved to 1380us");
      delay(1000);
      cutterServo.writeMicroseconds(500);
      Serial.println("Cutter returned to 500us");
      byte resp[8] = {0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 5
    case 0x07: // Wrapper CW (1650us for 3s, then stop)
    {
      // English: Move wrapper to 1650us for 3 seconds, then stop at 1500us
      wrapperServo.writeMicroseconds(1650);
      Serial.println("Wrapper activated at 1650us for 3 seconds");
      delay(3000);
      wrapperServo.writeMicroseconds(1500);
      Serial.println("Wrapper stopped at 1500us");
      byte resp[8] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 6
    case 0x08: // Holder to 1850us
    {
      // English: Move holder to 1850us
      holderServo.writeMicroseconds(1850);
      Serial.println("Holder moved to 1850us");
      byte resp[8] = {0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 7
    case 0x09: // Holder to 1650us
    {
      // English: Move holder to 1650us
      holderServo.writeMicroseconds(1650);
      Serial.println("Holder moved to 1650us");
      byte resp[8] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 8
    case 0x0A: // Gripper close to 1850us
    {
      // English: Move gripper to 1850us (close)
      gripperServo.writeMicroseconds(1850);
      Serial.println("Gripper moved to 1850us (close)");
      byte resp[8] = {0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 9
    case 0x0B: // Gripper open to 2000us
    {
      // English: Move gripper to 2000us (open)
      gripperServo.writeMicroseconds(2000);
      Serial.println("Gripper moved to 2000us (open)");
      byte resp[8] = {0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 10
    case 0x0C: // Elevator up to 1550us (ramp gradually)
    {
      // English: Move elevator up to 1550us with gradual ramp
      if (elevator_current_us >= 1550) {
        Serial.println("Elevator already at 1550us; skipping move");
      } else {
        for (int us = elevator_current_us; us <= 1550; us += 10) {
          elevatorServo.writeMicroseconds(us);
          delay(15);
        }
        elevator_current_us = 1550;
      }
      Serial.println("Elevator moved up to 1550us");
      byte resp[8] = {0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step 11
    case 0x0D: // Elevator home to 700us (ramp gradually)
    {
      // English: Move elevator to home 700us with gradual ramp
      if (elevator_current_us <= 700) {
        Serial.println("Elevator already at 700us; skipping move");
      } else {
        for (int us = elevator_current_us; us >= 700; us -= 10) {
          elevatorServo.writeMicroseconds(us);
          delay(15);
        }
        elevator_current_us = 700;
      }
      Serial.println("Elevator moved to home 700us");
      byte resp[8] = {0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

    //Step X
    case 0x0E: // Cutter sequence: 500us -> wait 300ms -> 1380us -> wait 1s -> 500us
    {
      // English: Move cutter to 500us, wait 300ms, move to 1380us, wait 1s, then back to 500us
      cutterServo.writeMicroseconds(1900);
      Serial.println("Cutter moved to 1380us");
      delay(1000);
      byte resp[8] = {0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }

      //Step Y
    case 0x0F: // Cutter sequence: 500us -> wait 300ms -> 1380us -> wait 1s -> 500us
    {
      // English: Move cutter to 500us, wait 300ms, move to 1380us, wait 1s, then back to 500us
      cutterServo.writeMicroseconds(800);
      Serial.println("Cutter moved to 500us");
      delay(300);
      byte resp[8] = {0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(resp);
      break;
    }


    default:
    {
      byte resp[8] = {command, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x02 Unknown/FAIL
      send_twai_response(resp);
      Serial.println("Unknown command.");
      break;
    }
  }
}
