/*
 * TapingV2 - CAN (TWAI) minimal integration
 * English: Initializes ESP32 TWAI CAN, listens for instructions, and responds. Implements default cases 0x01 (reset) and 0x02 (heartbeat).
 * Español: Inicializa CAN TWAI en ESP32, escucha instrucciones y responde. Implementa los casos por defecto 0x01 (reset) y 0x02 (heartbeat).
 * 日本語: ESP32 の TWAI CAN を初期化し、指示を受信・返信します。デフォルトの 0x01（リセット）と 0x02（ハートビート）を実装します。
 */

#include <ESP32-TWAI-CAN.hpp>

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

    default:
    {
      byte resp[8] = {command, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x02 Unknown/FAIL
      send_twai_response(resp);
      Serial.println("Unknown command.");
      break;
    }
  }
}