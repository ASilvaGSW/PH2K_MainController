/*
 * Safety System with CAN Bus Integration
 * 
 * CAN Command Reference:
 * 0x01 - Reset Micro (System health check)
 * 0x02 - Heartbeat (Verify system responsiveness)
 * 0x03 - Lock Sensor 1 (Equivalent to ON1 command)
 * 0x04 - Unlock Sensor 1 (Equivalent to OFF1 command)
 * 0x05 - Lock Sensor 2 (Equivalent to ON2 command)
 * 0x06 - Unlock Sensor 2 (Equivalent to OFF2 command)
 * 0x07 - Lock Both Sensors (Equivalent to ON3 command)
 * 0x08 - Unlock Both Sensors (Equivalent to OFF3 command)
 * 0x09 - Get Sensor Status (Returns current sensor states)
 * 0x0A - Get Lock Status (Returns current lock sensor state)
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
#define DEVICE_CAN_ID 0x020  // Different ID from lubrication feeder
#define RESPONSE_CAN_ID 0x420

// Queue to store instructions received from the CAN bus
QueueHandle_t instruction_queue;

// Acknowledgment control
bool enableAckMessages = false;

// Pines de entrada
const int input1Pin = 19;
const int input2Pin = 21;
const int input3Pin = 39;
const int input4Pin = 36;
 
// Pines de sensores candadeados (nuevos inputs con pull-up)
const int lockSensor1Pin = 13;
 
// Pines de LEDs de estado
const int led1Pin = 17;
const int led2Pin = 18;
 
// Pines de salida controlados por Serial
const int output1Pin = 26;
const int output2Pin = 27;
 
// Estados anteriores para detección de flancos
int prevInput1State = LOW;
int prevInput2State = LOW;
int prevLock1State = HIGH;
int prevLock2State = HIGH;
 
// Estados actuales de sensores
bool sensor1Active = false;
bool sensor2Active = false;
bool sensor3Active = false;  
bool sensor4Active = false;  
 
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 100; // 50 ms
int lastStableLock1State = HIGH; // Estado estable anterior
 
void setup()
{
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Initializing Safety System with CAN Bus Integration");
 
  pinMode(input1Pin, INPUT);
  pinMode(input2Pin, INPUT);
  pinMode(input3Pin, INPUT);  
  pinMode(input4Pin, INPUT);  
 
  pinMode(lockSensor1Pin, INPUT_PULLUP);
 
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
 
  pinMode(output1Pin, OUTPUT);
  pinMode(output2Pin, OUTPUT);
 
  digitalWrite(led1Pin, LOW);
  digitalWrite(led2Pin, LOW);
  digitalWrite(output1Pin, LOW);
  digitalWrite(output2Pin, LOW);

  // Create the instruction queue
  instruction_queue = xQueueCreate(10, sizeof(CanFrame));
  if (instruction_queue == NULL) {
    Serial.println("Error creating instruction queue");
    while(1);
  }

  // Initialize the CAN bus (TWAI - General Network)
  Serial.println("Initializing CAN0 (TWAI)...");
  ESP32Can.setPins(CAN0_TX, CAN0_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(125));
  if (ESP32Can.begin()) {
    Serial.println("CAN0 (TWAI) initialized successfully");
  } else {
    Serial.println("Error initializing CAN0 (TWAI)");
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
 
  Serial.println("Sistema iniciado");
  Serial.print("ON1: Lock sensor 1       "); Serial.println("OFF1: Unlock sensor 1");
  Serial.print("ON2: Lock sensor 2       "); Serial.println("OFF2: Unlock sensor 2");
  Serial.print("ON3: Lock sensor 1 y 2   "); Serial.println("OFF3: Unlock sensor 1 y 2");
  Serial.println("CAN Commands: 0x03-0x08 for sensor control, 0x09-0x0A for status");
}
 
void loop()
{
  // Check for CAN instructions
  CanFrame instruction;
  if (xQueueReceive(instruction_queue, &instruction, (TickType_t)0) == pdPASS)
  {
    Serial.println("Processing new CAN instruction from queue...");
    process_can_instruction(instruction);
  }

  int input1State = digitalRead(input1Pin);
  int input2State = digitalRead(input2Pin);
  int input3State = digitalRead(input3Pin);
  int input4State = digitalRead(input4Pin);
  int currentLock1Reading = digitalRead(lockSensor1Pin);
 
  if (currentLock1Reading != prevLock1State) {
    // Hubo un cambio: reiniciar temporizador de debounce
    lastDebounceTime = millis();
  }
 
  // Si ha pasado suficiente tiempo desde el último cambio
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Si el estado ha cambiado respecto al último estado estable
    if (currentLock1Reading != lastStableLock1State) {
      lastStableLock1State = currentLock1Reading;
 
      if (currentLock1Reading == HIGH) {
        Serial.println("Sensors Unlocked");
      } else {
        Serial.println("Sensors Locked");
      }
    }
  }
 
  // Actualizar lectura previa para comparar en el siguiente ciclo
  prevLock1State = currentLock1Reading;
 
  // Actualizar variable sensor1Active y LED1
  if (input1State == LOW)
  {
    if (!sensor1Active)
    {
      Serial.println("Sensor 1 activated");
    }
    sensor1Active = true;
    digitalWrite(led1Pin, HIGH);
  }
  else
  {
    if (sensor1Active)
    {
      Serial.println("Sensor 1 deactivated");
    }
    sensor1Active = false;
    digitalWrite(led1Pin, LOW);
    digitalWrite(output1Pin, LOW);  // apagar salida si se desactiva
  }
 
  // Actualizar variable sensor2Active y LED2
  if (input2State == LOW)
  {
    if (!sensor2Active)
    {
      Serial.println("Sensor 2 activated");
    }
    sensor2Active = true;
    digitalWrite(led2Pin, HIGH);
  }
  else
  {
    if (sensor2Active)
    {
      Serial.println("Sensor 2 deactivated");
    }
    sensor2Active = false;
    digitalWrite(led2Pin, LOW);
    digitalWrite(output2Pin, LOW);  // apagar salida si se desactiva
  }
// Sensor 3 (solo impresión por ahora)
  if (input3State == LOW)
  {
    if (!sensor3Active) Serial.println("NON CONTACT Sensor 1 activated");
    sensor3Active = true;
  }
  else
  {
    if (sensor3Active) Serial.println("NON CONTACT Sensor 1 deactivated");
    sensor3Active = false;
  }
 
  // Sensor 4 (solo impresión por ahora)
  if (input4State == LOW)
  {
    if (!sensor4Active) Serial.println("NON CONTACT Sensor 2 activated");
    sensor4Active = true;
  }
  else
  {
    if (sensor4Active) Serial.println("NON CONTACT Sensor 2 deactivated");
    sensor4Active = false;
  }
 
 
  // Leer comando serial
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Elimina espacios o saltos de línea
 
    if (cmd == "ON1")
    {
      if (sensor1Active)
      {
        digitalWrite(output1Pin, HIGH);
        Serial.println("Salida 1 ACTIVADA");
      }
      else
      {
        Serial.println("Sensor 1 no está activo. No se puede activar salida 1.");
      }
    }
 
    else if (cmd == "OFF1")
    {
      digitalWrite(output1Pin, LOW);
      Serial.println("Salida 1 DESACTIVADA");
    }
 
    else if (cmd == "ON2")
    {
      if (sensor2Active)
      {
        digitalWrite(output2Pin, HIGH);
        Serial.println("Salida 2 ACTIVADA");
      }
      else
      {
        Serial.println("Sensor 2 no está activo. No se puede activar salida 2.");
      }
    }
 
    else if (cmd == "OFF2")
    {
      digitalWrite(output2Pin, LOW);
      Serial.println("Salida 2 DESACTIVADA");
    }
 
    else if (cmd.startsWith("ON3"))
    {
      if (sensor1Active && sensor2Active)
      {
        digitalWrite(output1Pin, HIGH);
        Serial.println("Salida 1 ACTIVADA");
        digitalWrite(output2Pin, HIGH);
        Serial.println("Salida 2 ACTIVADA");
      }
      else
      {
        Serial.println("Sensor 1 o 2 no está activado. No se puede activar salida 1 y 2.");
      }
    }
 
    else if (cmd.startsWith("OFF3"))
    {
      digitalWrite(output1Pin, LOW);
      Serial.println("Salida 1 DESACTIVADA");
      digitalWrite(output2Pin, LOW);
      Serial.println("Salida 2 DESACTIVADA");
    }
      else
    {
      Serial.println("Comando no reconocido.");
    }
 
  }
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
    // Lock Sensor 1 (ON1)
    case 0x03:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x03: Lock Sensor 1 (CAN ON1)");
      
      if (sensor1Active)
      {
        digitalWrite(output1Pin, HIGH);
        Serial.println("Salida 1 ACTIVADA via CAN");
        byte response[] = {0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(response);
      }
      else
      {
        Serial.println("Sensor 1 no está activo. No se puede activar salida 1 via CAN.");
        byte response[] = {0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Unlock Sensor 1 (OFF1)
    case 0x04:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x04: Unlock Sensor 1 (CAN OFF1)");
      digitalWrite(output1Pin, LOW);
      Serial.println("Salida 1 DESACTIVADA via CAN");
      byte response[] = {0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x05 ***************************** //
    // Lock Sensor 2 (ON2)
    case 0x05:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x05: Lock Sensor 2 (CAN ON2)");
      
      if (sensor2Active)
      {
        digitalWrite(output2Pin, HIGH);
        Serial.println("Salida 2 ACTIVADA via CAN");
        byte response[] = {0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(response);
      }
      else
      {
        Serial.println("Sensor 2 no está activo. No se puede activar salida 2 via CAN.");
        byte response[] = {0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x06 ***************************** //
    // Unlock Sensor 2 (OFF2)
    case 0x06:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x06: Unlock Sensor 2 (CAN OFF2)");
      digitalWrite(output2Pin, LOW);
      Serial.println("Salida 2 DESACTIVADA via CAN");
      byte response[] = {0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x07 ***************************** //
    // Lock Both Sensors (ON3)
    case 0x07:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x07: Lock Both Sensors (CAN ON3)");
      
      if (sensor1Active && sensor2Active)
      {
        digitalWrite(output1Pin, HIGH);
        Serial.println("Salida 1 ACTIVADA via CAN");
        digitalWrite(output2Pin, HIGH);
        Serial.println("Salida 2 ACTIVADA via CAN");
        byte response[] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(response);
      }
      else
      {
        Serial.println("Sensor 1 o 2 no está activado. No se puede activar salida 1 y 2 via CAN.");
        byte response[] = {0x07, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(response);
      }
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Unlock Both Sensors (OFF3)
    case 0x08:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x08: Unlock Both Sensors (CAN OFF3)");
      digitalWrite(output1Pin, LOW);
      Serial.println("Salida 1 DESACTIVADA via CAN");
      digitalWrite(output2Pin, LOW);
      Serial.println("Salida 2 DESACTIVADA via CAN");
      byte response[] = {0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x09 ***************************** //
    // Get Sensor Status
    case 0x09:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x09: Get Sensor Status");
      byte sensor_states = 0;
      if (sensor1Active) sensor_states |= 0x01;
      if (sensor2Active) sensor_states |= 0x02;
      if (sensor3Active) sensor_states |= 0x04;
      if (sensor4Active) sensor_states |= 0x08;
      
      byte response[] = {0x09, 0x01, sensor_states, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0A ***************************** //
    // Get Lock Status
    case 0x0A:
    {
      // Send acknowledgment message
      if (enableAckMessages) {
        byte ack[] = {0xAA, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_twai_response(ack);
      }
      
      Serial.println("Case 0x0A: Get Lock Status");
      byte lock_status = (lastStableLock1State == LOW) ? 0x01 : 0x00; // 0x01 = locked, 0x00 = unlocked
      byte response[] = {0x0A, 0x01, lock_status, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0xFF ***************************** //
    // Emergency Stop
    case 0xFF:
    {
      Serial.println("Case 0xFF: Emergency Stop via CAN");
      digitalWrite(output1Pin, LOW);
      digitalWrite(output2Pin, LOW);
      Serial.println("Emergency Stop: All outputs deactivated");
      byte response[] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE DEFAULT ***************************** //
    default:
      Serial.println("Unknown CAN command.");
      byte errorResponse[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      send_twai_response(errorResponse);
      break;
  }
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
 