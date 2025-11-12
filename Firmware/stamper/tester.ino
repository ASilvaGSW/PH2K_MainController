///---Main libraries---///PROGRAMA DE ESTAMPADO FINAL CON BOTONERA
#include <ESP32-TWAI-CAN.hpp>
#include <EEPROM.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <MKS57_CAN.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include "AccelStepper.h"
 
 
///---Initial definitions---///
#define SERIAL_BAUDRATE 115200
#define CPU_FREQUENCY 80
 
//////------ID of Linear actuators-------//////
#define ID_AXIS_X 0x01     //
#define ID_AXIS_Z 0x02     //
#define ID_BOMBA 0x03      //BOMBA
 
// Array con los IDs
uint8_t axisIDs[] = {
    ID_AXIS_X,
    ID_AXIS_Z,
    ID_BOMBA
 
};
 
 
// Define stepper motor connections and motor interface type.
// Motor interface type must be set to 1 when using a driver:
#define dirPin1 18//Mini linear actuator axis Z
#define stepPin1 17
#define enable1 16
AccelStepper miniStepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
 
#define dirPin2 32  //Mini linear actuator axis X
#define stepPin2 33
#define enable2 25
AccelStepper miniStepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
 
 
 
#define CAN_TX 4 //Serial pin for ESP32 TX CAN TRANSCEIVER
#define CAN_RX 5 //Serial pin for ESP32 TX CAN TRANSCEIVER
#define TX_QUEUE_SIZE 10 //Set buffer lenght  for TX.
#define RX_QUEUE_SIZE 10 //Set buffer length for RX.
#define CAN_SPEED 1000 //Set CAN bus speed scaler for tx/rx data.
#define CAN_TIMEOUT 100 //Set timeout for CAN bus transmission/reception
#define WRITE_CAN_ID 0x402
#define MAX_FRAMES 100
#define DATA_SIZE 8
 
//Servo setup//
#define min_pwm 500
#define max_pwm 2500
#define min_angle 0
#define max_angle 270
Servo stamping;
 
#define min_pwm1 500
#define max_pwm1 2500
#define min_angle1 0
#define max_angle1 270
Servo cover;
 
//Servomotor stamp positions
#define servoStamp_home 150
#define servoStamp_takePaint 128
#define servoStamp_stamping 80
 
//Servo cover positions
#define servoCover_home 0
#define servoCover_final 95
 
//Fuyu Z
#define axisZ_highToLubricate 8400
#define axisZ_moving 1000
#define axisZ_home 0
 
//Fuyu X
#define axisX_takePaint 16777215-92550
#define axisX_stampingArea 16777215-59500
#define axisX_home 0
 
//Mini Z
#define miniZ_highPosition 4400
#define miniZ_LubricatePosition 3600
#define miniZ_home 0
 
//Mini X
#define miniX_takePaint 17000
#define miniX_home 0
 
#define lubricateLong_1 100000
#define lubricateLong_2 80000
#define lubricateLong_3 60000
#define lubricateLong_4 40000
#define lubricateLong_5 20000
 
#define lubricateShort_1 10000
#define lubricateShort_2 8000
#define lubricateShort_3 6000
#define lubricateShort_4 4000
#define lubricateShort_5 2000
 
// Variable global para controlar la ejecución del ciclo de "STAMP"
bool stampRunning = false;
 
//Sensors
#define VALVULA1_PIN 2  // Pin de salida para activar el relé/LED
 
 
// Variables
bool CAN_STATUS = false;
int instructions = 0;
unsigned long previousMillis = 0;
unsigned long lastMoveTimeClose = 0;  
unsigned long lastMoveTimeOpen = 0;  
String prev_status = "ok";
String cur_status = "ok";
static uint32_t lastStamp = 0;
uint8_t canInstructions[MAX_FRAMES][DATA_SIZE];
uint8_t currentInstruction[8];
uint8_t emptyArray[8] = {0,0,0,0,0,0,0,0};
 
// Instances
CanFrame rxFrame;
 
//---Declare object name of MKS Driver, set tx/rx pins, can speed, tx/rx buffer size---//
MKS57_CAN MKS_SERVO_57D(CAN_TX, CAN_RX,  CAN_SPEED,  TX_QUEUE_SIZE, RX_QUEUE_SIZE);
void sendCanbus(uint8_t b0,uint8_t b1,uint8_t b2,uint8_t b3,uint8_t b4,uint8_t b5,uint8_t b6,uint8_t b7);
void hearthbit();
void updateStatus(String new_status);
void clearInstruction();
void readCanbus();
void movestamping(int angle);
void moveCover(int angle);
void setMiniStepper();
void calculateFlowTask(void *parameter);
void StampLoop();
void LubricateLongLoop();
void HomeLoop();
void LubricateShortLoop();
 
void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
 
 
    //valvulas
    pinMode(VALVULA1_PIN, OUTPUT);
    digitalWrite(VALVULA1_PIN, LOW);  // Iniciar apagado
 
    // Set enable pins as outputs:
    pinMode(enable1, OUTPUT);
    pinMode(enable2, OUTPUT);
 
    // Enable the motors by setting enable pins LOW (or HIGH depending on the motor driver):
    digitalWrite(enable1, LOW);  // Enable motor 1
    digitalWrite(enable2, LOW);  // Enable motor 2
 
    setMiniStepper();
 
    stamping.attach(19);
    movestamping(servoStamp_home);
    cover.attach(21);
    moveCover(servoCover_home);
    MKS_SERVO_57D.begin();  //Initialize driver interface
   
    if(ESP32Can.begin())
    {
      CAN_STATUS = true;
    }
  else
    {
      CAN_STATUS = false;
    }
 
    updateStatus("Initializing");
    updateStatus("Ready");
    sendCanbus(0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA);
}
 
void loop()
{
 
 
    //****************************************************************** Reading and Running Commands ****************************************************
readCanbus();
//Serial.println("Start");
delay(1000);
 
 
if (Serial.available() > 0)
{
    String command = Serial.readStringUntil('\n');  // Lee la palabra o comando
    command.trim();  // Elimina cualquier espacio extra antes o después del comando
 
    Serial.print("Comando recibido: ");
 
    // Usamos if-else en lougar de switch para comparar las cadenas
    if (command == "HOME") {  HomeLoop(); Serial.println("All actuators to home"); }
 
    else if (command == "SHORT") {  LubricateShortLoop();  Serial.println("Lubricate Short starting"); }
 
    else if (command == "LONG") {  LubricateLongLoop();  Serial.println("Lubricate Long starting"); }
 
    else if (command == "STAMP") {  StampLoop();  Serial.println("Start stamp loop"); }
 
    else if (command == "V1ON") {  digitalWrite(VALVULA1_PIN, HIGH);  Serial.println("Valvula1 ACTIVADA"); }
 
    else if (command == "V1OFF") {  digitalWrite(VALVULA1_PIN, LOW);  Serial.println("Valvula1 DESACTIVADA"); }
 
    else if (command.startsWith("SERVO_STAMP"))
    {
      int position;
      int numParams = sscanf(command.c_str(), "STAMP %d", &position);
      if (numParams==1)
      {
        Serial.print("Moviendo stamp:"); Serial.println(position);
        movestamping(position);
        delay(300);
      }
    }
 
    else if (command.startsWith("SERVO_COVER"))
    {
      int position;
      int numParams = sscanf(command.c_str(), "COVER %d", &position);
      if (numParams==1)
      {
        Serial.print("Moviendo cover:"); Serial.println(position);
        moveCover(position);
        delay(200);
      }
    }
 
    else if (command.startsWith("MZ"))      //MZ 4500 PARA ESTAR ARRIBA DE COVER
    {
      int position;
      int numParams = sscanf(command.c_str(), "MZ %d", &position);
      if (numParams==1)
      {
        Serial.print("Moviendo miniz:"); Serial.println(position);
        miniStepper1.moveTo(position);//400PULSE 1 REV
        miniStepper1.runToPosition();
        delay(200);
      }
    }
 
    else if (command.startsWith("MX"))      //MZ 4500 PARA ESTAR ARRIBA DE COVER
    {
      int position;
      int numParams = sscanf(command.c_str(), "MX %d", &position);
      if (numParams==1)
      {
        Serial.print("Moviendo minix:"); Serial.println(position);
        miniStepper2.moveTo(position);//400PULSE 1 REV
        miniStepper2.runToPosition();
        delay(200);
      }
    }
 
    else if (command.startsWith("FUYUZ"))
    {
      // Comando esperado: BOMBA velocidad aceleracion pulsos
      int speed, acceleration, pulses;
      int numParams = sscanf(command.c_str(), "FUYUZ %d %d %d", &speed, &acceleration, &pulses);
       
      if (numParams == 3)
      {
        // Si los tres parámetros fueron leídos correctamente
        Serial.print("Controlando FUYUZ con: ");
        Serial.print("Velocidad: "); Serial.print(speed);
        Serial.print(", Aceleración: "); Serial.print(acceleration);
        Serial.print(", Pulsos: "); Serial.println(pulses);
 
        // Ejecutar movimiento con los pulsos especificados
 
        MKS_SERVO_57D.RunAbsoluteMotionByPulses(2, speed, acceleration, pulses);
        delay(4000);
        Serial.println("FUYU Z MOVIMIENTO TERMINADO");
      }
      }
 
      else if (command.startsWith("FUYUX"))
    {
      // Comando esperado: BOMBA velocidad aceleracion pulsos
      int speed, acceleration, pulses;
      int numParams = sscanf(command.c_str(), "FUYUX %d %d %d", &speed, &acceleration, &pulses);
       
      if (numParams == 3)
      {
        // Si los tres parámetros fueron leídos correctamente
        Serial.print("Controlando FUYUX con: ");
        Serial.print("Velocidad: "); Serial.print(speed);
        Serial.print(", Aceleración: "); Serial.print(acceleration);
        Serial.print(", Pulsos: "); Serial.println(pulses);
        pulses= 16777215-pulses;
        // Ejecutar movimiento con los pulsos especificados
        MKS_SERVO_57D.RunAbsoluteMotionByPulses(1, speed, acceleration, pulses);
        delay(4000);
        Serial.println("FUYU X MOVIMIENTO TERMINADO");
      }
      }
 
      else if (command.startsWith("B"))
      {
        // Comando esperado: BOMBA velocidad aceleracion pulsos
        int speed, acceleration, pulses;
        int numParams = sscanf(command.c_str(), "B %d %d %d", &speed, &acceleration, &pulses);
         
        if (numParams == 3)
        {
          // Si los tres parámetros fueron leídos correctamente
          Serial.print("Controlando bomba con: ");
          Serial.print("Velocidad: "); Serial.print(speed);
          Serial.print(", Aceleración: "); Serial.print(acceleration);
          Serial.print(", Pulsos: "); Serial.println(pulses);
   
          // Ejecutar movimiento con los pulsos especificados
          MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, speed, acceleration, pulses);
          delay(4000);
          MKS_SERVO_57D.SetZero(3);
          Serial.println("Movimiento de la bomba completado");
        }
        }
 
    else
    {
      Serial.println("No reconocido.");
    }
 
}
}
 
 
void readCanbus()
{
    if(ESP32Can.readFrame(rxFrame, CAN_TIMEOUT))
    {
        // Verificar si el identificador está en el array de IDs
        for (int i = 0; i < sizeof(axisIDs) / sizeof(axisIDs[0]); i++)
        {
            if (rxFrame.identifier == axisIDs[i])
            {  
                // Buffer Control Purpose. Si hay más de 100 instrucciones
                instructions++;
 
                if(instructions >= 100) // Reemplaza el último comando con el más nuevo.
                {
                    instructions = 99;
                }          
 
                memcpy(canInstructions[instructions-1], rxFrame.data, DATA_SIZE);
                break; // Salir del bucle si se encuentra el ID
            }
        }
    }
}
 
void sendCanbus(uint8_t b0,uint8_t b1,uint8_t b2,uint8_t b3,uint8_t b4,uint8_t b5,uint8_t b6,uint8_t b7)
  {
    CanFrame obdFrame = { 0 };
    obdFrame.identifier = WRITE_CAN_ID; // Default OBD2 address;
    obdFrame.extd = 0;
    obdFrame.data_length_code = 8;
    obdFrame.data[0] = b0;
    obdFrame.data[1] = b1;
    obdFrame.data[2] = b2;
    obdFrame.data[3] = b3;    // Best to use 0xAA (0b10101010) instead of 0
    obdFrame.data[4] = b4;    // CAN works better this way as it needs
    obdFrame.data[5] = b5;    // to avoid bit-stuffing
    obdFrame.data[6] = b6;
    obdFrame.data[7] = b7;
      // Accepts both pointers and references
 
    ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
  }
 
void hearthbit()
  {
    sendCanbus(0xEE,0xEE,0xEE,0xEE,0xEE,0xEE,0xEE,0xEE);
  }
 
void updateStatus(String new_status)
  {
    prev_status = cur_status;
    cur_status = new_status;
  }
 
  void clearInstruction()
  {
    memcpy(canInstructions[0], emptyArray, DATA_SIZE);
    for (int i = 0; i <= instructions; i++)
    {
      memcpy(canInstructions[i], canInstructions[i+1], DATA_SIZE);
    }
    instructions--;
  }
 
 
void movestamping(int angle)
{
  int target_angle = map(angle, min_angle, max_angle, min_pwm, max_pwm);
  stamping.writeMicroseconds(target_angle);
  //Serial.print("Angulo: ");
  //Serial.println(angle);
}
 
 void moveCover(int angle)
 {
   int target_angle = map(angle, min_angle1, max_angle1, min_pwm1, max_pwm1);
   cover.writeMicroseconds(target_angle);
   //Serial.print("Angulo: ");
   //Serial.println(angle);
 }
 
 
void setMiniStepper()
{
  miniStepper1.setMaxSpeed(1500000);
  miniStepper1.setAcceleration(1500000);
 
  miniStepper2.setMaxSpeed(1500000);
  miniStepper2.setAcceleration(1500000);
}
 
void StampLoop()
{
  // Almacenar el tiempo de inicio del ciclo
  unsigned long cycleStartTime = millis();
 
  Serial.println("StampLoop starting");
 
  digitalWrite(VALVULA1_PIN, LOW);  // Apagar válvula 1
  MKS_SERVO_57D.RunAbsoluteMotionByPulses(2,500,200,axisZ_highToLubricate);  //   ID_AXIS_Z buscar altura de fuyuz altura a lubricar
  movestamping(servoStamp_home);
  moveCover(servoCover_home);
  miniStepper1.moveTo(miniZ_highPosition);     miniStepper1.runToPosition();
  delay(2000);
 
  miniStepper2.moveTo(miniX_takePaint);    miniStepper2.runToPosition();
  delay(10);
 
  MKS_SERVO_57D.RunAbsoluteMotionByPulses(1,500,200,axisX_takePaint);  //   ID_AXIS_X
  miniStepper1.moveTo(miniZ_LubricatePosition);     miniStepper1.runToPosition();
  delay(4000);
 
  miniStepper1.moveTo(miniZ_highPosition);     miniStepper1.runToPosition();
  delay(10);
 
  miniStepper2.moveTo(miniX_home);        miniStepper2.runToPosition();
  delay(300);
 
 
  movestamping(servoStamp_takePaint);
  moveCover(servoCover_final);
  delay(1000);
 
  movestamping(servoStamp_home);
  moveCover(servoCover_home);
  delay(500);
 
  MKS_SERVO_57D.RunAbsoluteMotionByPulses(1,500,200,axisX_stampingArea);  //   ID_AXIS_X
  miniStepper2.moveTo(miniX_takePaint);     miniStepper2.runToPosition();
  delay(3000);
  MKS_SERVO_57D.RunAbsoluteMotionByPulses(2,500,200,axisZ_home);  //   ID_AXIS_Z
  miniStepper1.moveTo(miniZ_LubricatePosition);      miniStepper1.runToPosition();
  delay(2000);
 
  movestamping(servoStamp_stamping);
  delay(1000);
 
  MKS_SERVO_57D.RunAbsoluteMotionByPulses(2,500,200,axisZ_highToLubricate);  //   ID_AXIS_Z
  movestamping(servoStamp_home);
  delay(1000);
 
 // Calcular el tiempo del ciclo
  unsigned long cycleEndTime = millis();
  unsigned long cycleDuration = cycleEndTime - cycleStartTime;
 
  // Mostrar el tiempo de ciclo en el Serial Monitor
  Serial.print("Tiempo de ciclo StampLoop: ");
  Serial.print(cycleDuration);
  Serial.println(" ms");
}
void HomeLoop()
{
  Serial.println("HomeLoop starting");
 
 
  MKS_SERVO_57D.RunAbsoluteMotionByPulses(2,1000,240,axisZ_highToLubricate);  //   ID_AXIS_Z
  movestamping(servoStamp_home);
  moveCover(servoCover_home);
  miniStepper1.moveTo(miniZ_highPosition);     miniStepper1.runToPosition();
  delay(500);
 
  MKS_SERVO_57D.RunAbsoluteMotionByPulses(1,1000,240,axisX_takePaint);  //   ID_AXIS_X
  miniStepper2.moveTo(miniX_home);    miniStepper2.runToPosition();
  delay(3000);
 
  movestamping(servoStamp_takePaint);
  moveCover(servoCover_final);
}
 
void LubricateLongLoop()
{
  Serial.println("Lubricate Long starting");
 
     
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(2,800,240,axisZ_highToLubricate);  //   ID_AXIS_Z
     movestamping(servoStamp_home);
     moveCover(servoCover_home);
     miniStepper1.moveTo(miniZ_highPosition);     miniStepper1.runToPosition();
     delay(500);
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(1,800,240,axisX_takePaint);  //   ID_AXIS_X
     miniStepper2.moveTo(miniX_takePaint);    miniStepper2.runToPosition();
     delay(650);
 
     digitalWrite(VALVULA1_PIN, HIGH);  // Encender válvula 1
     Serial.println("Sensor ACTIVADO, SI PINTURA");
 
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 800, 240, lubricateLong_1);
     delay(4000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 800, 240, lubricateLong_2);
     delay(4000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 800, 240, lubricateLong_3);
     delay(4000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 800, 240, lubricateLong_4);
     delay(4000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 800, 240, lubricateLong_5);
     miniStepper1.moveTo(miniZ_LubricatePosition);      miniStepper1.runToPosition();
     delay(4000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
     digitalWrite(VALVULA1_PIN, LOW);  // Encender válvula 1
     miniStepper1.moveTo(miniZ_highPosition);     miniStepper1.runToPosition();
 
 
    delay(100); // Pequeña pausa para evitar rebotes
}
 
 
void LubricateShortLoop()
{
  Serial.println("Lubricate Short starting");
     
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(2,1000,240,axisZ_highToLubricate);  //   ID_AXIS_Z
     movestamping(servoStamp_home);
     moveCover(servoCover_home);
     miniStepper1.moveTo(miniZ_highPosition);     miniStepper1.runToPosition();
     delay(500);
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(1,1000,240,axisX_takePaint);  //   ID_AXIS_X
     miniStepper2.moveTo(miniX_takePaint);    miniStepper2.runToPosition();
     delay(650);
 
     digitalWrite(VALVULA1_PIN, HIGH);  // Encender válvula 1
     Serial.println("Sensor ACTIVADO, SI PINTURA");
 
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 200, 200, lubricateShort_1);//1000
     delay(4000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 200, 200, lubricateShort_2);//800
     delay(3000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
 
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 200, 200, lubricateShort_3);//600
     delay(2000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 200, 200, lubricateShort_4);//200
     delay(1000);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
     MKS_SERVO_57D.RunAbsoluteMotionByPulses(3, 200, 200, lubricateShort_5);//500
     miniStepper1.moveTo(miniZ_LubricatePosition);      miniStepper1.runToPosition();
     delay(500);
     MKS_SERVO_57D.SetZero(3);
     delay(400);
 
   
    digitalWrite(VALVULA1_PIN, LOW);  // Encender válvula 1
    miniStepper1.moveTo(miniZ_highPosition);     miniStepper1.runToPosition();
   
 
    delay(100); // Pequeña pausa para evitar rebotes
}