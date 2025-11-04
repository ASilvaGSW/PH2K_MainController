/////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                             //
//                FW version 9                                                                 //
//                                                                                             //
/////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Change of approach on the sync movement of the step12. Now the Servo1 moves by degrees

  by Gonzalo Martinez
*/

/////////////
//Libraries//
/////////////
#include <ESP32Servo.h>                                                                                       // Library to control the servos
#include <Wire.h>                                                                                             // Library to control I2C
#include <AS5600.h>                                                                                           // Library for Encoders
#include <Adafruit_TCS34725.h>                                                                                // Library for RGB sensors
#include <ESP32-TWAI-CAN.hpp>                                                                                 // Library for CAN bus TWAI communication

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/////////////////////
//Servos Parameters//
/////////////////////
#define angleMin 0                                                                                            // Set the minimum angle to control
#define angleMax 180                                                                                          // Set the maximum angle to control
#define minTravelAngle 0                                                                                      // This is for servos KST
#define MaxTravelAngle 120                                                                                    // This is for servos KST
#define microsecondsMin 500                                                                                   // Set the minimum usec, consult the datasheet
#define microsecondsMax 2500                                                                                  // Set the maximum usec, consult the datasheet
#define pulseLenghtMin 900                                                                                    // This is for servos KST
#define pulseLenghtMax 2100                                                                                   // This is for servos KST

////////////////////////////////
//Objects to handle the Servos//
////////////////////////////////
Servo servo1;                                                                                                 // Feeder
Servo servo2;                                                                                                 // Wrapper
Servo servo3;                                                                                                 // Cutter
Servo servo4;                                                                                                 // Holder
Servo servo5;                                                                                                 // Gripper
Servo servo6;                                                                                                 // Elevator

int servoPin1 = 2;                                                                                            // Servo 1 pin connection
int servoPin2 = 25;                                                                                           // Servo 2 pin connection
int servoPin3 = 15;                                                                                           // Servo 3 pin connection
int servoPin4 = 14;                                                                                           // Servo 4 pin connection
int servoPin5 = 19;                                                                                           // Servo 5 pin connection
int servoPin6 = 17;                                                                                           // Servo 6 pin connection

/////////////////////
//CAN Bus TWAI Pins//
/////////////////////
#define CAN0_TX 4                                                                                             // GPIO4 - TX for CAN0 (General Network)
#define CAN0_RX 5                                                                                             // GPIO5 - RX for CAN0 (General Network)

// Device CAN ID (only process messages with this ID)
#define DEVICE_CAN_ID 0x00A                                                                                   // Taping device CAN ID
#define RESPONSE_CAN_ID 0x40A                                                                                 // Response CAN ID

// Queue to store instructions received from the CAN bus (TWAI)
QueueHandle_t instruction_queue;

///////////////////
//SERVO1 [Feeder]//
///////////////////
unsigned long startTime1;                                                                                     // Variable to store start time of motion in Servo1
//int currentState1 = 0;                                                                                      // Servo state: 0 -> first direction, 1 -> second direction
const float GapGainFeeder = 6.486021505;                                                                      // Value to reduce the error between 'targetPosition - MovedAngle'
const unsigned long timeout = 5000;                                                                           // Value to set the maximum working time of the servo1 in a row. 5000 = 5 second

////////////////////
//SERVO2 [Wrapper]//
////////////////////
#define speedServo2_CCW  1200                                                                                 // Range from 500 to 1500   (the closer to 1500 the slower)
#define speedServo2_CW   1650                                                                                 // Range from 1500 to 2500  (the closer to 1500 the slower)
#define HALL_SENSOR_PIN 26                                                                                    // Pin to receive signal from Hall Effect Sensor located in the Wrapper
unsigned long startTime2;                                                                                     // Variable to store start time of motion
unsigned long startTime3;                                                                                     // Variable to store start time of motion
unsigned long startTime4;                                                                                     // Variable to store start time of motion
const unsigned long timeDirection1 = 500;                                                                     // 1st movement duration (500 miliseconds)
const unsigned long timeDirection2 = 1000;                                                                    // 2nd movement duration (1 second)
const unsigned long timeDirection3 = 2000;                                                                    // Last movement duration (2 seconds)
const unsigned long timeout2 = 5000;                                                                          // Value to set the maximum working time of the servo2 in a row. 5000 = 5 second
const float GapGainWrapper1 = 29.52992;                                                                       // Value to reduce the error between 'targetPosition - MovedAngle'
const float GEAR_RATIO = 2.333;                                                                               // Encoder gear spins 2.333 times per 1 wrapper turn
volatile bool hallDetected = false;                                                                           // Boolean variable that activates when the Hall Effect Sensor detects something
volatile unsigned long lastHallTime = 0;                                                                      // Hall Effect Sensor working time
volatile int hallCounter = 0;                                                                                 // Hall Effect Sensor event counter
const int TARGET_ROTATIONS = 4;                                                                               // Target to achieve to the hallCounter, this represents 3 spins of the Wrapper
void IRAM_ATTR hallSensorISR();                                                                               // Name of the function for the Hall Effect Sensor (Interrupt Service Routine for Hall Effect Sensor)
//int currentState2 = 0;                                                                                      // Servo state: 0 -> first direction, 1 -> second direction

///////////////////
//SERVO3 [Cutter]//
///////////////////
#define stopServo     1500                                                                                    // Do not modify, this is to stop the servo
#define zeroPosition  500                                                                                     // Do not modify, this is to place the Cutter servo in Cut position
int angle_cut = 80;                                                                                           // Angle of the servo to reach the Cut position
int angle_home = 0;                                                                                           // Angle of Home position

/////////////////////////////////
//SERVO4 [Holder] & 5 [Gripper]//
/////////////////////////////////
#define FinalPosition 2000                                                                                    // Final Position of Servo
#define HomePosition 1500                                                                                     // Home Position Servo   
int angle_hold4 = 95;                                                                                         // Angle of servo4 to reach and hold the tape
int angle_home4 = 70;                                                                                         // Angle of servo4 to reach the home position
int angle_hold5 = 95;                                                                                         // Angle of servo5 to close the gripper
int angle_home5 = 115;                                                                                        // Angle of servo5 to open the gripper

/////////////////////
//SERVO6 [Elevator]//
/////////////////////
#define S6FinalPos 1550                                                                                       // Up
#define S6HomePos 700                                                                                         // Down   
const float servo6StepDelay = 1;                                                                              // Value of delay used to control speed
                                                                                                            
////////////
//Encoders//
////////////
AS5600 encoder1;                                                                                              // Create Encoder object for Feeder, pins 21, 22
AS5600 encoder2(&Wire1);                                                                                      // Create Encoder object for Wrapper, pins 13, 23
const float encoderRes = 0.08789;                                                                             // Encoder resolution obtained from 360/4095. 360 is the degrees of full spin, 4095 is the maximum value of 12 bits
float initialAngle = 0;                                                                                       // To ensure that the Encoder will start with a value of 0
float targetAngle = 0;                                                                                        // "" it will change later on the code
const float degreesPerFeed = 158.0;                                                                           // Adjust the angle based on the lenght of tape to feed: 60 mm = 152.2 degrees (158 value obtained during tests)
const float degreesPerFeedSync = 177.66;                                                                      // Lenght of the tape that will be wrapped around the hose
float initialAngle2 =0;                                                                                       // To ensure that the Encoder will start with a value of 0
float targetAngle2 = 0;                                                                                       // "" it will change later on the code
const float degreesPerWrap = 120.0 * GEAR_RATIO;                                                              // 1st movement of the Wrapper. This represents 1/4 of turn CCW direction
const float degreesPerWrap2 = 90.0 * GEAR_RATIO;                                                              // 2nd movement of the Wrapper. This represents 1/4 of turn CW direction  
const float degreesPerWrap3 = 1800.0 * GEAR_RATIO;                                                            // Final movement of the Wrapper. This represents 5 turns CW direction

////////////////////
//Position Sensors//
////////////////////
const int Cutter = 18;                                                                                        // Pin of position sensor 'Cut', used to detect Cutter movement
const int ElevatorDown = 16;                                                                                  // Pin of position sensor 'Down', used to detect Elevator
  
///////////////  
//RGB Sensors//
///////////////
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);                  // Instance to set and control RGB Feeder Sensor
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);                  // Instance to set and control RGB Hose Sensor

///////////////////
//State variables//
///////////////////
bool gripperClosed = false;                                                                                   // Flag to indicate gripper is closed
bool elevatorIsDown = false;                                                                                  // Flag to indicate elevator is down
bool gripperReadyForHose = false;                                                                             // Flag to indicate the gripper is waiting for a trigger to open

////////////////
//Push Buttons//
////////////////
#define BUTTON_FULLCYCLE 27                                                                               // Push button for Full Cycle / Cut
#define BUTTON_FORWARD   32                                                                                   // Push button for Feeder Forward Movement
#define BUTTON_BACKWARD  33                                                                                   // Push button for Feeder Backwar Movement

// Debounce variables for Push Buttons //
int lastStableStateFullCycle = HIGH;                                                                          // Last confirmed stable state for FullCycle button
int lastRawStateFullCycle = HIGH;                                                                             // Last raw reading from FullCycle button pin
unsigned long lastDebounceTimeFullCycle = 0;                                                                  // Last debounce timer for FullCycle button

int lastStableStateForward = HIGH;                                                                            // Last confirmed stable state for Forward button
int lastRawStateForward = HIGH;                                                                               // Last raw reading from Forward button pin
unsigned long lastDebounceTimeForward = 0;                                                                    // Last debounce timer for Forward button

int lastStableStateBackward = HIGH;                                                                           // Last confirmed stable state for Backward button
int lastRawStateBackward = HIGH;                                                                              // Last raw reading from Backward button pin
unsigned long lastDebounceTimeBackward = 0;                                                                   // Last debounce timer for Backward button

const unsigned long debounceDelayTest = 50;                                                                   // Debounce delay in milliseconds for all push buttons

//////////////////////////////////
//State variables for automation//
//////////////////////////////////
bool fullCycleFirstPress = true;                                                                              // Tracks if FullCycle button has been pressed since reset
bool sequenceRunning = false;                                                                                 // Prevents overlapping sequences

// Add global variables for servo speeds
int speedServo1 = 885;                                                                                       // Default to 19mm tape
int speedServoSync = 885;                                                                                    // Default to 19mm tape

///////////////////////
//CAN Global Variables//
///////////////////////
byte replyData[8];                                                                                           // Buffer for CAN replies

// Function declarations for CAN
void process_instruction(CanFrame instruction);
void send_twai_response(const byte response_data[8]);
void twai_listener_task(void *pvParameters);

void setup()
{
  Serial.begin(115200);                                                                                       // Start serial communication

/////////////////////////
//Servos initialization//
/////////////////////////
servo1.attach(servoPin1);    
servo2.attach(servoPin2);    
servo3.attach(servoPin3);
servo4.attach(servoPin4); 
servo5.attach(servoPin5); 
servo6.attach(servoPin6);
 
servo1.writeMicroseconds(stopServo);                                                                          // Initialize the servo stopped
servo2.writeMicroseconds(stopServo);                                                                          // Initialize the servo stopped
servo3.writeMicroseconds(zeroPosition);                                                                       // Servo3 start position (Home)
servo4.writeMicroseconds(HomePosition);                                                                       // Servo4 start position (Home)
servo5.writeMicroseconds(FinalPosition);                                                                      // Servo5 final position (Open)
servo6.writeMicroseconds(S6HomePos);                                                                          // Servo6 start position (Down)

////////////////////
//Position Sensors//
////////////////////
pinMode(Cutter, INPUT_PULLUP);                                                                                // Declaring an internal Pull-Up resistor for the position sensor
pinMode(ElevatorDown, INPUT_PULLUP);                                                                          // Declaring an internal Pull-Up resistor for the position sensor

//////////////////////////////
//Hall Sensor Initialization//
//////////////////////////////
pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);                                                                       // Declares an internal Pull-Up resistor for the pin where the Hall Effect Sensor is going to be read
attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);                              // Calls the function when the Hall Effect Sensor detects a Magnetic Field change (falls)

//////////////////////
//Initialize Encoder//
//////////////////////
pinMode(13, INPUT_PULLUP);                                                                                    // SDA for Bus2 (Wrapper and Hose detection)
pinMode(23, INPUT_PULLUP);                                                                                    // SCL for Encoder2 (Wrapper)
pinMode(21, INPUT_PULLUP);                                                                                    // SDA for Encoder1 (Feeder)
pinMode(22, INPUT_PULLUP);                                                                                    // SCL for Encoder1 (Feeder)

Wire.begin(21, 22);                                                                                           // Start I2C communication for Bus 1: Feeder Encoder & RGB Feeder
if (!encoder1.begin())                                                                                        // Validation of Encoder initialization. Sends an Error Message if not successful
{
  Serial.println("FEEDER Encoder not detected.");                                                             // Message displayed
  while(1);
}
encoder1.setDirection(AS5600_CLOCK_WISE);                                                                     // Function to declare the direction of rotation of Encoder1

Wire1.begin(13, 23);                                                                                          // Start I2C communication for Wrapper Encoder
if (!encoder2.begin())                                                                                        // Validation of Encoder initialization. Sends an Error Message if not successful
{
  Serial.println("WRAPPER Encoder not detected");                                                             // Message displayed
  while(1);
}
encoder2.setDirection(AS5600_COUNTERCLOCK_WISE);                                                              // Function to declare the direction of rotation of Encoder2

//////////////////////////
//Initialize RGB sensors//
//////////////////////////
if (!tcs1.begin(TCS34725_ADDRESS, &Wire))                                                                     // Validation of RGB Feeder initialization. Sends an Error Message if not successful  
{
  Serial.println("TCS34725 #1 (Tape) not found!");                                                            // Message displayed
  while(1);
}
if (!tcs2.begin(TCS34725_ADDRESS, &Wire1))                                                                    // Validation of RGB Hose initialization. Sends an Error Message if not successful
{
  Serial.println("TCS34725 #2 (Hose) not found!");                                                            // Message displayed
  while(1);
}
Serial.println("I2C sensors initialized!");                                                                   // Successful Message

////////////////////////////
// Initialize push buttons//
////////////////////////////
pinMode(BUTTON_FULLCYCLE, INPUT_PULLUP);                                                                      // Declaring the pushbuttons as internal Input Pullup
pinMode(BUTTON_FORWARD, INPUT_PULLUP);
pinMode(BUTTON_BACKWARD, INPUT_PULLUP);

///////////////////////////////
// Initialize CAN bus (TWAI) //
///////////////////////////////
Serial.println("Initializing CAN system with FreeRTOS");

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

delay(500);                                                                                                   // Delay time to make sure everything is ready before begining. Setup completed
//Serial.println("System ready. Type a number from 1 to 11 to execute a step");                               // Message to display the setup is ready and waiting for an instruction
}

///////////////////////////////////////////////////////
//INTERRUPTION SERVICE ROUTINE FOR HALL EFFECT SENSOR//
///////////////////////////////////////////////////////
void IRAM_ATTR hallSensorISR()                                                                                
{
  static unsigned long lastInterrupt = 0;                                                                     // Register the time of the previous interruption
  unsigned long now = millis();                                                                               // Variable to register the time in miliseconds
  
  if (now - lastInterrupt > 50)                                                                               // Debounce: 50ms minimum between triggers 
  { 
    hallCounter++;                                                                                            // Hall Effect Sensor counter increments by 1
    lastInterrupt = now;                                                                                      // Save the time of the interruption
  }
}

void loop() 
{
  ////////////////////////////
  // Tape presence condition//
  ////////////////////////////
  /*if (!isTapePresent())                                                                                       // Tape check: block all steps if no tape 
  {
    Serial.println("No tape detected! System will not operate.");                                             // Displayed message
    delay(500);                                                                                               // Avoid spamming
    return;                                                                                                   // Blocks every function if there's no tape
  }*/
  elevatorIsDown = (digitalRead(ElevatorDown) == LOW);                                                        // Update elevator state
  // checkHoseAndOpenGripper();                                                                               // Check elevator sensor for gripper open

  ///////////////////////////////////
  //Push Button Debounce Test Logic//
  ///////////////////////////////////
  int readingFullCycle = digitalRead(BUTTON_FULLCYCLE);                                                       // Variable to detect the state of the FullCycle/Cut push button
  if (readingFullCycle != lastRawStateFullCycle)                                                              // If the current reading is different from the last raw state, reset debounce timer
  {
    lastDebounceTimeFullCycle = millis();                                                                     // Store the time of the state change
  }
  lastRawStateFullCycle = readingFullCycle;                                                                   // Update the last raw state with the current reading
  if ((millis() - lastDebounceTimeFullCycle) >= debounceDelayTest)                                            // Check if the debounce period has passed
  {
    if (readingFullCycle != lastStableStateFullCycle)                                                         // Check if the reading is different from the last stable state
    {
      lastStableStateFullCycle = readingFullCycle;                                                            // Update the last stable state
      if (lastStableStateFullCycle == LOW && !sequenceRunning)                                                // Button pressed and no sequence running
      {
        sequenceRunning = true;
        if (fullCycleFirstPress) {
          Serial.println("FullCycle: First press - step7(), step1()");
          step7(); // Ensure holder is in home position
          delay(10);
          step1();
          delay(10);
          fullCycleFirstPress = false;
        } else {
          Serial.println("FullCycle: step7(), step1(), step6(), step4()");
          step7(); // Ensure holder is in home position
          delay(10);
          step1();
          delay(10);
          step6();
          delay(10);
          step4();
          delay(10);
        }
        sequenceRunning = false;
      }
    }
  }

  int readingForward = digitalRead(BUTTON_FORWARD);                                                           // Variable to detect the state of the Forward push button
  if (readingForward != lastRawStateForward)                                                                  // If the current reading is different from the last raw state, reset debounce timer
  {
    lastDebounceTimeForward = millis();                                                                       // Store the time of the state change
  }
  lastRawStateForward = readingForward;                                                                       // Update the last raw state with the current reading
  if ((millis() - lastDebounceTimeForward) >= debounceDelayTest)                                              // If the debounce period has passed
  {
    if (readingForward != lastStableStateForward)                                                             // If the reading is different from the last stable state
    {
      lastStableStateForward = readingForward;                                                                // Update the last stable state
      if (lastStableStateForward == LOW && !sequenceRunning)                                                  // Button pressed and no sequence running
      {
        sequenceRunning = true;
        Serial.println("ButtonForward: step10(), delay, step8(), step7(), step12()");
        step10();
        delay(100);                                                                                           // Short delay to allow elevator to settle before closing gripper
        step8();
        gripperClosed = true;                                                                                 // Set gripper closed flag
        gripperReadyForHose = true;                                                                           // Set gripper ready for hose flag
        delay(10);
        step7();
        delay(10);
        step12();
        sequenceRunning = false;
        delay(10);
        step9();
      }
    }
  }

  int readingBackward = digitalRead(BUTTON_BACKWARD);                                                         // Variable to detect the state of the Backward push button
  if (readingBackward != lastRawStateBackward)                                                                // If the current reading is different from the last raw state, reset debounce timer
  {
    lastDebounceTimeBackward = millis();                                                                      // Store the time of the state change
  }
  lastRawStateBackward = readingBackward;                                                                     // Update the last raw state with the current reading
  if ((millis() - lastDebounceTimeBackward) >= debounceDelayTest)                                             // If the debounce period has passed
  {
    if (readingBackward != lastStableStateBackward)                                                           // If the reading is different from the last stable state
    {
      lastStableStateBackward = readingBackward;                                                              // Update the last stable state
      if (lastStableStateBackward == LOW && !sequenceRunning)                                                 // Button pressed and no sequence running
      {
        sequenceRunning = true;
        Serial.println("ButtonBackward: step2(), step3(), step4(), step6(), step5()");
        step2();
        delay(10);
        step3();
        delay(10);
        step4();
        delay(10);
        step6();
        delay(10);
        step5();
        sequenceRunning = false;
        delay(10);   
      }
    }
  }

  if (Serial.available() > 0)                                                                                 // Condition to execute the firmware if the Serial Communication started correctly 
  {
    int step = Serial.parseInt();                                                                             // Reads the typed number (instruction)
    //Serial.print("Executing step ");                                                                        // Message displayed indicating that an instruction is being executed
    //Serial.println(step);                                                                                   // Message showing the instruction ongoing

    switch (step)                                                                                             // Switch Loop. State Machine
    {
      case 1:                                                                                                 // Case 1: Feeder moves
       //if (currentState1 == 0)
       // {
          step1();
      //  }
      break;

      case 2:                                                                                                 // Case 2: Wrapper moves 1/4 of turn CCW direction
       //if (currentState2 == 0)
        //{  
          step2();
       // }
        break;

      case 3:                                                                                                 // Case 3: Wrapper moves 1/4 of turn CW direction
        //if(currentState2 == 1)
        //{  
           step3();
       // }
        break;

      case 4:                                                                                                 // Case 4: Cutter moves
        step4();
        break;

      case 5:                                                                                                 // Case 5: Wrapper moves 3 turns CW direction
       //if (currentState2 == 2)
       //{
        step5();
       //}
        break;

      case 6:                                                                                                 // Case 6: Holder moves to Hold position
        step6();
        break;

      case 7:                                                                                                 // Case 7: Holder moves to Home position
        step7();
        break;

      case 8:                                                                                                 // Case 8: Gripper moves to Close position
        step8();
        gripperClosed = true;                                                                                 // Set state when gripper closes                                                
        gripperReadyForHose = true;                                                                           // Gripper state waiting for hose trigger
        break;

      case 9:                                                                                                 // Case 9: Gripper moves to Open position
        step9();
        gripperClosed = false;                                                                                // Set state when gripper opens
        gripperReadyForHose = false;                                                                          // Disarm hose trigger
        break;

      case 10:                                                                                                // Case 10: Elevator moves to Up position
        step10();
        break;

      case 11:                                                                                                // Case 11: Elevator moves to Down position
        step11();
        break;

      case 12:                                                                                                // Case 12: Elevator moves to Down position while Feeder moves until the elevator reaches Down position
        step12();
        break;

      case 13:                                                                                                // Case 13: Feeder moves in reverse direction
        step13();
        break;

      case 14: {                                                                                              // Case 14: Part Number selection. This step changes the SpeedServo1 value to adjust it to the type of Tape
        Serial.println("Select Part Number: 1 for 19mm tape, 2 for 10mm tape");
        while (Serial.available() == 0) { /* wait for input */ }
        int pn = Serial.parseInt();
        if (pn == 1) {
          speedServo1 = 1390;
          speedServoSync = 1347;
          Serial.println("19mm tape selected. Speeds set.");
        } else if (pn == 2) {
          speedServo1 = 1410;
          speedServoSync = 1387;
          Serial.println("10mm tape selected. Speeds set.");
        } else {
          Serial.println("Invalid selection. Type 1 or 2.");
        }
        break;
      }

      case 20:                                                                                                // Case 20: Reset all the flags
       resetStep();
        break;

      default:
        //Serial.println("Not valid number. Type a number from 1 to 12 to execute a step");                   // Default Case displays a message indicating the need of a valid instruction
        break;
    }
  }

  //////////////////////////////
  // Process CAN Instructions //
  //////////////////////////////
  CanFrame instruction;
  if (xQueueReceive(instruction_queue, &instruction, (TickType_t)0) == pdPASS) {
    process_instruction(instruction);
  }
}

//////////
//FEEDER//
//////////
void step1()
{
  servo1.attach(servoPin1);                                                                                    // Function to ensure the communication with Servo1 (feeder)
  float initialAngle = encoder1.readAngle() * encoderRes;                                                      // Get starting angle reading the encoder value times the encoder resolution
  const float targetMovement = degreesPerFeed - GapGainFeeder;                                                 // Degrees to move
  
  unsigned long startTime = millis();                                                                          // Variable to register the time of initialization in milliseconds
  float totalMoved = 0.0;                                                                                      // Variable to register the amount of degrees that the servo has moved
  float previousAngle = initialAngle;                                                                          // Variable to register the last angle value before moving

  servo1.writeMicroseconds(speedServo1);                                                                       // Start moving servo1 (feeder)

  while (totalMoved < targetMovement && (millis() - startTime < timeout))                                      // Loop to read the angle and modify the angle before stopping the the servo. Conditions include Angle and Working Time 
  {
    float currentAngle = encoder1.readAngle() * encoderRes;                                                    // Variable to read in real-time the current position of the servo1 (feeder)    
    float delta = currentAngle - previousAngle;                                                                // Calculate delta movement with wrap-around
    if (delta < -180.0) delta += 360.0;                                                                        // Improved wrap-around detection
    if (delta > 180.0) delta -= 360.0;                                                                         // Improved wrap-around detection
    totalMoved += abs(delta);                                                                                  // Accumulate movement
    previousAngle = currentAngle;                                                                              // Reset the previousAngle value for last movement
    delay(20);                                                                                                 // Short delay for stability
  }
  //Serial.print("Moved: ");                                                                                   // Message to display the quantity of degrees moved
  Serial.println(totalMoved);                                                                                  // Shows to value of the Accumulate movement (degrees moved)
  //Serial.println("°");                                                                                       // Degree symbol
  servo1.writeMicroseconds(stopServo);                                                                         // Stop servo
  //servo1.detach();                                                                                           // Function to ensure the servo stopped. Not recommended to use

  if (totalMoved >= targetMovement)                                                                            // Condition to verify if the servo1 moved 
  {
    Serial.println("Feed complete.");                                                                          // Message displayed indicating that the movement was succesful
  } 
  else 
  {
    Serial.println("Timeout! Check mechanical load or increase timeout.");                                     // Error message indicating that the working time was complete
  }
}

/////////////////////////////////
//WRAPPER 1/4 spin CCW direction//
/////////////////////////////////
void step2() 
{
  encoder2.setDirection(AS5600_COUNTERCLOCK_WISE);                                                             // Confirm physical direction matches
  servo2.attach(servoPin2);                                                                                    // Ensures Servo2 is attached to the MCU
  float initialAngle2 = encoder2.readAngle() * encoderRes;                                                     // Variable to register the initial position of the Encoder Gear
  const float targetMovement = degreesPerWrap - GapGainWrapper1;                                               // Variable to calculate the degrees to move, the Gap Gain fixes the last position

  Serial.print("Target Movement: ");                                                                           // Print a message for debug
  Serial.println(targetMovement);                                                                              // Debugging

  unsigned long startTime = millis();                                                                          // Variable store the start time of execution in milliseconds   
  float totalMoved = 0.0;                                                                                      // Variable to accumulete the total quantity of degrees during the movement
  float previousAngle = initialAngle2;                                                                         // Variable to store the last angle value before move
  
  servo2.writeMicroseconds(speedServo2_CCW);                                                                   // Command to initialize the movement of Servo2 in CCW direction

  while (totalMoved < targetMovement && (millis() - startTime < timeout2))                                     // Conditionals to perform the movement of the Servo 
  {
    float currentAngle = encoder2.readAngle() * encoderRes;                                                    // Variable to continously read the angle read by the encoder
    float delta = currentAngle - previousAngle;                                                                // Handle wrap-around at 360° boundary
    if (delta < -180.0) delta += 360.0;                                                                        // Improved wrap-around detection
    if (delta > 180.0) delta -= 360.0;                                                                         // Improved wrap-around detection

    totalMoved += abs(delta);                                                                                  // Use absolute delta
    previousAngle = currentAngle;                                                                              // Resets the previous angle value to the final position

    Serial.print("Moved: ");                                                                                   // Print a message for debug
    Serial.print(totalMoved);                                                                                  // Prints the degrees moved
    Serial.print(" | Time: ");                                                                                 // Prints the time of movement for debug
    Serial.println(millis() - startTime);                                                                      // Debugging
    
    delay(20);                                                                                                 // Short delay for stability
  }

  servo2.writeMicroseconds(stopServo);                                                                         // After all conditions denied the servo will stop
  
  if (totalMoved >= targetMovement)                                                                            // Condition to validate the degrees moved and determine success or an error 
  {
    Serial.println("Step 2: Initial Wrap Completed");                                                          // Message indicating a succesfull movement
  } 
  else
  {
    Serial.println("Step 2: Timeout - Check Encoder or Mechanical Load");                                      // Message indicating an error
  }
}

/////////////////////////////////
//WRAPPER 1/2 spin CW direction//
/////////////////////////////////
void step3() 
{
  encoder2.setDirection(AS5600_CLOCK_WISE);                                                                    // Direction of the Encodeer, changed for CW movement
  servo2.attach(servoPin2);                                                                                    // Ensures the servo is communicating with the MCU
  float initialAngle2 = encoder2.readAngle() * encoderRes;                                                     // Variable to register the initial angle of the Encoder before moving
  const float targetMovement = degreesPerWrap2 - GapGainWrapper1;                                              // Variable to calculate the degrees to move, the Gap Gain fixes the last position

  Serial.print("Target Movement: ");                                                                           // Print a message for debug
  Serial.println(targetMovement);                                                                              // Debugging

  unsigned long startTime = millis();                                                                          // Variable store the start time of execution in milliseconds 
  float totalMoved = 0.0;                                                                                      // Variable to accumulete the total quantity of degrees during the movement
  float previousAngle = initialAngle2;                                                                         // Variable to store the last angle value before move
  
  servo2.writeMicroseconds(speedServo2_CW);                                                                    // Command to initialize the movement of Servo2 in CW direction

  // Add motion stabilization, these 2 variables are use to detect 'No movement' of the wrapper
  int stableCount = 0;                                                                                         // Counter of 'Not Movement' event 
  const int maxStableCount = 5;                                                                                // Max threshold for allowed consecutive 'Not Movement' iterations

  while (totalMoved < targetMovement && (millis() - startTime < timeout2))                                     // Conditionals to perform the movement of the Servo
  {
    float currentAngle = encoder2.readAngle() * encoderRes;                                                    // Variable to continously read the angle read by the encoder
    
    float delta = previousAngle - currentAngle;                                                                // CW movement decreases angle
    if (delta < -180.0) delta += 360.0;                                                                        // Improved wrap-around detection
    if (delta > 180.0) delta -= 360.0;                                                                         // Improved wrap-around detection

    totalMoved += abs(delta);                                                                                  // Use absolute delta
    previousAngle = currentAngle;                                                                              // Resets the previous angle value to the final position

    if (abs(delta) < 1.0)                                                                                      // Detect stalling. Adjust threshold as needed
    { 
      stableCount++;                                                                                           // Increments in 1 the 'Not Movement' counter
      if (stableCount > maxStableCount)                                                                        // Conditional to determine if there's an error 
      {
        Serial.println("Stall detected!");                                                                     // Prints message indcating an error
        break;                                                                                                 // Ends the loop
      }
    } 
    else 
    {
      stableCount = 0;                                                                                         // Resets the counter to 0
    }
    
    delay(20);                                                                                                 // Short delay for stabilization
  }

  servo2.writeMicroseconds(stopServo);                                                                         // After all conditions denied the servo will stop
  
  if (totalMoved >= targetMovement)                                                                            // Condition to validate the degrees moved and determine success or an error 
  {
    Serial.println("Step 3: Half-Turn Completed");                                                             // Message indicating a succesfull movement
  } 
  else 
  {
    Serial.print("Step 3: Partial Movement - ");                                                               // Message indicating an error
    Serial.println(totalMoved);
  }
}
//////////
//CUTTER//
//////////
void step4()
{
  servo3.attach(servoPin3);                                                                                    // Ensure servo is attached and powered
  int angle1 = map(angle_cut, angleMin, angleMax, microsecondsMin, microsecondsMax);                           // Function to map the Cut Angle based on the parameters of the servo
  int angle2 = map(angle_home, angleMin, angleMax, microsecondsMin, microsecondsMax);                          // Function to map the Home Angle based on the parameters of the servo
  servo3.writeMicroseconds(angle2);                                                                            // Wakeup pulse
  delay(300);                                                                                                  // 300 ms pause between movements
  servo3.writeMicroseconds(angle1);                                                                            // Servo3 (cutter) moves to Cut position
  Serial.println("Servo3 moved to cutting angle.");                                                            // Message indicating the movement performed
  delay(1000);                                                                                                 // 1000 ms pause between movements
  servo3.writeMicroseconds(angle2);                                                                            // Servo 3 returns to Home position
  Serial.println("Servo3 returned to Home position.");                                                         // Message indicating the movement performed
  delay(1000);                                                                                                 // 1000 ms pause between movements
  Serial.println("Step 4 completed.");                                                                         // Message indicating the end of the cycle

}

////////////////////////////////
//WRAPPER 5 spins CW direction//
////////////////////////////////
void step5() 
{
  // --- Robust: Wait for first trigger, then count 4 spins ---
  hallCounter = 0;                                                                                             // Reset Hall Effect Sensor counter
  encoder2.setDirection(AS5600_COUNTERCLOCK_WISE);                                                             // Direction of the Encoder, changed for CCW movement
  servo2.attach(servoPin2);                                                                                    // Ensures the servo is communicating with the MCU
  const float degreesPerRotation = 360.0 * GEAR_RATIO;                                                         // Variable directly declared in the function for better control. It represents a full spin
  const float targetMovement = degreesPerRotation * TARGET_ROTATIONS - GapGainWrapper1;                        // Variable to calculate the target angle to reach. It represents 4 spins

  unsigned long startTime = millis();                                                                          // Variable store the start time of execution in milliseconds
  float initialAngle = encoder2.readAngle() * encoderRes;                                                      // Variable to register the initial angle of the Encoder before moving
  float previousAngle = initialAngle;                                                                          // Variable to store the last angle value before move
  float totalMoved = 0.0;                                                                                      // Variable to accumulete the total quantity of degrees during the movement

  servo2.writeMicroseconds(speedServo2_CW);                                                                    // Command to initialize the movement of Servo2 in CW direction

  // Move until first Hall trigger
  Serial.println("Waiting for first Hall trigger to start spin count...");
  int initialHall = hallCounter;
  while ((millis() - startTime < timeout2) && (hallCounter == initialHall)) 
  {
    // Just wait for the first trigger
    delay(1);
  }
  // Now start counting spins
  hallCounter = 0;
  Serial.println("First Hall trigger detected. Starting spin count...");

  while((millis() - startTime < timeout2) && (hallCounter < TARGET_ROTATIONS))                                 // Conditionals to perform the movement of the Servo
  {
    float currentAngle = encoder2.readAngle() * encoderRes;                                                    // Variable to continously read the angle read by the encoder                                                 
    float delta = previousAngle - currentAngle;                                                                // CW movement decreases angle
    if(delta < 0) delta += 360.0;                                                                              // Improved wrap-around detection
    totalMoved += delta;                                                                                       // Increases Delta value to the totalMoved variable
    previousAngle = currentAngle;                                                                              // Resets the previous angle value to the final position

    Serial.print("Hall Count: ");                                                                              // Message printed for Debug
    Serial.print(hallCounter);                                                                                 // Prints the quantatity of times the Hall Sensor went triggered
    Serial.print("/4 | Moved: ");                                                                              // Message printed for Debug
    Serial.print(totalMoved);                                                                                  // Prints the quantity of degrees moved
    Serial.println("°");

    delay(20);                                                                                                 // Short delay for stabilization
  } 

  servo2.writeMicroseconds(stopServo);                                                                         // After all conditions denied the servo will stop
  if(hallCounter >= TARGET_ROTATIONS)                                                                          // Conditional to detect succesfull movement or an error based on the Hall Effect Sensor input 
  {
    Serial.println("Stopped by Hall Sensor. 4 rotations complete!");                                           // Message printed if succcesfull movement                                            
    // --- Begin CCW Correction ---
    float correctionDegrees = 80.0;                                                                            // Correction value for the wrapper to coincide with the structure
    float startAngle = encoder2.readAngle() * encoderRes;                                                      // Variable to save the stop position
    float moved = 0.0;                                                                                         // Variable to calculate the degress moved
    float prevAngle = startAngle;                                                                              // Variable to store the previous angle value
    servo2.writeMicroseconds(speedServo2_CCW);                                                                 // Begin movement of the wrapper in CCW direction
    unsigned long correctionStartTime = millis();                                                              // Reset timer for correction phase
    while (moved < correctionDegrees && (millis() - correctionStartTime < timeout2))                           // Correction loop with its own timeout
    {
        float currAngle = encoder2.readAngle() * encoderRes;                                                   // Variable to read in real time the current position of the wrapper
        float delta = currAngle - prevAngle;                                                                   // Delta calculation
        if (delta < -180.0) delta += 360.0;                                                                    // Handle wrap-around
        if (delta > 180.0) delta -= 360.0;                                                                     // Handle wrap-around
        moved += abs(delta);                                                                                   // Updates the degrees moved value
        prevAngle = currAngle;                                                                                 // Updates the previous angle value
        Serial.print("Correction moved: ");                                                                    // Debug print
        Serial.print(moved);
        Serial.print(" / ");
        Serial.println(correctionDegrees);
        delay(20);                                                                                             // Short delay for stabilization
    }
    servo2.writeMicroseconds(stopServo);                                                                       // Stop movement after correction
    Serial.println("CCW correction complete.");                                                                // Message printed indicating a succesful correction
  } 
  else 
  {
    Serial.println("Timeout!");                                                                                // Message printed if Servo stopped due to Timeout
  }
}

////////////////////////
//HOLDER Hold Position//
////////////////////////
void step6()
{
  int angle_holdTape = map(angle_hold4, minTravelAngle, MaxTravelAngle, pulseLenghtMin, pulseLenghtMax);       // Function to map the Holding Angle based on the servo4 parameters
  servo4.writeMicroseconds(angle_holdTape);                                                                    // Servo4 moves to Holding position
  Serial.println("Servo 4 moved to holding angle.");                                                           // Message displayed indicating the servo4 is in Holding position
  delay(200);                                                                                                  // Short Delay for system stability
}

////////////////////////
//HOLDER Home Position//
////////////////////////
void step7()
{
  int angle_holdHome = map(angle_home4, minTravelAngle, MaxTravelAngle, pulseLenghtMin, pulseLenghtMax);       // Function to map the Home Angle based on the servo4 parameters
  servo4.writeMicroseconds(angle_holdHome);                                                                    // Servo4 moves to Home position
  Serial.println("Servo 4 moved to home angle.");                                                              // Message displayed indicating the servo4 is in Home position
  delay(200);                                                                                                  // Short delay for system stability
}

//////////////////////////
//GRIPPER Close Position//
//////////////////////////
void step8()
{
  int angle_holdTape = map(angle_hold5, minTravelAngle, MaxTravelAngle, pulseLenghtMin, pulseLenghtMax);       // Function to map the Hold Angle based on the servo5 parameters
  servo5.writeMicroseconds(angle_holdTape);                                                                    // Servo5 moves to Close position
  Serial.println("Servo 5 moved to holding angle.");                                                           // Message printed indicating the end of the movement                                                       
  delay(200);                                                                                                  // Short delay for system stability
}

/////////////////////////
//GRIPPER Open Position//
/////////////////////////
void step9()
{
  int angle_holdHome = map(angle_home5, minTravelAngle, MaxTravelAngle, pulseLenghtMin, pulseLenghtMax);       // Function to map the Open Angle based on the servo5 parameters
  servo5.writeMicroseconds(angle_holdHome);                                                                    // Servo5 moves to Open position
  Serial.println("Servo 5 moved to home angle.");                                                              // Message printed indicating the end of the movement
  delay(700);                                                                                                  // Short delay for system stability
}

////////////////////////
//ELEVATOR Up Position//
////////////////////////
void step10()
{
  servo6.attach(servoPin6);                                                                                    // Ensures the servo6 is communicating with the MCU
  step9();                                                                                                     // Open gripper while elevator goes up
  for (int angle = S6HomePos; angle <= S6FinalPos; angle += 1)                                                 // For loop to move the servo degree per degree 
  {
    servo6.writeMicroseconds(angle);                                                                           // Servo moves to the angle calculated in the For loop
    delay(servo6StepDelay);                                                                                    // Delay to determine the angular speed of the movement
  }
  servo6.writeMicroseconds(S6FinalPos);                                                                        // Command to ensure elevator reaches the Up position
  Serial.println("Elevator UP.");                                                                              // Message printed at the end of the movement
}

//////////////////////////
//ELEVATOR Down Position//
//////////////////////////
void step11()
{
  servo6.attach(servoPin6);                                                                                    // Ensures the servo6 is communicating with the MCU
  for (int angle = S6FinalPos; angle >= S6HomePos; angle -= 1)                                                 // For loop to move the servo degree per degree 
  {
    servo6.writeMicroseconds(angle);                                                                           // Servo moves to the angle calculated in the For loop                                                                        
    delay(servo6StepDelay);                                                                                    // Delay to determine the angular speed of the movement
  }
  servo6.writeMicroseconds(S6HomePos);                                                                         // Command to ensure the elevator reaches the Down position
  Serial.println("Elevator DOWN.");                                                                            // Message printed at the end of the movement
}

//////////////////////////////////////
//SYNCHRONIZED MOVEMENT Servos 1 & 6//
//////////////////////////////////////
void step12() 
{
  servo1.attach(servoPin1);                                                                                     // Ensures servo1 is communicating with the MCU
  servo6.attach(servoPin6);                                                                                     // Ensures servo6 is communicating with the MCU
  servo6.writeMicroseconds(S6FinalPos);                                                                         // Writes the current current position to avoid unwanted movement

  float initialAngle = encoder1.readAngle() * encoderRes;                                                       // Get starting angle from encoder1 (Feeder)
  const float targetMovement = degreesPerFeedSync - GapGainFeeder;                                              // Degrees to move for servo1, adjusted by gap gain
  float totalMoved = 0.0;                                                                                       // Variable to register the degrees moved by servo1
  float previousAngle = initialAngle;                                                                           // Variable to register the previous angle before movement (for delta calculation)
  bool servo1Done = false;                                                                                      // Boolean variable to detect if servo1 reached its target

  int currentAngle6 = S6FinalPos;                                                                               // Variable to register the current position of servo6 (Elevator), starts at Up
  bool servo6Done = false;                                                                                      // Boolean variable to detect if servo6 completed its movement
  unsigned long lastMoveTime6 = millis();                                                                       // Variable to register the time of the last movement of servo6
  unsigned long stepDelay6 = (unsigned long)servo6StepDelay;                                                    // Delay between servo6 steps, converted to unsigned long

  servo1.writeMicroseconds(speedServoSync);                                                                     // Start Servo1 at the speed for synchronized movement                                                                                                 // Give Servo1 a 150ms head start before starting Servo6
  delay(100);

  while (!servo1Done || !servo6Done)                                                                            // Loop until both servos have completed their movements
  {   
    if (!servo1Done)                                                                                            // If servo1 has not finished its movement
    {
      float currentAngle1 = encoder1.readAngle() * encoderRes;                                                  // Read the current angle from encoder1
      float delta = currentAngle1 - previousAngle;                                                              // Calculate the change in angle since last check
      if (delta < -180.0) delta += 360.0;                                                                       // Handle wrap-around (negative direction)
      if (delta > 180.0) delta -= 360.0;                                                                        // Handle wrap-around (positive direction)
      totalMoved += abs(delta);                                                                                 // Accumulate the absolute movement
      previousAngle = currentAngle1;                                                                            // Update previous angle for next iteration
      if (totalMoved >= targetMovement)                                                                         // If the target movement is reached
      {
        servo1.writeMicroseconds(stopServo);                                                                    // Stop servo1
        servo1.detach();                                                                                        // Detach servo1 to ensure it stops
        servo1Done = true;                                                                                      // Mark servo1 as done
        Serial.println("Servo1: Synchronized feed complete.");                                                  // Print completion message
      }
    }
    if (!servo6Done)                                                                                            // If servo6 has not finished its movement
    {
      unsigned long now = millis();                                                                             // Get the current time
      if (currentAngle6 > S6HomePos && (now - lastMoveTime6 >= stepDelay6))                                     // If enough time has passed and servo6 is not yet at the bottom
      {
        currentAngle6 -= 1.5;                                                                                     // Move servo6 down by 1 unit
        servo6.writeMicroseconds(currentAngle6);                                                                // Command servo6 to new position
        lastMoveTime6 = now;                                                                                    // Update last movement time
      } 
      else if (currentAngle6 <= S6HomePos)                                                                      // If servo6 has reached or passed the bottom
      {
        servo6.writeMicroseconds(S6HomePos);                                                                    // Ensure servo6 is at the bottom
        servo6Done = true;                                                                                      // Mark servo6 as done
        Serial.println("Servo6: Elevator down complete.");                                                      // Print completion message
      }
    }
  }
  Serial.println("Both servos synchronized movement complete.");                                                // Print final completion message
}

void resetStep()
{
  //currentState2 = 0;  // Reset to first direction
 // currentState1 = 0;  // Reset to first direction
}

////////////////////////////////
// Tape presence check logic //
////////////////////////////////
bool isTapePresent()                                                                                            // Helper: Tape presence check 
{
  uint16_t r, g, b, c;                                                                                          // Variables to store the raw data from the RGB sensor
  tcs1.getRawData(&r, &g, &b, &c);                                                                              // Read raw data from the RGB sensor (Feeder)
  return c > 500;                                                                                               // Return true if the clear channel value is above threshold (tape present)
}


///////////////////////////////////////////////////
// Gripper open triggered by ElevatorDown sensor //
///////////////////////////////////////////////////
void checkHoseAndOpenGripper()                                                                                  // Helper: Gripper open by ElevatorDown
{
  // Function disabled: gripper will not open automatically when elevator is down.
  // if (gripperClosed && elevatorIsDown && gripperReadyForHose)                                                // Only trigger if all conditions are met
  // {
  //   int openPulse = map(angle_home5, minTravelAngle, MaxTravelAngle, pulseLenghtMin, pulseLenghtMax);         // Calculate the pulse width to open the gripper
  //   servo5.writeMicroseconds(openPulse);                                                                      // Command the gripper servo to open
  //   Serial.println("ElevatorDown detected! Gripper opened.");                                                 // Print message to Serial Monitor
  //   delay(7);                                                                                                 // Allow time for gripper to open, but keep <10ms for high sample rate
  //   gripperClosed = false;                                                                                    // Update gripper state to open
  //   gripperReadyForHose = false;                                                                              // Disarm trigger until next cycle
  // }
}

void step13()
{
  servo1.attach(servoPin1);                                                                                    // Ensure communication with Servo1 (feeder)
  float initialAngle = encoder1.readAngle() * encoderRes;                                                      // Get starting angle
  const float targetMovement = degreesPerFeed - GapGainFeeder;                                                 // Degrees to move

  unsigned long startTime = millis();                                                                          // Register start time
  float totalMoved = 0.0;                                                                                      // Register degrees moved
  float previousAngle = initialAngle;                                                                          // Last angle value

  // Move feeder in reverse direction
  servo1.writeMicroseconds(2000);                                                                              // >1500us  reverse continuous rotation servos

  while (totalMoved < targetMovement && (millis() - startTime < timeout))                                      // Loop to move until target or timeout
  {
    float currentAngle = encoder1.readAngle() * encoderRes;
    float delta = previousAngle - currentAngle;                                                                // Reverse direction
    if (delta < -180.0) delta += 360.0;
    if (delta > 180.0) delta -= 360.0;
    totalMoved += abs(delta);
    previousAngle = currentAngle;
    delay(20);
  }
  Serial.println(totalMoved);
  servo1.writeMicroseconds(stopServo);                                                                         // Stop servo

  if (totalMoved >= targetMovement)
  {
    Serial.println("Reverse feed complete.");
  } 
  else 
  {
    Serial.println("Timeout! Check mechanical load or increase timeout.");
  }
}

///////////////////////
// CAN Bus Functions //
///////////////////////

/*
 * twai_listener_task(void *pvParameters)
 * --------------------------------------
 * FreeRTOS task running on core 0. Continuously listens for new CAN frames on the TWAI bus.
 * - Accepts only frames matching DEVICE_CAN_ID.
 * - Places valid instructions in the instruction_queue for processing by the main loop.
 * - Ignores frames not addressed to this device.
 * - Uses a small delay to avoid CPU saturation.
 */
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

/*
 * process_instruction(CanFrame instruction)
 * ----------------------------------------
 * Decodes and executes CAN instructions received from the queue.
 * Basic implementation with heartbeat and reset commands.
 * More commands can be added as needed for taping operations.
 */
void process_instruction(CanFrame instruction)
{
  Serial.print("Processing command: 0x");
  Serial.println(instruction.data[0], HEX);

  switch (instruction.data[0])
  {
    // ***************************** CASE 0x01 ***************************** //
    // Reset microcontroller
    case 0x01: 
    {
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
      Serial.println("Case 0x02: Send Heartbeat");
      byte response[] = {0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x03 ***************************** //
    // Execute step1 - Feeder
    case 0x03:
    {
      Serial.println("Case 0x03: Execute step1 - Feeder");
      step1();
      byte response[] = {0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x04 ***************************** //
    // Execute step2 - Cutter
    case 0x04:
    {
      Serial.println("Case 0x04: Execute step2 - Cutter");
      step2();
      byte response[] = {0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x05 ***************************** //
    // Execute step3 - Applicator
    case 0x05:
    {
      Serial.println("Case 0x05: Execute step3 - Applicator");
      step3();
      byte response[] = {0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x06 ***************************** //
    // Execute step4 - Holder
    case 0x06:
    {
      Serial.println("Case 0x06: Execute step4 - Holder");
      step4();
      byte response[] = {0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x07 ***************************** //
    // Execute step5 - Holder Home
    case 0x07:
    {
      Serial.println("Case 0x07: Execute step5 - Holder Home");
      step5();
      byte response[] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x08 ***************************** //
    // Execute step6 - Applicator Home
    case 0x08:
    {
      Serial.println("Case 0x08: Execute step6 - Applicator Home");
      step6();
      byte response[] = {0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x09 ***************************** //
    // Execute step7 - Holder Home Position
    case 0x09:
    {
      Serial.println("Case 0x09: Execute step7 - Holder Home Position");
      step7();
      byte response[] = {0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0A ***************************** //
    // Execute step8 - Gripper Close
    case 0x0A:
    {
      Serial.println("Case 0x0A: Execute step8 - Gripper Close");
      step8();
      byte response[] = {0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0B ***************************** //
    // Execute step9 - Gripper Open
    case 0x0B:
    {
      Serial.println("Case 0x0B: Execute step9 - Gripper Open");
      step9();
      byte response[] = {0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0C ***************************** //
    // Execute step10 - Elevator Down
    case 0x0C:
    {
      Serial.println("Case 0x0C: Execute step10 - Elevator Down");
      step10();
      byte response[] = {0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0D ***************************** //
    // Execute step11 - Elevator Up
    case 0x0D:
    {
      Serial.println("Case 0x0D: Execute step11 - Elevator Up");
      step11();
      byte response[] = {0x0D, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0E ***************************** //
    // Execute step12 - Feeder Reverse
    case 0x0E:
    {
      Serial.println("Case 0x0E: Execute step12 - Feeder Reverse");
      step12();
      byte response[] = {0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x0F ***************************** //
    // Execute step13 - Feeder Reverse (Alternative)
    case 0x0F:
    {
      Serial.println("Case 0x0F: Execute step13 - Feeder Reverse (Alternative)");
      step13();
      byte response[] = {0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x10 ***************************** //
    // Execute FullCycle sequence
    case 0x10:
    {
      Serial.println("Case 0x10: Execute FullCycle sequence");
      if (fullCycleFirstPress) {
        Serial.println("FullCycle: First press - step7(), step1()");
        step7(); // Ensure holder is in home position
        delay(10);
        step1();
        delay(10);
        fullCycleFirstPress = false;
      } else {
        Serial.println("FullCycle: step7(), step1(), step6(), step4()");
        step7(); // Ensure holder is in home position
        delay(10);
        step1();
        delay(10);
        step6();
        delay(10);
        step4();
        delay(10);
      }
      byte response[] = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x11 ***************************** //
    // Execute Forward sequence
    case 0x11:
    {
      Serial.println("Case 0x11: Execute Forward sequence");
      Serial.println("ButtonForward: step10(), delay, step8(), step7(), step12()");
      step10();
      delay(100); // Short delay to allow elevator to settle before closing gripper
      step8();
      gripperClosed = true; // Set gripper closed flag
      gripperReadyForHose = true; // Set gripper ready for hose flag
      delay(10);
      step7();
      delay(10);
      step12();
      delay(10);
      step9();
      byte response[] = {0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
    }
    break;

    // ***************************** CASE 0x12 ***************************** //
    // Execute Backward sequence
    case 0x12:
    {
      Serial.println("Case 0x12: Execute Backward sequence");
      Serial.println("ButtonBackward: step2(), step3(), step4(), step6(), step5()");
      step2();
      delay(10);
      step3();
      delay(10);
      step4();
      delay(10);
      step6();
      delay(10);
      step5();
      delay(10);
      byte response[] = {0x12, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_twai_response(response);
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

/*
 * send_twai_response(const byte response_data[8])
 * -----------------------------------------------
 * Sends a response frame on the TWAI (CAN0) bus with the RESPONSE_CAN_ID.
 * - Used to acknowledge commands, report status, or send data back to the master controller.
 * - The response_data buffer must be 8 bytes.
 */
void send_twai_response(const byte response_data[8]) {
  CanFrame tx_frame;
  tx_frame.identifier = RESPONSE_CAN_ID;  // Response ID
  tx_frame.extd = 0;
  tx_frame.data_length_code = 8;
  memcpy(tx_frame.data, response_data, 8);
  ESP32Can.writeFrame(tx_frame);
}
