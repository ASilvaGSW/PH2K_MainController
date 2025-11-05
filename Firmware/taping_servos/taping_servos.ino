#include <ESP32Servo.h>

// Define servo pins
const int FEEDER_PIN = 2;
const int WRAPPER_PIN = 25;
const int CUTTER_PIN = 15;
const int HOLDER_PIN = 14;
const int GRIPPER_PIN = 19;
const int ELEVATOR_PIN = 17;

// Create servo objects
Servo feederServo;
Servo wrapperServo;
Servo cutterServo;
Servo holderServo;
Servo gripperServo;
Servo elevatorServo;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Attach servos to their respective pins
  feederServo.attach(FEEDER_PIN);
  wrapperServo.attach(WRAPPER_PIN);
  cutterServo.attach(CUTTER_PIN);
  holderServo.attach(HOLDER_PIN);
  gripperServo.attach(GRIPPER_PIN);
  elevatorServo.attach(ELEVATOR_PIN);

  Serial.println("Servo controller ready.");
  Serial.println("Send commands in the format: <servo_name>:<microseconds>");
  Serial.println("Example: feeder:1500");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    int colonIndex = command.indexOf(':');
    if (colonIndex > 0) {
      String servoName = command.substring(0, colonIndex);
      int microseconds = command.substring(colonIndex + 1).toInt();

      if (servoName.equalsIgnoreCase("feeder")) {
        feederServo.writeMicroseconds(microseconds);
        Serial.print("Feeder set to ");
        Serial.print(microseconds);
        Serial.println("us");
      } else if (servoName.equalsIgnoreCase("wrapper")) {
        wrapperServo.writeMicroseconds(microseconds);
        Serial.print("Wrapper set to ");
        Serial.print(microseconds);
        Serial.println("us");
      } else if (servoName.equalsIgnoreCase("cutter")) {
        cutterServo.writeMicroseconds(microseconds);
        Serial.print("Cutter set to ");
        Serial.print(microseconds);
        Serial.println("us");
      } else if (servoName.equalsIgnoreCase("holder")) {
        holderServo.writeMicroseconds(microseconds);
        Serial.print("Holder set to ");
        Serial.print(microseconds);
        Serial.println("us");
      } else if (servoName.equalsIgnoreCase("gripper")) {
        gripperServo.writeMicroseconds(microseconds);
        Serial.print("Gripper set to ");
        Serial.print(microseconds);
        Serial.println("us");
      } else if (servoName.equalsIgnoreCase("elevator")) {
        elevatorServo.writeMicroseconds(microseconds);
        Serial.print("Elevator set to ");
        Serial.print(microseconds);
        Serial.println("us");
      } else {
        Serial.print("Unknown servo: ");
        Serial.println(servoName);
      }
    } else {
      Serial.println("Invalid command format. Use: <servo_name>:<microseconds>");
    }
  }
}