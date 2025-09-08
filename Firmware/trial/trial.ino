#include <AccelStepper.h>

// Pin definitions
const int STEP_PIN = 13;    // GPIO13 - Step pin
const int DIR_PIN = 14;     // GPIO14 - Direction pin
const int ENABLE_PIN = 21;  // GPIO21 - Enable pin (LOW = enabled, HIGH = disabled)

// Motor specs
const float STEPS_PER_MM = 100;  // 200 steps/rev ÷ 2mm pitch = 100 steps/mm
const float SPEED_MM_S = 40;     // 40 mm/s
const float SPEED_STEPS_S = (SPEED_MM_S * STEPS_PER_MM) * 25;  // 4000 steps/s

// Create stepper object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  
  // Configure enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable the stepper driver (LOW = enabled)
  
  // Configure stepper
  stepper.setMaxSpeed(SPEED_STEPS_S);
  stepper.setAcceleration(SPEED_STEPS_S);
  stepper.setCurrentPosition(0);
  
  Serial.println("Simple ESP32 Stepper Test");
  Serial.println("Commands: f=forward 50mm, b=back to 0, s=stop, e=enable, d=disable, m=move to position");
}

void moveToPosition() {
  Serial.println("Enter target position in steps (negative for reverse):");
  
  // Wait for user input
  while (!Serial.available()) {
    stepper.run(); // Keep running current movement
  }
  
  // Read the target position
  long targetPosition = Serial.parseInt();
  
  Serial.print("Moving to position: ");
  Serial.print(targetPosition);
  Serial.println(" steps");
  
  stepper.moveTo(targetPosition);
}

void loop() {
  stepper.run();
  
  if (Serial.available()) {
    char cmd = Serial.read();
    
    if (cmd == 'f') {
      Serial.println("Moving forward 50mm");
      stepper.moveTo(-150000);  // 50mm * 100 steps/mm = -5000 steps (dirección invertida)
    }
    else if (cmd == 'b') {
      Serial.println("Moving back to 0");
      stepper.moveTo(0);
    }
    else if (cmd == 's') {
      Serial.println("Stop");
      stepper.stop();
    }
    else if (cmd == 'e') {
      Serial.println("Enabling stepper");
      digitalWrite(ENABLE_PIN, LOW);  // Enable stepper
    }
    else if (cmd == 'd') {
      Serial.println("Disabling stepper");
      digitalWrite(ENABLE_PIN, HIGH); // Disable stepper
    }
    else if (cmd == 'm') {
      moveToPosition();
    }
  }
}