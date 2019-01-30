// Arduino stepper motor control code

#include <Stepper.h> // Include the header file

// Define Variables
// ########################################
const int dirPin = 2; // Direction
const int stepPin = 3; // Step

// Motor steps per rotation
const int STEPS_PER_REV = 200;

// Revolutions required
float Rev;

// Maximum range of motion

const float motionRange = 18; // mm

// Variable checks if range boundaries are exceeded.
float counterCheck = 0;

// Velocity
const int Vel = 1900;

// ########################################

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  Serial.println("Please enter your command: ");
  while (Serial.available() == 0) {
  }

  Rev = Serial.parseFloat();
  Serial.println(Rev);

  counterCheck = counterCheck + Rev;
  if (counterCheck >= 0 && counterCheck <= motionRange) {

    if (Rev < 0) {
      digitalWrite(dirPin, LOW);
    }
    else if (Rev >= 0) {
      digitalWrite(dirPin, HIGH);
    }

    for (int x = 0; x < abs(Rev) * STEPS_PER_REV; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000000. / Vel);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000000. / Vel);
    }
  }
  else {
    Serial.println("Didn't move because you exceed range of motion! Change your command: ");
    Serial.println("");
    counterCheck = counterCheck - Rev;
  }
  
  Serial.print("Your hight is currently: ");
  Serial.print(counterCheck);
  Serial.println(" mm from the ground.");
}
