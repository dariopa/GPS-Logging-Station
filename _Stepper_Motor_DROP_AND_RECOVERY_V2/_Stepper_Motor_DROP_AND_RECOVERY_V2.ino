// Arduino stepper motor control code

#include <Stepper.h> // Include the header file for the Stepper motor

// Define Variables
// ########################################
const int dirPin = 2; // Direction
const int stepPin = 3; // Step
// Motor steps per rotation
const int STEPS_PER_REV = 200;

// Pitch of Screw in mm/rev: M6 = 1 mm/rev, M5 = 0.8 mm/rev, M4 = 0.7 mm/rev
float screwPitch = 1.0;

// Height of elevation / descend, in mm
float heightCarrier;

// Revolutions required, in rev
float Rev;

// Maximum range of motion
const float motionRange = 18; // mm

// Variable checks if range boundaries are exceeded.
float counterCheck = 0;

// Velocity
const int Vel = 1900;
// ########################################

void setup() {
  // Setup Serial Communication and Pins.
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // ######################################
  // HERE WILL COME THE CALIBRATION CODE FOR THE STEPPER WHEN SWITCHED ON!
  // ######################################
}

void loop() {
  Serial.println("Please enter your command: ");

  // Wait here until an Input arrives.
  while (Serial.available() == 0) {
  }

  // Read and print height. Value is in mm.
  heightCarrier = Serial.parseFloat();

  // Calculate the number of revolutions required for your stepper motor.
  Rev = heightCarrier / screwPitch;
  Serial.print(Rev);
  Serial.print(" revolutions required for ");
  Serial.print(heightCarrier);
  Serial.println( "mm.");

  // This block checks whether you're passing the range boundaries (crush the stepper) or if you're still in available range.
  counterCheck = counterCheck + heightCarrier;
  if (counterCheck >= 0 && counterCheck <= motionRange) {

    // Defines in which direction you want the stepper to go.
    if (Rev < 0) {
      digitalWrite(dirPin, LOW); // Descent
    }
    else if (Rev >= 0) {
      digitalWrite(dirPin, HIGH); // Lift
    }

    // Steps are done in this for-loop.
    for (int x = 0; x < abs(Rev) * STEPS_PER_REV; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000000. / Vel);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000000. / Vel);
    }
  }

  // if you exceeded the range, you will get this warning.
  else {
    Serial.println("Didn't move because you exceed range of motion! Change your command: ");
    counterCheck = counterCheck - heightCarrier;
  }

  // When steps are done, you are told on which position the stepper is.
  Serial.print("Your hight is currently: ");
  Serial.print(counterCheck);
  Serial.println(" mm from the ground.");
}
