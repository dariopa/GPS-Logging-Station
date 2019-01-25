// Arduino stepper motor control code

#include <Stepper.h> // Include the header file

// Define Variables
// ########################################
const int dirPin = 2; // Direction
const int stepPin = 3; // Step

// Motor steps per rotation
const int STEPS_PER_REV = 200;

// Revolutions required
const int Rev = 15;

// Velocity
const int Vel = 1900;
// ########################################

// Variable that is entered via Serial Port. 1 for elevating, 2 for descending.
int val;

// Variable that checks which was the last entry.
int counter_check;

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {

  if (Serial.available())
  {
    val = Serial.parseInt();
    Serial.println(val);

    if (val == 1 && counter_check != 1) {
      // Set motor direction clockwise
      digitalWrite(dirPin, HIGH);

      // Spin motor one rotation slowly
      for (int x = 0; x < Rev * STEPS_PER_REV; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000000. / Vel);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000000. / Vel);
      }
      counter_check = val;
    }
    else if (val == 2 && counter_check != 2) {
      // Set motor direction clockwise
      digitalWrite(dirPin, LOW);

      // Spin motor one rotation slowly
      for (int x = 0; x < Rev * STEPS_PER_REV; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000000. / Vel);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000000. / Vel);
      }
      counter_check = val;      
    }
  }
}
