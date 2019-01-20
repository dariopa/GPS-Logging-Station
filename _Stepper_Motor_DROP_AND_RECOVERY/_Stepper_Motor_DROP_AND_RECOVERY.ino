// Arduino stepper motor control code

#include <Stepper.h> // Include the header file

// ###################################
// change this to the number of steps on your motor
float turns = 4; // required revolutions for the package to fall down
// ###################################

const int revolution = 2048;  // steps per revolution
int vel = 18; // speed of motor
int steps = turns * revolution; // steps motor will have to do
int val; // variable that is entered via Serial Port. 1 for elevating, 2 for descending. 

// create an instance of the stepper class using the steps and pins
Stepper stepper(revolution, 8, 10, 9, 11); // Defines steps and pins

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(vel);
}

void loop() {
  if (Serial.available())
  {
    val = Serial.parseInt();
    Serial.println(val);

    if (val == 1)
    {
      stepper.step(steps);
    }

    else if (val == 2)
    {
      stepper.step(-steps);
    }
  }

}
