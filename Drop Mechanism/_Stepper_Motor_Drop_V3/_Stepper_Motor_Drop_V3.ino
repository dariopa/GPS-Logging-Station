// ========================================
// Dynamixel XL-320 Arduino library example
// ========================================

// Read more:
// https://github.com/hackerspace-adelaide/XL320

#include "XL320.h"

// Name your robot!
XL320 robot;

// If you want to use Software Serial, uncomment this line
#include <SoftwareSerial.h>

// Set the SoftwareSerial RX & TX pins
SoftwareSerial mySerial(8, 9); // (RX, TX)

// Set some variables for incrementing position & LED colour
char rgb[] = "rgbypcwo";
int servoPosition = 0;
int ledColour = 0;

// Set the default servoID to talk to
int servoID = 1;

void setup() {
  mySerial.begin(1000000);
  robot.begin(mySerial);
  delay(100);

  // writePacket(1, XL_BAUD_RATE, x) sets the baud rate:
  // 0: 9600, 1:57600, 2:115200, 3:1Mbps
  robot.sendPacket(servoID, XL_BAUD_RATE, 2);
  delay(100);

  // Setup Software Serial
  mySerial.begin(115200);

  // Initialise your robot
  robot.begin(mySerial); // Hand in the serial object you're using

  // I like fast moving servos, so set the joint speed to max!
  robot.setJointSpeed(servoID, 1023);

}

void loop() {

  // Set a delay to account for the receive delay period
  delay(100);


  // Servo test.. select a random servoID and colour
  robot.moveJoint(servoID, random(0, 1023));

  // Servo test.. increment the servo position by 100 each loop
  //  robot.moveJoint(servoID, servoPosition);
  servoPosition = (servoPosition + 100) % 1023;

  // Set a delay to account for the receive delay period
  delay(100);
}
