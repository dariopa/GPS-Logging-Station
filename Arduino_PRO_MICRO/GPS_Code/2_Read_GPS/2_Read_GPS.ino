#import <SoftwareSerial.h>;
#include "SparkFun_Ublox_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS

SFE_UBLOX_GPS myGPS;

const byte rxPin = 8;
const byte txPin = 9;
SoftwareSerial mySerial(rxPin, txPin);


void setup() {
  // initialize both serial ports:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Ublox GPS I2C Test");

  mySerial.begin(9600);

  myGPS.begin(Wire);
  if (myGPS.isConnected() == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
}

void loop() {
  // read from port 1, send to port 0:
  if (mySerial.available()) {
    int Coord = mySerial.read();
    Serial.write(Coord);
  }
}
