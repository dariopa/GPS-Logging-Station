#include <Wire.h> //Needed for I2C to GPS

#include <SD.h>
File NMEAFile;
const int ChipSelect = 10; // For communication with SD card

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

const int DonePin = 4; // Signal to timer TPL5110

void setup()
{
  Serial.begin(115200);
  // ##################  INITIALIZE TIMERS  #####################
  pinMode(DonePin, OUTPUT);
  digitalWrite(DonePin, LOW);

  // ################## INITIALIZE SD CARD ######################
  Serial.print("Initializing SD card...");
  pinMode(ChipSelect, OUTPUT);

  Serial.println("Ublox GPS I2C Test");
  if (!SD.begin(ChipSelect)) {
    Serial.println("initialization failed!");
    return;
  }

  // ################## INITIALIZE GPS ######################
  myGPS.begin(Wire);
  if (myGPS.isConnected() == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
}

void loop()
{
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  delay(250); //Don't pound too hard on the I2C bus
  digitalWrite(DonePin, HIGH); // toggle DONE so TPL knows to cut power!
}


//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
  NMEAFile = SD.open("NMEA.txt", FILE_WRITE);

  Serial.print(incoming);
  NMEAFile.write(incoming);

  NMEAFile.close();

}
