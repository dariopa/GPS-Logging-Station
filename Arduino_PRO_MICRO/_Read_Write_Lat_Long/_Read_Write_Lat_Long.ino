/*
  Read NMEA sentences over I2C using Ublox module SAM-M8Q, NEO-M8P, etc
  By: Nathan Seidle
  SparkFun Electronics
  Date: August 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads the NMEA characters over I2C and pipes them to MicroNMEA
  This example will output your current long/lat and satellites in view

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14980

  For more MicroNMEA info see https://github.com/stevemarple/MicroNMEA

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  Go outside! Wait ~25 seconds and you should see your lat/long
*/

#include <Wire.h> //Needed for I2C to GPS

#include <SD.h>
File LatLongFile;
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
  LatLongFile = SD.open("LatLong.txt", FILE_WRITE);

  if (LatLongFile) {
    Serial.println("Writing to LatLong.txt...");
    
    if (nmea.isValid() == true)
    {
      long latitude_mdeg = nmea.getLatitude();
      long longitude_mdeg = nmea.getLongitude();

      Serial.println("");
      Serial.print("Latitude (deg): ");
      Serial.println(latitude_mdeg / 1000000., 6);
      Serial.print("Longitude (deg): ");
      Serial.println(longitude_mdeg / 1000000., 6);

      LatLongFile.print("Latitude (deg): ");
      LatLongFile.println(latitude_mdeg / 1000000., 6);
      LatLongFile.print("Longitude (deg): ");
      LatLongFile.println(longitude_mdeg / 1000000., 6);
    }
    else
    {
      Serial.print("No Fix - ");
      Serial.print("Num. satellites: ");
      Serial.println(nmea.getNumSatellites());
    }

    // close the file:
    LatLongFile.close();

  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening LatLongFile.txt");
  }

  delay(1000); //Don't pound too hard on the I2C bus
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

}
