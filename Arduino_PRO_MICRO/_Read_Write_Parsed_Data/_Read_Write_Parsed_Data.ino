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
File GPSFile;
const int ChipSelect = 10; // For communication with SD card

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

const int DonePin = 4; // Signal to timer TPL5110

long int alt = 400; // Trial for altitude

void setup()
{
  Serial.begin(115200);
  // ##################  INITIALIZE TIMERS  #####################
  pinMode(DonePin, OUTPUT);
  digitalWrite(DonePin, LOW);

  // ################## INITIALIZE SD CARD ######################
  Serial.print("Initializing SD card...");
  pinMode(ChipSelect, OUTPUT);
  if (!SD.begin(ChipSelect)) {
    Serial.println("initialization failed!");
    return;
  }

  // Write separation line in .txt file
  GPSFile = SD.open("GPS.txt", FILE_WRITE);
  GPSFile.println("--------------------");
  GPSFile.println("");
  GPSFile.close();  

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
  GPSFile = SD.open("GPS.txt", FILE_WRITE);

  if (GPSFile) {
    if (nmea.isValid() == true)
    {
      uint8_t num_satellites = nmea.getNumSatellites();
      uint8_t HDOP = nmea.getHDOP();
      long latitude_mdeg = nmea.getLatitude();
      long longitude_mdeg = nmea.getLongitude();
      // boolean altitude_mm = nmea.getAltitude(alt);
      uint8_t day = nmea.getDay();
      uint8_t month = nmea.getMonth();
      uint16_t year = nmea.getYear();   
      uint8_t hour = nmea.getHour();
      uint8_t minute = nmea.getMinute();
      uint8_t second = nmea.getSecond();
      uint16_t hundredths = nmea.getHundredths();

      Serial.println("Printing Position...");

      GPSFile.print("Number of Satelites: ");
      GPSFile.println(num_satellites);

      GPSFile.print("Horizontal Dilution: ");
      GPSFile.println(HDOP);

      GPSFile.print("Latitude (deg): ");
      GPSFile.println(latitude_mdeg / 1000000., 6);

      GPSFile.print("Longitude (deg): ");
      GPSFile.println(longitude_mdeg / 1000000., 6);

      // GPSFile.print("Altitude (mm): ");
      // GPSFile.println(altitude_mm);

      GPSFile.print("Date: ");
      GPSFile.print(day);
      GPSFile.print(".");
      GPSFile.print(month);
      GPSFile.print(".");
      GPSFile.println(year);

      GPSFile.print("Time: ");
      GPSFile.print(hour);
      GPSFile.print(":");
      GPSFile.print(minute);
      GPSFile.print(":");
      GPSFile.print(second);
      GPSFile.print(":");
      GPSFile.println(hundredths);

      GPSFile.println("");
      
      Serial.println("Done!");
    }
    else
    {
      Serial.print("No Fix - Num. satellites: ");
      Serial.println(nmea.getNumSatellites());
    }

    // close the file:
    GPSFile.close();

  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening GPSFile.txt");
  }

  delay(1000); //Don't pound too hard on the I2C bus


  digitalWrite(DonePin, HIGH); // toggle DONE so TPL knows to cut power!
  delay(1);
  digitalWrite(DonePin, LOW); // toggle DONE so TPL can return in old state!
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
