
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

  delay(10000); // delay so that the GPS receiver can converge
}

void loop()
{
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.
  GPSFile = SD.open("GPS.txt", FILE_WRITE);

  if (GPSFile) {
    if (nmea.isValid() == true)
    {
      Serial.println("Printing Position...");

      uint8_t num_satellites = nmea.getNumSatellites();
      GPSFile.print("Number of Satelites: ");
      GPSFile.println(num_satellites);

      uint8_t HDOP = nmea.getHDOP();
      GPSFile.print("Horizontal Dilution: ");
      GPSFile.println(HDOP / 10., 2);

      long latitude_mdeg = nmea.getLatitude();
      GPSFile.print("Latitude (deg): ");
      GPSFile.println(latitude_mdeg / 1000000., 6);

      long longitude_mdeg = nmea.getLongitude();
      GPSFile.print("Longitude (deg): ");
      GPSFile.println(longitude_mdeg / 1000000., 6);

      long alt;
      if (nmea.getAltitude(alt))
      {
        GPSFile.print("Altitude (m): ");
        GPSFile.println(alt / 1000., 3);
      }
      else
      {
        GPSFile.println("Not available.");
      }

      uint8_t day = nmea.getDay();
      uint8_t month = nmea.getMonth();
      uint16_t year = nmea.getYear();
      GPSFile.print("Date: ");
      GPSFile.print(day);
      GPSFile.print(".");
      GPSFile.print(month);
      GPSFile.print(".");
      GPSFile.println(year);

      uint8_t hour = nmea.getHour();
      uint8_t minute = nmea.getMinute();
      uint8_t second = nmea.getSecond();
      uint16_t hundredths = nmea.getHundredths();
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

  // Switch off LED // will be used with TPL5110 later on!
  while (1 == 2)
  {
    digitalWrite(DonePin, HIGH); // toggle DONE so TPL knows to cut power!
    delay(1);
    digitalWrite(DonePin, LOW); // toggle DONE so TPL can return in old state! --> Won't be needed in final application!
  }
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
