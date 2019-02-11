// Include all relevant libraries.

// GPS RECEIVER
#include <SoftwareSerial.h>

// TEMPERATURE SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>

// SD CARD
#include <SD.h>

// Define all your variables.
// ###################################################################################################
// GPS RECEIVER
const byte rxPin = 8; // Declare receiver-pin for serial communication
const byte txPin = 9; // Declare transmitter-pin for serial communication
SoftwareSerial serial = SoftwareSerial(rxPin, txPin);

const char UBLOX_INIT[] PROGMEM = {
  // Revert to default configuration
  0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A,

  // Disable NMEA
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

  // Disable POSLLH
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off

  // Enable POSLLH
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, //NAV-POSLLH on

  // Rate
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD, // (0.5Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xD9, 0x41, // (0.33Hz)

};

// TEMPERATURE SENSOR
#define ONE_WIRE_BUS 5 // Pin 5 on Board. 
float temp; // Variable to store temperature value.
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass oneWire reference to Dallas Temperature.

// SD CARD
File GPSFile;
const int CS = 10; // Chip Select Pin for communication with SD card.

// TPL5110
const int DonePin = 4; // Signal to timer TPL5110
int counter = 0;
int measPoints = 20; // Define how many measurment points you want to collect. 
// ###################################################################################################

// Declare structure for NAV-POSLLH message
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

NAV_POSLLH posllh;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( serial.available() ) {
    byte c = serial.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos - 2) < payloadSize )
        ((unsigned char*)(&posllh))[fpos - 2] = c;

      fpos++;

      if ( fpos == (payloadSize + 2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize + 3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize + 4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize + 4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

void setup() {
  Serial.begin(9600); // Start serial port

  // GPS RECEIVER
  serial.begin(9600); // Start serial port with GPS receiver
  // initialize both serial ports:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // send configuration data in UBX protocol
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    serial.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }

  // TEMPERATURE SENSOR
  sensors.begin(); // Start up the library. IC Default 9 bit. If you have troubles consider upping it up to 12. Ups the the delay giving the IC more time to process the temperature measurement
  sensors.requestTemperatures(); // Send the command to get temperatures
  temp = sensors.getTempCByIndex(0);
  Serial.print("Temperature for Device is: ");
  Serial.println(temp);

  // SD CARD
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    Serial.println("initialization of SD card failed!");
    return;
  }
  // Write separation line in .txt file
  GPSFile = SD.open("GPS.txt", FILE_WRITE);
  GPSFile.println("-----------------------------------");
  GPSFile.println("");
  GPSFile.close();

  // TPL5110
  pinMode(DonePin,OUTPUT);
  digitalWrite(DonePin, LOW);
}


void loop() {
  if ( processGPS() ) {
    Serial.print("I'm here: ");
    Serial.print("iTOW: ");
    Serial.print(posllh.iTOW);
    Serial.print(" lat/lon: "); 
    Serial.print(posllh.lat / 10000000.0f);
    Serial.print(","); 
    Serial.print(posllh.lon / 10000000.0f);
    Serial.print(" height: ");  
    Serial.print(posllh.height / 1000.0f);
    Serial.println();

    GPSFile = SD.open("GPS.txt", FILE_WRITE);
    GPSFile.print("iTOW: ");
    GPSFile.println(posllh.iTOW);
    GPSFile.print("lat/lon: ");
    GPSFile.print(posllh.lat / 10000000.0f); 
    GPSFile.print(", ");
    GPSFile.println(posllh.lon / 10000000.0f);
    GPSFile.print("height: ");
    GPSFile.println(posllh.height / 1000.0f);
    GPSFile.close();

    counter = counter + 1;
    if (counter == measPoints) {
      digitalWrite(DonePin, HIGH); // toggle DONE so TPL knows to cut power!
    }
  }
}
