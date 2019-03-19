// Include all relevant libraries.

// TEMPERATURE SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>

// SD CARD
#include <SD.h>

// ###################################################################################################
const char UBLOX_INIT_POSLLH[] PROGMEM = {
  // Disable NMEA
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x46, // RXM-RAWX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x38, // RXM-SFRBX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, // NAV-POSLLH off

  // Enable POSLLH
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, // NAV-POSLLH on

  // Rate
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD, // (0.5Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xD9, 0x41, // (0.33Hz)

};

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


const char UBLOX_INIT_RAWX[] PROGMEM = {
  // Disable NMEA
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x46, // RXM-RAWX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x38, // RXM-SFRBX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, // NAV-POSLLH off

  // Enable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x27, 0x4B, // RXM-RAWX on
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x25, 0x3D, // RXM-SFRBX on

  // Rate
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD, // (0.5Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xD9, 0x41, // (0.33Hz)

};

// GPS RECEIVER
const int bufLen = 1300;

// TEMPERATURE SENSOR
#define ONE_WIRE_BUS 5 // Pin 5 on Board. 
float temp; // Variable to store temperature value.
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass oneWire reference to Dallas Temperature.

// VOLTAGE MEASUREMENT
int analogValue; // Analog value that Arduino reads
const float maxVoltageBattery = 4.2; // Maximum voltage on battery when charged
const float maxVoltageArduino = 3; // Maximum voltage Arduino should read
const float lowVolt = -3.35; // change sign from - to + in the final version!
const float vpp = maxVoltageBattery / 1023; // Volts per point <-> accuracy of measurement
const float voltageRatio = maxVoltageBattery / maxVoltageArduino; // Ratio to mathematically upscale voltage
float voltage; // Real voltage

// SD CARD
File bmsFile; // Declare file where BMS data will be written onto.
File gpsFile; // Declare file where GPS data will be written onto.
File root;
const int CS = 10; // Chip Select Pin for communication with SD card.

// TPL5110
const int donePin = 4; // Signal to timer TPL5110
unsigned long startTime;
unsigned long currTime;
unsigned long weekTime;
float measTime = 2; // in Minutes!

// ###################################################################################################

void setup() {
  // Initialize all serial ports:
  Serial.begin(9600); // Start serial port
  Serial1.begin(9600); // Start serial port with GPS receiver
  Serial2.begin(9600); // Start serial port with XBEE module
  delay(5);

  // Initialise TPL5110
  pinMode(donePin, OUTPUT);
  digitalWrite(donePin, LOW);
  delay(5);

  sdInit(); // Initialize SD Card
  delay(500);

  gpsInit(); // send configuration data to get NAV-POSLLH
  delay(500);

  // Send Heartbeats to homebase, i.e. send NAV-POSLLH to homebase via XBEE
  int counter = 0;
  while (1) {
    if (processGPS() ) {
      // Send just fixed position via Xbee
      Serial2.print("iTOW: ");
      Serial2.print(posllh.iTOW);
      Serial2.print(" , lat/lon :");
      Serial2.print(posllh.lat / 10000000., 9);
      Serial2.print(", ");
      Serial2.print(posllh.lon / 10000000., 9);
      Serial2.print(", height: ");
      Serial2.println(posllh.height / 1000., 6);
      counter += 1;
    }
    if (counter == 10) {
      weekTime = posllh.iTOW;
      break;
    }
  }
  delay(500);

  // Battery Management System
  if ( bms() ) {
    for (int i = 0; i < 5; i++) {
      Serial2.println("LOW BATTERY VOLTAGE - PICK ME UP!");
    }
    Serial2.flush();
    delay(5);
    digitalWrite(donePin, HIGH); // switch off whole system
  }
  delay(500);

  gpsConfig(); // send configuration data in UBX protocol to receive RAWX and SFRBX
  delay(500);

  // Open GPS File
  root = SD.open("/");
  openFile(root);
  delay(500);

  startTime = millis();
}

void loop() {
  char buf[bufLen];
  int bufIndex = 0;
  while (Serial1.available()) {
    buf[bufIndex] = (char) Serial1.read(); // Store byte into buffer
    Serial2.write(buf[bufIndex]); // Send byte via xbee to homebase
    bufIndex += 1;
  }

  if (bufIndex != 0) {
    gpsFile.write(buf , bufIndex);
    gpsFile.flush();
  }

  currTime = millis();
  if (currTime - startTime > measTime * 60 * 1000) {
    gpsFile.close();
    delay(20);
    digitalWrite(donePin, HIGH);
  }
}

// ###################################################################################################

void sdInit() {
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    return;
  }
}

void openFile(File dir) {
  int fileCount = -1; // -1 as bms file will also be counted.
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      gpsFile = SD.open("ROV" + String(fileCount) + ".bin", FILE_WRITE);
      break;
    }
    else {
      fileCount += 1;
    }
  }
}

void gpsInit() {
  for (int i = 0; i < sizeof(UBLOX_INIT_POSLLH); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT_POSLLH + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

void gpsConfig() {
  for (int i = 0; i < sizeof(UBLOX_INIT_RAWX); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT_RAWX + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

bool bms() {
  bmsFile = SD.open("bms.txt", FILE_WRITE);

  // TEMPERATURE SENSOR
  sensors.begin(); // Start up the library. IC Default 9 bit. If you have troubles consider upping it up to 12. Ups the the delay giving the IC more time to process the temperature measurement
  sensors.requestTemperatures(); // Send the command to get temperatures
  temp = sensors.getTempCByIndex(0); // Read the temperature
  delay(5);

  // VOLTAGE
  analogValue = analogRead(A0);
  voltage = (analogValue * vpp) * voltageRatio;
  delay(5);

  // STORE DATA
  bmsFile.print("iTOW: ");
  bmsFile.print(weekTime);
  bmsFile.print(", Temperature: ");
  bmsFile.print(temp);
  bmsFile.print("°C, Voltage: ");
  bmsFile.print(voltage);
  bmsFile.println("V");
  bmsFile.close();
  delay(5);

  if (voltage < lowVolt) {
    return true;
  }
  else {
    return false;
  }
}

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

  while ( Serial1.available() ) {
    byte c = Serial1.read();
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
