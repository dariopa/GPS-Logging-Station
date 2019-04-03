// Include all relevant libraries.
#include <SD.h>
#include <SoftwareSerial.h>

const byte rxPinGPS = 8;
const byte txPinGPS = 9;

SoftwareSerial serialGPS = SoftwareSerial(rxPinGPS, txPinGPS);

// ###################################################################################################
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

  // Enable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x27, 0x4B, // RXM-RAWX on
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x25, 0x3D, // RXM-SFRBX on

  // Rate
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD, // (0.5Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xD9, 0x41, // (0.33Hz)

};

// GPS RECEIVER
const int bufLen = 800;

// SD CARD
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
  delay(1000);
  // Initialize all serial ports:
  Serial.begin(9600); // Start serial port
  pinMode(rxPinGPS, INPUT);
  pinMode(txPinGPS, OUTPUT);
  serialGPS.begin(9600); // Start serial port with GPS receiver
  delay(500);

  // Initialise TPL5110
  pinMode(donePin, OUTPUT);
  digitalWrite(donePin, LOW);
  delay(5);

  sdInit(); // Initialize SD Card
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
  while (serialGPS.available()) {
    buf[bufIndex] = (char) serialGPS.read(); // Store byte into buffer
    Serial.write(buf[bufIndex]); // Send byte via xbee to homebase
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

void gpsConfig() {
  for (int i = 0; i < sizeof(UBLOX_INIT_RAWX); i++) {
    serialGPS.write( pgm_read_byte(UBLOX_INIT_RAWX + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}
