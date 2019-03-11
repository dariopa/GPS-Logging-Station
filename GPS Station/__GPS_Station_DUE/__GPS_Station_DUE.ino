// Include all relevant libraries.

// TEMPERATURE SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>

// SD CARD
#include <SD.h>

// ###################################################################################################

const char UBLOX_INIT[] PROGMEM = {
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
const int CS = 10; // Chip Select Pin for communication with SD card.

// TPL5110
const int donePin = 4; // Signal to timer TPL5110
unsigned long startTime;
unsigned long currTime;
float measTime = 2; // in Minutes!

// ###################################################################################################
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

  // Battery Management System
  if ( bms() ) {
    for (int i = 0; i < 50; i++) {
      Serial2.println("COME PICK ME UP!");
    }
    Serial2.flush();
    delay(5);
    digitalWrite(donePin, HIGH); // switch off whole system
  }
  delay(500);

  // send configuration data in UBX protocol
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  delay(500);

  // Initialize SD Card
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    return;
  }
  delay(500);

  // Open GPS File
  gpsFile = SD.open("ROV.bin", FILE_WRITE);
  delay(500);

  startTime = millis();
}

void loop() {
  char buf[bufLen];
  int bufIndex = 0;
  while (Serial1.available()) {
    int ci = Serial1.read();
    char c = ci;
    buf[bufIndex] = c;
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
// ###################################################################################################

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
  bmsFile.print("Temperature: ");
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
