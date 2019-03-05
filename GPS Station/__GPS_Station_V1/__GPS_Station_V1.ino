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
File GPSFile; // Declare file where GPS data will be written onto.
const int CS = 10; // Chip Select Pin for communication with SD card.

// TPL5110
const int DonePin = 4; // Signal to timer TPL5110
unsigned long startTime;
unsigned long currTime;
float measTime = 2; // in Minutes!
// ###################################################################################################


void setup() {
  // initialize both serial ports:
  Serial.begin(9600); // Start serial port
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  serial.begin(9600); // Start serial port with GPS receiver

  delay(1000);

  // GPS RECEIVER
  // send configuration data in UBX protocol
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    serial.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }

  delay(1000);

  // TEMPERATURE SENSOR
  sensors.begin(); // Start up the library. IC Default 9 bit. If you have troubles consider upping it up to 12. Ups the the delay giving the IC more time to process the temperature measurement
  sensors.requestTemperatures(); // Send the command to get temperatures
  temp = sensors.getTempCByIndex(0); // Read the temperature
  Serial.print("Temperature for Device is: ");
  Serial.println(temp);

  delay(1000);

  // SD CARD
  // Initialize SD Card
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    Serial.println("initialization of SD card failed!");
    return;
  }
  // Write separation line in .txt file
  GPSFile = SD.open("ROV.bin", FILE_WRITE);
  GPSFile.println("-----------------------------------");
  GPSFile.println("");
  GPSFile.flush();

  delay(1000);

  // TPL5110
  pinMode(DonePin, OUTPUT);
  digitalWrite(DonePin, LOW);

  delay(200);

  startTime = millis();
}

void loop() {
  if (serial.available()) {
    int ci = serial.read();
    if (ci == -1) {
      Serial.print("Reading failed!");
      return;
    }
    char c = ci;
    Serial.write(c);
    GPSFile.write(c);
    GPSFile.flush();
  }
  
  currTime = millis();
  if (currTime - startTime >= (measTime * 60 * 1000)) {
    GPSFile.close();
    delay(20);
    digitalWrite(DonePin, HIGH); // toggle DONE so TPL5110 knows to cut power!
  }
}
