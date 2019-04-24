// Include all relevant libraries.

// TEMPERATURE SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>

// SD CARD
#include <SD.h>

// Declare measurment time
float measurment_time = 2; // in Minutes!
bool bms_state = false; // if bms connected, then true. Otherwise, false.

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
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, // NAV-POSLLH off

  // Rate
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD, // (0.5Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xD9, 0x41, // (0.33Hz)
};

const char UBLOX_INIT_RAWX[] PROGMEM = {
  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x46, // RXM-RAWX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x38, // RXM-SFRBX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, // NAV-POSLLH off

  // Enable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x27, 0x4B, // RXM-RAWX on
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x25, 0x3D, // RXM-SFRBX on
};

const char UBLOX_INIT_POSLLH[] PROGMEM = {
  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x46, // RXM-RAWX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x38, // RXM-SFRBX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, // NAV-POSLLH off

  // Enable POSLLH
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, // NAV-POSLLH on
};

const unsigned char UBX_HEADER_POSLLH[] = { 0xB5, 0x62 };

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

// GPS RECEIVER
const int buffer_length = 2000;

// TEMPERATURE SENSOR
float temperature; // Variable to store temperature value.
OneWire oneWire(5); // Setup a oneWire instance to communicate with any OneWire devices on pin 5 on Arduino Board (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass oneWire reference to Dallas Temperature.

// VOLTAGE MEASUREMENT
int analog_value; // Analog value that Arduino reads
const float kMaxVoltageBattery = 4.2; // Maximum voltage on battery when charged
const float kMaxVoltageArduino = 3.3; // Maximum voltage Arduino should read
const float kLowVoltage = 3.3; // Lowest Voltage where TPL5110 has to be toggled
const float kVpp = kMaxVoltageBattery / 1023; // Volts per point <-> accuracy of measurement
const float kVoltageRatio = kMaxVoltageBattery / kMaxVoltageArduino; // Ratio to mathematically upscale voltage
float real_voltage; // Real voltage

// SD CARD
const int kChipSelect = 10; // Chip Select Pin for communication with SD card.
File bmsFile; // Declare file where BMS data will be written onto.
File gpsFile; // Declare file where GPS data will be written onto.
File root;

// TPL5110
const int kDonePin = 4; // Signal to timer TPL5110
unsigned long start_time;
unsigned long current_time;
unsigned long week_time;

// ###################################################################################################

void setup() {
  // Initialize all serial ports:
  Serial.begin(9600); // Start serial port
  Serial1.begin(9600); // Start serial port with GPS receiver
  Serial2.begin(9600); // Start serial port with XBEE module
  pinMode(LED_BUILTIN, OUTPUT);
  delay(20);

  // Initialise TPL5110
  pinMode(kDonePin, OUTPUT);
  digitalWrite(kDonePin, LOW);
  delay(20);

  SdInit(); // Initialize SD Card
  delay(20);

  GpsInit(); // send configuration for GPS initialisation
  delay(20);

  PosllhConfig(); // send configuration data to get NAV-POSLLH
  delay(8000); // wait 8 seconds until position lock on GPS receiver

  // Store weektime for BMS
  int counter = 0;
  while (true) {
    if (ProcessGPS() ) {
      counter += 1;
      delay(20);
    }
    if (counter == 5) {
      week_time = posllh.iTOW;
      break;
    }
    // If for any reason ProcessGPS gets stuck here, it will switch off after measurment_time
    current_time = millis(); // Track time
    if (current_time - start_time > measurment_time * 60 * 1000) { // if measurment_time is exceeded, toggle TPL5110
      Serial2.println("stuck in processGPS");
      delay(20);
      digitalWrite(kDonePin, HIGH); // toggle TPL5110
    }
  }
  delay(20);

  // Battery Management System
  if ( Bms() ) {
    Serial2.println("LOW BATTERY VOLTAGE - PICK ME UP");
    delay(20);
    digitalWrite(kDonePin, HIGH); // switch off whole system
  }
  delay(20);

  RawxConfig(); // send configuration data in UBX protocol to receive RAWX and SFRBX
  delay(20);

  // Open GPS File
  root = SD.open("/");
  OpenFile(root);
  delay(500);

  start_time = millis();
}

void loop() {
  char rawx_buffer[buffer_length]; // declare a buffer
  int buffer_index = 0; // declare buffer index
  while (Serial1.available()) { // while GPS receiver transmits bytes, write them into buffer
    rawx_buffer[buffer_index] = (char) Serial1.read(); // Store byte into buffer
    Serial2.write(rawx_buffer[buffer_index]); // Send byte via xbee to homebase
    buffer_index++;
    if (buffer_index > buffer_length) { // if more data available on Serial1 than buffer_length, delete message
      buffer_index = 0;
      break;
    }
  }

  if (buffer_index != 0) {
    gpsFile.write(rawx_buffer , buffer_index); // when GPS receiver is done transmitting data, store it on microSD
    gpsFile.flush();
  }

  current_time = millis(); // measure current time
  if (current_time - start_time >  measurment_time * 60 * 1000) { // if measurment_time has been exceeded, toggle TPL5110
    gpsFile.close();
    delay(20);
    digitalWrite(kDonePin, HIGH); // toggle TPL5110
  }
}

// ###################################################################################################

void SdInit() {
  pinMode(kChipSelect, OUTPUT);
  if (!SD.begin(kChipSelect)) {
    Serial.println("initialisation failed");
    return;
  }
  Serial.println("initialisation succeeded");
}

void OpenFile(File dir) {
  int file_count = -1; // -1 as bms-file will also be counted.
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      gpsFile = SD.open("ROV" + String(file_count) + ".bin", FILE_WRITE);
      break;
    }
    else {
      file_count += 1;
    }
  }
}

void GpsInit() {
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

void PosllhConfig() {
  for (int i = 0; i < sizeof(UBLOX_INIT_POSLLH); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT_POSLLH + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

void RawxConfig() {
  for (int i = 0; i < sizeof(UBLOX_INIT_RAWX); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT_RAWX + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

bool Bms() {
  bmsFile = SD.open("bms.txt", FILE_WRITE);

  // temperatureERATURE SENSOR
  sensors.begin(); // Start up the library. IC Default 9 bit.
  sensors.requestTemperatures(); // Send the command to get temperatures
  temperature = sensors.getTempCByIndex(0); // Read the temperature
  delay(5);

  // VOLTAGE
  analog_value = analogRead(A0);
  real_voltage = (analog_value * kVpp) * kVoltageRatio;
  delay(5);

  // STORE DATA
  bmsFile.print("iTOW: ");
  bmsFile.print(week_time);
  bmsFile.print(", Temperature: ");
  bmsFile.print(temperature);
  bmsFile.print("°C, Voltage: ");
  bmsFile.print(real_voltage);
  bmsFile.println("V");
  bmsFile.close();
  delay(5);

  if (bms_state == false) { // if bms is not connected, i.e. bms_state = false, then return false
    return false;
  }

  if (real_voltage < kLowVoltage) { // if voltage is below voltage tolerance, then return true to toggle TPL5110
    return true;
  }
  else { // if voltage is above voltage tolerance, program can continue
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

bool ProcessGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( Serial1.available() ) {
    byte c = Serial1.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER_POSLLH[fpos] )
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

void test() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
}
