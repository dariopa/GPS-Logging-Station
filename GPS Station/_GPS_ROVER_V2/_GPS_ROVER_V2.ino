#include "DropRecovery.h"

SettingGPS gps;
SettingSD sd;
SettingBMS bms;
SettingTPL tpl;

float measurment_time = 2; // Declare measurment time in minutes
bool bms_switch = false; // If bms is connected, then true. If not connected, then false.

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

  // Rate
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD, // (0.5Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xD9, 0x41, // (0.33Hz)
};

const char UBLOX_INIT_RAWX[] PROGMEM = {
  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x46, // RXM-RAWX off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x38, // RXM-SFRBX off

  // Enable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x27, 0x4B, // RXM-RAWX on
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x25, 0x3D, // RXM-SFRBX on
};

void setup() {
  // Initialize all serial ports:
  Serial1.begin(9600); // Start serial port with GPS receiver
  Serial2.begin(9600); // Start serial port with XBEE module

  tpl.TPLInit(); // Initialize TPL5110
  sd.SdInit(); // Initialize SD Card
  GpsInit(); // send configuration for GPS initialisation
  RawxConfig(); // send configuration data in UBX protocol to receive RAWX and SFRBX

  //###### BMS #######
  float temp = bms.Temperature();
  float volt = bms.Voltage();
  sd.bmsFile = SD.open("BMS.txt", FILE_WRITE);
  sd.bmsFile.print(" Temperature ");  sd.bmsFile.print(temp);
  sd.bmsFile.print("°C, Voltage "); sd.bmsFile.print(volt); sd.bmsFile.println("V");
  sd.bmsFile.close();
  if (bms_switch) {
    if (temp < 0.0 or volt < bms.LowVoltage) {
      Serial2.print("Battery or temperature low!");
      delay(20);
      tpl.TPLToggle();
    }
  }
  else {
    if (temp < 0.0) {
      Serial2.print("Temperature below 0 degree");
      delay(20);
      tpl.TPLToggle();
    }
  }
  //###### BMS #######

  // Open GPS File
  sd.root = SD.open("/");
  sd.OpenFile(sd.root);

  // Start measuring the time
  tpl.start_time = millis();
}

void loop() {
  ReadWriteRAWX();
  tpl.current_time = millis(); // measure current time
  if (tpl.TPLMeasureTime(tpl.current_time, tpl.start_time, measurment_time)) {
    tpl.TPLToggle();
  }
}


void GpsInit() {
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  delay(8000); // wait 8 seconds until position lock on GPS receiver
}

void RawxConfig() {
  for (int i = 0; i < sizeof(UBLOX_INIT_RAWX); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT_RAWX + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  delay(20);
}

void ReadWriteRAWX() {
  char rawx_buffer[gps.buffer_length]; // declare a buffer
  int buffer_index = 0; // declare buffer index
  while (Serial1.available()) { // while GPS receiver transmits bytes, write them into buffer
    rawx_buffer[buffer_index] = (char) Serial1.read(); // Store byte into buffer
    Serial2.write(rawx_buffer[buffer_index]); // Send byte via xbee to homebase
    buffer_index++;
    if (buffer_index > gps.buffer_length) { // if more data available on Serial1 than gps.buffer_length, delete message
      buffer_index = 0;
      break;
    }
  }

  if (buffer_index != 0) {
    sd.gpsFile.write(rawx_buffer , buffer_index); // when GPS receiver is done transmitting data, store it on microSD
    sd.gpsFile.flush();
  }
}
