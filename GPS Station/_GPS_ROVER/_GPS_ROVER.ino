
#include "DropRecovery.h"

SettingGPS gps;
SettingSD sd;
SettingBMS bms;
SettingTPL tpl;

float measurment_time = 3; // Declare measurment time in minutes
bool bms_switch = true; // If bms is connected, then true. If not connected, then false.

void setup() {
  // Initialize all serial ports:
  Serial.begin(9600); // Start serial port for debugging
  Serial1.begin(9600); // Start serial port with GPS receiver
  Serial2.begin(9600); // Start serial port with XBEE module

  tpl.TPLInit(); // Initialize TPL5110
  sd.SdInit(); // Initialize SD Card
  gps.GpsInit(); // send configuration for GPS initialisation
  gps.RawxConfig(); // send configuration data in UBX protocol to receive RAWX and SFRBX

  // BMS 
  float temp = bms.Temperature();
  float volt = bms.Voltage();
  sd.WriteBmsLog(temp, volt);
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

  // Open GPS File
  sd.root = SD.open("/");
  sd.OpenFile(sd.root);

  // Start measuring the time
  tpl.start_time = millis();
}

void loop() {
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
  tpl.current_time = millis(); // measure current time
  if (tpl.TPLMeasureTime(tpl.current_time, tpl.start_time, measurment_time)) {
    tpl.TPLToggle();
  }
}
