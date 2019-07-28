/*
   Box 1: Kanal 2607
   Box 2: Kanal 2608
   Box 3: Kanal 2609
   Box 4: Kanal 2610
   Box 5: Kanal 2611
*/

#include "DropRecovery.h"
SettingGPS gps;
SettingSD sd;
SettingBMS bms;
SettingTPL tpl;

const unsigned char UBX_HEADER[] = { 0xB5, 0x62};

struct UBX_RAWX {
  uint64_t rcvTow; // 8 bytes
  uint16_t week; // 2 bytes
};

UBX_RAWX rawx;

void parseGPS() {
  int byte_position = 0;
  const int header_size = sizeof(UBX_HEADER);
  const int payload_size = sizeof(UBX_RAWX);

  while ( Serial1.available() ) {
    byte c = Serial1.read();
    if ( byte_position < header_size ) {
      if ( c == UBX_HEADER[byte_position] ) {
        byte_position++;
      }
      else {
        byte_position = 0;
      }
    }
    else if ( (byte_position - header_size) < payload_size ) {
      ((unsigned char*)(&rawx))[byte_position - header_size] = c;
      byte_position++;
    }
    else if ( (byte_position - header_size) >= payload_size ) {
      byte_position = 0;
    }
  }
  delay(20000);
  // Serial.write(rawx.rcvTow);
  Serial.print(rawx.week);
}

float measurment_time = 3; // Declare measurment time in minutes
int cts = 30; // clear to send for XBee
int led_green = 2;
int led_red = 3;

void setup() {
  // Initialize all serial ports:
  Serial.begin(115200); // Start serial port for debugging
  Serial1.begin(38400); // Start serial port with GPS receiver
  Serial2.begin(115200); // Start serial port with XBEE module

  pinMode(led_green, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(cts, INPUT);

  tpl.TPLInit(); // Initialize TPL5110
  sd.SdInit(led_green, led_red); // Initialize SD Card
  gps.GpsInit(); // send configuration for GPS initialisation
  gps.RawxConfig(); // send configuration data in UBX protocol to receive RAWX and SFRBX

  // BMS
  float temp = bms.Temperature();
  float volt = bms.Voltage();
  sd.WriteBmsLog(temp, volt);
  if (temp < 0.0 or volt < bms.LowVoltage) {
    Serial2.print("Battery or temperature low!");
    delay(20);
    tpl.TPLToggle();
  }

  // Open GPS File
  sd.root = SD.open("/");
  sd.OpenFile(sd.root);

  parseGPS();

  // Start measuring the time
  tpl.start_time = millis();
}

void loop() {
  char rawx_buffer[gps.buffer_length]; // declare a buffer
  int buffer_index = 0; // declare buffer index
  int sent_index = 0; // declare sent index for telemetry
  while (Serial1.available()) { // while GPS receiver transmits bytes, write them into buffer
    int avail = Serial1.available();
    char* rawx_buffer_ptr = rawx_buffer + buffer_index;
    Serial1.readBytes(rawx_buffer_ptr, avail);
    buffer_index += avail;
    if (buffer_index > gps.buffer_length) { // if more data available on Serial1 than gps.buffer_length, delete message
      Serial.print("index");
      buffer_index = 0;
      break;
    }
  }

  if (buffer_index != 0) {
    // when GPS receiver is done transmitting data, store it on microSD
    sd.gpsFile.write(rawx_buffer , buffer_index);
    sd.gpsFile.flush();

    // Send data via telemetry

    int cts_counter = 0;
    while (sent_index < buffer_index && cts_counter < 150) {
      if ((int) digitalRead(cts) == 0) {
        Serial2.write(rawx_buffer[sent_index]);
        sent_index++;
      }
      else {
        delay(3);
        cts_counter++;
      }
    }
  }

  tpl.current_time = millis(); // measure current time
  if (tpl.TPLMeasureTime(tpl.current_time, tpl.start_time, measurment_time)) {
    tpl.TPLToggle();
  }
}
