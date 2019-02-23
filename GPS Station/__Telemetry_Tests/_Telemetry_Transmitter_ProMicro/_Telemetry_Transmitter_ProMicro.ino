// Arduino; TRANSMITTER
#include <SoftwareSerial.h>

const byte rxPin_GPS = 8;
const byte txPin_GPS = 9;
const byte rxPin_XBee = 1;
const byte txPin_XBee = 0;

// Connect the GPS RX/TX to arduino pins 8 and 9
SoftwareSerial serial_GPS = SoftwareSerial(rxPin_GPS, txPin_GPS);
SoftwareSerial serial_Xbee = SoftwareSerial(rxPin_XBee, txPin_XBee);

#include <SD.h>

File GPSFile;
const int CS = 10; // ChipSelect

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

  while ( serial_GPS.available() ) {
    byte c = serial_GPS.read();
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

void setup()
{
  serial_GPS.begin(9600);
  serial_Xbee.begin(9600);
  Serial.begin(9600);
  // initialize both serial ports:
  pinMode(rxPin_GPS, INPUT);
  pinMode(txPin_GPS, OUTPUT);
  pinMode(rxPin_XBee, INPUT);
  pinMode(txPin_XBee, OUTPUT);

  // send configuration data in UBX protocol
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    serial_GPS.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }

  // SD CARD
  // Initialize SD Card
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    Serial.println("initialization of SD card failed!");
    return;
  }
}

void loop() {
  if ( processGPS() ) {
    Serial.print("I'm here: ");
    Serial.print("iTOW:");      Serial.print(posllh.iTOW);
    Serial.print(" lat/lon: "); Serial.print(posllh.lat / 10000000.0f); Serial.print(","); Serial.print(posllh.lon / 10000000.0f);
    Serial.print(" height: ");  Serial.print(posllh.height / 1000.0f);
    Serial.println();

    serial_Xbee.print("I'm here: ");
    serial_Xbee.print("iTOW:");      serial_Xbee.print(posllh.iTOW);
    serial_Xbee.print(" lat/lon: "); serial_Xbee.print(posllh.lat / 10000000.0f); serial_Xbee.print(","); serial_Xbee.print(posllh.lon / 10000000.0f);
    serial_Xbee.print(" height: ");  serial_Xbee.print(posllh.height / 1000.0f);
    serial_Xbee.println();

    // Store data on SD card
    GPSFile = SD.open("GPS_ROVER.txt", FILE_WRITE);
    GPSFile.print("iTOW: ");
    GPSFile.println(posllh.iTOW);
    GPSFile.print("lat/lon: ");
    GPSFile.print(posllh.lat / 10000000.0f);
    GPSFile.print(", ");
    GPSFile.println(posllh.lon / 10000000.0f);
    GPSFile.print("height: ");
    GPSFile.println(posllh.height / 1000.0f);
    GPSFile.close();
  }
}
