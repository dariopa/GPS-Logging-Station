// RETRIEVE RAWX MESSAGE FOR RINEX GENERATION.

// Disable NMEA sentences and enable UBX-RXM-RAWX message.
// Set rate to deliberate value.
// Microcontroller: Arduino Pro Micro 3.3V/ 8MHz (https://www.sparkfun.com/products/12587)
// GPS Receiver: NEO-M8P-2 (https://www.sparkfun.com/products/15005)

#include <SoftwareSerial.h>

const byte rxPin = 8;
const byte txPin = 9;

// Connect the GPS RX/TX to arduino pins 8 and 9
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

  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x46, // RXM-RAWX off

  // Enable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x02, 0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x27, 0x4B, // RXM-RAWX on

  // Rate
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xD0, 0x07, 0x01, 0x00, 0x01, 0x00, 0xED, 0xBD, // (0.5Hz)
  // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xB8, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xD9, 0x41, // (0.33Hz)

};

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct RXM_RAWX {
  struct HEADER {
    unsigned char cls;
    unsigned char id;
    unsigned short len;
    double rcvTow;
    unsigned short week;
    signed char leapS;
    unsigned char numMeas;
    char recStat;
    unsigned char reserved1;
  } header;

  // Start of repeated block (numMeas times)
  struct RXM_REPEATED {
    double prMes;
    double cpMes;
    float doMes;
    unsigned char gnssId;
    unsigned char svId;
    unsigned char reserved2;
    unsigned char freqId;
    unsigned short locktime;
    unsigned char cno;
    char prdStdev;
    char cpStdev;
    char doStdev;
    char trkStat;
    unsigned char reserved3;
  } rxm_repeated[30];
} rawx;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(rawx.header) + rawx.header.numMeas * (int)sizeof(rawx.rxm_repeated[0]); i++) {
    CK[0] += ((unsigned char*)(&rawx))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  int payloadSize = sizeof(rawx.header);

  while ( serial.available() ) {
    byte c = serial.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if (fpos == 12) {
        payloadSize += rawx.header.numMeas * (int)sizeof(rawx.rxm_repeated[0]);
      }
      if ( (fpos - 2) < payloadSize )
        ((unsigned char*)(&rawx))[fpos - 2] = c;
        
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
  Serial.println("-----");
  Serial.println(fpos);
  Serial.println(payloadSize);
  Serial.println(rawx.header.numMeas);
  //delay(1000);
  return false;
}

void setup()
{
  serial.begin(9600);
  Serial.begin(9600);
  // initialize both serial ports:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // send configuration data in UBX protocol
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    serial.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

void loop() {
  if ( processGPS() ) {
    Serial.print("rcvTow:");     // Serial.print(rawx.rxm_repeated[1].rcvTow);
    
    Serial.println();
  }
}
