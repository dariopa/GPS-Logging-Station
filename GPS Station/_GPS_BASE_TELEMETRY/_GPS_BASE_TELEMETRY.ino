// Arduino RECEIVER
#include <SD.h>

File gpsFile;
const int CS = 10; // Chipselect
const int bufLen = 1300;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

  // SD CARD
  // Initialize SD Card
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    Serial.println("Initialization of SD card failed!");
    return;
  }
  gpsFile = SD.open("Rov_Tel.bin", FILE_WRITE);
}

void loop() {
  char buf[bufLen];
  int bufIndex = 0;
  while (Serial1.available()) {
    buf[bufIndex] = (char) Serial1.read();
    bufIndex += 1;
  }

  if (bufIndex != 0) {
    gpsFile.write(buf , bufIndex);
    gpsFile.flush();
  }
}
