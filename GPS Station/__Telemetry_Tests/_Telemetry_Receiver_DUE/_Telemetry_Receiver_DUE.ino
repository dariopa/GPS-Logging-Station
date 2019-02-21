// Arduino RECEIVER

#include <SD.h>

File GPSFile;
const int CS = 10; // Chipselect

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

  // SD CARD
  // Initialize SD Card
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    Serial.println("initialization of SD card failed!");
    return;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial1.available() > 0) {
    char textBuffer[100] = {Serial1.read()};
    Serial.write(textBuffer);

    GPSFile = SD.open("BASE.txt", FILE_WRITE);
    GPSFile.print(textBuffer);
    GPSFile.close();
  }
}
