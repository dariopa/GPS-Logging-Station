#include <SoftwareSerial.h>

const byte rx = 8;
const byte tx = 9;

SoftwareSerial mySerial (rx, tx);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(rx, INPUT);
  pinMode(tx, OUTPUT);
  mySerial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while(mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
