#import <SoftwareSerial.h>;

const byte rxPin = 8;
const byte txPin = 9;

SoftwareSerial serial(rxPin, txPin);


void setup() {
  // initialize both serial ports:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  serial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // read from port serial, send to port Serial:
  if (serial.available()) {
    int Coord = serial.read();
    Serial.write(Coord);
  }
}
