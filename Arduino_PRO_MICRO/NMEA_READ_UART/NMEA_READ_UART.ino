#import <SoftwareSerial.h>;

const byte rxPin = 8;
const byte txPin = 9;

SoftwareSerial mySerial(rxPin, txPin);


void setup() {
  // initialize both serial ports:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // read from port 1, send to port 0:
  if (mySerial.available()) {
    int Coord = mySerial.read();
    Serial.write(Coord);
  }
}
