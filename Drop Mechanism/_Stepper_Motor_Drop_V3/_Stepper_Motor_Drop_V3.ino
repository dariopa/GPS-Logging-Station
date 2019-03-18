#include <AX12A.h>
#include <SoftwareSerial.h>

#define DirectionPin (10u)
#define BaudRate (ul) // Default
#define ID (1u)

const int rxPin = 8;
const int txPin = 9;
SoftwareSerial serial = SoftwareSerial(rxPin, txPin);


int reg = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  serial.begin(9600);
  
  delay(1000);
  ax12a.begin(BaudRate, DirectionPin, &serial);
  ax12a.move(1,100);
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  reg = ax12a.readPosition(ID);
  Serial.println(reg);
  delay(1000);
}
