#include <SD.h>

File myFile;

const int ChipSelect = 10;

void setup() {
  // initialize required serial ports:
  Serial.begin(9600);
  Serial1.begin(9600);

  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(ChipSelect, OUTPUT);

  if (!SD.begin(ChipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

}

void loop() {
  // read from port 1, send to port 0:

  int var = 0;
  while (var < 500) {
    if (Serial1.available()) {
      int Coord = Serial1.read();
      Serial.write(Coord);
      var++;
    }
  }

  myFile = SD.open("test.txt", FILE_WRITE);

  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");

    // close the file:
    myFile.close();
    Serial.println("done.");

  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  delay(250);
}
