#include <SD.h>

File myFile;
File root;
const int ChipSelect = 10;
unsigned long weekTime = 13250329;
unsigned long startTime;
unsigned long currTime;
float measTime = 0.5; // in Minutes!

void setup()
{
  Serial.begin(9600);
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
  // myFile = SD.open("test" + String(weekTime)+ ".txt", FILE_WRITE);

  root = SD.open("/");
  countFiles(root);
  delay(5);

  startTime = millis();
}

void loop()
{
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");

    // close the file:
    myFile.flush();
    Serial.println("done.");

  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening .txt-File");
  }
  delay(1000);
  currTime = millis();
  if (currTime - startTime >= measTime * 60 * 1000) {
    myFile.close();
    while (1) {}
  }
}

void countFiles(File dir) {
  int fileCount = 0;
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      myFile = SD.open("ROV_" + String(fileCount) + ".txt", FILE_WRITE);
      break;
    }
    
    else {
      fileCount += 1;
    }
  }
}
