#include "DropRecovery.h"

SettingSD::SettingSD() {
  // Anything required when instantiating an object, goes here.
}

void SettingSD::SdInit() {
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("initialisation failed");
    delay(20);
    return;
  }
  Serial.println("initialisation succeeded");
  delay(20);
}

void SettingSD::OpenFile(File dir) {
  int file_count = -1; // -1 as bms-file will also be counted.
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      gpsFile = SD.open("ROV" + String(file_count) + ".bin", FILE_WRITE);
      break;
    }
    else {
      file_count += 1;
      entry.close();
    }
   
  }
}

void SettingSD::WriteBmsLog(double temp, double volt) {
  bmsFile = SD.open("BMS.txt", FILE_WRITE);
  bmsFile.print(" Temperature ");  bmsFile.print(temp);
  bmsFile.print("°C, Voltage "); bmsFile.print(volt); bmsFile.println("V");
  bmsFile.close();
}
