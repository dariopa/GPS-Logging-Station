#include "DropRecovery.h"


SettingTPL::SettingTPL() {
  // Anything required when instantiating an object, goes here.
}

void SettingTPL::TPLInit() {
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  delay(20);
}

void SettingTPL::TPLToggle(unsigned long current_time, unsigned long start_time, unsigned long measurment_time) {
  if (current_time - start_time > measurment_time * 60 * 1000) { // if measurment_time is exceeded, toggle TPL5110
    Serial2.println("stuck in processGPS");
    delay(20);
    digitalWrite(4, HIGH); // toggle TPL5110
  }
  else
    delay(20);
}
