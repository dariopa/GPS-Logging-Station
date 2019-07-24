#include "DropRecovery.h"


SettingTPL::SettingTPL() {
  // Anything required when instantiating an object, goes here.
  led_green = 2;
  led_red = 3;
}

void SettingTPL::LEDInit() {
  pinMode(led_green, OUTPUT);
  pinMode(led_red, OUTPUT);
  
  // Feedback, that system is on
  digitalWrite(led_green, HIGH);
  delay(2000);
  digitalWrite(led_green, LOW);
}

void SettingTPL::TPLInit() {
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  delay(20);
}

bool SettingTPL::TPLMeasureTime(unsigned long current_time, unsigned long start_time, unsigned long measurment_time) {
  if (current_time - start_time > measurment_time * 60 * 1000) // if measurment_time is exceeded, toggle TPL5110
    return true;
  else
    return false;
}

void SettingTPL::TPLToggle() {
  digitalWrite(4, HIGH); // toggle TPL5110
}
