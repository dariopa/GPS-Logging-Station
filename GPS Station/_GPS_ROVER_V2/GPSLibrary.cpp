#include "DropRecovery.h"

SettingGPS::SettingGPS() {
  // Anything required when instantiating an object, goes here.
  buffer_length = 2000;
}

void SettingGPS::GpsInit() {
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  delay(8000); // wait 8 seconds until position lock on GPS receiver
}

void SettingGPS::RawxConfig() {
  for (int i = 0; i < sizeof(UBLOX_INIT_RAWX); i++) {
    Serial1.write( pgm_read_byte(UBLOX_INIT_RAWX + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  delay(20);
}
