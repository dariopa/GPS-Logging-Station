#include "DropRecovery.h"
#include <OneWire.h>
#include <DallasTemperature.h>

SettingBMS::SettingBMS() {
  // Anything required when instantiating an object, goes here.
  const float _MaxVoltageBattery = 4.2; // Maximum voltage on battery when charged
  const float _MaxVoltageArduino = 3.3; // Maximum voltage Arduino should read
  const float _LowVoltage = 3.3; // Lowest Voltage where TPL5110 has to be toggled
  const float _Vpp = _MaxVoltageBattery / 1023; // Volts per point <-> accuracy of measurement
  const float _VoltageRatio = _MaxVoltageBattery / _MaxVoltageArduino; // Ratio to mathematically upscale voltage
}

float SettingBMS::Bms() {

  // Temperature
  OneWire oneWire(5); // Setup a oneWire instance to communicate with any OneWire devices on pin 5 on Arduino Board (not just Maxim/Dallas temperature ICs)
  DallasTemperature sensors(&oneWire); // Pass oneWire reference to Dallas Temperature.
  sensors.begin(); // Start up the library. IC Default 9 bit.
  sensors.requestTemperatures(); // Send the command to get temperatures
  temperature = sensors.getTempCByIndex(0); // Read the temperature
  delay(5);

  // Voltage
  analog_value = analogRead(A0);
  real_voltage = (analog_value * _Vpp) * _VoltageRatio;
  delay(5);

  return temperature, real_voltage;
}
