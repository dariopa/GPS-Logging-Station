#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 5 on the Arduino Pro Micro 3.3V
#define ONE_WIRE_BUS 5

float temp; 

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

void setup(void) {
  //start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it up to 12. Ups the the delay giving the IC more time to process the temperature measurement
}

void loop(void) {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures ... ");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");

  temp = sensors.getTempCByIndex(0);
  Serial.print("Temperature for Device is: ");
  Serial.println(temp);  

  delay(1000);
}
