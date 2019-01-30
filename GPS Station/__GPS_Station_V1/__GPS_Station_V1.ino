// Include all relevant libraries.
// TEMPERATURE SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>
// SD CARD
#include <SD.h>

// Define all your variables.
// #################################
// TEMPERATURE SENSOR
#define ONE_WIRE_BUS 5 // Pin 5 on Board. 
float temp; // Variable to store temperature value.
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass oneWire reference to Dallas Temperature.

// SD CARD
File GPSFile;
const int CS = 10; // Chip Select Pin for communication with SD card.
// #################################

void setup() {
  Serial.begin(9600); // Start serial port

  // TEMPERATURE SENSOR
  sensors.begin(); // Start up the library. IC Default 9 bit. If you have troubles consider upping it up to 12. Ups the the delay giving the IC more time to process the temperature measurement

  // SD CARD
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    Serial.println("initialization of SD card failed!");
    return;
  }
}
void loop() {
  // TEMPERATURE SENSOR
  sensors.requestTemperatures(); // Send the command to get temperatures
  temp = sensors.getTempCByIndex(0);
  Serial.print("Temperature for Device is: ");
  Serial.println(temp);
  delay(1000);
}
