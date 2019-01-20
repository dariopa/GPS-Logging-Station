int analogValue; // Analog value that Arduino reads
const float maxVoltageBattery = 4.2; // Maximum voltage on battery when charged
const float maxVoltageArduino = 3; // Maximum voltage Arduino should read
const float vpp = maxVoltageBattery / 1023; // Volts per point <-> accuracy of measurement
const float voltageRatio = maxVoltageBattery / maxVoltageArduino; // Ratio to mathematically upscale voltage
float voltage; // Real voltage
 
void setup() {
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogValue = analogRead(A0);
  voltage = (analogValue*vpp)*voltageRatio;
  Serial.println(voltage);
  delay(1000);
  
} 
