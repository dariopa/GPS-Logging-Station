int donePin = 4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Start serial port
  // Initialise TPL5110
  pinMode(donePin, OUTPUT);
  digitalWrite(donePin, LOW);
  delay(5);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  digitalWrite(donePin, HIGH);
}
