unsigned long startTime;
unsigned long currTime;
float measTime = 0.5; // in Minutes

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  startTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  currTime = millis();
  Serial.println(currTime - startTime);
  
  if (currTime - startTime > (measTime * 60 * 1000)) {
    while (1) {
      Serial.println("I'm done!!");
    }
  }
}
