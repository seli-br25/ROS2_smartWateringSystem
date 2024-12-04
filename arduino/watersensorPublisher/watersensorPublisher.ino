#define MOISTURE_PIN A0 

void setup() {
  pinMode(MOISTURE_PIN, INPUT);
  Serial.begin(57600);
}

void loop() {
  int moisture = analogRead(MOISTURE_PIN);
  if (moisture < 50) { 
    Serial.println("dry");
  } else {
    Serial.println("moist");
  }
  delay(1000); 
}
