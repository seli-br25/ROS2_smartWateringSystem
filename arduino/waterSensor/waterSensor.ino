#define MOISTURE_PIN A0

void setup() {
  Serial.begin(9600);
}

void loop() {
  int moisture = analogRead(MOISTURE_PIN);
  Serial.print("Moisture Level: ");
  Serial.println(moisture);
  delay(1000);
}
