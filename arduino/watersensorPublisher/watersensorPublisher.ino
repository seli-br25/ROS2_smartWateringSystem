#define WATERSENSOR_PIN A0

void setup() {
  pinMode(WATERSENSOR_PIN, INPUT);
  Serial.begin(57600); 
}

void loop() {
  // measure soil moisture
  int moisture = analogRead(WATERSENSOR_PIN);
  String status;

  if (moisture < 50) {
    status = "dry";
  } else {
    status = "moist";
  }

  // send moisture status over serial
  Serial.println(status);

  delay(1000); 
}
