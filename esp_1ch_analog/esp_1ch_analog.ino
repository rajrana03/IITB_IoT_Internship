int16_t analog=0;

void setup() {
  Serial.begin(1500000);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  analog = analogRead(34);
  // Serial.println(analog);
  
    Serial.write(0xAA);
    Serial.write((uint8_t *)&(analog), sizeof(analog));
}