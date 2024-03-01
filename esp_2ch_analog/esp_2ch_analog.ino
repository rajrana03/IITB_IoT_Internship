int16_t analog1 = 0;
int16_t analog2 = 0;

void setup() {
  Serial.begin(1500000);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  analog1 = analogRead(34);
  analog2 = analogRead(35);
  // Serial.println(analog);
  
    Serial.write(0xAA);
    Serial.write((uint8_t *)&(analog1), sizeof(analog1));
    Serial.write((uint8_t *)&(analog2), sizeof(analog2));
}