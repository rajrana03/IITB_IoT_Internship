const int reset_pin = 2; 

void setup() {
  // Initialize the digital pin as an output
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, LOW);
}

void loop() {
  // Turn the pin HIGH (ON)
  digitalWrite(reset_pin, HIGH);

  // Wait for 1 second (1000 milliseconds)
  delay(1000);

  // Turn the pin LOW (OFF)
  digitalWrite(reset_pin, LOW);

  while(1);
}




