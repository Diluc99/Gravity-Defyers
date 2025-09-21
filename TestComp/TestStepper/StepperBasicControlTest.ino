int PUL = 18; // Pulse pin
int DIR = 19; // Direction pin
int ENA = 21; // Enable pin

void setup() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);
}

void loop() {
  // Forward 6400 steps
  digitalWrite(DIR, LOW);
  digitalWrite(ENA, HIGH);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL, LOW);
    delayMicroseconds(50);

  // Backward 6400 steps

}
