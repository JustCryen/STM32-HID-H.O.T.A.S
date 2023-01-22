void setup() {
  Serial.begin(9600);
}

void loop() {
  int Throttle = analogRead(A7);
  Serial.print('x');
  Serial.println(Throttle);
  delay(100);
}
