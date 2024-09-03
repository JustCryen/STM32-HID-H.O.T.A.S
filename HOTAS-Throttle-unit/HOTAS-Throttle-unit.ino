//int send = 0;
int Buttons = 0;
int sentButtons = 0;
int Throttle = 0;
int sentThrottle = 0;
uint8_t refresh = 0;

void setup() {
  // SoftwareSerial BtSerial = SoftwareSerial(A4, A5);
  Serial.begin(115200);
  // BtSerial.begin(115200);
}

void loop() {
  Throttle = analogRead(A7);

  if (Throttle != sentThrottle | refresh % 16 == 0) {
    sentThrottle = Throttle;
    Serial.print('z');
    if (sentThrottle < 10) Serial.print('0');
    if (sentThrottle < 100) Serial.print('0');
    if (sentThrottle < 1000) Serial.print('0');
    Serial.println(sentThrottle);
  }
  
  Buttons = (digitalRead(2)  << 5) |
            (digitalRead(3)  << 4) |
            (digitalRead(4)  << 3) |
            (digitalRead(5)  << 2) |
            (digitalRead(11) << 1) |
            (digitalRead(12)     );
  
  if (Buttons != sentButtons | (refresh+1) % 16 == 0) {
    sentButtons = Buttons;
    Serial.print('b');
    Serial.println(sentButtons);
  }
  --refresh;
  delay(100);
}
