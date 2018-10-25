int pins[7] = {22, 24, 26, 28, 30, 32, 34};
byte numbers[5] = {0x7E, 0x60, 0x3d, 0x79, 0x63};
int current = 0;
int INTERRUPT_PIN = 52;

void setup() {
  for (int i = 0; i < sizeof(pins) / sizeof(int); i++) {
    pinMode(pins[i], OUTPUT);
  }
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(INTERRUPT_PIN, showNext, CHANGE);
  Serial.begin(9600);
}

void showNext() {
  byte this_number = numbers[current];
  byte one = 1;
  Serial.print("this number: ");
  Serial.print(this_number, HEX);
  Serial.println("");
  for (int i = 0; i < sizeof(pins) / sizeof(int); i++) {
    if (this_number & (one << i)) {
      digitalWrite(pins[i], HIGH);
    } else {
      digitalWrite(pins[i], LOW);
    }
  }
  current = (current + 1) % sizeof(numbers);

}

void loop() {
  delay(1000);
}
