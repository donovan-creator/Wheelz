#include <Arduino.h>

// Motor driver pins (L298N example)
const int ENA = D5;
const int IN1 = D6;
const int IN2 = D7;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Start serial for debugging
  Serial.begin(115200);
}

void loop() {
  // Basic forward test
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 512); // Half speed (0–1023 on ESP8266)

  delay(2000);

  // Stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  delay(2000);
}