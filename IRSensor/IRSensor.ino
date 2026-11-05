#include <Arduino.h>

const int LEFT_IR = A5;
const int RIGHT_IR = A2;

unsigned long lastLog = 0;

void setup() {
  // Stop both motors.
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  analogWrite(10, 0);
  analogWrite(9, 0);

  pinMode(A4, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);

  digitalWrite(A4, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);

  pinMode(LEFT_IR, INPUT_PULLUP);
  pinMode(RIGHT_IR, INPUT_PULLUP);

  analogReadResolution(10);  // Readings from 0 to 1023.
  Serial.begin(9600);
  lastLog = millis();
}

void logSensor(const char* name, int pin) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(100);

  int digitalLevel = digitalRead(pin);
  int outputLevel = analogRead(pin);

  // Restore digital input configuration after analog measurement.
  pinMode(pin, INPUT_PULLUP);

  Serial.print(' ');
  Serial.print(name);
  Serial.print("_digital=");
  Serial.print(digitalLevel);

  Serial.print(' ');
  Serial.print(name);
  Serial.print("_adc=");
  Serial.print(outputLevel);
}

void loop() {
  unsigned long now = millis();

  if (!Serial || now - lastLog < 200) {
    return;
  }
  lastLog = now;

  Serial.print("IR_TEST motors=OFF");
  logSensor("left_A5", LEFT_IR);
  logSensor("right_A2", RIGHT_IR);
  Serial.println("");
}
