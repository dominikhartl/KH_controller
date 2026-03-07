#include <Arduino.h>
#include "stirrer.h"
#include <pins.h>

void initStirrer() {
  pinMode(STIRRER_PIN, OUTPUT);
  digitalWrite(STIRRER_PIN, LOW);
}

void startStirrer() {
  digitalWrite(STIRRER_PIN, HIGH);
}

void stopStirrer() {
  digitalWrite(STIRRER_PIN, LOW);
}
