#include <Arduino.h>
#include "stirrer.h"
#include <pins.h>
#include <config.h>

void startStirrer() {
  analogWrite(STIRRER_PIN, STIRRER_SPEED);
}

void stopStirrer() {
  analogWrite(STIRRER_PIN, 0);
}
