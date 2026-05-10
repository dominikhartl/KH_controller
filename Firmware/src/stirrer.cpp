#include <Arduino.h>
#include "stirrer.h"
#include "config.h"
#include <pins.h>

static bool stirrerRunning = false;

void initStirrer() {
  pinMode(STIRRER_PIN, OUTPUT);
  analogWrite(STIRRER_PIN, 0);
}

void startStirrer() {
  // STIRRER_SPEED_PCT is a compile-time constant in [0, 100], so duty fits in [0, 255]
  int duty = (STIRRER_SPEED_PCT * 255) / 100;
  analogWrite(STIRRER_PIN, duty);
  stirrerRunning = true;
}

void stopStirrer() {
  analogWrite(STIRRER_PIN, 0);
  stirrerRunning = false;
}

bool isStirrerRunning() {
  return stirrerRunning;
}
