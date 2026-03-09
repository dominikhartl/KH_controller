#include <Arduino.h>
#include "stirrer.h"
#include "config_store.h"
#include <pins.h>

static bool stirrerRunning = false;

void initStirrer() {
  pinMode(STIRRER_PIN, OUTPUT);
  analogWrite(STIRRER_PIN, 0);
}

void startStirrer() {
  int pct = configStore.getStirrerSpeed();  // 80-100%
  int duty = (pct * 255) / 100;
  if (duty > 255) duty = 255;
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
