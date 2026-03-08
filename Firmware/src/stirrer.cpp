#include <Arduino.h>
#include "stirrer.h"
#include "config_store.h"
#include <pins.h>

void initStirrer() {
  pinMode(STIRRER_PIN, OUTPUT);
  analogWrite(STIRRER_PIN, 0);
}

void startStirrer() {
  int pct = configStore.getStirrerSpeed();  // 80-100%
  int duty = (pct * 255) / 100;
  analogWrite(STIRRER_PIN, duty);
}

void stopStirrer() {
  analogWrite(STIRRER_PIN, 0);
}
