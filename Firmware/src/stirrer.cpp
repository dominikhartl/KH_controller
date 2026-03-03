#include <Arduino.h>
#include "stirrer.h"
#include "config_store.h"
#include <pins.h>

static uint8_t currentDuty = 0;

void initStirrer() {
  analogWrite(STIRRER_PIN, 0);
  currentDuty = 0;
}

void startStirrer() {
  currentDuty = configStore.getStirrerSpeed() * 255 / 100;
  // Kick at full speed briefly to overcome stall at low duty cycles
  if (currentDuty < 255) {
    analogWrite(STIRRER_PIN, 255);
    delay(1000);
  }
  analogWrite(STIRRER_PIN, currentDuty);
}

void stopStirrer() {
  currentDuty = 0;
  analogWrite(STIRRER_PIN, 0);
}

void pauseStirrer() {
  analogWrite(STIRRER_PIN, 0);
}

void resumeStirrer() {
  analogWrite(STIRRER_PIN, currentDuty);
}
