#include <Arduino.h>
#include "stirrer.h"
#include "config_store.h"
#include <pins.h>

static const uint8_t STIRRER_CHANNEL = 0;
static const uint32_t STIRRER_FREQ = 25000;  // 25 kHz — silent PWM for PC-style fans
static const uint8_t STIRRER_RES = 8;        // 8-bit (0-255)
static uint8_t currentDuty = 0;              // Track current duty for pause/resume

void initStirrer() {
  ledcSetup(STIRRER_CHANNEL, STIRRER_FREQ, STIRRER_RES);
  ledcAttachPin(STIRRER_PIN, STIRRER_CHANNEL);
  ledcWrite(STIRRER_CHANNEL, 0);  // Off after reboot
  currentDuty = 0;
}

void startStirrer() {
  currentDuty = configStore.getStirrerSpeed() * 255 / 100;
  // Kick at full speed briefly to overcome stall at low duty cycles
  if (currentDuty < 255) {
    ledcWrite(STIRRER_CHANNEL, 255);
    delay(1000);
  }
  ledcWrite(STIRRER_CHANNEL, currentDuty);
}

void stopStirrer() {
  currentDuty = 0;
  ledcWrite(STIRRER_CHANNEL, 0);
}

void pauseStirrer() {
  ledcWrite(STIRRER_CHANNEL, 0);
}

void resumeStirrer() {
  ledcWrite(STIRRER_CHANNEL, currentDuty);
}
