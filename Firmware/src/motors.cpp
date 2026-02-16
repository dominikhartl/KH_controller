#include <Arduino.h>
#include "motors.h"
#include <pins.h>
#include <config.h>

static MotorYieldCallback yieldCb = nullptr;
static MotorProgressCallback progressCb = nullptr;

// Wash progress tracking (shared between removeSample/takeSample when called from washSample)
static int washTotalVol = 0;
static int washBaseVol = 0;  // volume completed before current phase

void setMotorYieldCallback(MotorYieldCallback cb) {
  yieldCb = cb;
}

void setMotorProgressCallback(MotorProgressCallback cb) {
  progressCb = cb;
}

// Single step helper
static inline void stepPulse(uint8_t pin, float halfPeriodUs) {
  digitalWrite(pin, HIGH);
  delayMicroseconds((unsigned int)halfPeriodUs);
  digitalWrite(pin, LOW);
  delayMicroseconds((unsigned int)halfPeriodUs);
}

// Count how many steps the acceleration/deceleration ramp takes
static int rampStepCount() {
  int count = 0;
  float acc = MOTOR_START_SPEED;
  while (acc > MOTOR_TARGET_SPEED) {
    acc *= MOTOR_ACCEL_FACTOR;
    count++;
  }
  return count;
}

// Shared sample pump logic — direction is the only difference between remove and fill
static void runSamplePump(int volume, bool forward) {
  digitalWrite(EN_PIN1, LOW);
  delay(MOTOR_ENABLE_DELAY_MS);
  digitalWrite(DIR_PIN1, forward ? HIGH : LOW);
  unsigned long startTime = millis();

  int totalSteps = volume * STEPS_PER_REVOLUTION;
  int rampLen = rampStepCount();

  // Ensure we have room for both accel and decel within totalSteps
  int decelStart = totalSteps - rampLen;
  if (decelStart < rampLen) decelStart = totalSteps / 2;

  int stepsDone = 0;
  bool timedOut = false;

  // Acceleration phase
  float acc = MOTOR_START_SPEED;
  while (acc > MOTOR_TARGET_SPEED && stepsDone < decelStart) {
    stepPulse(STEP_PIN1, acc);
    acc *= MOTOR_ACCEL_FACTOR;
    stepsDone++;
  }

  // Constant speed phase
  while (stepsDone < decelStart) {
    stepPulse(STEP_PIN1, MOTOR_TARGET_SPEED);
    stepsDone++;
    if (stepsDone % (STEPS_PER_REVOLUTION * MOTOR_YIELD_INTERVAL) == 0) {
      if (yieldCb) yieldCb();
      if (washTotalVol > 0 && progressCb) {
        int revsDone = stepsDone / STEPS_PER_REVOLUTION;
        int done = washBaseVol + revsDone;
        progressCb((done * 100) / washTotalVol);
      }
    }
    if (millis() - startTime > SAMPLE_PUMP_TIMEOUT_MS) {
      Serial.println("ERROR: Sample pump timeout!");
      timedOut = true;
      break;
    }
  }

  // Deceleration phase (skip on timeout — stop immediately)
  if (!timedOut) {
    acc = MOTOR_TARGET_SPEED;
    while (stepsDone < totalSteps) {
      stepPulse(STEP_PIN1, acc);
      acc /= MOTOR_ACCEL_FACTOR;
      if (acc > MOTOR_START_SPEED) acc = MOTOR_START_SPEED;
      stepsDone++;
    }
  }

  delay(MOTOR_HOLD_MS);
  digitalWrite(EN_PIN1, HIGH);
}

void removeSample(int volume) {
  runSamplePump(volume, false);
}

void takeSample(int volume) {
  runSamplePump(volume, true);
}

void washSample(float remPart, float fillPart) {
  int removeVol = (int)(SAMPLE_PUMP_VOLUME * remPart);
  int fillVol = (int)(SAMPLE_PUMP_VOLUME * fillPart);
  washTotalVol = removeVol + fillVol;
  washBaseVol = 0;

  if (progressCb) progressCb(0);
  removeSample(removeVol);

  washBaseVol = removeVol;
  takeSample(fillVol);

  washTotalVol = 0;
  if (progressCb) progressCb(100);
}

void titrate(int volume, int stepDelay) {
  // Only add enable settle delay if motor wasn't already on
  if (digitalRead(EN_PIN2) != LOW) {
    digitalWrite(EN_PIN2, LOW);
    delay(MOTOR_ENABLE_DELAY_MS);
  }
  digitalWrite(DIR_PIN2, LOW);
  unsigned long startTime = millis();

  int totalSteps = volume * MOTOR_STEPS_PER_DROP;

  if (volume > TITRATE_ACCEL_THRESHOLD) {
    // Large volume: use acceleration/deceleration
    // Ramp from MOTOR_START_SPEED down to stepDelay
    int rampLen = 0;
    float tmp = MOTOR_START_SPEED;
    while (tmp > (float)stepDelay) {
      tmp *= MOTOR_ACCEL_FACTOR;
      rampLen++;
    }

    int decelStart = totalSteps - rampLen;
    if (decelStart < rampLen) decelStart = totalSteps / 2;

    int stepsDone = 0;

    // Acceleration
    float acc = MOTOR_START_SPEED;
    while (acc > (float)stepDelay && stepsDone < decelStart) {
      stepPulse(STEP_PIN2, acc);
      acc *= MOTOR_ACCEL_FACTOR;
      stepsDone++;
    }

    // Constant speed
    while (stepsDone < decelStart) {
      stepPulse(STEP_PIN2, (float)stepDelay);
      stepsDone++;
      if (stepsDone % (MOTOR_STEPS_PER_DROP * MOTOR_YIELD_INTERVAL * 50) == 0) {
        if (yieldCb) yieldCb();
        if (millis() - startTime > TITRATION_TIMEOUT_MS) {
          Serial.println("ERROR: Titration timeout!");
          return;
        }
      }
    }

    // Deceleration
    acc = (float)stepDelay;
    while (stepsDone < totalSteps) {
      stepPulse(STEP_PIN2, acc);
      acc /= MOTOR_ACCEL_FACTOR;
      if (acc > MOTOR_START_SPEED) acc = MOTOR_START_SPEED;
      stepsDone++;
    }
  } else {
    // Small volume (typical titration drops): run directly, no ramp needed
    for (int i = 0; i < totalSteps; i++) {
      stepPulse(STEP_PIN2, (float)stepDelay);
    }
  }

  // No hold/disable here — caller manages EN_PIN2 to avoid
  // enable/disable overhead on every small titration step
}
