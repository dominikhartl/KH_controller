#include <Arduino.h>
#include "motors.h"
#include <pins.h>
#include <config.h>

// Pre-computed half-period for acceleration start speed
static const float startUs  = rpmToHalfPeriodUs(MOTOR_START_RPM);

static MotorYieldCallback yieldCb = nullptr;
static MotorProgressCallback progressCb = nullptr;

// Wash progress tracking (shared between removeSample/takeSample when called from washSample)
static int washTotalVol = 0;
static int washBaseVol = 0;  // volume completed before current phase

// Multi-wash context: spans progress across sequential washSample() calls
static int multiWashTotal = 0;   // total number of washes in sequence (0 = disabled)
static int multiWashIndex = 0;   // current wash index (0-based)

void setMotorYieldCallback(MotorYieldCallback cb) {
  yieldCb = cb;
}

void setMotorProgressCallback(MotorProgressCallback cb) {
  progressCb = cb;
}

void setMultiWashContext(int numWashes) {
  multiWashTotal = numWashes;
  multiWashIndex = 0;
}

void clearMultiWashContext() {
  multiWashTotal = 0;
  multiWashIndex = 0;
}

// Precise step helper — used by titration motor where volume accuracy matters
static inline void stepPulse(uint8_t pin, float halfPeriodUs) {
  unsigned int hp = (unsigned int)halfPeriodUs;
  digitalWrite(pin, HIGH);
  delayMicroseconds(hp);
  digitalWrite(pin, LOW);
  delayMicroseconds(hp);
}

// Spread-spectrum step helper — ±10% jitter to smear stepping tone into broadband noise.
// Used only by sample pump where audible noise matters more than sub-step timing precision.
// Jitter is compensated per full step (+d HIGH, -d LOW) so average period is unchanged.
static inline void stepPulseJittered(uint8_t pin, float halfPeriodUs) {
  int jitter = (int)(halfPeriodUs * 0.1f);
  int d = (jitter > 0) ? random(-jitter, jitter + 1) : 0;
  unsigned int hpHigh = (unsigned int)halfPeriodUs + d;
  unsigned int hpLow  = (unsigned int)halfPeriodUs - d;
  digitalWrite(pin, HIGH);
  delayMicroseconds(hpHigh);
  digitalWrite(pin, LOW);
  delayMicroseconds(hpLow);
}

// Count how many steps the acceleration/deceleration ramp takes
static int rampStepCount(float targetUs) {
  int count = 0;
  float acc = startUs;
  while (acc > targetUs) {
    acc *= MOTOR_ACCEL_FACTOR;
    count++;
  }
  return count;
}

// Track last direction for backlash compensation
static bool lastSampleDirection = true;

// Shared sample pump logic — direction is the only difference between remove and fill
// Returns false on timeout
static bool runSamplePump(int volume, bool forward, float speedRpm) {
  float targetUs = rpmToHalfPeriodUs(speedRpm);
  digitalWrite(EN_PIN1, LOW);
  delay(MOTOR_ENABLE_DELAY_MS);
  digitalWrite(DIR_PIN1, forward ? HIGH : LOW);

  // Backlash compensation on direction reversal
  if (forward != lastSampleDirection) {
    for (int i = 0; i < BACKLASH_COMPENSATION_STEPS; i++) {
      stepPulse(STEP_PIN1, startUs);
    }
  }
  lastSampleDirection = forward;
  unsigned long startTime = millis();

  int totalSteps = volume * STEPS_PER_REVOLUTION;
  int rampLen = rampStepCount(targetUs);

  // Ensure we have room for both accel and decel within totalSteps
  int decelStart = totalSteps - rampLen;
  if (decelStart < rampLen) decelStart = totalSteps / 2;

  int stepsDone = 0;
  bool timedOut = false;

  // Acceleration phase
  float acc = startUs;
  while (acc > targetUs && stepsDone < decelStart) {
    stepPulseJittered(STEP_PIN1, acc);
    acc *= MOTOR_ACCEL_FACTOR;
    stepsDone++;
  }

  // Constant speed phase
  while (stepsDone < decelStart) {
    stepPulseJittered(STEP_PIN1, targetUs);
    stepsDone++;
    if (stepsDone % (STEPS_PER_REVOLUTION * MOTOR_YIELD_INTERVAL) == 0) {
      if (yieldCb) yieldCb();
      if (washTotalVol > 0 && progressCb) {
        int revsDone = stepsDone / STEPS_PER_REVOLUTION;
        int done = washBaseVol + revsDone;
        int singlePct = (done * 100) / washTotalVol;
        if (multiWashTotal > 0) {
          // Scale into overall multi-wash progress
          progressCb((multiWashIndex * 100 + singlePct) / multiWashTotal);
        } else {
          progressCb(singlePct);
        }
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
    acc = targetUs;
    while (stepsDone < totalSteps) {
      stepPulseJittered(STEP_PIN1, acc);
      acc /= MOTOR_ACCEL_FACTOR;
      if (acc > startUs) acc = startUs;
      stepsDone++;
    }
  }

  delay(MOTOR_HOLD_MS);
  digitalWrite(EN_PIN1, HIGH);
  return !timedOut;
}

bool removeSample(int volume, float speedRpm) {
  return runSamplePump(volume, false, speedRpm);
}

bool takeSample(int volume, float speedRpm) {
  return runSamplePump(volume, true, speedRpm);
}

bool washSample(float remPart, float fillPart, float speedRpm) {
  int removeVol = (int)(SAMPLE_PUMP_VOLUME * remPart);
  int fillVol = (int)(SAMPLE_PUMP_VOLUME * fillPart);
  washTotalVol = removeVol + fillVol;
  washBaseVol = 0;

  if (progressCb) {
    if (multiWashTotal > 0) {
      progressCb((multiWashIndex * 100) / multiWashTotal);
    } else {
      progressCb(0);
    }
  }

  bool ok = removeSample(removeVol, speedRpm);

  if (ok) {
    washBaseVol = removeVol;
    ok = takeSample(fillVol, speedRpm);
  }

  washTotalVol = 0;

  if (multiWashTotal > 0) {
    multiWashIndex++;
    if (progressCb) progressCb((multiWashIndex * 100) / multiWashTotal);
  } else {
    if (progressCb) progressCb(100);
  }
  return ok;
}

bool titrate(int volume, float speedRpm, bool noAccel) {
  float speedUs = rpmToHalfPeriodUs(speedRpm);

  // Only add enable settle delay if motor wasn't already on
  if (digitalRead(EN_PIN2) != LOW) {
    digitalWrite(EN_PIN2, LOW);
    delay(MOTOR_ENABLE_DELAY_MS);
  }
  digitalWrite(DIR_PIN2, LOW);
  unsigned long startTime = millis();

  int totalSteps = volume * MOTOR_STEPS_PER_UNIT;

  if (volume > TITRATE_ACCEL_THRESHOLD && !noAccel) {
    // Large volume: use acceleration/deceleration
    // Ramp from startUs (slow) down to speedUs (fast)
    int rampLen = 0;
    float tmp = startUs;
    while (tmp > speedUs) {
      tmp *= MOTOR_ACCEL_FACTOR;
      rampLen++;
    }

    int decelStart = totalSteps - rampLen;
    if (decelStart < rampLen) decelStart = totalSteps / 2;

    int stepsDone = 0;

    // Acceleration
    float acc = startUs;
    while (acc > speedUs && stepsDone < decelStart) {
      stepPulse(STEP_PIN2, acc);
      acc *= MOTOR_ACCEL_FACTOR;
      stepsDone++;
    }

    // Record actual speed reached (may not have hit target if ramp > totalSteps/2)
    float speedReached = acc;

    // Constant speed
    while (stepsDone < decelStart) {
      stepPulse(STEP_PIN2, speedUs);
      stepsDone++;
      speedReached = speedUs;
      if (stepsDone % (MOTOR_STEPS_PER_UNIT * MOTOR_YIELD_INTERVAL * 50) == 0) {
        if (yieldCb) yieldCb();
        if (millis() - startTime > TITRATION_TIMEOUT_MS) {
          Serial.println("ERROR: Titration timeout!");
          return false;
        }
      }
    }

    // Deceleration — start from actual speed, not target
    acc = speedReached;
    while (stepsDone < totalSteps) {
      stepPulse(STEP_PIN2, acc);
      acc /= MOTOR_ACCEL_FACTOR;
      if (acc > startUs) acc = startUs;
      stepsDone++;
    }
  } else {
    // Small volume: absolute-time stepping immune to interrupt jitter.
    // No spread-spectrum here — titration motor needs precise timing for volume accuracy.
    unsigned int halfPeriod = (unsigned int)speedUs;
    unsigned long t = micros();
    for (int i = 0; i < totalSteps; i++) {
      t += halfPeriod;
      digitalWrite(STEP_PIN2, HIGH);
      while ((long)(micros() - t) < 0) {}
      t += halfPeriod;
      digitalWrite(STEP_PIN2, LOW);
      while ((long)(micros() - t) < 0) {}
    }
  }

  // No hold/disable here — caller manages EN_PIN2 to avoid
  // enable/disable overhead on every small titration step
  return true;
}
