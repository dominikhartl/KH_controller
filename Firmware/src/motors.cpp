#include <Arduino.h>
#include "motors.h"
#include "tmc_driver.h"
#include "config_store.h"
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

// Per-operation SG stats for tube wear tracking
static uint32_t sampleSGSum = 0, sampleSGCount = 0;
static uint16_t sampleSGMin = 65535;
static uint32_t titrateSGSum = 0, titrateSGCount = 0;
static uint16_t titrateSGMin = 65535;

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
  unsigned int hp = (unsigned int)(halfPeriodUs + 0.5f);
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
  unsigned int hpHigh = (unsigned int)(halfPeriodUs + 0.5f) + d;
  unsigned int hpLow  = (unsigned int)(halfPeriodUs + 0.5f) - d;
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
  clearStallFlag();
  sampleSGSum = 0; sampleSGCount = 0; sampleSGMin = 65535;
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
      { uint16_t sg = getSampleSG();
        sampleSGSum += sg; sampleSGCount++;
        if (sg < sampleSGMin) sampleSGMin = sg;
        if (sg < (uint16_t)configStore.getSampleStallSG()) {
          Serial.printf("ERROR: Sample pump stall (SG=%d)\n", sg);
          setStallFlag();
          timedOut = true;
          break;
        }
      }
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
    if (ok) multiWashIndex++;
    if (progressCb) progressCb((multiWashIndex * 100) / multiWashTotal);
  } else {
    if (progressCb) progressCb(ok ? 100 : 0);
  }
  return ok;
}

bool washSampleVol(int removeRevs, int fillRevs, float speedRpm) {
  washTotalVol = removeRevs + fillRevs;
  washBaseVol = 0;

  if (progressCb) {
    if (multiWashTotal > 0) {
      progressCb((multiWashIndex * 100) / multiWashTotal);
    } else {
      progressCb(0);
    }
  }

  bool ok = removeSample(removeRevs, speedRpm);

  if (ok) {
    washBaseVol = removeRevs;
    ok = takeSample(fillRevs, speedRpm);
  }

  washTotalVol = 0;

  if (multiWashTotal > 0) {
    if (ok) multiWashIndex++;
    if (progressCb) progressCb((multiWashIndex * 100) / multiWashTotal);
  } else {
    if (progressCb) progressCb(ok ? 100 : 0);
  }
  return ok;
}

bool titrate(int volume, float speedRpm, bool noAccel) {
  clearStallFlag();
  titrateSGSum = 0; titrateSGCount = 0; titrateSGMin = 65535;
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
        if (isTitrateStalled()) {
          Serial.println("ERROR: Titration pump stall detected (DIAG)!");
          setStallFlag();
          return false;
        }
        { uint16_t sg = getTitrateSG();
          titrateSGSum += sg; titrateSGCount++;
          if (sg < titrateSGMin) titrateSGMin = sg;
          if (sg < (uint16_t)configStore.getTitrateStallSG()) {
            Serial.printf("ERROR: Titration pump stall (SG=%d)\n", sg);
            setStallFlag();
            return false;
          }
        }
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
    unsigned int halfPeriod = (unsigned int)(speedUs + 0.5f);
    unsigned long t = micros();
    for (int i = 0; i < totalSteps; i++) {
      t += halfPeriod;
      digitalWrite(STEP_PIN2, HIGH);
      while ((long)(micros() - t) < 0) {}
      t += halfPeriod;
      digitalWrite(STEP_PIN2, LOW);
      while ((long)(micros() - t) < 0) {}
      // Check DIAG pin every 128 steps for stall detection
      if ((i & 0x7F) == 0x7F && isTitrateStalled()) {
        Serial.println("ERROR: Titration pump stall detected (DIAG)!");
        setStallFlag();
        return false;
      }
    }
  }

  // No hold/disable here — caller manages EN_PIN2 to avoid
  // enable/disable overhead on every small titration step
  return true;
}

// Diagnostic: run sample pump for N revolutions, collect SG every revolution
int diagStepSample(int revolutions, float rpm, SGSample* samples, int maxSamples) {
  if (!isTMCDetected()) return 0;
  float halfPeriodUs = rpmToHalfPeriodUs(rpm);
  int totalSteps = revolutions * STEPS_PER_REVOLUTION;
  int nSamples = 0;

  digitalWrite(EN_PIN1, LOW);
  delay(MOTOR_ENABLE_DELAY_MS);
  digitalWrite(DIR_PIN1, HIGH);

  for (int i = 0; i < totalSteps; i++) {
    stepPulseJittered(STEP_PIN1, halfPeriodUs);
    if ((i + 1) % STEPS_PER_REVOLUTION == 0 && nSamples < maxSamples) {
      samples[nSamples].sg = getSampleSG();
      samples[nSamples].diag = digitalRead(DIAG_SAMPLE);
      nSamples++;
    }
  }

  delay(MOTOR_HOLD_MS);
  digitalWrite(EN_PIN1, HIGH);
  return nSamples;
}

// Diagnostic: run titration pump for N revolutions, collect SG every revolution
int diagStepTitrate(int revolutions, float rpm, SGSample* samples, int maxSamples) {
  if (!isTMCDetected()) return 0;
  float halfPeriodUs = rpmToHalfPeriodUs(rpm);
  int totalSteps = revolutions * STEPS_PER_REVOLUTION;
  int nSamples = 0;

  digitalWrite(EN_PIN2, LOW);
  delay(MOTOR_ENABLE_DELAY_MS);
  digitalWrite(DIR_PIN2, LOW);

  for (int i = 0; i < totalSteps; i++) {
    stepPulse(STEP_PIN2, halfPeriodUs);
    if ((i + 1) % STEPS_PER_REVOLUTION == 0 && nSamples < maxSamples) {
      samples[nSamples].sg = getTitrateSG();
      samples[nSamples].diag = digitalRead(DIAG_TITRATE);
      nSamples++;
    }
  }

  delay(MOTOR_HOLD_MS);
  digitalWrite(EN_PIN2, HIGH);
  return nSamples;
}

// Stall speed ramp: ramp sample pump from startRPM to maxRPM in stepRPM increments
// Returns RPM at which stall detected, or 0.0 if no stall within range
float diagStallRamp(float startRPM, float maxRPM, float stepRPM, int revsPerStep,
                    SGSample* samples, int maxSamples, int* totalSamples,
                    bool dirForward, StallRampCallback rpmCb) {
  if (!isTMCDetected()) { *totalSamples = 0; return 0.0f; }

  int nSamples = 0;
  int stallSG = configStore.getSampleStallSG();

  // Disable hardware DIAG stall — SGTHRS=50 triggers at SG≤100 which overlaps
  // normal operating range (SG min ~94). Use software SG check only.
  disableSampleStallGuard();  // SGTHRS=0 for entire ramp
  clearStallFlag();
  digitalWrite(EN_PIN1, LOW);
  delay(MOTOR_ENABLE_DELAY_MS);
  digitalWrite(DIR_PIN1, dirForward ? HIGH : LOW);
  delay(100);

  // Warm-up: run 5 revolutions at start speed so StallGuard stabilizes in new direction
  {
    float warmupUs = rpmToHalfPeriodUs(startRPM);
    for (int rev = 0; rev < 5; rev++) {
      for (int step = 0; step < STEPS_PER_REVOLUTION; step++) {
        stepPulseJittered(STEP_PIN1, warmupUs);
      }
    }
    delay(50);
  }

  // Software SG stall detection only (DIAG disabled).
  // Skip first 4 speed steps for StallGuard settling after direction change.
  int stepCount = 0;
  for (float rpm = startRPM; rpm <= maxRPM; rpm += stepRPM) {
    if (rpmCb) rpmCb(rpm);
    float halfPeriodUs = rpmToHalfPeriodUs(rpm);

    for (int rev = 0; rev < revsPerStep; rev++) {
      for (int step = 0; step < STEPS_PER_REVOLUTION; step++) {
        stepPulseJittered(STEP_PIN1, halfPeriodUs);
      }
      // Collect SG after each revolution
      if (nSamples < maxSamples) {
        samples[nSamples].sg = getSampleSG();
        samples[nSamples].diag = digitalRead(DIAG_SAMPLE);
        nSamples++;
      }
      // Stall check via SG value (skip first 4 speed steps for settling)
      if (stepCount >= 4 && stallSG > 0 && nSamples > 0
          && samples[nSamples-1].sg <= (uint16_t)stallSG) {
        *totalSamples = nSamples;
        delay(MOTOR_HOLD_MS);
        digitalWrite(EN_PIN1, HIGH);
        enableSampleStallGuard();  // restore for normal operation
        return rpm;
      }
    }
    stepCount++;

    delay(50); // settle between speed changes
  }

  *totalSamples = nSamples;
  delay(MOTOR_HOLD_MS);
  digitalWrite(EN_PIN1, HIGH);
  enableSampleStallGuard();  // restore for normal operation
  return 0.0f;
}

float diagStallRampTitrate(float startRPM, float maxRPM, float stepRPM, int revsPerStep,
                    SGSample* samples, int maxSamples, int* totalSamples,
                    bool dirForward, StallRampCallback rpmCb) {
  if (!isTMCDetected()) { *totalSamples = 0; return 0.0f; }

  int nSamples = 0;
  int stallSG = configStore.getTitrateStallSG();

  // Disable hardware DIAG stall — use software SG check only
  disableTitrateStallGuard();  // SGTHRS=0 for entire ramp
  clearStallFlag();
  digitalWrite(EN_PIN2, LOW);
  delay(MOTOR_ENABLE_DELAY_MS);
  digitalWrite(DIR_PIN2, dirForward ? LOW : HIGH);
  delay(100);

  // Warm-up: run 5 revolutions at start speed so StallGuard stabilizes
  {
    float warmupUs = rpmToHalfPeriodUs(startRPM);
    for (int rev = 0; rev < 5; rev++) {
      for (int step = 0; step < STEPS_PER_REVOLUTION; step++) {
        stepPulse(STEP_PIN2, warmupUs);
      }
    }
    delay(50);
  }

  // Software SG stall detection only (DIAG disabled).
  int stepCount = 0;
  for (float rpm = startRPM; rpm <= maxRPM; rpm += stepRPM) {
    if (rpmCb) rpmCb(rpm);
    float halfPeriodUs = rpmToHalfPeriodUs(rpm);

    for (int rev = 0; rev < revsPerStep; rev++) {
      for (int step = 0; step < STEPS_PER_REVOLUTION; step++) {
        stepPulse(STEP_PIN2, halfPeriodUs);
      }
      if (nSamples < maxSamples) {
        samples[nSamples].sg = getTitrateSG();
        samples[nSamples].diag = digitalRead(DIAG_TITRATE);
        nSamples++;
      }
      // Stall check via SG value (skip first 4 speed steps for settling)
      if (stepCount >= 4 && stallSG > 0 && nSamples > 0
          && samples[nSamples-1].sg <= (uint16_t)stallSG) {
        *totalSamples = nSamples;
        delay(MOTOR_HOLD_MS);
        digitalWrite(EN_PIN2, HIGH);
        enableTitrateStallGuard();  // restore for normal operation
        return rpm;
      }
    }
    stepCount++;

    delay(50);
  }

  *totalSamples = nSamples;
  delay(MOTOR_HOLD_MS);
  digitalWrite(EN_PIN2, HIGH);
  enableTitrateStallGuard();  // restore for normal operation
  return 0.0f;
}

void getLastSampleSGStats(uint16_t* avg, uint16_t* min) {
  *avg = sampleSGCount > 0 ? (uint16_t)(sampleSGSum / sampleSGCount) : 0;
  *min = sampleSGCount > 0 ? sampleSGMin : 0;
}

void getLastTitrateSGStats(uint16_t* avg, uint16_t* min) {
  *avg = titrateSGCount > 0 ? (uint16_t)(titrateSGSum / titrateSGCount) : 0;
  *min = titrateSGCount > 0 ? titrateSGMin : 0;
}
