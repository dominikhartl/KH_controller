#include <Arduino.h>
#include <esp_task_wdt.h>
#include "motors.h"
#include "tmc_driver.h"
#include "config_store.h"
#include <pins.h>
#include <config.h>

extern void publishMessage(const char* message);

// Pre-computed half-period for acceleration start speed
static const float startUs  = rpmToHalfPeriodUs(MOTOR_START_RPM);

// --- Stall detection constants ---
static const int   SG_BUFFER_SIZE        = 8;     // Rolling SG history for median
static const float SG_STALL_MEDIAN_RATIO = 0.3f;  // Stall if SG < 30% of median
static const int   SG_STALL_FLOOR        = 10;    // Skip stall check if median < this
static const int   SG_CONFIRM_STEPS      = 64;    // Steps to run before re-checking after trigger
static const int   SG_CONFIRM_COUNT      = 2;     // Consecutive fails needed to declare stall

// Rolling median helper for cyclical-load-aware stall detection
struct SGBuffer {
  uint16_t buf[SG_BUFFER_SIZE];
  int count;
  int idx;

  void reset() { count = 0; idx = 0; }

  void add(uint16_t sg) {
    buf[idx] = sg;
    idx = (idx + 1) % SG_BUFFER_SIZE;
    if (count < SG_BUFFER_SIZE) count++;
  }

  uint16_t median() const {
    if (count == 0) return 0;
    // Copy and sort (small fixed-size array — insertion sort is fine)
    uint16_t tmp[SG_BUFFER_SIZE];
    for (int i = 0; i < count; i++) tmp[i] = buf[i];
    for (int i = 1; i < count; i++) {
      uint16_t key = tmp[i];
      int j = i - 1;
      while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
      tmp[j + 1] = key;
    }
    return tmp[count / 2];
  }

  bool settled() const { return count >= 4; }
};

static MotorYieldCallback yieldCb = nullptr;
static MotorProgressCallback progressCb = nullptr;
static MotorAbortCallback abortCb = nullptr;

// Wash progress tracking (shared between removeSample/takeSample when called from washSample)
static int washTotalVol = 0;
static int washBaseVol = 0;  // volume completed before current phase

// Multi-wash context: spans progress across sequential washSample() calls
static int multiWashTotal = 0;   // total number of washes in sequence (0 = disabled)
static int multiWashIndex = 0;   // current wash index (0-based)

// Per-operation SG stats for tube wear tracking
// Note: written by loopTask during motor ops, read by AsyncTCP for diagnostics display.
// No synchronization — acceptable for non-critical diagnostic data.
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

void setMotorAbortCallback(MotorAbortCallback cb) {
  abortCb = cb;
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
  // Dynamic timeout: 3× expected time or minimum SAMPLE_PUMP_TIMEOUT_MS, whichever is greater
  unsigned long expectedMs = (unsigned long)((float)volume / speedRpm * 60000.0f);
  unsigned long timeout = max((unsigned long)SAMPLE_PUMP_TIMEOUT_MS, expectedMs * 3UL);

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
    if (stepsDone % STEPS_PER_REVOLUTION == 0) esp_task_wdt_reset();
  }

  // Clear latched DIAG after acceleration (before stall monitoring begins)
  // DIAG clearing removed — stall detection disabled

  // Constant speed phase
  while (stepsDone < decelStart) {
    stepPulseJittered(STEP_PIN1, targetUs);
    stepsDone++;
    // Feed watchdog every revolution to stay well within WDT timeout
    if (stepsDone % STEPS_PER_REVOLUTION == 0) esp_task_wdt_reset();

    if (stepsDone % (STEPS_PER_REVOLUTION * MOTOR_YIELD_INTERVAL) == 0) {
      if (yieldCb) yieldCb();
      // Collect SG stats for diagnostics (no stall detection)
      { uint16_t sg = getSampleSG();
        sampleSGSum += sg; sampleSGCount++;
        if (sg < sampleSGMin) sampleSGMin = sg;
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
    if (abortCb && abortCb()) {
      Serial.println("Sample pump aborted by user");
      timedOut = true;
      break;
    }
    if (millis() - startTime > timeout) {
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
    esp_task_wdt_reset();
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
    int rampLen = rampStepCount(speedUs);

    int decelStart = totalSteps - rampLen;
    if (decelStart < rampLen) decelStart = totalSteps / 2;

    int stepsDone = 0;

    // Acceleration
    float acc = startUs;
    while (acc > speedUs && stepsDone < decelStart) {
      stepPulse(STEP_PIN2, acc);
      acc *= MOTOR_ACCEL_FACTOR;
      stepsDone++;
      if (stepsDone % STEPS_PER_REVOLUTION == 0) esp_task_wdt_reset();
    }

    // Record actual speed reached (may not have hit target if ramp > totalSteps/2)
    float speedReached = acc;

    // Clear latched DIAG after acceleration
    // DIAG clearing removed — stall detection disabled

    // Constant speed
    while (stepsDone < decelStart) {
      stepPulse(STEP_PIN2, speedUs);
      stepsDone++;
      speedReached = speedUs;
      // Feed watchdog every revolution to stay well within WDT timeout
      if (stepsDone % STEPS_PER_REVOLUTION == 0) esp_task_wdt_reset();

      if (stepsDone % (MOTOR_STEPS_PER_UNIT * MOTOR_YIELD_INTERVAL * 50) == 0) {
        if (yieldCb) yieldCb();
        // Collect SG stats for diagnostics (no stall detection)
        { uint16_t sg = getTitrateSG();
          titrateSGSum += sg; titrateSGCount++;
          if (sg < titrateSGMin) titrateSGMin = sg;
        }
        if (millis() - startTime > TITRATION_TIMEOUT_MS) {
          Serial.println("ERROR: Titration timeout!");
          digitalWrite(EN_PIN2, HIGH);
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
    esp_task_wdt_reset();
  } else {
    // Small volume: absolute-time stepping immune to interrupt jitter.
    // No spread-spectrum here — titration motor needs precise timing for volume accuracy.
    unsigned int halfPeriod = (unsigned int)(speedUs + 0.5f);
    unsigned long t = micros();
    for (int i = 0; i < totalSteps; i++) {
      if ((i & 0x7F) == 0) esp_task_wdt_reset();  // Feed watchdog every 128 steps
      t += halfPeriod;
      digitalWrite(STEP_PIN2, HIGH);
      while ((long)(micros() - t) < 0) {}
      t += halfPeriod;
      digitalWrite(STEP_PIN2, LOW);
      while ((long)(micros() - t) < 0) {}
      // Stall detection disabled — SG unreliable with peristaltic pump in StealthChop
    }
    // Collect SG after small-volume titration (register retains last stepping value)
    if (isTMCDetected()) {
      uint16_t sg = getTitrateSG();
      titrateSGSum += sg; titrateSGCount++;
      if (sg < titrateSGMin) titrateSGMin = sg;
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
    if ((i + 1) % STEPS_PER_REVOLUTION == 0) {
      esp_task_wdt_reset();
      if (nSamples < maxSamples) {
        samples[nSamples].sg = getSampleSG();
        samples[nSamples].diag = digitalRead(DIAG_SAMPLE);
        nSamples++;
      }
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
    if ((i + 1) % STEPS_PER_REVOLUTION == 0) {
      esp_task_wdt_reset();
      if (nSamples < maxSamples) {
        samples[nSamples].sg = getTitrateSG();
        samples[nSamples].diag = digitalRead(DIAG_TITRATE);
        nSamples++;
      }
    }
  }

  delay(MOTOR_HOLD_MS);
  digitalWrite(EN_PIN2, HIGH);
  return nSamples;
}

// Helper: compute median and IQR from an array of SG values
static void computeMedianIQR(uint16_t* vals, int n, uint16_t* median, uint16_t* iqr) {
  if (n == 0) { *median = 0; *iqr = 0; return; }
  // Insertion sort (small arrays)
  for (int i = 1; i < n; i++) {
    uint16_t key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) { vals[j + 1] = vals[j]; j--; }
    vals[j + 1] = key;
  }
  *median = vals[n / 2];
  *iqr = (n >= 4) ? (vals[n * 3 / 4] - vals[n / 4]) : vals[n - 1] - vals[0];
}

static const int RAMP_SGTHRS_BACKSTOP = 0;  // Disabled — rely on software stall criteria during ramp

// Stall speed ramp: ramp sample pump from startRPM to maxRPM in stepRPM increments
// Returns RPM at which stall detected, or 0.0 if no stall within range
float diagStallRamp(float startRPM, float maxRPM, float stepRPM, int revsPerStep,
                    SGSample* samples, int maxSamples, int* totalSamples,
                    bool dirForward, StallRampCallback rpmCb) {
  if (!isTMCDetected()) { *totalSamples = 0; return 0.0f; }

  int nSamples = 0;

  // Set conservative SGTHRS as hardware backstop (DIAG fires at SG < 2*BACKSTOP = 10)
  setSampleSGTHRS(RAMP_SGTHRS_BACKSTOP);
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
      esp_task_wdt_reset();
    }
    delay(50);
  }

  // Per-step median tracking for stall detection
  int stepCount = 0;
  float medianAvg = 0;     // Running average of per-step medians
  int medianAvgCount = 0;
  uint16_t prevIQR = 0;
  int stallConfirmCount = 0;

  for (float rpm = startRPM; rpm <= maxRPM; rpm += stepRPM) {
    if (rpmCb) rpmCb(rpm);
    float halfPeriodUs = rpmToHalfPeriodUs(rpm);

    // Collect per-revolution SG values for this speed step
    uint16_t stepSGs[16];  // max revsPerStep
    int stepSGCount = 0;

    for (int rev = 0; rev < revsPerStep; rev++) {
      for (int step = 0; step < STEPS_PER_REVOLUTION; step++) {
        stepPulseJittered(STEP_PIN1, halfPeriodUs);
      }
      esp_task_wdt_reset();

      if (nSamples < maxSamples) {
        uint16_t sg = getSampleSG();
        samples[nSamples].sg = sg;
        samples[nSamples].diag = false;
        Serial.printf("SampleRamp: %.0f RPM  SG=%d  medAvg=%.0f\n",
                       rpm, sg, medianAvg);
        nSamples++;
        if (stepSGCount < 16) stepSGs[stepSGCount++] = sg;
      }
    }

    // Log once per speed step via WebSocket
    {
      char buf[80];
      snprintf(buf, sizeof(buf), "S %.0f RPM SG=%d..%d medAvg=%.0f",
               rpm, stepSGCount > 0 ? stepSGs[0] : 0,
               stepSGCount > 0 ? stepSGs[stepSGCount - 1] : 0, medianAvg);
      publishMessage(buf);
    }

    // Compute per-step stats
    uint16_t stepMedian = 0, stepIQR = 0;
    computeMedianIQR(stepSGs, stepSGCount, &stepMedian, &stepIQR);

    // Stall detection: track peak median and look for sustained SG drop.
    // In StealthChop, stalled motors don't go to SG=0 — they drop to a floor (~158)
    // and oscillate wildly. Use step MIN (not median) to catch intermittent stalls
    // where motor skips steps but partially recovers between revolutions.
    uint16_t stepMin = stepSGCount > 0 ? stepSGs[0] : 0;  // sorted by computeMedianIQR

    if (stepCount >= 6) {
      bool stallStep = false;
      const char* reason = "";

      // Step min < 80% of running average — catches both full and intermittent stalls.
      // 3 consecutive steps required prevents false triggers from sporadic 158 glitches.
      if (medianAvg > 50 && stepMin < (uint16_t)(medianAvg * 0.80f)) {
        stallStep = true; reason = "SG drop";
      }

      if (stallStep) {
        stallConfirmCount++;
        // Require 5 consecutive bad steps — transient dips (peristaltic roller variation)
        // recover within 3-4 steps, real stalls persist indefinitely
        if (stallConfirmCount >= 5) {
          Serial.printf("SampleRamp: STALL at %.0f RPM  min=%d median=%d IQR=%d (%s)\n",
                         rpm, stepMin, stepMedian, stepIQR, reason);
          { char buf[96];
            snprintf(buf, sizeof(buf), "Stall at %.0f RPM: %s (min=%d median=%d peakAvg=%.0f)",
                     rpm, reason, stepMin, stepMedian, medianAvg);
            publishMessage(buf);
          }
          *totalSamples = nSamples;
          delay(MOTOR_HOLD_MS);
          digitalWrite(EN_PIN1, HIGH);
          enableSampleStallGuard();
          return rpm;
        }
      } else {
        stallConfirmCount = 0;
      }

      // Update running average — only when SG is healthy (not during stall candidate steps)
      if (!stallStep) {
        if (medianAvgCount == 0) {
          medianAvg = stepMedian;
        } else {
          medianAvg = medianAvg * 0.85f + stepMedian * 0.15f;
        }
        medianAvgCount++;
      }
    } else if (stepCount == 5) {
      // Initialize median average from the settling steps
      medianAvg = stepMedian;
      medianAvgCount = 1;
    }

    prevIQR = stepIQR;
    stepCount++;
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

  // Set conservative SGTHRS as hardware backstop
  setTitrateSGTHRS(RAMP_SGTHRS_BACKSTOP);
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
      esp_task_wdt_reset();
    }
    delay(50);
  }

  // Per-step median tracking for stall detection
  int stepCount = 0;
  float medianAvg = 0;
  int medianAvgCount = 0;
  uint16_t prevIQR = 0;
  int stallConfirmCount = 0;

  for (float rpm = startRPM; rpm <= maxRPM; rpm += stepRPM) {
    if (rpmCb) rpmCb(rpm);
    float halfPeriodUs = rpmToHalfPeriodUs(rpm);

    uint16_t stepSGs[16];
    int stepSGCount = 0;

    for (int rev = 0; rev < revsPerStep; rev++) {
      for (int step = 0; step < STEPS_PER_REVOLUTION; step++) {
        stepPulse(STEP_PIN2, halfPeriodUs);
      }
      esp_task_wdt_reset();
      if (nSamples < maxSamples) {
        uint16_t sg = getTitrateSG();
        samples[nSamples].sg = sg;
        samples[nSamples].diag = false;
        Serial.printf("TitrateRamp: %.0f RPM  SG=%d  medAvg=%.0f\n",
                       rpm, sg, medianAvg);
        nSamples++;
        if (stepSGCount < 16) stepSGs[stepSGCount++] = sg;
      }
    }

    {
      char buf[80];
      snprintf(buf, sizeof(buf), "T %.0f RPM SG=%d..%d medAvg=%.0f",
               rpm, stepSGCount > 0 ? stepSGs[0] : 0,
               stepSGCount > 0 ? stepSGs[stepSGCount - 1] : 0, medianAvg);
      publishMessage(buf);
    }

    uint16_t stepMedian = 0, stepIQR = 0;
    computeMedianIQR(stepSGs, stepSGCount, &stepMedian, &stepIQR);
    uint16_t stepMin = stepSGCount > 0 ? stepSGs[0] : 0;  // sorted by computeMedianIQR

    if (stepCount >= 6) {
      bool stallStep = false;
      const char* reason = "";

      if (medianAvg > 50 && stepMin < (uint16_t)(medianAvg * 0.80f)) {
        stallStep = true; reason = "SG drop";
      }

      if (stallStep) {
        stallConfirmCount++;
        if (stallConfirmCount >= 5) {
          Serial.printf("TitrateRamp: STALL at %.0f RPM  min=%d median=%d IQR=%d (%s)\n",
                         rpm, stepMin, stepMedian, stepIQR, reason);
          { char buf[96];
            snprintf(buf, sizeof(buf), "Stall at %.0f RPM: %s (min=%d median=%d peakAvg=%.0f)",
                     rpm, reason, stepMin, stepMedian, medianAvg);
            publishMessage(buf);
          }
          *totalSamples = nSamples;
          delay(MOTOR_HOLD_MS);
          digitalWrite(EN_PIN2, HIGH);
          enableTitrateStallGuard();
          return rpm;
        }
      } else {
        stallConfirmCount = 0;
      }

      if (!stallStep) {
        if (medianAvgCount == 0) {
          medianAvg = stepMedian;
        } else {
          medianAvg = medianAvg * 0.85f + stepMedian * 0.15f;
        }
        medianAvgCount++;
      }
    } else if (stepCount == 5) {
      medianAvg = stepMedian;
      medianAvgCount = 1;
    }

    prevIQR = stepIQR;
    stepCount++;
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
