#include <Arduino.h>
#include <esp_task_wdt.h>
#include <FastAccelStepper.h>
#include "motors.h"
#include "mqtt_manager.h"
#include "tmc_driver.h"
#include <pins.h>
#include <config.h>

extern void publishMessage(const char* message);

// KHpro diagnostic globals defined in vendored FastAccelStepper's
// StepperISR_esp32.cpp — used to surface FAS-task health to the dashboard.
extern volatile int khpro_fas_task_core;
extern volatile int khpro_fas_task_prio;
extern volatile uint32_t khpro_fas_max_gap_us;
extern volatile uint32_t khpro_fas_gap_count;
extern volatile uint32_t khpro_fas_cycle_count;

// RTC memory survives a panic reset — used to identify which motor operation crashed
RTC_NOINIT_ATTR char motorCrashHint[48];
RTC_NOINIT_ATTR uint32_t motorCrashMagic;
static const uint32_t CRASH_MAGIC = 0xFEEDC0DE;

static void setCrashHint(const char* hint) {
  strncpy(motorCrashHint, hint, sizeof(motorCrashHint) - 1);
  motorCrashHint[sizeof(motorCrashHint) - 1] = '\0';
  motorCrashMagic = CRASH_MAGIC;
}

const char* getMotorCrashHint() {
  return (motorCrashMagic == CRASH_MAGIC) ? motorCrashHint : nullptr;
}

void clearMotorCrashHint() { motorCrashMagic = 0; }

static FastAccelStepperEngine* pStepperEngine = nullptr;
static FastAccelStepper* sampleStepper = nullptr;
static FastAccelStepper* titrateStepper = nullptr;

static inline uint32_t rpmToHz(float rpm) {
  return (uint32_t)(rpm * STEPS_PER_REVOLUTION / 60.0f + 0.5f);
}

static MotorYieldCallback yieldCb = nullptr;
static MotorProgressCallback progressCb = nullptr;
static MotorAbortCallback abortCb = nullptr;

// Blocking delay that yields to WiFi/MQTT/OTA, preventing connection starvation
static void yieldingDelay(unsigned long ms) {
  unsigned long end = millis() + ms;
  while (millis() < end) {
    if (yieldCb) yieldCb();
    delay(10);
  }
}

// Wash progress tracking (shared between removeSample/takeSample when called from washSampleVol)
static int washTotalVol = 0;
static int washBaseVol = 0;  // volume completed before current phase

// Multi-wash context: spans progress across sequential washSampleVol() calls
static int multiWashTotal = 0;   // total number of washes in sequence (0 = disabled)
static int multiWashIndex = 0;   // current wash index (0-based)

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

// Track last direction for backlash compensation
static bool lastSampleDirection = true;

void initMotors() {
  pStepperEngine = new FastAccelStepperEngine();
  // Pin StepperTask to Core 1 (same core as loopTask) so FAS at MAX priority
  // deterministically preempts the motor wait loop and the yield callback —
  // unpinned default lets FreeRTOS migrate it to Core 0, where AsyncTCP/WiFi
  // can starve it and drain the FAS command queue → silent RMT underrun.
  pStepperEngine->init(1);

  sampleStepper = pStepperEngine->stepperConnectToPin(STEP_PIN1);
  sampleStepper->setDirectionPin(DIR_PIN1);
  sampleStepper->setSpeedInHz(rpmToHz(MOTOR_TARGET_RPM));
  sampleStepper->setAcceleration(MOTOR_ACCEL_STEPS_S2);

  titrateStepper = pStepperEngine->stepperConnectToPin(STEP_PIN2);
  titrateStepper->setDirectionPin(DIR_PIN2);
  titrateStepper->setSpeedInHz(rpmToHz(TITRATION_RPM));
  titrateStepper->setAcceleration(MOTOR_ACCEL_STEPS_S2);
}

// Wait for stepper to finish, feeding watchdog and servicing callbacks every 50ms.
// Returns false if aborted or timed out.
static bool waitForStepper(FastAccelStepper* stepper, unsigned long timeoutMs,
                            int32_t startPos, bool trackProgress = false) {
  unsigned long startTime = millis();
  int lastPct = -1;
  while (stepper->isRunning()) {
    esp_task_wdt_reset();
    if (yieldCb) yieldCb();

    if (trackProgress && progressCb && washTotalVol > 0) {
      int32_t stepsDone = abs(stepper->getCurrentPosition() - startPos);
      int revsDone = (int)(stepsDone / STEPS_PER_REVOLUTION);
      int done = washBaseVol + revsDone;
      int singlePct = (done * 100) / washTotalVol;
      int pct = (multiWashTotal > 0) ? (multiWashIndex * 100 + singlePct) / multiWashTotal : singlePct;
      if (pct != lastPct) { lastPct = pct; progressCb(pct); }
    }

    if (abortCb && abortCb()) {
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
      return false;
    }
    if (millis() - startTime > timeoutMs) {
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
      return false;
    }
    delay(50);
  }
  return true;
}

// Shared sample pump logic — direction is the only difference between remove and fill
// Returns false on timeout/abort
static bool runSamplePump(int volume, bool forward, float speedRpm) {
  { char hint[48]; snprintf(hint, sizeof(hint), "%s %d revs",
      forward ? "takeSample" : "removeSample", volume);
    setCrashHint(hint); }
  mqttManager.setMotorMode(true);  // Prevent blocking TCP connects during motor ops

  digitalWrite(EN_PIN1, LOW);
  delay(MOTOR_ENABLE_DELAY_MS);

  // Diagnostic: TMC DRV_STATUS before pump starts (rules in/out chip-side stall/overload)
  uint32_t drvStatusPre = isTMCDetected() ? getSampleDrvStatus() : 0;

  // Reset FAS-task health counters so the post-pump message reflects only this run
  khpro_fas_max_gap_us = 0;
  khpro_fas_gap_count = 0;
  uint32_t fasCyclesAtStart = khpro_fas_cycle_count;

  // Backlash compensation on direction reversal
  if (forward != lastSampleDirection) {
    sampleStepper->setSpeedInHz(rpmToHz(MOTOR_START_RPM));
    sampleStepper->setAcceleration(MOTOR_ACCEL_STEPS_S2);
    sampleStepper->move(forward ? BACKLASH_COMPENSATION_STEPS : -BACKLASH_COMPENSATION_STEPS);
    while (sampleStepper->isRunning()) { esp_task_wdt_reset(); if (yieldCb) yieldCb(); delay(5); }
  }
  lastSampleDirection = forward;

  int totalSteps = volume * STEPS_PER_REVOLUTION;
  sampleStepper->setSpeedInHz(rpmToHz(speedRpm));
  sampleStepper->setAcceleration(MOTOR_ACCEL_STEPS_S2);
  int32_t startPos = sampleStepper->getCurrentPosition();
  sampleStepper->move(forward ? totalSteps : -totalSteps);

  // Dynamic timeout: 3× expected time or minimum SAMPLE_PUMP_TIMEOUT_MS, whichever is greater
  unsigned long expectedMs = (unsigned long)((float)volume / speedRpm * 60000.0f);
  unsigned long timeout = max((unsigned long)SAMPLE_PUMP_TIMEOUT_MS, expectedMs * 3UL);

  unsigned long startTime = millis();
  int lastPct = -1;
  while (sampleStepper->isRunning()) {
    esp_task_wdt_reset();
    if (yieldCb) yieldCb();
    if (abortCb && abortCb()) {
      sampleStepper->forceStopAndNewPosition(sampleStepper->getCurrentPosition());
      yieldingDelay(MOTOR_HOLD_MS);
      digitalWrite(EN_PIN1, HIGH);
      mqttManager.setMotorMode(false);
      return false;
    }
    if (millis() - startTime > timeout) {
      sampleStepper->forceStopAndNewPosition(sampleStepper->getCurrentPosition());
      yieldingDelay(MOTOR_HOLD_MS);
      digitalWrite(EN_PIN1, HIGH);
      mqttManager.setMotorMode(false);
      return false;
    }
    if (progressCb && washTotalVol > 0) {
      int32_t stepsDone = abs(sampleStepper->getCurrentPosition() - startPos);
      int revsDone = (int)(stepsDone / STEPS_PER_REVOLUTION);
      int done = washBaseVol + revsDone;
      int singlePct = (done * 100) / washTotalVol;
      int pct = (multiWashTotal > 0) ? (multiWashIndex * 100 + singlePct) / multiWashTotal : singlePct;
      if (pct != lastPct) { lastPct = pct; progressCb(pct); }
    }
    delay(50);
  }
  int32_t endPos = sampleStepper->getCurrentPosition();
  int32_t actualSteps = abs(endPos - startPos);
  bool ok = ((int)actualSteps >= totalSteps - 2);

  unsigned long durationMs = millis() - startTime;

  // Diagnostic: TMC DRV_STATUS after pump completes (compare against drvStatusPre)
  uint32_t drvStatusPost = isTMCDetected() ? getSampleDrvStatus() : 0;

  uint32_t fasCycles = khpro_fas_cycle_count - fasCyclesAtStart;
  uint32_t fasMaxGapUs = khpro_fas_max_gap_us;
  uint32_t fasGapCount = khpro_fas_gap_count;
  int fasCore = khpro_fas_task_core;
  int fasPrio = khpro_fas_task_prio;

  Serial.printf("SamplePump %s: %d revs @ %.0f RPM | steps exp=%d act=%d delta=%d | dur=%lums exp=%lums | DRV pre=0x%08X post=0x%08X | FAS core=%d prio=%d cycles=%u max_gap=%u us gaps>8ms=%u\n",
                forward ? "FILL" : "REMOVE", volume, speedRpm,
                totalSteps, (int)actualSteps, (int)actualSteps - totalSteps,
                durationMs, expectedMs, drvStatusPre, drvStatusPost,
                fasCore, fasPrio, (unsigned)fasCycles,
                (unsigned)fasMaxGapUs, (unsigned)fasGapCount);

  // Mirror the diagnostic to MQTT so it shows up in the dashboard activity log
  {
    char dbuf[220];
    snprintf(dbuf, sizeof(dbuf),
             "%s %dr@%.0frpm dur=%lu/%lums delta=%d DRV=0x%08X->0x%08X FAS:c%d p%d cyc=%u maxgap=%uus n=%u",
             forward ? "FILL" : "REM", volume, speedRpm,
             durationMs, expectedMs, (int)actualSteps - totalSteps,
             drvStatusPre, drvStatusPost,
             fasCore, fasPrio, (unsigned)fasCycles,
             (unsigned)fasMaxGapUs, (unsigned)fasGapCount);
    publishMessage(dbuf);
  }

  yieldingDelay(MOTOR_HOLD_MS);
  digitalWrite(EN_PIN1, HIGH);
  mqttManager.setMotorMode(false);
  return ok;
}

bool removeSample(int volume, float speedRpm) {
  return runSamplePump(volume, false, speedRpm);
}

bool takeSample(int volume, float speedRpm) {
  return runSamplePump(volume, true, speedRpm);
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

bool titrate(int volume, float speedRpm, bool noAccel, uint32_t accelOverride) {
  { char hint[48]; snprintf(hint, sizeof(hint), "titrate %d units", volume);
    setCrashHint(hint); }
  // Only add enable settle delay if motor wasn't already on
  if (digitalRead(EN_PIN2) != LOW) {
    digitalWrite(EN_PIN2, LOW);
    delay(MOTOR_ENABLE_DELAY_MS);
  }

  int totalSteps = volume * MOTOR_STEPS_PER_UNIT;
  titrateStepper->setSpeedInHz(rpmToHz(speedRpm));
  // Small volumes and noAccel: instant acceleration (hardware stepping still precise)
  if (accelOverride > 0) {
    titrateStepper->setAcceleration(accelOverride);
  } else if (noAccel || volume <= TITRATE_ACCEL_THRESHOLD) {
    titrateStepper->setAcceleration(100000);
  } else {
    titrateStepper->setAcceleration(MOTOR_ACCEL_STEPS_S2);
  }
  int32_t startPos = titrateStepper->getCurrentPosition();
  titrateStepper->move(-totalSteps);  // DIR_PIN2=LOW direction = negative move

  bool ok = waitForStepper(titrateStepper, TITRATION_TIMEOUT_MS, startPos);

  if (!ok) {
    Serial.println("ERROR: Titration stopped (timeout or abort)!");
    // Emergency disable — caller normally manages EN_PIN2 but we stop here
    digitalWrite(EN_PIN2, HIGH);
    return false;
  }

  // No hold/disable here — caller manages EN_PIN2 to avoid
  // enable/disable overhead on every small titration step
  return true;
}

// Motor ramp test: run motor at increasing speeds to check for mechanical issues.
// User listens for abnormal sounds and can abort via the UI.
bool motorRampTest(bool isSample, float startRPM, float maxRPM, float stepRPM,
                   int revsPerStep, RampProgressCallback rpmCb, float* stoppedAtRPM,
                   uint32_t accel) {
  uint8_t enPin = isSample ? EN_PIN1 : EN_PIN2;
  FastAccelStepper* stepper = isSample ? sampleStepper : titrateStepper;

  *stoppedAtRPM = 0;

  mqttManager.setMotorMode(true);
  digitalWrite(enPin, LOW);
  delay(MOTOR_ENABLE_DELAY_MS);

  for (float rpm = startRPM; rpm <= maxRPM; rpm += stepRPM) {
    if (rpmCb) rpmCb(rpm);

    stepper->setSpeedInHz(rpmToHz(rpm));
    stepper->setAcceleration(accel > 0 ? accel : MOTOR_ACCEL_STEPS_S2);
    int32_t startPos = stepper->getCurrentPosition();
    int totalSteps = revsPerStep * STEPS_PER_REVOLUTION;
    stepper->move(-totalSteps);  // removal direction (both pumps wired so negative = removal)

    // Wait for this step to complete
    unsigned long timeout = (unsigned long)((float)revsPerStep / rpm * 60000.0f * 3.0f) + 5000UL;
    unsigned long startTime = millis();
    while (stepper->isRunning()) {
      esp_task_wdt_reset();
      if (yieldCb) yieldCb();
      if (abortCb && abortCb()) {
        stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
        *stoppedAtRPM = rpm;
        yieldingDelay(MOTOR_HOLD_MS);
        digitalWrite(enPin, HIGH);
        mqttManager.setMotorMode(false);
        return false;
      }
      if (millis() - startTime > timeout) {
        stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
        *stoppedAtRPM = rpm;
        yieldingDelay(MOTOR_HOLD_MS);
        digitalWrite(enPin, HIGH);
        mqttManager.setMotorMode(false);
        return false;
      }
      delay(50);
    }

    *stoppedAtRPM = rpm;
  }

  yieldingDelay(MOTOR_HOLD_MS);
  digitalWrite(enPin, HIGH);
  mqttManager.setMotorMode(false);
  return true;
}
