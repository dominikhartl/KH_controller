#include <Arduino.h>
#include "stepper_isr.h"

// ---------- ISR state (all volatile, accessed from ISR + main) ----------

static hw_timer_t* stepTimer = nullptr;

static volatile uint32_t pinMask;          // Bit mask for the step GPIO
static volatile int32_t  stepsRemaining;   // Counts down to 0
static volatile int32_t  stepsDone;        // Counts up (for progress)
static volatile bool     active;           // True while ISR is running

// Acceleration state machine
enum Phase : uint8_t { ACCEL, CRUISE, DECEL, DONE };
static volatile Phase    phase;
static volatile float    currentUs;        // Current half-period (us)
static volatile float    targetUs;         // Cruise half-period
static volatile float    accelFact;        // Multiplier (< 1 for accel)
static volatile float    decelFact;        // Divider   (> 1 for decel, = 1/accelFact)
static volatile int32_t  decelStart;       // Step count at which decel begins
static volatile int32_t  totalStepCount;   // Total steps requested
static volatile float    startUs;          // Half-period at ramp start (for decel clamp)

// ---------- ISR ----------

void IRAM_ATTR onStepTimer() {
  if (!active) return;

  // Generate one step pulse (HIGH → brief hold → LOW)
  GPIO.out_w1ts = pinMask;                 // Set HIGH
  // TMC2209 minimum pulse width is 100 ns; 8 NOPs ≈ 200 ns at 240 MHz
  __asm__ volatile("nop; nop; nop; nop; nop; nop; nop; nop");
  GPIO.out_w1tc = pinMask;                 // Set LOW

  stepsDone++;
  stepsRemaining--;

  if (stepsRemaining <= 0) {
    active = false;
    timerAlarmDisable(stepTimer);
    return;
  }

  // Update period based on current phase
  int32_t done = stepsDone;

  switch (phase) {
    case ACCEL:
      currentUs *= accelFact;              // Speed up (period decreases)
      if (currentUs <= targetUs) {
        currentUs = targetUs;
        phase = CRUISE;
      } else if (done >= decelStart) {
        phase = DECEL;                     // Short move: skip cruise
      }
      timerAlarmWrite(stepTimer, (uint64_t)(currentUs + 0.5f), true);
      break;

    case CRUISE:
      if (done >= decelStart) {
        phase = DECEL;
      }
      // Period unchanged during cruise — no timerAlarmWrite needed
      break;

    case DECEL:
      currentUs *= decelFact;              // Slow down (period increases)
      if (currentUs > startUs) currentUs = startUs;
      timerAlarmWrite(stepTimer, (uint64_t)(currentUs + 0.5f), true);
      break;

    case DONE:
      break;
  }
}

// ---------- Public API ----------

void stepISR_init() {
  // Timer 1, prescaler 80 → 1 MHz (1 us ticks), count up
  stepTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
}

void stepISR_start(uint8_t stepPin, float startHalfPeriodUs,
                   float targetHalfPeriodUs, int32_t totalSteps,
                   float accelFactor) {
  // Stop any previous run
  stepISR_stop();

  // Pre-compute ramp length (same algorithm as rampStepCount in motors.cpp)
  int rampLen = 0;
  {
    float acc = startHalfPeriodUs;
    while (acc > targetHalfPeriodUs) {
      acc *= accelFactor;
      rampLen++;
    }
  }

  int32_t ds = totalSteps - rampLen;
  if (ds < rampLen) ds = totalSteps / 2;

  // Set ISR state before enabling
  pinMask        = (uint32_t)1 << stepPin;
  stepsRemaining = totalSteps;
  stepsDone      = 0;
  totalStepCount = totalSteps;
  decelStart     = ds;
  currentUs      = startHalfPeriodUs;
  targetUs       = targetHalfPeriodUs;
  startUs        = startHalfPeriodUs;
  accelFact      = accelFactor;
  decelFact      = 1.0f / accelFactor;

  // If ramp is 0 (start already at target), go straight to cruise
  if (rampLen == 0) {
    phase     = CRUISE;
    currentUs = targetHalfPeriodUs;
  } else {
    phase = ACCEL;
  }

  // Configure and start timer
  timerAlarmWrite(stepTimer, (uint64_t)(currentUs + 0.5f), true);
  timerWrite(stepTimer, 0);
  active = true;
  timerAlarmEnable(stepTimer);
}

void stepISR_stop() {
  if (stepTimer) {
    timerAlarmDisable(stepTimer);
  }
  active = false;
}

bool stepISR_running() {
  return active;
}

int32_t stepISR_stepsDone() {
  return stepsDone;
}
