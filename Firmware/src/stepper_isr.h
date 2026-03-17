#ifndef STEPPER_ISR_H
#define STEPPER_ISR_H

#include <stdint.h>

// Initialise hardware timer (call once in setup())
void     stepISR_init();

// Start background stepping.  The ISR handles accel → cruise → decel
// automatically using the same geometric ramp as the old software loop.
//   stepPin           – GPIO number for STEP output
//   startHalfPeriodUs – half-period at ramp start (slow speed)
//   targetHalfPeriodUs– half-period at cruise (fast speed)
//   totalSteps        – exact number of step pulses to generate
//   accelFactor       – per-step multiplier for acceleration (e.g. 0.999)
void     stepISR_start(uint8_t stepPin, float startHalfPeriodUs,
                       float targetHalfPeriodUs, int32_t totalSteps,
                       float accelFactor);

// Emergency stop – disables timer immediately, no deceleration.
void     stepISR_stop();

// Returns true while the ISR is still generating pulses.
bool     stepISR_running();

// Number of step pulses completed so far (for progress reporting).
int32_t  stepISR_stepsDone();

#endif // STEPPER_ISR_H
