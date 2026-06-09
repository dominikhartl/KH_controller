#ifndef MOTORS_H
#define MOTORS_H

// Yield callback called periodically during long motor operations
// to keep MQTT, OTA, WebSocket alive
typedef void (*MotorYieldCallback)();
void setMotorYieldCallback(MotorYieldCallback cb);

// Abort check callback — returns true if measurement should be aborted
typedef bool (*MotorAbortCallback)();
void setMotorAbortCallback(MotorAbortCallback cb);

// Progress callback called during wash operations with completion percentage (0-100)
typedef void (*MotorProgressCallback)(int percent);
void setMotorProgressCallback(MotorProgressCallback cb);

// Multi-wash progress: tracks overall progress across sequential washSampleVol() calls
// Call before a sequence of washes, then clearMultiWashContext() after
void setMultiWashContext(int numWashes);
void clearMultiWashContext();

// Initialize FastAccelStepper engine and stepper objects — call once during setup
void initMotors();

// All motor functions return true on success, false on timeout
bool removeSample(int volume, float speedRpm);
bool takeSample(int volume, float speedRpm);
// Wash cycle (absolute revolution counts). scavengeRevs > 0 inserts a stir
// pulse + short second removal between remove and fill (residual consistency).
bool washSampleVol(int removeRevs, int fillRevs, float speedRpm, int scavengeRevs = 0);
bool titrate(int volume, float speedRpm, bool noAccel = false, uint32_t accelOverride = 0);

// Small reverse move of the titration pump (anti-drip). Caller must keep
// EN_PIN2 enabled until this returns.
void titrationAntiSuckback(int steps);

// Motor ramp test: run motor at increasing speeds from startRPM to maxRPM
// Returns true if completed full range, false if aborted
// rpmCb is called at each speed step with the current RPM
// *stoppedAtRPM is set to the last completed RPM
typedef void (*RampProgressCallback)(float rpm);
bool motorRampTest(bool isSample, float startRPM, float maxRPM, float stepRPM,
                   int revsPerStep, RampProgressCallback rpmCb, float* stoppedAtRPM,
                   uint32_t accel = 0);

// Crash hint: RTC memory that survives a panic reset — log on next boot to identify crash location
const char* getMotorCrashHint();  // returns hint string, or nullptr if none
void clearMotorCrashHint();

#endif // MOTORS_H
