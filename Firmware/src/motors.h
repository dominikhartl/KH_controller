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
bool washSampleVol(int removeRevs, int fillRevs, float speedRpm); // Absolute revolution counts
bool titrate(int volume, float speedRpm, bool noAccel = false, uint32_t accelOverride = 0);

// Per-operation SG stats for tube wear tracking
void getLastSampleSGStats(uint16_t* avg, uint16_t* min);
void getLastTitrateSGStats(uint16_t* avg, uint16_t* min);

// Motor diagnostic: run revolutions and collect SG samples
// Returns number of samples collected (0 if TMC not detected)
struct SGSample { uint16_t sg; bool diag; };
int diagStepSample(int revolutions, float rpm, SGSample* samples, int maxSamples);
int diagStepTitrate(int revolutions, float rpm, SGSample* samples, int maxSamples);

// Stall speed ramp: run sample pump at increasing RPM until stall detected
// Returns RPM at which stall occurred (0.0 if no stall within range)
// dirForward: true = forward (DIR HIGH), false = reverse (DIR LOW)
// Optional rpmCallback is called at each speed step with the current RPM
typedef void (*StallRampCallback)(float rpm);
float diagStallRamp(float startRPM, float maxRPM, float stepRPM, int revsPerStep,
                    SGSample* samples, int maxSamples, int* totalSamples,
                    bool dirForward = true, StallRampCallback rpmCb = nullptr);

// Stall speed ramp for titration pump (same interface, uses EN_PIN2/STEP_PIN2/DIR_PIN2)
float diagStallRampTitrate(float startRPM, float maxRPM, float stepRPM, int revsPerStep,
                    SGSample* samples, int maxSamples, int* totalSamples,
                    bool dirForward = true, StallRampCallback rpmCb = nullptr);

// Crash hint: RTC memory that survives a panic reset — log on next boot to identify crash location
const char* getMotorCrashHint();  // returns hint string, or nullptr if none
void clearMotorCrashHint();

#endif // MOTORS_H
