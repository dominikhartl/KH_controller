#ifndef MOTORS_H
#define MOTORS_H

// Yield callback called periodically during long motor operations
// to keep MQTT, OTA, WebSocket alive
typedef void (*MotorYieldCallback)();
void setMotorYieldCallback(MotorYieldCallback cb);

// Progress callback called during washSample with completion percentage (0-100)
typedef void (*MotorProgressCallback)(int percent);
void setMotorProgressCallback(MotorProgressCallback cb);

// Multi-wash progress: tracks overall progress across sequential washSample() calls
// Call before a sequence of washes, then clearMultiWashContext() after
void setMultiWashContext(int numWashes);
void clearMultiWashContext();

// All motor functions return true on success, false on timeout
bool removeSample(int volume, float speedRpm);
bool takeSample(int volume, float speedRpm);
bool washSample(float remPart, float fillPart, float speedRpm);
bool titrate(int volume, float speedRpm, bool noAccel = false);

// Per-operation SG stats for tube wear tracking
void getLastSampleSGStats(uint16_t* avg, uint16_t* min);
void getLastTitrateSGStats(uint16_t* avg, uint16_t* min);

// Motor diagnostic: run revolutions and collect SG samples
// Returns number of samples collected (0 if TMC not detected)
struct SGSample { uint16_t sg; bool diag; };
int diagStepSample(int revolutions, float rpm, SGSample* samples, int maxSamples);
int diagStepTitrate(int revolutions, float rpm, SGSample* samples, int maxSamples);

#endif // MOTORS_H
