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
bool removeSample(int volume);
bool takeSample(int volume);
bool washSample(float remPart, float fillPart);
bool titrate(int volume, float speedRpm, bool noAccel = false);

#endif // MOTORS_H
