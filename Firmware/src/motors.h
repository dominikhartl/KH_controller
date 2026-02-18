#ifndef MOTORS_H
#define MOTORS_H

// Yield callback called periodically during long motor operations
// to keep MQTT, OTA, WebSocket alive
typedef void (*MotorYieldCallback)();
void setMotorYieldCallback(MotorYieldCallback cb);

// Progress callback called during washSample with completion percentage (0-100)
typedef void (*MotorProgressCallback)(int percent);
void setMotorProgressCallback(MotorProgressCallback cb);

// All motor functions return true on success, false on timeout
bool removeSample(int volume);
bool takeSample(int volume);
bool washSample(float remPart, float fillPart);
bool titrate(int volume, float speedRpm);

#endif // MOTORS_H
