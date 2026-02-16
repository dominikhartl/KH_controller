#ifndef MOTORS_H
#define MOTORS_H

// Yield callback called periodically during long motor operations
// to keep MQTT, OTA, WebSocket alive
typedef void (*MotorYieldCallback)();
void setMotorYieldCallback(MotorYieldCallback cb);

// Progress callback called during washSample with completion percentage (0-100)
typedef void (*MotorProgressCallback)(int percent);
void setMotorProgressCallback(MotorProgressCallback cb);

void removeSample(int volume);
void takeSample(int volume);
void washSample(float remPart, float fillPart);
void titrate(int volume, int stepDelay);

#endif // MOTORS_H
