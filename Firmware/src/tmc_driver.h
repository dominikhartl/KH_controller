#ifndef TMC_DRIVER_H
#define TMC_DRIVER_H

#include <stdint.h>

bool initTMCDrivers();
bool isTMCDetected();
bool isSampleStalled();
bool isTitrateStalled();
bool wasMotorStall();
void setStallFlag();
void clearStallFlag();
uint16_t getSampleSG();
uint16_t getTitrateSG();
void setSampleSpreadCycle(bool enable);
void setTitrateSpreadCycle(bool enable);
void printTMCDebug();

#endif // TMC_DRIVER_H
