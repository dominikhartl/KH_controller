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
void resetSampleStallGuard();
void resetTitrateStallGuard();
void clearSampleDIAG();
void clearTitrateDIAG();
void disableSampleStallGuard();
void enableSampleStallGuard();
void disableTitrateStallGuard();
void enableTitrateStallGuard();
void applyStallGuardConfig();  // Call after configStore.begin() to set SGTHRS from NVS
void setSampleSGTHRS(uint8_t val);
void setTitrateSGTHRS(uint8_t val);
void printTMCDebug();

// Diagnostic register accessors (read-only, for hardware diagnostics)
uint32_t getSampleDrvStatus();
uint32_t getTitrateDrvStatus();
uint32_t getSampleIOIN();
uint32_t getTitrateIOIN();

#endif // TMC_DRIVER_H
