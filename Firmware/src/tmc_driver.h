#ifndef TMC_DRIVER_H
#define TMC_DRIVER_H

#include <stdint.h>

bool initTMCDrivers();
bool isTMCDetected();
void reinitSampleDriver();  // Reset StealthChop autotune; call before each sample-pump op
void setSampleSpreadCycle(bool enable);
void setTitrateSpreadCycle(bool enable);
void printTMCDebug();

// Diagnostic register accessors (read-only, for hardware diagnostics)
uint32_t getSampleDrvStatus();
uint32_t getTitrateDrvStatus();
uint32_t getSampleIOIN();
uint32_t getTitrateIOIN();

#endif // TMC_DRIVER_H
