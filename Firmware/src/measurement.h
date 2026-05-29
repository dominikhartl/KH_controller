#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include "gran_analysis.h"

// Measurement state - defined in measurement.cpp
extern float voltage;
extern float pH;
extern float voltage_4PH;
extern float voltage_7PH;
extern float voltage_10PH;

// Initialize ADC with calibrated attenuation
void initADC();

// Initialize external pH sensor — ADS1115 or EZO (call after configStore.begin())
void initExternalADC();
bool isExternalADCActive();    // True if ADS1115 is being used for readings
bool isExternalADCFallback();  // True if configured for ADS1115 but fell back to internal

// Atlas Scientific EZO pH circuit
bool isEZOActive();              // True if EZO is being used for pH readings
bool isEZOAvailable();           // True if EZO hardware was detected (regardless of config)
uint8_t getEZOCalPoints();       // 0-3: number of EZO calibration points
float getEZOAcidSlope();         // Acid slope % from SLOPE,? (ideal ~99.7%)
float getEZOBaseSlope();         // Base slope % from SLOPE,?
bool ezoCalibrate(const char* point, float pH);  // Send calibration command
void ezoQueryCalStatus();        // Refresh cal points and slope from device
float ezoReadPH(float tempC);    // Single pH reading with temperature compensation
const char* getActivePHSensor(); // "Internal" / "ADS1115" / "EZO pH"
const char* getEZOInitLog();     // Deferred init log (for web UI display after boot)

// Recompute linear fit coefficients from current calibration voltages
void updateCalibrationFit();

// Update Nernst temperature correction factor for measurement temperature
void updateNernstTempCorrection(float measTempC);

// Check if calibration voltages are valid (not NaN, sufficient separation)
bool isCalibrationValid();

// Wait for ADC readings to converge (call after acid addition + mixing delay)
void waitForPHStabilization();

// Register a callback invoked periodically during stabilization waits
// to keep UI/MQTT responsive. Called approximately every 500ms.
void setStabilizationYieldCallback(void (*cb)());

// Measurement functions
void measurePH(int nreadings);
void measurePHStabilized(int nreadings);  // Like measurePH but skips internal stabilization
void measurePHFast(int nreadings);  // No stabilization, 8x oversample — for far from endpoint
float measureVoltage(int nreadings);

// Stabilization configuration
void setStabilizationTimeoutMs(int ms);  // Override default timeout (call before measurement)

// Stabilization statistics (reset per measurement cycle)
void resetStabilizationStats();
int getStabilizationTimeoutCount();       // Number of times stabilization timed out
unsigned long getTotalStabilizationMs();  // Cumulative stabilization time
bool getLastStabilizationTimedOut();      // Whether last stabilization call timed out

// Noise statistics (reset per measurement cycle)
void resetNoiseStats();
void setStabNoiseCaptureEnabled(bool e); // Capture probe noise only at start-pH (settled solution)
float getLastStabNoiseMv();              // StdDev of mV readings during last stabilization
float getAvgStabNoiseMv();               // Average noise StdDev across all stabilizations
float getMaxStabNoiseMv();               // Peak noise StdDev across all stabilizations
int getHighNoiseCount();                 // Count of stabilizations with noise > PROBE_NOISE_GOOD_MV

// Probe health metrics
unsigned long getLastStabilizationMs();  // Last stabilization time in ms
float getProbeSlope();                   // Average conditioned mV/pH from calibration
float getAcidSlope();                    // Conditioned mV/pH for pH 4→7 segment
float getAlkalineSlope();                // Conditioned mV/pH for pH 7→10 segment
float getAcidEfficiency();               // Nernst efficiency % for acid segment
float getAlkalineEfficiency();           // Nernst efficiency % for alkaline segment
float getProbeAsymmetry();               // % difference between acid/base slopes
const char* getProbeHealth();            // "Good", "Fair", or "Replace"
const char* getProbeHealthDetail(char* reasonBuf, size_t reasonLen);  // Same, with reason string

// Reset EMA filter state (call when switching measurement contexts)
void resetADCFilter();

// Diagnostic ADC reads (for hardware diagnostics module)
// Read ADS1115 with configurable MUX and data rate (returns raw 16-bit value, or INT16_MIN on error)
// muxBits: 100=AIN0/GND (pH), 101=AIN1/GND (baseline), etc.
// drBits:  000=8SPS, 001=16SPS, 010=32SPS, 011=64SPS, 100=128SPS, 101=250SPS, 110=475SPS, 111=860SPS
int16_t readADS1115RawDiag(uint8_t muxBits, uint8_t drBits);

// Read ESP32 internal ADC in millivolts (single sample, no oversampling)
float readInternalADCmV();

// Check if ADS1115 hardware is available (regardless of config)
bool isADS1115Available();

#endif // MEASUREMENT_H
