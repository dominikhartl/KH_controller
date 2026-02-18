#ifndef MEASUREMENT_H
#define MEASUREMENT_H

// Measurement state - defined in measurement.cpp
extern float voltage;
extern float pH;
extern float voltage_4PH;
extern float voltage_7PH;
extern float voltage_10PH;

// Initialize ADC with calibrated attenuation
void initADC();

// Recompute linear fit coefficients from current calibration voltages
void updateCalibrationFit();

// Check if calibration voltages are valid (not NaN, sufficient separation)
bool isCalibrationValid();

// Wait for ADC readings to converge (call after acid addition + mixing delay)
void waitForPHStabilization();

// Measurement functions
void measurePH(int nreadings);
void measurePHFast(int nreadings);  // No stabilization, 8x oversample — for far from endpoint
float measureVoltage(int nreadings);

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

// Gran transformation endpoint detection
struct TitrationPoint {
  float units;
  float pH;
};

// Determine equivalence point via Gran function linearization
// Returns equivalence units, or NAN on failure. outR2 receives R² of fit.
// If reasonBuf is non-null, writes failure reason on NAN return.
float granAnalysis(TitrationPoint* points, int nPoints,
                   float sampleVol, float titVol, float calUnits,
                   float* outR2, char* reasonBuf = nullptr, size_t reasonLen = 0);

// Fallback: linear interpolation to find where pH crosses targetPH
float interpolateAtPH(TitrationPoint* points, int nPoints, float targetPH);

#endif // MEASUREMENT_H
