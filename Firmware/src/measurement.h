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

// Initialize ADS1115 external ADC (call after configStore.begin())
void initExternalADC();
bool isExternalADCActive();    // True if ADS1115 is being used for readings
bool isExternalADCFallback();  // True if configured for ADS1115 but fell back to internal

// Recompute linear fit coefficients from current calibration voltages
void updateCalibrationFit();

// Temperature-compensated buffer pH (IUPAC/NIST standard coefficients)
float bufferPHAtTemp(int nominal, float tempC);

// Check if calibration voltages are valid (not NaN, sufficient separation)
bool isCalibrationValid();

// Wait for ADC readings to converge (call after acid addition + mixing delay)
void waitForPHStabilization();

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

// Gran transformation endpoint detection
struct TitrationPoint {
  float units;
  float pH;
  float mV;           // raw ADC voltage
  uint16_t stabMs;    // stabilization time in ms (0 = no stab wait)
  uint8_t phase;      // 0=fast, 1=medium, 2=gran
  uint8_t flags;      // bit 0: stabTimedOut
};

// Gran window search result for R² distribution plot
#define MAX_GRAN_WINDOWS 12
struct GranWindowResult {
    float low, high, r2;
    bool valid;
    float eqUnits;  // equivalence point in stepper units (NAN if invalid)
};

// Determine equivalence point via Gran function linearization
// Returns equivalence units, or NAN on failure. outR2 receives R² of fit.
// outWinLow/outWinHigh receive the selected pH window bounds.
// If reasonBuf is non-null, writes failure reason on NAN return.
float granAnalysis(TitrationPoint* points, int nPoints,
                   float sampleVol, float titVol, float calUnits,
                   float* outR2,
                   float* outWinLow = nullptr, float* outWinHigh = nullptr,
                   char* reasonBuf = nullptr, size_t reasonLen = 0,
                   float* outSlope = nullptr, float* outIntercept = nullptr,
                   GranWindowResult* windowResults = nullptr, int* nWindowResults = nullptr);

// Fallback: linear interpolation to find where pH crosses targetPH
float interpolateAtPH(TitrationPoint* points, int nPoints, float targetPH);

#endif // MEASUREMENT_H
