#ifndef GRAN_ANALYSIS_H
#define GRAN_ANALYSIS_H

#include <config.h>
#include <stdint.h>
#include <stddef.h>

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
    float eqSE;     // standard error of equivalence point (0 if invalid)
};

// Temperature-compensated buffer pH (IUPAC/NIST standard coefficients)
float bufferPHAtTemp(int nominal, float tempC);

// Maximum in-window points for interpolation array
static const int MAX_GRAN_INTERP_PTS = 50;

// OLS regression on Gran function values within a pH window.
// excluded[] marks points to skip; returns false if regression fails.
// interpSpacing > 0 inserts linearly interpolated F(V) points between measured
// data points that are spaced wider than interpSpacing, reducing drop-size bias.
bool granRegression(TitrationPoint* points, int nPoints,
                    float sampleVol, float k, bool* excluded,
                    float pHLow, float pHHigh,
                    float* outSlope, float* outIntercept,
                    float* outR2, float* outSsRes, int* outCount,
                    float* outVarSlope = nullptr,
                    float* outVarIntercept = nullptr,
                    float* outCovSI = nullptr,
                    float interpSpacing = 0.0f);

// Try Gran analysis with a specific pH window, including outlier removal.
// Returns equivalence point in units, or NAN on failure.
float tryGranWindow(TitrationPoint* points, int nPoints,
                    float sampleVol, float k,
                    float pHLow, float pHHigh,
                    float* outR2,
                    float* outSlope = nullptr,
                    float* outIntercept = nullptr,
                    float* outEqSE = nullptr,
                    float interpSpacing = 0.0f);

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
                   GranWindowResult* windowResults = nullptr, int* nWindowResults = nullptr,
                   float* outEqUnitsSE = nullptr);

// Fallback: linear interpolation to find where pH crosses targetPH
float interpolateAtPH(TitrationPoint* points, int nPoints, float targetPH);

#endif // GRAN_ANALYSIS_H
