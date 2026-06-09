#include "gran_analysis.h"
#include "config_store.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// Temperature coefficients for commercial calibration buffers (pH/°C):
//   pH 4: +0.001  (phthalate, very stable)
//   pH 7: -0.002  (phosphate)
//   pH 10: -0.010 (carbonate/borate)
static const float BUF4_TEMPCO  =  0.001f;
static const float BUF7_TEMPCO  = -0.002f;
static const float BUF10_TEMPCO = -0.010f;

// User-configurable buffer pH values are stored at 25°C (as printed on bottle).
// bufferPHAtTemp() applies temperature compensation at runtime.
float bufferPHAtTemp(int nominal, float tempC) {
  float dT = tempC - 25.0f;
  switch (nominal) {
    case 4:  return configStore.getBufferPH4()  + BUF4_TEMPCO  * dT;
    case 7:  return configStore.getBufferPH7()  + BUF7_TEMPCO  * dT;
    case 10: return configStore.getBufferPH10() + BUF10_TEMPCO * dT;
    default: return (float)nominal;
  }
}

// --- Gran transformation endpoint detection ---

// Returns true if a point may participate in the Gran regression: not excluded,
// inside the pH window, and measured in the Gran zone (phase 2 — full mix delay
// plus stabilization). Fast/medium-phase points (phase 0/1) are unstabilized
// (±0.3 pH fast mode) and would silently degrade the fit if fast_ph is
// configured low enough to push them into the regression window.
static inline bool granUsable(const TitrationPoint& p, const bool* excluded, int i,
                              float pHLow, float pHHigh) {
  if (excluded && excluded[i]) return false;
  if (p.phase != TITRATION_PHASE_GRAN) return false;
  return p.pH < pHHigh && p.pH > pHLow;
}

// OLS regression on Gran function values within a pH window.
// OLS (w=1) is the standard practice for Gran analysis (USGS, Dickson SOP 3b, textbooks).
// Previous experiments with w=1/y² and w=1/|y| showed no precision benefit and introduced
// pathological weight sensitivity near the equivalence point.
// Internally accumulates in double on mean-centered x: raw float sums of x²
// (x up to ~16k units) suffer catastrophic cancellation in n·Σx²−(Σx)².
// excluded[] marks points to skip; returns false if regression fails.
bool granRegression(TitrationPoint* points, int nPoints,
                    float sampleVol, float k, bool* excluded,
                    float pHLow, float pHHigh,
                    float* outSlope, float* outIntercept,
                    float* outR2, float* outSsRes, int* outCount,
                    float* outVarSlope,
                    float* outVarIntercept,
                    float* outCovSI) {
  // Pass 1: means
  double sumX = 0, sumY = 0;
  int count = 0;
  for (int i = 0; i < nPoints; i++) {
    if (!granUsable(points[i], excluded, i, pHLow, pHHigh)) continue;
    double x = points[i].units;
    double y = (sampleVol + x * k) * pow(10.0, -(double)points[i].pH);
    sumX += x;
    sumY += y;
    count++;
  }

  if (count < MIN_GRAN_POINTS) return false;
  double meanX = sumX / count;
  double meanY = sumY / count;

  // Pass 2: centered second moments
  double sxx = 0, sxy = 0, syy = 0;
  for (int i = 0; i < nPoints; i++) {
    if (!granUsable(points[i], excluded, i, pHLow, pHHigh)) continue;
    double dx = (double)points[i].units - meanX;
    double dy = (sampleVol + (double)points[i].units * k) * pow(10.0, -(double)points[i].pH) - meanY;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }

  if (sxx < 1e-12) return false;

  double slope = sxy / sxx;
  double intercept = meanY - slope * meanX;
  double ssRes = syy - slope * sxy;  // residual SS of the OLS fit
  if (ssRes < 0) ssRes = 0;          // guard against rounding

  *outSlope = (float)slope;
  *outIntercept = (float)intercept;
  *outCount = count;
  *outR2 = (syy > 1e-12) ? (float)(1.0 - ssRes / syy) : 0.0f;
  *outSsRes = (float)ssRes;

  // Parameter variances for confidence interval computation
  if (count > 2 && (outVarSlope || outVarIntercept || outCovSI)) {
    double s2 = ssRes / (double)(count - 2);
    if (outVarSlope) *outVarSlope = (float)(s2 / sxx);
    if (outVarIntercept) *outVarIntercept = (float)(s2 * (1.0 / count + meanX * meanX / sxx));
    if (outCovSI) *outCovSI = (float)(-meanX * s2 / sxx);
  }
  return true;
}

// Try Gran analysis with a specific pH window, including outlier removal.
// Returns equivalence point in units, or NAN on failure.
float tryGranWindow(TitrationPoint* points, int nPoints,
                    float sampleVol, float k,
                    float pHLow, float pHHigh,
                    float* outR2,
                    float* outSlope,
                    float* outIntercept,
                    float* outEqSE) {
  bool excluded[MAX_TITRATION_POINTS];
  for (int i = 0; i < nPoints; i++) excluded[i] = false;

  float slope, intercept, r2, ssRes;
  float varSlope = 0, varIntercept = 0, covSI = 0;
  int count;

  if (!granRegression(points, nPoints, sampleVol, k, excluded,
                      pHLow, pHHigh,
                      &slope, &intercept, &r2, &ssRes, &count,
                      &varSlope, &varIntercept, &covSI))
    return NAN;

  // Iterative outlier rejection: up to 2 rounds, remove worst 2σ weighted outlier
  for (int round = 0; round < 2; round++) {
    if (count <= MIN_GRAN_POINTS) break;

    float sigma = sqrtf(ssRes / (float)(count - 2));
    float worstRes = 0;
    int worstIdx = -1;

    for (int i = 0; i < nPoints; i++) {
      if (points[i].phase != TITRATION_PHASE_GRAN) continue;
      if (excluded[i]) continue;
      if (points[i].pH < pHHigh && points[i].pH > pHLow) {
        float x = points[i].units;
        float totalVol = sampleVol + x * k;
        float y = totalVol * powf(10.0f, -points[i].pH);
        float w = 1.0f;
        float wRes = sqrtf(w) * fabsf(y - (slope * x + intercept));
        if (wRes > 2.0f * sigma && wRes > worstRes) {
          worstRes = wRes;
          worstIdx = i;
        }
      }
    }
    if (worstIdx < 0) break;

    excluded[worstIdx] = true;
    if (!granRegression(points, nPoints, sampleVol, k, excluded,
                        pHLow, pHHigh,
                        &slope, &intercept, &r2, &ssRes, &count,
                        &varSlope, &varIntercept, &covSI))
      return NAN;
  }

  if (slope <= 0) return NAN;

  float eqUnits = -intercept / slope;
  if (eqUnits < 0 || eqUnits > points[nPoints - 1].units) return NAN;

  if (outR2) *outR2 = r2;
  if (outSlope) *outSlope = slope;
  if (outIntercept) *outIntercept = intercept;

  // Compute standard error of equivalence point via error propagation
  if (outEqSE) {
    float s2inv = 1.0f / (slope * slope);
    float varEq = s2inv * varIntercept
                + eqUnits * eqUnits * s2inv * varSlope
                + 2.0f * eqUnits * s2inv * covSI;
    *outEqSE = (varEq > 0) ? sqrtf(varEq) : 0;
  }
  return eqUnits;
}

float granAnalysis(TitrationPoint* points, int nPoints,
                   float sampleVol, float titVol, float calUnits,
                   float* outR2,
                   float* outWinLow, float* outWinHigh,
                   char* reasonBuf, size_t reasonLen,
                   float* outSlope, float* outIntercept,
                   GranWindowResult* windowResults, int* nWindowResults,
                   float* outEqUnitsSE) {
  auto fail = [&](const char* reason) -> float {
    if (reasonBuf && reasonLen > 0) snprintf(reasonBuf, reasonLen, "%s", reason);
    return NAN;
  };

  if (nPoints < 3) return fail("Too few data points");
  if (calUnits <= 0) return fail("Invalid calibration units");

  // Sanity check: require at least MIN_GRAN_POINTS in the Gran region. Without
  // this, a probe-failure or aborted titration that produced a handful of low-pH
  // points could pass through to regression and yield a false-confidence result.
  // Only Gran-zone (phase 2) points count — fast/medium points are unstabilized
  // and are excluded from the regression as well.
  int granRegionCount = 0;
  for (int i = 0; i < nPoints; i++) {
    if (points[i].phase != TITRATION_PHASE_GRAN) continue;
    if (points[i].pH < GRAN_REGION_PH && points[i].pH > GRAN_STOP_PH - 0.5f) granRegionCount++;
  }
  if (granRegionCount < MIN_GRAN_POINTS) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Only %d Gran-region points (need %d)", granRegionCount, MIN_GRAN_POINTS);
    return fail(buf);
  }

  float k = titVol / calUnits;

  // Adaptive window selection: try multiple pH bounds, select by minimum eqSE (best precision)
  static const float upperBounds[] = {4.5f, 4.4f, 4.3f, 4.2f, 4.1f};
  static const int nBounds = sizeof(upperBounds) / sizeof(upperBounds[0]);

  // Compute data-adaptive lower bound: 10th percentile pH of Gran-region points
  // to trim noisy extreme-low-pH points
  float adaptiveLow = GRAN_STOP_PH;
  {
    int granCount = 0;
    float minPH = 99.0f;
    for (int i = 0; i < nPoints; i++) {
      if (points[i].phase != TITRATION_PHASE_GRAN) continue;
      if (points[i].pH < GRAN_REGION_PH && points[i].pH > GRAN_STOP_PH - 0.5f) {
        granCount++;
        if (points[i].pH < minPH) minPH = points[i].pH;
      }
    }
    if (granCount >= 10) {
      // Use 10th percentile: skip lowest 10% of points
      int skipCount = granCount / 10;
      if (skipCount > 0) {
        // Find the (skipCount)-th lowest pH
        float sorted[MAX_TITRATION_POINTS];
        int n = 0;
        for (int i = 0; i < nPoints; i++) {
          if (points[i].phase != TITRATION_PHASE_GRAN) continue;
          if (points[i].pH < GRAN_REGION_PH && points[i].pH > GRAN_STOP_PH - 0.5f) {
            sorted[n++] = points[i].pH;
          }
        }
        // Simple insertion sort (small array)
        for (int i = 1; i < n; i++) {
          float val = sorted[i];
          int j = i - 1;
          while (j >= 0 && sorted[j] > val) { sorted[j+1] = sorted[j]; j--; }
          sorted[j+1] = val;
        }
        adaptiveLow = sorted[skipCount];  // 10th percentile pH
      }
    }
  }

  // Try lower bounds in 0.1 pH steps from GRAN_STOP_PH up to adaptive low
  static const int MAX_LOWER = 6;
  float lowerBounds[MAX_LOWER];
  int nLower = 0;
  for (float lb = GRAN_STOP_PH; lb <= adaptiveLow + 0.01f && nLower < MAX_LOWER; lb += 0.1f) {
    lowerBounds[nLower++] = lb;
  }
  if (nLower == 0) lowerBounds[nLower++] = GRAN_STOP_PH;

  float bestR2 = 0;
  float bestEqUnits = NAN;
  float bestLow = GRAN_STOP_PH, bestHigh = 0;
  float bestSlope = 0, bestIntercept = 0;
  float bestEqSE = 0;
  int winCount = 0;

  for (int lb = 0; lb < nLower; lb++) {
    for (int b = 0; b < nBounds; b++) {
      if (upperBounds[b] <= lowerBounds[lb]) continue;
      if (upperBounds[b] - lowerBounds[lb] < 0.15f) continue;
      float r2 = 0, s = 0, ic = 0, eqSE = 0;
      float eq = tryGranWindow(points, nPoints, sampleVol, k,
                               lowerBounds[lb], upperBounds[b], &r2, &s, &ic, &eqSE);
      if (windowResults && winCount < MAX_GRAN_WINDOWS) {
        windowResults[winCount] = {lowerBounds[lb], upperBounds[b], r2, !isnan(eq), eq, eqSE};
        winCount++;
      }
      // Select window by minimum eqSE (best precision) among windows passing the R² threshold.
      // Maximising R² is not the right criterion — curved early points can maintain high R² while
      // inflating residual variance and therefore widening the CI. Min-eqSE directly optimises
      // the quantity reported to the user.
      // Fallback: if eqSE is unavailable (degenerate variance), use R² as tiebreaker.
      bool better = false;
      if (!isnan(eq) && r2 >= configStore.getGranMinR2()) {
        if (eqSE > 0)
          better = isnan(bestEqUnits) || eqSE < bestEqSE;
        else
          better = isnan(bestEqUnits) || (bestEqSE == 0 && r2 > bestR2);
      }
      if (better) {
        bestR2 = r2;
        bestEqUnits = eq;
        bestLow = lowerBounds[lb];
        bestHigh = upperBounds[b];
        bestSlope = s;
        bestIntercept = ic;
        bestEqSE = eqSE;
      }
    }
  }
  if (nWindowResults) *nWindowResults = winCount;

  if (isnan(bestEqUnits)) return fail("No valid Gran window found");

  if (outWinLow) *outWinLow = bestLow;
  if (outWinHigh) *outWinHigh = bestHigh;
  if (outSlope) *outSlope = bestSlope;
  if (outIntercept) *outIntercept = bestIntercept;
  if (outEqUnitsSE) *outEqUnitsSE = bestEqSE;

  if (outR2) {
    *outR2 = bestR2;
    if (bestR2 < configStore.getGranMinR2()) {
      if (reasonBuf && reasonLen > 0)
        snprintf(reasonBuf, reasonLen, "Poor fit (R²=%.3f)", bestR2);
      return NAN;
    }
  }

  return bestEqUnits;
}

float interpolateAtPH(TitrationPoint* points, int nPoints, float targetPH) {
  // Find two consecutive points that bracket the target pH (pH decreasing)
  for (int i = 1; i < nPoints; i++) {
    if (points[i - 1].pH > targetPH && points[i].pH <= targetPH) {
      float prevPH = points[i - 1].pH;
      float currPH = points[i].pH;
      if (fabsf(prevPH - currPH) < 1e-6f) continue;
      float fraction = (prevPH - targetPH) / (prevPH - currPH);
      return points[i - 1].units + fraction * (points[i].units - points[i - 1].units);
    }
  }
  return NAN;  // No crossing found
}
