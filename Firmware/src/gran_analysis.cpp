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

// OLS regression on Gran function values within a pH window.
// OLS (w=1) is the standard practice for Gran analysis (USGS, Dickson SOP 3b, textbooks).
// Previous experiments with w=1/y² and w=1/|y| showed no precision benefit and introduced
// pathological weight sensitivity near the equivalence point.
// excluded[] marks points to skip; returns false if regression fails.
bool granRegression(TitrationPoint* points, int nPoints,
                    float sampleVol, float k, bool* excluded,
                    float pHLow, float pHHigh,
                    float* outSlope, float* outIntercept,
                    float* outR2, float* outSsRes, int* outCount,
                    float* outVarSlope,
                    float* outVarIntercept,
                    float* outCovSI) {
  float sumW = 0, sumWX = 0, sumWY = 0, sumWXX = 0, sumWXY = 0, sumWYY = 0;
  int count = 0;

  for (int i = 0; i < nPoints; i++) {
    if (excluded[i]) continue;
    if (points[i].pH < pHHigh && points[i].pH > pHLow) {
      float x = points[i].units;
      float totalVol = sampleVol + x * k;
      float y = totalVol * powf(10.0f, -points[i].pH);
      float w = 1.0f;  // OLS — standard practice for Gran analysis
      sumW += w;
      sumWX += w * x;
      sumWY += w * y;
      sumWXX += w * x * x;
      sumWXY += w * x * y;
      sumWYY += w * y * y;
      count++;
    }
  }

  if (count < MIN_GRAN_POINTS) return false;

  float denom = sumW * sumWXX - sumWX * sumWX;
  if (fabsf(denom) < 1e-12f) return false;

  *outSlope = (sumW * sumWXY - sumWX * sumWY) / denom;
  *outIntercept = (sumWY - *outSlope * sumWX) / sumW;
  *outCount = count;

  // Compute weighted R²
  float meanWY = sumWY / sumW;
  float ssTot = sumWYY - sumW * meanWY * meanWY;
  float ssRes = 0;
  for (int i = 0; i < nPoints; i++) {
    if (excluded[i]) continue;
    if (points[i].pH < pHHigh && points[i].pH > pHLow) {
      float x = points[i].units;
      float totalVol = sampleVol + x * k;
      float y = totalVol * powf(10.0f, -points[i].pH);
      float w = 1.0f;
      float pred = *outSlope * x + *outIntercept;
      float res = y - pred;
      ssRes += w * res * res;
    }
  }
  *outR2 = (ssTot > 1e-12f) ? 1.0f - ssRes / ssTot : 0.0f;
  *outSsRes = ssRes;

  // Parameter variances for confidence interval computation
  if (count > 2 && (outVarSlope || outVarIntercept || outCovSI)) {
    float s2 = ssRes / (float)(count - 2);
    if (outVarSlope) *outVarSlope = sumW / denom * s2;
    if (outVarIntercept) *outVarIntercept = sumWXX / denom * s2;
    if (outCovSI) *outCovSI = -sumWX / denom * s2;
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
  int granRegionCount = 0;
  for (int i = 0; i < nPoints; i++) {
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
