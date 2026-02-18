#include <Arduino.h>
#include "measurement.h"
#include <pins.h>
#include <config.h>
#include <math.h>
#include "driver/adc.h"

// Measurement state
float voltage = 0;
float pH = 0;
float voltage_4PH = 0;
float voltage_7PH = 0;
float voltage_10PH = 0;

// Cached linear fit coefficients: pH = phSlope * voltage + phOffset
static float phSlope = 0;
static float phOffset = 7.0;

// Response time tracking
static unsigned long lastStabilizationMs = 0;

// Configurable stabilization timeout
static int stabilizationTimeoutMs = STABILIZATION_TIMEOUT_MS;

// Stabilization statistics (per measurement cycle)
static int stabTimeoutCount = 0;
static unsigned long stabTotalMs = 0;

void setStabilizationTimeoutMs(int ms) { stabilizationTimeoutMs = ms; }
void resetStabilizationStats() { stabTimeoutCount = 0; stabTotalMs = 0; }
int getStabilizationTimeoutCount() { return stabTimeoutCount; }
unsigned long getTotalStabilizationMs() { return stabTotalMs; }

// Sort an array of floats in ascending order (bubble sort — sufficient for small N)
static void sortFloats(float* arr, int count) {
  for (int i = 0; i < count - 1; i++) {
    for (int j = 0; j < count - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

// Compute the mean of sorted values within outlierThreshold of the median
static float medianFilteredMean(float* sorted, int count, float outlierThreshold) {
  float median;
  if (count % 2 == 0) {
    median = (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0f;
  } else {
    median = sorted[count / 2];
  }

  float sum = 0.0f;
  int inlierCount = 0;
  for (int i = 0; i < count; i++) {
    if (fabsf(sorted[i] - median) <= outlierThreshold) {
      sum += sorted[i];
      inlierCount++;
    }
  }
  return (inlierCount > 0) ? sum / inlierCount : median;
}

// Oversampled ADC read with trimmed mean to reject WiFi-induced noise spikes.
// Sorts raw samples and discards top/bottom 25% before averaging.
static float readADCTrimmed(int nSamples, int interSampleDelayMs) {
  static float adcBuf[ADC_OVERSAMPLING];
  if (nSamples > ADC_OVERSAMPLING) nSamples = ADC_OVERSAMPLING;
  for (int i = 0; i < nSamples; i++) {
    adcBuf[i] = (float)analogReadMilliVolts(PH_PIN);
    delay(interSampleDelayMs);
  }
  sortFloats(adcBuf, nSamples);
  int trim = nSamples / 4;
  float sum = 0;
  for (int i = trim; i < nSamples - trim; i++) {
    sum += adcBuf[i];
  }
  return sum / (nSamples - 2 * trim);
}

void initADC() {
  analogSetPinAttenuation(PH_PIN, ADC_11db);
  adc_power_acquire();  // Keep ADC powered to prevent hall sensor glitches on GPIO35
  analogReadMilliVolts(PH_PIN);  // Dummy read to prime SAR ADC
}

void updateCalibrationFit() {
  float v[3] = {voltage_4PH, voltage_7PH, voltage_10PH};
  float p[3] = {4.0f, 7.0f, 10.0f};
  float sumV = v[0] + v[1] + v[2];
  float sumP = 21.0f;
  float sumVV = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
  float sumVP = v[0]*p[0] + v[1]*p[1] + v[2]*p[2];
  float denom = 3.0f * sumVV - sumV * sumV;
  if (fabs(denom) < 1e-6f) {
    // Fallback: approximate Nernst slope at 25C (~-59mV/pH ≈ -1/17 pH/mV)
    phSlope = -1.0f / 173.0f;
    phOffset = 7.0f + voltage_7PH / 173.0f;
    return;
  }
  phSlope = (3.0f * sumVP - sumV * sumP) / denom;
  phOffset = (sumP - phSlope * sumV) / 3.0f;
}

bool isCalibrationValid() {
  if (isnan(voltage_4PH) || isnan(voltage_7PH) || isnan(voltage_10PH)) return false;
  if (voltage_4PH <= 0 || voltage_7PH <= 0 || voltage_10PH <= 0) return false;
  if (fabs(voltage_4PH - voltage_7PH) < 50.0f) return false;
  if (fabs(voltage_7PH - voltage_10PH) < 50.0f) return false;
  return true;
}

// --- Probe health metrics ---

unsigned long getLastStabilizationMs() {
  return lastStabilizationMs;
}

float getProbeSlope() {
  if (!isCalibrationValid()) return NAN;
  float slopeAcid = (voltage_7PH - voltage_4PH) / 3.0f;
  float slopeBase = (voltage_10PH - voltage_7PH) / 3.0f;
  return (slopeAcid + slopeBase) / 2.0f;
}

float getAcidSlope() {
  if (!isCalibrationValid()) return NAN;
  return (voltage_7PH - voltage_4PH) / 3.0f;  // Conditioned mV/pH for pH 4→7
}

float getAlkalineSlope() {
  if (!isCalibrationValid()) return NAN;
  return (voltage_10PH - voltage_7PH) / 3.0f;  // Conditioned mV/pH for pH 7→10
}

// Nernst efficiency: raw probe slope vs theoretical at measurement temperature
// raw_slope = conditioned_slope / amplifier_gain
// efficiency = |raw_slope| / nernst_at_temp * 100%
static float slopeToEfficiency(float conditionedSlope) {
  if (isnan(conditionedSlope)) return NAN;
  float nernst = NERNST_FACTOR * (273.15f + MEASUREMENT_TEMP_C);
  float rawSlope = fabsf(conditionedSlope) / PH_AMP_GAIN;
  return (rawSlope / nernst) * 100.0f;
}

float getAcidEfficiency() {
  return slopeToEfficiency(getAcidSlope());
}

float getAlkalineEfficiency() {
  return slopeToEfficiency(getAlkalineSlope());
}

float getProbeAsymmetry() {
  if (!isCalibrationValid()) return NAN;
  float slopeAcid = fabsf((voltage_7PH - voltage_4PH) / 3.0f);
  float slopeBase = fabsf((voltage_10PH - voltage_7PH) / 3.0f);
  float avg = (slopeAcid + slopeBase) / 2.0f;
  if (avg < 1.0f) return NAN;
  return fabsf(slopeAcid - slopeBase) / avg * 100.0f;
}

const char* getProbeHealth() {
  return getProbeHealthDetail(nullptr, 0);
}

const char* getProbeHealthDetail(char* reasonBuf, size_t reasonLen) {
  float acidEff = getAcidEfficiency();
  if (isnan(acidEff)) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "no calibration data");
    return "Unknown";
  }

  // Primary check: Nernst efficiency of acid slope (measurement range)
  if (acidEff < PROBE_EFFICIENCY_FAIR) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "acid slope efficiency %.0f%% (need >%.0f%%)", acidEff, PROBE_EFFICIENCY_FAIR);
    return "Replace";
  }
  if (acidEff < PROBE_EFFICIENCY_GOOD) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "acid slope efficiency %.0f%% (need >%.0f%%)", acidEff, PROBE_EFFICIENCY_GOOD);
    return "Fair";
  }

  // Secondary: asymmetry and response time
  float asym = getProbeAsymmetry();
  if (!isnan(asym) && asym > PROBE_ASYMMETRY_FAIR) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "asymmetry %.1f%% (limit %.0f%%)", asym, PROBE_ASYMMETRY_FAIR);
    return "Replace";
  }
  if (lastStabilizationMs >= PROBE_RESPONSE_FAIR_MS) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "response time %lums (limit %lums)", lastStabilizationMs, PROBE_RESPONSE_FAIR_MS);
    return "Replace";
  }
  if (lastStabilizationMs >= PROBE_RESPONSE_GOOD_MS) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "response time %lums (limit %lums)", lastStabilizationMs, PROBE_RESPONSE_GOOD_MS);
    return "Fair";
  }
  if (!isnan(asym) && asym > PROBE_ASYMMETRY_GOOD) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "asymmetry %.1f%% (limit %.0f%%)", asym, PROBE_ASYMMETRY_GOOD);
    return "Fair";
  }

  if (reasonBuf && reasonLen > 0) reasonBuf[0] = '\0';
  return "Good";
}

// Wait for ADC readings to converge instead of fixed delay
static void waitForStabilization() {
  analogReadMilliVolts(PH_PIN);  // Dummy read to prime ADC after idle
  delayMicroseconds(100);
  float prev = (float)analogReadMilliVolts(PH_PIN);
  delay(50);
  unsigned long start = millis();
  while (millis() - start < (unsigned long)stabilizationTimeoutMs) {
    float curr = (float)analogReadMilliVolts(PH_PIN);
    if (fabs(curr - prev) < STABILIZATION_THRESHOLD_MV) {
      unsigned long elapsed = millis() - start;
      lastStabilizationMs = elapsed;
      stabTotalMs += elapsed;
      return;
    }
    prev = curr;
    delay(50);
  }
  lastStabilizationMs = stabilizationTimeoutMs;  // Timed out
  stabTotalMs += stabilizationTimeoutMs;
  stabTimeoutCount++;
}

void waitForPHStabilization() {
  waitForStabilization();
}

// Core pH reading logic shared by measurePH and measurePHStabilized
static void measurePHCore(int nreadings) {
  static float pHReadings[100];
  const int maxReadings = (nreadings > 100) ? 100 : nreadings;
  int validReadings = 0;

  for (int t = 0; t < maxReadings; t++) {
    voltage = readADCTrimmed(ADC_OVERSAMPLING, ADC_INTER_SAMPLE_DELAY_MS);

    float calculatedPH = phSlope * voltage + phOffset;

    if (!isnan(calculatedPH) && calculatedPH > 0.0 && calculatedPH < 14.0) {
      pHReadings[validReadings] = calculatedPH;
      validReadings++;
    }

    delay(MEASUREMENT_DELAY_MS);
  }

  if (validReadings == 0) {
    pH = NAN;
    return;
  }

  sortFloats(pHReadings, validReadings);
  pH = medianFilteredMean(pHReadings, validReadings, PH_OUTLIER_THRESHOLD);
}

// pH measurement with adaptive stabilization, oversampling, and outlier removal
void measurePH(int nreadings) {
  waitForStabilization();
  measurePHCore(nreadings);
}

// pH measurement WITHOUT internal stabilization — use when caller has already
// waited for mixing + called waitForPHStabilization() before this call
void measurePHStabilized(int nreadings) {
  measurePHCore(nreadings);
}

// Fast pH measurement — no stabilization, reduced oversampling
// For use far from endpoint where ±0.3 pH accuracy is sufficient
void measurePHFast(int nreadings) {
  static float pHReadings[100];
  const int maxReadings = (nreadings > 100) ? 100 : nreadings;
  int validReadings = 0;

  for (int t = 0; t < maxReadings; t++) {
    voltage = readADCTrimmed(ADC_OVERSAMPLING_FAST, ADC_INTER_SAMPLE_DELAY_FAST_MS);

    float calculatedPH = phSlope * voltage + phOffset;

    if (!isnan(calculatedPH) && calculatedPH > 0.0 && calculatedPH < 14.0) {
      pHReadings[validReadings] = calculatedPH;
      validReadings++;
    }

    delay(MEASUREMENT_DELAY_FAST_MS);
  }

  if (validReadings == 0) {
    pH = NAN;
    return;
  }

  sortFloats(pHReadings, validReadings);
  pH = medianFilteredMean(pHReadings, validReadings, PH_FAST_OUTLIER_THRESHOLD);
}

// Voltage measurement with adaptive stabilization and oversampling
float measureVoltage(int nreadings) {
  waitForStabilization();

  static float voltageReadings[100];
  const int maxReadings = (nreadings > 100) ? 100 : nreadings;

  for (int t = 0; t < maxReadings; t++) {
    voltageReadings[t] = readADCTrimmed(ADC_OVERSAMPLING, ADC_INTER_SAMPLE_DELAY_MS);
    delay(MEASUREMENT_DELAY_MS);
  }

  sortFloats(voltageReadings, maxReadings);

  // Calculate mean of middle 80% (discard extreme values)
  int startIdx = maxReadings / 10;
  int endIdx = maxReadings - maxReadings / 10;
  if (endIdx <= startIdx) {
    endIdx = maxReadings;
    startIdx = 0;
  }

  float sum = 0.0;
  for (int i = startIdx; i < endIdx; i++) {
    sum += voltageReadings[i];
  }

  float avgVoltage = sum / (endIdx - startIdx);
  voltage = avgVoltage;
  return avgVoltage;
}

// --- Gran transformation endpoint detection ---

// Internal: linear regression on Gran function values within a pH window
// excluded[] marks points to skip; returns false if regression fails
static bool granRegression(TitrationPoint* points, int nPoints,
                           float sampleVol, float k, bool* excluded,
                           float pHLow, float pHHigh,
                           float* outSlope, float* outIntercept,
                           float* outR2, float* outSsRes, int* outCount) {
  float sumX = 0, sumY = 0, sumXX = 0, sumXY = 0, sumYY = 0;
  int count = 0;

  for (int i = 0; i < nPoints; i++) {
    if (excluded[i]) continue;
    if (points[i].pH < pHHigh && points[i].pH > pHLow) {
      float x = points[i].units;
      float totalVol = sampleVol + x * k;
      float y = totalVol * powf(10.0f, -points[i].pH);
      sumX += x; sumY += y; sumXX += x * x; sumXY += x * y; sumYY += y * y;
      count++;
    }
  }

  if (count < MIN_GRAN_POINTS) return false;

  float denom = (float)count * sumXX - sumX * sumX;
  if (fabsf(denom) < 1e-12f) return false;

  *outSlope = ((float)count * sumXY - sumX * sumY) / denom;
  *outIntercept = (sumY - *outSlope * sumX) / (float)count;
  *outCount = count;

  // Compute R²
  float meanY = sumY / (float)count;
  float ssTot = sumYY - (float)count * meanY * meanY;
  float ssRes = 0;
  for (int i = 0; i < nPoints; i++) {
    if (excluded[i]) continue;
    if (points[i].pH < pHHigh && points[i].pH > pHLow) {
      float x = points[i].units;
      float totalVol = sampleVol + x * k;
      float y = totalVol * powf(10.0f, -points[i].pH);
      float pred = *outSlope * x + *outIntercept;
      float res = y - pred;
      ssRes += res * res;
    }
  }
  *outR2 = (ssTot > 1e-12f) ? 1.0f - ssRes / ssTot : 0.0f;
  *outSsRes = ssRes;
  return true;
}

// Try Gran analysis with a specific pH window, including outlier removal.
// Returns equivalence point in units, or NAN on failure.
static float tryGranWindow(TitrationPoint* points, int nPoints,
                           float sampleVol, float k,
                           float pHLow, float pHHigh,
                           float* outR2) {
  bool excluded[MAX_TITRATION_POINTS];
  for (int i = 0; i < nPoints; i++) excluded[i] = false;

  float slope, intercept, r2, ssRes;
  int count;

  if (!granRegression(points, nPoints, sampleVol, k, excluded,
                      pHLow, pHHigh,
                      &slope, &intercept, &r2, &ssRes, &count))
    return NAN;

  // Iterative outlier rejection: up to 2 rounds, remove worst 2σ outlier
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
        float res = fabsf(y - (slope * x + intercept));
        if (res > 2.0f * sigma && res > worstRes) {
          worstRes = res;
          worstIdx = i;
        }
      }
    }
    if (worstIdx < 0) break;

    excluded[worstIdx] = true;
    if (!granRegression(points, nPoints, sampleVol, k, excluded,
                        pHLow, pHHigh,
                        &slope, &intercept, &r2, &ssRes, &count))
      return NAN;
  }

  if (slope <= 0) return NAN;

  float eqUnits = -intercept / slope;
  if (eqUnits < 0 || eqUnits > points[nPoints - 1].units) return NAN;

  *outR2 = r2;
  return eqUnits;
}

float granAnalysis(TitrationPoint* points, int nPoints,
                   float sampleVol, float titVol, float calUnits,
                   float* outR2, char* reasonBuf, size_t reasonLen) {
  auto fail = [&](const char* reason) -> float {
    if (reasonBuf && reasonLen > 0) snprintf(reasonBuf, reasonLen, "%s", reason);
    return NAN;
  };

  if (nPoints < 3) return fail("Too few data points");
  if (calUnits <= 0) return fail("Invalid calibration units");

  float k = titVol / calUnits;

  // Adaptive window selection: try multiple upper pH bounds, keep best R²
  static const float upperBounds[] = {4.5f, 4.3f, 4.1f, 3.9f, 3.7f};
  static const int nBounds = sizeof(upperBounds) / sizeof(upperBounds[0]);

  float bestR2 = 0;
  float bestEqUnits = NAN;

  for (int b = 0; b < nBounds; b++) {
    float r2 = 0;
    float eq = tryGranWindow(points, nPoints, sampleVol, k,
                             GRAN_STOP_PH, upperBounds[b], &r2);
    if (!isnan(eq) && r2 > bestR2) {
      bestR2 = r2;
      bestEqUnits = eq;
    }
  }

  if (isnan(bestEqUnits)) return fail("No valid Gran window found");

  if (outR2) {
    *outR2 = bestR2;
    if (bestR2 < GRAN_MIN_R2) {
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
