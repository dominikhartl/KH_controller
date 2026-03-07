#include <Arduino.h>
#include <Wire.h>
#include "measurement.h"
#include "config_store.h"
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

// ADS1115 external ADC state (direct I2C, no library needed)
static bool useExternalADC = false;
static bool adsAvailable = false;
static bool adsFallback = false;

// ADS1115 register addresses
static const uint8_t ADS_REG_CONVERSION = 0x00;
static const uint8_t ADS_REG_CONFIG     = 0x01;
// Config register bits for single-shot, AIN0 vs GND, GAIN_TWO (±2.048V), 16 SPS
// [15]    OS=1 (start single conversion)
// [14:12] MUX=100 (AIN0 vs GND)
// [11:9]  PGA=001 (±4.096V, GAIN_ONE — 2061 mV headroom at pH 4)
// [8]     MODE=1 (single-shot)
// [7:5]   DR=001 (16 SPS → 62.5ms conversion)
// [4:0]   COMP disabled (00011)
static const uint16_t ADS_CONFIG_SINGLE_A0 = 0xC323;

// Piecewise linear coefficients: pH = slope * voltage + offset
// Acid segment (pH 4→7, voltage >= voltage_7PH) and base segment (pH 7→10)
static float acidSlope = 0, acidOffset = 7.0;
static float baseSlope = 0, baseOffset = 7.0;

// Convert voltage to pH using piecewise interpolation
static inline float voltageToPH(float v) {
  if (v >= voltage_7PH) {
    return acidSlope * v + acidOffset;
  } else {
    return baseSlope * v + baseOffset;
  }
}

// Response time tracking
static unsigned long lastStabilizationMs = 0;

// Configurable stabilization timeout
static int stabilizationTimeoutMs = STABILIZATION_TIMEOUT_MS;

// Stabilization statistics (per measurement cycle)
static int stabTimeoutCount = 0;
static unsigned long stabTotalMs = 0;
static bool lastStabTimedOut = false;

// Noise tracking (per measurement cycle)
static float lastStabNoiseMv = 0;      // StdDev of mV readings during last stabilization
static float stabNoiseMvSum = 0;       // Running sum of noise StdDevs
static int stabNoiseMvCount = 0;       // Number of stabilizations measured
static float stabNoiseMvMax = 0;       // Peak noise across all stabilizations
static int stabNoiseMvHighCount = 0;   // Count of stabilizations with noise > PROBE_NOISE_GOOD_MV

void setStabilizationTimeoutMs(int ms) { stabilizationTimeoutMs = ms; }
void resetStabilizationStats() { stabTimeoutCount = 0; stabTotalMs = 0; lastStabTimedOut = false; }
void resetNoiseStats() { lastStabNoiseMv = 0; stabNoiseMvSum = 0; stabNoiseMvCount = 0; stabNoiseMvMax = 0; stabNoiseMvHighCount = 0; }
int getStabilizationTimeoutCount() { return stabTimeoutCount; }
unsigned long getTotalStabilizationMs() { return stabTotalMs; }
bool getLastStabilizationTimedOut() { return lastStabTimedOut; }
float getLastStabNoiseMv() { return lastStabNoiseMv; }
float getAvgStabNoiseMv() { return (stabNoiseMvCount > 0) ? stabNoiseMvSum / stabNoiseMvCount : 0; }
float getMaxStabNoiseMv() { return stabNoiseMvMax; }
int getHighNoiseCount() { return stabNoiseMvHighCount; }

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

// ADS1115 helpers (must precede readADCTrimmed)
static bool adsActive() { return useExternalADC && adsAvailable; }
static int effectiveOversampling() { return adsActive() ? ADS_OVERSAMPLING : ADC_OVERSAMPLING; }
static int effectiveOversamplingFast() { return adsActive() ? ADS_OVERSAMPLING_FAST : ADC_OVERSAMPLING_FAST; }
static int effectiveStabSamples() { return adsActive() ? ADS_STAB_SAMPLES : 16; }
static float effectiveStabThreshold() { return adsActive() ? ADS_STABILIZATION_THRESHOLD_MV : STABILIZATION_THRESHOLD_MV; }

static float readADS1115MilliVolts() {
  // Start single-shot conversion
  Wire.beginTransmission(ADS1115_I2C_ADDR);
  Wire.write(ADS_REG_CONFIG);
  Wire.write((uint8_t)(ADS_CONFIG_SINGLE_A0 >> 8));
  Wire.write((uint8_t)(ADS_CONFIG_SINGLE_A0 & 0xFF));
  if (Wire.endTransmission() != 0) return NAN;

  // Wait for conversion (16 SPS = 62.5ms, add margin)
  delay(75);

  // Read conversion result
  Wire.beginTransmission(ADS1115_I2C_ADDR);
  Wire.write(ADS_REG_CONVERSION);
  if (Wire.endTransmission() != 0) return NAN;

  if (Wire.requestFrom((uint8_t)ADS1115_I2C_ADDR, (uint8_t)2) != 2) return NAN;
  int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();

  if (raw < 0 || raw >= 32767) return NAN;
  return (float)raw * ADS_MV_PER_BIT;
}

// Oversampled ADC read with trimmed mean to reject noise spikes.
// Sorts raw samples and discards top/bottom 25% before averaging.
static float readADCTrimmed(int nSamples, int interSampleDelayMs) {
  if (nSamples <= 0) return 0;
  static float adcBuf[ADC_OVERSAMPLING];  // 64 elements — always large enough
  int maxSamples = adsActive() ? ADS_OVERSAMPLING : ADC_OVERSAMPLING;
  if (nSamples > maxSamples) nSamples = maxSamples;

  int valid = 0;
  for (int i = 0; i < nSamples; i++) {
    float sample;
    if (adsActive()) {
      sample = readADS1115MilliVolts();
      // ADS1115 conversion time (~62ms at 16 SPS) provides inherent filtering
    } else {
      sample = (float)analogReadMilliVolts(PH_PIN);
      delay(interSampleDelayMs);
    }
    if (!isnan(sample)) {
      adcBuf[valid++] = sample;
    }
  }

  if (valid == 0) return 0;
  sortFloats(adcBuf, valid);
  int trim = valid / 4;
  float sum = 0;
  for (int i = trim; i < valid - trim; i++) {
    sum += adcBuf[i];
  }
  int count = valid - 2 * trim;
  return (count > 0) ? sum / count : adcBuf[valid / 2];
}

void initADC() {
  analogSetPinAttenuation(PH_PIN, ADC_11db);
  adc_power_acquire();  // Keep ADC powered to prevent hall sensor glitches on GPIO35
  analogReadMilliVolts(PH_PIN);  // Dummy read to prime SAR ADC
}

void initExternalADC() {
  useExternalADC = configStore.getUseADS1115();
  if (!useExternalADC) return;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);  // 100 kHz standard mode

  // Probe ADS1115: read config register
  Wire.beginTransmission(ADS1115_I2C_ADDR);
  Wire.write(ADS_REG_CONFIG);
  if (Wire.endTransmission() != 0) {
    adsFallback = true;
    Serial.println("ADS1115 not found at 0x48 — falling back to internal ADC");
    return;
  }

  // Verify with a test conversion
  float testMv = readADS1115MilliVolts();
  if (!isnan(testMv)) {
    adsAvailable = true;
    adsFallback = false;
    Serial.printf("ADS1115 initialized (test read: %.2f mV)\n", testMv);
  } else {
    adsFallback = true;
    Serial.println("ADS1115 test read failed — falling back to internal ADC");
  }
}

bool isExternalADCActive() { return adsActive(); }
bool isExternalADCFallback() { return adsFallback; }

void updateCalibrationFit() {
  // Piecewise linear: two segments that pass exactly through calibration points
  float dv;

  // Acid segment: pH 4 → pH 7
  dv = voltage_7PH - voltage_4PH;
  if (fabsf(dv) > 1e-6f) {
    acidSlope = (BUFFER_PH_7 - BUFFER_PH_4) / dv;
    acidOffset = BUFFER_PH_4 - acidSlope * voltage_4PH;
  } else {
    acidSlope = -1.0f / 173.0f;
    acidOffset = BUFFER_PH_7 + voltage_7PH / 173.0f;
  }

  // Base segment: pH 7 → pH 10
  dv = voltage_10PH - voltage_7PH;
  if (fabsf(dv) > 1e-6f) {
    baseSlope = (BUFFER_PH_10 - BUFFER_PH_7) / dv;
    baseOffset = BUFFER_PH_7 - baseSlope * voltage_7PH;
  } else {
    baseSlope = acidSlope;
    baseOffset = acidOffset;
  }
}

bool isCalibrationValid() {
  if (isnan(voltage_4PH) || isnan(voltage_7PH) || isnan(voltage_10PH)) return false;
  if (voltage_4PH <= 0 || voltage_7PH <= 0 || voltage_10PH <= 0) return false;
  if (fabs(voltage_4PH - voltage_7PH) < 50.0f) return false;
  if (fabs(voltage_7PH - voltage_10PH) < 50.0f) return false;
  // Verify monotonic ordering (lower pH → higher voltage on DFRobot board)
  if (!(voltage_4PH > voltage_7PH && voltage_7PH > voltage_10PH)) return false;
  return true;
}

// --- Probe health metrics ---

unsigned long getLastStabilizationMs() {
  return lastStabilizationMs;
}

float getProbeSlope() {
  if (!isCalibrationValid()) return NAN;
  float slopeAcid = (voltage_7PH - voltage_4PH) / (BUFFER_PH_7 - BUFFER_PH_4);
  float slopeBase = (voltage_10PH - voltage_7PH) / (BUFFER_PH_10 - BUFFER_PH_7);
  return (slopeAcid + slopeBase) / 2.0f;
}

float getAcidSlope() {
  if (!isCalibrationValid()) return NAN;
  return (voltage_7PH - voltage_4PH) / (BUFFER_PH_7 - BUFFER_PH_4);
}

float getAlkalineSlope() {
  if (!isCalibrationValid()) return NAN;
  return (voltage_10PH - voltage_7PH) / (BUFFER_PH_10 - BUFFER_PH_7);
}

// Nernst efficiency: probe slope vs theoretical at measurement temperature
// For internal ADC: raw_slope = conditioned_slope / PH_AMP_GAIN (board amplifier)
// For ADS1115: the effective gain differs from the nominal 3x due to ESP32 ADC
//   nonlinearity, so we use a separate empirical gain factor.
static float slopeToEfficiency(float conditionedSlope) {
  if (isnan(conditionedSlope)) return NAN;
  float nernst = NERNST_FACTOR * (273.15f + configStore.getMeasTempC());
  float gain = PH_AMP_GAIN;
  float rawSlope = fabsf(conditionedSlope) / gain;
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
  float slopeAcid = fabsf((voltage_7PH - voltage_4PH) / (BUFFER_PH_7 - BUFFER_PH_4));
  float slopeBase = fabsf((voltage_10PH - voltage_7PH) / (BUFFER_PH_10 - BUFFER_PH_7));
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

  // Secondary: asymmetry check
  float asym = getProbeAsymmetry();
  if (!isnan(asym) && asym > PROBE_ASYMMETRY_FAIR) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "asymmetry %.1f%% (limit %.0f%%)", asym, PROBE_ASYMMETRY_FAIR);
    return "Replace";
  }
  if (!isnan(asym) && asym > PROBE_ASYMMETRY_GOOD) {
    if (reasonBuf) snprintf(reasonBuf, reasonLen, "asymmetry %.1f%% (limit %.0f%%)", asym, PROBE_ASYMMETRY_GOOD);
    return "Fair";
  }

  // Tertiary: noise check (only after enough stabilization readings for a meaningful average)
  float avgNoise = getAvgStabNoiseMv();
  if (avgNoise > 0 && stabNoiseMvCount >= 5) {
    if (avgNoise > PROBE_NOISE_FAIR_MV) {
      if (reasonBuf) snprintf(reasonBuf, reasonLen, "high noise %.1f mV (limit %.0f mV)", avgNoise, PROBE_NOISE_FAIR_MV);
      return "Fair";
    }
    if (avgNoise > PROBE_NOISE_GOOD_MV) {
      if (reasonBuf) snprintf(reasonBuf, reasonLen, "elevated noise %.1f mV (limit %.0f mV)", avgNoise, PROBE_NOISE_GOOD_MV);
      return "Fair";
    }
  }

  if (reasonBuf && reasonLen > 0) reasonBuf[0] = '\0';
  return "Good";
}

// Wait for filtered ADC readings to converge instead of fixed delay.
// Uses trimmed-mean reads (16 samples) instead of raw single-sample reads
// to avoid false instability from ADC/WiFi noise spikes.
static void waitForStabilization() {
  lastStabTimedOut = false;
  if (!adsActive()) {
    analogReadMilliVolts(PH_PIN);  // Dummy read to prime ADC after idle
    delayMicroseconds(100);
  }

  // Collect readings for both convergence check and noise computation
  static const int MAX_STAB_READINGS = 80;  // 4s / 50ms = 80 max
  float stabReadings[MAX_STAB_READINGS];
  int nReadings = 0;
  int stabSamples = effectiveStabSamples();
  float stabThresh = effectiveStabThreshold();

  float prev = readADCTrimmed(stabSamples, ADC_INTER_SAMPLE_DELAY_MS);
  stabReadings[nReadings++] = prev;
  if (!adsActive()) delay(50);  // ADS1115 conversion time (75ms/sample) provides spacing
  unsigned long start = millis();
  bool converged = false;
  int consecCount = 0;
  while (millis() - start < (unsigned long)stabilizationTimeoutMs) {
    float curr = readADCTrimmed(stabSamples, ADC_INTER_SAMPLE_DELAY_MS);
    if (nReadings < MAX_STAB_READINGS) stabReadings[nReadings++] = curr;
    if (fabs(curr - prev) < stabThresh) {
      consecCount++;
      if (consecCount >= STAB_CONSEC_REQUIRED) {
        unsigned long elapsed = millis() - start;
        lastStabilizationMs = elapsed;
        stabTotalMs += elapsed;
        converged = true;
        break;
      }
    } else {
      consecCount = 0;
    }
    prev = curr;
    if (!adsActive()) delay(50);
  }
  if (!converged) {
    lastStabilizationMs = stabilizationTimeoutMs;
    stabTotalMs += stabilizationTimeoutMs;
    stabTimeoutCount++;
    lastStabTimedOut = true;
  }

  // Compute noise StdDev from post-convergence readings only (excludes mixing drift).
  // On timeout, fall back to all readings (best effort).
  int noiseStart = 0;
  if (converged) {
    noiseStart = nReadings - (STAB_CONSEC_REQUIRED + 1);
    if (noiseStart < 0) noiseStart = 0;
  }
  int noiseCount = nReadings - noiseStart;
  if (noiseCount >= 2) {
    float sum = 0;
    for (int i = noiseStart; i < nReadings; i++) sum += stabReadings[i];
    float mean = sum / noiseCount;
    float sumSq = 0;
    for (int i = noiseStart; i < nReadings; i++) {
      float d = stabReadings[i] - mean;
      sumSq += d * d;
    }
    lastStabNoiseMv = sqrtf(sumSq / (noiseCount - 1));
    stabNoiseMvSum += lastStabNoiseMv;
    stabNoiseMvCount++;
    if (lastStabNoiseMv > stabNoiseMvMax) stabNoiseMvMax = lastStabNoiseMv;
    if (lastStabNoiseMv > PROBE_NOISE_GOOD_MV) stabNoiseMvHighCount++;
  }
}

void waitForPHStabilization() {
  waitForStabilization();
}

// Core pH reading logic shared by measurePH and measurePHStabilized
static void measurePHCore(int nreadings) {
  static float pHReadings[100];
  const int maxReadings = (nreadings > 100) ? 100 : nreadings;
  int validReadings = 0;

  int oversample = effectiveOversampling();
  for (int t = 0; t < maxReadings; t++) {
    voltage = readADCTrimmed(oversample, ADC_INTER_SAMPLE_DELAY_MS);

    float calculatedPH = voltageToPH(voltage);

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

  int oversampleFast = effectiveOversamplingFast();
  for (int t = 0; t < maxReadings; t++) {
    voltage = readADCTrimmed(oversampleFast, ADC_INTER_SAMPLE_DELAY_FAST_MS);

    float calculatedPH = voltageToPH(voltage);

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

  int oversample = effectiveOversampling();
  for (int t = 0; t < maxReadings; t++) {
    voltageReadings[t] = readADCTrimmed(oversample, ADC_INTER_SAMPLE_DELAY_MS);
    delay(MEASUREMENT_DELAY_MS);
  }

  sortFloats(voltageReadings, maxReadings);
  float avgVoltage = medianFilteredMean(voltageReadings, maxReadings, VOLTAGE_OUTLIER_THRESHOLD);
  voltage = avgVoltage;
  return avgVoltage;
}

// --- Gran transformation endpoint detection ---

// Internal: weighted linear regression on Gran function values within a pH window.
// Uses inverse-variance weighting (w = 1/y²) so that noisier high-value points
// near the equivalence point don't dominate the fit.
// excluded[] marks points to skip; returns false if regression fails.
static bool granRegression(TitrationPoint* points, int nPoints,
                           float sampleVol, float k, bool* excluded,
                           float pHLow, float pHHigh,
                           float* outSlope, float* outIntercept,
                           float* outR2, float* outSsRes, int* outCount) {
  float sumW = 0, sumWX = 0, sumWY = 0, sumWXX = 0, sumWXY = 0, sumWYY = 0;
  int count = 0;

  for (int i = 0; i < nPoints; i++) {
    if (excluded[i]) continue;
    if (points[i].pH < pHHigh && points[i].pH > pHLow) {
      float x = points[i].units;
      float totalVol = sampleVol + x * k;
      float y = totalVol * powf(10.0f, -points[i].pH);
      float w = 1.0f / (y * y);  // inverse-variance weight
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
      float w = 1.0f / (y * y);
      float pred = *outSlope * x + *outIntercept;
      float res = y - pred;
      ssRes += w * res * res;
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
                           float* outR2,
                           float* outSlope = nullptr,
                           float* outIntercept = nullptr) {
  bool excluded[MAX_TITRATION_POINTS];
  for (int i = 0; i < nPoints; i++) excluded[i] = false;

  float slope, intercept, r2, ssRes;
  int count;

  if (!granRegression(points, nPoints, sampleVol, k, excluded,
                      pHLow, pHHigh,
                      &slope, &intercept, &r2, &ssRes, &count))
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
        float w = 1.0f / (y * y);
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
                        &slope, &intercept, &r2, &ssRes, &count))
      return NAN;
  }

  if (slope <= 0) return NAN;

  float eqUnits = -intercept / slope;
  if (eqUnits < 0 || eqUnits > points[nPoints - 1].units) return NAN;

  *outR2 = r2;
  if (outSlope) *outSlope = slope;
  if (outIntercept) *outIntercept = intercept;
  return eqUnits;
}

float granAnalysis(TitrationPoint* points, int nPoints,
                   float sampleVol, float titVol, float calUnits,
                   float* outR2,
                   float* outWinLow, float* outWinHigh,
                   char* reasonBuf, size_t reasonLen,
                   float* outSlope, float* outIntercept,
                   GranWindowResult* windowResults, int* nWindowResults) {
  auto fail = [&](const char* reason) -> float {
    if (reasonBuf && reasonLen > 0) snprintf(reasonBuf, reasonLen, "%s", reason);
    return NAN;
  };

  if (nPoints < 3) return fail("Too few data points");
  if (calUnits <= 0) return fail("Invalid calibration units");

  float k = titVol / calUnits;

  // Adaptive window selection: try multiple pH bounds, keep best R²
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
  int winCount = 0;

  for (int lb = 0; lb < nLower; lb++) {
    for (int b = 0; b < nBounds; b++) {
      if (upperBounds[b] <= lowerBounds[lb]) continue;
      if (upperBounds[b] - lowerBounds[lb] < 0.15f) continue;
      float r2 = 0, s = 0, ic = 0;
      float eq = tryGranWindow(points, nPoints, sampleVol, k,
                               lowerBounds[lb], upperBounds[b], &r2, &s, &ic);
      if (windowResults && winCount < MAX_GRAN_WINDOWS) {
        windowResults[winCount] = {lowerBounds[lb], upperBounds[b], r2, !isnan(eq), eq};
        winCount++;
      }
      if (!isnan(eq) && r2 > bestR2) {
        bestR2 = r2;
        bestEqUnits = eq;
        bestLow = lowerBounds[lb];
        bestHigh = upperBounds[b];
        bestSlope = s;
        bestIntercept = ic;
      }
    }
  }
  if (nWindowResults) *nWindowResults = winCount;

  if (isnan(bestEqUnits)) return fail("No valid Gran window found");

  if (outWinLow) *outWinLow = bestLow;
  if (outWinHigh) *outWinHigh = bestHigh;
  if (outSlope) *outSlope = bestSlope;
  if (outIntercept) *outIntercept = bestIntercept;

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
