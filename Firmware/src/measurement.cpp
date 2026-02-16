#include <Arduino.h>
#include "measurement.h"
#include <pins.h>
#include <config.h>
#include <math.h>

// Measurement state
float voltage = 0;
float pH = 0;
float voltage_4PH = 0;
float voltage_7PH = 0;
float voltage_10PH = 0;

// Cached linear fit coefficients: pH = phSlope * voltage + phOffset
static float phSlope = 0;
static float phOffset = 7.0;

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

void initADC() {
  analogSetPinAttenuation(PH_PIN, ADC_11db);
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

// Wait for ADC readings to converge instead of fixed delay
static void waitForStabilization() {
  float prev = (float)analogReadMilliVolts(PH_PIN);
  delay(50);
  unsigned long start = millis();
  while (millis() - start < STABILIZATION_TIMEOUT_MS) {
    float curr = (float)analogReadMilliVolts(PH_PIN);
    if (fabs(curr - prev) < STABILIZATION_THRESHOLD_MV) {
      return;
    }
    prev = curr;
    delay(50);
  }
}

// pH measurement with adaptive stabilization, oversampling, and outlier removal
void measurePH(int nreadings) {
  waitForStabilization();

  static float pHReadings[100];
  const int maxReadings = (nreadings > 100) ? 100 : nreadings;
  int validReadings = 0;

  for (int t = 0; t < maxReadings; t++) {
    float voltageSum = 0.0;
    for (int s = 0; s < ADC_OVERSAMPLING; s++) {
      voltageSum += (float)analogReadMilliVolts(PH_PIN);
      delay(ADC_INTER_SAMPLE_DELAY_MS);
    }
    voltage = voltageSum / ADC_OVERSAMPLING;

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

// Fast pH measurement — no stabilization, reduced oversampling
// For use far from endpoint where ±0.3 pH accuracy is sufficient
void measurePHFast(int nreadings) {
  static float pHReadings[100];
  const int maxReadings = (nreadings > 100) ? 100 : nreadings;
  int validReadings = 0;

  for (int t = 0; t < maxReadings; t++) {
    float voltageSum = 0.0;
    for (int s = 0; s < ADC_OVERSAMPLING_FAST; s++) {
      voltageSum += (float)analogReadMilliVolts(PH_PIN);
      delay(ADC_INTER_SAMPLE_DELAY_FAST_MS);
    }
    voltage = voltageSum / ADC_OVERSAMPLING_FAST;

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
    float voltageSum = 0.0;
    for (int s = 0; s < ADC_OVERSAMPLING; s++) {
      voltageSum += (float)analogReadMilliVolts(PH_PIN);
      delay(ADC_INTER_SAMPLE_DELAY_MS);
    }
    voltageReadings[t] = voltageSum / ADC_OVERSAMPLING;
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
