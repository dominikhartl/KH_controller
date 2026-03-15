#include "hw_diagnostics.h"
#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <math.h>
#include "config.h"
#include "pins.h"
#include "measurement.h"
#include "tmc_driver.h"
#include "motors.h"
#include "temperature.h"
#include "config_store.h"
#include "web_server.h"

extern char deviceName[];
extern void subtractHCl(int unitsUsed);

// --- Report data structures ---

struct NoiseStats {
  int nSamples;
  float meanMv, medianMv, stddevMv, minMv, maxMv, ptpMv;
  float noiseLsbs;  // ADS1115 only
};

struct MultiRateEntry {
  uint16_t sps;
  int nSamples;
  float stddevMv, ptpMv, convMs;
};

struct MotorDiagData {
  uint16_t sgMin, sgMax;
  float sgAvg;
};

// Maximum samples for rapid noise test time series
static const int RAPID_SAMPLES = 500;
static const int RAPID_TS_DOWNSAMPLE = 100;  // Keep ≤100 to fit in TCP chunk (~1460 bytes)
// Multi-rate test samples per rate
static const int MULTIRATE_SAMPLES = 50;
// Histogram bins
static const int HIST_BINS = 16;

// Report storage
static bool diagRunning = false;
static bool reportReady = false;
static unsigned long diagStartMs = 0;
static unsigned long diagDurationMs = 0;

// System health
static uint32_t sysUptime;
static int sysResetReason;
static uint32_t sysFreeHeap, sysMaxAlloc, sysHeapMin;
static uint32_t sysFlashTotal, sysFlashUsed;
static int8_t sysRSSI;
static uint8_t sysWifiChannel;

// Hardware detection
static bool hwADS1115, hwTMC2209, hwDS18B20;

// I2C test results
static bool i2cSkipped;
static bool i2cAdsDetected, i2cConfigOk;
static int i2cNakCount, i2cConvOk, i2cConvTotal;
static float i2cAvgConvMs, i2cMaxConvMs;
static bool i2cRdyOk;
static uint8_t i2cDevices[16];
static int i2cDeviceCount;

// ADC noise: pH channel
static NoiseStats adcPhNoise;
static float adcPhHistBins[HIST_BINS];
static float adcPhHistEdges[HIST_BINS + 1];
// ADC noise: baseline (AIN1)
static bool adcBaselineSkipped;
static NoiseStats adcBaselineNoise;
static float adcNoiseRatio;
// Multi-rate
static bool adcMultiRateSkipped;
static MultiRateEntry adcMultiRate[8];
static int adcMultiRateCount;
// Internal ADC (ESP32 on GPIO36, board noise reference)
static NoiseStats internalAdcNoise;

// Motor noise comparison (motors off vs on)
static bool motorNoiseSkipped;
static NoiseStats motorOffNoise, motorOnNoise;

// pH noise equivalent (computed from ADC noise + calibration slope)
static float phSlope = 0;
static float noiseStddevPH = 0;
static float noisePtpPH = 0;
static float noiseKhPct = 0;  // estimated relative KH uncertainty %
static bool phNoiseValid = false;

// Temperature
static bool tempSkipped;
static float tempReadings[5];
static int tempReadingCount;
static float tempMean, tempStddev;
static bool tempCrcError, tempPorDetected;

// TMC drivers
static bool tmcSkipped;
static bool tmcSampleIoinOk, tmcTitrateIoinOk;
static uint32_t tmcSampleDrvStatus, tmcTitrateDrvStatus;

// Motors
static bool motorsSkipped;
static MotorDiagData motorSampleSC, motorSampleSP;
static MotorDiagData motorTitrateSC, motorTitrateSP;

// SG Profile (30 revs at operational speed for stall threshold calibration)
static const int SG_PROFILE_REVS = 30;
struct SGProfileData {
  uint16_t sg[30];  // per-revolution SG values
  int count;
  uint16_t sgMin, sgMax;
  float sgMean, sgStddev;
  uint16_t recommendedStallSG;  // min * 0.4
};
static SGProfileData sgProfileSample, sgProfileTitrate;

// GPIO
struct GPIOState {
  const char* name;
  uint8_t pin;
  int state;
  int expected;
  bool ok;
};
static GPIOState gpioStates[8];
static int gpioCount;

// Probe
static float probeVoltageMv;
static bool probeCalibrated;

// Time series for rapid test (stored on heap)
static int16_t* rapidRawSamples = nullptr;

// --- Helper: compute noise stats from raw int16 samples ---

static void computeNoiseStatsI16(int16_t* raw, int n, float mvPerBit, NoiseStats* out) {
  out->nSamples = n;
  if (n == 0) { *out = {}; return; }

  // Convert to mV and compute stats
  float sum = 0, minV = 1e9, maxV = -1e9;
  for (int i = 0; i < n; i++) {
    float mv = raw[i] * mvPerBit;
    sum += mv;
    if (mv < minV) minV = mv;
    if (mv > maxV) maxV = mv;
  }
  out->meanMv = sum / n;
  out->minMv = minV;
  out->maxMv = maxV;
  out->ptpMv = maxV - minV;

  // Median (simple selection on copy)
  float* tmp = (float*)malloc(n * sizeof(float));
  if (tmp) {
    for (int i = 0; i < n; i++) tmp[i] = raw[i] * mvPerBit;
    // Insertion sort for median
    for (int i = 1; i < n; i++) {
      float v = tmp[i]; int j = i - 1;
      while (j >= 0 && tmp[j] > v) { tmp[j+1] = tmp[j]; j--; }
      tmp[j+1] = v;
    }
    out->medianMv = (n % 2) ? tmp[n/2] : (tmp[n/2-1] + tmp[n/2]) / 2.0f;
    free(tmp);
  } else {
    out->medianMv = out->meanMv;
  }

  // StdDev
  float sumSq = 0;
  for (int i = 0; i < n; i++) {
    float d = raw[i] * mvPerBit - out->meanMv;
    sumSq += d * d;
  }
  out->stddevMv = (n > 1) ? sqrtf(sumSq / (n - 1)) : 0;
  out->noiseLsbs = (mvPerBit > 0) ? out->stddevMv / mvPerBit : 0;
}

// --- Helper: compute noise stats from float mV samples ---

static void computeNoiseStatsF(float* samples, int n, NoiseStats* out) {
  out->nSamples = n;
  out->noiseLsbs = 0;
  if (n == 0) { *out = {}; return; }

  float sum = 0, minV = 1e9, maxV = -1e9;
  for (int i = 0; i < n; i++) {
    sum += samples[i];
    if (samples[i] < minV) minV = samples[i];
    if (samples[i] > maxV) maxV = samples[i];
  }
  out->meanMv = sum / n;
  out->medianMv = out->meanMv;  // simplified
  out->minMv = minV;
  out->maxMv = maxV;
  out->ptpMv = maxV - minV;

  float sumSq = 0;
  for (int i = 0; i < n; i++) {
    float d = samples[i] - out->meanMv;
    sumSq += d * d;
  }
  out->stddevMv = (n > 1) ? sqrtf(sumSq / (n - 1)) : 0;
}

// --- Individual test functions ---

static void testSystemHealth() {
  sysUptime = millis() / 1000;
  sysResetReason = (int)esp_reset_reason();
  sysFreeHeap = ESP.getFreeHeap();
  sysMaxAlloc = ESP.getMaxAllocHeap();
  sysHeapMin = ESP.getMinFreeHeap();
  sysFlashTotal = LittleFS.totalBytes();
  sysFlashUsed = LittleFS.usedBytes();
  sysRSSI = WiFi.isConnected() ? WiFi.RSSI() : 0;
  sysWifiChannel = WiFi.isConnected() ? WiFi.channel() : 0;
}

static void testI2CBus() {
  if (!hwADS1115) {
    i2cSkipped = true;
    return;
  }
  i2cSkipped = false;
  i2cNakCount = 0;
  i2cConvOk = 0;
  i2cConvTotal = 10;

  // Config register readback test
  Wire.beginTransmission(ADS1115_I2C_ADDR);
  Wire.write(0x01);  // config register
  i2cAdsDetected = (Wire.endTransmission() == 0);

  // Write config and read back
  i2cConfigOk = true;
  {
    uint16_t testConfig = 0xC320;
    Wire.beginTransmission(ADS1115_I2C_ADDR);
    Wire.write(0x01);
    Wire.write((uint8_t)(testConfig >> 8));
    Wire.write((uint8_t)(testConfig & 0xFF));
    if (Wire.endTransmission() != 0) { i2cConfigOk = false; i2cNakCount++; }

    Wire.beginTransmission(ADS1115_I2C_ADDR);
    Wire.write(0x01);
    if (Wire.endTransmission() != 0) { i2cConfigOk = false; i2cNakCount++; }
    if (Wire.requestFrom((uint8_t)ADS1115_I2C_ADDR, (uint8_t)2) == 2) {
      uint16_t readback = ((uint16_t)Wire.read() << 8) | Wire.read();
      // OS bit auto-clears, so mask it out
      if ((readback & 0x7FFF) != (testConfig & 0x7FFF)) i2cConfigOk = false;
    } else {
      i2cConfigOk = false;
    }
  }

  // Conversion timing test
  float convTimes[10];
  i2cMaxConvMs = 0;
  float convSum = 0;
  for (int i = 0; i < i2cConvTotal; i++) {
    unsigned long t0 = millis();
    int16_t raw = readADS1115RawDiag(0x04, 0x01);  // AIN0/GND, 16 SPS
    unsigned long elapsed = millis() - t0;
    convTimes[i] = (float)elapsed;
    if (raw != INT16_MIN) {
      i2cConvOk++;
      convSum += convTimes[i];
      if (convTimes[i] > i2cMaxConvMs) i2cMaxConvMs = convTimes[i];
    }
  }
  i2cAvgConvMs = (i2cConvOk > 0) ? convSum / i2cConvOk : 0;

  // RDY pin test: start conversion, check if RDY goes low
  i2cRdyOk = false;
  {
    Wire.beginTransmission(ADS1115_I2C_ADDR);
    Wire.write(0x01);
    Wire.write(0xC3); Wire.write(0x20);
    Wire.endTransmission();
    delay(80);
    if (!digitalRead(ADS_RDY_PIN)) i2cRdyOk = true;
  }

  // I2C bus scan
  i2cDeviceCount = 0;
  for (uint8_t addr = 1; addr < 127 && i2cDeviceCount < 16; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      i2cDevices[i2cDeviceCount++] = addr;
    }
  }
}

static void testADCNoisePH() {
  broadcastMessage("ADC noise test: pH channel...");

  if (hwADS1115) {
    // ADS1115 rapid sampling on AIN0 (pH circuit)
    rapidRawSamples = (int16_t*)malloc(RAPID_SAMPLES * sizeof(int16_t));
    if (!rapidRawSamples) {
      adcPhNoise = {};
      return;
    }

    int valid = 0;
    for (int i = 0; i < RAPID_SAMPLES; i++) {
      int16_t raw = readADS1115RawDiag(0x04, 0x01);  // AIN0/GND, 16 SPS
      if (raw != INT16_MIN) {
        rapidRawSamples[valid++] = raw;
      }
    }

    computeNoiseStatsI16(rapidRawSamples, valid, ADS_MV_PER_BIT, &adcPhNoise);

    // Build histogram
    if (valid > 0) {
      float range = adcPhNoise.maxMv - adcPhNoise.minMv;
      if (range < 0.001f) range = 0.125f;  // minimum 1 LSB
      float binWidth = range / HIST_BINS;
      for (int i = 0; i <= HIST_BINS; i++) {
        adcPhHistEdges[i] = adcPhNoise.minMv + i * binWidth;
      }
      for (int i = 0; i < HIST_BINS; i++) adcPhHistBins[i] = 0;
      for (int i = 0; i < valid; i++) {
        float mv = rapidRawSamples[i] * ADS_MV_PER_BIT;
        int bin = (int)((mv - adcPhNoise.minMv) / binWidth);
        if (bin >= HIST_BINS) bin = HIST_BINS - 1;
        if (bin < 0) bin = 0;
        adcPhHistBins[bin]++;
      }
    }
    // Keep rapidRawSamples for time series serialization; freed after report served
  } else {
    // Internal ADC rapid sampling
    float* samples = (float*)malloc(RAPID_SAMPLES * sizeof(float));
    if (!samples) { adcPhNoise = {}; return; }
    int valid = 0;
    for (int i = 0; i < RAPID_SAMPLES; i++) {
      samples[valid++] = readInternalADCmV();
      delay(2);
    }
    computeNoiseStatsF(samples, valid, &adcPhNoise);

    // Build histogram from float samples
    if (valid > 0) {
      float range = adcPhNoise.maxMv - adcPhNoise.minMv;
      if (range < 1.0f) range = 1.0f;
      float binWidth = range / HIST_BINS;
      for (int i = 0; i <= HIST_BINS; i++) adcPhHistEdges[i] = adcPhNoise.minMv + i * binWidth;
      for (int i = 0; i < HIST_BINS; i++) adcPhHistBins[i] = 0;
      for (int i = 0; i < valid; i++) {
        int bin = (int)((samples[i] - adcPhNoise.minMv) / binWidth);
        if (bin >= HIST_BINS) bin = HIST_BINS - 1;
        if (bin < 0) bin = 0;
        adcPhHistBins[bin]++;
      }
    }
    free(samples);
  }
}

static void testADCBaseline() {
  if (!hwADS1115) {
    adcBaselineSkipped = true;
    return;
  }
  adcBaselineSkipped = false;
  broadcastMessage("ADC noise test: baseline (AIN1)...");

  const int N = 200;
  int16_t* samples = (int16_t*)malloc(N * sizeof(int16_t));
  if (!samples) { adcBaselineNoise = {}; return; }

  int valid = 0;
  for (int i = 0; i < N; i++) {
    int16_t raw = readADS1115RawDiag(0x05, 0x01);  // AIN1/GND, 16 SPS
    if (raw != INT16_MIN) samples[valid++] = raw;
  }

  computeNoiseStatsI16(samples, valid, ADS_MV_PER_BIT, &adcBaselineNoise);
  free(samples);

  // Compute noise ratio
  adcNoiseRatio = (adcBaselineNoise.stddevMv > 0.001f)
    ? adcPhNoise.stddevMv / adcBaselineNoise.stddevMv : 0;
}

static void testADCMultiRate() {
  if (!hwADS1115) {
    adcMultiRateSkipped = true;
    return;
  }
  adcMultiRateSkipped = false;
  broadcastMessage("ADC multi-rate noise test...");

  static const uint16_t spsTable[] = {8, 16, 32, 64, 128, 250, 475, 860};
  adcMultiRateCount = 8;

  int16_t samples[MULTIRATE_SAMPLES];

  for (int r = 0; r < 8; r++) {
    unsigned long t0 = millis();
    int valid = 0;
    for (int i = 0; i < MULTIRATE_SAMPLES; i++) {
      int16_t raw = readADS1115RawDiag(0x04, (uint8_t)r);  // AIN0/GND, rate r
      if (raw != INT16_MIN) samples[valid++] = raw;
    }
    unsigned long elapsed = millis() - t0;

    adcMultiRate[r].sps = spsTable[r];
    adcMultiRate[r].nSamples = valid;
    adcMultiRate[r].convMs = (valid > 0) ? (float)elapsed / valid : 0;

    // Compute stddev and ptp
    if (valid > 0) {
      float sum = 0, minV = 1e9, maxV = -1e9;
      for (int i = 0; i < valid; i++) {
        float mv = samples[i] * ADS_MV_PER_BIT;
        sum += mv;
        if (mv < minV) minV = mv;
        if (mv > maxV) maxV = mv;
      }
      float mean = sum / valid;
      float sumSq = 0;
      for (int i = 0; i < valid; i++) {
        float d = samples[i] * ADS_MV_PER_BIT - mean;
        sumSq += d * d;
      }
      adcMultiRate[r].stddevMv = (valid > 1) ? sqrtf(sumSq / (valid - 1)) : 0;
      adcMultiRate[r].ptpMv = maxV - minV;
    } else {
      adcMultiRate[r].stddevMv = 0;
      adcMultiRate[r].ptpMv = 0;
    }
  }
}

static void testInternalADC() {
  broadcastMessage("ESP32 ADC board noise test (GPIO36)...");

  pinMode(NOISE_REF_PIN, INPUT);
  analogSetPinAttenuation(NOISE_REF_PIN, ADC_11db);
  analogReadMilliVolts(NOISE_REF_PIN);  // Dummy read to prime ADC

  const int N = 200;
  float* samples = (float*)malloc(N * sizeof(float));
  if (!samples) { internalAdcNoise = {}; return; }

  for (int i = 0; i < N; i++) {
    samples[i] = (float)analogReadMilliVolts(NOISE_REF_PIN);
    delay(2);
  }

  computeNoiseStatsF(samples, N, &internalAdcNoise);
  free(samples);
}

static void testMotorNoiseComparison() {
  if (!hwADS1115 || !hwTMC2209) {
    motorNoiseSkipped = true;
    return;
  }
  motorNoiseSkipped = false;
  broadcastMessage("Motor noise comparison (off vs on)...");

  const int N = 200;
  int16_t samples[N];  // 400 bytes on stack — fine for ESP32

  // Motors OFF (EN HIGH = disabled)
  digitalWrite(EN_PIN1, HIGH);
  digitalWrite(EN_PIN2, HIGH);
  delay(50);

  int valid = 0;
  for (int i = 0; i < N; i++) {
    int16_t raw = readADS1115RawDiag(0x04, 0x01);  // AIN0/GND, 16 SPS
    if (raw != INT16_MIN) samples[valid++] = raw;
  }
  computeNoiseStatsI16(samples, valid, ADS_MV_PER_BIT, &motorOffNoise);

  // Motors ON (EN LOW = enabled, but no stepping)
  digitalWrite(EN_PIN1, LOW);
  digitalWrite(EN_PIN2, LOW);
  delay(100);  // let charge pumps stabilize

  valid = 0;
  for (int i = 0; i < N; i++) {
    int16_t raw = readADS1115RawDiag(0x04, 0x01);
    if (raw != INT16_MIN) samples[valid++] = raw;
  }
  computeNoiseStatsI16(samples, valid, ADS_MV_PER_BIT, &motorOnNoise);

  // Restore motors OFF
  digitalWrite(EN_PIN1, HIGH);
  digitalWrite(EN_PIN2, HIGH);
}

static void testTemperature() {
  if (!hwDS18B20) {
    tempSkipped = true;
    return;
  }
  tempSkipped = false;
  broadcastMessage("Temperature sensor test...");

  tempCrcError = false;
  tempPorDetected = false;
  tempReadingCount = 0;

  for (int i = 0; i < 5; i++) {
    float t = getWaterTemperatureC();
    if (t == DEFAULT_MEASUREMENT_TEMP_C && hasTemperatureSensor()) {
      // getWaterTemperatureC returns default on DEVICE_DISCONNECTED_C (-127)
      tempCrcError = true;
    } else if (fabsf(t - 85.0f) < 0.1f) {
      tempPorDetected = true;
    }
    tempReadings[tempReadingCount++] = t;
    delay(200);
  }

  // Compute mean/stddev
  float sum = 0;
  for (int i = 0; i < tempReadingCount; i++) sum += tempReadings[i];
  tempMean = sum / tempReadingCount;
  float sumSq = 0;
  for (int i = 0; i < tempReadingCount; i++) {
    float d = tempReadings[i] - tempMean;
    sumSq += d * d;
  }
  tempStddev = (tempReadingCount > 1) ? sqrtf(sumSq / (tempReadingCount - 1)) : 0;
}

static void testTMCDrivers() {
  if (!hwTMC2209) {
    tmcSkipped = true;
    return;
  }
  tmcSkipped = false;
  broadcastMessage("TMC2209 driver test...");

  tmcSampleDrvStatus = getSampleDrvStatus();
  tmcTitrateDrvStatus = getTitrateDrvStatus();

  // IOIN read test — non-zero means communication succeeded
  uint32_t sIoin = getSampleIOIN();
  uint32_t tIoin = getTitrateIOIN();
  tmcSampleIoinOk = (sIoin != 0);
  tmcTitrateIoinOk = (tIoin != 0);
}

static void testMotors() {
  if (!hwTMC2209) {
    motorsSkipped = true;
    return;
  }
  motorsSkipped = false;
  broadcastMessage("Motor diagnostics...");

  const int DIAG_REVS = 5;
  const int MAX_S = 20;
  SGSample samples[MAX_S];

  float sampleRPM = configStore.getSamplePumpRPM();
  float titrateRPM = TITRATION_RPM;

  auto fillResult = [](SGSample* s, int n, MotorDiagData* out) {
    out->sgMin = 65535; out->sgMax = 0; out->sgAvg = 0;
    if (n == 0) return;
    uint32_t sum = 0;
    for (int i = 0; i < n; i++) {
      if (s[i].sg < out->sgMin) out->sgMin = s[i].sg;
      if (s[i].sg > out->sgMax) out->sgMax = s[i].sg;
      sum += s[i].sg;
    }
    out->sgAvg = (float)sum / n;
  };

  // Sample pump: StealthChop
  setSampleSpreadCycle(false);
  delay(50);
  int n = diagStepSample(DIAG_REVS, sampleRPM, samples, MAX_S);
  fillResult(samples, n, &motorSampleSC);

  // Sample pump: SpreadCycle
  setSampleSpreadCycle(true);
  delay(50);
  n = diagStepSample(DIAG_REVS, sampleRPM, samples, MAX_S);
  fillResult(samples, n, &motorSampleSP);

  setSampleSpreadCycle(configStore.getSampleSpreadCycle());

  // Titration pump: StealthChop
  setTitrateSpreadCycle(false);
  delay(50);
  n = diagStepTitrate(DIAG_REVS, titrateRPM, samples, MAX_S);
  fillResult(samples, n, &motorTitrateSC);

  // Titration pump: SpreadCycle
  setTitrateSpreadCycle(true);
  delay(50);
  n = diagStepTitrate(DIAG_REVS, titrateRPM, samples, MAX_S);
  fillResult(samples, n, &motorTitrateSP);

  setTitrateSpreadCycle(configStore.getTitrateSpreadCycle());

  // Subtract HCl used by titration pump diagnostics (2× DIAG_REVS revolutions)
  int diagUnits = DIAG_REVS * 2 * (STEPS_PER_REVOLUTION / MOTOR_STEPS_PER_UNIT);
  subtractHCl(diagUnits);

  // SG Profile: 30 revolutions at operational speed for stall threshold calibration
  broadcastMessage("SG profiling...");

  auto fillProfile = [](SGSample* s, int n, SGProfileData* out) {
    out->count = n;
    out->sgMin = 65535; out->sgMax = 0;
    uint32_t sum = 0;
    for (int i = 0; i < n && i < SG_PROFILE_REVS; i++) {
      out->sg[i] = s[i].sg;
      if (s[i].sg < out->sgMin) out->sgMin = s[i].sg;
      if (s[i].sg > out->sgMax) out->sgMax = s[i].sg;
      sum += s[i].sg;
    }
    out->sgMean = n > 0 ? (float)sum / n : 0;
    // Compute stddev
    float sumSq = 0;
    for (int i = 0; i < n; i++) {
      float d = s[i].sg - out->sgMean;
      sumSq += d * d;
    }
    out->sgStddev = n > 1 ? sqrtf(sumSq / (n - 1)) : 0;
    out->recommendedStallSG = (uint16_t)(out->sgMin * 0.4f);
  };

  SGSample profileSamples[SG_PROFILE_REVS];

  // Sample pump profile (StealthChop — the operational mode)
  setSampleSpreadCycle(configStore.getSampleSpreadCycle());
  delay(50);
  n = diagStepSample(SG_PROFILE_REVS, sampleRPM, profileSamples, SG_PROFILE_REVS);
  fillProfile(profileSamples, n, &sgProfileSample);

  // Titration pump profile
  setTitrateSpreadCycle(configStore.getTitrateSpreadCycle());
  delay(50);
  n = diagStepTitrate(SG_PROFILE_REVS, titrateRPM, profileSamples, SG_PROFILE_REVS);
  fillProfile(profileSamples, n, &sgProfileTitrate);

  // Subtract HCl used by titration pump SG profile
  int profileUnits = SG_PROFILE_REVS * (STEPS_PER_REVOLUTION / MOTOR_STEPS_PER_UNIT);
  subtractHCl(profileUnits);
}

static void testGPIOStates() {
  gpioCount = 0;

  auto addPin = [](const char* name, uint8_t pin, int expected) {
    GPIOState& g = gpioStates[gpioCount];
    g.name = name;
    g.pin = pin;
    g.state = digitalRead(pin);
    g.expected = expected;
    g.ok = (g.state == expected);
    gpioCount++;
  };

  // Motor enables should be HIGH (disabled) when idle
  addPin("EN_SAMPLE", EN_PIN1, HIGH);
  addPin("EN_TITRATE", EN_PIN2, HIGH);

  if (hwTMC2209) {
    // DIAG pins should be LOW (no stall) when motors are off
    addPin("DIAG_SAMPLE", DIAG_SAMPLE, LOW);
    addPin("DIAG_TITRATE", DIAG_TITRATE, LOW);
  }

  if (hwADS1115) {
    // RDY pin is LOW when idle (last conversion complete, no new conversion pending)
    addPin("ADS_RDY", ADS_RDY_PIN, LOW);
  }
}

static void testProbeHealth() {
  probeVoltageMv = voltage;  // current cached voltage
  probeCalibrated = isCalibrationValid();
}

// --- Main diagnostic runner ---

void runHardwareDiagnostics() {
  if (diagRunning) return;
  diagRunning = true;
  reportReady = false;
  diagStartMs = millis();

  broadcastMessage("Hardware diagnostics starting...");

  // Detect hardware
  hwADS1115 = isADS1115Available();
  hwTMC2209 = isTMCDetected();
  hwDS18B20 = hasTemperatureSensor();

  // Phase 1: System Health
  broadcastProgress(5);
  testSystemHealth();

  // Phase 2: I2C Bus
  broadcastProgress(10);
  testI2CBus();

  // Ensure motor drivers are disabled during ADC noise tests
  digitalWrite(EN_PIN1, HIGH);
  digitalWrite(EN_PIN2, HIGH);

  // Phase 3: ADC Rapid Noise (pH channel)
  broadcastProgress(15);
  testADCNoisePH();

  // Phase 4: ADC Baseline (AIN1)
  broadcastProgress(35);
  testADCBaseline();

  // Phase 5: Multi-rate noise
  broadcastProgress(45);
  testADCMultiRate();

  // Phase 6: Motor noise comparison (off vs on)
  broadcastProgress(55);
  testMotorNoiseComparison();

  // Phase 7: ESP32 ADC board noise (GPIO36, always runs)
  broadcastProgress(60);
  testInternalADC();

  // Phase 8: Temperature
  broadcastProgress(70);
  testTemperature();

  // Phase 9: TMC drivers
  broadcastProgress(75);
  testTMCDrivers();

  // Phase 10: Motors
  broadcastProgress(80);
  testMotors();

  // Phase 11: GPIO + Probe
  broadcastProgress(95);
  testGPIOStates();
  testProbeHealth();

  // Compute pH-equivalent noise from ADC noise + calibration slope
  phNoiseValid = false;
  if (probeCalibrated && adcPhNoise.nSamples > 0) {
    phSlope = (voltage_4PH - voltage_10PH) / 6.0f;
    if (phSlope > 50.0f) {  // sanity: slope must be positive and reasonable
      noiseStddevPH = adcPhNoise.stddevMv / phSlope;
      noisePtpPH = adcPhNoise.ptpMv / phSlope;
      // Estimated KH uncertainty %: Gran regression over ~15 points
      // σ_KH/KH ≈ σ_pH * ln(10) / √N
      noiseKhPct = noiseStddevPH * 2.302585f / sqrtf(15.0f) * 100.0f;
      phNoiseValid = true;
    }
  }

  diagDurationMs = millis() - diagStartMs;

  broadcastProgress(100);
  reportReady = true;
  diagRunning = false;
  broadcastMessage("Hardware diagnostics complete");

  // Notify UI
  ws.textAll("{\"type\":\"hwDiagDone\"}");
}

bool isHWDiagRunning() { return diagRunning; }
bool isHWDiagReportReady() { return reportReady; }

// --- Chunked JSON serialization ---

static const int SECTION_COUNT = 12;

int getHWDiagSectionCount() { return SECTION_COUNT; }

bool serveHWDiagChunk(int section, char* buf, size_t bufSize, size_t* written) {
  size_t pos = 0;
  // Helper macro for printf-style appending to buffer (guards against overflow)
  #define P(fmt, ...) do { \
    if (pos < bufSize) \
      pos += snprintf(buf + pos, bufSize - pos, fmt, ##__VA_ARGS__); \
  } while(0)

  switch (section) {
    case 0:  // Opening + hardware
      P("{\"type\":\"hw_diagnostics\",\"version\":1,\"device\":\"%s\",\"firmware\":\"%s\",",
        deviceName, FW_VERSION);
      P("\"timestamp\":%lu,\"duration_sec\":%.1f,",
        (unsigned long)time(nullptr), diagDurationMs / 1000.0f);
      P("\"hardware\":{\"ads1115\":%s,\"tmc2209\":%s,\"ds18b20\":%s,\"adc_source\":\"%s\"},",
        hwADS1115 ? "true" : "false",
        hwTMC2209 ? "true" : "false",
        hwDS18B20 ? "true" : "false",
        hwADS1115 ? "ADS1115" : "Internal");
      break;

    case 1:  // System
      P("\"system\":{\"uptime_sec\":%lu,\"reset_reason\":%d,",
        sysUptime, sysResetReason);
      P("\"free_heap\":%lu,\"max_alloc\":%lu,\"heap_min\":%lu,",
        sysFreeHeap, sysMaxAlloc, sysHeapMin);
      P("\"flash_total\":%lu,\"flash_used\":%lu,",
        sysFlashTotal, sysFlashUsed);
      P("\"wifi_rssi\":%d,\"wifi_channel\":%d},",
        sysRSSI, sysWifiChannel);
      break;

    case 2:  // I2C
      if (i2cSkipped) {
        P("\"i2c\":{\"skipped\":true},");
      } else {
        P("\"i2c\":{\"skipped\":false,\"ads1115_detected\":%s,\"config_readback_ok\":%s,",
          i2cAdsDetected ? "true" : "false", i2cConfigOk ? "true" : "false");
        P("\"nak_count\":%d,\"conversions_ok\":%d,\"conversions_total\":%d,",
          i2cNakCount, i2cConvOk, i2cConvTotal);
        P("\"avg_conv_ms\":%.1f,\"max_conv_ms\":%.1f,\"rdy_pin_ok\":%s,",
          i2cAvgConvMs, i2cMaxConvMs, i2cRdyOk ? "true" : "false");
        P("\"bus_devices\":[");
        for (int i = 0; i < i2cDeviceCount; i++) {
          P("%s%d", i ? "," : "", i2cDevices[i]);
        }
        P("]},");
      }
      break;

    case 3:  // ADC noise: pH channel
      P("\"adc_noise\":{\"adc_type\":\"%s\",", hwADS1115 ? "ADS1115" : "Internal");
      P("\"ph_channel\":{\"label\":\"%s\",",
        hwADS1115 ? "AIN0 (pH circuit)" : "GPIO34 (pH circuit)");
      P("\"n_samples\":%d,", adcPhNoise.nSamples);
      if (hwADS1115) P("\"rate_sps\":16,");
      P("\"mean_mv\":%.3f,\"median_mv\":%.3f,\"stddev_mv\":%.3f,",
        adcPhNoise.meanMv, adcPhNoise.medianMv, adcPhNoise.stddevMv);
      P("\"min_mv\":%.3f,\"max_mv\":%.3f,\"ptp_mv\":%.3f,",
        adcPhNoise.minMv, adcPhNoise.maxMv, adcPhNoise.ptpMv);
      if (hwADS1115) P("\"noise_lsbs\":%.2f,", adcPhNoise.noiseLsbs);
      // Histogram
      P("\"histogram\":{\"bin_edges\":[");
      for (int i = 0; i <= HIST_BINS; i++) P("%s%.3f", i ? "," : "", adcPhHistEdges[i]);
      P("],\"counts\":[");
      for (int i = 0; i < HIST_BINS; i++) P("%s%.0f", i ? "," : "", adcPhHistBins[i]);
      P("]},");
      // Time series (inline with buffer guard)
      P("\"time_series\":[");
      if (hwADS1115 && rapidRawSamples && adcPhNoise.nSamples > 0) {
        int step = adcPhNoise.nSamples / RAPID_TS_DOWNSAMPLE;
        if (step < 1) step = 1;
        bool first = true;
        for (int i = 0; i < adcPhNoise.nSamples && pos + 12 < bufSize; i += step) {
          P("%s%.3f", first ? "" : ",", rapidRawSamples[i] * ADS_MV_PER_BIT);
          first = false;
        }
      } else {
        Serial.printf("[DIAG] time_series skip: hwADS=%d rawPtr=%p nSamp=%d\n",
          hwADS1115, rapidRawSamples, adcPhNoise.nSamples);
      }
      P("]},");  // close ph_channel
      break;

    case 4: {  // Baseline + noise_ph
      if (adcBaselineSkipped) {
        P("\"baseline\":{\"skipped\":true},\"noise_ratio\":null,");
      } else {
        P("\"baseline\":{\"skipped\":false,\"label\":\"AIN1 (unconnected, ADC noise floor)\",");
        P("\"n_samples\":%d,\"rate_sps\":16,", adcBaselineNoise.nSamples);
        P("\"mean_mv\":%.3f,\"stddev_mv\":%.3f,\"ptp_mv\":%.3f},",
          adcBaselineNoise.meanMv, adcBaselineNoise.stddevMv, adcBaselineNoise.ptpMv);
        P("\"noise_ratio\":%.2f,", adcNoiseRatio);
        P("\"noise_ratio_desc\":\"pH circuit noise is %.1fx the ADC noise floor\",",
          adcNoiseRatio);
      }

      if (phNoiseValid) {
        const char* impact = (noiseStddevPH < 0.05f) ? "Negligible" :
                             (noiseStddevPH < 0.15f) ? "Low" :
                             (noiseStddevPH < 0.30f) ? "Moderate" : "High";
        P("\"noise_ph\":{\"valid\":true,\"stddev_ph\":%.3f,\"ptp_ph\":%.3f,",
          noiseStddevPH, noisePtpPH);
        P("\"slope_mv_per_ph\":%.1f,\"est_kh_pct\":%.1f,\"impact\":\"%s\"},",
          phSlope, noiseKhPct, impact);
      } else {
        P("\"noise_ph\":{\"valid\":false},");
      }
      break;
    }

    case 5:  // Multi-rate + datasheet comparison + close adc_noise
      if (adcMultiRateSkipped) {
        P("\"multi_rate\":{\"skipped\":true},");
      } else {
        P("\"multi_rate\":[");
        for (int i = 0; i < adcMultiRateCount; i++) {
          P("%s{\"sps\":%d,\"n\":%d,\"stddev_mv\":%.3f,\"ptp_mv\":%.3f,\"conv_ms\":%.1f}",
            i ? "," : "",
            adcMultiRate[i].sps, adcMultiRate[i].nSamples,
            adcMultiRate[i].stddevMv, adcMultiRate[i].ptpMv, adcMultiRate[i].convMs);
        }
        P("],");
      }
      if (hwADS1115) {
        float measuredUv = adcPhNoise.stddevMv * 1000.0f;
        float datasheetUv = 7.8f;
        P("\"datasheet_comparison\":{\"measured_uv\":%.1f,\"datasheet_uv\":%.1f,\"ratio\":%.1f,",
          measuredUv, datasheetUv, measuredUv / datasheetUv);
        P("\"note\":\"Ratio >10 indicates external circuit dominates noise\"}},");
      } else {
        P("\"datasheet_comparison\":null},");
      }
      break;

    case 6:  // Motor noise comparison
      if (motorNoiseSkipped) {
        P("\"motor_noise_comparison\":{\"skipped\":true},");
      } else {
        float deltaSd = motorOnNoise.stddevMv - motorOffNoise.stddevMv;
        float deltaPct = (motorOffNoise.stddevMv > 0.001f)
          ? (deltaSd / motorOffNoise.stddevMv) * 100.0f : 0;
        P("\"motor_noise_comparison\":{\"skipped\":false,");
        P("\"motors_off\":{\"n_samples\":%d,\"mean_mv\":%.3f,\"stddev_mv\":%.3f,\"ptp_mv\":%.3f},",
          motorOffNoise.nSamples, motorOffNoise.meanMv, motorOffNoise.stddevMv, motorOffNoise.ptpMv);
        P("\"motors_on\":{\"n_samples\":%d,\"mean_mv\":%.3f,\"stddev_mv\":%.3f,\"ptp_mv\":%.3f},",
          motorOnNoise.nSamples, motorOnNoise.meanMv, motorOnNoise.stddevMv, motorOnNoise.ptpMv);
        P("\"delta_stddev_mv\":%.3f,\"delta_pct\":%.1f},", deltaSd, deltaPct);
      }
      break;

    case 7:  // Internal ADC — board noise reference
      if (internalAdcNoise.nSamples > 0) {
        P("\"internal_adc\":{\"n_samples\":%d,", internalAdcNoise.nSamples);
        P("\"mean_mv\":%.1f,\"stddev_mv\":%.2f,\"min_mv\":%.1f,\"max_mv\":%.1f,\"ptp_mv\":%.1f,",
          internalAdcNoise.meanMv, internalAdcNoise.stddevMv,
          internalAdcNoise.minMv, internalAdcNoise.maxMv, internalAdcNoise.ptpMv);
        P("\"note\":\"ESP32 12-bit ADC on GPIO36 (VP, unconnected — board noise reference)\"},");
      } else {
        P("\"internal_adc\":{\"skipped\":true},");
      }
      break;

    case 8:  // Temperature
      if (tempSkipped) {
        P("\"temperature\":{\"skipped\":true},");
      } else {
        P("\"temperature\":{\"skipped\":false,\"present\":true,");
        P("\"readings\":[");
        for (int i = 0; i < tempReadingCount; i++) P("%s%.1f", i ? "," : "", tempReadings[i]);
        P("],\"mean_c\":%.2f,\"stddev_c\":%.3f,", tempMean, tempStddev);
        P("\"crc_errors\":%s,\"power_on_reset\":%s},",
          tempCrcError ? "true" : "false", tempPorDetected ? "true" : "false");
      }
      break;

    case 9:  // TMC + Motors
      if (tmcSkipped) {
        P("\"tmc_drivers\":{\"skipped\":true},\"motors\":{\"skipped\":true},");
      } else {
        // TMC drivers
        P("\"tmc_drivers\":{\"skipped\":false,\"detected\":true,");
        // DRV_STATUS bit decoding
        auto drvBits = [&](const char* name, uint32_t status, bool ioinOk) {
          P("\"%s\":{\"ioin_ok\":%s,", name, ioinOk ? "true" : "false");
          P("\"overtemp\":%s,", (status & (1<<1)) ? "true" : "false");       // ot
          P("\"overtemp_warning\":%s,", (status & (1<<0)) ? "true" : "false"); // otpw
          P("\"open_load_a\":%s,", (status & (1<<6)) ? "true" : "false");    // ola
          P("\"open_load_b\":%s,", (status & (1<<7)) ? "true" : "false");    // olb
          P("\"short_a\":%s,", (status & (1<<12)) ? "true" : "false");       // s2ga
          P("\"short_b\":%s}", (status & (1<<13)) ? "true" : "false");       // s2gb
        };
        drvBits("sample", tmcSampleDrvStatus, tmcSampleIoinOk);
        P(",");
        drvBits("titrate", tmcTitrateDrvStatus, tmcTitrateIoinOk);
        P("},");

        // Motors
        if (motorsSkipped) {
          P("\"motors\":{\"skipped\":true},");
        } else {
          P("\"motors\":{\"skipped\":false,");
          P("\"sample\":{\"stealthchop\":{\"sg_min\":%d,\"sg_max\":%d,\"sg_avg\":%.0f},",
            motorSampleSC.sgMin, motorSampleSC.sgMax, motorSampleSC.sgAvg);
          P("\"spreadcycle\":{\"sg_min\":%d,\"sg_max\":%d,\"sg_avg\":%.0f}},",
            motorSampleSP.sgMin, motorSampleSP.sgMax, motorSampleSP.sgAvg);
          P("\"titrate\":{\"stealthchop\":{\"sg_min\":%d,\"sg_max\":%d,\"sg_avg\":%.0f},",
            motorTitrateSC.sgMin, motorTitrateSC.sgMax, motorTitrateSC.sgAvg);
          P("\"spreadcycle\":{\"sg_min\":%d,\"sg_max\":%d,\"sg_avg\":%.0f}},",
            motorTitrateSP.sgMin, motorTitrateSP.sgMax, motorTitrateSP.sgAvg);
          // SG Profiles
          P("\"sg_profile\":{\"sample\":{\"min\":%d,\"max\":%d,\"mean\":%.1f,\"stddev\":%.1f,\"recommended_sg\":%d,\"values\":[",
            sgProfileSample.sgMin, sgProfileSample.sgMax, sgProfileSample.sgMean,
            sgProfileSample.sgStddev, sgProfileSample.recommendedStallSG);
          for (int i = 0; i < sgProfileSample.count; i++) {
            P("%s%d", i ? "," : "", sgProfileSample.sg[i]);
          }
          P("]},\"titrate\":{\"min\":%d,\"max\":%d,\"mean\":%.1f,\"stddev\":%.1f,\"recommended_sg\":%d,\"values\":[",
            sgProfileTitrate.sgMin, sgProfileTitrate.sgMax, sgProfileTitrate.sgMean,
            sgProfileTitrate.sgStddev, sgProfileTitrate.recommendedStallSG);
          for (int i = 0; i < sgProfileTitrate.count; i++) {
            P("%s%d", i ? "," : "", sgProfileTitrate.sg[i]);
          }
          P("]}}},"  );
        }
      }
      break;

    case 10:  // GPIO
      P("\"gpio\":{");
      for (int i = 0; i < gpioCount; i++) {
        P("%s\"%s\":{\"pin\":%d,\"state\":%d,\"expected\":%d,\"ok\":%s}",
          i ? "," : "",
          gpioStates[i].name, gpioStates[i].pin,
          gpioStates[i].state, gpioStates[i].expected,
          gpioStates[i].ok ? "true" : "false");
      }
      P("},");
      break;

    case 11: {  // Probe + close
      P("\"probe\":{\"voltage_mv\":%.1f,\"calibrated\":%s",
        probeVoltageMv, probeCalibrated ? "true" : "false");
      if (probeCalibrated) {
        char reasonBuf[80];
        const char* health = getProbeHealthDetail(reasonBuf, sizeof(reasonBuf));
        P(",\"health\":\"%s\",\"reason\":\"%s\"", health, reasonBuf);
        P(",\"cal_v4\":%.1f,\"cal_v7\":%.1f,\"cal_v10\":%.1f",
          voltage_4PH, voltage_7PH, voltage_10PH);
        P(",\"acid_eff_pct\":%.1f,\"alk_eff_pct\":%.1f,\"asymmetry_pct\":%.1f",
          getAcidEfficiency(), getAlkalineEfficiency(), getProbeAsymmetry());
        P(",\"avg_noise_mv\":%.2f,\"max_noise_mv\":%.2f",
          getAvgStabNoiseMv(), getMaxStabNoiseMv());
        uint32_t calTs = configStore.getCalTimestamp();
        int calAgeDays = 0;
        if (calTs > 0 && time(nullptr) > (time_t)calTs) {
          calAgeDays = (time(nullptr) - calTs) / 86400;
        }
        P(",\"cal_age_days\":%d,\"cal_timestamp\":%lu", calAgeDays, calTs);
      }
      P("}}");  // close probe + root

      // Free rapid samples if still allocated
      if (rapidRawSamples) {
        free(rapidRawSamples);
        rapidRawSamples = nullptr;
      }
      break;
    }
  }

  #undef P
  *written = pos;
  return section < SECTION_COUNT - 1;
}
