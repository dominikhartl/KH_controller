#include <Arduino.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <config.h>
#include <ArduinoJson.h>
#include <pins.h>
#include <time.h>

#include "motors.h"
#include "stirrer.h"
#include "measurement.h"
#include "config_store.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "scheduler.h"
#include "ha_discovery.h"
#include "web_server.h"

// --- Global state ---
int units = 0;
float startPH = 0;
bool discoveryPublished = false;
unsigned long lastDiagnosticsTime = 0;
unsigned long lastBroadcastTime = 0;

// Deferred command queue: WebSocket/MQTT sets the command, loop() executes it
// This prevents long operations from blocking the AsyncTCP task
static volatile char pendingCmd = 0;

// MQTT topic buffers
char topicCmd[50];
char MQmsg[50];
char MQerr[50];
char MQKH[50];
char MQstartpH[50];
char MQmespH[50];
char MQkhValue[50];
char MQconfidence[50];
char MQkhSlope[50];
char MQgranR2[50];
char MQcrossVal[50];
char MQdataPoints[50];
char MQmeasTime[50];

// --- Deferred command execution ---
// Long-running commands (measureKH, calibrate) must run on loopTask, not AsyncTCP.
// WebSocket/MQTT handlers set pendingCmd; loop() picks it up.
void queueCommand(char cmd) {
  pendingCmd = cmd;
}

void publishMessage(const char* message);
void publishError(const char* errorMessage);
void calibrateTitrationPump();

KHResult measureKH();
void measureKHWithValidation();

// Publish a finalized KH result to MQTT, config store, and history
static void publishKHResult(const KHResult& r) {
  storeLastKHResult(r);
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", r.khValue);
    mqttManager.publish(MQkhValue, mqBuf, true); }
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", r.startPH);
    mqttManager.publish(MQstartpH, mqBuf, true); }
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", r.confidence);
    mqttManager.publish(MQconfidence, mqBuf, true); }
  configStore.setLastKH(r.khValue);
  configStore.setLastStartPH(r.startPH);
  lastConfidence = r.confidence;
  uint32_t ts = (uint32_t)time(nullptr);
  appendHistory("kh", r.khValue, ts);
  appendHistory("ph", r.startPH, ts);
  appendGranHistory(r.granR2, r.hclUsed, r.endpointPH, r.usedGran, r.confidence, r.khGran, r.khEndpoint, r.probeNoiseMv, r.phReversals, configStore.getDropVolumeUL(), configStore.getTitrationRPM(), ts);

  // Quality metrics
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.4f", r.granR2);
    mqttManager.publish(MQgranR2, mqBuf, true); }
  if (!isnan(r.crossValDiff)) {
    char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.3f", r.crossValDiff);
    mqttManager.publish(MQcrossVal, mqBuf, true);
  }
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%d", r.dataPointCount);
    mqttManager.publish(MQdataPoints, mqBuf, true); }
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%lu", r.elapsedSec);
    mqttManager.publish(MQmeasTime, mqBuf, true); }

  // Compute and publish KH trend slope (dKH/day) from 72h history
  float slope = computeKHSlope();
  if (!isnan(slope)) {
    char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.3f", slope);
    mqttManager.publish(MQkhSlope, mqBuf, true);
  }

  // Update web UI with validated result
  broadcastState();
}

// Compute median of a float array (sorts in-place)
static float computeMedian(float* values, int count) {
  for (int i = 1; i < count; i++) {
    float key = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = key;
  }
  if (count % 2 == 0)
    return (values[count / 2 - 1] + values[count / 2]) / 2.0f;
  return values[count / 2];
}

// Check if a measurement is suspect (outlier or failed cross-validation)
static bool isSuspect(const KHResult& r, float median, bool hasMedian, char* reasonBuf, size_t reasonLen) {
  if (hasMedian) {
    float dev = fabsf(r.khValue - median);
    if (dev > KH_OUTLIER_THRESHOLD_DKH) {
      snprintf(reasonBuf, reasonLen, "Outlier: %.2f dKH (median %.2f, dev %.2f)",
               r.khValue, median, dev);
      return true;
    }
  }
  if (!isnan(r.crossValDiff) && r.crossValDiff > CROSS_VALIDATION_THRESHOLD_DKH) {
    snprintf(reasonBuf, reasonLen, "Cross-val failed (diff %.2f dKH)", r.crossValDiff);
    return true;
  }
  if (r.granR2 > 0 && r.granR2 < GRAN_MIN_R2) {
    snprintf(reasonBuf, reasonLen, "Poor Gran fit (R²=%.3f)", r.granR2);
    return true;
  }
  return false;
}

// Measure KH with outlier and cross-validation checks against recent history
void measureKHWithValidation() {
  // Read recent history BEFORE measuring (so current measurement is not included)
  float recent[10];
  int histCount = getRecentKHValues(recent, KH_OUTLIER_HISTORY_COUNT);

  KHResult r1 = measureKH();
  if (isnan(r1.khValue)) return;  // measurement failed
  storeLastKHResult(r1);  // store immediately so diagnostics always has data

  bool hasMedian = (histCount >= KH_OUTLIER_HISTORY_COUNT);
  float median = hasMedian ? computeMedian(recent, histCount) : 0;

  char reason[96];
  if (!isSuspect(r1, median, hasMedian, reason, sizeof(reason))) {
    publishKHResult(r1);
    return;
  }

  // Suspect measurement — re-measure
  char buf[128];
  snprintf(buf, sizeof(buf), "%s. Re-measuring...", reason);
  publishMessage(buf);

  KHResult r2 = measureKH();
  if (isnan(r2.khValue)) {
    publishMessage("Re-measurement failed, keeping first value");
    publishKHResult(r1);
    return;
  }

  if (!isSuspect(r2, median, hasMedian, reason, sizeof(reason))) {
    snprintf(buf, sizeof(buf), "Re-measurement %.2f dKH accepted", r2.khValue);
    publishMessage(buf);
    publishKHResult(r2);
    return;
  }

  // Both suspect — pick closest to median if available, else smaller cross-val diff
  bool pickFirst;
  if (hasMedian) {
    pickFirst = fabsf(r1.khValue - median) <= fabsf(r2.khValue - median);
  } else {
    float cv1 = isnan(r1.crossValDiff) ? 999.0f : r1.crossValDiff;
    float cv2 = isnan(r2.crossValDiff) ? 999.0f : r2.crossValDiff;
    pickFirst = cv1 <= cv2;
  }

  const KHResult& best = pickFirst ? r1 : r2;
  snprintf(buf, sizeof(buf), "Both suspect. Using %.2f dKH (better of two)", best.khValue);
  publishMessage(buf);
  publishKHResult(best);
}

void processPendingCommand() {
  char cmd = pendingCmd;
  if (cmd == 0) return;
  pendingCmd = 0;

  switch (cmd) {
    case 'k':
      publishMessage("Measuring KH!");
      measureKHWithValidation();
      broadcastState();
      break;
    case 't':
      calibrateTitrationPump();
      broadcastState();
      break;
    case '4':
      calibratePH(4);
      broadcastState();
      break;
    case '7':
      calibratePH(7);
      broadcastState();
      break;
    case 'A':  // pH 10 (can't store "10" in single char)
      calibratePH(10);
      broadcastState();
      break;
    case 'p':
      startStirrer();
      delay(STIRRER_WARMUP_MS);
      measurePH(100);
      stopStirrer();
      if (isnan(pH)) {
        publishError("Error: pH probe not working");
      } else {
        char phBuf[16];
        snprintf(phBuf, sizeof(phBuf), "%.2f", pH);
        mqttManager.publish(MQmespH, phBuf, true);
        snprintf(phBuf, sizeof(phBuf), "pH: %.2f", pH);
        publishMessage(phBuf);
      }
      broadcastState();
      break;
  }
}

// --- Publish helpers ---

void publishError(const char* errorMessage) {
  if (!errorMessage || errorMessage[0] == '\0') return;
  mqttManager.publish(MQerr, errorMessage);
  broadcastError(errorMessage);
}

void publishMessage(const char* message) {
  mqttManager.publish(MQmsg, message);
  broadcastMessage(message);
}

// --- HCl tracking helper ---
void subtractHCl(int unitsUsed) {
  float calUnits = (float)configStore.getCalUnits();
  if (calUnits <= 0) {
    publishError("Error: calUnits is zero, cannot track HCl!");
    return;
  }
  float titVol = configStore.getTitrationVolume();
  float hclVol = configStore.getHClVolume();
  float used = ((float)unitsUsed / calUnits) * titVol;
  float remaining = hclVol - used;
  if (remaining < 0) remaining = 0;
  configStore.setHClVolume(remaining);
}

// --- Pump calibration ---

void calibrateTitrationPump() {
  publishMessage("Calibrating pump");

  units = 0;
  const int BATCH = 40;
  char buf[48];

  while (units < CALIBRATION_TARGET_UNITS) {
    int batch = min(BATCH, CALIBRATION_TARGET_UNITS - units);
    if (!titrate(batch, TITRATION_RPM)) {
      publishError("Error: titration pump timeout during calibration");
      digitalWrite(EN_PIN2, HIGH);
      return;
    }
    units += batch;
    delay(TITRATION_MIX_DELAY_FAST_MS);
    ArduinoOTA.handle();
    snprintf(buf, sizeof(buf), "Cal: %d / %d", units, CALIBRATION_TARGET_UNITS);
    publishMessage(buf);
  }

  subtractHCl(CALIBRATION_TARGET_UNITS);
  units = 0;
  digitalWrite(EN_PIN2, HIGH);
  publishMessage("Pump calibration done");
}

// --- Measurement confidence score ---
// Combines multiple quality signals into a 0.0-1.0 score
static float computeConfidence(float granR2, bool usedGran, int nPoints,
                                int stabTimeouts, const char* probeHealth,
                                float crossValDiff) {
  float score = 1.0f;
  // Gran R² contribution
  if (usedGran && granR2 > 0) {
    score -= (1.0f - granR2) * 40.0f;  // R²=0.99 → -0.4, R²=0.999 → -0.04
  }
  // Cross-validation: Gran vs Endpoint disagreement
  if (!isnan(crossValDiff)) {
    if (crossValDiff > 0.5f) score -= 0.3f;
    else if (crossValDiff > 0.3f) score -= 0.15f;
    else if (crossValDiff > 0.15f) score -= 0.05f;
  }
  // Data point count
  if (nPoints < 15) score -= 0.1f;
  if (nPoints < 10) score -= 0.1f;
  // Stabilization timeout penalty
  if (stabTimeouts > 0) score -= 0.1f;
  if (stabTimeouts > 3) score -= 0.1f;
  // Probe health penalty
  if (strcmp(probeHealth, "Fair") == 0) score -= 0.1f;
  else if (strcmp(probeHealth, "Replace") == 0) score -= 0.2f;
  if (score < 0.0f) score = 0.0f;
  return score;
}

// --- Adaptive fast-phase batch size ---
// Reduces batch as pH approaches the fast/precise threshold to avoid overshoot
static int computeFastBatch(float currentPH, float thresholdPH) {
  if (currentPH >= FAST_RAMP_START_PH) return FAST_BATCH_MAX;
  float lower = thresholdPH + 0.5f;
  if (currentPH <= lower) return FAST_BATCH_MIN;
  float fraction = (currentPH - lower) / (FAST_RAMP_START_PH - lower);
  return FAST_BATCH_MIN + (int)(fraction * (FAST_BATCH_MAX - FAST_BATCH_MIN));
}

// --- MeasureKH ---

KHResult measureKH() {
  KHResult result = {};
  result.khValue = NAN;
  result.crossValDiff = NAN;

  // Re-entrancy guard: prevent concurrent measurements
  static bool measuring = false;
  if (measuring) {
    publishError("Measurement already in progress");
    return result;
  }
  measuring = true;

  // Configure stabilization from NVS and reset per-measurement stats
  setStabilizationTimeoutMs(configStore.getStabilizationTimeout());
  resetStabilizationStats();
  resetNoiseStats();

  broadcastTitrationStart();  // Signal dashboard to clear live pH chart
  int errorflag = 0;
  units = 0;
  unsigned long measStartMs = millis();
  const char* errorMessage = "";
  publishError("");  // Clear previous error

  // Track WiFi RSSI range during measurement
  int8_t rssiMin = 0, rssiMax = -127;

  // Compute prefill volume in units from µL config
  float prefillUL = configStore.getPrefillVolumeUL();
  float calU = (float)configStore.getCalUnits();
  float titV = configStore.getTitrationVolume();
  int prefillUnits = max(2, (int)round(prefillUL * calU / (titV * 1000.0f)));

  // Validate calibration before starting
  if (!isCalibrationValid()) {
    publishError("Error: pH calibration invalid. Re-calibrate with pH 4/7/10 buffers.");
    measuring = false;
    return result;
  }

  if (!titrate(prefillUnits, configStore.getTitrationRPM(), true)) {
    publishError("Error: titration pump timeout during prefill");
    measuring = false;
    return result;
  }
  digitalWrite(EN_PIN2, HIGH);
  publishMessage("Taking sample");
  // Double wash: first rinse cleans the chamber, second takes the actual sample
  setMultiWashContext(2);
  if (!washSample(1.2, 1.0)) {
    clearMultiWashContext();
    publishError("Error: sample pump timeout during wash (1st rinse)");
    measuring = false;
    return result;
  }
  delay(1000);
  if (!washSample(1.2, 1.0)) {
    clearMultiWashContext();
    publishError("Error: sample pump timeout during wash (2nd rinse)");
    measuring = false;
    return result;
  }
  clearMultiWashContext();
  delay(100);
  startStirrer();
  delay(STIRRER_WARMUP_MS);  // Wait for solution to homogenize
  measurePH(100);
  float minStartPH = configStore.getMinStartPH();
  if (isnan(pH)) {
    errorMessage = "Error: pH probe not working";
    stopStirrer();
    digitalWrite(EN_PIN2, HIGH);
    errorflag = 1;
  } else if (pH < CARRYOVER_RETRY_PH) {
    startPH = pH;
    errorMessage = "Error: Starting pH critically low (acid carryover)";
    stopStirrer();
    digitalWrite(EN_PIN2, HIGH);
    errorflag = 1;
  } else if (pH < minStartPH) {
    // Possible carryover — attempt one extra rinse
    static char retryBuf[80];
    snprintf(retryBuf, sizeof(retryBuf), "Warning: Starting pH %.2f < %.1f, extra rinse...", pH, minStartPH);
    publishMessage(retryBuf);
    stopStirrer();
    setMultiWashContext(2);
    washSample(1.5, 1.0);
    delay(2000);
    washSample(1.2, 1.0);
    clearMultiWashContext();
    delay(100);
    startStirrer();
    delay(STIRRER_WARMUP_MS);
    measurePH(100);
    if (isnan(pH) || pH < minStartPH) {
      startPH = isnan(pH) ? 0 : pH;
      snprintf(retryBuf, sizeof(retryBuf), "Error: Starting pH still %.2f after rinse", pH);
      errorMessage = retryBuf;
      stopStirrer();
      digitalWrite(EN_PIN2, HIGH);
      errorflag = 1;
    } else {
      snprintf(retryBuf, sizeof(retryBuf), "Extra rinse recovered pH to %.2f", pH);
      publishMessage(retryBuf);
    }
  }

  if (errorflag == 0) {
    startPH = pH;
    configStore.setLastStartPH(startPH);
    broadcastState();
    mqttManager.loop();

    // Check probe health and calibration age
    char reason[96];
    const char* health = getProbeHealthDetail(reason, sizeof(reason));
    if (strcmp(health, "Good") != 0) {
      char hBuf[128];
      snprintf(hBuf, sizeof(hBuf), "Warning: Probe %s — %s", health, reason);
      publishError(hBuf);
    }
    uint32_t calTs = configStore.getCalTimestamp();
    time_t now = time(nullptr);
    if (calTs > 0 && now > MIN_VALID_EPOCH) {
      int calAgeDays = (int)((now - calTs) / 86400);
      if (calAgeDays > CALIBRATION_AGE_WARNING_DAYS) {
        char aBuf[64];
        snprintf(aBuf, sizeof(aBuf), "Warning: Calibration is %d days old", calAgeDays);
        publishError(aBuf);
      }
    }

    float fastPH = configStore.getFastTitrationPH();

    // --- Fast phase: adaptive batch size, reduces near threshold to avoid overshoot ---
    float lastFastPH = startPH;
    int stallCount = 0;
    publishMessage("Fast titration");

    // Data point storage — declared early so fast-phase can store points near endpoint
    static TitrationPoint dataPoints[MAX_TITRATION_POINTS];
    int nPoints = 0;
    int granCount = 0;
    static const float DATA_STORE_PH = 5.0f;

    while (pH > fastPH && units < MAX_TITRATION_UNITS && errorflag == 0) {
      int batch = computeFastBatch(pH, fastPH);
      if (!titrate(batch, TITRATION_RPM)) {
        errorMessage = "Error: titration pump timeout in fast phase";
        errorflag = 1;
        break;
      }
      units += batch;
      delay(TITRATION_MIX_DELAY_FAST_MS);
      measurePHFast(5);
      broadcastTitrationPH(pH, units);
      mqttManager.loop();
      ArduinoOTA.handle();
      { int8_t rssi = wifiManager.getRSSI();
        if (rssi < rssiMin) rssiMin = rssi;
        if (rssi > rssiMax) rssiMax = rssi; }

      if (isnan(pH)) {
        errorMessage = "Error: pH probe not working!";
        errorflag = 1;
      } else {
        // Store data points near the endpoint even during fast phase
        if (pH < DATA_STORE_PH && nPoints < MAX_TITRATION_POINTS) {
          dataPoints[nPoints++] = {(float)units, pH, voltage, 0, 0, 0};
        }
        if (pH < GRAN_REGION_PH) granCount++;

        if (pH > lastFastPH - 0.02) {
          stallCount++;
          if (stallCount >= 20) {
            errorMessage = "Error: insufficient pH change";
            errorflag = 1;
          }
        } else {
          stallCount = 0;
          lastFastPH = pH;
        }
      }
    }

    // --- Precise phase: adaptive steps with Gran transformation ---

    if (errorflag == 0) {
      char buf[32];
      snprintf(buf, sizeof(buf), "Precise phase (pH %.1f)", pH);
      publishMessage(buf);
    }

    float lastPrecisePH = pH;
    int preciseStall = 0;

    // Gran zone noise tracking
    float prevGranPH = NAN;
    int phReversals = 0;
    int granStepCount = 0;
    float stepDeltaSum = 0;

    uint8_t epMethod = configStore.getEndpointMethod();
    float stopPH = (epMethod == 1) ? FIXED_ENDPOINT_STOP_PH : GRAN_STOP_PH;

    while (!isnan(pH) && pH > stopPH && units < MAX_TITRATION_UNITS
           && errorflag == 0) {
      mqttManager.loop();

      int stepVol;
      uint8_t curPhase;
      if (pH > GRAN_REGION_PH) {
        // Medium zone (pH above Gran region): large steps, rough tracking only
        // No stabilization needed — we just need to detect when to enter Gran zone
        curPhase = 1;
        stepVol = TITRATION_STEP_SIZE * MEDIUM_STEP_MULTIPLIER;  // 48 units
        if (!titrate(stepVol, TITRATION_RPM)) {
          errorMessage = "Error: titration pump timeout in precise phase";
          errorflag = 1;
          break;
        }
        delay(TITRATION_MIX_DELAY_MEDIUM_MS);
        measurePHStabilized(8);
      } else {
        // Gran zone (pH below GRAN_REGION_PH): smaller steps, stabilization, accurate readings
        curPhase = 2;
        // Compute step volume from configurable drop size (µL → units via calibration)
        float dropUL = configStore.getDropVolumeUL();
        float calU = (float)configStore.getCalUnits();
        float titV = configStore.getTitrationVolume();
        float unitsPerUL = calU / (titV * 1000.0f);
        stepVol = max(2, (int)round(dropUL * unitsPerUL));
        float granRPM = configStore.getTitrationRPM();
        if (!titrate(stepVol, granRPM)) {
          errorMessage = "Error: titration pump timeout in Gran zone";
          errorflag = 1;
          break;
        }
        delay(configStore.getGranMixDelay());
        waitForPHStabilization();
        measurePHStabilized(20);
      }
      units += stepVol;

      // Store data points near the endpoint for Gran analysis and interpolation
      if (pH < DATA_STORE_PH && nPoints < MAX_TITRATION_POINTS) {
        uint16_t sMs = (curPhase == 2) ? (uint16_t)min((unsigned long)0xFFFF, getLastStabilizationMs()) : (uint16_t)0;
        uint8_t fl = (curPhase == 2 && getLastStabilizationTimedOut()) ? 1 : 0;
        dataPoints[nPoints++] = {(float)units, pH, voltage, sMs, curPhase, fl};
      }
      if (pH < GRAN_REGION_PH) granCount++;

      // Track step-to-step noise in Gran zone
      if (curPhase == 2 && !isnan(pH)) {
        if (!isnan(prevGranPH)) {
          float delta = prevGranPH - pH;  // Expected positive (pH decreasing)
          stepDeltaSum += fabsf(delta);
          if (delta < 0) phReversals++;   // pH went UP after acid = reversal
        }
        prevGranPH = pH;
        granStepCount++;
      }

      mqttManager.loop();
      ArduinoOTA.handle();
      { char phBuf[16]; snprintf(phBuf, sizeof(phBuf), "%.2f", pH);
        mqttManager.publish(MQmespH, phBuf); }
      broadcastTitrationPH(pH, units);
      { int8_t rssi = wifiManager.getRSSI();
        if (rssi < rssiMin) rssiMin = rssi;
        if (rssi > rssiMax) rssiMax = rssi; }

      if (isnan(pH)) {
        errorMessage = "Error: pH probe not working!";
        errorflag = 1;
      } else if (pH > lastPrecisePH - 0.01f) {
        preciseStall++;
        if (preciseStall >= 50) {
          errorMessage = "Error: pH stalled in precise phase";
          errorflag = 1;
        }
      } else {
        preciseStall = 0;
        lastPrecisePH = pH;
      }
      if (units >= MAX_TITRATION_UNITS - stepVol) {
        errorMessage = "Error: reached acid max!";
        errorflag = 1;
      }
    }

    mqttManager.loop();
    if (errorflag == 0) {
      // Get calibration parameters
      float calUnits = (float)configStore.getCalUnits();
      float titVol = configStore.getTitrationVolume();
      float samVol = configStore.getSampleVolume();
      float corrF = configStore.getCorrectionFactor();
      float hclMol = configStore.getHClMolarity();

      if (calUnits <= 0 || samVol <= 0) {
        errorMessage = "Error: invalid calibration (calUnits or samVol is zero)";
        errorflag = 1;
      } else {
        // Determine equivalence point
        float granR2 = 0;
        float exactUnits;
        bool usedGran = false;

        if (epMethod == 0) {
          // Gran mode: try Gran analysis, fall back to endpoint if it fails
          if (granCount >= MIN_GRAN_POINTS) {
            char granReason[64] = "";
            exactUnits = granAnalysis(dataPoints, nPoints, samVol, titVol, calUnits, &granR2, granReason, sizeof(granReason));
            if (!isnan(exactUnits)) {
              usedGran = true;
            } else {
              char failBuf[128];
              snprintf(failBuf, sizeof(failBuf), "Gran failed: %s. Falling back to endpoint.", granReason);
              publishMessage(failBuf);
              exactUnits = interpolateAtPH(dataPoints, nPoints, ENDPOINT_PH);
              if (isnan(exactUnits)) {
                errorMessage = "Error: both Gran and endpoint interpolation failed";
                errorflag = 1;
              }
            }
          } else {
            publishMessage("Insufficient Gran points. Falling back to endpoint.");
            exactUnits = interpolateAtPH(dataPoints, nPoints, ENDPOINT_PH);
            if (isnan(exactUnits)) {
              errorMessage = "Error: insufficient data for any method";
              errorflag = 1;
            }
          }
        } else {
          // Fixed endpoint mode: interpolate at ENDPOINT_PH
          exactUnits = interpolateAtPH(dataPoints, nPoints, ENDPOINT_PH);
          if (isnan(exactUnits)) {
            errorMessage = "Error: could not interpolate at endpoint pH";
            errorflag = 1;
          }
          publishMessage("Fixed endpoint mode");
        }

        // Broadcast Gran diagnostics to web dashboard
        {
          float k = titVol / calUnits;
          static const int MAX_GRAN_DIAG = 50;
          float granPtML[MAX_GRAN_DIAG];
          float granPtF[MAX_GRAN_DIAG];
          int nGranPts = 0;
          for (int i = 0; i < nPoints && nGranPts < MAX_GRAN_DIAG; i++) {
            if (dataPoints[i].pH < GRAN_REGION_PH && dataPoints[i].pH > GRAN_STOP_PH) {
              granPtML[nGranPts] = dataPoints[i].units * k;
              granPtF[nGranPts] = (samVol + dataPoints[i].units * k) * powf(10.0f, -dataPoints[i].pH);
              nGranPts++;
            }
          }
          float eqML = isnan(exactUnits) ? 0 : exactUnits * k;
          broadcastGranData(granR2, eqML, usedGran, granPtML, granPtF, nGranPts);
        }

        if (isnan(exactUnits)) {
          errorMessage = "Error: could not determine equivalence point";
          errorflag = 1;
        } else {
          float hclUsed = (exactUnits / calUnits) * titVol;
          float khValue = (hclUsed / samVol) * 2800.0f * hclMol * corrF;
          subtractHCl(units + prefillUnits);

          // Interpolate pH at equivalence point
          float endpointPHVal = ENDPOINT_PH;
          if (usedGran) {
            for (int i = 1; i < nPoints; i++) {
              if (dataPoints[i-1].units <= exactUnits && dataPoints[i].units >= exactUnits) {
                float frac = (exactUnits - dataPoints[i-1].units) / (dataPoints[i].units - dataPoints[i-1].units);
                endpointPHVal = dataPoints[i-1].pH + frac * (dataPoints[i].pH - dataPoints[i-1].pH);
                break;
              }
            }
          }

          // Publish informational messages (endpoint info, method, HCl warning)
          char volBuf[80];
          if (usedGran) {
            snprintf(volBuf, sizeof(volBuf), "Endpoint: %.2f mL @ pH %.2f (Gran, R²=%.3f)", hclUsed, endpointPHVal, granR2);
          } else {
            snprintf(volBuf, sizeof(volBuf), "Endpoint: %.2f mL @ pH %.1f (interpolation)", hclUsed, ENDPOINT_PH);
          }
          publishMessage(volBuf);

          float remainingHCl = configStore.getHClVolume();
          if (remainingHCl <= 0) {
            publishError("Warning: HCl supply empty! Refill needed.");
          } else if (remainingHCl < HCL_LOW_THRESHOLD_ML) {
            char warnBuf[64];
            snprintf(warnBuf, sizeof(warnBuf), "Warning: HCl low (%.0f mL remaining)", remainingHCl);
            publishError(warnBuf);
          }

          // Always compute both Gran and endpoint KH for cross-validation and CSV
          float khGran = NAN;
          float khEp = NAN;
          float crossValDiff = NAN;

          if (usedGran) {
            khGran = khValue;
            float epUnits = interpolateAtPH(dataPoints, nPoints, ENDPOINT_PH);
            if (!isnan(epUnits)) {
              float epHclUsed = (epUnits / calUnits) * titVol;
              khEp = (epHclUsed / samVol) * 2800.0f * hclMol * corrF;
              crossValDiff = fabsf(khGran - khEp);
              char cvBuf[80];
              snprintf(cvBuf, sizeof(cvBuf), "Cross-val: Gran=%.2f Endpoint=%.2f diff=%.2f dKH",
                       khGran, khEp, crossValDiff);
              publishMessage(cvBuf);
            }
          } else {
            khEp = khValue;
            // Try Gran even when endpoint method is primary (for CSV record)
            if (granCount >= MIN_GRAN_POINTS) {
              float tryR2 = 0;
              char granReason2[64] = "";
              float granUnits2 = granAnalysis(dataPoints, nPoints, samVol, titVol, calUnits, &tryR2, granReason2, sizeof(granReason2));
              if (!isnan(granUnits2)) {
                granR2 = tryR2;
                float granHcl = (granUnits2 / calUnits) * titVol;
                khGran = (granHcl / samVol) * 2800.0f * hclMol * corrF;
                crossValDiff = fabsf(khGran - khEp);
              }
            }
          }

          // Populate result struct — publishing deferred to measureKHWithValidation()
          result.khValue = khValue;
          result.khGran = khGran;
          result.khEndpoint = khEp;
          result.startPH = startPH;
          result.hclUsed = hclUsed;
          result.granR2 = granR2;
          result.endpointPH = endpointPHVal;
          result.usedGran = usedGran;
          result.crossValDiff = crossValDiff;
          result.dataPointCount = nPoints;
          storeAnalysisPoints(dataPoints, nPoints);
          result.stabTimeouts = getStabilizationTimeoutCount();
          result.elapsedSec = (millis() - measStartMs) / 1000;
          result.rssiMin = rssiMin;
          result.rssiMax = rssiMax;
          result.probeNoiseMv = getAvgStabNoiseMv();
          result.stepNoisePh = (granStepCount > 1) ? stepDeltaSum / (granStepCount - 1) : 0;
          result.phReversals = phReversals;
          result.granStepCount = granStepCount;
          result.confidence = computeConfidence(granR2, usedGran, nPoints,
                                                 result.stabTimeouts, getProbeHealth(),
                                                 crossValDiff);

          // KH value deferred to publishKHResult() after validation
        }
      }
    } else {
      publishError(errorMessage);
    }
  }
  // Anti-suckback: small reverse to prevent drip from titration nozzle
  unsigned int suckbackUs = (unsigned int)rpmToHalfPeriodUs(MOTOR_START_RPM);
  digitalWrite(DIR_PIN2, HIGH);  // Reverse
  for (int i = 0; i < ANTI_SUCKBACK_STEPS; i++) {
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(suckbackUs);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(suckbackUs);
  }
  digitalWrite(DIR_PIN2, LOW);   // Restore forward
  digitalWrite(EN_PIN2, HIGH);

  stopStirrer();

  // Compute extra removal to compensate for HCl volume added during titration
  // hclPart = HCl volume / sample volume (in mL/mL), adds to removal fraction
  float hclPart = 0;
  {
    float calU = (float)configStore.getCalUnits();
    float titV = configStore.getTitrationVolume();
    float samV = configStore.getSampleVolume();
    if (calU > 0 && samV > 0) {
      hclPart = ((float)(units + prefillUnits) / calU) * titV / samV;
    }
  }

  // Single post-wash rinse (pre-measurement double wash handles carryover)
  if (!washSample(1.5f + hclPart, 1.0)) {
    publishError("Warning: sample pump timeout during post-wash");
  }

  // Post-wash pH verification: quick check to detect incomplete wash
  startStirrer();
  delay(1000);
  measurePHFast(8);
  stopStirrer();
  if (!isnan(pH) && pH < POST_WASH_PH_THRESHOLD) {
    char postBuf[80];
    snprintf(postBuf, sizeof(postBuf), "Warning: Post-wash pH %.2f (possible incomplete wash)", pH);
    publishError(postBuf);
  }

  unsigned long elapsed = (millis() - measStartMs) / 1000;
  char doneBuf[48];
  if (errorflag == 0) {
    snprintf(doneBuf, sizeof(doneBuf), "Done! (%lum %lus)", elapsed / 60, elapsed % 60);
  } else {
    snprintf(doneBuf, sizeof(doneBuf), "Finished with errors (%lum %lus)", elapsed / 60, elapsed % 60);
  }
  publishMessage(doneBuf);
  measuring = false;
  return result;
}

// --- MQTT callback ---

void onMqttMessage(char* topic, byte* message, unsigned int length) {
  String messageM;
  for (unsigned int i = 0; i < length; i++) {
    messageM += (char)message[i];
  }

  // HA birth message - re-publish everything when HA restarts
  if (strcmp(topic, "homeassistant/status") == 0 && messageM == "online") {
    discoveryPublished = false;  // triggers re-publish in loop()
    return;
  }

  // Route config/set messages to HA discovery handler
  String topicStr = String(topic);
  if (topicStr.indexOf("/config/") >= 0 && topicStr.endsWith("/set")) {
    handleConfigSet(topic, messageM.c_str());
    return;
  }

  if (topicStr == topicCmd) {
    executeCommand(messageM.c_str());
  }
}

// --- Setup ---

void setup() {
  pinMode(EN_PIN1, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(EN_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STIRRER_PIN, OUTPUT);
  pinMode(PH_PIN, INPUT);
  Serial.begin(115200);

  // Initialize calibrated ADC
  initADC();

  // Initialize config store (NVS) and migrate from EEPROM if needed
  configStore.begin();

  // Keep MQTT/OTA alive during long motor operations (washSample takes ~16 min)
  setMotorYieldCallback([]() {
    mqttManager.loop();
    ArduinoOTA.handle();
  });

  // Report wash progress to web dashboard (WebSocket only, no MQTT during motor ops)
  setMotorProgressCallback([](int percent) {
    broadcastProgress(percent);
  });

  // Build MQTT topic strings
  snprintf(topicCmd, sizeof(topicCmd), "%s/cmd", DEVICE_NAME);
  snprintf(MQmsg, sizeof(MQmsg), "%s/message", DEVICE_NAME);
  snprintf(MQerr, sizeof(MQerr), "%s/error", DEVICE_NAME);
  snprintf(MQKH, sizeof(MQKH), "%s/KH", DEVICE_NAME);
  snprintf(MQstartpH, sizeof(MQstartpH), "%s/startPH", DEVICE_NAME);
  snprintf(MQmespH, sizeof(MQmespH), "%s/mes_pH", DEVICE_NAME);
  snprintf(MQkhValue, sizeof(MQkhValue), "%s/kh_value", DEVICE_NAME);
  snprintf(MQconfidence, sizeof(MQconfidence), "%s/confidence", DEVICE_NAME);
  snprintf(MQkhSlope, sizeof(MQkhSlope), "%s/kh_slope", DEVICE_NAME);
  snprintf(MQgranR2, sizeof(MQgranR2), "%s/gran_r2", DEVICE_NAME);
  snprintf(MQcrossVal, sizeof(MQcrossVal), "%s/cross_val_diff", DEVICE_NAME);
  snprintf(MQdataPoints, sizeof(MQdataPoints), "%s/data_points", DEVICE_NAME);
  snprintf(MQmeasTime, sizeof(MQmeasTime), "%s/meas_time", DEVICE_NAME);

  // Non-blocking WiFi
  wifiManager.begin(WIFI_SSID, WIFI_PASSWORD);

  // MQTT with LWT
  mqttManager.begin();
  mqttManager.setCallback(onMqttMessage);
  mqttManager.subscribe(topicCmd);
  mqttManager.subscribe("homeassistant/status");
  mqttManager.onDisconnect([]() {
    discoveryPublished = false;  // Re-publish discovery on reconnect
  });

  // OTA setup
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  // Load pH calibration from NVS
  voltage_4PH = configStore.getVoltage4PH();
  voltage_7PH = configStore.getVoltage7PH();
  voltage_10PH = configStore.getVoltage10PH();

  // Compute linear fit from calibration values
  updateCalibrationFit();

  // Web server + WebSocket dashboard
  setupWebServer();
  publishMessage("BOOT");

  // Scheduler with NTP
  scheduler.begin();
  scheduler.onMeasurementDue([]() {
    mqttManager.publish(MQmsg, "Scheduled measurement starting");
    measureKHWithValidation();
  });
}

// --- Main loop ---

void loop() {
  processPendingCommand();
  wifiManager.loop();
  mqttManager.loop();
  ArduinoOTA.handle();
  scheduler.loop();
  ws.cleanupClients();

  // Publish HA Discovery on first MQTT connect or after HA restart
  if (mqttManager.isConnected() && !discoveryPublished) {
    publishAllDiscovery();
    publishAllConfigStates();

    // Re-publish last measurement values so HA has them immediately
    float lastKH = configStore.getLastKH();
    float lastSPH = configStore.getLastStartPH();
    if (lastKH > 0) { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", lastKH);
      mqttManager.publish(MQkhValue, mqBuf, true); }
    if (lastSPH > 0) { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", lastSPH);
      mqttManager.publish(MQstartpH, mqBuf, true); }

    discoveryPublished = true;
  }

  // Broadcast state to WebSocket clients every 2s
  if (millis() - lastBroadcastTime > 2000) {
    lastBroadcastTime = millis();
    broadcastState();
  }

  // Publish diagnostics every 60s
  if (mqttManager.isConnected() && millis() - lastDiagnosticsTime > 60000) {
    lastDiagnosticsTime = millis();
    publishDiagnostics();
  }

}
