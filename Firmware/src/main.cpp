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

// --- Deferred command execution ---
// Long-running commands (measureKH, calibrate) must run on loopTask, not AsyncTCP.
// WebSocket/MQTT handlers set pendingCmd; loop() picks it up.
void queueCommand(char cmd) {
  pendingCmd = cmd;
}

void publishMessage(const char* message);
void calibrateTitrationPump();

struct KHResult {
  float khValue;       // NAN on error
  float startPH;
  float hclUsed;
  float granR2;
  float endpointPH;
  bool usedGran;
  float confidence;    // 0.0-1.0 composite quality score
  int dataPointCount;
  int stabTimeouts;
  unsigned long elapsedSec;
};

KHResult measureKH();
void measureKHWithValidation();

// Publish a finalized KH result to MQTT, config store, and history
static void publishKHResult(const KHResult& r) {
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", r.khValue);
    mqttManager.publish(MQkhValue, mqBuf, true); }
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", r.confidence);
    mqttManager.publish(MQconfidence, mqBuf, true); }
  configStore.setLastKH(r.khValue);
  configStore.setLastStartPH(r.startPH);
  appendHistory("kh", r.khValue);
  appendHistory("ph", r.startPH);
  appendGranHistory(r.granR2, r.hclUsed, r.endpointPH, r.usedGran);
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

// Measure KH with outlier validation against recent history
void measureKHWithValidation() {
  // Read recent history BEFORE measuring (so current measurement is not included)
  float recent[10];
  int histCount = getRecentKHValues(recent, KH_OUTLIER_HISTORY_COUNT);

  KHResult r1 = measureKH();
  if (isnan(r1.khValue)) return;  // measurement failed

  // Skip validation if insufficient history
  if (histCount < KH_OUTLIER_HISTORY_COUNT) {
    publishKHResult(r1);
    return;
  }

  float median = computeMedian(recent, histCount);
  float dev1 = fabsf(r1.khValue - median);

  if (dev1 <= KH_OUTLIER_THRESHOLD_DKH) {
    publishKHResult(r1);
    return;
  }

  // Outlier detected — re-measure
  char buf[96];
  snprintf(buf, sizeof(buf), "Outlier: %.2f dKH (median %.2f, dev %.2f). Re-measuring...",
           r1.khValue, median, dev1);
  publishMessage(buf);

  KHResult r2 = measureKH();
  if (isnan(r2.khValue)) {
    publishMessage("Re-measurement failed, keeping first value");
    publishKHResult(r1);
    return;
  }

  float dev2 = fabsf(r2.khValue - median);

  if (dev2 <= KH_OUTLIER_THRESHOLD_DKH) {
    snprintf(buf, sizeof(buf), "Re-measurement %.2f dKH accepted (within range)", r2.khValue);
    publishMessage(buf);
    publishKHResult(r2);
    return;
  }

  // Both outliers — pick closest to median
  if (dev1 <= dev2) {
    snprintf(buf, sizeof(buf), "Both outliers. Using first (%.2f, closer to median %.2f)", r1.khValue, median);
    publishMessage(buf);
    publishKHResult(r1);
  } else {
    snprintf(buf, sizeof(buf), "Both outliers. Using second (%.2f, closer to median %.2f)", r2.khValue, median);
    publishMessage(buf);
    publishKHResult(r2);
  }
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
                                int stabTimeouts, const char* probeHealth) {
  float score = 1.0f;
  // Gran R² contribution
  if (usedGran && granR2 > 0) {
    score -= (1.0f - granR2) * 40.0f;  // R²=0.99 → -0.4, R²=0.999 → -0.04
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

  broadcastTitrationStart();  // Signal dashboard to clear live pH chart
  int errorflag = 0;
  units = 0;
  unsigned long measStartMs = millis();
  const char* errorMessage = "";
  publishError("");  // Clear previous error

  // Validate calibration before starting
  if (!isCalibrationValid()) {
    publishError("Error: pH calibration invalid. Re-calibrate with pH 4/7/10 buffers.");
    measuring = false;
    return result;
  }

  if (!titrate(FILL_VOLUME, PREFILL_RPM)) {
    publishError("Error: titration pump timeout during prefill");
    measuring = false;
    return result;
  }
  digitalWrite(EN_PIN2, HIGH);
  publishMessage("Taking sample");
  if (!washSample(1.2, 1.0)) {
    publishError("Error: sample pump timeout during wash");
    measuring = false;
    return result;
  }
  delay(100);
  startStirrer();
  delay(STIRRER_WARMUP_MS);  // Wait for solution to homogenize
  measurePH(40);
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
    washSample(1.5, 1.0);
    delay(2000);
    washSample(1.2, 1.0);
    delay(100);
    startStirrer();
    delay(STIRRER_WARMUP_MS);
    measurePH(40);
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
    if (calTs > 0 && now > 1000000000) {
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

      if (isnan(pH)) {
        errorMessage = "Error: pH probe not working!";
        errorflag = 1;
      } else {
        // Store data points near the endpoint even during fast phase
        if (pH < DATA_STORE_PH && nPoints < MAX_TITRATION_POINTS) {
          dataPoints[nPoints++] = {(float)units, pH};
        }
        if (pH < GRAN_REGION_PH) granCount++;

        if (pH > lastFastPH - 0.05) {
          stallCount++;
          if (stallCount >= 10) {
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

    uint8_t epMethod = configStore.getEndpointMethod();
    float stopPH = (epMethod == 1) ? FIXED_ENDPOINT_STOP_PH : GRAN_STOP_PH;

    while (!isnan(pH) && pH > stopPH && units < MAX_TITRATION_UNITS
           && errorflag == 0) {
      mqttManager.loop();

      int stepVol;
      if (pH > 4.5f) {
        // Medium zone (pH > 4.5): large steps, rough tracking only
        // No stabilization needed — we just need to detect when to enter Gran zone
        stepVol = TITRATION_STEP_SIZE * MEDIUM_STEP_MULTIPLIER;  // 48 units
        if (!titrate(stepVol, TITRATION_RPM)) {
          errorMessage = "Error: titration pump timeout in precise phase";
          errorflag = 1;
          break;
        }
        delay(TITRATION_MIX_DELAY_MEDIUM_MS);
        measurePHStabilized(8);
      } else {
        // Gran zone (pH < 4.5): smaller steps, one stabilization, accurate readings
        stepVol = TITRATION_STEP_SIZE * 4;  // 8 units
        if (!titrate(stepVol, TITRATION_RPM)) {
          errorMessage = "Error: titration pump timeout in Gran zone";
          errorflag = 1;
          break;
        }
        delay(TITRATION_MIX_DELAY_GRAN_MS);
        waitForPHStabilization();
        measurePHStabilized(14);
      }
      units += stepVol;

      // Store data points near the endpoint for Gran analysis and interpolation
      if (pH < DATA_STORE_PH && nPoints < MAX_TITRATION_POINTS) {
        dataPoints[nPoints++] = {(float)units, pH};
      }
      if (pH < GRAN_REGION_PH) granCount++;

      mqttManager.loop();
      ArduinoOTA.handle();
      { char phBuf[16]; snprintf(phBuf, sizeof(phBuf), "%.2f", pH);
        mqttManager.publish(MQmespH, phBuf); }
      broadcastTitrationPH(pH, units);

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
          // Gran mode: strict R² cutoff, no fallback to interpolation
          if (granCount >= MIN_GRAN_POINTS) {
            char granReason[64] = "";
            exactUnits = granAnalysis(dataPoints, nPoints, samVol, titVol, calUnits, &granR2, granReason, sizeof(granReason));
            if (!isnan(exactUnits)) {
              usedGran = true;
            } else {
              char failBuf[128];
              snprintf(failBuf, sizeof(failBuf), "Gran failed: %s", granReason);
              errorMessage = "Error: Gran analysis failed";
              publishMessage(failBuf);
              errorflag = 1;
            }
          } else {
            errorMessage = "Error: insufficient Gran data points";
            errorflag = 1;
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
          subtractHCl(units + FILL_VOLUME);

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
          { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", hclUsed);
            mqttManager.publish(MQKH, mqBuf); }
          { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", startPH);
            mqttManager.publish(MQstartpH, mqBuf); }

          char methodBuf[48];
          if (usedGran) {
            snprintf(methodBuf, sizeof(methodBuf), "Gran (R²=%.3f)", granR2);
          } else {
            snprintf(methodBuf, sizeof(methodBuf), "Interpolation");
          }
          mqttManager.publish(MQmsg, methodBuf);

          float remainingHCl = configStore.getHClVolume();
          if (remainingHCl <= 0) {
            publishError("Warning: HCl supply empty! Refill needed.");
          } else if (remainingHCl < HCL_LOW_THRESHOLD_ML) {
            char warnBuf[64];
            snprintf(warnBuf, sizeof(warnBuf), "Warning: HCl low (%.0f mL remaining)", remainingHCl);
            publishError(warnBuf);
          }

          // Populate result struct — publishing deferred to measureKHWithValidation()
          result.khValue = khValue;
          result.startPH = startPH;
          result.hclUsed = hclUsed;
          result.granR2 = granR2;
          result.endpointPH = endpointPHVal;
          result.usedGran = usedGran;
          result.dataPointCount = nPoints;
          result.stabTimeouts = getStabilizationTimeoutCount();
          result.elapsedSec = (millis() - measStartMs) / 1000;
          result.confidence = computeConfidence(granR2, usedGran, nPoints,
                                                 result.stabTimeouts, getProbeHealth());
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
      hclPart = ((float)(units + FILL_VOLUME) / calU) * titV / samV;
    }
  }

  // Double rinse to eliminate acid carryover
  if (!washSample(1.5f + hclPart, 1.0)) {
    publishError("Warning: sample pump timeout during post-wash (1st rinse)");
  }
  delay(2000);
  if (!washSample(1.2, 1.0)) {
    publishError("Warning: sample pump timeout during post-wash (2nd rinse)");
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
