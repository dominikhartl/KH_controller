#include <Arduino.h>
#include <esp_system.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <config.h>
#include <pins.h>
#include <time.h>
#include <atomic>

#include "motors.h"
#include "stirrer.h"
#include "measurement.h"
#include "config_store.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "scheduler.h"
#include "ha_discovery.h"
#include "web_server.h"
#include "temperature.h"
#include "tmc_driver.h"
#include "hw_diagnostics.h"

// Device name (loaded from NVS at boot, used by mDNS, MQTT, HA, OTA, web UI)
char deviceName[21] = "KHpro";

// --- Global state ---
// These are accessed from both loopTask and AsyncTCP task — marked volatile
volatile int units = 0;
volatile float startPH = 0;
volatile bool discoveryPublished = false;
volatile unsigned long lastDiagnosticsTime = 0;
volatile unsigned long lastBroadcastTime = 0;
volatile uint32_t heapMin = UINT32_MAX;

// Deferred command queue: WebSocket/MQTT sets the command, loop() executes it
// This prevents long operations from blocking the AsyncTCP task
static std::atomic<char> pendingCmd{0};

// Abort flag: set by WebSocket handler, checked in measurement loops
static volatile bool abortRequested = false;
void requestAbort() { abortRequested = true; }
bool isAbortRequested() { return abortRequested; }

// Yield during measurement: keeps UI responsive
static void measurementYield() {
  broadcastState();
  ws.cleanupClients();
  wifiManager.loop();
  mqttManager.loop();
  ArduinoOTA.handle();
}

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
  pendingCmd.store(cmd, std::memory_order_release);
}

void publishMessage(const char* message);
void publishError(const char* errorMessage);
void calibrateTitrationPump();
void subtractHCl(int unitsUsed);

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

  // Motor health (SG stats from this measurement's pump operations)
  if (isTMCDetected()) {
    uint16_t sAvg, sMin, tAvg, tMin;
    getLastSampleSGStats(&sAvg, &sMin);
    getLastTitrateSGStats(&tAvg, &tMin);
    if (sAvg > 0 || tAvg > 0) {
      appendMotorHealth(ts, sAvg, sMin, tAvg, tMin);
    }
  }

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

  // Compute and publish KH trend slope (dKH/day) from configured window
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

void runMotorDiagnostic(char mode = 'd');  // forward declaration
void calibrateSamplePump();               // forward declaration

void processPendingCommand() {
  char cmd = pendingCmd.load(std::memory_order_acquire);
  if (cmd == 0) return;
  pendingCmd.store(0, std::memory_order_release);

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
    case 'p': {
      float pTemp = getWaterTemperatureC();
      configStore.setMeasTempC(pTemp);
      Serial.printf("Water temperature: %.1f °C%s\n", pTemp,
                    hasTemperatureSensor() ? "" : " (default, no sensor)");
      startStirrer();
      delay(STIRRER_WARMUP_MS);
      measurePH(isExternalADCActive() ? 20 : 100);
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
    case 'f': {
      publishMessage("Filling 1 mL");
      float fCalU = (float)configStore.getCalUnits();
      float fTitV = configStore.getTitrationVolume();
      int fUnits = max(2, (int)round(1000.0f * fCalU / (fTitV * 1000.0f)));
      if (!titrate(fUnits, configStore.getTitrationRPM(), false)) {
        publishError(wasMotorStall() ? "Error: titration pump stall during fill" : "Error: titration pump timeout during fill");
      } else {
        publishMessage("Fill done");
      }
      subtractHCl(fUnits);
      digitalWrite(EN_PIN2, HIGH);
      broadcastState();
      break;
    }
    case 's': {
      stopStirrer();
      publishMessage("Washing sample");
      int wFill = (int)round(configStore.getSampleVolume() * configStore.getSampleCalRevsPerML());
      int wRemove = (int)(wFill * 1.2f);
      if (!washSampleVol(wRemove, wFill, configStore.getSamplePumpRPM())) {
        publishError(wasMotorStall() ? "Error: sample pump stall during wash" : "Error: sample pump timeout during wash");
      } else {
        publishMessage("Wash done");
      }
      broadcastState();
      break;
    }
    case 'r': {
      stopStirrer();
      publishMessage("Removing sample");
      int rVol = (int)round(configStore.getSampleVolume() * configStore.getSampleCalRevsPerML() * 1.2f);
      if (!removeSample(rVol, configStore.getSamplePumpRPM())) {
        publishError(wasMotorStall() ? "Error: sample pump stall during remove" : "Error: sample pump timeout during remove");
      } else {
        publishMessage("Sample removed");
      }
      broadcastState();
      break;
    }
    case 'v': {
      float v = measureVoltage(100);
      char vBuf[32];
      snprintf(vBuf, sizeof(vBuf), "Voltage: %.1f mV", v);
      publishMessage(vBuf);
      broadcastState();
      break;
    }
    case 'm':
      startStirrer();
      publishMessage("Stirrer started");
      broadcastState();
      break;
    case 'e':
      stopStirrer();
      publishMessage("Stirrer stopped");
      broadcastState();
      break;
    case 'd':
      runMotorDiagnostic('d');
      broadcastState();
      break;
    case 'B':
      runMotorDiagnostic('B');
      broadcastState();
      break;
    case 'C':
      runMotorDiagnostic('C');
      broadcastState();
      break;
    case 'S':
      calibrateSamplePump();
      broadcastState();
      break;
    case 'W': {
      publishMessage("Cleaning tube...");
      stopStirrer();
      float cwRevsPerML = configStore.getSampleCalRevsPerML();
      int cwRevs = (int)round(10.0f * cwRevsPerML);
      float cwMaxRpm = configStore.getSampleMaxRPM();
      float cwRpm = (cwMaxRpm > 0) ? cwMaxRpm : configStore.getSamplePumpRPM();
      bool cwOk = true;
      for (int i = 0; i < 5 && cwOk; i++) {
        char cwBuf[32];
        snprintf(cwBuf, sizeof(cwBuf), "Clean cycle %d/5", i + 1);
        publishMessage(cwBuf);
        cwOk = removeSample(cwRevs, cwRpm);
        if (cwOk) cwOk = takeSample(cwRevs, cwRpm);
      }
      if (cwOk) {
        publishMessage("Tube cleaning done");
      } else {
        publishError("Tube cleaning failed (stall/timeout)");
      }
      break;
    }
    case 'H':
      if (isHWDiagRunning()) {
        publishMessage("Hardware diagnostics already running");
      } else {
        runHardwareDiagnostics();
        broadcastState();
      }
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

// --- Motor diagnostics ---

void broadcastMotorDiag(const char* json);  // forward decl (web_server.cpp)

struct MotorDiagResult {
  uint16_t sgMin, sgMax;
  float sgAvg;
  bool diagAlwaysLow;
};

static MotorDiagResult analyzeSamples(SGSample* samples, int n) {
  MotorDiagResult r = {65535, 0, 0, true};
  if (n == 0) return r;
  uint32_t sum = 0;
  for (int i = 0; i < n; i++) {
    if (samples[i].sg < r.sgMin) r.sgMin = samples[i].sg;
    if (samples[i].sg > r.sgMax) r.sgMax = samples[i].sg;
    sum += samples[i].sg;
    if (samples[i].diag) r.diagAlwaysLow = false;
  }
  r.sgAvg = (float)sum / n;
  return r;
}

// Stall ramp RPM progress callback
static void stallRpmProgress(float rpm) {
  char buf[32];
  snprintf(buf, sizeof(buf), "Stall test: %.0f RPM...", rpm);
  publishMessage(buf);
}

// mode: 'B' = sample only, 'C' = titrate only, 'd' = both
void runMotorDiagnostic(char mode) {
  if (!isTMCDetected()) {
    publishError("Motor diagnostics requires TMC2209");
    return;
  }

  const int DIAG_REVS = 5;
  const int MAX_S = 20;
  SGSample samples[MAX_S];

  // Sample pump results (zeroed defaults for when skipped)
  MotorDiagResult scSample = {0, 0, 0, true};
  MotorDiagResult spSample = {0, 0, 0, true};
  bool sampleRecSC = false;
  uint16_t sBestMin = 0;
  float sampleStallRPM = 0.0f;
  float sampleRecommendedRPM = 0.0f;

  // Titration pump results
  MotorDiagResult scTitrate = {0, 0, 0, true};
  MotorDiagResult spTitrate = {0, 0, 0, true};
  bool titrateRecSC = false;
  uint16_t tBestMin = 0;
  float titrateStallRPM = 0.0f;
  float titrateRecommendedRPM = 0.0f;

  bool doSample = (mode == 'd' || mode == 'B');
  bool doTitrate = (mode == 'd' || mode == 'C');

  publishMessage("Running motor diagnostics...");

  if (doSample) {
    // --- Sample pump ---
    publishMessage("Testing sample pump...");
    float sampleRPM = configStore.getSamplePumpRPM();

    // StealthChop
    setSampleSpreadCycle(false);
    delay(50);
    int n = diagStepSample(DIAG_REVS, sampleRPM, samples, MAX_S);
    scSample = analyzeSamples(samples, n);

    // SpreadCycle
    setSampleSpreadCycle(true);
    delay(50);
    n = diagStepSample(DIAG_REVS, sampleRPM, samples, MAX_S);
    spSample = analyzeSamples(samples, n);

    // Restore current setting
    setSampleSpreadCycle(configStore.getSampleSpreadCycle());

    sampleRecSC = scSample.sgMin > spSample.sgMin;
    sBestMin = sampleRecSC ? scSample.sgMin : spSample.sgMin;

    // --- Stall speed ramp (sample pump, both directions) ---
    // SG diagnostic ran in DIR HIGH — pause to let DIAG settle before removal (DIR LOW)
    setSampleSpreadCycle(!sampleRecSC);
    clearStallFlag();
    delay(3000);

    publishMessage("Testing stall speed (removal)...");

    const int RAMP_MAX_SAMPLES = 288;  // (500-30)/5 * 3 = 282 max
    SGSample rampSamples[RAMP_MAX_SAMPLES];
    int rampSampleCount = 0;

    float stallRemoval = diagStallRamp(30.0f, 500.0f, 5.0f, 3,
                                    rampSamples, RAMP_MAX_SAMPLES, &rampSampleCount,
                                    false, stallRpmProgress);

    if (stallRemoval > 0) {
      char buf[48];
      snprintf(buf, sizeof(buf), "Removal stall at %.0f RPM", stallRemoval);
      publishMessage(buf);
    } else {
      publishMessage("No removal stall detected");
    }

    // Reset stall signal and pause before reversing direction
    clearStallFlag();
    delay(3000);

    publishMessage("Testing stall speed (fill)...");
    rampSampleCount = 0;

    float stallFill = diagStallRamp(30.0f, 500.0f, 5.0f, 3,
                                    rampSamples, RAMP_MAX_SAMPLES, &rampSampleCount,
                                    true, stallRpmProgress);

    if (stallFill > 0) {
      char buf[48];
      snprintf(buf, sizeof(buf), "Fill stall at %.0f RPM", stallFill);
      publishMessage(buf);
    } else {
      publishMessage("No fill stall detected");
    }

    // Restore configured chopper mode
    setSampleSpreadCycle(configStore.getSampleSpreadCycle());

    // Take the lower stall RPM from both directions
    if (stallRemoval > 0 && stallFill > 0) {
      sampleStallRPM = (stallRemoval < stallFill) ? stallRemoval : stallFill;
    } else if (stallRemoval > 0) {
      sampleStallRPM = stallRemoval;
    } else {
      sampleStallRPM = stallFill;  // 0 if neither stalled
    }
    sampleRecommendedRPM = sampleStallRPM > 0 ? sampleStallRPM * 0.8f : 0.0f;

    // Persist max RPM for speed indication
    if (sampleStallRPM > 0) {
      float maxRpm = floor(sampleRecommendedRPM / 5.0f) * 5.0f;
      configStore.setSampleMaxRPM(maxRpm);
    }

    if (sampleStallRPM > 0) {
      char buf[80];
      snprintf(buf, sizeof(buf), "Sample stall at %.0f RPM (removal:%.0f fill:%.0f, max: %.0f RPM)",
               sampleStallRPM, stallRemoval, stallFill, sampleRecommendedRPM);
      publishMessage(buf);
    } else {
      publishMessage("Sample: no stall detected within test range (30-500 RPM)");
    }
  }

  if (doTitrate) {
    // --- Titration pump ---
    publishMessage("Testing titration pump...");
    float titrateRPM = TITRATION_RPM;

    // StealthChop
    setTitrateSpreadCycle(false);
    delay(50);
    int n = diagStepTitrate(DIAG_REVS, titrateRPM, samples, MAX_S);
    scTitrate = analyzeSamples(samples, n);

    // SpreadCycle
    setTitrateSpreadCycle(true);
    delay(50);
    n = diagStepTitrate(DIAG_REVS, titrateRPM, samples, MAX_S);
    spTitrate = analyzeSamples(samples, n);

    // Restore current setting
    setTitrateSpreadCycle(configStore.getTitrateSpreadCycle());

    titrateRecSC = scTitrate.sgMin > spTitrate.sgMin;
    tBestMin = titrateRecSC ? scTitrate.sgMin : spTitrate.sgMin;

    // --- Stall speed ramp (titration pump, forward only, max 150 RPM) ---
    // 1 rev per step, 10 RPM increments to minimize HCl usage
    publishMessage("Testing titration stall speed...");
    setTitrateSpreadCycle(!titrateRecSC);
    clearStallFlag();
    delay(3000);

    const int TRIT_RAMP_MAX = 13;  // (150-30)/10 * 1 = 12 max + 1
    SGSample titRampSamples[TRIT_RAMP_MAX];
    int titRampCount = 0;

    float titStall = diagStallRampTitrate(30.0f, 150.0f, 10.0f, 1,
                                    titRampSamples, TRIT_RAMP_MAX, &titRampCount,
                                    true, stallRpmProgress);

    setTitrateSpreadCycle(configStore.getTitrateSpreadCycle());

    if (titStall > 0) {
      titrateStallRPM = titStall;
    }
    titrateRecommendedRPM = titrateStallRPM > 0 ? titrateStallRPM * 0.8f : 0.0f;

    if (titrateStallRPM > 0) {
      float maxRpm = floor(titrateRecommendedRPM / 5.0f) * 5.0f;
      configStore.setTitrateMaxRPM(maxRpm);
      char buf[64];
      snprintf(buf, sizeof(buf), "Titration stall at %.0f RPM (max: %.0f RPM)",
               titrateStallRPM, titrateRecommendedRPM);
      publishMessage(buf);
    } else {
      publishMessage("Titration: no stall detected within test range (30-150 RPM)");
    }
  }

  // Build JSON with snprintf to avoid JsonDocument DRAM overhead
  char* json = (char*)malloc(1024);
  if (!json) { publishError("Motor diag: out of memory"); return; }
  snprintf(json, 1024,
    "{\"type\":\"motorDiag\",\"mode\":\"%c\","
    "\"sample\":{\"stealthchop\":{\"sgMin\":%d,\"sgMax\":%d,\"sgAvg\":%d},"
    "\"spreadcycle\":{\"sgMin\":%d,\"sgMax\":%d,\"sgAvg\":%d},"
    "\"recommended\":\"%s\",\"suggestedThreshold\":%d,"
    "\"stallRPM\":%.0f,\"maxRPM\":%.0f},"
    "\"titrate\":{\"stealthchop\":{\"sgMin\":%d,\"sgMax\":%d,\"sgAvg\":%d},"
    "\"spreadcycle\":{\"sgMin\":%d,\"sgMax\":%d,\"sgAvg\":%d},"
    "\"recommended\":\"%s\",\"suggestedThreshold\":%d,"
    "\"stallRPM\":%.0f,\"maxRPM\":%.0f}}",
    mode,
    scSample.sgMin, scSample.sgMax, (int)scSample.sgAvg,
    spSample.sgMin, spSample.sgMax, (int)spSample.sgAvg,
    sampleRecSC ? "stealthchop" : "spreadcycle", sBestMin / 2,
    sampleStallRPM, sampleRecommendedRPM,
    scTitrate.sgMin, scTitrate.sgMax, (int)scTitrate.sgAvg,
    spTitrate.sgMin, spTitrate.sgMax, (int)spTitrate.sgAvg,
    titrateRecSC ? "stealthchop" : "spreadcycle", tBestMin / 2,
    titrateStallRPM, titrateRecommendedRPM);
  broadcastMotorDiag(json);
  free(json);
  publishMessage("Motor diagnostics complete");
}

// --- Pump calibration ---

void calibrateSamplePump() {
  publishMessage("Calibrating sample pump: removing old water...");
  float sampRpm = configStore.getSamplePumpRPM();
  // Remove existing water first (same volume as calibration run)
  if (!removeSample(SAMPLE_CAL_REVOLUTIONS, sampRpm)) {
    publishError(wasMotorStall() ? "Error: sample pump stall during remove" : "Error: sample pump timeout during remove");
    return;
  }
  delay(500);
  char buf[80];
  snprintf(buf, sizeof(buf), "Taking %d revolutions at %.0f RPM",
           SAMPLE_CAL_REVOLUTIONS, sampRpm);
  publishMessage(buf);
  if (!takeSample(SAMPLE_CAL_REVOLUTIONS, sampRpm)) {
    publishError(wasMotorStall() ? "Error: sample pump stall during calibration" : "Error: sample pump timeout during calibration");
    return;
  }
  configStore.setSampleCalTimestamp((uint32_t)time(nullptr));
  publishMessage("Sample pump calibration done. Measure dispensed volume and enter in 'Sample Cal Volume (mL)'.");
}

void calibrateTitrationPump() {
  // Compute dynamic target: HCl volume needed for 7.5 dKH at configured sample volume
  int targetUnits = CALIBRATION_TARGET_UNITS;  // fallback 6000
  float calU = (float)configStore.getCalUnits();
  float titV = configStore.getTitrationVolume();
  float samVol = configStore.getSampleVolume();
  float hclMol = configStore.getHClMolarity();
  float corrF = configStore.getCorrectionFactor();

  if (calU > 0 && titV > 0 && hclMol > 0) {
    float hclNeeded = 7.5f * samVol / (2800.0f * hclMol * corrF);
    int computed = (int)round(hclNeeded * calU / titV);
    if (computed >= 1000 && computed <= 20000) {
      targetUnits = computed;
    }
  }

  float expectedML = (float)targetUnits / calU * titV;
  char buf[80];
  snprintf(buf, sizeof(buf), "Calibrating pump (target: %d units, ~%.1f mL for 7.5 dKH)", targetUnits, expectedML);
  publishMessage(buf);

  units = 0;
  const int BATCH = 40;

  while (units < targetUnits) {
    int batch = min(BATCH, targetUnits - units);
    if (!titrate(batch, TITRATION_RPM)) {
      publishError(wasMotorStall() ? "Error: titration pump stall during calibration" : "Error: titration pump timeout during calibration");
      digitalWrite(EN_PIN2, HIGH);
      return;
    }
    // Read SG immediately after batch (motor was just stepping, SG still reflects load)
    if (isTMCDetected()) {
      uint16_t sg = getTitrateSG();
      if (sg < (uint16_t)configStore.getTitrateStallSG()) {
        Serial.printf("ERROR: Titration pump stall (SG=%d)\n", sg);
        setStallFlag();
        publishError("Error: titration pump stall during calibration");
        digitalWrite(EN_PIN2, HIGH);
        return;
      }
    }
    units += batch;
    delay(TITRATION_MIX_DELAY_FAST_MS);
    ArduinoOTA.handle();
    snprintf(buf, sizeof(buf), "Cal: %d / %d", units, targetUnits);
    publishMessage(buf);
  }

  subtractHCl(targetUnits);
  units = 0;
  digitalWrite(EN_PIN2, HIGH);
  configStore.setTitrationCalTimestamp((uint32_t)time(nullptr));
  publishMessage("Pump calibration done");
}

// --- Measurement confidence score ---
// Combines multiple quality signals into a 0.0-1.0 score
static float computeConfidence(float granR2, bool usedGran, int nPoints,
                                int stabTimeouts, const char* probeHealth,
                                float crossValDiff, float probeNoiseMv,
                                int phReversals, int granStepCount) {
  float score = 1.0f;
  // Gran R² — continuous: R²=0.99 → -0.20, R²=0.999 → -0.02
  if (usedGran && granR2 > 0) {
    score -= (1.0f - granR2) * 20.0f;
  }
  // Cross-validation penalty — only penalize large Gran/endpoint disagreements
  if (!isnan(crossValDiff) && crossValDiff > 0.3f) {
    score -= min(0.15f, (crossValDiff - 0.3f) * 0.3f);
  }
  // Probe noise — continuous penalty starting at 2 mV
  score -= min(0.25f, max(0.0f, (probeNoiseMv - 4.0f) * 0.05f));
  // Data point count
  if (nPoints < 15) score -= 0.1f;
  if (nPoints < 10) score -= 0.1f;
  // Stabilization timeout penalty
  // ADS1115 detects sub-mV drift invisible to internal ADC; timeouts are expected and
  // don't indicate poor measurement quality (R², reversals, cross-val are unaffected)
  if (!isExternalADCActive()) {
    float timeoutRate = (nPoints > 0) ? (float)stabTimeouts / nPoints : 0;
    if (timeoutRate > 0.1f) score -= 0.1f;
    if (timeoutRate > 0.3f) score -= 0.1f;
  }
  // pH reversal penalty — reversals indicate probe drift or incomplete mixing
  float reversalRate = (granStepCount > 0) ? (float)phReversals / granStepCount : 0;
  if (reversalRate > 0.05f) score -= 0.1f;
  if (reversalRate > 0.10f) score -= 0.1f;
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
  abortRequested = false;  // Clear any stale abort

  // Read water temperature from sensor (or use default if no sensor)
  float waterTemp = getWaterTemperatureC();
  configStore.setMeasTempC(waterTemp);
  Serial.printf("Water temperature: %.1f °C%s\n", waterTemp,
                hasTemperatureSensor() ? "" : " (default, no sensor)");

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
    publishError(wasMotorStall() ? "Error: titration pump stall during prefill" : "Error: titration pump timeout during prefill");
    digitalWrite(EN_PIN2, HIGH);
    measuring = false;
    return result;
  }
  // Keep titration motor enabled after prefill to prevent suckback
  publishMessage("Taking sample");
  // Compute sample pump revolutions from volume config and calibration
  float sampVolML = configStore.getSampleVolume();
  float revsPerML = configStore.getSampleCalRevsPerML();
  int sampleFillRevs = (int)round(sampVolML * revsPerML);
  int sampleRemoveRevs = (int)(sampleFillRevs * 1.2f);
  // Double wash: first rinse cleans the chamber, second takes the actual sample
  setMultiWashContext(2);
  float sampRpm = configStore.getSamplePumpRPM();
  if (!washSampleVol(sampleRemoveRevs, sampleFillRevs, sampRpm)) {
    clearMultiWashContext();
    publishError("Error: sample pump timeout during wash (1st rinse)");
    measuring = false;
    return result;
  }
  measurementYield();
  if (abortRequested) { abortRequested = false; stopStirrer(); digitalWrite(EN_PIN2, HIGH); clearMultiWashContext(); publishError("Measurement aborted"); measuring = false; return result; }
  delay(1000);
  if (!washSampleVol(sampleRemoveRevs, sampleFillRevs, sampRpm)) {
    clearMultiWashContext();
    publishError("Error: sample pump timeout during wash (2nd rinse)");
    measuring = false;
    return result;
  }
  clearMultiWashContext();
  measurementYield();
  if (abortRequested) { abortRequested = false; stopStirrer(); digitalWrite(EN_PIN2, HIGH); publishError("Measurement aborted"); measuring = false; return result; }
  delay(100);
  startStirrer();
  delay(STIRRER_WARMUP_MS);  // Wait for solution to homogenize
  measurementYield();
  measurePH(isExternalADCActive() ? 20 : 100);
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
    washSampleVol((int)(sampleFillRevs * 1.5f), sampleFillRevs, sampRpm);
    delay(2000);
    washSampleVol(sampleRemoveRevs, sampleFillRevs, sampRpm);
    clearMultiWashContext();
    delay(100);
    startStirrer();
    delay(STIRRER_WARMUP_MS);
    measurePH(isExternalADCActive() ? 20 : 100);
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
        errorMessage = wasMotorStall() ? "Error: titration pump stall in fast phase" : "Error: titration pump timeout in fast phase";
        errorflag = 1;
        break;
      }
      units += batch;
      int mixDelay = TITRATION_MIX_DELAY_FAST_MS +
          (FAST_BATCH_MAX - batch) * 600 / FAST_BATCH_MAX;
      delay(mixDelay);
      int nReadings = isExternalADCActive()
          ? 3 + (FAST_BATCH_MAX - batch) * 5 / FAST_BATCH_MAX
          : 5 + (FAST_BATCH_MAX - batch) * 15 / FAST_BATCH_MAX;
      measurePHFast(nReadings);
      broadcastTitrationPH(pH, units);
      measurementYield();
      { int8_t rssi = wifiManager.getRSSI();
        if (rssi < rssiMin) rssiMin = rssi;
        if (rssi > rssiMax) rssiMax = rssi; }
      if (abortRequested) { errorMessage = "Measurement aborted"; errorflag = 1; break; }

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

    // Reset noise stats so only Gran zone stabilization noise is counted
    resetNoiseStats();

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
          errorMessage = wasMotorStall() ? "Error: titration pump stall in precise phase" : "Error: titration pump timeout in precise phase";
          errorflag = 1;
          break;
        }
        delay(TITRATION_MIX_DELAY_MEDIUM_MS);
        measurePHStabilized(isExternalADCActive() ? 3 : 8);
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
          errorMessage = wasMotorStall() ? "Error: titration pump stall in Gran zone" : "Error: titration pump timeout in Gran zone";
          errorflag = 1;
          break;
        }
        delay(configStore.getGranMixDelay());
        waitForPHStabilization();
        measurePHStabilized(isExternalADCActive() ? 5 : 20);
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

      { char phBuf[16]; snprintf(phBuf, sizeof(phBuf), "%.2f", pH);
        mqttManager.publish(MQmespH, phBuf); }
      broadcastTitrationPH(pH, units);
      measurementYield();
      { int8_t rssi = wifiManager.getRSSI();
        if (rssi < rssiMin) rssiMin = rssi;
        if (rssi > rssiMax) rssiMax = rssi; }
      if (abortRequested) { errorMessage = "Measurement aborted"; errorflag = 1; break; }

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
        float granSlope = 0, granIntercept = 0;
        GranWindowResult granWindows[MAX_GRAN_WINDOWS];
        int nGranWindows = 0;
        float exactUnits;
        bool usedGran = false;

        if (epMethod == 0) {
          // Gran mode: try Gran analysis, fall back to endpoint if it fails
          if (granCount >= MIN_GRAN_POINTS) {
            char granReason[64] = "";
            exactUnits = granAnalysis(dataPoints, nPoints, samVol, titVol, calUnits, &granR2, &result.granWinLow, &result.granWinHigh, granReason, sizeof(granReason), &granSlope, &granIntercept, granWindows, &nGranWindows);
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

          // Collect Gran region points by proximity to window:
          // Pass 1: inside the window (must all be shown)
          // Pass 2: within 0.3 pH of window bounds (transition zone)
          // Pass 3: remaining Gran region (far context)
          auto addGranPt = [&](int i) {
            granPtML[nGranPts] = dataPoints[i].units * k;
            granPtF[nGranPts] = (samVol + dataPoints[i].units * k) * powf(10.0f, -dataPoints[i].pH);
            nGranPts++;
          };
          float wLo = result.granWinLow, wHi = result.granWinHigh;
          // Track which points are already added
          bool added[MAX_TITRATION_POINTS] = {};
          for (int i = 0; i < nPoints && nGranPts < MAX_GRAN_DIAG; i++) {
            if (dataPoints[i].pH < wHi && dataPoints[i].pH > wLo) {
              addGranPt(i); added[i] = true;
            }
          }
          for (int i = 0; i < nPoints && nGranPts < MAX_GRAN_DIAG; i++) {
            if (!added[i] && dataPoints[i].pH < GRAN_REGION_PH && dataPoints[i].pH > GRAN_STOP_PH &&
                (dataPoints[i].pH >= wHi - 0.3f || dataPoints[i].pH <= wLo + 0.3f)) {
              addGranPt(i); added[i] = true;
            }
          }
          for (int i = 0; i < nPoints && nGranPts < MAX_GRAN_DIAG; i++) {
            if (!added[i] && dataPoints[i].pH < GRAN_REGION_PH && dataPoints[i].pH > GRAN_STOP_PH) {
              addGranPt(i);
            }
          }
          float eqML = isnan(exactUnits) ? 0 : exactUnits * k;
          // Compute Gran window mL bounds from pH bounds (iterate all points, not just buffer)
          float winLowML = 0, winHighML = 0;
          if (result.granWinHigh > 0) {
            bool first = true;
            for (int i = 0; i < nPoints; i++) {
              if (dataPoints[i].pH < result.granWinHigh && dataPoints[i].pH > result.granWinLow) {
                float ml = dataPoints[i].units * k;
                if (first) { winLowML = winHighML = ml; first = false; }
                else { if (ml < winLowML) winLowML = ml; if (ml > winHighML) winHighML = ml; }
              }
            }
          }
          // Compute per-window KH values from each window's equivalence point
          for (int i = 0; i < nGranWindows; i++) {
            if (granWindows[i].valid && !isnan(granWindows[i].eqUnits)) {
              float eqMLw = granWindows[i].eqUnits * k;
              granWindows[i].eqUnits = (eqMLw / samVol) * 2800.0f * hclMol * corrF; // reuse field as KH
            } else {
              granWindows[i].eqUnits = 0;
            }
          }
          // Convert regression from units-space to mL-space: F = (slope/k)*mL + intercept
          float slopeML = usedGran ? granSlope / k : 0;
          float interceptF = usedGran ? granIntercept : 0;
          broadcastGranData(granR2, eqML, usedGran, granPtML, granPtF, nGranPts, winLowML, winHighML, slopeML, interceptF, granWindows, nGranWindows);
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
              float granUnits2 = granAnalysis(dataPoints, nPoints, samVol, titVol, calUnits, &tryR2, nullptr, nullptr, granReason2, sizeof(granReason2));
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
          result.maxNoiseMv = getMaxStabNoiseMv();
          result.highNoiseCount = getHighNoiseCount();
          result.stepNoisePh = (granStepCount > 1) ? stepDeltaSum / (granStepCount - 1) : 0;
          result.phReversals = phReversals;
          result.granStepCount = granStepCount;
          result.confidence = computeConfidence(granR2, usedGran, nPoints,
                                                 result.stabTimeouts, getProbeHealth(),
                                                 crossValDiff, result.probeNoiseMv,
                                                 phReversals, granStepCount);

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
  int postFillRevs = (int)round(configStore.getSampleVolume() * configStore.getSampleCalRevsPerML());
  int postRemoveRevs = (int)(postFillRevs * (1.5f + hclPart));
  if (!washSampleVol(postRemoveRevs, postFillRevs, configStore.getSamplePumpRPM())) {
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
  abortRequested = false;
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
  initStirrer();
  Serial.begin(115200);

  // Detect TMC2209 stepper drivers (must be before initADC — IO35 is shared)
  initTMCDrivers();

  // Configure PH_PIN (GPIO34) as analog input for internal ADC fallback
  // (when ADS1115 is present, initExternalADC() reconfigures it as RDY pin)
  if (!isTMCDetected()) {
    pinMode(PH_PIN, INPUT);
  }

  // Initialize calibrated ADC
  initADC();

  // Initialize config store (NVS) and migrate from EEPROM if needed
  configStore.begin();

  // Load device name from NVS (used by mDNS, MQTT, HA, OTA, web UI)
  configStore.getDeviceName(deviceName, sizeof(deviceName));

  // Initialize ADS1115 external ADC if configured (must be after configStore.begin())
  initExternalADC();

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
  snprintf(topicCmd, sizeof(topicCmd), "%s/cmd", deviceName);
  snprintf(MQmsg, sizeof(MQmsg), "%s/message", deviceName);
  snprintf(MQerr, sizeof(MQerr), "%s/error", deviceName);
  snprintf(MQKH, sizeof(MQKH), "%s/KH", deviceName);
  snprintf(MQstartpH, sizeof(MQstartpH), "%s/startPH", deviceName);
  snprintf(MQmespH, sizeof(MQmespH), "%s/mes_pH", deviceName);
  snprintf(MQkhValue, sizeof(MQkhValue), "%s/kh_value", deviceName);
  snprintf(MQconfidence, sizeof(MQconfidence), "%s/confidence", deviceName);
  snprintf(MQkhSlope, sizeof(MQkhSlope), "%s/kh_slope", deviceName);
  snprintf(MQgranR2, sizeof(MQgranR2), "%s/gran_r2", deviceName);
  snprintf(MQcrossVal, sizeof(MQcrossVal), "%s/cross_val_diff", deviceName);
  snprintf(MQdataPoints, sizeof(MQdataPoints), "%s/data_points", deviceName);
  snprintf(MQmeasTime, sizeof(MQmeasTime), "%s/meas_time", deviceName);

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
  ArduinoOTA.setHostname(deviceName);
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

  // Load pH calibration from NVS (use external ADC calibration if ADS1115 is active)
  if (isExternalADCActive()) {
    voltage_4PH = configStore.getVoltage4PHExt();
    voltage_7PH = configStore.getVoltage7PHExt();
    voltage_10PH = configStore.getVoltage10PHExt();
  } else {
    voltage_4PH = configStore.getVoltage4PH();
    voltage_7PH = configStore.getVoltage7PH();
    voltage_10PH = configStore.getVoltage10PH();
  }

  // Compute linear fit from calibration values
  updateCalibrationFit();

  // Web server + WebSocket dashboard
  setupWebServer();
  computeKHSlope();  // populate cached slope for broadcastState()
  {
    int reason = esp_reset_reason();
    Serial.printf("Reset reason: %d\n", reason);
    char bootBuf[64];
    snprintf(bootBuf, sizeof(bootBuf), "BOOT (reason=%d, heap=%u)", reason, ESP.getFreeHeap());
    publishMessage(bootBuf);
    if (isExternalADCActive()) {
      publishMessage("ADS1115 external ADC active");
    } else if (isExternalADCFallback()) {
      publishError("ADS1115 configured but not detected — using internal ADC");
    }
    if (isTMCDetected()) {
      publishMessage("TMC2209 stepper drivers active");
      if (!isExternalADCActive()) {
        publishError("TMC2209 uses IO35 for DIAG — pH requires ADS1115");
      }
    }
    initTemperature();
  }

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
  processPendingReplay();
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

  // Track heap watermark with low-heap warning
  uint32_t h = ESP.getFreeHeap();
  if (h < heapMin) {
    heapMin = h;
    if (h < HEAP_WARNING_THRESHOLD) {
      static unsigned long lastHeapWarn = 0;
      if (millis() - lastHeapWarn > 60000) {
        lastHeapWarn = millis();
        Serial.printf("WARNING: Free heap low: %lu bytes (min: %lu)\n",
          (unsigned long)h, (unsigned long)heapMin);
      }
    }
  }

  // Publish diagnostics every 60s
  if (mqttManager.isConnected() && millis() - lastDiagnosticsTime > 60000) {
    lastDiagnosticsTime = millis();
    publishDiagnostics();
  }

}
