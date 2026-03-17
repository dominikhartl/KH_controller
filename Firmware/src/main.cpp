#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
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
// These are accessed from both loopTask and AsyncTCP task.
// Atomic for cross-task safety on ESP32 dual-core.
std::atomic<int> units{0};
// startPH: float has no std::atomic on ESP32 Arduino. Torn reads are theoretically
// possible but harmless — value is only written once per measurement, read for display.
volatile float startPH = 0;
volatile bool discoveryPublished = false;
volatile unsigned long lastDiagnosticsTime = 0;
volatile unsigned long lastBroadcastTime = 0;
volatile uint32_t heapMin = UINT32_MAX;

// Deferred command queue: WebSocket/MQTT sets the command, loop() executes it
// This prevents long operations from blocking the AsyncTCP task
static std::atomic<char> pendingCmd{0};

// AP mode flag: when true, loop() only processes DNS requests
static bool apModeActive = false;

// BOOT button (GPIO 0) long-press tracking for WiFi reset
static unsigned long bootBtnPressStart = 0;
static const unsigned long BOOT_BTN_HOLD_MS = 5000;  // 5 seconds

// Measuring flag: true while measureKH() is running (any trigger: UI, schedule, precision test)
// Atomic: read by AsyncTCP task (broadcastState), written by loopTask
std::atomic<bool> isMeasuringKH{false};

// Abort flag: set by AsyncTCP task (WebSocket handler), checked by loopTask in measurement loops
static std::atomic<bool> abortRequested{false};
void requestAbort() { abortRequested.store(true, std::memory_order_relaxed); }
bool isAbortRequested() { return abortRequested.load(std::memory_order_relaxed); }

// Yield during measurement: keeps UI responsive
// Light yield runs every call (~500ms): WiFi/MQTT/OTA + client cleanup
// Full broadcastState runs at most every 2s to avoid flooding WebSocket/SPI flash
static void measurementYield() {
  esp_task_wdt_reset();  // Feed watchdog during long measurement operations
  ws.cleanupClients();
  wifiManager.loop();
  mqttManager.loop();
  ArduinoOTA.handle();
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast >= 2000) {
    lastBroadcast = millis();
    broadcastState();
  }
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
char MQkhCI[50];

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
  appendGranHistory(r.granR2, r.hclUsed, r.endpointPH, r.usedGran, r.confidence, r.khGran, r.khEndpoint, r.probeNoiseMv, r.phReversals, configStore.getDropVolumeUL(), configStore.getTitrationRPM(), r.khCI, ts, r.startPH, getAcidEfficiency());

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
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.5f", r.granR2);
    mqttManager.publish(MQgranR2, mqBuf, true); }
  if (!isnan(r.khCI)) {
    char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.3f", r.khCI);
    mqttManager.publish(MQkhCI, mqBuf, true);
  }

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

// Precision Test: run N consecutive full measurement cycles, report SD
static const int PRECISION_TEST_COUNT = 3;

static void measureKHPrecisionTest() {
  float results[PRECISION_TEST_COUNT];
  int validCount = 0;
  unsigned long testStart = millis();

  char buf[128];
  snprintf(buf, sizeof(buf), "Precision test: %d consecutive measurements", PRECISION_TEST_COUNT);
  publishMessage(buf);

  for (int i = 0; i < PRECISION_TEST_COUNT; i++) {
    if (isAbortRequested()) {
      publishMessage("Precision test aborted");
      return;
    }
    snprintf(buf, sizeof(buf), "Precision test %d/%d starting...", i + 1, PRECISION_TEST_COUNT);
    publishMessage(buf);
    broadcastProgress((i * 100) / PRECISION_TEST_COUNT);

    KHResult r = measureKH();
    if (isnan(r.khValue)) {
      snprintf(buf, sizeof(buf), "Measurement %d/%d failed, skipping", i + 1, PRECISION_TEST_COUNT);
      publishError(buf);
      continue;
    }

    // Publish and log like a normal measurement
    publishKHResult(r);
    results[validCount++] = r.khValue;

    snprintf(buf, sizeof(buf), "Precision test %d/%d: %.2f dKH", i + 1, PRECISION_TEST_COUNT, r.khValue);
    publishMessage(buf);
  }

  broadcastProgress(100);

  if (validCount < 2) {
    publishError("Precision test: not enough valid measurements for statistics");
    return;
  }

  // Compute mean
  float sum = 0;
  for (int i = 0; i < validCount; i++) sum += results[i];
  float mean = sum / validCount;

  // Compute SD
  float ssq = 0;
  for (int i = 0; i < validCount; i++) {
    float d = results[i] - mean;
    ssq += d * d;
  }
  float sd = sqrtf(ssq / (validCount - 1));

  // Find min/max
  float vmin = results[0], vmax = results[0];
  for (int i = 1; i < validCount; i++) {
    if (results[i] < vmin) vmin = results[i];
    if (results[i] > vmax) vmax = results[i];
  }

  unsigned long elapsed = (millis() - testStart) / 1000;
  snprintf(buf, sizeof(buf),
    "Precision: ±%.3f dKH SD (n=%d, mean=%.2f, range=%.2f–%.2f, %lum%lus)",
    sd, validCount, mean, vmin, vmax, elapsed / 60, elapsed % 60);
  publishMessage(buf);

  // Store persistently
  uint32_t ts = (uint32_t)time(nullptr);
  appendPrecisionHistory(ts, validCount, mean, sd, vmin, vmax, elapsed);
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
      updateNernstTempCorrection(pTemp);
      resetADCFilter();
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
      publishMessage("Filling 10 mL");
      float fCalU = (float)configStore.getCalUnits();
      float fTitV = configStore.getTitrationVolume();
      int fUnits = max(2, (int)round(10000.0f * fCalU / (fTitV * 1000.0f)));
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
      int wFill = configStore.getSampleCalRevolutions();
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
      int rVol = (int)(configStore.getSampleCalRevolutions() * 1.2f);
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
      float cwRpm = configStore.getSampleMaxRPM();
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
    case 'P':
      measureKHPrecisionTest();
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
  if (titVol <= 0) {
    publishError("Error: titration volume is zero, cannot track HCl!");
    return;
  }
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

    float stallRemoval = diagStallRamp(70.0f, 500.0f, 5.0f, 3,
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

    float stallFill = diagStallRamp(70.0f, 500.0f, 5.0f, 3,
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

    if (sampleStallRPM > 0) {
      char buf[80];
      snprintf(buf, sizeof(buf), "Sample stall at %.0f RPM (removal:%.0f fill:%.0f, max: %.0f RPM)",
               sampleStallRPM, stallRemoval, stallFill, sampleRecommendedRPM);
      publishMessage(buf);
    } else {
      publishMessage("Sample: no stall detected within test range (70-500 RPM)");
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

    const int TRIT_RAMP_MAX = 28;  // (300-30)/10 * 1 = 27 max + 1
    SGSample titRampSamples[TRIT_RAMP_MAX];
    int titRampCount = 0;

    float titStall = diagStallRampTitrate(70.0f, 300.0f, 10.0f, 1,
                                    titRampSamples, TRIT_RAMP_MAX, &titRampCount,
                                    true, stallRpmProgress);

    setTitrateSpreadCycle(configStore.getTitrateSpreadCycle());

    if (titStall > 0) {
      titrateStallRPM = titStall;
    }
    titrateRecommendedRPM = titrateStallRPM > 0 ? titrateStallRPM * 0.8f : 0.0f;

    if (titrateStallRPM > 0) {
      char buf[64];
      snprintf(buf, sizeof(buf), "Titration stall at %.0f RPM (max: %.0f RPM)",
               titrateStallRPM, titrateRecommendedRPM);
      publishMessage(buf);
    } else {
      publishMessage("Titration: no stall detected within test range (70-300 RPM)");
    }

    // Subtract HCl used by titration diagnostics:
    // 2× diagStepTitrate(DIAG_REVS) + stall ramp (5 warmup + titRampCount ramp revs)
    int diagTitrateRevs = DIAG_REVS * 2 + 5 + titRampCount;
    int diagTitrateUnits = diagTitrateRevs * (STEPS_PER_REVOLUTION / MOTOR_STEPS_PER_UNIT);
    subtractHCl(diagTitrateUnits);
  }

  // --- SG Profiling (30 revolutions at operational speed) ---
  const int PROFILE_REVS = 30;
  SGSample profileSamples[PROFILE_REVS];
  uint16_t sProfileSGs[PROFILE_REVS] = {0};
  uint16_t tProfileSGs[PROFILE_REVS] = {0};
  int sProfileCount = 0, tProfileCount = 0;
  uint16_t sProfileMin = 0, tProfileMin = 0;

  if (doSample) {
    publishMessage("SG profiling sample pump...");
    setSampleSpreadCycle(configStore.getSampleSpreadCycle());
    delay(50);
    sProfileCount = diagStepSample(PROFILE_REVS, configStore.getSamplePumpRPM(),
                                    profileSamples, PROFILE_REVS);
    sProfileMin = 65535;
    for (int i = 0; i < sProfileCount; i++) {
      sProfileSGs[i] = profileSamples[i].sg;
      if (profileSamples[i].sg < sProfileMin) sProfileMin = profileSamples[i].sg;
    }
    if (sProfileCount == 0) sProfileMin = 0;
  }

  if (doTitrate) {
    publishMessage("SG profiling titration pump...");
    setTitrateSpreadCycle(configStore.getTitrateSpreadCycle());
    delay(50);
    tProfileCount = diagStepTitrate(PROFILE_REVS, TITRATION_RPM,
                                     profileSamples, PROFILE_REVS);
    tProfileMin = 65535;
    for (int i = 0; i < tProfileCount; i++) {
      tProfileSGs[i] = profileSamples[i].sg;
      if (profileSamples[i].sg < tProfileMin) tProfileMin = profileSamples[i].sg;
    }
    if (tProfileCount == 0) tProfileMin = 0;
    // Subtract HCl for profile run
    int profileUnits = PROFILE_REVS * (STEPS_PER_REVOLUTION / MOTOR_STEPS_PER_UNIT);
    subtractHCl(profileUnits);
  }

  // Build JSON — need larger buffer for profile arrays
  const int JSON_SIZE = 2048;
  char* json = (char*)malloc(JSON_SIZE);
  if (!json) { publishError("Motor diag: out of memory"); return; }
  int pos = snprintf(json, JSON_SIZE,
    "{\"type\":\"motorDiag\",\"mode\":\"%c\","
    "\"sample\":{\"stealthchop\":{\"sgMin\":%d,\"sgMax\":%d,\"sgAvg\":%d},"
    "\"spreadcycle\":{\"sgMin\":%d,\"sgMax\":%d,\"sgAvg\":%d},"
    "\"recommended\":\"%s\",\"suggestedThreshold\":%d,"
    "\"stallRPM\":%.0f,\"maxRPM\":%.0f},"
    "\"titrate\":{\"stealthchop\":{\"sgMin\":%d,\"sgMax\":%d,\"sgAvg\":%d},"
    "\"spreadcycle\":{\"sgMin\":%d,\"sgMax\":%d,\"sgAvg\":%d},"
    "\"recommended\":\"%s\",\"suggestedThreshold\":%d,"
    "\"stallRPM\":%.0f,\"maxRPM\":%.0f},"
    "\"sgProfile\":{\"sample\":{\"min\":%d,\"recSG\":%d,\"values\":[",
    mode,
    scSample.sgMin, scSample.sgMax, (int)scSample.sgAvg,
    spSample.sgMin, spSample.sgMax, (int)spSample.sgAvg,
    sampleRecSC ? "stealthchop" : "spreadcycle", sBestMin / 2,
    sampleStallRPM, sampleRecommendedRPM,
    scTitrate.sgMin, scTitrate.sgMax, (int)scTitrate.sgAvg,
    spTitrate.sgMin, spTitrate.sgMax, (int)spTitrate.sgAvg,
    titrateRecSC ? "stealthchop" : "spreadcycle", tBestMin / 2,
    titrateStallRPM, titrateRecommendedRPM,
    (int)sProfileMin, (int)(sProfileMin * 0.4f));
  for (int i = 0; i < sProfileCount && pos < JSON_SIZE - 100; i++) {
    pos += snprintf(json + pos, JSON_SIZE - pos, "%s%d", i ? "," : "", sProfileSGs[i]);
  }
  pos += snprintf(json + pos, JSON_SIZE - pos,
    "]},\"titrate\":{\"min\":%d,\"recSG\":%d,\"values\":[",
    (int)tProfileMin, (int)(tProfileMin * 0.4f));
  for (int i = 0; i < tProfileCount && pos < JSON_SIZE - 50; i++) {
    pos += snprintf(json + pos, JSON_SIZE - pos, "%s%d", i ? "," : "", tProfileSGs[i]);
  }
  snprintf(json + pos, JSON_SIZE - pos, "]}}}");
  broadcastMotorDiag(json);
  free(json);
  publishMessage("Motor diagnostics complete");
}

// --- Pump calibration ---

void calibrateSamplePump() {
  publishMessage("Calibrating sample pump: removing old water...");
  broadcastProgress(0);
  float sampRpm = configStore.getSamplePumpRPM();
  int calRevs = configStore.getSampleCalRevolutions();
  // Remove existing water first (same volume as calibration run)
  if (!removeSample(calRevs, sampRpm)) {
    publishError(wasMotorStall() ? "Error: sample pump stall during remove" : "Error: sample pump timeout during remove");
    broadcastProgress(100);
    return;
  }
  broadcastProgress(50);
  delay(500);
  char buf[80];
  snprintf(buf, sizeof(buf), "Taking %d revolutions at %.0f RPM",
           calRevs, sampRpm);
  publishMessage(buf);
  if (!takeSample(calRevs, sampRpm)) {
    publishError(wasMotorStall() ? "Error: sample pump stall during calibration" : "Error: sample pump timeout during calibration");
    broadcastProgress(100);
    return;
  }
  configStore.setSampleCalTimestamp((uint32_t)time(nullptr));
  publishMessage("Sample pump calibration done. Measure dispensed volume and enter in 'Sample Cal Volume (mL)'.");
  broadcastProgress(100);
}

void calibrateTitrationPump() {
  int targetUnits = configStore.getCalUnits();
  char buf[80];
  snprintf(buf, sizeof(buf), "Calibrating titration pump (%d revolutions)", targetUnits / 100);
  publishMessage(buf);
  broadcastProgress(0);

  units = 0;
  // Batch size ~65 µL for stall detection between batches
  float calU_cal = (float)configStore.getCalUnits();
  float titV_cal = configStore.getTitrationVolume();
  float unitsPerUL_cal = (titV_cal > 0) ? calU_cal / (titV_cal * 1000.0f) : 1.0f;
  const int BATCH = max(4, (int)round(65.0f * unitsPerUL_cal));

  while (units < targetUnits) {
    int batch = min(BATCH, targetUnits - units);
    if (!titrate(batch, TITRATION_RPM)) {
      publishError(wasMotorStall() ? "Error: titration pump stall during calibration" : "Error: titration pump timeout during calibration");
      if (units > 0) subtractHCl(units);
      units = 0;
      digitalWrite(EN_PIN2, HIGH);
      broadcastProgress(100);
      return;
    }
    units += batch;
    int pct = (units * 99) / targetUnits;  // cap at 99% until fully done
    broadcastProgress(pct);
    delay(TITRATION_MIX_DELAY_FAST_MS);
    ArduinoOTA.handle();
    snprintf(buf, sizeof(buf), "Cal: %d / %d revolutions", units / 100, targetUnits / 100);
    publishMessage(buf);
  }

  subtractHCl(targetUnits);
  units = 0;
  digitalWrite(EN_PIN2, HIGH);
  configStore.setTitrationCalTimestamp((uint32_t)time(nullptr));
  publishMessage("Pump calibration done");
  broadcastProgress(100);
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
static int computeFastBatch(float currentPH, float thresholdPH, int batchMax, int batchMin) {
  if (currentPH >= FAST_RAMP_START_PH) return batchMax;
  float lower = thresholdPH + 0.5f;
  if (currentPH <= lower) return batchMin;
  float fraction = (currentPH - lower) / (FAST_RAMP_START_PH - lower);
  return batchMin + (int)(fraction * (batchMax - batchMin));
}

// --- MeasureKH ---

KHResult measureKH() {
  KHResult result = {};
  result.khValue = NAN;
  result.crossValDiff = NAN;

  // Re-entrancy guard: prevent concurrent measurements
  if (isMeasuringKH) {
    publishError("Measurement already in progress");
    return result;
  }
  isMeasuringKH = true;
  abortRequested = false;  // Clear any stale abort

  // Read water temperature from sensor (or use default if no sensor)
  float waterTemp = getWaterTemperatureC();
  configStore.setMeasTempC(waterTemp);
  updateNernstTempCorrection(waterTemp);  // Adjust calibration slopes for measurement temperature
  Serial.printf("Water temperature: %.1f °C%s\n", waterTemp,
                hasTemperatureSensor() ? "" : " (default, no sensor)");

  // Configure stabilization from NVS and reset per-measurement stats
  setStabilizationTimeoutMs(configStore.getStabilizationTimeout());
  resetStabilizationStats();
  resetNoiseStats();
  resetADCFilter();  // Clear stale EMA state from previous measurement

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
  if (titV <= 0 || calU <= 0) {
    publishError("Error: invalid calibration or titration volume config");
    isMeasuringKH = false;
    return result;
  }
  int prefillUnits = max(2, (int)round(prefillUL * calU / (titV * 1000.0f)));

  // Validate calibration before starting
  if (!isCalibrationValid()) {
    publishError("Error: pH calibration invalid. Re-calibrate with pH 4/7/10 buffers.");
    isMeasuringKH = false;
    return result;
  }

  if (!titrate(prefillUnits, configStore.getTitrationRPM())) {
    publishError(wasMotorStall() ? "Error: titration pump stall during prefill" : "Error: titration pump timeout during prefill");
    digitalWrite(EN_PIN2, HIGH);
    isMeasuringKH = false;
    return result;
  }
  // Keep titration motor enabled after prefill to prevent suckback
  publishMessage("Taking sample");
  float revsPerML = configStore.getSampleCalRevsPerML();
  int sampleFillRevs = configStore.getSampleCalRevolutions();
  float sampVolML = (revsPerML > 0) ? (float)sampleFillRevs / revsPerML : 0.0f;
  int sampleRemoveRevs = (int)(sampleFillRevs * 1.2f);
  // Configurable wash count: rinses clean the chamber, last wash takes the actual sample
  int numWashes = configStore.getNumWashes();
  setMultiWashContext(numWashes);
  float sampRpm = configStore.getSamplePumpRPM();
  for (int w = 0; w < numWashes; w++) {
    if (!washSampleVol(sampleRemoveRevs, sampleFillRevs, sampRpm)) {
      clearMultiWashContext();
      static char washErr[64];
      snprintf(washErr, sizeof(washErr), "Error: sample pump timeout during wash (%d/%d)", w + 1, numWashes);
      publishError(washErr);
      isMeasuringKH = false;
      return result;
    }
    measurementYield();
    if (abortRequested) { abortRequested = false; stopStirrer(); digitalWrite(EN_PIN2, HIGH); clearMultiWashContext(); publishError("Measurement aborted"); isMeasuringKH = false; return result; }
    if (w < numWashes - 1) delay(1000);
  }
  clearMultiWashContext();
  measurementYield();
  if (abortRequested) { abortRequested = false; stopStirrer(); digitalWrite(EN_PIN2, HIGH); publishError("Measurement aborted"); isMeasuringKH = false; return result; }
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
    bool rinse1 = washSampleVol((int)(sampleFillRevs * 1.5f), sampleFillRevs, sampRpm);
    delay(2000);
    bool rinse2 = washSampleVol(sampleRemoveRevs, sampleFillRevs, sampRpm);
    clearMultiWashContext();
    if (!rinse1 || !rinse2) {
      publishError("Warning: sample pump timeout during extra rinse");
      errorflag = 1;
    }
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

    // Compute unit limits from mL config using current calibration
    float maxAcidML = configStore.getMaxAcidML();
    int maxUnits = min((int)(maxAcidML * calU / titV), MAX_TITRATION_UNITS);
    float fastStepML = configStore.getFastStepUL() / 1000.0f;
    int fastBatchMax = max(20, (int)round(fastStepML * calU / titV));
    int fastBatchMin = max(2, fastBatchMax / 10);

    // --- Fast phase: adaptive batch size, reduces near threshold to avoid overshoot ---
    float lastFastPH = startPH;
    int stallCount = 0;
    float fastRPM = configStore.getFastPhaseRPM();
    publishMessage("Fast titration");

    // Data point storage — declared early so fast-phase can store points near endpoint
    static TitrationPoint dataPoints[MAX_TITRATION_POINTS];
    int nPoints = 0;
    int granCount = 0;
    static const float DATA_STORE_PH = 5.0f;

    while (pH > fastPH && units < maxUnits && errorflag == 0) {
      int batch = computeFastBatch(pH, fastPH, fastBatchMax, fastBatchMin);
      if (!titrate(batch, fastRPM)) {
        errorMessage = wasMotorStall() ? "Error: titration pump stall in fast phase" : "Error: titration pump timeout in fast phase";
        errorflag = 1;
        break;
      }
      units += batch;
      int mixDelay = TITRATION_MIX_DELAY_FAST_MS +
          max(0, fastBatchMax - batch) * 600 / fastBatchMax;
      delay(mixDelay);
      int nReadings = isExternalADCActive()
          ? 3 + max(0, fastBatchMax - batch) * 5 / fastBatchMax
          : 5 + max(0, fastBatchMax - batch) * 15 / fastBatchMax;
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
      if (pH > fastPH) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Warning: Fast phase ended at pH %.2f (%.1f mL acid limit reached)", pH, maxAcidML);
        publishMessage(buf);
      }
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

    // Cache config values at measurement start to prevent mid-measurement changes via web UI
    float cachedDropUL = configStore.getDropVolumeUL();
    float cachedCalU = (float)configStore.getCalUnits();
    float cachedTitV = configStore.getTitrationVolume();
    float cachedUnitsPerUL = cachedCalU / (cachedTitV * 1000.0f);
    int cachedGranStepVol = max(2, (int)round(cachedDropUL * cachedUnitsPerUL));
    // Medium zone: ~40 µL per step (coarser than Gran, just needs to get to Gran region)
    int cachedMediumStepVol = max(4, (int)round(40.0f * cachedUnitsPerUL));
    float cachedGranRPM = configStore.getTitrationRPM();
    int cachedGranMixDelay = configStore.getGranMixDelay();

    while (!isnan(pH) && pH > stopPH && units < maxUnits
           && errorflag == 0) {
      mqttManager.loop();

      int stepVol;
      uint8_t curPhase;
      if (pH > GRAN_REGION_PH) {
        // Medium zone (pH above Gran region): large steps, rough tracking only
        // No stabilization needed — we just need to detect when to enter Gran zone
        curPhase = 1;
        stepVol = cachedMediumStepVol;
        if (!titrate(stepVol, fastRPM)) {
          errorMessage = wasMotorStall() ? "Error: titration pump stall in precise phase" : "Error: titration pump timeout in precise phase";
          errorflag = 1;
          break;
        }
        delay(TITRATION_MIX_DELAY_MEDIUM_MS);
        measurePHStabilized(isExternalADCActive() ? 3 : 8);
      } else {
        // Gran zone (pH below GRAN_REGION_PH): smaller steps, stabilization, accurate readings
        curPhase = 2;
        stepVol = cachedGranStepVol;
        if (!titrate(stepVol, cachedGranRPM)) {
          errorMessage = wasMotorStall() ? "Error: titration pump stall in Gran zone" : "Error: titration pump timeout in Gran zone";
          errorflag = 1;
          break;
        }
        // Yielding mix delay: feed UI/MQTT every 500ms instead of blocking
        { unsigned long mixEnd = millis() + cachedGranMixDelay;
          while (millis() < mixEnd) {
            delay(500);
            measurementYield();
          }
        }
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
      if (units >= maxUnits - stepVol) {
        errorMessage = "Error: reached acid max!";
        errorflag = 1;
      }
    }

    mqttManager.loop();
    if (errorflag == 0) {
      // Get calibration parameters
      float calUnits = (float)configStore.getCalUnits();
      float titVol = configStore.getTitrationVolume();
      float revsPerMLkh = configStore.getSampleCalRevsPerML();
      float samVol = (revsPerMLkh > 0) ? (float)configStore.getSampleCalRevolutions() / revsPerMLkh : 0.0f;
      float corrF = configStore.getCorrectionFactor();
      float hclMol = configStore.getHClMolarity();

      if (calUnits <= 0 || samVol <= 0 || revsPerMLkh <= 0) {
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
        float eqUnitsSE = 0;

        if (epMethod == 0) {
          // Gran mode: try Gran analysis, fall back to endpoint if it fails
          if (granCount >= MIN_GRAN_POINTS) {
            char granReason[64] = "";
            exactUnits = granAnalysis(dataPoints, nPoints, samVol, titVol, calUnits, &granR2, &result.granWinLow, &result.granWinHigh, granReason, sizeof(granReason), &granSlope, &granIntercept, granWindows, &nGranWindows, &eqUnitsSE);
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

          // Compute 95% confidence interval from Gran regression SE
          float khCI = NAN;
          if (usedGran && eqUnitsSE > 0 && exactUnits > 0) {
            khCI = 1.96f * eqUnitsSE * (khValue / exactUnits);
          }

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
          result.khCI = khCI;

          // KH value deferred to publishKHResult() after validation
        }
      }
    } else {
      publishError(errorMessage);
    }
  }

  // Always subtract HCl used (even on error/abort — acid was dispensed regardless)
  if (units + prefillUnits > 0) {
    subtractHCl(units + prefillUnits);
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

  Serial.println("Stopping stirrer (post-titration)");
  stopStirrer();

  // Compute extra removal to compensate for HCl volume added during titration
  // hclPart = HCl volume / sample volume (in mL/mL), adds to removal fraction
  float hclPart = 0;
  {
    float calU = (float)configStore.getCalUnits();
    float titV = configStore.getTitrationVolume();
    float revsPerMLpost = configStore.getSampleCalRevsPerML();
    float samV = (revsPerMLpost > 0) ? (float)configStore.getSampleCalRevolutions() / revsPerMLpost : 0.0f;
    if (calU > 0 && samV > 0) {
      hclPart = ((float)(units + prefillUnits) / calU) * titV / samV;
    }
  }

  // Single post-wash rinse (pre-measurement double wash handles carryover)
  int postFillRevs = configStore.getSampleCalRevolutions();
  int postRemoveRevs = (int)(postFillRevs * (1.5f + hclPart));
  if (!washSampleVol(postRemoveRevs, postFillRevs, configStore.getSamplePumpRPM())) {
    publishError("Warning: sample pump timeout during post-wash");
    errorflag = 1;  // Flag result as unreliable — incomplete wash risks contaminating next measurement
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
  isMeasuringKH = false;
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
  initMotors();
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
  // Apply SGTHRS from configStore now that NVS is initialized
  // (initTMCDrivers runs before configStore.begin, so SGTHRS was left at 0)
  applyStallGuardConfig();

  // Load device name from NVS (used by mDNS, MQTT, HA, OTA, web UI)
  configStore.getDeviceName(deviceName, sizeof(deviceName));

  // Initialize ADS1115 external ADC if configured (must be after configStore.begin())
  initExternalADC();

  // Keep MQTT/OTA alive during long motor operations (washSample takes ~16 min)
  setMotorYieldCallback([]() {
    mqttManager.loop();
    ArduinoOTA.handle();
  });

  // Keep UI responsive during pH stabilization waits (up to 4s per Gran step)
  setStabilizationYieldCallback([]() {
    measurementYield();
  });

  // Report wash progress to web dashboard (WebSocket only, no MQTT during motor ops)
  setMotorProgressCallback([](int percent) {
    broadcastProgress(percent);
  });

  // Allow aborting motor operations from web UI
  setMotorAbortCallback([]() -> bool {
    return abortRequested;
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
  snprintf(MQkhCI, sizeof(MQkhCI), "%s/kh_ci", deviceName);

  // --- Triple power-cycle detection for WiFi reset ---
  {
    uint8_t bootCount = configStore.getBootCount();
    uint32_t lastBoot = configStore.getLastBootTime();
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);  // ms since boot (always small)
    // Use millis() for inter-boot timing — but on fresh boot it's ~0,
    // so we use the stored timestamp and compare with a flag approach:
    // If last boot was recent (stored as a monotonic counter), detect rapid cycles.
    // We store the millis() value AFTER setup completes. On next boot, if the stored
    // value indicates the previous boot was short-lived (<10s uptime), it's a rapid cycle.
    // Simpler approach: just use the boot count + a timeout that resets in loop().
    if (bootCount >= 2) {
      Serial.println("Triple power-cycle detected — clearing WiFi, entering AP mode");
      configStore.clearBootCount();
      configStore.clearWifiCredentials();
      // Fall through to AP mode below (hasWifiCredentials will be false)
    } else {
      configStore.setBootCount(bootCount + 1);
      Serial.printf("Boot count: %d (power-cycle 3x within 10s to reset WiFi)\n", bootCount + 1);
    }
  }

  // --- WiFi: AP mode or STA mode ---
  // BOOT button (GPIO 0) for hardware WiFi reset
  pinMode(0, INPUT_PULLUP);

  static char wifiSSID[33];
  static char wifiPass[65];
  configStore.getWifiSSID(wifiSSID, sizeof(wifiSSID));
  configStore.getWifiPassword(wifiPass, sizeof(wifiPass));

  if (strlen(wifiSSID) == 0) {
    // No WiFi credentials — start AP mode with captive portal
    Serial.println("No WiFi credentials — starting AP mode");
    wifiManager.beginAP(deviceName);
    setupAPWebServer();
    apModeActive = true;
    return;
  }

  // Try to connect to WiFi (blocking wait up to 15s for initial connection)
  wifiManager.begin(wifiSSID, wifiPass);
  {
    unsigned long wifiStart = millis();
    while (!wifiManager.isConnected() && millis() - wifiStart < 15000) {
      wifiManager.loop();
      delay(100);
    }
  }

  if (!wifiManager.isConnected()) {
    // STA connection failed — fall back to AP mode
    Serial.println("WiFi connection failed — starting AP mode");
    WiFi.disconnect(true);
    wifiManager.beginAP(deviceName);
    setupAPWebServer();
    apModeActive = true;
    return;
  }

  // --- STA mode: full operation ---

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

  // Clear boot counter after successful STA setup (prevents false triple-cycle detection)
  configStore.clearBootCount();
}

// --- BOOT button (GPIO 0) long-press check ---
static void checkBootButton() {
  if (digitalRead(0) == LOW) {
    if (bootBtnPressStart == 0) {
      bootBtnPressStart = millis();
    } else if (millis() - bootBtnPressStart >= BOOT_BTN_HOLD_MS) {
      Serial.println("BOOT button held 5s — clearing WiFi, entering AP mode");
      configStore.clearWifiCredentials();
      configStore.clearBootCount();
      delay(500);
      ESP.restart();
    }
  } else {
    bootBtnPressStart = 0;
  }
}

// --- Main loop ---

void loop() {
  // BOOT button check runs in both AP and STA modes
  checkBootButton();

  if (apModeActive) {
    // AP mode: only process DNS requests for captive portal
    wifiManager.loop();
    return;
  }

  // --- STA mode: full operation ---
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

  // Reset boot counter 10s after boot (prevents false triple power-cycle detection)
  static bool bootCounterCleared = false;
  if (!bootCounterCleared && millis() > 10000) {
    configStore.clearBootCount();
    bootCounterCleared = true;
  }
}
