#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
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

// RTC crash hint for main loop — survives INT_WDT/panic reset
RTC_NOINIT_ATTR static char loopCrashHint[24];
RTC_NOINIT_ATTR static uint32_t loopCrashMagic;
static const uint32_t LOOP_CRASH_MAGIC = 0xDEAD100F;
static void setLoopHint(const char* h) { strncpy(loopCrashHint, h, sizeof(loopCrashHint)-1); loopCrashHint[sizeof(loopCrashHint)-1]='\0'; loopCrashMagic = LOOP_CRASH_MAGIC; }
static void clearLoopHint() { loopCrashMagic = 0; }
static const char* getLoopCrashHint() { return (loopCrashMagic == LOOP_CRASH_MAGIC) ? loopCrashHint : nullptr; }

// --- Global state ---
// These are accessed from both loopTask and AsyncTCP task.
// Atomic for cross-task safety on ESP32 dual-core.
std::atomic<int> units{0};
// startPH: float has no std::atomic on ESP32 Arduino. Torn reads are theoretically
// possible but harmless — value is only written once per measurement, read for display.
volatile float startPH = 0;
volatile bool discoveryPublished = false;
char lastCrashInfo[64] = "";  // Populated at boot from RTC hints, shown in UI until clean boot
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

// Measurement phase: 0=idle, 1=wash, 2=titrate, 3=cleanup. Drives the progress UI.
// The client maps these to weighted slices of the overall progress bar.
std::atomic<int> currentMeasPhase{0};
void setMeasPhase(int p) { currentMeasPhase.store(p, std::memory_order_release); }


// Abort flag: set by AsyncTCP task (WebSocket handler), checked by loopTask in measurement loops
static std::atomic<bool> abortRequested{false};
void requestAbort() { abortRequested.store(true, std::memory_order_relaxed); }
bool isAbortRequested() { return abortRequested.load(std::memory_order_relaxed); }

// Yield during measurement: keeps UI responsive
// Light yield runs every call (~500ms): WiFi/MQTT/OTA + client cleanup
// Full broadcastState runs at most every 2s to avoid flooding WebSocket/SPI flash
static void measurementYield() {
  esp_task_wdt_reset();  // Feed watchdog during long measurement operations
  // Rate-limit cleanupClients to 1/s — too frequent during motor loops (every 50ms)
  // can kill clients experiencing brief network blips
  static unsigned long lastCleanup = 0;
  if (millis() - lastCleanup >= 1000) {
    lastCleanup = millis();
    ws.cleanupClients();
  }
  wifiManager.loop();
  mqttManager.loop();
  ArduinoOTA.handle();
  esp_task_wdt_reset();  // Feed again after potentially blocking network I/O
  handlePendingWSClient();  // Process deferred WS connects during measurement
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast >= 2000) {
    lastBroadcast = millis();
    broadcastStateLight();  // Zero NVS reads — avoids INT_WDT during measurement
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
char MQkhSmooth[50];
char MQkhDevAlert[50];
char MQkhDeviation[50];

// --- Deferred command execution ---
// Long-running commands (measureKH, calibrate) must run on loopTask, not AsyncTCP.
// WebSocket/MQTT handlers set pendingCmd; loop() picks it up.
void queueCommand(char cmd) {
  pendingCmd.store(cmd, std::memory_order_release);
}

void publishMessage(const char* message);
void publishError(const char* errorMessage);
void calibrateTitrationPump();
static void checkBootButton();
void subtractHCl(int unitsUsed);
int runBurstFill(bool &ok);

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
  updateCachedKH(r.khValue);
  updateCachedLastStartPH(r.startPH);
  lastConfidence = r.confidence;
  uint32_t ts = (uint32_t)time(nullptr);
  appendHistory("kh", r.khValue, ts);
  appendHistory("ph", r.startPH, ts);
  appendGranHistory(r.granR2, r.hclUsed, r.endpointPH, r.usedGran, r.confidence, r.khGran, r.khEndpoint, r.probeNoiseMv, r.phReversals, configStore.getDropVolumeUL(), configStore.getGranBurstRPM(), r.khCI, ts, r.startPH, getAcidEfficiency(), r.granWinLow, r.granWinHigh, isnan(r.granSlopeRatio) ? 0 : r.granSlopeRatio);

  // Track the systematic Gran-vs-endpoint offset (chemistry: true eq pH ≈4.3
  // vs fixed 4.5 endpoint). isSuspect() compares each run's cross-val diff
  // against this baseline instead of the raw value. Runs with an acid-delivery
  // anomaly (Gran slope out of band) are excluded — they would corrupt it.
  bool slopeOk = isnan(r.granSlopeRatio) ||
                 (r.granSlopeRatio >= GRAN_SLOPE_RATIO_MIN && r.granSlopeRatio <= GRAN_SLOPE_RATIO_MAX);
  if (!isnan(r.crossValDiff) && slopeOk) {
    float prev = configStore.getCrossValEMA();
    float ema = isnan(prev) ? r.crossValDiff : 0.2f * r.crossValDiff + 0.8f * prev;
    configStore.setCrossValEMA(ema);
  }

  // Quality metrics
  { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.5f", r.granR2);
    mqttManager.publish(MQgranR2, mqBuf, true); }
  // KH CI no longer published to HA (removed by user preference)

  // Compute and publish EMA-smoothed KH
  { float alpha = configStore.getKHEMAAlpha();
    float prev = configStore.getKHEMA();
    float ema = isnan(prev) ? r.khValue : (alpha * r.khValue + (1.0f - alpha) * prev);
    configStore.setKHEMA(ema);
    char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", ema);
    mqttManager.publish(MQkhSmooth, mqBuf, true);
  }

  // Compute and publish KH trend slope (dKH/day) from configured window
  float slope = computeKHSlope();
  computePredictionCurve();
  if (!isnan(slope)) {
    char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.3f", slope);
    mqttManager.publish(MQkhSlope, mqBuf, true);
  }

  // Update web UI with validated result
  broadcastState();
}

// --- Trend-based outlier detection ---

struct KHPrediction {
  float predicted;  // Expected KH at current time
  float sigma;      // Residual scatter (StdDev from OLS fit)
  bool  hasTrend;   // True if ≥3 points and trend is plausible
};

// Get fractional local hour (0.0–23.99) from a Unix timestamp using the device timezone.
static float localHourFrac(uint32_t ts) {
  time_t t = (time_t)ts;
  struct tm tm;
  localtime_r(&t, &tm);
  return tm.tm_hour + tm.tm_min / 60.0f;
}

// Solve a 4×4 linear system Ax = b in-place via Gaussian elimination with partial pivoting.
// aug is a 4×5 augmented matrix [A|b]. Solution returned in aug[i][4]. Returns false if singular.
static bool solve4x4(float aug[4][5]) {
  for (int col = 0; col < 4; col++) {
    // Partial pivot
    int best = col;
    float bestVal = fabsf(aug[col][col]);
    for (int row = col + 1; row < 4; row++) {
      if (fabsf(aug[row][col]) > bestVal) { bestVal = fabsf(aug[row][col]); best = row; }
    }
    if (best != col) {
      for (int k = 0; k < 5; k++) { float tmp = aug[col][k]; aug[col][k] = aug[best][k]; aug[best][k] = tmp; }
    }
    if (fabsf(aug[col][col]) < 1e-9f) return false;
    // Eliminate below
    for (int row = col + 1; row < 4; row++) {
      float f = aug[row][col] / aug[col][col];
      for (int k = col; k < 5; k++) aug[row][k] -= f * aug[col][k];
    }
  }
  // Back-substitution
  for (int row = 3; row >= 0; row--) {
    for (int k = row + 1; k < 4; k++) aug[row][4] -= aug[row][k] * aug[k][4];
    aug[row][4] /= aug[row][row];
  }
  return true;
}

// Predict expected KH at time 'now' using recent history.
// Fallback chain: ≥8 pts → linear+diurnal (4-param), 3-7 → linear (2-param), 1-2 → last value, 0 → NAN.
static KHPrediction predictKH(const uint32_t* timestamps, const float* values,
                               int count, uint32_t now) {
  KHPrediction p = { NAN, 0, false };
  if (count == 0) return p;

  if (count < 3) {
    p.predicted = values[count - 1];
    return p;
  }

  // Guard: if newest measurement is >24h old, trend is stale
  if (now > timestamps[count - 1] + 86400UL) {
    p.predicted = values[count - 1];
    return p;
  }

  uint32_t t0 = timestamps[0];

  // --- 4-parameter fit: KH = a + b*hours + c*sin(2π*localHour/24) + d*cos(2π*localHour/24) ---
  if (count >= KH_DIURNAL_MIN_POINTS) {
    // Build X'X (4×4) and X'y (4×1)
    float XtX[4][4] = {};
    float Xty[4] = {};
    for (int i = 0; i < count; i++) {
      float x1 = (float)(timestamps[i] - t0) / 3600.0f;
      float angle = 2.0f * M_PI * localHourFrac(timestamps[i]) / 24.0f;
      float row[4] = { 1.0f, x1, sinf(angle), cosf(angle) };
      for (int j = 0; j < 4; j++) {
        for (int k = j; k < 4; k++) XtX[j][k] += row[j] * row[k];
        Xty[j] += row[j] * values[i];
      }
    }
    // Fill symmetric lower triangle
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < j; k++) XtX[j][k] = XtX[k][j];

    // Augmented matrix [XtX | Xty]
    float aug[4][5];
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) aug[j][k] = XtX[j][k];
      aug[j][4] = Xty[j];
    }

    if (solve4x4(aug)) {
      float a = aug[0][4], b = aug[1][4], c = aug[2][4], d = aug[3][4];

      // Guard: reject implausible linear slope (>1 dKH/hour)
      if (fabsf(b) <= 1.0f) {
        float xNow = (float)(now - t0) / 3600.0f;
        float angleNow = 2.0f * M_PI * localHourFrac(now) / 24.0f;
        p.predicted = a + b * xNow + c * sinf(angleNow) + d * cosf(angleNow);

        // Residual scatter σ = sqrt(SSR / (n-4))
        float ssr = 0;
        for (int i = 0; i < count; i++) {
          float x1 = (float)(timestamps[i] - t0) / 3600.0f;
          float angle = 2.0f * M_PI * localHourFrac(timestamps[i]) / 24.0f;
          float pred = a + b * x1 + c * sinf(angle) + d * cosf(angle);
          float r = values[i] - pred;
          ssr += r * r;
        }
        p.sigma = sqrtf(ssr / ((float)count - 4.0f));
        p.hasTrend = true;
        return p;
      }
    }
    // Fall through to 2-parameter if singular or implausible slope
  }

  // --- 2-parameter fit: KH = intercept + slope * hours ---
  float sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
  for (int i = 0; i < count; i++) {
    float x = (float)(timestamps[i] - t0) / 3600.0f;
    float y = values[i];
    sumX += x; sumY += y; sumXX += x * x; sumXY += x * y;
  }
  float n = (float)count;
  float denom = n * sumXX - sumX * sumX;
  if (fabsf(denom) < 1e-9f) {
    p.predicted = sumY / n;
    return p;
  }

  float slope = (n * sumXY - sumX * sumY) / denom;
  float intercept = (sumY - slope * sumX) / n;

  if (fabsf(slope) > 1.0f) {
    p.predicted = sumY / n;
    return p;
  }

  float tNow = (float)(now - t0) / 3600.0f;
  p.predicted = intercept + slope * tNow;

  float ssr = 0;
  for (int i = 0; i < count; i++) {
    float x = (float)(timestamps[i] - t0) / 3600.0f;
    float residual = values[i] - (intercept + slope * x);
    ssr += residual * residual;
  }
  p.sigma = sqrtf(ssr / (n - 2.0f));
  p.hasTrend = true;
  return p;
}

// Check if a measurement is suspect (outlier vs trend, cross-validation, or poor fit)
static bool isSuspect(const KHResult& r, const KHPrediction& pred, char* reasonBuf, size_t reasonLen) {
  if (!isnan(pred.predicted)) {
    float dev = fabsf(r.khValue - pred.predicted);
    float threshold = pred.hasTrend
        ? fmaxf(KH_OUTLIER_MIN_THRESHOLD, KH_OUTLIER_SIGMA_MULT * pred.sigma)
        : KH_OUTLIER_FALLBACK_THRESHOLD;
    if (dev > threshold) {
      snprintf(reasonBuf, reasonLen, "Outlier: %.2f dKH (predicted %.2f, dev %.2f, thr %.2f)",
               r.khValue, pred.predicted, dev, threshold);
      return true;
    }
  }
  if (!isnan(r.crossValDiff)) {
    // Gran vs endpoint carries a systematic chemistry offset (~0.13 dKH: true
    // eq pH ≈4.3 vs fixed 4.5). Compare against the learned baseline so the
    // threshold tests the anomaly, not the known offset.
    float base = configStore.getCrossValEMA();
    float dev = isnan(base) ? r.crossValDiff : fabsf(r.crossValDiff - base);
    if (dev > CROSS_VALIDATION_THRESHOLD_DKH) {
      if (isnan(base)) {
        snprintf(reasonBuf, reasonLen, "Cross-val failed (diff %.2f dKH)", r.crossValDiff);
      } else {
        snprintf(reasonBuf, reasonLen, "Cross-val failed (diff %.2f, baseline %.2f dKH)", r.crossValDiff, base);
      }
      return true;
    }
  }
  if (r.granR2 > 0 && r.granR2 < configStore.getGranMinR2()) {
    snprintf(reasonBuf, reasonLen, "Poor Gran fit (R²=%.3f)", r.granR2);
    return true;
  }
  // Acid-delivery anomaly: Gran slope physically out of band (stall/air/siphon)
  if (!isnan(r.granSlopeRatio) &&
      (r.granSlopeRatio < GRAN_SLOPE_RATIO_MIN || r.granSlopeRatio > GRAN_SLOPE_RATIO_MAX)) {
    snprintf(reasonBuf, reasonLen, "Acid delivery anomaly (Gran slope ratio %.2f)", r.granSlopeRatio);
    return true;
  }
  return false;
}

// Returns true if r deviates from pred by more than the outlier threshold.
// Returns false (and *deviationOut = NAN) if no prediction is available.
// Threshold matches isSuspect() so the alert tracks internal validation logic.
static bool isDeviation(const KHResult& r, const KHPrediction& pred,
                        float* deviationOut) {
  if (isnan(pred.predicted)) {
    if (deviationOut) *deviationOut = NAN;
    return false;
  }
  float dev = r.khValue - pred.predicted;
  if (deviationOut) *deviationOut = dev;
  float threshold = pred.hasTrend
      ? fmaxf(KH_OUTLIER_MIN_THRESHOLD, KH_OUTLIER_SIGMA_MULT * pred.sigma)
      : KH_OUTLIER_FALLBACK_THRESHOLD;
  return fabsf(dev) > threshold;
}

// Publish retained ALERT/OK + signed deviation magnitude for HA.
static void publishDeviationAlert(const KHResult& r, const KHPrediction& pred) {
  float dev;
  bool alert = isDeviation(r, pred, &dev);
  mqttManager.publish(MQkhDevAlert, alert ? "ALERT" : "OK", true);
  char buf[16];
  if (isnan(dev)) {
    snprintf(buf, sizeof(buf), "nan");
  } else {
    snprintf(buf, sizeof(buf), "%.2f", dev);
  }
  mqttManager.publish(MQkhDeviation, buf, true);
}

// Measure KH with trend-based outlier detection and cross-validation checks
void measureKHWithValidation() {
  // Read recent timestamped history BEFORE measuring (so current measurement is not included)
  uint32_t recentTs[30];
  float recent[30];
  int histCount = getRecentKHValuesWithTime(recentTs, recent, KH_OUTLIER_HISTORY_COUNT);
  uint32_t now = (uint32_t)time(nullptr);
  KHPrediction pred = predictKH(recentTs, recent, histCount, now);

  KHResult r1 = measureKH();
  if (isnan(r1.khValue)) return;  // measurement failed
  storeLastKHResult(r1);  // store immediately so diagnostics always has data

  char reason[128];
  if (!isSuspect(r1, pred, reason, sizeof(reason))) {
    publishKHResult(r1);
    publishDeviationAlert(r1, pred);
    return;
  }

  // Suspect measurement — re-measure
  char buf[160];
  snprintf(buf, sizeof(buf), "%s. Re-measuring...", reason);
  publishMessage(buf);

  KHResult r2 = measureKH();
  if (isnan(r2.khValue)) {
    publishMessage("Re-measurement failed, keeping first value");
    publishKHResult(r1);
    publishDeviationAlert(r1, pred);
    return;
  }

  if (!isSuspect(r2, pred, reason, sizeof(reason))) {
    snprintf(buf, sizeof(buf), "Re-measurement %.2f dKH accepted", r2.khValue);
    publishMessage(buf);
    publishKHResult(r2);
    publishDeviationAlert(r2, pred);
    return;
  }

  // Both suspect — pick closest to predicted value, else smaller cross-val diff
  bool pickFirst;
  if (!isnan(pred.predicted)) {
    pickFirst = fabsf(r1.khValue - pred.predicted) <= fabsf(r2.khValue - pred.predicted);
  } else {
    float cv1 = isnan(r1.crossValDiff) ? 999.0f : r1.crossValDiff;
    float cv2 = isnan(r2.crossValDiff) ? 999.0f : r2.crossValDiff;
    pickFirst = cv1 <= cv2;
  }

  const KHResult& best = pickFirst ? r1 : r2;
  snprintf(buf, sizeof(buf), "Both suspect. Using %.2f dKH (better of two)", best.khValue);
  publishMessage(buf);
  publishKHResult(best);
  publishDeviationAlert(best, pred);
}

// Precision Test: run N consecutive full measurement cycles, report SD
static const int PRECISION_TEST_COUNT = 3;

static void measureKHPrecisionTest() {
  abortRequested = false;  // Clear any stale abort from a previous measurement
  float results[PRECISION_TEST_COUNT];
  float resultHours[PRECISION_TEST_COUNT];  // completion time in hours since test start
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
    resultHours[validCount] = (millis() - testStart) / 3600000.0f;
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

  // Drift-corrected SD: back-to-back runs span 1-1.5h, during which the tank's
  // real KH moves (diurnal consumption/dosing). Remove a linear trend so the
  // residual SD reflects measurement repeatability, not tank dynamics.
  // With n=3 this has 1 degree of freedom — treat it as indicative, not exact.
  float sdDetrended = NAN;
  float trendPerHour = NAN;
  if (validCount >= 3) {
    float mh = 0;
    for (int i = 0; i < validCount; i++) mh += resultHours[i];
    mh /= validCount;
    float sxx = 0, sxy = 0;
    for (int i = 0; i < validCount; i++) {
      float dx = resultHours[i] - mh;
      sxx += dx * dx;
      sxy += dx * (results[i] - mean);
    }
    if (sxx > 1e-6f) {
      trendPerHour = sxy / sxx;
      float ssr = 0;
      for (int i = 0; i < validCount; i++) {
        float res = (results[i] - mean) - trendPerHour * (resultHours[i] - mh);
        ssr += res * res;
      }
      sdDetrended = sqrtf(ssr / (validCount - 2));
    }
  }

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
  if (!isnan(sdDetrended)) {
    snprintf(buf, sizeof(buf),
      "Drift-corrected: ±%.3f dKH SD (tank trend %+.2f dKH/h removed)",
      sdDetrended, trendPerHour);
    publishMessage(buf);
  }

  // Store persistently
  uint32_t ts = (uint32_t)time(nullptr);
  appendPrecisionHistory(ts, validCount, mean, sd, vmin, vmax, elapsed,
                         isnan(sdDetrended) ? 0 : sdDetrended,
                         isnan(trendPerHour) ? 0 : trendPerHour);
}

void calibrateSamplePump();               // forward declaration

static void runMotorDiagnostic(char mode);
static void runDispenseTest(char mode);

void processPendingCommand() {
  char cmd = pendingCmd.load(std::memory_order_acquire);
  if (cmd == 0) return;
  pendingCmd.store(0, std::memory_order_release);
  abortRequested = false;  // Clear stale abort from previous command

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

      if (isEZOActive()) {
        // EZO precision test: 20 readings, report mean/min/max/SD
        const int N = 20;
        float readings[N];
        int validCount = 0;
        for (int i = 0; i < N; i++) {
          float r = ezoReadPH(pTemp);
          if (!isnan(r)) readings[validCount++] = r;
          esp_task_wdt_reset();
        }
        stopStirrer();
        if (validCount < 2) {
          publishError("Error: EZO pH probe not responding");
        } else {
          float sum = 0, minV = readings[0], maxV = readings[0];
          for (int i = 0; i < validCount; i++) {
            sum += readings[i];
            if (readings[i] < minV) minV = readings[i];
            if (readings[i] > maxV) maxV = readings[i];
          }
          float mean = sum / validCount;
          float sumSq = 0;
          for (int i = 0; i < validCount; i++) {
            float d = readings[i] - mean;
            sumSq += d * d;
          }
          float sd = sqrtf(sumSq / (validCount - 1));
          pH = mean;  // update global pH for UI display
          char phBuf[16];
          snprintf(phBuf, sizeof(phBuf), "%.2f", mean);
          mqttManager.publish(MQmespH, phBuf, true);
          char msgBuf[96];
          snprintf(msgBuf, sizeof(msgBuf),
            "pH: %.2f (min: %.2f, max: %.2f, SD: %.3f, n=%d)",
            mean, minV, maxV, sd, validCount);
          publishMessage(msgBuf);
        }
      } else {
        // Analog pH: single reading as before
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
      }
      broadcastState();
      break;
    }
    case 'f': {
      publishMessage("Filling (burst mode)");
      bool fillOk = true;
      int unitsDispensed = runBurstFill(fillOk);  // titrate() auto-enables EN_PIN2
      if (!fillOk) {
        publishError("Error: titration pump timeout during fill");
      } else {
        publishMessage("Fill done");
      }
      subtractHCl(unitsDispensed);
      digitalWrite(EN_PIN2, HIGH);  // standalone: disable driver after
      broadcastState();
      break;
    }
    case 'G': {
      publishMessage("Gran burst test: 5 drops");
      float gCalU  = (float)configStore.getCalUnits();
      float gTitV  = configStore.getTitrationVolume();
      float gDropUL = configStore.getDropVolumeUL();
      int gStepVol  = max(2, (int)round(gDropUL * gCalU / (gTitV * 1000.0f)));
      float gRPM    = configStore.getGranBurstRPM();
      uint32_t gAccel = configStore.getGranBurstAccel();
      digitalWrite(EN_PIN2, LOW);
      delay(MOTOR_ENABLE_DELAY_MS);
      for (int i = 0; i < 5; i++) {
        if (!titrate(gStepVol, gRPM, false, gAccel)) { publishError("Error: titration pump timeout during Gran burst test"); break; }
        delay(1000);
      }
      digitalWrite(EN_PIN2, HIGH);
      publishMessage("Gran burst test done");
      break;
    }
    case 's': {
      stopStirrer();
      publishMessage("Washing sample");
      int wFill = configStore.getSampleCalRevolutions();
      int wRemove = (int)(wFill * 1.5f);
      if (!washSampleVol(wRemove, wFill, configStore.getSamplePumpRPM())) {
        publishError("Error: sample pump timeout during wash");
      } else {
        publishMessage("Wash done");
      }
      broadcastState();
      break;
    }
    case 'r': {
      stopStirrer();
      publishMessage("Removing sample");
      int rVol = (int)(configStore.getSampleCalRevolutions() * 1.5f);
      if (!removeSample(rVol, configStore.getSamplePumpRPM())) {
        publishError("Error: sample pump timeout during remove");
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
        cwOk = removeSample((int)(cwRevs * 1.5f), cwRpm);
        if (cwOk) cwOk = takeSample(cwRevs, cwRpm);
      }
      if (cwOk) {
        publishMessage("Tube cleaning done");
      } else {
        publishError("Tube cleaning failed (stall/timeout)");
      }
      break;
    }
    case 'd':  // Both motors ramp test
    case 'B':  // Sample pump only
    case 'C':  // Titration pump only
      runMotorDiagnostic(cmd);
      break;
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
    case 'X':  // dispense test: fast mode
    case 'Y':  // dispense test: Gran bursts 200ms
    case 'Z':  // dispense test: Gran bursts 7s (realistic)
      runDispenseTest(cmd);
      break;
  }
}

// --- Motor ramp diagnostic ---
static void runMotorDiagnostic(char mode) {
  bool doSample  = (mode == 'd' || mode == 'B');
  bool doTitrate = (mode == 'd' || mode == 'C');

  abortRequested = false;

  struct RampResult { float maxRPM; bool aborted; };
  RampResult sResult = {0, false};
  RampResult tResult = {0, false};

  auto rpmCb = [](float rpm) {
    char buf[48];
    snprintf(buf, sizeof(buf), "Ramp: %.0f RPM", rpm);
    publishMessage(buf);
  };

  if (doSample) {
    publishMessage("Sample pump ramp test...");
    float maxSampleRPM = configStore.getSampleMaxRPM();
    if (maxSampleRPM < 100) maxSampleRPM = 500;
    float stoppedAt = 0;
    bool ok = motorRampTest(true, 70, maxSampleRPM, 10, 3, rpmCb, &stoppedAt);
    sResult.maxRPM = stoppedAt;
    sResult.aborted = !ok;
    if (ok) {
      publishMessage("Sample pump: OK");
      configStore.setSampleMaxRPM(stoppedAt);
    } else {
      char buf[64];
      snprintf(buf, sizeof(buf), "Sample pump: %s at %.0f RPM",
               abortRequested ? "aborted" : "stopped", stoppedAt);
      publishMessage(buf);
      if (!abortRequested && stoppedAt > 0) configStore.setSampleMaxRPM(stoppedAt);
    }
  }

  if (doTitrate && !abortRequested) {
    publishMessage("Titration pump ramp test...");
    float maxTitrateRPM = configStore.getTitrateMaxRPM();
    if (maxTitrateRPM < 100) maxTitrateRPM = 300;
    float stoppedAt = 0;
    uint32_t burstAccel = configStore.getGranBurstAccel();
    bool ok = motorRampTest(false, 70, maxTitrateRPM, 10, 3, rpmCb, &stoppedAt, burstAccel);
    tResult.maxRPM = stoppedAt;
    tResult.aborted = !ok;
    if (ok) {
      publishMessage("Titration pump: OK");
      configStore.setTitrateMaxRPM(stoppedAt);
    } else {
      char buf[64];
      snprintf(buf, sizeof(buf), "Titration pump: %s at %.0f RPM",
               abortRequested ? "aborted" : "stopped", stoppedAt);
      publishMessage(buf);
      if (!abortRequested && stoppedAt > 0) configStore.setTitrateMaxRPM(stoppedAt);
    }
  }

  // Broadcast result JSON to UI
  char json[256];
  int pos = snprintf(json, sizeof(json), "{\"type\":\"motorDiag\"");
  if (doSample)
    pos += snprintf(json + pos, sizeof(json) - pos,
      ",\"sample\":{\"maxRPM\":%.0f,\"aborted\":%s}",
      sResult.maxRPM, sResult.aborted ? "true" : "false");
  if (doTitrate)
    pos += snprintf(json + pos, sizeof(json) - pos,
      ",\"titrate\":{\"maxRPM\":%.0f,\"aborted\":%s}",
      tResult.maxRPM, tResult.aborted ? "true" : "false");
  snprintf(json + pos, sizeof(json) - pos, "}");
  broadcastRawJson(json);
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
  updateCachedHClVol(remaining);
}

// Two-phase bubble-clearing fill, shared by the Fill button and the
// pre-measurement fill. Phase 1: detach bursts of FILL_BURST_UL; Phase 2:
// flush pulses of FILL_PULSE_UL — both at Gran-burst speed, 200 ms apart.
// Counts come from config. Honors abortRequested (breaks early, returning
// units dispensed so far). Relies on titrate()'s auto-enable and leaves
// EN_PIN2 enabled; the caller disables it if standalone. Sets ok=false on
// pump timeout. Returns total units dispensed (for HCl accounting).
int runBurstFill(bool &ok) {
  ok = true;
  float calU = (float)configStore.getCalUnits();
  float titV = configStore.getTitrationVolume();
  if (calU <= 0 || titV <= 0) { ok = false; return 0; }

  float burstRPM      = configStore.getGranBurstRPM();
  uint32_t burstAccel = configStore.getGranBurstAccel();
  int burstCount = configStore.getFillBurstCount();
  int pulseCount = configStore.getFillPulseCount();
  int burstUnits = max(2, (int)round(FILL_BURST_UL * calU / (titV * 1000.0f)));
  int pulseUnits = max(2, (int)round(FILL_PULSE_UL * calU / (titV * 1000.0f)));
  int dispensed = 0;

  // Phase 1: detach bubbles with small bursts
  for (int i = 0; i < burstCount; i++) {
    if (abortRequested) return dispensed;
    if (!titrate(burstUnits, burstRPM, false, burstAccel)) { ok = false; return dispensed; }
    dispensed += burstUnits;
    if (i < burstCount - 1) delay(200);
  }
  // Phase 2: flush bubbles out with larger pulses
  for (int i = 0; i < pulseCount; i++) {
    if (abortRequested) return dispensed;
    if (!titrate(pulseUnits, burstRPM, false, burstAccel)) { ok = false; return dispensed; }
    dispensed += pulseUnits;
    if (i < pulseCount - 1) delay(200);
  }
  return dispensed;
}


// --- Pump calibration ---

void calibrateSamplePump() {
  broadcastProgress(0);
  float sampRpm = configStore.getSamplePumpRPM();
  int fillRevs = configStore.getSampleCalRevolutions();
  int removeRevs = (int)(fillRevs * 1.5f);
  // Match the measurement's multi-wash cycle so the weighed volume
  // includes the same residual water that remains during a real measurement.
  int numWashes = configStore.getNumWashes();
  {
    char logBuf[96];
    snprintf(logBuf, sizeof(logBuf), "CAL: fill=%d rem=%d rpm=%.0f washes=%d revsPerML=%.2f",
             fillRevs, removeRevs, sampRpm, numWashes,
             configStore.getSampleCalRevsPerML());
    publishMessage(logBuf);
  }
  setMultiWashContext(numWashes);
  publishMessage("Calibrating sample pump...");
  // Match the measurement exactly: scavenge before the final fill when enabled,
  // so the weighed volume includes the same residual as a real measurement
  int calScavRevs = configStore.getScavengeEnabled() ? max(1, fillRevs / 10) : 0;
  for (int w = 0; w < numWashes; w++) {
    if (!washSampleVol(removeRevs, fillRevs, sampRpm,
                       (w == numWashes - 1) ? calScavRevs : 0)) {
      clearMultiWashContext();
      publishError("Error: sample pump timeout during calibration");
      broadcastProgress(100);
      return;
    }
    if (w < numWashes - 1) delay(1000);
  }
  clearMultiWashContext();
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
  float calU_cal       = (float)configStore.getCalUnits();
  float titV_cal       = configStore.getTitrationVolume();
  float unitsPerUL_cal = (titV_cal > 0) ? calU_cal / (titV_cal * 1000.0f) : 1.0f;

  // Match measurement's three-phase titration structure so the calibration
  // volume reflects the same mix of fast/medium/Gran-speed dispensing.
  const int FAST_STEP  = max(4, (int)round(configStore.getFastStepUL() * unitsPerUL_cal));
  float fastRPM_cal    = configStore.getFastPhaseRPM();

  // Medium phase: ~40 µL steps at fast RPM (matches measurement's medium zone)
  const int MEDIUM_STEP       = max(4, (int)round(40.0f * unitsPerUL_cal));
  const int MEDIUM_STEPS      = 20;
  const int mediumPhaseUnits  = MEDIUM_STEPS * MEDIUM_STEP;

  // Gran phase: drop-sized steps at Gran RPM (~100 steps, matches measurement's Gran zone)
  const int GRAN_STEP         = max(2, (int)round(configStore.getDropVolumeUL() * unitsPerUL_cal));
  const int GRAN_STEPS        = 100;
  const int granPhaseUnits    = GRAN_STEPS * GRAN_STEP;

  const int fastPhaseTarget   = max(0, targetUnits - mediumPhaseUnits - granPhaseUnits);

  // --- Fast phase ---
  while (units < fastPhaseTarget) {
    int batch = min(FAST_STEP, fastPhaseTarget - units);
    if (!titrate(batch, fastRPM_cal)) {
      publishError("Error: titration pump timeout during calibration");
      if (units > 0) subtractHCl(units);
      units = 0;
      digitalWrite(EN_PIN2, HIGH);
      broadcastProgress(100);
      return;
    }
    units += batch;
    broadcastProgress((units * 99) / targetUnits);
    delay(TITRATION_MIX_DELAY_FAST_MS);
    ArduinoOTA.handle();
  }

  // --- Medium phase ---
  int mediumTarget = fastPhaseTarget + mediumPhaseUnits;
  while (units < mediumTarget && units < targetUnits) {
    int step = min(MEDIUM_STEP, mediumTarget - units);
    if (!titrate(step, fastRPM_cal)) {
      publishError("Error: titration pump timeout during calibration");
      if (units > 0) subtractHCl(units);
      units = 0;
      digitalWrite(EN_PIN2, HIGH);
      broadcastProgress(100);
      return;
    }
    units += step;
    broadcastProgress((units * 99) / targetUnits);
    delay(TITRATION_MIX_DELAY_MEDIUM_MS);
    ArduinoOTA.handle();
  }

  // --- Gran phase ---
  float granRPM_cal      = configStore.getGranBurstRPM();
  uint32_t granAccel_cal = configStore.getGranBurstAccel();
  while (units < targetUnits) {
    int step = min(GRAN_STEP, targetUnits - units);
    if (!titrate(step, granRPM_cal, false, granAccel_cal)) {
      publishError("Error: titration pump timeout during calibration");
      if (units > 0) subtractHCl(units);
      units = 0;
      digitalWrite(EN_PIN2, HIGH);
      broadcastProgress(100);
      return;
    }
    units += step;
    broadcastProgress((units * 99) / targetUnits);
    delay(TITRATION_MIX_DELAY_FAST_MS);
    ArduinoOTA.handle();
  }

  subtractHCl(targetUnits);
  units = 0;
  digitalWrite(EN_PIN2, HIGH);
  configStore.setTitrationCalTimestamp((uint32_t)time(nullptr));
  publishMessage("Pump calibration done");
  broadcastProgress(100);
}

// --- Single-mode dispense test ---
// Dispenses a fixed quantity using exactly ONE dosing mode so each mode can be
// weighed separately and compared against the blended pump calibration (which
// mixes fast/medium/Gran phases in proportions a real titration doesn't use).
// Discriminates speed-dependent slip (fast vs Gran-200ms) from dwell-time
// effects like in-line bubble growth (Gran-200ms vs Gran-realistic-7s).
// 7500 units ≈ 4.9 mL at current calibration → ±5 mg scale = ±0.1%.
static const int DISPENSE_TEST_UNITS = 7500;

static void runDispenseTest(char mode) {
  abortRequested = false;
  float calU = (float)configStore.getCalUnits();
  float titV = configStore.getTitrationVolume();
  if (calU <= 0 || titV <= 0) {
    publishError("Error: invalid pump calibration for dispense test");
    return;
  }
  float unitsPerUL = calU / (titV * 1000.0f);
  float expectedML = (float)DISPENSE_TEST_UNITS / unitsPerUL / 1000.0f;

  // Labels and messages stay short + ASCII: broadcastMessage serializes into a
  // bounded buffer and the activity log stores 95 chars — a multi-byte char
  // truncated mid-sequence produces an invalid WS text frame (closes strict
  // clients with 1007).
  const char* label = (mode == 'X') ? "fast" : (mode == 'Y') ? "gran-200ms" : "gran-7s";
  char buf[128];
  snprintf(buf, sizeof(buf), "Dispense test: %d units, %s mode (expect ~%.3f mL)",
           DISPENSE_TEST_UNITS, label, expectedML);
  publishMessage(buf);

  int dispensed = 0;
  bool ok = true;
  if (mode == 'X') {
    // Fast-phase replica: full batches at fast RPM, 200ms apart
    int batch = max(20, (int)round(configStore.getFastStepUL() * unitsPerUL));
    float rpm = configStore.getFastPhaseRPM();
    while (dispensed < DISPENSE_TEST_UNITS && ok && !abortRequested) {
      int step = min(batch, DISPENSE_TEST_UNITS - dispensed);
      ok = titrate(step, rpm);
      if (ok) dispensed += step;
      delay(TITRATION_MIX_DELAY_FAST_MS);
      measurementYield();
    }
  } else {
    // Gran replica: drop-sized bursts at burst RPM/accel
    int step = max(2, (int)round(configStore.getDropVolumeUL() * unitsPerUL));
    float rpm = configStore.getGranBurstRPM();
    uint32_t accel = configStore.getGranBurstAccel();
    int pauseMs = (mode == 'Y') ? 200 : 7000;
    while (dispensed < DISPENSE_TEST_UNITS && ok && !abortRequested) {
      int s = min(step, DISPENSE_TEST_UNITS - dispensed);
      ok = titrate(s, rpm, false, accel);
      if (ok) dispensed += s;
      unsigned long pauseEnd = millis() + pauseMs;
      while (millis() < pauseEnd && !abortRequested) {
        delay(100);
        esp_task_wdt_reset();
        if (pauseMs > 1000) measurementYield();
      }
    }
  }
  if (dispensed > 0) subtractHCl(dispensed);
  digitalWrite(EN_PIN2, HIGH);

  if (!ok) {
    publishError("Error: titration pump timeout during dispense test");
  } else if (abortRequested) {
    abortRequested = false;
    snprintf(buf, sizeof(buf), "Dispense test aborted at %d units", dispensed);
    publishMessage(buf);
  } else {
    snprintf(buf, sizeof(buf), "Dispense test done: %d units (%s), cal predicts %.3f mL",
             dispensed, label, expectedML);
    publishMessage(buf);
  }
  broadcastState();
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
  // Probe noise — continuous penalty starting at 4 mV (skip for EZO: no mV noise data)
  if (!isEZOActive()) {
    score -= min(0.25f, max(0.0f, (probeNoiseMv - 4.0f) * 0.05f));
  }
  // Data point count
  if (nPoints < 15) score -= 0.1f;
  if (nPoints < 10) score -= 0.1f;
  // Stabilization timeout penalty
  // ADS1115/EZO: timeouts are expected due to slow ADC/read cycles and
  // don't indicate poor measurement quality (R², reversals, cross-val are unaffected)
  if (!isExternalADCActive() && !isEZOActive()) {
    float timeoutRate = (nPoints > 0) ? (float)stabTimeouts / nPoints : 0;
    if (timeoutRate > 0.1f) score -= 0.1f;
    if (timeoutRate > 0.3f) score -= 0.1f;
  }
  // pH reversal penalty — reversals indicate probe drift or incomplete mixing.
  // No Gran-zone steps at all means the endpoint came from unstabilized
  // fast/medium data only — inherently low confidence.
  if (granStepCount > 0) {
    float reversalRate = (float)phReversals / granStepCount;
    if (reversalRate > 0.05f) score -= 0.1f;
    if (reversalRate > 0.10f) score -= 0.1f;
  } else {
    score -= 0.2f;
  }
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
  result.granSlopeRatio = NAN;

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
  setStabNoiseCaptureEnabled(true);  // Capture clean probe noise at the settled start-pH only
  resetADCFilter();      // Clear stale EMA state from previous measurement

  broadcastTitrationStart();  // Signal dashboard to clear live pH chart
  int errorflag = 0;
  units = 0;
  unsigned long measStartMs = millis();
  const char* errorMessage = "";
  publishError("");  // Clear previous error

  // Track WiFi RSSI range during measurement
  int8_t rssiMin = 127, rssiMax = -127;

  // calU / titV are reused below (tip-clear burst sizing, dilution compensation)
  float calU = (float)configStore.getCalUnits();
  float titV = configStore.getTitrationVolume();
  if (titV <= 0 || calU <= 0) {
    publishError("Error: invalid calibration or titration volume config");
    isMeasuringKH = false; setMeasPhase(0);
    return result;
  }

  // Validate calibration before starting
  if (!isCalibrationValid()) {
    publishError("Error: pH calibration invalid. Re-calibrate with pH 4/7/10 buffers.");
    isMeasuringKH = false; setMeasPhase(0);
    return result;
  }

  // Pre-measurement bubble-clearing fill — same routine as the Fill button.
  // Fill acid is flushed out before the sample, so it counts toward HCl tank
  // depletion but NOT toward the post-titration dilution compensation (hclPart).
  bool fillOk = true;
  int fillUnits = runBurstFill(fillOk);
  if (!fillOk) {
    publishError("Error: titration pump timeout during prefill");
    if (fillUnits > 0) subtractHCl(fillUnits);  // account for acid dispensed before timeout
    digitalWrite(EN_PIN2, HIGH);
    isMeasuringKH = false; setMeasPhase(0);
    return result;
  }
  // 5 Gran burst drops to eject any hanging drop from the tip before measurement
  int burstUnits = 0;  // acid dispensed by tip-clear bursts — counted for HCl accounting
  {
    float dropUL   = configStore.getDropVolumeUL();
    int burstStep  = max(2, (int)round(dropUL * calU / (titV * 1000.0f)));
    float burstRPM = configStore.getGranBurstRPM();
    uint32_t burstAccel = configStore.getGranBurstAccel();
    for (int i = 0; i < 5; i++) {
      if (abortRequested) break;  // honor abort promptly during tip-clear
      if (!titrate(burstStep, burstRPM, false, burstAccel)) {
        publishError("Error: titration pump timeout during tip clear");
        // account for fill + completed bursts dispensed before the timeout
        if (fillUnits + burstUnits > 0) subtractHCl(fillUnits + burstUnits);
        digitalWrite(EN_PIN2, HIGH);
        isMeasuringKH = false; setMeasPhase(0);
        return result;
      }
      burstUnits += burstStep;
    }
  }
  // Account for fill + tip-clear acid now: it is flushed before the sample, and
  // doing it here (rather than at end-of-run) ensures it is tracked even when a
  // later abort returns early before the final subtractHCl.
  if (fillUnits + burstUnits > 0) subtractHCl(fillUnits + burstUnits);
  // Keep titration motor enabled after fill to prevent suckback
  publishMessage("Taking sample");
  int sampleFillRevs = configStore.getSampleCalRevolutions();
  int sampleRemoveRevs = (int)(sampleFillRevs * 1.5f);
  // Configurable wash count: rinses clean the chamber, last wash takes the actual sample
  int numWashes = configStore.getNumWashes();
  setMeasPhase(1);  // wash phase
  setMultiWashContext(numWashes);
  float sampRpm = configStore.getSamplePumpRPM();
  // Scavenge (if enabled) only before the LAST fill — that's the one whose
  // residual defines the actual sample volume
  int scavRevs = configStore.getScavengeEnabled() ? max(1, sampleFillRevs / 10) : 0;
  for (int w = 0; w < numWashes; w++) {
    if (!washSampleVol(sampleRemoveRevs, sampleFillRevs, sampRpm,
                       (w == numWashes - 1) ? scavRevs : 0)) {
      clearMultiWashContext();
      static char washErr[64];
      snprintf(washErr, sizeof(washErr), "Error: sample pump timeout during wash (%d/%d)", w + 1, numWashes);
      publishError(washErr);
      digitalWrite(EN_PIN2, HIGH);  // titration driver was left enabled after fill
      isMeasuringKH = false; setMeasPhase(0);
      return result;
    }
    measurementYield();
    if (abortRequested) { abortRequested = false; stopStirrer(); digitalWrite(EN_PIN2, HIGH); clearMultiWashContext(); publishError("Measurement aborted"); isMeasuringKH = false; setMeasPhase(0); return result; }
    if (w < numWashes - 1) delay(1000);
  }
  clearMultiWashContext();
  measurementYield();
  if (abortRequested) { abortRequested = false; stopStirrer(); digitalWrite(EN_PIN2, HIGH); publishError("Measurement aborted"); isMeasuringKH = false; setMeasPhase(0); return result; }
  delay(100);
  startStirrer();
  delay(STIRRER_WARMUP_MS);  // Wait for solution to homogenize
  measurementYield();
  // Extra settling time for pH probe to equilibrate from previous
  // acidified solution to fresh sample (~5 pH unit transition)
  { unsigned long settleEnd = millis() + PROBE_SETTLE_MS;
    while (millis() < settleEnd) {
      delay(500);
      measurementYield();
    }
  }
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
    // rinse2 takes the actual sample — scavenge like the normal final wash
    bool rinse2 = washSampleVol(sampleRemoveRevs, sampleFillRevs, sampRpm, scavRevs);
    clearMultiWashContext();
    if (!rinse1 || !rinse2) {
      publishError("Warning: sample pump timeout during extra rinse");
      errorflag = 1;
    }
    delay(100);
    startStirrer();
    delay(STIRRER_WARMUP_MS);
    // Same probe settle as the first attempt — without it the retry reads a
    // systematically lower (still-settling) pH and can falsely fail or pass
    { unsigned long settleEnd = millis() + PROBE_SETTLE_MS;
      while (millis() < settleEnd) {
        delay(500);
        measurementYield();
      }
    }
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
    // Endpoint pH (used here for titration-progress estimation; epMethod is read again
    // below for the precise-phase loop, but stopPH is fixed for the whole titration)
    uint8_t titrEpMethod = configStore.getEndpointMethod();
    float titrStopPH = (titrEpMethod == 1) ? FIXED_ENDPOINT_STOP_PH : GRAN_STOP_PH;
    auto titrateProgressPct = [&](float curPH) -> int {
      if (startPH <= titrStopPH || isnan(curPH)) return 0;
      int p = (int)((startPH - curPH) / (startPH - titrStopPH) * 100.0f);
      if (p < 0) p = 0;
      if (p > 99) p = 99;
      return p;
    };
    setMeasPhase(2);  // titrate phase
    publishMessage("Fast titration");

    // Data point storage — declared early so fast-phase can store points near endpoint
    static TitrationPoint dataPoints[MAX_TITRATION_POINTS];
    int nPoints = 0;
    int granCount = 0;
    static const float DATA_STORE_PH = 5.0f;

    while (pH > fastPH && units < maxUnits && errorflag == 0) {
      int batch = computeFastBatch(pH, fastPH, fastBatchMax, fastBatchMin);
      if (!titrate(batch, fastRPM)) {
        errorMessage = "Error: titration pump timeout in fast phase";
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
      broadcastProgress(titrateProgressPct(pH));
      measurementYield();
      { int8_t rssi = wifiManager.getRSSI();
        if (rssi < rssiMin) rssiMin = rssi;
        if (rssi > rssiMax) rssiMax = rssi; }
      if (abortRequested) { errorMessage = "Measurement aborted"; errorflag = 1; break; }

      if (isnan(pH)) {
        errorMessage = "Error: pH probe not working!";
        errorflag = 1;
      } else {
        // Store data points near the endpoint even during fast phase.
        // These are unstabilized (phase 0) — used for endpoint interpolation
        // only; the Gran regression skips them.
        if (pH < DATA_STORE_PH && nPoints < MAX_TITRATION_POINTS) {
          int16_t mvI = isnan(voltage) ? 0 : (int16_t)constrain((int)lroundf(voltage), -32768, 32767);
          dataPoints[nPoints++] = {(float)units, pH, mvI, 0, TITRATION_PHASE_FAST, 0};
        }

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

    // Probe noise was captured during the settled start-pH reading. Stop capturing
    // now so titration stabilizations — contaminated by unfinished mixing of the
    // just-added acid — don't pollute the probe_noise_mv figure.
    setStabNoiseCaptureEnabled(false);

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
    float cachedGranRPM      = configStore.getGranBurstRPM();
    uint32_t cachedGranAccel = configStore.getGranBurstAccel();
    int cachedMixDelay       = configStore.getMixDelay();
    int cachedGranReadings   = configStore.getGranReadings();

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
          errorMessage = "Error: titration pump timeout in precise phase";
          errorflag = 1;
          break;
        }
        { unsigned long mixEnd = millis() + cachedMixDelay;
          while (millis() < mixEnd) {
            if (abortRequested) break;  // Don't make user wait full mix delay on abort
            delay(500);
            measurementYield();
          }
        }
        if (abortRequested) { errorMessage = "Measurement aborted"; errorflag = 1; break; }
        measurePHStabilized(isExternalADCActive() ? 3 : 8);
      } else {
        // Gran zone (pH below GRAN_REGION_PH): smaller steps, stabilization, accurate readings
        curPhase = 2;
        stepVol = cachedGranStepVol;
        if (!titrate(stepVol, cachedGranRPM, false, cachedGranAccel)) {
          errorMessage = "Error: titration pump timeout in Gran zone";
          errorflag = 1;
          break;
        }
        // First Gran-zone dose: probe is still settling from the fast/medium
        // phase exit. Give it extra mix time so this first Gran data point is read
        // on a properly settled solution. (Noise capture is already off here.)
        bool isFirstGranDose = (granStepCount == 0);
        int thisMixDelay = cachedMixDelay + (isFirstGranDose ? GRAN_FIRST_DOSE_EXTRA_MIX_MS : 0);
        // Yielding mix delay: feed UI/MQTT every 500ms instead of blocking
        { unsigned long mixEnd = millis() + thisMixDelay;
          while (millis() < mixEnd) {
            if (abortRequested) break;  // Don't make user wait full mix delay on abort
            delay(500);
            measurementYield();
          }
        }
        if (abortRequested) { errorMessage = "Measurement aborted"; errorflag = 1; break; }
        waitForPHStabilization();
        measurePHStabilized(isExternalADCActive() ? cachedGranReadings : 20);
      }
      units += stepVol;

      // Store data points near the endpoint for Gran analysis and interpolation
      if (pH < DATA_STORE_PH && nPoints < MAX_TITRATION_POINTS) {
        uint16_t sMs = (curPhase == TITRATION_PHASE_GRAN) ? (uint16_t)min((unsigned long)0xFFFF, getLastStabilizationMs()) : (uint16_t)0;
        uint8_t fl = (curPhase == TITRATION_PHASE_GRAN && getLastStabilizationTimedOut()) ? 1 : 0;
        int16_t mvI = isnan(voltage) ? 0 : (int16_t)constrain((int)lroundf(voltage), -32768, 32767);
        dataPoints[nPoints++] = {(float)units, pH, mvI, sMs, curPhase, fl};
      }
      // Count only Gran-zone (stabilized) points — the regression uses nothing else
      if (curPhase == TITRATION_PHASE_GRAN && pH < GRAN_REGION_PH) granCount++;

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
      broadcastProgress(titrateProgressPct(pH));
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
      // Acid cap reached — only an error if the endpoint was NOT reached on
      // this very step (otherwise the titration completed legitimately)
      if (units >= maxUnits - stepVol && !isnan(pH) && pH > stopPH) {
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
        // Audit the correction factor — if it's been tweaked away from 1.0 by
        // more than 5% the user is probably masking a calibration issue.
        // We warn but proceed so result is still produced.
        if (fabsf(corrF - 1.0f) > 0.05f) {
          char warnBuf[96];
          snprintf(warnBuf, sizeof(warnBuf), "Warning: correction factor %.3f deviates >5%% from 1.0", corrF);
          publishMessage(warnBuf);
        }
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

          // Acid-delivery sanity: the Gran slope beyond the equivalence point
          // physically equals the effective acid normality in the chamber
          // (≈0.56 × [HCl], see GRAN_SLOPE_RATIO_*). A collapsed slope means
          // indicated acid never arrived (stall/lost steps, air in line) or
          // was neutralized by inflow — the KH result is then inflated.
          if (usedGran && hclMol > 0) {
            float kSlope = titVol / calUnits;             // mL per unit
            float slopeML2 = granSlope / kSlope;          // F per mL
            result.granSlopeRatio = slopeML2 / hclMol;
            if (result.granSlopeRatio < GRAN_SLOPE_RATIO_MIN ||
                result.granSlopeRatio > GRAN_SLOPE_RATIO_MAX) {
              char sgBuf[128];
              snprintf(sgBuf, sizeof(sgBuf),
                       "Warning: Gran slope ratio %.2f (healthy %.2f-%.2f) - acid delivery anomaly, KH suspect",
                       result.granSlopeRatio, GRAN_SLOPE_RATIO_MIN, GRAN_SLOPE_RATIO_MAX);
              publishError(sgBuf);
            }
          }

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
                float denom = dataPoints[i].units - dataPoints[i-1].units;
                if (fabsf(denom) < 1e-6f) continue;  // identical units — avoid div-by-zero
                float frac = (exactUnits - dataPoints[i-1].units) / denom;
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

  // Subtract titration acid. Fill + tip-clear acid was already accounted for
  // right after sampling (it is flushed before the sample).
  if (units > 0) {
    subtractHCl(units);
  }

  // Titration driver thermal check — the driver stays energized for the whole
  // titration (25+ min; longer on back-to-back precision-test runs). Overtemp
  // means torque derating → lost steps → under-delivery (see Gran slope guard).
  if (isTMCDetected()) {
    uint32_t drv = getTitrateDrvStatus();
    if (drv & 0x2) {
      publishError("Warning: titration driver OVERTEMP shutdown - steps were lost");
    } else if (drv & 0x1) {
      publishError("Warning: titration driver overtemp - torque derating (stall risk)");
    }
  }

  // Anti-suckback: small reverse to prevent drip from titration nozzle.
  // Must go through FastAccelStepper — raw digitalWrite pulses never reach
  // the driver because the pins are routed to the RMT peripheral.
  titrationAntiSuckback(ANTI_SUCKBACK_STEPS);
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
      hclPart = ((float)units / calU) * titV / samV;
    }
  }

  // Single post-wash rinse (pre-measurement double wash handles carryover)
  setMeasPhase(3);  // cleanup phase
  int postFillRevs = configStore.getSampleCalRevolutions();
  int postRemoveRevs = (int)(postFillRevs * (1.5f + hclPart));
  if (!washSampleVol(postRemoveRevs, postFillRevs, configStore.getSamplePumpRPM())) {
    publishError("Warning: sample pump timeout during post-wash");
    errorflag = 1;
    // Incomplete wash risks contaminating the NEXT measurement — reduce the
    // published confidence so the result is visibly marked as unreliable
    if (!isnan(result.khValue)) result.confidence *= 0.8f;
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
  setStabNoiseCaptureEnabled(true);  // Restore default for diagnostics/calibration/next run
  isMeasuringKH = false; setMeasPhase(0);
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
  // Suppress Preferences "NOT_FOUND" errors for optional NVS keys (they return defaults)
  esp_log_level_set("Preferences", ESP_LOG_NONE);

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
  // Load device name from NVS (used by mDNS, MQTT, HA, OTA, web UI)
  configStore.getDeviceName(deviceName, sizeof(deviceName));

  // Initialize ADS1115 external ADC if configured (must be after configStore.begin())
  initExternalADC();

  // Keep WebSocket/OTA alive during long motor operations (pump runs 10-30s each).
  // Minimal work only — no WiFi reconnection (autoReconnect handles it on Core 0),
  // no MQTT reconnection (blocking TCP connect triggers INT_WDT).
  setMotorYieldCallback([]() {
    // MQTT: service existing connection only (non-blocking packet processing)
    if (mqttManager.isConnected()) {
      mqttManager.getClient().loop();
    }
    ArduinoOTA.handle();
    // Lightweight state broadcast every 2s to keep WebSocket alive (browser watchdog = 15s)
    static unsigned long lastBroadcast = 0;
    if (millis() - lastBroadcast >= 2000) {
      lastBroadcast = millis();
      handlePendingWSClient();
      broadcastStateLight();
    }
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
  snprintf(MQkhSmooth, sizeof(MQkhSmooth), "%s/kh_smooth", deviceName);
  snprintf(MQkhDevAlert, sizeof(MQkhDevAlert), "%s/kh_deviation_alert", deviceName);
  snprintf(MQkhDeviation, sizeof(MQkhDeviation), "%s/kh_deviation", deviceName);

  // --- WiFi: AP mode or STA mode ---
  // BOOT button (GPIO 0) held 5s = clear WiFi, enter AP mode
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

  // Connect to WiFi — wait up to 30s, then continue setup regardless.
  // Web server, MQTT, and OTA must be initialized even without WiFi,
  // otherwise the device is unreachable (no OTA, no UI) until WiFi connects.
  wifiManager.begin(wifiSSID, wifiPass);
  {
    unsigned long wifiStart = millis();
    while (!wifiManager.isConnected() && millis() - wifiStart < 30000) {
      wifiManager.loop();
      checkBootButton();  // Allow BOOT button AP mode entry during WiFi wait
      delay(100);
      // Log progress every 10 seconds
      if ((millis() - wifiStart) % 10000 < 100) {
        Serial.printf("WiFi connecting... (%lus)\n", (millis() - wifiStart) / 1000);
      }
    }
    if (!wifiManager.isConnected()) {
      Serial.println("WiFi not connected after 30s — continuing setup, will retry in background");
    }
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
  initBroadcastCache();  // populate RAM caches (KH, startPH, HCl) from NVS — before first broadcast
  computeKHSlope();  // populate cached slope for broadcastState()
  computePredictionCurve();
  {
    int reason = esp_reset_reason();
    Serial.printf("Reset reason: %d\n", reason);
    char bootBuf[96];
    snprintf(bootBuf, sizeof(bootBuf), "BOOT %s (reason=%d, heap=%u)", FW_BUILD, reason, ESP.getFreeHeap());
    publishMessage(bootBuf);
    if (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT) {
      // Build persistent crash info from RTC hints — stays in UI until clean boot
      const char* hint = getMotorCrashHint();
      const char* fsHint = getFSCrashHint();
      const char* loopHint = getLoopCrashHint();
      if (hint) {
        snprintf(lastCrashInfo, sizeof(lastCrashInfo), "motor: %s", hint);
      } else if (fsHint) {
        snprintf(lastCrashInfo, sizeof(lastCrashInfo), "FS: %s", fsHint);
      } else if (loopHint) {
        snprintf(lastCrashInfo, sizeof(lastCrashInfo), "loop: %s", loopHint);
      } else {
        snprintf(lastCrashInfo, sizeof(lastCrashInfo), "reason=%d (no hint)", reason);
      }
      publishMessage(lastCrashInfo);
    }
    clearMotorCrashHint();
    clearLoopHint();
    if (getEZOInitLog()[0] != '\0') {
      publishMessage(getEZOInitLog());
    }
    if (isEZOActive()) {
      publishMessage("EZO pH circuit active");
    } else if (isExternalADCActive()) {
      publishMessage("ADS1115 external ADC active");
    } else if (isExternalADCFallback()) {
      publishError("ADS1115 configured but not detected — using internal ADC");
    }
    if (isTMCDetected()) {
      publishMessage("TMC2209 stepper drivers active");
      if (!isExternalADCActive() && !isEZOActive()) {
        publishError("TMC2209 uses IO35 for DIAG — pH requires ADS1115 or EZO");
      }
    }
    initTemperature();
  }

  // Scheduler with NTP
  scheduler.begin();
  scheduler.onMeasurementDue([]() {
    setLoopHint("sched:measure");
    mqttManager.publish(MQmsg, "Scheduled measurement starting");
    measureKHWithValidation();
  });

}

// --- BOOT button (GPIO 0) long-press check ---
static void checkBootButton() {
  if (digitalRead(0) == LOW) {
    if (bootBtnPressStart == 0) {
      bootBtnPressStart = millis();
    } else if (millis() - bootBtnPressStart >= BOOT_BTN_HOLD_MS) {
      Serial.println("BOOT button held 5s — clearing WiFi, entering AP mode");
      configStore.clearWifiCredentials();
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
  setLoopHint("wifi.loop");
  wifiManager.loop();
  setLoopHint("mqtt.loop");
  mqttManager.loop();
  // ArduinoOTA: rate-limit to 1/s — mDNS processing inside handle() can block
  // interrupts long enough to trigger INT_WDT when called every loop iteration
  { static unsigned long lastOTA = 0;
    if (millis() - lastOTA >= 1000) {
      lastOTA = millis();
      setLoopHint("ota.handle");
      ArduinoOTA.handle();
    }
  }
  setLoopHint("scheduler");
  scheduler.loop();

  // Recompute slope once after NTP sync (boot-time call runs before NTP is ready)
  static bool slopeRecomputed = false;
  if (!slopeRecomputed && scheduler.isTimeSynced()) {
    slopeRecomputed = true;
    float slope = computeKHSlope();
    computePredictionCurve();
    if (!isnan(slope)) {
      char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.3f", slope);
      mqttManager.publish(MQkhSlope, mqBuf, true);
    } else {
      broadcastMessage("Trend slope: insufficient data (need 2+ calendar days in window)");
    }
    broadcastState();
  }

  setLoopHint("ws.cleanup");
  ws.cleanupClients();
  setLoopHint("pendingWS");
  handlePendingWSClient();  // Send state to newly connected WS clients (deferred from Core 0)

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
    { float ema = configStore.getKHEMA();
      if (!isnan(ema)) { char mqBuf[16]; snprintf(mqBuf, sizeof(mqBuf), "%.2f", ema);
        mqttManager.publish(MQkhSmooth, mqBuf, true); }
    }

    // New entity states (measuring, next_measurement) published in first diagnostics cycle

    discoveryPublished = true;
  }

  // Broadcast state to WebSocket clients:
  // - Every 2s: lightweight (dynamic values only, zero NVS reads)
  // - Every 30s: full state with config/probe/schedule (ensures config arrives
  //   even if the initial connect broadcast was dropped by AsyncWebSocket)
  if (millis() - lastBroadcastTime > 2000) {
    lastBroadcastTime = millis();
    static uint8_t lightCount = 0;
    if (++lightCount >= 15) {  // 15 × 2s = 30s
      lightCount = 0;
      setLoopHint("broadcastFull");
      broadcastState();
    } else {
      setLoopHint("broadcastLight");
      broadcastStateLight();
    }
  }

  // WebSocket ping for dead connection detection (every 15s)
  static unsigned long lastWsPing = 0;
  if (millis() - lastWsPing > 15000) {
    lastWsPing = millis();
    setLoopHint("ws.ping");
    for (auto& c : ws.getClients()) {
      if (c.status() == WS_CONNECTED && c.canSend()) c.ping();
    }
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

  // Proactive restart if heap stays critically low — better than an uncontrolled panic
  // that could corrupt an in-progress flash write
  {
    static unsigned long heapCriticalSince = 0;
    if (h < HEAP_RESTART_THRESHOLD) {
      if (heapCriticalSince == 0) heapCriticalSince = millis();
      else if (millis() - heapCriticalSince > 30000) {
        Serial.printf("CRITICAL: Heap below %u for 30s (current: %u). Restarting.\n",
                      HEAP_RESTART_THRESHOLD, h);
        publishError("Heap critically low — restarting");
        delay(100);
        ESP.restart();
      }
    } else {
      heapCriticalSince = 0;
    }
  }

  // Publish diagnostics + new entity states every 60s
  if (mqttManager.isConnected() && millis() - lastDiagnosticsTime > 60000) {
    lastDiagnosticsTime = millis();
    setLoopHint("diagnostics");
    publishDiagnostics();
    mqttManager.publish(topicMeasuring, isMeasuringKH ? "ON" : "OFF", true);
    { String nextM = scheduler.getNextMeasurementTime();
      mqttManager.publish(topicNextMeas, nextM.c_str(), true); }
  }

  clearLoopHint();
}
