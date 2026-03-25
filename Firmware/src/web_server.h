#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <ESPAsyncWebServer.h>
#include <atomic>

struct GranWindowResult;

struct KHResult {
  float khValue;       // NAN on error (selected method's result)
  float khGran;        // Gran KH (NAN if unavailable)
  float khEndpoint;    // Fixed-pH endpoint KH (NAN if unavailable)
  float startPH;
  float hclUsed;
  float granR2;
  float endpointPH;
  bool usedGran;
  float confidence;    // 0.0-1.0 composite quality score
  int dataPointCount;
  int stabTimeouts;
  unsigned long elapsedSec;
  float crossValDiff;  // |KH_gran - KH_endpoint|, NAN if unavailable
  int8_t rssiMin;      // WiFi RSSI min during measurement
  int8_t rssiMax;      // WiFi RSSI max during measurement
  float probeNoiseMv;  // Average probe noise (mV StdDev during stabilization)
  float stepNoisePh;   // Average |delta pH| per Gran step
  int phReversals;     // Count of pH reversals in Gran zone
  int granStepCount;   // Total Gran zone steps
  float granWinLow;    // Selected Gran window lower pH bound
  float granWinHigh;   // Selected Gran window upper pH bound
  float maxNoiseMv;    // Peak noise StdDev across all stabilizations
  int highNoiseCount;  // Count of stabilizations with noise > PROBE_NOISE_GOOD_MV
  float khCI;          // ±dKH 95% confidence interval from Gran regression
};

void storeLastKHResult(const KHResult& r);
struct TitrationPoint;
void storeAnalysisPoints(const TitrationPoint* points, int count);
void setupWebServer();
void setupAPWebServer();
void registerOTAUploadHandler();
void executeCommand(const char* cmd);
void calibratePH(int bufferPH);
void broadcastState();
void broadcastStateLight();  // Dynamic values only, zero NVS reads — safe during motor/measurement yield
void handlePendingWSClient();  // Process deferred WS connect + config change broadcasts (Core 1 only)
void broadcastTitrationPH(float ph, int units);
void broadcastTitrationStart();
void broadcastMessage(const char* msg);
void broadcastError(const char* msg);
void broadcastProgress(int percent);
void broadcastRawJson(const char* json);
void broadcastGranData(float r2, float eqML, bool usedGran,
                       float* pointsML, float* pointsF, int nPts,
                       float winLowML = 0, float winHighML = 0,
                       float slopeML = 0, float intercept = 0,
                       GranWindowResult* windows = nullptr, int nWindows = 0);
void appendHistory(const char* sensor, float value, uint32_t ts);
void appendGranHistory(float r2, float eqML, float endpointPH, bool usedGran, float confidence, float khGran, float khEndpoint, float probeNoiseMv, int phReversals, float dropUL, float titrationRPM, float khCI, uint32_t ts, float startPH = NAN, float acidEff = NAN);
void appendPrecisionHistory(uint32_t ts, int n, float mean, float sd, float vmin, float vmax, unsigned long elapsedSec);
int getRecentKHValues(float* outValues, int maxCount);
int getRecentKHValuesWithTime(uint32_t* outTimestamps, float* outValues, int maxCount);
float computeKHSlope();
extern float lastConfidence;
extern std::atomic<bool> isMeasuringKH;

extern AsyncWebServer server;
extern AsyncWebSocket ws;

// RTC crash hint for filesystem operations (survives INT_WDT/panic reset)
const char* getFSCrashHint();

#endif // WEB_SERVER_H
