#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <ESPAsyncWebServer.h>

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
};

void storeLastKHResult(const KHResult& r);
void setupWebServer();
void executeCommand(const char* cmd);
void calibratePH(int bufferPH);
void broadcastState();
void broadcastTitrationPH(float ph, int units);
void broadcastTitrationStart();
void broadcastMessage(const char* msg);
void broadcastError(const char* msg);
void broadcastProgress(int percent);
void broadcastGranData(float r2, float eqML, bool usedGran,
                       float* pointsML, float* pointsF, int nPts);
void appendHistory(const char* sensor, float value, uint32_t ts);
void appendGranHistory(float r2, float eqML, float endpointPH, bool usedGran, float confidence, float khGran, float khEndpoint, uint32_t ts);
int getRecentKHValues(float* outValues, int maxCount);
float computeKHSlope();
extern float lastConfidence;

extern AsyncWebServer server;
extern AsyncWebSocket ws;

#endif // WEB_SERVER_H
