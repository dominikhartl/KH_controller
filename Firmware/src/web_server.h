#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <ESPAsyncWebServer.h>

void setupWebServer();
void executeCommand(const char* cmd);
void broadcastState();
void broadcastTitrationPH(float ph, int units);
void broadcastTitrationStart();
void broadcastMessage(const char* msg);
void broadcastError(const char* msg);
void broadcastProgress(int percent);
void broadcastGranData(float r2, float eqML, bool usedGran,
                       float* pointsML, float* pointsF, int nPts);
void appendHistory(const char* sensor, float value);

extern AsyncWebServer server;
extern AsyncWebSocket ws;

#endif // WEB_SERVER_H
