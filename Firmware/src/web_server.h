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
void appendHistory(const char* sensor, float value);

extern AsyncWebServer server;
extern AsyncWebSocket ws;

#endif // WEB_SERVER_H
