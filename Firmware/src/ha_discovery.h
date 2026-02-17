#ifndef HA_DISCOVERY_H
#define HA_DISCOVERY_H

#include "mqtt_manager.h"

void publishAllDiscovery();
void publishAllConfigStates();
void publishDiagnostics();
void handleConfigSet(const char* topic, const char* payload);

// Config state topics for publishing current values
extern char topicCfgTitVol[60];
extern char topicCfgSamVol[60];
extern char topicCfgCorrF[60];
extern char topicCfgHclMol[60];
extern char topicCfgHclVol[60];
extern char topicCfgCalDrops[60];
extern char topicCfgFastPH[60];
extern char topicCfgSched[8][60];
extern char topicCfgSchedMode[60];
extern char topicCfgIntervalHours[60];
extern char topicCfgAnchorTime[60];
extern char topicDiagnostics[60];

#endif // HA_DISCOVERY_H
