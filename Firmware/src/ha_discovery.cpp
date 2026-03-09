#include "ha_discovery.h"
#include "config_store.h"
#include "scheduler.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "measurement.h"
#include "motors.h"
#include "temperature.h"
#include <config.h>
#include <ArduinoJson.h>

extern char deviceName[];
#include <time.h>
#include <math.h>

// Config state topics
char topicCfgTitVol[60];
char topicCfgSamVol[60];
char topicCfgCorrF[60];
char topicCfgHclMol[60];
char topicCfgHclVol[60];
char topicCfgCalDrops[60];
char topicCfgFastPH[60];
char topicCfgEpMethod[60];
char topicCfgMinStartPH[60];
char topicCfgStabTimeout[60];
char topicCfgSched[8][60];
char topicCfgSchedMode[60];
char topicCfgIntervalHours[60];
char topicCfgAnchorTime[60];
char topicCfgMeasTemp[60];
char topicDiagnostics[60];

// Set command topics
static char topicCfgTitVolSet[60];
static char topicCfgSamVolSet[60];
static char topicCfgCorrFSet[60];
static char topicCfgHclMolSet[60];
static char topicCfgHclVolSet[60];
static char topicCfgCalDropsSet[60];
static char topicCfgFastPHSet[60];
static char topicCfgEpMethodSet[60];
static char topicCfgMinStartPHSet[60];
static char topicCfgStabTimeoutSet[60];
static char topicCfgSchedSet[8][60];
static char topicCfgSchedModeSet[60];
static char topicCfgIntervalHoursSet[60];
static char topicCfgMeasTempSet[60];
static char topicCfgAnchorTimeSet[60];

static const char* availability_topic = nullptr;
static char availTopic[60];
static char deviceIdLower[32]; // lowercase deviceName for HA discovery paths

static void initTopics() {
  strncpy(deviceIdLower, deviceName, sizeof(deviceIdLower) - 1);
  deviceIdLower[sizeof(deviceIdLower) - 1] = '\0';
  for (char* p = deviceIdLower; *p; p++) *p = tolower(*p);

  snprintf(availTopic, sizeof(availTopic), "%s/availability", deviceName);
  availability_topic = availTopic;

  snprintf(topicCfgTitVol, sizeof(topicCfgTitVol), "%s/config/titration_vol", deviceName);
  snprintf(topicCfgSamVol, sizeof(topicCfgSamVol), "%s/config/sample_vol", deviceName);
  snprintf(topicCfgCorrF, sizeof(topicCfgCorrF), "%s/config/correction_factor", deviceName);
  snprintf(topicCfgHclMol, sizeof(topicCfgHclMol), "%s/config/hcl_molarity", deviceName);
  snprintf(topicCfgHclVol, sizeof(topicCfgHclVol), "%s/config/hcl_volume", deviceName);
  snprintf(topicCfgCalDrops, sizeof(topicCfgCalDrops), "%s/config/cal_drops", deviceName);
  snprintf(topicCfgTitVolSet, sizeof(topicCfgTitVolSet), "%s/config/titration_vol/set", deviceName);
  snprintf(topicCfgSamVolSet, sizeof(topicCfgSamVolSet), "%s/config/sample_vol/set", deviceName);
  snprintf(topicCfgCorrFSet, sizeof(topicCfgCorrFSet), "%s/config/correction_factor/set", deviceName);
  snprintf(topicCfgHclMolSet, sizeof(topicCfgHclMolSet), "%s/config/hcl_molarity/set", deviceName);
  snprintf(topicCfgHclVolSet, sizeof(topicCfgHclVolSet), "%s/config/hcl_volume/set", deviceName);
  snprintf(topicCfgCalDropsSet, sizeof(topicCfgCalDropsSet), "%s/config/cal_drops/set", deviceName);
  snprintf(topicCfgFastPH, sizeof(topicCfgFastPH), "%s/config/fast_ph", deviceName);
  snprintf(topicCfgFastPHSet, sizeof(topicCfgFastPHSet), "%s/config/fast_ph/set", deviceName);
  snprintf(topicCfgEpMethod, sizeof(topicCfgEpMethod), "%s/config/endpoint_method", deviceName);
  snprintf(topicCfgEpMethodSet, sizeof(topicCfgEpMethodSet), "%s/config/endpoint_method/set", deviceName);
  snprintf(topicCfgMinStartPH, sizeof(topicCfgMinStartPH), "%s/config/min_start_ph", deviceName);
  snprintf(topicCfgMinStartPHSet, sizeof(topicCfgMinStartPHSet), "%s/config/min_start_ph/set", deviceName);
  snprintf(topicCfgStabTimeout, sizeof(topicCfgStabTimeout), "%s/config/stab_timeout", deviceName);
  snprintf(topicCfgStabTimeoutSet, sizeof(topicCfgStabTimeoutSet), "%s/config/stab_timeout/set", deviceName);
  snprintf(topicCfgMeasTemp, sizeof(topicCfgMeasTemp), "%s/config/meas_temp", deviceName);
  snprintf(topicCfgMeasTempSet, sizeof(topicCfgMeasTempSet), "%s/config/meas_temp/set", deviceName);
  snprintf(topicDiagnostics, sizeof(topicDiagnostics), "%s/diagnostics", deviceName);

  for (int i = 0; i < 8; i++) {
    snprintf(topicCfgSched[i], sizeof(topicCfgSched[i]), "%s/config/sched_%d", deviceName, i);
    snprintf(topicCfgSchedSet[i], sizeof(topicCfgSchedSet[i]), "%s/config/sched_%d/set", deviceName, i);
  }

  snprintf(topicCfgSchedMode, sizeof(topicCfgSchedMode), "%s/config/sched_mode", deviceName);
  snprintf(topicCfgSchedModeSet, sizeof(topicCfgSchedModeSet), "%s/config/sched_mode/set", deviceName);
  snprintf(topicCfgIntervalHours, sizeof(topicCfgIntervalHours), "%s/config/interval_hours", deviceName);
  snprintf(topicCfgIntervalHoursSet, sizeof(topicCfgIntervalHoursSet), "%s/config/interval_hours/set", deviceName);
  snprintf(topicCfgAnchorTime, sizeof(topicCfgAnchorTime), "%s/config/anchor_time", deviceName);
  snprintf(topicCfgAnchorTimeSet, sizeof(topicCfgAnchorTimeSet), "%s/config/anchor_time/set", deviceName);
}

// Helper: convert minutes from midnight to "HH:MM" string
static void minsToTimeStr(uint16_t mins, char* buf, size_t len) {
  snprintf(buf, len, "%02d:%02d", mins / 60, mins % 60);
}

// Helper: convert "HH:MM" string to minutes from midnight
static uint16_t timeStrToMins(const char* str) {
  int h = 0, m = 0;
  sscanf(str, "%d:%d", &h, &m);
  return (uint16_t)(h * 60 + m);
}

static void addDeviceBlock(JsonObject& doc) {
  JsonObject dev = doc["dev"].to<JsonObject>();
  JsonArray ids = dev["ids"].to<JsonArray>();
  ids.add(deviceIdLower);
  dev["name"] = deviceName;
  dev["mf"] = "DIY";
  dev["mdl"] = deviceName;
  dev["sw"] = FW_VERSION;
  char cuBuf[64];
  snprintf(cuBuf, sizeof(cuBuf), "http://%s.local/", deviceIdLower);
  dev["cu"] = cuBuf;
}

static void publishDiscoveryPayload(const char* discoveryTopic, JsonDocument& doc) {
  char buf[768];
  size_t len = serializeJson(doc, buf, sizeof(buf));
  bool ok = mqttManager.publish(discoveryTopic, buf, true);
  if (!ok) {
    Serial.printf("HA Discovery FAILED: %s (len=%u)\n", discoveryTopic, (unsigned)len);
  }
  delay(100);
  mqttManager.getClient().loop();
}

static void publishSensorDiscovery(const char* id, const char* name, const char* statTopic,
                                    const char* unit, const char* devClass,
                                    const char* valTpl, const char* entityCat) {
  char uid[64];
  snprintf(uid, sizeof(uid), "%s_%s", deviceIdLower, id);

  JsonDocument doc;
  doc["name"] = name;
  doc["stat_t"] = statTopic;
  doc["uniq_id"] = uid;
  doc["avty_t"] = availability_topic;
  if (unit) doc["unit_of_meas"] = unit;
  if (devClass) doc["dev_cla"] = devClass;
  if (valTpl) doc["val_tpl"] = valTpl;
  if (entityCat) doc["ent_cat"] = entityCat;
  doc["stat_cla"] = "measurement";
  JsonObject root = doc.as<JsonObject>();
  addDeviceBlock(root);

  char discTopic[128];
  snprintf(discTopic, sizeof(discTopic), "homeassistant/sensor/%s/%s/config", deviceIdLower, id);
  publishDiscoveryPayload(discTopic, doc);
}

static void publishNumberDiscovery(const char* id, const char* name,
                                    const char* statTopic, const char* cmdTopic,
                                    float minVal, float maxVal, float step,
                                    const char* unit) {
  char uid[64];
  snprintf(uid, sizeof(uid), "%s_%s", deviceIdLower, id);

  JsonDocument doc;
  doc["name"] = name;
  doc["stat_t"] = statTopic;
  doc["cmd_t"] = cmdTopic;
  doc["uniq_id"] = uid;
  doc["avty_t"] = availability_topic;
  doc["min"] = minVal;
  doc["max"] = maxVal;
  doc["step"] = step;
  if (unit) doc["unit_of_meas"] = unit;
  doc["mode"] = "box";
  doc["ent_cat"] = "config";
  JsonObject root = doc.as<JsonObject>();
  addDeviceBlock(root);

  char discTopic[128];
  snprintf(discTopic, sizeof(discTopic), "homeassistant/number/%s/%s/config", deviceIdLower, id);
  publishDiscoveryPayload(discTopic, doc);
}

static void publishButtonDiscovery(const char* id, const char* name,
                                    const char* cmdTopic, const char* payload,
                                    const char* entityCat) {
  char uid[64];
  snprintf(uid, sizeof(uid), "%s_%s", deviceIdLower, id);

  JsonDocument doc;
  doc["name"] = name;
  doc["cmd_t"] = cmdTopic;
  doc["pl_prs"] = payload;
  doc["uniq_id"] = uid;
  doc["avty_t"] = availability_topic;
  if (entityCat) doc["ent_cat"] = entityCat;
  JsonObject root = doc.as<JsonObject>();
  addDeviceBlock(root);

  char discTopic[128];
  snprintf(discTopic, sizeof(discTopic), "homeassistant/button/%s/%s/config", deviceIdLower, id);
  publishDiscoveryPayload(discTopic, doc);
}

static void publishSelectDiscovery(const char* id, const char* name,
                                    const char* statTopic, const char* cmdTopic,
                                    const char** options, uint8_t optCount) {
  char uid[64];
  snprintf(uid, sizeof(uid), "%s_%s", deviceIdLower, id);

  JsonDocument doc;
  doc["name"] = name;
  doc["stat_t"] = statTopic;
  doc["cmd_t"] = cmdTopic;
  doc["uniq_id"] = uid;
  doc["avty_t"] = availability_topic;
  doc["ent_cat"] = "config";
  JsonArray opts = doc["options"].to<JsonArray>();
  for (uint8_t i = 0; i < optCount; i++) opts.add(options[i]);
  JsonObject root = doc.as<JsonObject>();
  addDeviceBlock(root);

  char discTopic[128];
  snprintf(discTopic, sizeof(discTopic), "homeassistant/select/%s/%s/config", deviceIdLower, id);
  publishDiscoveryPayload(discTopic, doc);
}

void publishAllDiscovery() {
  initTopics();

  char cmdTopic[50];
  snprintf(cmdTopic, sizeof(cmdTopic), "%s/cmd", deviceName);
  char khValueTopic[50];
  snprintf(khValueTopic, sizeof(khValueTopic), "%s/kh_value", deviceName);
  char startPhTopic[50];
  snprintf(startPhTopic, sizeof(startPhTopic), "%s/startPH", deviceName);
  char mesPhTopic[50];
  snprintf(mesPhTopic, sizeof(mesPhTopic), "%s/mes_pH", deviceName);
  char errorTopic[50];
  snprintf(errorTopic, sizeof(errorTopic), "%s/error", deviceName);
  char messageTopic[50];
  snprintf(messageTopic, sizeof(messageTopic), "%s/message", deviceName);

  // Sensors
  publishSensorDiscovery("khv3_kh", "KH", khValueTopic, "dKH", nullptr, nullptr, nullptr);
  publishSensorDiscovery("khv3_ph", "pH", startPhTopic, "pH", nullptr, nullptr, nullptr);
  { char confTopic[50];
    snprintf(confTopic, sizeof(confTopic), "%s/confidence", deviceName);
    publishSensorDiscovery("khv3_confidence", "Measurement Confidence", confTopic, nullptr, nullptr, nullptr, "diagnostic");
  }
  { char slopeTopic[50];
    snprintf(slopeTopic, sizeof(slopeTopic), "%s/kh_slope", deviceName);
    publishSensorDiscovery("khv3_kh_slope", "KH Trend", slopeTopic, "dKH/day", nullptr, nullptr, nullptr);
  }
  // Quality metrics
  { char t[50]; snprintf(t, sizeof(t), "%s/gran_r2", deviceName);
    publishSensorDiscovery("khv3_gran_r2", "Gran R\u00b2", t, nullptr, nullptr, nullptr, "diagnostic"); }

  publishSensorDiscovery("khv3_mes_ph", "Measured pH", mesPhTopic, "pH", nullptr, nullptr, nullptr);
  publishSensorDiscovery("khv3_rssi", "WiFi Signal", topicDiagnostics, "dBm", "signal_strength",
                          "{{ value_json.rssi }}", "diagnostic");
  publishSensorDiscovery("khv3_uptime", "Uptime", topicDiagnostics, "s", "duration",
                          "{{ value_json.uptime }}", "diagnostic");

  // Probe health sensors
  publishSensorDiscovery("khv3_probe_asym", "Probe Asymmetry", topicDiagnostics, "%", nullptr,
                          "{{ value_json.probe_asymmetry }}", "diagnostic");
  publishSensorDiscovery("khv3_cal_age", "Calibration Age", topicDiagnostics, "d", nullptr,
                          "{{ value_json.cal_age }}", "diagnostic");
  publishSensorDiscovery("khv3_sample_cal_age", "Sample Pump Cal Age", topicDiagnostics, "d", nullptr,
                          "{{ value_json.sample_cal_age }}", "diagnostic");
  publishSensorDiscovery("khv3_titrate_cal_age", "Titration Pump Cal Age", topicDiagnostics, "d", nullptr,
                          "{{ value_json.titrate_cal_age }}", "diagnostic");
  publishSensorDiscovery("khv3_water_temp", "Water Temperature", topicDiagnostics, "°C", "temperature",
                          "{{ value_json.water_temp }}", nullptr);

  // Motor/tube health sensors
  publishSensorDiscovery("khv3_sample_sg", "Sample Pump Load", topicDiagnostics, nullptr, nullptr,
                          "{{ value_json.sample_sg }}", "diagnostic");
  publishSensorDiscovery("khv3_titrate_sg", "Titration Pump Load", topicDiagnostics, nullptr, nullptr,
                          "{{ value_json.titrate_sg }}", "diagnostic");
  {
    char uid[64];
    snprintf(uid, sizeof(uid), "%s_tube_health", deviceIdLower);
    JsonDocument doc;
    doc["name"] = "Tube Health";
    doc["stat_t"] = topicDiagnostics;
    doc["uniq_id"] = uid;
    doc["avty_t"] = availability_topic;
    doc["val_tpl"] = "{{ value_json.tube_health }}";
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    char dt[128];
    snprintf(dt, sizeof(dt), "homeassistant/sensor/%s/tube_health/config", deviceIdLower);
    publishDiscoveryPayload(dt, doc);
  }

  // Probe health text sensor (no unit, no state_class)
  {
    char uid[64];
    snprintf(uid, sizeof(uid), "%s_probe_health", deviceIdLower);
    JsonDocument doc;
    doc["name"] = "Probe Health";
    doc["stat_t"] = topicDiagnostics;
    doc["uniq_id"] = uid;
    doc["avty_t"] = availability_topic;
    doc["val_tpl"] = "{{ value_json.probe_health }}";
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    char dt[128];
    snprintf(dt, sizeof(dt), "homeassistant/sensor/%s/probe_health/config", deviceIdLower);
    publishDiscoveryPayload(dt, doc);
  }

  // Text sensors (no unit, no state_class)
  {
    char uid[64];
    snprintf(uid, sizeof(uid), "%s_error", deviceIdLower);
    JsonDocument doc;
    doc["name"] = "Last Error";
    doc["stat_t"] = errorTopic;
    doc["uniq_id"] = uid;
    doc["avty_t"] = availability_topic;
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    char dt[128];
    snprintf(dt, sizeof(dt), "homeassistant/sensor/%s/error/config", deviceIdLower);
    publishDiscoveryPayload(dt, doc);
  }
  {
    char uid[64];
    snprintf(uid, sizeof(uid), "%s_message", deviceIdLower);
    JsonDocument doc;
    doc["name"] = "Last Message";
    doc["stat_t"] = messageTopic;
    doc["uniq_id"] = uid;
    doc["avty_t"] = availability_topic;
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    char dt[128];
    snprintf(dt, sizeof(dt), "homeassistant/sensor/%s/message/config", deviceIdLower);
    publishDiscoveryPayload(dt, doc);
  }

  // Binary sensor - connectivity
  {
    char uid[64];
    snprintf(uid, sizeof(uid), "%s_connectivity", deviceIdLower);
    JsonDocument doc;
    doc["name"] = "Connectivity";
    doc["stat_t"] = availability_topic;
    doc["uniq_id"] = uid;
    doc["dev_cla"] = "connectivity";
    doc["pl_on"] = "online";
    doc["pl_off"] = "offline";
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    char dt[128];
    snprintf(dt, sizeof(dt), "homeassistant/binary_sensor/%s/connectivity/config", deviceIdLower);
    publishDiscoveryPayload(dt, doc);
  }

  // Number inputs
  publishNumberDiscovery("khv3_tit_vol", "Titration Volume",
                          topicCfgTitVol, topicCfgTitVolSet, 0.1, 50.0, 0.1, "mL");
  publishNumberDiscovery("khv3_sam_vol", "Sample Volume",
                          topicCfgSamVol, topicCfgSamVolSet, 1.0, 200.0, 0.1, "mL");
  publishNumberDiscovery("khv3_corr_f", "Correction Factor",
                          topicCfgCorrF, topicCfgCorrFSet, 0.5, 2.0, 0.01, nullptr);
  publishNumberDiscovery("khv3_hcl_mol", "HCl Molarity",
                          topicCfgHclMol, topicCfgHclMolSet, 0.001, 1.0, 0.001, "mol/L");
  publishNumberDiscovery("khv3_hcl_vol", "HCl Volume",
                          topicCfgHclVol, topicCfgHclVolSet, 0, 5000, 1, "mL");
  publishNumberDiscovery("khv3_cal_drops", "Calibration Units",
                          topicCfgCalDrops, topicCfgCalDropsSet, 1000, 20000, 100, nullptr);
  publishNumberDiscovery("khv3_fast_ph", "Fast Titration pH",
                          topicCfgFastPH, topicCfgFastPHSet, 4.5, 7.0, 0.1, "pH");
  publishNumberDiscovery("khv3_min_start_ph", "Min Start pH",
                          topicCfgMinStartPH, topicCfgMinStartPHSet, 6.0, 9.0, 0.1, "pH");
  publishNumberDiscovery("khv3_stab_timeout", "Stabilization Timeout",
                          topicCfgStabTimeout, topicCfgStabTimeoutSet, 500, 5000, 100, "ms");
  publishNumberDiscovery("khv3_meas_temp", "Measurement Temperature",
                          topicCfgMeasTemp, topicCfgMeasTempSet, 0.0, 40.0, 0.5, "°C");

  {
    const char* epOpts[] = {"Gran", "Fixed"};
    publishSelectDiscovery("khv3_ep_method", "Endpoint Method",
                            topicCfgEpMethod, topicCfgEpMethodSet,
                            epOpts, 2);
  }

  // Schedule text inputs (HH:MM format)
  for (int i = 0; i < 8; i++) {
    char id[20], name[30], uid[64];
    snprintf(id, sizeof(id), "sched_%d", i);
    snprintf(uid, sizeof(uid), "%s_sched_%d", deviceIdLower, i);
    snprintf(name, sizeof(name), "Schedule %d", i + 1);

    JsonDocument doc;
    doc["name"] = name;
    doc["stat_t"] = topicCfgSched[i];
    doc["cmd_t"] = topicCfgSchedSet[i];
    doc["uniq_id"] = uid;
    doc["avty_t"] = availability_topic;
    doc["pattern"] = "^([01]?[0-9]|2[0-3]):[0-5][0-9]$";
    doc["ent_cat"] = "config";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);

    char discTopic[128];
    snprintf(discTopic, sizeof(discTopic), "homeassistant/text/%s/%s/config", deviceIdLower, id);
    publishDiscoveryPayload(discTopic, doc);
  }

  // Schedule mode select
  {
    const char* modeOpts[] = {"custom", "interval", "never"};
    publishSelectDiscovery("khv3_sched_mode", "Schedule Mode",
                            topicCfgSchedMode, topicCfgSchedModeSet,
                            modeOpts, 3);
  }

  // Interval hours select
  {
    const char* intOpts[] = {"1", "2", "3", "4", "6", "8", "12", "24"};
    publishSelectDiscovery("khv3_interval_hours", "Interval Hours",
                            topicCfgIntervalHours, topicCfgIntervalHoursSet,
                            intOpts, 8);
  }

  // Anchor time text input
  {
    char uid[64];
    snprintf(uid, sizeof(uid), "%s_anchor_time", deviceIdLower);
    JsonDocument doc;
    doc["name"] = "Anchor Time";
    doc["stat_t"] = topicCfgAnchorTime;
    doc["cmd_t"] = topicCfgAnchorTimeSet;
    doc["uniq_id"] = uid;
    doc["avty_t"] = availability_topic;
    doc["pattern"] = "^([01]?[0-9]|2[0-3]):[0-5][0-9]$";
    doc["ent_cat"] = "config";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    char dt[128];
    snprintf(dt, sizeof(dt), "homeassistant/text/%s/anchor_time/config", deviceIdLower);
    publishDiscoveryPayload(dt, doc);
  }

  // Buttons
  publishButtonDiscovery("khv3_btn_kh", "Measure KH", cmdTopic, "k", nullptr);
  publishButtonDiscovery("khv3_btn_ph", "Measure pH", cmdTopic, "p", nullptr);
  publishButtonDiscovery("khv3_btn_sample", "Measure Sample", cmdTopic, "s", "config");
  publishButtonDiscovery("khv3_btn_titration", "Measure Titration", cmdTopic, "t", "config");
  publishButtonDiscovery("khv3_btn_fill", "Fill Titration", cmdTopic, "f", "config");
  publishButtonDiscovery("khv3_btn_voltage", "Measure Voltage", cmdTopic, "v", "diagnostic");
  publishButtonDiscovery("khv3_btn_cal4", "Calibrate pH 4", cmdTopic, "4", "config");
  publishButtonDiscovery("khv3_btn_cal7", "Calibrate pH 7", cmdTopic, "7", "config");
  publishButtonDiscovery("khv3_btn_cal10", "Calibrate pH 10", cmdTopic, "10", "config");
  publishButtonDiscovery("khv3_btn_restart", "Restart", cmdTopic, "o", "config");

  Serial.printf("HA Discovery published (%s, MQTT %s)\n",
                 deviceName, mqttManager.isConnected() ? "connected" : "DISCONNECTED");

  // Subscribe to config set topics
  mqttManager.subscribe(topicCfgTitVolSet);
  mqttManager.subscribe(topicCfgSamVolSet);
  mqttManager.subscribe(topicCfgCorrFSet);
  mqttManager.subscribe(topicCfgHclMolSet);
  mqttManager.subscribe(topicCfgHclVolSet);
  mqttManager.subscribe(topicCfgCalDropsSet);
  mqttManager.subscribe(topicCfgFastPHSet);
  mqttManager.subscribe(topicCfgEpMethodSet);
  mqttManager.subscribe(topicCfgMinStartPHSet);
  mqttManager.subscribe(topicCfgStabTimeoutSet);
  mqttManager.subscribe(topicCfgMeasTempSet);
  for (int i = 0; i < 8; i++) {
    mqttManager.subscribe(topicCfgSchedSet[i]);
  }
  mqttManager.subscribe(topicCfgSchedModeSet);
  mqttManager.subscribe(topicCfgIntervalHoursSet);
  mqttManager.subscribe(topicCfgAnchorTimeSet);
}

void publishAllConfigStates() {
  mqttManager.publish(topicCfgTitVol, String(configStore.getTitrationVolume(), 1).c_str(), true);
  mqttManager.publish(topicCfgSamVol, String(configStore.getSampleVolume(), 1).c_str(), true);
  mqttManager.publish(topicCfgCorrF, String(configStore.getCorrectionFactor(), 2).c_str(), true);
  mqttManager.publish(topicCfgHclMol, String(configStore.getHClMolarity(), 3).c_str(), true);
  mqttManager.publish(topicCfgHclVol, String(configStore.getHClVolume(), 0).c_str(), true);
  mqttManager.publish(topicCfgCalDrops, String(configStore.getCalUnits()).c_str(), true);
  mqttManager.publish(topicCfgFastPH, String(configStore.getFastTitrationPH(), 1).c_str(), true);
  mqttManager.publish(topicCfgEpMethod,
                      (configStore.getEndpointMethod() == 1) ? "Fixed" : "Gran", true);
  mqttManager.publish(topicCfgMinStartPH, String(configStore.getMinStartPH(), 1).c_str(), true);
  mqttManager.publish(topicCfgStabTimeout, String(configStore.getStabilizationTimeout()).c_str(), true);
  mqttManager.publish(topicCfgMeasTemp, String(configStore.getMeasTempC(), 1).c_str(), true);

  for (int i = 0; i < 8; i++) {
    char timeBuf[6];
    minsToTimeStr(configStore.getScheduleTime(i), timeBuf, sizeof(timeBuf));
    mqttManager.publish(topicCfgSched[i], timeBuf, true);
  }

  // Schedule mode
  {
    uint8_t m = configStore.getScheduleMode();
    const char* ms = (m == 2) ? "never" : (m == 1) ? "interval" : "custom";
    mqttManager.publish(topicCfgSchedMode, ms, true);
  }
  mqttManager.publish(topicCfgIntervalHours,
                      String(configStore.getIntervalHours()).c_str(), true);
  {
    char timeBuf[6];
    minsToTimeStr(configStore.getAnchorTime(), timeBuf, sizeof(timeBuf));
    mqttManager.publish(topicCfgAnchorTime, timeBuf, true);
  }
}

void publishDiagnostics() {
  JsonDocument doc;
  doc["rssi"] = wifiManager.getRSSI();
  doc["uptime"] = millis() / 1000;
  doc["heap"] = ESP.getFreeHeap();
  extern uint32_t heapMin;
  doc["heap_min"] = heapMin;

  // Probe health metrics
  float asym = getProbeAsymmetry();
  doc["probe_asymmetry"] = isnan(asym) ? 0 : asym;
  doc["probe_health"] = getProbeHealth();

  doc["water_temp"] = getWaterTemperatureC();

  // Calibration age in days
  uint32_t calTs = configStore.getCalTimestamp();
  time_t now = time(nullptr);
  if (calTs > 0 && now > 1000000000) {
    doc["cal_age"] = (int)((now - calTs) / 86400);
  } else {
    doc["cal_age"] = -1;  // No calibration timestamp recorded
  }

  // Pump calibration ages
  {
    uint32_t sampCalTs = configStore.getSampleCalTimestamp();
    uint32_t titCalTs = configStore.getTitrationCalTimestamp();
    time_t now2 = time(nullptr);
    doc["sample_cal_age"] = (sampCalTs > 0 && now2 > 1000000000) ? (int)((now2 - sampCalTs) / 86400) : -1;
    doc["titrate_cal_age"] = (titCalTs > 0 && now2 > 1000000000) ? (int)((now2 - titCalTs) / 86400) : -1;
  }

  // Motor/tube health
  uint16_t sAvg, sMin, tAvg, tMin;
  getLastSampleSGStats(&sAvg, &sMin);
  getLastTitrateSGStats(&tAvg, &tMin);
  doc["sample_sg"] = sAvg;
  doc["titrate_sg"] = tAvg;
  const char* th = getTubeHealth();
  doc["tube_health"] = th ? th : "Unknown";

  char buf[640];
  serializeJson(doc, buf, sizeof(buf));
  mqttManager.publish(topicDiagnostics, buf, true);
}

void handleConfigSet(const char* topic, const char* payload) {
  float val = atof(payload);

  if (strcmp(topic, topicCfgTitVolSet) == 0) {
    configStore.setTitrationVolume(val);
    mqttManager.publish(topicCfgTitVol, String(val, 1).c_str(), true);
  } else if (strcmp(topic, topicCfgSamVolSet) == 0) {
    configStore.setSampleVolume(val);
    mqttManager.publish(topicCfgSamVol, String(val, 1).c_str(), true);
  } else if (strcmp(topic, topicCfgCorrFSet) == 0) {
    configStore.setCorrectionFactor(val);
    mqttManager.publish(topicCfgCorrF, String(val, 2).c_str(), true);
  } else if (strcmp(topic, topicCfgHclMolSet) == 0) {
    configStore.setHClMolarity(val);
    mqttManager.publish(topicCfgHclMol, String(val, 3).c_str(), true);
  } else if (strcmp(topic, topicCfgHclVolSet) == 0) {
    configStore.setHClVolume(val);
    mqttManager.publish(topicCfgHclVol, String(val, 0).c_str(), true);
  } else if (strcmp(topic, topicCfgCalDropsSet) == 0) {
    configStore.setCalUnits((int)val);
    mqttManager.publish(topicCfgCalDrops, String((int)val).c_str(), true);
  } else if (strcmp(topic, topicCfgFastPHSet) == 0) {
    configStore.setFastTitrationPH(val);
    mqttManager.publish(topicCfgFastPH, String(val, 1).c_str(), true);
  } else if (strcmp(topic, topicCfgEpMethodSet) == 0) {
    uint8_t method = (strcmp(payload, "Fixed") == 0) ? 1 : 0;
    configStore.setEndpointMethod(method);
    mqttManager.publish(topicCfgEpMethod, (method == 1) ? "Fixed" : "Gran", true);
  } else if (strcmp(topic, topicCfgMinStartPHSet) == 0) {
    configStore.setMinStartPH(val);
    mqttManager.publish(topicCfgMinStartPH, String(val, 1).c_str(), true);
  } else if (strcmp(topic, topicCfgStabTimeoutSet) == 0) {
    configStore.setStabilizationTimeout((int)val);
    setStabilizationTimeoutMs(configStore.getStabilizationTimeout());
    mqttManager.publish(topicCfgStabTimeout, String(configStore.getStabilizationTimeout()).c_str(), true);
  } else if (strcmp(topic, topicCfgMeasTempSet) == 0) {
    configStore.setMeasTempC(val);
    mqttManager.publish(topicCfgMeasTemp, String(val, 1).c_str(), true);
  } else if (strcmp(topic, topicCfgSchedModeSet) == 0) {
    uint8_t mode = (strcmp(payload, "interval") == 0) ? 1 :
                    (strcmp(payload, "never") == 0) ? 2 : 0;
    configStore.setScheduleMode(mode);
    scheduler.resetDailyFlags();
    const char* modeStr = (mode == 2) ? "never" : (mode == 1) ? "interval" : "custom";
    mqttManager.publish(topicCfgSchedMode, modeStr, true);
  } else if (strcmp(topic, topicCfgIntervalHoursSet) == 0) {
    uint8_t h = (uint8_t)atoi(payload);
    if (configStore.setIntervalHours(h)) {
      scheduler.resetDailyFlags();
    }
    mqttManager.publish(topicCfgIntervalHours, String(configStore.getIntervalHours()).c_str(), true);
  } else if (strcmp(topic, topicCfgAnchorTimeSet) == 0) {
    uint16_t mins = timeStrToMins(payload);
    configStore.setAnchorTime(mins);
    scheduler.resetDailyFlags();
    char timeBuf[6];
    minsToTimeStr(mins, timeBuf, sizeof(timeBuf));
    mqttManager.publish(topicCfgAnchorTime, timeBuf, true);
  } else {
    // Check schedule topics (payload is "HH:MM" string)
    for (int i = 0; i < 8; i++) {
      if (strcmp(topic, topicCfgSchedSet[i]) == 0) {
        uint16_t mins = timeStrToMins(payload);
        configStore.setScheduleTime(i, mins);
        // Expand schedule count if setting a slot beyond current count
        uint8_t currentCount = configStore.getScheduleCount();
        if (i >= currentCount) {
          configStore.setScheduleCount(i + 1);
        }
        char timeBuf[6];
        minsToTimeStr(mins, timeBuf, sizeof(timeBuf));
        mqttManager.publish(topicCfgSched[i], timeBuf, true);
        break;
      }
    }
  }

  // Push updated config to WebSocket clients immediately
  broadcastState();
}
