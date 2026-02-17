#include "ha_discovery.h"
#include "config_store.h"
#include "scheduler.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "measurement.h"
#include <config.h>
#include <ArduinoJson.h>
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
char topicCfgSched[8][60];
char topicCfgSchedMode[60];
char topicCfgIntervalHours[60];
char topicCfgAnchorTime[60];
char topicDiagnostics[60];

// Set command topics
static char topicCfgTitVolSet[60];
static char topicCfgSamVolSet[60];
static char topicCfgCorrFSet[60];
static char topicCfgHclMolSet[60];
static char topicCfgHclVolSet[60];
static char topicCfgCalDropsSet[60];
static char topicCfgFastPHSet[60];
static char topicCfgSchedSet[8][60];
static char topicCfgSchedModeSet[60];
static char topicCfgIntervalHoursSet[60];
static char topicCfgAnchorTimeSet[60];

static const char* availability_topic = nullptr;
static char availTopic[60];

static void initTopics() {
  snprintf(availTopic, sizeof(availTopic), "%s/availability", DEVICE_NAME);
  availability_topic = availTopic;

  snprintf(topicCfgTitVol, sizeof(topicCfgTitVol), "%s/config/titration_vol", DEVICE_NAME);
  snprintf(topicCfgSamVol, sizeof(topicCfgSamVol), "%s/config/sample_vol", DEVICE_NAME);
  snprintf(topicCfgCorrF, sizeof(topicCfgCorrF), "%s/config/correction_factor", DEVICE_NAME);
  snprintf(topicCfgHclMol, sizeof(topicCfgHclMol), "%s/config/hcl_molarity", DEVICE_NAME);
  snprintf(topicCfgHclVol, sizeof(topicCfgHclVol), "%s/config/hcl_volume", DEVICE_NAME);
  snprintf(topicCfgCalDrops, sizeof(topicCfgCalDrops), "%s/config/cal_drops", DEVICE_NAME);
  snprintf(topicCfgTitVolSet, sizeof(topicCfgTitVolSet), "%s/config/titration_vol/set", DEVICE_NAME);
  snprintf(topicCfgSamVolSet, sizeof(topicCfgSamVolSet), "%s/config/sample_vol/set", DEVICE_NAME);
  snprintf(topicCfgCorrFSet, sizeof(topicCfgCorrFSet), "%s/config/correction_factor/set", DEVICE_NAME);
  snprintf(topicCfgHclMolSet, sizeof(topicCfgHclMolSet), "%s/config/hcl_molarity/set", DEVICE_NAME);
  snprintf(topicCfgHclVolSet, sizeof(topicCfgHclVolSet), "%s/config/hcl_volume/set", DEVICE_NAME);
  snprintf(topicCfgCalDropsSet, sizeof(topicCfgCalDropsSet), "%s/config/cal_drops/set", DEVICE_NAME);
  snprintf(topicCfgFastPH, sizeof(topicCfgFastPH), "%s/config/fast_ph", DEVICE_NAME);
  snprintf(topicCfgFastPHSet, sizeof(topicCfgFastPHSet), "%s/config/fast_ph/set", DEVICE_NAME);
  snprintf(topicDiagnostics, sizeof(topicDiagnostics), "%s/diagnostics", DEVICE_NAME);

  for (int i = 0; i < 8; i++) {
    snprintf(topicCfgSched[i], sizeof(topicCfgSched[i]), "%s/config/sched_%d", DEVICE_NAME, i);
    snprintf(topicCfgSchedSet[i], sizeof(topicCfgSchedSet[i]), "%s/config/sched_%d/set", DEVICE_NAME, i);
  }

  snprintf(topicCfgSchedMode, sizeof(topicCfgSchedMode), "%s/config/sched_mode", DEVICE_NAME);
  snprintf(topicCfgSchedModeSet, sizeof(topicCfgSchedModeSet), "%s/config/sched_mode/set", DEVICE_NAME);
  snprintf(topicCfgIntervalHours, sizeof(topicCfgIntervalHours), "%s/config/interval_hours", DEVICE_NAME);
  snprintf(topicCfgIntervalHoursSet, sizeof(topicCfgIntervalHoursSet), "%s/config/interval_hours/set", DEVICE_NAME);
  snprintf(topicCfgAnchorTime, sizeof(topicCfgAnchorTime), "%s/config/anchor_time", DEVICE_NAME);
  snprintf(topicCfgAnchorTimeSet, sizeof(topicCfgAnchorTimeSet), "%s/config/anchor_time/set", DEVICE_NAME);
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
  JsonObject dev = doc.createNestedObject("dev");
  JsonArray ids = dev.createNestedArray("ids");
  ids.add("khcontrollerv3");
  dev["name"] = "KH Controller V3";
  dev["mf"] = "DIY";
  dev["mdl"] = "KHcontrollerV3";
  dev["sw"] = "2.0.0";
  dev["cu"] = "http://khcontrollerv3.local/";
}

static void publishDiscoveryPayload(const char* discoveryTopic, JsonDocument& doc) {
  char buf[768];
  serializeJson(doc, buf, sizeof(buf));
  mqttManager.publish(discoveryTopic, buf, true);
  delay(100);
  mqttManager.getClient().loop();
}

static void publishSensorDiscovery(const char* id, const char* name, const char* statTopic,
                                    const char* unit, const char* devClass,
                                    const char* valTpl, const char* entityCat) {
  StaticJsonDocument<768> doc;
  doc["name"] = name;
  doc["stat_t"] = statTopic;
  doc["uniq_id"] = id;
  doc["avty_t"] = availability_topic;
  if (unit) doc["unit_of_meas"] = unit;
  if (devClass) doc["dev_cla"] = devClass;
  if (valTpl) doc["val_tpl"] = valTpl;
  if (entityCat) doc["ent_cat"] = entityCat;
  doc["stat_cla"] = "measurement";
  JsonObject root = doc.as<JsonObject>();
  addDeviceBlock(root);

  char discTopic[128];
  snprintf(discTopic, sizeof(discTopic), "homeassistant/sensor/khcontrollerv3/%s/config", id);
  publishDiscoveryPayload(discTopic, doc);
}

static void publishNumberDiscovery(const char* id, const char* name,
                                    const char* statTopic, const char* cmdTopic,
                                    float minVal, float maxVal, float step,
                                    const char* unit) {
  StaticJsonDocument<768> doc;
  doc["name"] = name;
  doc["stat_t"] = statTopic;
  doc["cmd_t"] = cmdTopic;
  doc["uniq_id"] = id;
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
  snprintf(discTopic, sizeof(discTopic), "homeassistant/number/khcontrollerv3/%s/config", id);
  publishDiscoveryPayload(discTopic, doc);
}

static void publishButtonDiscovery(const char* id, const char* name,
                                    const char* cmdTopic, const char* payload,
                                    const char* entityCat) {
  StaticJsonDocument<512> doc;
  doc["name"] = name;
  doc["cmd_t"] = cmdTopic;
  doc["pl_prs"] = payload;
  doc["uniq_id"] = id;
  doc["avty_t"] = availability_topic;
  if (entityCat) doc["ent_cat"] = entityCat;
  JsonObject root = doc.as<JsonObject>();
  addDeviceBlock(root);

  char discTopic[128];
  snprintf(discTopic, sizeof(discTopic), "homeassistant/button/khcontrollerv3/%s/config", id);
  publishDiscoveryPayload(discTopic, doc);
}

static void publishSelectDiscovery(const char* id, const char* name,
                                    const char* statTopic, const char* cmdTopic,
                                    const char** options, uint8_t optCount) {
  StaticJsonDocument<768> doc;
  doc["name"] = name;
  doc["stat_t"] = statTopic;
  doc["cmd_t"] = cmdTopic;
  doc["uniq_id"] = id;
  doc["avty_t"] = availability_topic;
  doc["ent_cat"] = "config";
  JsonArray opts = doc.createNestedArray("options");
  for (uint8_t i = 0; i < optCount; i++) opts.add(options[i]);
  JsonObject root = doc.as<JsonObject>();
  addDeviceBlock(root);

  char discTopic[128];
  snprintf(discTopic, sizeof(discTopic), "homeassistant/select/khcontrollerv3/%s/config", id);
  publishDiscoveryPayload(discTopic, doc);
}

void publishAllDiscovery() {
  initTopics();

  char cmdTopic[50];
  snprintf(cmdTopic, sizeof(cmdTopic), "%s/cmd", DEVICE_NAME);
  char khValueTopic[50];
  snprintf(khValueTopic, sizeof(khValueTopic), "%s/kh_value", DEVICE_NAME);
  char startPhTopic[50];
  snprintf(startPhTopic, sizeof(startPhTopic), "%s/startPH", DEVICE_NAME);
  char mesPhTopic[50];
  snprintf(mesPhTopic, sizeof(mesPhTopic), "%s/mes_pH", DEVICE_NAME);
  char errorTopic[50];
  snprintf(errorTopic, sizeof(errorTopic), "%s/error", DEVICE_NAME);
  char messageTopic[50];
  snprintf(messageTopic, sizeof(messageTopic), "%s/message", DEVICE_NAME);

  // Sensors
  publishSensorDiscovery("khv3_kh", "KH", khValueTopic, "dKH", nullptr, nullptr, nullptr);
  publishSensorDiscovery("khv3_ph", "pH", startPhTopic, "pH", nullptr, nullptr, nullptr);
  publishSensorDiscovery("khv3_mes_ph", "Measurement pH", mesPhTopic, "pH", nullptr, nullptr, "diagnostic");
  publishSensorDiscovery("khv3_rssi", "WiFi Signal", topicDiagnostics, "dBm", "signal_strength",
                          "{{ value_json.rssi }}", "diagnostic");
  publishSensorDiscovery("khv3_uptime", "Uptime", topicDiagnostics, "s", "duration",
                          "{{ value_json.uptime }}", "diagnostic");

  // Probe health sensors
  publishSensorDiscovery("khv3_acid_eff", "Acid Slope Efficiency", topicDiagnostics, "%", nullptr,
                          "{{ value_json.acid_efficiency }}", "diagnostic");
  publishSensorDiscovery("khv3_alk_eff", "Alkaline Slope Efficiency", topicDiagnostics, "%", nullptr,
                          "{{ value_json.alk_efficiency }}", "diagnostic");
  publishSensorDiscovery("khv3_probe_asym", "Probe Asymmetry", topicDiagnostics, "%", nullptr,
                          "{{ value_json.probe_asymmetry }}", "diagnostic");
  publishSensorDiscovery("khv3_probe_resp", "Probe Response Time", topicDiagnostics, "ms", nullptr,
                          "{{ value_json.probe_response }}", "diagnostic");
  publishSensorDiscovery("khv3_cal_age", "Calibration Age", topicDiagnostics, "d", nullptr,
                          "{{ value_json.cal_age }}", "diagnostic");

  // Probe health text sensor (no unit, no state_class)
  {
    StaticJsonDocument<512> doc;
    doc["name"] = "Probe Health";
    doc["stat_t"] = topicDiagnostics;
    doc["uniq_id"] = "khv3_probe_health";
    doc["avty_t"] = availability_topic;
    doc["val_tpl"] = "{{ value_json.probe_health }}";
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    publishDiscoveryPayload(
      "homeassistant/sensor/khcontrollerv3/khv3_probe_health/config", doc);
  }

  // Text sensors (no unit, no state_class)
  {
    StaticJsonDocument<512> doc;
    doc["name"] = "Last Error";
    doc["stat_t"] = errorTopic;
    doc["uniq_id"] = "khv3_error";
    doc["avty_t"] = availability_topic;
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    char dt[128];
    snprintf(dt, sizeof(dt), "homeassistant/sensor/khcontrollerv3/khv3_error/config");
    publishDiscoveryPayload(dt, doc);
  }
  {
    StaticJsonDocument<512> doc;
    doc["name"] = "Last Message";
    doc["stat_t"] = messageTopic;
    doc["uniq_id"] = "khv3_message";
    doc["avty_t"] = availability_topic;
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    char dt[128];
    snprintf(dt, sizeof(dt), "homeassistant/sensor/khcontrollerv3/khv3_message/config");
    publishDiscoveryPayload(dt, doc);
  }

  // Binary sensor - connectivity
  {
    StaticJsonDocument<512> doc;
    doc["name"] = "Connectivity";
    doc["stat_t"] = availability_topic;
    doc["uniq_id"] = "khv3_connectivity";
    doc["dev_cla"] = "connectivity";
    doc["pl_on"] = "online";
    doc["pl_off"] = "offline";
    doc["ent_cat"] = "diagnostic";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    publishDiscoveryPayload("homeassistant/binary_sensor/khcontrollerv3/connectivity/config", doc);
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
  publishNumberDiscovery("khv3_cal_drops", "Calibration Drops",
                          topicCfgCalDrops, topicCfgCalDropsSet, 1000, 20000, 100, nullptr);
  publishNumberDiscovery("khv3_fast_ph", "Fast Titration pH",
                          topicCfgFastPH, topicCfgFastPHSet, 4.5, 7.0, 0.1, "pH");

  // Schedule text inputs (HH:MM format)
  for (int i = 0; i < 8; i++) {
    char id[20], name[30];
    snprintf(id, sizeof(id), "khv3_sched_%d", i);
    snprintf(name, sizeof(name), "Schedule %d", i + 1);

    StaticJsonDocument<768> doc;
    doc["name"] = name;
    doc["stat_t"] = topicCfgSched[i];
    doc["cmd_t"] = topicCfgSchedSet[i];
    doc["uniq_id"] = id;
    doc["avty_t"] = availability_topic;
    doc["pattern"] = "^([01]?[0-9]|2[0-3]):[0-5][0-9]$";
    doc["ent_cat"] = "config";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);

    char discTopic[128];
    snprintf(discTopic, sizeof(discTopic), "homeassistant/text/khcontrollerv3/%s/config", id);
    publishDiscoveryPayload(discTopic, doc);
  }

  // Schedule mode select
  {
    const char* modeOpts[] = {"custom", "interval"};
    publishSelectDiscovery("khv3_sched_mode", "Schedule Mode",
                            topicCfgSchedMode, topicCfgSchedModeSet,
                            modeOpts, 2);
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
    StaticJsonDocument<768> doc;
    doc["name"] = "Anchor Time";
    doc["stat_t"] = topicCfgAnchorTime;
    doc["cmd_t"] = topicCfgAnchorTimeSet;
    doc["uniq_id"] = "khv3_anchor_time";
    doc["avty_t"] = availability_topic;
    doc["pattern"] = "^([01]?[0-9]|2[0-3]):[0-5][0-9]$";
    doc["ent_cat"] = "config";
    JsonObject root = doc.as<JsonObject>();
    addDeviceBlock(root);
    publishDiscoveryPayload(
      "homeassistant/text/khcontrollerv3/khv3_anchor_time/config", doc);
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

  Serial.println("HA Discovery published");

  // Subscribe to config set topics
  mqttManager.subscribe(topicCfgTitVolSet);
  mqttManager.subscribe(topicCfgSamVolSet);
  mqttManager.subscribe(topicCfgCorrFSet);
  mqttManager.subscribe(topicCfgHclMolSet);
  mqttManager.subscribe(topicCfgHclVolSet);
  mqttManager.subscribe(topicCfgCalDropsSet);
  mqttManager.subscribe(topicCfgFastPHSet);
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
  mqttManager.publish(topicCfgCalDrops, String(configStore.getCalDrops()).c_str(), true);
  mqttManager.publish(topicCfgFastPH, String(configStore.getFastTitrationPH(), 1).c_str(), true);

  for (int i = 0; i < 8; i++) {
    char timeBuf[6];
    minsToTimeStr(configStore.getScheduleTime(i), timeBuf, sizeof(timeBuf));
    mqttManager.publish(topicCfgSched[i], timeBuf, true);
  }

  // Schedule mode
  mqttManager.publish(topicCfgSchedMode,
                      (configStore.getScheduleMode() == 1) ? "interval" : "custom", true);
  mqttManager.publish(topicCfgIntervalHours,
                      String(configStore.getIntervalHours()).c_str(), true);
  {
    char timeBuf[6];
    minsToTimeStr(configStore.getAnchorTime(), timeBuf, sizeof(timeBuf));
    mqttManager.publish(topicCfgAnchorTime, timeBuf, true);
  }
}

void publishDiagnostics() {
  StaticJsonDocument<512> doc;
  doc["rssi"] = wifiManager.getRSSI();
  doc["uptime"] = millis() / 1000;
  doc["heap"] = ESP.getFreeHeap();

  // Probe health metrics — Nernst efficiency per segment
  float acidEff = getAcidEfficiency();
  float alkEff = getAlkalineEfficiency();
  float asym = getProbeAsymmetry();
  doc["acid_efficiency"] = isnan(acidEff) ? 0 : (int)(acidEff + 0.5f);
  doc["alk_efficiency"] = isnan(alkEff) ? 0 : (int)(alkEff + 0.5f);
  doc["probe_asymmetry"] = isnan(asym) ? 0 : asym;
  doc["probe_response"] = getLastStabilizationMs();
  doc["probe_health"] = getProbeHealth();

  // Calibration age in days
  uint32_t calTs = configStore.getCalTimestamp();
  time_t now = time(nullptr);
  if (calTs > 0 && now > 1000000000) {
    doc["cal_age"] = (int)((now - calTs) / 86400);
  } else {
    doc["cal_age"] = -1;  // No calibration timestamp recorded
  }

  char buf[512];
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
    configStore.setCalDrops((int)val);
    mqttManager.publish(topicCfgCalDrops, String((int)val).c_str(), true);
  } else if (strcmp(topic, topicCfgFastPHSet) == 0) {
    configStore.setFastTitrationPH(val);
    mqttManager.publish(topicCfgFastPH, String(val, 1).c_str(), true);
  } else if (strcmp(topic, topicCfgSchedModeSet) == 0) {
    uint8_t mode = (strcmp(payload, "interval") == 0) ? 1 : 0;
    configStore.setScheduleMode(mode);
    scheduler.resetDailyFlags();
    mqttManager.publish(topicCfgSchedMode, (mode == 1) ? "interval" : "custom", true);
  } else if (strcmp(topic, topicCfgIntervalHoursSet) == 0) {
    uint8_t h = (uint8_t)atoi(payload);
    configStore.setIntervalHours(h);
    scheduler.resetDailyFlags();
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
