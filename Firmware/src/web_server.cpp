#include "web_server.h"
#include "ha_discovery.h"
#include "measurement.h"
#include "motors.h"
#include "config_store.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "scheduler.h"
#include "stirrer.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <config.h>
#include <pins.h>
#include <time.h>
#include <math.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// In-memory buffer for live titration pH data (survives page reload)
// When full, downsamples by averaging pairs -> halves count, then continues
static const uint16_t MES_BUF_MAX = 2000;
struct MesPoint { float ml; float ph; };
static MesPoint mesBuffer[MES_BUF_MAX];
static uint16_t mesCount = 0;

// In-memory ring buffer for event log entries (survives page reload)
static const uint8_t LOG_BUF_MAX = 10;
struct LogEntry {
  char type;       // 'm' = msg, 'e' = error
  char text[96];
  uint32_t ts;     // epoch seconds
};
static LogEntry logBuffer[LOG_BUF_MAX];
static uint8_t logCount = 0;   // total entries stored (up to LOG_BUF_MAX)
static uint8_t logHead = 0;    // next write position

// In-memory cache for last Gran measurement (replayed to new clients on connect)
static const uint8_t GRAN_BUF_MAX = 50;
struct GranBufPoint { float ml; float f; };
static GranBufPoint granBuffer[GRAN_BUF_MAX];
static uint8_t granBufCount = 0;
static float granBufR2 = 0;
static float granBufEqML = 0;
static bool granBufUsed = false;

// Forward declarations for command/config handlers from main
extern int units;
extern float startPH;
extern void measureKH();
extern void calibrateTitrationPump();
extern void subtractHCl(int unitsUsed);
extern void publishMessage(const char* message);
extern void publishError(const char* errorMessage);

// Internal forward declarations
static void handleWebSocketMessage(void* arg, uint8_t* data, size_t len);
static void sendMesData(AsyncWebSocketClient* client);
static void sendLogData(AsyncWebSocketClient* client);
static void sendGranData(AsyncWebSocketClient* client);

static void addLogEntry(char type, const char* text) {
  LogEntry& e = logBuffer[logHead];
  e.type = type;
  strncpy(e.text, text, sizeof(e.text) - 1);
  e.text[sizeof(e.text) - 1] = '\0';
  e.ts = (uint32_t)time(nullptr);
  logHead = (logHead + 1) % LOG_BUF_MAX;
  if (logCount < LOG_BUF_MAX) logCount++;
}

static void sendLogData(AsyncWebSocketClient* client) {
  if (logCount == 0) return;

  // Walk ring buffer oldest-first
  uint8_t start = (logCount < LOG_BUF_MAX) ? 0 : logHead;
  DynamicJsonDocument doc(4096);
  doc["type"] = "logData";
  JsonArray arr = doc.createNestedArray("entries");
  for (uint8_t i = 0; i < logCount; i++) {
    uint8_t idx = (start + i) % LOG_BUF_MAX;
    JsonObject entry = arr.createNestedObject();
    entry["t"] = (logBuffer[idx].type == 'e') ? "error" : "msg";
    entry["text"] = logBuffer[idx].text;
    entry["ts"] = logBuffer[idx].ts;
  }

  static char buf[4096];
  size_t written = serializeJson(doc, buf, sizeof(buf));
  if (written > 0) {
    client->text(buf);
  }
}

static void sendMesData(AsyncWebSocketClient* client) {
  if (mesCount == 0) return;

  // Send in chunks of 500 points to avoid huge stack allocations
  static const uint16_t CHUNK = 500;
  uint16_t totalChunks = (mesCount + CHUNK - 1) / CHUNK;

  for (uint16_t c = 0; c < totalChunks; c++) {
    uint16_t start = c * CHUNK;
    uint16_t end = min((uint16_t)(start + CHUNK), mesCount);

    DynamicJsonDocument doc(8192);
    doc["type"] = "mesData";
    doc["chunk"] = c;
    doc["total"] = totalChunks;
    JsonArray dataArr = doc.createNestedArray("data");
    for (uint16_t i = start; i < end; i++) {
      JsonArray point = dataArr.createNestedArray();
      point.add(mesBuffer[i].ml);
      point.add(mesBuffer[i].ph);
    }

    static char buf[8192];
    size_t written = serializeJson(doc, buf, sizeof(buf));
    if (written > 0) {
      client->text(buf);
    }
  }
}

static void sendGranData(AsyncWebSocketClient* client) {
  if (granBufCount == 0) return;

  DynamicJsonDocument doc(2048);
  doc["type"] = "granData";
  doc["r2"] = granBufR2;
  doc["eqML"] = granBufEqML;
  doc["used"] = granBufUsed;

  JsonArray pts = doc.createNestedArray("points");
  for (uint8_t i = 0; i < granBufCount; i++) {
    JsonArray pt = pts.createNestedArray();
    pt.add(granBuffer[i].ml);
    pt.add(granBuffer[i].f);
  }

  static char buf[2048];
  size_t written = serializeJson(doc, buf, sizeof(buf));
  if (written > 0) client->text(buf);
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      broadcastState();
      sendMesData(client);
      sendLogData(client);
      sendGranData(client);
      break;
    case WS_EVT_DISCONNECT:
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

static void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, (char*)data, len);
    if (err) return;

    const char* type = doc["type"];
    if (!type) return;

    if (strcmp(type, "cmd") == 0) {
      const char* cmd = doc["cmd"];
      if (cmd) executeCommand(cmd);
    } else if (strcmp(type, "config") == 0) {
      const char* key = doc["key"];
      if (!key) return;
      if (!doc.containsKey("value")) return;
      float value = doc["value"];

      if (strcmp(key, "titration_vol") == 0) configStore.setTitrationVolume(value);
      else if (strcmp(key, "sample_vol") == 0) configStore.setSampleVolume(value);
      else if (strcmp(key, "correction_factor") == 0) configStore.setCorrectionFactor(value);
      else if (strcmp(key, "hcl_molarity") == 0) configStore.setHClMolarity(value);
      else if (strcmp(key, "hcl_volume") == 0) configStore.setHClVolume(value);
      else if (strcmp(key, "cal_drops") == 0) configStore.setCalUnits((int)value);
      else if (strcmp(key, "fast_ph") == 0) configStore.setFastTitrationPH(value);

      broadcastState(); // Confirm the update
    } else if (strcmp(type, "schedule") == 0) {
      // Schedule mode (0=custom, 1=interval)
      if (doc.containsKey("mode")) {
        configStore.setScheduleMode(doc["mode"].as<uint8_t>());
        scheduler.resetDailyFlags();
      }
      if (doc.containsKey("intervalHours")) {
        configStore.setIntervalHours(doc["intervalHours"].as<uint8_t>());
        scheduler.resetDailyFlags();
      }
      if (doc.containsKey("anchorTime")) {
        configStore.setAnchorTime(doc["anchorTime"].as<uint16_t>());
        scheduler.resetDailyFlags();
      }
      // Custom mode slots
      JsonArray slots = doc["slots"];
      if (slots) {
        uint8_t count = min((size_t)slots.size(), (size_t)8);
        configStore.setScheduleCount(count);
        for (uint8_t i = 0; i < count; i++) {
          configStore.setScheduleTime(i, (uint16_t)slots[i].as<int>());
        }
      }
      publishAllConfigStates();
      broadcastState();
    } else if (strcmp(type, "getHistory") == 0) {
      const char* sensor = doc["sensor"];
      if (!sensor) return;

      char filename[32];
      snprintf(filename, sizeof(filename), "/history/%s.csv", sensor);

      if (!LittleFS.exists(filename)) return;

      File f = LittleFS.open(filename, "r");
      if (!f) return;

      uint32_t cutoff = (uint32_t)time(nullptr) - (7 * 86400);
      bool isGran = (strcmp(sensor, "gran") == 0);

      if (isGran) {
        // Gran CSV: timestamp,r2,eqML,endpointPH,method
        DynamicJsonDocument resp(4096);
        resp["type"] = "history";
        resp["sensor"] = "gran";
        JsonArray dataArr = resp.createNestedArray("data");

        while (f.available() && dataArr.size() < 150) {
          String line = f.readStringUntil('\n');
          if (line.length() == 0) continue;
          int c1 = line.indexOf(',');
          if (c1 < 0) continue;
          uint32_t ts = line.substring(0, c1).toInt();
          if (ts < cutoff) continue;
          int c2 = line.indexOf(',', c1 + 1);
          int c3 = line.indexOf(',', c2 + 1);
          int c4 = line.indexOf(',', c3 + 1);
          if (c2 < 0 || c3 < 0 || c4 < 0) continue;
          float r2  = line.substring(c1 + 1, c2).toFloat();
          float eq  = line.substring(c2 + 1, c3).toFloat();
          float eph = line.substring(c3 + 1, c4).toFloat();
          int   mth = line.substring(c4 + 1).toInt();
          JsonArray pt = dataArr.createNestedArray();
          pt.add(ts);
          pt.add(r2);
          pt.add(eq);
          pt.add(eph);
          pt.add(mth);
        }
        f.close();

        static char buf[4096];
        size_t written = serializeJson(resp, buf, sizeof(buf));
        if (written > 0) ws.textAll(buf);
      } else {
        // KH/pH CSV: timestamp,value
        StaticJsonDocument<512> resp;
        resp["type"] = "history";
        resp["sensor"] = sensor;
        JsonArray dataArr = resp.createNestedArray("data");

        while (f.available() && dataArr.size() < 150) {
          String line = f.readStringUntil('\n');
          if (line.length() == 0) continue;
          int comma = line.indexOf(',');
          if (comma < 0) continue;
          uint32_t ts = line.substring(0, comma).toInt();
          if (ts < cutoff) continue;
          float val = line.substring(comma + 1).toFloat();
          JsonArray point = dataArr.createNestedArray();
          point.add(ts);
          point.add(val);
        }
        f.close();

        static char buf[6144];
        size_t written = serializeJson(resp, buf, sizeof(buf));
        if (written > 0) ws.textAll(buf);
      }
    }
  }
}

static void calibratePH(int bufferPH) {
  startStirrer();
  delay(STIRRER_WARMUP_MS);
  float v = measureVoltage(100);
  stopStirrer();
  switch (bufferPH) {
    case 4:  voltage_4PH = v;  configStore.setVoltage4PH(v);  break;
    case 7:  voltage_7PH = v;  configStore.setVoltage7PH(v);  break;
    case 10: voltage_10PH = v; configStore.setVoltage10PH(v); break;
  }
  updateCalibrationFit();
  configStore.setCalTimestamp((uint32_t)time(nullptr));
  if (isCalibrationValid()) configStore.addSlopeEntry((uint32_t)time(nullptr), getAcidSlope());
  char msg[24];
  snprintf(msg, sizeof(msg), "Calibrated pH %d", bufferPH);
  publishMessage(msg);
  char reason[96];
  const char* health = getProbeHealthDetail(reason, sizeof(reason));
  if (strcmp(health, "Good") != 0) {
    char hBuf[128];
    snprintf(hBuf, sizeof(hBuf), "Warning: Probe %s — %s", health, reason);
    publishError(hBuf);
  }
}

extern void queueCommand(char cmd);

void executeCommand(const char* cmd) {
  if (strcmp(cmd, "k") == 0 || strcmp(cmd, "t") == 0) {
    // Defer long-running ops to loopTask — running them on the AsyncTCP
    // task blocks WebSocket/WiFi processing and causes watchdog resets
    queueCommand(cmd[0]);
    return;
  } else if (strcmp(cmd, "p") == 0) {
    measurePH(100);
    if (isnan(pH)) {
      publishError("Error: pH probe not working");
    } else {
      char phBuf[16];
      snprintf(phBuf, sizeof(phBuf), "pH: %.2f", pH);
      publishMessage(phBuf);
    }
  } else if (strcmp(cmd, "f") == 0) {
    publishMessage("Filling");
    titrate(FILL_VOLUME, PREFILL_RPM);
    subtractHCl(FILL_VOLUME);
    digitalWrite(EN_PIN2, HIGH);
    publishMessage("Fill done");
  } else if (strcmp(cmd, "s") == 0) {
    publishMessage("Washing sample");
    washSample(1.2, 1.0);
    publishMessage("Wash done");
  } else if (strcmp(cmd, "m") == 0) {
    startStirrer();
    publishMessage("Stirrer started");
  } else if (strcmp(cmd, "e") == 0) {
    stopStirrer();
    publishMessage("Stirrer stopped");
  } else if (strcmp(cmd, "r") == 0) {
    publishMessage("Removing sample");
    removeSample(SAMPLE_PUMP_VOLUME);
    publishMessage("Sample removed");
  } else if (strcmp(cmd, "o") == 0) {
    publishMessage("Restarting...");
    delay(100);
    ESP.restart();
  } else if (strcmp(cmd, "v") == 0) {
    float v = measureVoltage(100);
    char vBuf[32];
    snprintf(vBuf, sizeof(vBuf), "Voltage: %.1f mV", v);
    publishMessage(vBuf);
  } else if (strcmp(cmd, "4") == 0) {
    calibratePH(4);
  } else if (strcmp(cmd, "7") == 0) {
    calibratePH(7);
  } else if (strcmp(cmd, "10") == 0) {
    calibratePH(10);
  }
  broadcastState();
}

void broadcastTitrationStart() {
  mesCount = 0;
  granBufCount = 0;
  ws.textAll("{\"type\":\"mesStart\"}");
}

void broadcastState() {
  if (ws.count() == 0) return;

  StaticJsonDocument<1536> doc;
  doc["type"] = "state";
  doc["ph"] = pH;
  doc["startPh"] = startPH;
  doc["units"] = units;
  doc["kh"] = configStore.getLastKH();
  doc["lastStartPh"] = configStore.getLastStartPH();
  doc["hclVol"] = configStore.getHClVolume();
  doc["rssi"] = wifiManager.getRSSI();
  doc["uptime"] = millis() / 1000;
  doc["mqttOk"] = mqttManager.isConnected();
  doc["wifiOk"] = wifiManager.isConnected();
  doc["ntpOk"] = scheduler.isTimeSynced();
  doc["nextMeas"] = scheduler.getNextMeasurementTime();

  // Probe health
  JsonObject probe = doc.createNestedObject("probe");
  probe["acidEff"] = getAcidEfficiency();
  probe["alkEff"] = getAlkalineEfficiency();
  probe["asymmetry"] = getProbeAsymmetry();
  probe["response"] = getLastStabilizationMs();
  probe["health"] = getProbeHealth();
  uint32_t calTs = configStore.getCalTimestamp();
  if (calTs > 0) {
    time_t now = time(nullptr);
    probe["calAge"] = (now > (time_t)calTs) ? (int)((now - calTs) / 86400) : 0;
  } else {
    probe["calAge"] = -1;  // never calibrated
  }

  // Efficiency history (acid slope over time, converted to Nernst %)
  ConfigStore::SlopeEntry slopeHist[ConfigStore::MAX_SLOPE_HISTORY];
  int slopeCount = configStore.getSlopeHistory(slopeHist, ConfigStore::MAX_SLOPE_HISTORY);
  if (slopeCount > 0) {
    float nernst = NERNST_FACTOR * (273.15f + MEASUREMENT_TEMP_C);
    JsonArray eh = probe.createNestedArray("effHist");
    for (int i = 0; i < slopeCount; i++) {
      JsonArray entry = eh.createNestedArray();
      entry.add(slopeHist[i].timestamp);
      float eff = fabsf(slopeHist[i].slope) / PH_AMP_GAIN / nernst * 100.0f;
      entry.add((int)(eff * 10.0f + 0.5f) / 10.0f);  // 1 decimal
    }
  }

  // Config values
  JsonObject cfg = doc.createNestedObject("config");
  cfg["titration_vol"] = configStore.getTitrationVolume();
  cfg["sample_vol"] = configStore.getSampleVolume();
  cfg["correction_factor"] = configStore.getCorrectionFactor();
  cfg["hcl_molarity"] = configStore.getHClMolarity();
  cfg["hcl_volume"] = configStore.getHClVolume();
  cfg["cal_drops"] = configStore.getCalUnits();
  cfg["fast_ph"] = configStore.getFastTitrationPH();

  // Schedule
  doc["schedMode"] = configStore.getScheduleMode();
  doc["intervalHours"] = configStore.getIntervalHours();
  doc["anchorTime"] = configStore.getAnchorTime();

  JsonArray sched = doc.createNestedArray("schedule");
  uint8_t count = configStore.getScheduleCount();
  for (uint8_t i = 0; i < count; i++) {
    sched.add(configStore.getScheduleTime(i));
  }

  char buf[1536];
  serializeJson(doc, buf, sizeof(buf));
  ws.textAll(buf);
}

void broadcastTitrationPH(float phVal, int unitsVal) {
  // Convert units to mL on the firmware side (avoids JS sync issues)
  float calU = (float)configStore.getCalUnits();
  float titV = configStore.getTitrationVolume();
  float ml = (calU > 0) ? ((float)unitsVal / calU * titV) : 0;

  // Downsample when buffer is full: average pairs to halve the count
  if (mesCount >= MES_BUF_MAX) {
    uint16_t newCount = mesCount / 2;
    for (uint16_t i = 0; i < newCount; i++) {
      mesBuffer[i].ml = mesBuffer[i * 2 + 1].ml; // keep later volume
      mesBuffer[i].ph = (mesBuffer[i * 2].ph + mesBuffer[i * 2 + 1].ph) / 2.0f;
    }
    mesCount = newCount;
  }

  mesBuffer[mesCount].ml = ml;
  mesBuffer[mesCount].ph = phVal;
  mesCount++;

  if (ws.count() == 0) return;

  StaticJsonDocument<64> doc;
  doc["type"] = "mesPh";
  doc["ph"] = phVal;
  doc["ml"] = ml;

  char buf[64];
  serializeJson(doc, buf, sizeof(buf));
  ws.textAll(buf);
}

void broadcastMessage(const char* msg) {
  addLogEntry('m', msg);
  if (ws.count() == 0) return;
  StaticJsonDocument<128> doc;
  doc["type"] = "msg";
  doc["text"] = msg;
  char buf[128];
  serializeJson(doc, buf, sizeof(buf));
  ws.textAll(buf);
}

void broadcastError(const char* msg) {
  if (!msg || msg[0] == '\0') return;  // Skip empty errors
  addLogEntry('e', msg);
  if (ws.count() == 0) return;
  StaticJsonDocument<128> doc;
  doc["type"] = "error";
  doc["text"] = msg;
  char buf[128];
  serializeJson(doc, buf, sizeof(buf));
  ws.textAll(buf);
}

void broadcastProgress(int percent) {
  if (ws.count() == 0) return;
  char buf[32];
  snprintf(buf, sizeof(buf), "{\"type\":\"progress\",\"pct\":%d}", percent);
  ws.textAll(buf);
}

void broadcastGranData(float r2, float eqML, bool usedGran,
                       float* pointsML, float* pointsF, int nPts) {
  // Cache for replay to new WebSocket clients
  granBufR2 = r2;
  granBufEqML = eqML;
  granBufUsed = usedGran;
  granBufCount = (uint8_t)min(nPts, (int)GRAN_BUF_MAX);
  for (uint8_t i = 0; i < granBufCount; i++) {
    granBuffer[i] = { pointsML[i], pointsF[i] };
  }

  if (ws.count() == 0 || nPts == 0) return;

  DynamicJsonDocument doc(2048);
  doc["type"] = "granData";
  doc["r2"] = r2;
  doc["eqML"] = eqML;
  doc["used"] = usedGran;

  JsonArray pts = doc.createNestedArray("points");
  for (int i = 0; i < nPts; i++) {
    JsonArray pt = pts.createNestedArray();
    pt.add(pointsML[i]);
    pt.add(pointsF[i]);
  }

  char buf[2048];
  size_t written = serializeJson(doc, buf, sizeof(buf));
  if (written > 0) ws.textAll(buf);
}

void appendHistory(const char* sensor, float value) {
  char filename[32];
  snprintf(filename, sizeof(filename), "/history/%s.csv", sensor);

  // Ensure directory exists
  if (!LittleFS.exists("/history")) {
    LittleFS.mkdir("/history");
  }

  // Remove entries older than 7 days
  if (LittleFS.exists(filename)) {
    uint32_t cutoff = (uint32_t)time(nullptr) - (7 * 86400);
    File f = LittleFS.open(filename, "r");
    if (f) {
      String kept;
      while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.length() == 0) continue;
        int comma = line.indexOf(',');
        if (comma < 0) continue;
        uint32_t ts = line.substring(0, comma).toInt();
        if (ts >= cutoff) {
          kept += line + "\n";
        }
      }
      f.close();
      File fw = LittleFS.open(filename, "w");
      if (fw) {
        fw.print(kept);
        fw.close();
      }
    }
  }

  time_t now = time(nullptr);
  if (now < 1000000000) return;  // NTP not yet synced — skip bogus timestamp

  File f = LittleFS.open(filename, "a");
  if (f) {
    f.printf("%u,%.2f\n", (uint32_t)now, value);
    f.close();
  }
}

void appendGranHistory(float r2, float eqML, float endpointPH, bool usedGran) {
  const char* filename = "/history/gran.csv";

  if (!LittleFS.exists("/history")) {
    LittleFS.mkdir("/history");
  }

  // Remove entries older than 7 days
  if (LittleFS.exists(filename)) {
    uint32_t cutoff = (uint32_t)time(nullptr) - (7 * 86400);
    File f = LittleFS.open(filename, "r");
    if (f) {
      String kept;
      while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.length() == 0) continue;
        int comma = line.indexOf(',');
        if (comma < 0) continue;
        uint32_t ts = line.substring(0, comma).toInt();
        if (ts >= cutoff) kept += line + "\n";
      }
      f.close();
      File fw = LittleFS.open(filename, "w");
      if (fw) { fw.print(kept); fw.close(); }
    }
  }

  time_t now = time(nullptr);
  if (now < 1000000000) return;  // NTP not synced

  File f = LittleFS.open(filename, "a");
  if (f) {
    f.printf("%u,%.4f,%.3f,%.2f,%d\n", (uint32_t)now, r2, eqML, endpointPH, usedGran ? 1 : 0);
    f.close();
  }
}

void setupWebServer() {
  LittleFS.begin(true); // Format if mount fails

  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/www/").setDefaultFile("index.html").setCacheControl("no-cache");

  // WebSocket handler
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Fallback API for state
  server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest* request) {
    StaticJsonDocument<256> doc;
    doc["ph"] = pH;
    doc["hclVol"] = configStore.getHClVolume();
    doc["rssi"] = wifiManager.getRSSI();
    doc["uptime"] = millis() / 1000;
    char buf[256];
    serializeJson(doc, buf, sizeof(buf));
    request->send(200, "application/json", buf);
  });

  server.begin();
  Serial.println("Web server started on port 80");
}
