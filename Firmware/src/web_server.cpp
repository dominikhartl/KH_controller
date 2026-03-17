#include "web_server.h"
#include "ha_discovery.h"
#include "measurement.h"
#include "motors.h"
#include "config_store.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "scheduler.h"
#include "stirrer.h"
#include "temperature.h"
#include "tmc_driver.h"
#include "hw_diagnostics.h"
#include "setup_page.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Update.h>
#include <config.h>
#include <pins.h>
#include <time.h>
#include <math.h>

extern char deviceName[];
extern char MQmespH[];
extern void requestAbort();
extern volatile uint32_t heapMin;

float lastConfidence = NAN;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// In-memory buffer for live titration pH data (survives page reload)
// When full, downsamples by averaging pairs -> halves count, then continues
static const uint16_t MES_BUF_MAX = 1500;
struct MesPoint { float ml; float ph; float mV; };
static MesPoint mesBuffer[MES_BUF_MAX];
static uint16_t mesCount = 0;

// In-memory ring buffer for event log entries (survives page reload)
static const uint8_t LOG_BUF_MAX = 40;
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
static float granBufWinLowML = 0, granBufWinHighML = 0;
static float granBufSlopeML = 0, granBufIntercept = 0;
static GranWindowResult granBufWindows[MAX_GRAN_WINDOWS];
static uint8_t granBufNWindows = 0;

// Cached KH slope — updated after each measurement, avoids filesystem reads in broadcastState()
static float cachedKHSlope = NAN;
static float cachedKHIntercept = NAN;
static uint32_t cachedKHSlopeDay0 = 0;
static float cachedSlopeCI = NAN;
static int cachedSlopeNDays = 0;

// Last measurement result (for diagnostics download)
static KHResult lastKHResult = {};
static bool hasLastKHResult = false;
static uint32_t lastMeasTimestamp = 0;

void storeLastKHResult(const KHResult& r) {
  lastKHResult = r;
  hasLastKHResult = true;
  lastMeasTimestamp = (uint32_t)time(nullptr);
}

// Analysis data points (pH < 5.0 subset used for Gran/interpolation)
static TitrationPoint analysisBuf[MAX_TITRATION_POINTS];
static int analysisCount = 0;

void storeAnalysisPoints(const TitrationPoint* points, int count) {
  analysisCount = (count > MAX_TITRATION_POINTS) ? MAX_TITRATION_POINTS : count;
  memcpy(analysisBuf, points, analysisCount * sizeof(TitrationPoint));
}

// Forward declarations for command/config handlers from main
extern std::atomic<int> units;
extern float startPH;
extern void calibrateTitrationPump();
extern void subtractHCl(int unitsUsed);
extern void publishMessage(const char* message);
extern void publishError(const char* errorMessage);

// Internal forward declarations
static void handleWebSocketMessage(AsyncWebSocketClient* client, void* arg, uint8_t* data, size_t len);
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
  JsonDocument doc;
  doc["type"] = "logData";
  JsonArray arr = doc["entries"].to<JsonArray>();
  for (uint8_t i = 0; i < logCount; i++) {
    uint8_t idx = (start + i) % LOG_BUF_MAX;
    JsonObject entry = arr.add<JsonObject>();
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

  // Send in chunks of 400 points to avoid huge stack allocations
  static const uint16_t CHUNK = 400;
  uint16_t totalChunks = (mesCount + CHUNK - 1) / CHUNK;

  for (uint16_t c = 0; c < totalChunks; c++) {
    uint16_t start = c * CHUNK;
    uint16_t end = min((uint16_t)(start + CHUNK), mesCount);

    JsonDocument doc;
    doc["type"] = "mesData";
    doc["chunk"] = c;
    doc["total"] = totalChunks;
    JsonArray dataArr = doc["data"].to<JsonArray>();
    for (uint16_t i = start; i < end; i++) {
      JsonArray point = dataArr.add<JsonArray>();
      point.add(mesBuffer[i].ml);
      point.add(mesBuffer[i].ph);
      point.add(mesBuffer[i].mV);
    }

    size_t jsonLen = measureJson(doc) + 1;
    char* buf = (char*)malloc(jsonLen);
    if (buf) {
      serializeJson(doc, buf, jsonLen);
      client->text(buf);
      free(buf);
    }
  }
}

static void sendGranData(AsyncWebSocketClient* client) {
  if (granBufCount == 0) return;

  JsonDocument doc;
  doc["type"] = "granData";
  doc["r2"] = granBufR2;
  doc["eqML"] = granBufEqML;
  doc["used"] = granBufUsed;
  if (granBufWinLowML > 0) doc["winLowML"] = granBufWinLowML;
  if (granBufWinHighML > 0) doc["winHighML"] = granBufWinHighML;
  if (granBufSlopeML != 0) doc["slope"] = granBufSlopeML;
  if (granBufIntercept != 0) doc["intercept"] = granBufIntercept;

  JsonArray pts = doc["points"].to<JsonArray>();
  for (uint8_t i = 0; i < granBufCount; i++) {
    JsonArray pt = pts.add<JsonArray>();
    pt.add(granBuffer[i].ml);
    pt.add(granBuffer[i].f);
  }

  if (granBufNWindows > 0) {
    JsonArray wins = doc["windows"].to<JsonArray>();
    for (uint8_t i = 0; i < granBufNWindows; i++) {
      JsonArray w = wins.add<JsonArray>();
      w.add(granBufWindows[i].low);
      w.add(granBufWindows[i].high);
      w.add(granBufWindows[i].r2);
      w.add(granBufWindows[i].valid ? 1 : 0);
      w.add(serialized(String(granBufWindows[i].eqUnits, 2)));
    }
  }

  static char buf[2048];
  size_t written = serializeJson(doc, buf, sizeof(buf));
  if (written > 0) client->text(buf);
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      broadcastState();  // lightweight (cached slope, no filesystem I/O)
      sendMesData(client);
      sendLogData(client);
      sendGranData(client);
      break;
    case WS_EVT_DISCONNECT:
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(client, arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

static void handleWebSocketMessage(AsyncWebSocketClient* client, void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    JsonDocument doc;
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
      if (!doc["value"].is<JsonVariant>()) return;

      // Helper: send config result to the requesting client
      auto sendConfigResult = [&](const char* k, bool ok) {
        char buf[128];
        snprintf(buf, sizeof(buf), "{\"type\":\"configResult\",\"key\":\"%s\",\"ok\":%s}", k, ok ? "true" : "false");
        client->text(buf);
      };

      // Device name is a string config — handle separately
      if (strcmp(key, "device_name") == 0) {
        const char* name = doc["value"].as<const char*>();
        if (name && strlen(name) >= 1 && strlen(name) <= 20) {
          // Validate: alphanumeric and hyphens only
          bool valid = true;
          for (const char* p = name; *p; p++) {
            if (!isalnum(*p) && *p != '-') { valid = false; break; }
          }
          if (valid) {
            configStore.setDeviceName(name);
            sendConfigResult(key, true);
            broadcastMessage("Device name changed. Reboot to apply.");
            broadcastState();
          } else {
            sendConfigResult(key, false);
          }
        } else {
          sendConfigResult(key, false);
        }
        return;
      }
      if (strcmp(key, "timezone") == 0) {
        const char* tz = doc["value"].as<const char*>();
        if (tz && strlen(tz) >= 3 && strlen(tz) <= 45) {
          configStore.setTimezone(tz);
          sendConfigResult(key, true);
          broadcastMessage("Timezone changed. Reboot to apply.");
        } else {
          sendConfigResult(key, false);
        }
        return;
      }

      float value = doc["value"];
      bool saved = true;

      if (strcmp(key, "titration_vol") == 0 && value > 0) configStore.setTitrationVolume(value);
      else if (strcmp(key, "sample_vol") == 0 && value > 0) configStore.setSampleVolume(value);
      else if (strcmp(key, "correction_factor") == 0 && value >= 0.5f && value <= 2.0f) configStore.setCorrectionFactor(value);
      else if (strcmp(key, "hcl_molarity") == 0 && value > 0 && value <= 1) configStore.setHClMolarity(value);
      else if (strcmp(key, "hcl_volume") == 0 && value >= 0) configStore.setHClVolume(value);
      else if (strcmp(key, "cal_drops") == 0 && (int)value > 0) configStore.setCalUnits((int)value);
      else if (strcmp(key, "fast_ph") == 0 && value >= 4.0f && value <= 7.0f) configStore.setFastTitrationPH(value);
      else if (strcmp(key, "endpoint_method") == 0) configStore.setEndpointMethod((uint8_t)value);
      else if (strcmp(key, "min_start_ph") == 0 && value >= 6.0f && value <= 9.0f) configStore.setMinStartPH(value);
      else if (strcmp(key, "stab_timeout") == 0) {
        configStore.setStabilizationTimeout((int)value);
        setStabilizationTimeoutMs(configStore.getStabilizationTimeout());
      }
      else if (strcmp(key, "gran_mix_delay") == 0) {
        configStore.setGranMixDelay((int)value);
      }
      else if (strcmp(key, "num_washes") == 0) { configStore.setNumWashes((int)value); }
      else if (strcmp(key, "drop_ul") == 0) { configStore.setDropVolumeUL(value); }
      else if (strcmp(key, "titration_rpm") == 0) { configStore.setTitrationRPM(value); }
      else if (strcmp(key, "fast_phase_rpm") == 0) { configStore.setFastPhaseRPM(value); }
      else if (strcmp(key, "sample_pump_rpm") == 0 && value >= 20.0f && value <= 250.0f) { configStore.setSamplePumpRPM(value); }
      else if (strcmp(key, "sample_cal_revs") == 0 && (int)value >= 50) {
        configStore.setSampleCalRevolutions((int)value);
      }
      else if (strcmp(key, "sample_cal_vol") == 0 && value > 0.1f) {
        // User entered measured volume from sample pump calibration
        int calRevs = configStore.getSampleCalRevolutions();
        float revsPerML = (float)calRevs / value;
        configStore.setSampleCalRevsPerML(revsPerML);
        char buf[64];
        snprintf(buf, sizeof(buf), "Sample pump: %.2f revs/mL (from %.1f mL)", revsPerML, value);
        publishMessage(buf);
      }
      else if (strcmp(key, "prefill_ul") == 0) { configStore.setPrefillVolumeUL(value); }
      else if (strcmp(key, "max_acid_ml") == 0) { configStore.setMaxAcidML(value); }
      else if (strcmp(key, "fast_step_ul") == 0) { configStore.setFastStepUL((int)value); }
      else if (strcmp(key, "meas_temp_c") == 0) { configStore.setMeasTempC(value); }
      else if (strcmp(key, "buf_ph4") == 0 && value >= 3.0f && value <= 5.0f) {
        configStore.setBufferPH4(value); updateCalibrationFit();
      }
      else if (strcmp(key, "buf_ph7") == 0 && value >= 6.0f && value <= 8.0f) {
        configStore.setBufferPH7(value); updateCalibrationFit();
      }
      else if (strcmp(key, "buf_ph10") == 0 && value >= 9.0f && value <= 11.0f) {
        configStore.setBufferPH10(value); updateCalibrationFit();
      }
      else if (strcmp(key, "slope_hours") == 0) {
        configStore.setSlopeWindowHours((int)value);
        computeKHSlope();  // refresh cached slope with new window
      }
      else if (strcmp(key, "use_ads1115") == 0) {
        configStore.setUseADS1115((int)value == 1);
        publishMessage("ADS1115 setting changed. Reboot to apply.");
      }
      else if (strcmp(key, "sample_spreadcycle") == 0) {
        configStore.setSampleSpreadCycle((int)value == 1);
        setSampleSpreadCycle((int)value == 1);
      }
      else if (strcmp(key, "sample_stall_sg") == 0) {
        configStore.setSampleStallSG((int)value);
      }
      else if (strcmp(key, "titrate_spreadcycle") == 0) {
        configStore.setTitrateSpreadCycle((int)value == 1);
        setTitrateSpreadCycle((int)value == 1);
      }
      else if (strcmp(key, "titrate_stall_sg") == 0) {
        configStore.setTitrateStallSG((int)value);
      }
      else if (strcmp(key, "sample_sg_baseline") == 0) {
        configStore.setSampleSGBaseline((int)value);
      }
      else if (strcmp(key, "titrate_sg_baseline") == 0) {
        configStore.setTitrateSGBaseline((int)value);
      }
      else {
        saved = false;
      }

      sendConfigResult(key, saved);
      if (saved) {
        broadcastState();
      }
    } else if (strcmp(type, "schedule") == 0) {
      // Schedule mode (0=custom, 1=interval, 2=never)
      if (doc["mode"].is<JsonVariant>()) {
        configStore.setScheduleMode(doc["mode"].as<uint8_t>());
        scheduler.resetDailyFlags();
      }
      if (doc["intervalHours"].is<JsonVariant>()) {
        if (configStore.setIntervalHours(doc["intervalHours"].as<uint8_t>())) {
          scheduler.resetDailyFlags();
        } else {
          broadcastMessage("Invalid interval hours value");
        }
      }
      if (doc["anchorTime"].is<JsonVariant>()) {
        configStore.setAnchorTime(doc["anchorTime"].as<uint16_t>());
        scheduler.resetDailyFlags();
      }
      // Custom mode slots
      JsonArray slots = doc["slots"];
      if (slots) {
        uint8_t count = min((size_t)slots.size(), (size_t)8);
        configStore.setScheduleCount(count);
        for (uint8_t i = 0; i < count; i++) {
          int t = slots[i].as<int>();
          if (t < 0) t = 0;
          if (t > 1439) t = 1439;
          configStore.setScheduleTime(i, (uint16_t)t);
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

      bool isGran = (strcmp(sensor, "gran") == 0);
      bool isMotor = (strcmp(sensor, "motor") == 0);
      uint32_t cutoff = (uint32_t)time(nullptr) - (isMotor ? (90UL * 86400UL) : (7UL * 86400UL));

      if (isMotor) {
        // Motor CSV: timestamp,sampleAvg,sampleMin,titrateAvg,titrateMin
        JsonDocument resp;
        resp["type"] = "history";
        resp["sensor"] = "motor";
        JsonArray dataArr = resp["data"].to<JsonArray>();

        while (f.available() && dataArr.size() < 500) {
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
          JsonArray pt = dataArr.add<JsonArray>();
          pt.add(ts);
          pt.add(line.substring(c1 + 1, c2).toInt());
          pt.add(line.substring(c2 + 1, c3).toInt());
          pt.add(line.substring(c3 + 1, c4).toInt());
          pt.add(line.substring(c4 + 1).toInt());
        }
        f.close();

        static char buf[4096];
        size_t written = serializeJson(resp, buf, sizeof(buf));
        if (written > 0) ws.textAll(buf);
      } else if (strcmp(sensor, "precision") == 0) {
        // Precision CSV: timestamp,n,mean,sd,min,max,elapsedSec
        JsonDocument resp;
        resp["type"] = "history";
        resp["sensor"] = "precision";
        JsonArray dataArr = resp["data"].to<JsonArray>();

        while (f.available() && dataArr.size() < 100) {
          String line = f.readStringUntil('\n');
          if (line.length() == 0) continue;
          int c1 = line.indexOf(',');
          if (c1 < 0) continue;
          uint32_t ts = line.substring(0, c1).toInt();
          int c2 = line.indexOf(',', c1 + 1);
          int c3 = line.indexOf(',', c2 + 1);
          int c4 = line.indexOf(',', c3 + 1);
          int c5 = line.indexOf(',', c4 + 1);
          int c6 = line.indexOf(',', c5 + 1);
          if (c2 < 0 || c3 < 0 || c4 < 0 || c5 < 0 || c6 < 0) continue;
          JsonArray pt = dataArr.add<JsonArray>();
          pt.add(ts);
          pt.add(line.substring(c1 + 1, c2).toInt());    // n
          pt.add(line.substring(c2 + 1, c3).toFloat());  // mean
          pt.add(line.substring(c3 + 1, c4).toFloat());  // sd
          pt.add(line.substring(c4 + 1, c5).toFloat());  // min
          pt.add(line.substring(c5 + 1, c6).toFloat());  // max
          pt.add(line.substring(c6 + 1).toInt());         // elapsedSec
        }
        f.close();

        String out;
        serializeJson(resp, out);
        if (out.length() > 0) ws.textAll(out);
      } else if (isGran) {
        // Gran CSV: timestamp,r2,eqML,endpointPH,method,confidence,khGran,khEndpoint
        JsonDocument resp;
        resp["type"] = "history";
        resp["sensor"] = "gran";
        JsonArray dataArr = resp["data"].to<JsonArray>();

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
          // Parse remaining fields: method, confidence, khGran, khEndpoint, noise, reversals, dropUL, titRPM, khCI
          int c5 = line.indexOf(',', c4 + 1);
          int mth = 0;
          float conf = 0, khG = 0, khE = 0, noiseMv = 0, ci = 0;
          int reversals = 0;
          if (c5 > 0) {
            mth = line.substring(c4 + 1, c5).toInt();
            int c6 = line.indexOf(',', c5 + 1);
            if (c6 > 0) {
              conf = line.substring(c5 + 1, c6).toFloat();
              int c7 = line.indexOf(',', c6 + 1);
              if (c7 > 0) {
                khG = line.substring(c6 + 1, c7).toFloat();
                int c8 = line.indexOf(',', c7 + 1);
                if (c8 > 0) {
                  khE = line.substring(c7 + 1, c8).toFloat();
                  int c9 = line.indexOf(',', c8 + 1);
                  if (c9 > 0) {
                    noiseMv = line.substring(c8 + 1, c9).toFloat();
                    int c10 = line.indexOf(',', c9 + 1);
                    if (c10 > 0) {
                      reversals = line.substring(c9 + 1, c10).toInt();
                      // Skip dropUL (c10), titrationRPM (c11), read khCI (c12)
                      int c11 = line.indexOf(',', c10 + 1);
                      if (c11 > 0) {
                        int c12 = line.indexOf(',', c11 + 1);
                        if (c12 > 0) {
                          ci = line.substring(c12 + 1).toFloat();
                        }
                      }
                    } else {
                      reversals = line.substring(c9 + 1).toInt();
                    }
                  } else {
                    noiseMv = line.substring(c8 + 1).toFloat();
                  }
                } else {
                  khE = line.substring(c7 + 1).toFloat();
                }
              }
            }
          } else {
            mth = line.substring(c4 + 1).toInt();
          }
          JsonArray pt = dataArr.add<JsonArray>();
          pt.add(ts);
          pt.add(r2);
          pt.add(eq);
          pt.add(eph);
          pt.add(mth);
          pt.add(khG);
          pt.add(khE);
          pt.add(noiseMv);
          pt.add(reversals);
          pt.add(conf);
          pt.add(ci);
        }
        f.close();

        static char buf[6144];
        size_t written = serializeJson(resp, buf, sizeof(buf));
        if (written > 0) ws.textAll(buf);
      } else {
        // KH/pH CSV: timestamp,value
        JsonDocument resp;
        resp["type"] = "history";
        resp["sensor"] = sensor;
        JsonArray dataArr = resp["data"].to<JsonArray>();

        while (f.available() && dataArr.size() < 150) {
          String line = f.readStringUntil('\n');
          if (line.length() == 0) continue;
          int comma = line.indexOf(',');
          if (comma < 0) continue;
          uint32_t ts = line.substring(0, comma).toInt();
          if (ts < cutoff) continue;
          float val = line.substring(comma + 1).toFloat();
          JsonArray point = dataArr.add<JsonArray>();
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

void calibratePH(int bufferPH) {
  // Read temperature for buffer pH compensation (DS18B20 → fallback to config)
  float calTemp;
  if (hasTemperatureSensor()) {
    calTemp = getWaterTemperatureC();
  } else {
    calTemp = configStore.getMeasTempC();
  }
  configStore.setCalTempC(calTemp);

  char calMsg[64];
  snprintf(calMsg, sizeof(calMsg), "Calibrating pH %d at %.1f°C...", bufferPH, calTemp);
  publishMessage(calMsg);
  startStirrer();
  delay(STIRRER_WARMUP_MS);
  float v = measureVoltage(isExternalADCActive() ? 20 : 100);
  stopStirrer();
  float actualPH;
  bool ext = isExternalADCActive();
  switch (bufferPH) {
    case 4:
      voltage_4PH = v;
      if (ext) configStore.setVoltage4PHExt(v); else configStore.setVoltage4PH(v);
      actualPH = bufferPHAtTemp(4, calTemp);
      break;
    case 7:
      voltage_7PH = v;
      if (ext) configStore.setVoltage7PHExt(v); else configStore.setVoltage7PH(v);
      actualPH = bufferPHAtTemp(7, calTemp);
      break;
    case 10:
      voltage_10PH = v;
      if (ext) configStore.setVoltage10PHExt(v); else configStore.setVoltage10PH(v);
      actualPH = bufferPHAtTemp(10, calTemp);
      break;
    default: return;
  }
  updateCalibrationFit();
  uint32_t now = (uint32_t)time(nullptr);
  if (now > MIN_VALID_EPOCH) {
    configStore.setCalTimestamp(now);
    if (isCalibrationValid()) configStore.addSlopeEntry(now, getAcidSlope(), getProbeAsymmetry());
  }
  char msg[32];
  snprintf(msg, sizeof(msg), "Calibrated pH %.2f", actualPH);
  publishMessage(msg);
  if (isCalibrationValid()) {
    char reason[96];
    const char* health = getProbeHealthDetail(reason, sizeof(reason));
    if (strcmp(health, "Good") != 0) {
      char hBuf[128];
      snprintf(hBuf, sizeof(hBuf), "Warning: Probe %s — %s", health, reason);
      publishError(hBuf);
    }
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
    // Defer to loopTask — measurePH blocks ~7s (stabilization + 100 readings)
    queueCommand('p');
    return;
  } else if (strcmp(cmd, "f") == 0 || strcmp(cmd, "s") == 0 || strcmp(cmd, "r") == 0 || strcmp(cmd, "v") == 0) {
    // Defer motor/ADC ops to loopTask — these block for seconds
    queueCommand(cmd[0]);
    return;
  } else if (strcmp(cmd, "m") == 0) {
    // Defer to loopTask — startStirrer() has a 1s kick delay
    queueCommand('m');
    return;
  } else if (strcmp(cmd, "e") == 0) {
    // Defer to loopTask — keep AsyncTCP task free
    queueCommand('e');
    return;
  } else if (strcmp(cmd, "d") == 0) {
    // Defer motor diagnostics to loopTask — runs motors for ~30s
    queueCommand('d');
    return;
  } else if (strcmp(cmd, "ds") == 0) {
    // Sample pump diagnostics only
    queueCommand('B');
    return;
  } else if (strcmp(cmd, "dt") == 0) {
    // Titration pump diagnostics only
    queueCommand('C');
    return;
  } else if (strcmp(cmd, "H") == 0) {
    // Defer hardware diagnostics to loopTask — runs ~90-120s
    queueCommand('H');
    return;
  } else if (strcmp(cmd, "precision") == 0) {
    // Defer precision test to loopTask — runs 3 full measurements (~2+ hours)
    queueCommand('P');
    return;
  } else if (strcmp(cmd, "abort") == 0) {
    requestAbort();
    publishMessage("Aborting measurement...");
    return;
  } else if (strcmp(cmd, "cs") == 0) {
    // Defer sample pump calibration to loopTask
    queueCommand('S');
    return;
  } else if (strcmp(cmd, "cw") == 0) {
    // Defer tube cleaning to loopTask
    queueCommand('W');
    return;
  } else if (strcmp(cmd, "resetwifi") == 0) {
    configStore.clearWifiCredentials();
    publishMessage("WiFi credentials cleared. Restarting into AP mode...");
    delay(500);
    ESP.restart();
  } else if (strcmp(cmd, "o") == 0) {
    publishMessage("Restarting...");
    delay(100);
    ESP.restart();
  } else if (strcmp(cmd, "4") == 0 || strcmp(cmd, "7") == 0 || strcmp(cmd, "10") == 0) {
    // Defer to loopTask — calibratePH blocks ~10s (stirrer warmup + voltage measurement)
    queueCommand(strcmp(cmd, "10") == 0 ? 'A' : cmd[0]);  // '4','7','A' (A=pH10)
    return;
  }
  broadcastState();
}

void broadcastTitrationStart() {
  mesCount = 0;
  granBufCount = 0;
  ws.textAll("{\"type\":\"mesStart\"}");
}

void processPendingReplay() {
  // Replay now happens directly in WS_EVT_CONNECT; kept as no-op for API compat
}

void broadcastState() {
  if (ws.count() == 0) return;

  JsonDocument doc;
  doc["type"] = "state";
  doc["deviceName"] = deviceName;
  doc["fwVersion"] = FW_VERSION;
  doc["ph"] = pH;
  doc["startPh"] = startPH;
  doc["units"] = units.load();
  doc["kh"] = configStore.getLastKH();
  doc["lastStartPh"] = configStore.getLastStartPH();
  doc["hclVol"] = configStore.getHClVolume();
  doc["rssi"] = wifiManager.getRSSI();
  doc["uptime"] = millis() / 1000;
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["heapMin"] = heapMin;
  doc["mqttOk"] = mqttManager.isConnected();
  doc["wifiOk"] = wifiManager.isConnected();
  doc["ntpOk"] = scheduler.isTimeSynced();
  doc["nextMeas"] = scheduler.getNextMeasurementTime();
  doc["water_temp"] = getWaterTemperatureC();
  doc["temp_sensor"] = hasTemperatureSensor();
  doc["tmc"] = isTMCDetected();
  doc["stirrer"] = isStirrerRunning();
  doc["measuring"] = isMeasuringKH.load();

  // KH trend slope (dKH/day) and regression parameters for client trend line
  if (!isnan(cachedKHSlope)) {
    doc["khSlope"] = serialized(String(cachedKHSlope, 4));
    doc["khIntercept"] = serialized(String(cachedKHIntercept, 3));
    doc["khSlopeDay0"] = cachedKHSlopeDay0;
    doc["slopeNDays"] = cachedSlopeNDays;
    if (!isnan(cachedSlopeCI)) doc["slopeCI"] = serialized(String(cachedSlopeCI, 3));
  }
  if (!isnan(lastConfidence)) doc["confidence"] = lastConfidence;
  if (!isnan(lastKHResult.khCI)) doc["khCI"] = serialized(String(lastKHResult.khCI, 3));

  // Probe health
  JsonObject probe = doc["probe"].to<JsonObject>();
  probe["acidEff"] = getAcidEfficiency();
  probe["alkEff"] = getAlkalineEfficiency();
  probe["asymmetry"] = getProbeAsymmetry();
  probe["response"] = getLastStabilizationMs();
  probe["mV"] = voltage;
  probe["v4"] = voltage_4PH;
  probe["v7"] = voltage_7PH;
  probe["v10"] = voltage_10PH;
  float avgNoise = getAvgStabNoiseMv();
  if (avgNoise > 0) probe["noise"] = serialized(String(avgNoise, 1));
  probe["health"] = getProbeHealth();
  uint32_t calTs = configStore.getCalTimestamp();
  if (calTs > 0) {
    time_t now = time(nullptr);
    probe["calAge"] = (now > (time_t)calTs) ? (int)((now - calTs) / 86400) : 0;
  } else {
    probe["calAge"] = -1;  // never calibrated
  }

  // Asymmetry history over calibrations
  ConfigStore::SlopeEntry slopeHist[ConfigStore::MAX_SLOPE_HISTORY];
  int slopeCount = configStore.getSlopeHistory(slopeHist, ConfigStore::MAX_SLOPE_HISTORY);
  if (slopeCount > 0) {
    JsonArray eh = probe["effHist"].to<JsonArray>();
    for (int i = 0; i < slopeCount; i++) {
      JsonArray entry = eh.add<JsonArray>();
      entry.add(slopeHist[i].timestamp);
      entry.add((int)(slopeHist[i].asymmetry * 10.0f + 0.5f) / 10.0f);  // 1 decimal
    }
  }

  // Config values
  JsonObject cfg = doc["config"].to<JsonObject>();
  cfg["device_name"] = deviceName;
  cfg["titration_vol"] = configStore.getTitrationVolume();
  cfg["sample_vol"] = configStore.getSampleVolume();
  cfg["correction_factor"] = configStore.getCorrectionFactor();
  cfg["hcl_molarity"] = configStore.getHClMolarity();
  cfg["hcl_volume"] = configStore.getHClVolume();
  cfg["cal_drops"] = configStore.getCalUnits();
  cfg["fast_ph"] = configStore.getFastTitrationPH();
  cfg["endpoint_method"] = configStore.getEndpointMethod();
  cfg["min_start_ph"] = configStore.getMinStartPH();
  cfg["stab_timeout"] = configStore.getStabilizationTimeout();
  cfg["gran_mix_delay"] = configStore.getGranMixDelay();
  cfg["drop_ul"] = configStore.getDropVolumeUL();
  cfg["titration_rpm"] = configStore.getTitrationRPM();
  cfg["fast_phase_rpm"] = configStore.getFastPhaseRPM();
  cfg["sample_pump_rpm"] = configStore.getSamplePumpRPM();
  cfg["sample_cal_revs_per_ml"] = configStore.getSampleCalRevsPerML();
  cfg["sample_cal_revs"] = configStore.getSampleCalRevolutions();
  { float rpm = configStore.getSampleCalRevsPerML();
    int calRevs = configStore.getSampleCalRevolutions();
    cfg["sample_cal_vol"] = (rpm > 0) ? (float)calRevs / rpm : 0;
  }
  cfg["prefill_ul"] = configStore.getPrefillVolumeUL();
  cfg["max_acid_ml"] = configStore.getMaxAcidML();
  cfg["fast_step_ul"] = configStore.getFastStepUL();
  cfg["meas_temp_c"] = configStore.getMeasTempC();
  cfg["buf_ph4"] = configStore.getBufferPH4();
  cfg["buf_ph7"] = configStore.getBufferPH7();
  cfg["buf_ph10"] = configStore.getBufferPH10();
  cfg["slope_hours"] = configStore.getSlopeWindowHours();
  cfg["num_washes"] = configStore.getNumWashes();
  cfg["timezone"] = configStore.getTimezone();
  cfg["use_ads1115"] = configStore.getUseADS1115();
  cfg["ads_active"] = isExternalADCActive();
  cfg["ads_fallback"] = isExternalADCFallback();
  cfg["sample_spreadcycle"] = configStore.getSampleSpreadCycle();
  cfg["sample_stall_sg"] = configStore.getSampleStallSG();
  cfg["titrate_spreadcycle"] = configStore.getTitrateSpreadCycle();
  cfg["titrate_stall_sg"] = configStore.getTitrateStallSG();
  cfg["sample_sg_baseline"] = configStore.getSampleSGBaseline();
  cfg["titrate_sg_baseline"] = configStore.getTitrateSGBaseline();

  // Pump calibration ages (days, -1 = never)
  time_t nowTs = time(nullptr);
  uint32_t sampCalTs = configStore.getSampleCalTimestamp();
  cfg["sample_cal_age"] = (sampCalTs > 0 && nowTs > (time_t)sampCalTs) ? (int)((nowTs - sampCalTs) / 86400) : -1;
  uint32_t titCalTs = configStore.getTitrationCalTimestamp();
  cfg["titration_cal_age"] = (titCalTs > 0 && nowTs > (time_t)titCalTs) ? (int)((nowTs - titCalTs) / 86400) : -1;

  const char* th = getTubeHealth();
  if (th) doc["tubeHealth"] = th;

  // Schedule
  doc["schedMode"] = configStore.getScheduleMode();
  doc["intervalHours"] = configStore.getIntervalHours();
  doc["anchorTime"] = configStore.getAnchorTime();

  JsonArray sched = doc["schedule"].to<JsonArray>();
  uint8_t count = configStore.getScheduleCount();
  for (uint8_t i = 0; i < count; i++) {
    sched.add(configStore.getScheduleTime(i));
  }

  static char buf[2048];
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
    uint16_t pairs = mesCount / 2;
    for (uint16_t i = 0; i < pairs; i++) {
      mesBuffer[i].ml = mesBuffer[i * 2 + 1].ml; // keep later volume
      mesBuffer[i].ph = (mesBuffer[i * 2].ph + mesBuffer[i * 2 + 1].ph) / 2.0f;
      mesBuffer[i].mV = (mesBuffer[i * 2].mV + mesBuffer[i * 2 + 1].mV) / 2.0f;
    }
    // Preserve last point if odd count
    if (mesCount % 2 != 0) {
      mesBuffer[pairs] = mesBuffer[mesCount - 1];
      mesCount = pairs + 1;
    } else {
      mesCount = pairs;
    }
  }

  mesBuffer[mesCount].ml = ml;
  mesBuffer[mesCount].ph = phVal;
  mesBuffer[mesCount].mV = voltage;
  mesCount++;

  if (ws.count() == 0) return;

  JsonDocument doc;
  doc["type"] = "mesPh";
  doc["ph"] = phVal;
  doc["ml"] = ml;
  doc["mV"] = voltage;

  char buf[96];
  serializeJson(doc, buf, sizeof(buf));
  ws.textAll(buf);
}

void broadcastMessage(const char* msg) {
  addLogEntry('m', msg);
  if (ws.count() == 0) return;
  JsonDocument doc;
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
  JsonDocument doc;
  doc["type"] = "error";
  doc["text"] = msg;
  char buf[128];
  serializeJson(doc, buf, sizeof(buf));
  ws.textAll(buf);
}

void broadcastMotorDiag(const char* json) {
  if (ws.count() == 0) return;
  ws.textAll(json);
}

void broadcastProgress(int percent) {
  if (ws.count() == 0) return;
  char buf[32];
  snprintf(buf, sizeof(buf), "{\"type\":\"progress\",\"pct\":%d}", percent);
  ws.textAll(buf);
}

void broadcastGranData(float r2, float eqML, bool usedGran,
                       float* pointsML, float* pointsF, int nPts,
                       float winLowML, float winHighML,
                       float slopeML, float intercept,
                       GranWindowResult* windows, int nWindows) {
  // Cache for replay to new WebSocket clients
  granBufR2 = r2;
  granBufEqML = eqML;
  granBufUsed = usedGran;
  granBufWinLowML = winLowML;
  granBufWinHighML = winHighML;
  granBufSlopeML = slopeML;
  granBufIntercept = intercept;
  granBufCount = (uint8_t)min(nPts, (int)GRAN_BUF_MAX);
  for (uint8_t i = 0; i < granBufCount; i++) {
    granBuffer[i] = { pointsML[i], pointsF[i] };
  }
  granBufNWindows = (uint8_t)min(nWindows, (int)MAX_GRAN_WINDOWS);
  if (windows) {
    for (uint8_t i = 0; i < granBufNWindows; i++) granBufWindows[i] = windows[i];
  } else {
    granBufNWindows = 0;
  }

  if (ws.count() == 0 || nPts == 0) return;

  JsonDocument doc;
  doc["type"] = "granData";
  doc["r2"] = r2;
  doc["eqML"] = eqML;
  doc["used"] = usedGran;
  if (winLowML > 0) doc["winLowML"] = winLowML;
  if (winHighML > 0) doc["winHighML"] = winHighML;
  if (slopeML != 0) doc["slope"] = slopeML;
  if (intercept != 0) doc["intercept"] = intercept;

  JsonArray pts = doc["points"].to<JsonArray>();
  for (int i = 0; i < nPts; i++) {
    JsonArray pt = pts.add<JsonArray>();
    pt.add(pointsML[i]);
    pt.add(pointsF[i]);
  }

  if (nWindows > 0 && windows) {
    JsonArray wins = doc["windows"].to<JsonArray>();
    for (int i = 0; i < nWindows; i++) {
      JsonArray w = wins.add<JsonArray>();
      w.add(windows[i].low);
      w.add(windows[i].high);
      w.add(windows[i].r2);
      w.add(windows[i].valid ? 1 : 0);
      w.add(serialized(String(windows[i].eqUnits, 2))); // KH value (reused field)
      w.add(serialized(String(windows[i].eqSE, 4)));   // SE of equivalence point
    }
  }

  char buf[3072];
  size_t written = serializeJson(doc, buf, sizeof(buf));
  if (written > 0) ws.textAll(buf);
}

void appendHistory(const char* sensor, float value, uint32_t ts) {
  if (!sensor || strlen(sensor) > 10) return;
  char filename[32];
  snprintf(filename, sizeof(filename), "/history/%s.csv", sensor);

  // Ensure directory exists
  if (!LittleFS.exists("/history")) {
    if (!LittleFS.mkdir("/history")) {
      Serial.println("Error: failed to create /history directory");
      return;
    }
  }

  // Remove entries older than 7 days (atomic: write to temp file, then rename)
  if (LittleFS.exists(filename)) {
    uint32_t cutoff = ts - (7 * 86400);
    File f = LittleFS.open(filename, "r");
    if (f) {
      char tmpFile[36];
      snprintf(tmpFile, sizeof(tmpFile), "%s.tmp", filename);
      File fw = LittleFS.open(tmpFile, "w");
      if (fw) {
        int lines = 0;
        bool writeOk = true;
        while (f.available() && lines < 200) {
          String line = f.readStringUntil('\n');
          lines++;
          if (line.length() == 0) continue;
          int comma = line.indexOf(',');
          if (comma < 0) continue;
          uint32_t lineTs = line.substring(0, comma).toInt();
          if (lineTs >= cutoff) {
            if (fw.print(line + "\n") == 0) { writeOk = false; break; }
          }
        }
        fw.close();
        f.close();
        if (writeOk) {
          LittleFS.remove(filename);
          LittleFS.rename(tmpFile, filename);
        } else {
          LittleFS.remove(tmpFile);  // Discard partial file, keep original intact
          Serial.printf("Warning: write failed during %s pruning, keeping original\n", filename);
        }
      } else {
        f.close();
        Serial.printf("Warning: failed to open %s for pruning\n", tmpFile);
      }
    }
  }

  if (ts < MIN_VALID_EPOCH) {
    // NTP not yet synced — warn user that data won't be saved
    Serial.printf("WARNING: Skipping history write for %s — NTP not synced (ts=%u)\n", sensor, ts);
    publishError("Warning: Time not synced (NTP) — measurement not saved to history");
    return;
  }

  File f = LittleFS.open(filename, "a");
  if (f) {
    f.printf("%u,%.2f\n", ts, value);
    f.close();
  } else {
    Serial.printf("Warning: failed to append to %s\n", filename);
  }
}

void appendGranHistory(float r2, float eqML, float endpointPH, bool usedGran, float confidence, float khGran, float khEndpoint, float probeNoiseMv, int phReversals, float dropUL, float titrationRPM, float khCI, uint32_t ts, float startPH, float acidEff) {
  const char* filename = "/history/gran.csv";

  if (!LittleFS.exists("/history")) {
    if (!LittleFS.mkdir("/history")) {
      Serial.println("Error: failed to create /history directory");
      return;
    }
  }

  // Remove entries older than 7 days
  if (LittleFS.exists(filename)) {
    uint32_t cutoff = ts - (7 * 86400);
    File f = LittleFS.open(filename, "r");
    if (f) {
      String kept;
      kept.reserve(f.size());
      int lines = 0;
      while (f.available() && lines < 200) {
        String line = f.readStringUntil('\n');
        lines++;
        if (line.length() == 0) continue;
        int comma = line.indexOf(',');
        if (comma < 0) continue;
        uint32_t lineTs = line.substring(0, comma).toInt();
        if (lineTs >= cutoff) kept += line + "\n";
      }
      f.close();
      File fw = LittleFS.open(filename, "w");
      if (fw) { fw.print(kept); fw.close(); }
      else { Serial.println("Warning: failed to open gran.csv for pruning"); }
    }
  }

  if (ts < MIN_VALID_EPOCH) return;  // NTP not synced

  File f = LittleFS.open(filename, "a");
  if (f) {
    f.printf("%u,%.5f,%.3f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%d,%.1f,%.1f,%.3f,%.2f,%.1f\n", ts, r2, eqML, endpointPH, usedGran ? 1 : 0, confidence,
             isnan(khGran) ? 0.0f : khGran, isnan(khEndpoint) ? 0.0f : khEndpoint,
             isnan(probeNoiseMv) ? 0.0f : probeNoiseMv, phReversals, dropUL, titrationRPM,
             isnan(khCI) ? 0.0f : khCI,
             isnan(startPH) ? 0.0f : startPH,
             isnan(acidEff) ? 0.0f : acidEff);
    f.close();
  } else {
    Serial.println("Warning: failed to append to gran.csv");
  }
}

static uint16_t lastSampleSGAvgStored = 0;
static uint16_t lastTitrateSGAvgStored = 0;

void appendMotorHealth(uint32_t ts, int sampleAvg, int sampleMin, int titrateAvg, int titrateMin) {
  const char* filename = "/history/motor.csv";

  if (!LittleFS.exists("/history")) {
    if (!LittleFS.mkdir("/history")) return;
  }

  // Remove entries older than 90 days
  if (LittleFS.exists(filename)) {
    uint32_t cutoff = ts - (90UL * 86400UL);
    File f = LittleFS.open(filename, "r");
    if (f) {
      String kept;
      kept.reserve(f.size());
      int lines = 0;
      while (f.available() && lines < 500) {
        String line = f.readStringUntil('\n');
        lines++;
        if (line.length() == 0) continue;
        int comma = line.indexOf(',');
        if (comma < 0) continue;
        uint32_t lineTs = line.substring(0, comma).toInt();
        if (lineTs >= cutoff) kept += line + "\n";
      }
      f.close();
      File fw = LittleFS.open(filename, "w");
      if (fw) { fw.print(kept); fw.close(); }
    }
  }

  lastSampleSGAvgStored = sampleAvg;
  lastTitrateSGAvgStored = titrateAvg;

  if (ts < MIN_VALID_EPOCH) return;

  File f = LittleFS.open(filename, "a");
  if (f) {
    f.printf("%u,%d,%d,%d,%d\n", ts, sampleAvg, sampleMin, titrateAvg, titrateMin);
    f.close();
  }
}

void appendPrecisionHistory(uint32_t ts, int n, float mean, float sd, float vmin, float vmax, unsigned long elapsedSec) {
  const char* filename = "/history/precision.csv";

  if (!LittleFS.exists("/history")) {
    if (!LittleFS.mkdir("/history")) return;
  }

  // Keep last 90 days (precision tests are infrequent)
  if (LittleFS.exists(filename)) {
    uint32_t cutoff = ts - (90UL * 86400UL);
    File f = LittleFS.open(filename, "r");
    if (f) {
      String kept;
      kept.reserve(f.size());
      while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.length() == 0) continue;
        int comma = line.indexOf(',');
        if (comma < 0) continue;
        uint32_t lineTs = line.substring(0, comma).toInt();
        if (lineTs >= cutoff) kept += line + "\n";
      }
      f.close();
      File fw = LittleFS.open(filename, "w");
      if (fw) { fw.print(kept); fw.close(); }
    }
  }

  if (ts < MIN_VALID_EPOCH) return;

  File f = LittleFS.open(filename, "a");
  if (f) {
    f.printf("%u,%d,%.2f,%.3f,%.2f,%.2f,%lu\n", ts, n, mean, sd, vmin, vmax, elapsedSec);
    f.close();
  }
}

// Tube health: compare latest SG to baseline
const char* getTubeHealth() {
  int sampleBL = configStore.getSampleSGBaseline();
  int titrateBL = configStore.getTitrateSGBaseline();
  if (sampleBL == 0 && titrateBL == 0) return nullptr;

  // Use the pump with a baseline; if both have baselines, use the worse ratio
  float ratio = 1.0f;
  if (sampleBL > 0 && lastSampleSGAvgStored > 0) {
    float r = (float)lastSampleSGAvgStored / sampleBL;
    if (r < ratio) ratio = r;
  }
  if (titrateBL > 0 && lastTitrateSGAvgStored > 0) {
    float r = (float)lastTitrateSGAvgStored / titrateBL;
    if (r < ratio) ratio = r;
  }
  if (ratio > 0.8f) return "Good";
  if (ratio > 0.6f) return "Aging";
  return "Replace";
}

int getRecentKHValues(float* outValues, int maxCount) {
  const char* filename = "/history/kh.csv";
  if (!LittleFS.exists(filename)) return 0;

  File f = LittleFS.open(filename, "r");
  if (!f) return 0;

  // Read all values into a ring buffer to get the last N
  int cap = (maxCount > 10) ? 10 : maxCount;
  float ring[10];
  int count = 0;
  int head = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() == 0) continue;
    int comma = line.indexOf(',');
    if (comma < 0) continue;
    float val = line.substring(comma + 1).toFloat();
    if (val <= 0) continue;

    ring[head] = val;
    head = (head + 1) % cap;
    if (count < cap) count++;
  }
  f.close();

  // Copy ring buffer oldest-first into output
  int start = (count < cap) ? 0 : head;
  for (int i = 0; i < count; i++) {
    outValues[i] = ring[(start + i) % cap];
  }
  return count;
}

// Compute KH trend slope (dKH/day) from configurable lookback window using linear regression.
// Returns NAN if insufficient data (< 3 points).
float computeKHSlope() {
  const char* filename = "/history/kh.csv";
  if (!LittleFS.exists(filename)) return NAN;

  uint32_t now = (uint32_t)time(nullptr);
  if (now < MIN_VALID_EPOCH) return NAN;
  int slopeHours = configStore.getSlopeWindowHours();
  uint32_t cutoff = now - ((uint32_t)slopeHours * 3600);

  // Read recent KH values with timestamps
  static const int MAX_PTS = 200;
  uint32_t ts[MAX_PTS];
  float kh[MAX_PTS];
  int n = 0;

  File f = LittleFS.open(filename, "r");
  if (!f) return NAN;
  while (f.available() && n < MAX_PTS) {
    String line = f.readStringUntil('\n');
    if (line.length() == 0) continue;
    int comma = line.indexOf(',');
    if (comma < 0) continue;
    uint32_t t = line.substring(0, comma).toInt();
    float v = line.substring(comma + 1).toFloat();
    if (t >= cutoff && v > 0) {
      ts[n] = t;
      kh[n] = v;
      n++;
    }
  }
  f.close();

  if (n < 3) return NAN;

  // Group measurements by calendar day and compute median per day.
  // This eliminates diurnal cycle bias that plagued raw regression.
  struct DayBucket { float vals[30]; int cnt; uint32_t dayIdx; };
  DayBucket days[8];
  int nDays = 0;

  for (int i = 0; i < n; i++) {
    uint32_t dayIdx = ts[i] / 86400;
    int d = -1;
    for (int j = 0; j < nDays; j++) {
      if (days[j].dayIdx == dayIdx) { d = j; break; }
    }
    if (d < 0) {
      if (nDays >= 8) continue;
      d = nDays++;
      days[d].dayIdx = dayIdx;
      days[d].cnt = 0;
    }
    if (days[d].cnt < 30) {
      days[d].vals[days[d].cnt++] = kh[i];
    }
  }

  if (nDays < 3) return NAN;

  // Compute median of each day bucket and regress on (dayIdx, median)
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  double d0 = (double)days[0].dayIdx;
  double medians[8];
  double xvals[8];
  for (int d = 0; d < nDays; d++) {
    // Sort values to find median
    for (int i = 1; i < days[d].cnt; i++) {
      float key = days[d].vals[i];
      int j = i - 1;
      while (j >= 0 && days[d].vals[j] > key) {
        days[d].vals[j + 1] = days[d].vals[j];
        j--;
      }
      days[d].vals[j + 1] = key;
    }
    medians[d] = (double)days[d].vals[days[d].cnt / 2];
    xvals[d] = (double)days[d].dayIdx - d0;

    sx += xvals[d]; sy += medians[d]; sxx += xvals[d] * xvals[d]; sxy += xvals[d] * medians[d];
  }
  double denom = (double)nDays * sxx - sx * sx;
  if (fabs(denom) < 1e-12) return NAN;

  // Slope and intercept (dKH/day, x-axis is days)
  double slope = ((double)nDays * sxy - sx * sy) / denom;
  double intercept = (sy - slope * sx) / nDays;

  // Residual SE and slope CI
  double Sxx = sxx - sx * sx / nDays;
  double ssRes = 0;
  for (int d = 0; d < nDays; d++) {
    double pred = slope * xvals[d] + intercept;
    double r = medians[d] - pred;
    ssRes += r * r;
  }
  double se = (nDays > 2) ? sqrt(ssRes / (nDays - 2)) : 0;

  cachedKHSlope = (float)slope;
  cachedKHIntercept = (float)intercept;
  cachedKHSlopeDay0 = (uint32_t)days[0].dayIdx;
  cachedSlopeNDays = nDays;
  cachedSlopeCI = (Sxx > 0 && se > 0) ? (float)(1.96 * se / sqrt(Sxx)) : NAN;
  return cachedKHSlope;
}

// NAN-safe float formatter for JSON diagnostics output
static int diagFloat(char* buf, size_t len, float val, int dec) {
  if (isnan(val)) return snprintf(buf, len, "null");
  return snprintf(buf, len, "%.*f", dec, (double)val);
}

void setupWebServer() {
  LittleFS.begin(true); // Format if mount fails

  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/www/").setDefaultFile("index.html").setCacheControl("no-store, must-revalidate");

  // Serve history files for backup before filesystem upload
  server.serveStatic("/history/", LittleFS, "/history/");

  // WebSocket handler
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // CSV export: merge kh, ph, gran history into one download
  server.on("/api/export.csv", HTTP_GET, [](AsyncWebServerRequest* request) {
    // Collect entries keyed by timestamp
    struct Row { float kh; float ph; float r2; float eqML; float epPH; int method; float confidence; float khGran; float khEndpoint; float probeNoiseMv; int phReversals; float khCI; bool hasGran; };
    static const int MAX_ROWS = 200;
    static uint32_t timestamps[200];
    static Row rows[200];
    int nRows = 0;

    auto findOrAdd = [&](uint32_t ts) -> int {
      for (int i = 0; i < nRows; i++) { if (timestamps[i] == ts) return i; }
      if (nRows >= MAX_ROWS) return -1;
      timestamps[nRows] = ts;
      rows[nRows] = {NAN, NAN, NAN, NAN, NAN, -1, NAN, NAN, NAN, NAN, -1, NAN, false};
      return nRows++;
    };

    // Read kh.csv
    if (LittleFS.exists("/history/kh.csv")) {
      File f = LittleFS.open("/history/kh.csv", "r");
      while (f && f.available()) {
        String line = f.readStringUntil('\n');
        int c = line.indexOf(',');
        if (c < 0) continue;
        int idx = findOrAdd(line.substring(0, c).toInt());
        if (idx >= 0) rows[idx].kh = line.substring(c + 1).toFloat();
      }
      if (f) f.close();
    }

    // Read ph.csv
    if (LittleFS.exists("/history/ph.csv")) {
      File f = LittleFS.open("/history/ph.csv", "r");
      while (f && f.available()) {
        String line = f.readStringUntil('\n');
        int c = line.indexOf(',');
        if (c < 0) continue;
        int idx = findOrAdd(line.substring(0, c).toInt());
        if (idx >= 0) rows[idx].ph = line.substring(c + 1).toFloat();
      }
      if (f) f.close();
    }

    // Read gran.csv
    if (LittleFS.exists("/history/gran.csv")) {
      File f = LittleFS.open("/history/gran.csv", "r");
      while (f && f.available()) {
        String line = f.readStringUntil('\n');
        int c1 = line.indexOf(',');
        if (c1 < 0) continue;
        int c2 = line.indexOf(',', c1 + 1);
        int c3 = line.indexOf(',', c2 + 1);
        int c4 = line.indexOf(',', c3 + 1);
        if (c2 < 0 || c3 < 0 || c4 < 0) continue;
        int idx = findOrAdd(line.substring(0, c1).toInt());
        if (idx >= 0) {
          rows[idx].r2 = line.substring(c1 + 1, c2).toFloat();
          rows[idx].eqML = line.substring(c2 + 1, c3).toFloat();
          rows[idx].epPH = line.substring(c3 + 1, c4).toFloat();
          // Parse method, confidence, khGran, khEndpoint (fields 5-8)
          int c5 = line.indexOf(',', c4 + 1);
          if (c5 > 0) {
            rows[idx].method = line.substring(c4 + 1, c5).toInt();
            int c6 = line.indexOf(',', c5 + 1);
            if (c6 > 0) {
              rows[idx].confidence = line.substring(c5 + 1, c6).toFloat();
              int c7 = line.indexOf(',', c6 + 1);
              if (c7 > 0) {
                float g = line.substring(c6 + 1, c7).toFloat();
                int c8 = line.indexOf(',', c7 + 1);
                if (c8 > 0) {
                  float e = line.substring(c7 + 1, c8).toFloat();
                  if (g > 0) rows[idx].khGran = g;
                  if (e > 0) rows[idx].khEndpoint = e;
                  // Parse noise fields (probeNoiseMv, phReversals, dropUL, titRPM, khCI) if present
                  int c9 = line.indexOf(',', c8 + 1);
                  if (c9 > 0) {
                    rows[idx].probeNoiseMv = line.substring(c8 + 1, c9).toFloat();
                    int c10 = line.indexOf(',', c9 + 1);
                    if (c10 > 0) {
                      rows[idx].phReversals = line.substring(c9 + 1, c10).toInt();
                      // Skip dropUL (c10), titRPM (c11), read khCI (c12)
                      int c11 = line.indexOf(',', c10 + 1);
                      if (c11 > 0) {
                        int c12 = line.indexOf(',', c11 + 1);
                        if (c12 > 0) {
                          float ciVal = line.substring(c12 + 1).toFloat();
                          if (ciVal > 0) rows[idx].khCI = ciVal;
                        }
                      }
                    } else {
                      rows[idx].phReversals = line.substring(c9 + 1).toInt();
                    }
                  }
                } else {
                  float e = line.substring(c7 + 1).toFloat();
                  if (g > 0) rows[idx].khGran = g;
                  if (e > 0) rows[idx].khEndpoint = e;
                }
              }
            } else {
              rows[idx].confidence = line.substring(c5 + 1).toFloat();
            }
          } else {
            rows[idx].method = line.substring(c4 + 1).toInt();
          }
          rows[idx].hasGran = true;
        }
      }
      if (f) f.close();
    }

    // Sort by timestamp
    for (int i = 1; i < nRows; i++) {
      for (int j = i; j > 0 && timestamps[j] < timestamps[j - 1]; j--) {
        uint32_t tt = timestamps[j]; timestamps[j] = timestamps[j - 1]; timestamps[j - 1] = tt;
        Row tr = rows[j]; rows[j] = rows[j - 1]; rows[j - 1] = tr;
      }
    }

    // Stream response
    String csv;
    csv.reserve(nRows * 80 + 80);
    csv += "timestamp,datetime,kh,ph,r2,eq_ml,endpoint_ph,method,confidence,kh_gran,kh_endpoint,probe_noise_mv,ph_reversals,kh_ci\n";
    for (int i = 0; i < nRows; i++) {
      time_t t = (time_t)timestamps[i];
      struct tm tm;
      localtime_r(&t, &tm);
      char dt[20];
      strftime(dt, sizeof(dt), "%Y-%m-%d %H:%M", &tm);
      csv += String(timestamps[i]) + "," + dt;
      csv += isnan(rows[i].kh) ? ",": ("," + String(rows[i].kh, 2));
      csv += isnan(rows[i].ph) ? "," : ("," + String(rows[i].ph, 2));
      if (rows[i].hasGran) {
        csv += "," + String(rows[i].r2, 5) + "," + String(rows[i].eqML, 3)
             + "," + String(rows[i].epPH, 2) + "," + String(rows[i].method);
        csv += isnan(rows[i].confidence) ? "," : ("," + String(rows[i].confidence, 2));
        csv += isnan(rows[i].khGran) ? "," : ("," + String(rows[i].khGran, 2));
        csv += isnan(rows[i].khEndpoint) ? "," : ("," + String(rows[i].khEndpoint, 2));
        csv += isnan(rows[i].probeNoiseMv) ? "," : ("," + String(rows[i].probeNoiseMv, 2));
        csv += (rows[i].phReversals >= 0) ? ("," + String(rows[i].phReversals)) : ",";
        csv += isnan(rows[i].khCI) ? "," : ("," + String(rows[i].khCI, 3));
      } else {
        csv += ",,,,,,,,,,";
      }
      csv += "\n";
    }

    AsyncWebServerResponse* resp = request->beginResponse(200, "text/csv", csv);
    char cdHeader[80];
    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);
    snprintf(cdHeader, sizeof(cdHeader),
      "attachment; filename=\"kh_history_%04d%02d%02d_%02d%02d.csv\"",
      tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min);
    resp->addHeader("Content-Disposition", cdHeader);
    request->send(resp);
  });

  // Raw history file download (used by backup script before uploadfs)
  // Captures filename at registration time to prevent path traversal via URL parsing
  const char* historyFiles[] = {"kh", "ph", "gran", "precision"};
  for (int i = 0; i < 4; i++) {
    const char* name = historyFiles[i];
    server.on(
      (String("/api/history/") + name).c_str(), HTTP_GET,
      [name](AsyncWebServerRequest* request) {
        char filePath[32];
        snprintf(filePath, sizeof(filePath), "/history/%s.csv", name);
        if (LittleFS.exists(filePath)) {
          request->send(LittleFS, filePath, "text/csv");
        } else {
          request->send(204);  // No content
        }
      }
    );
  }

  // Diagnostics JSON download — chunked to avoid large heap allocation
  server.on("/api/diagnostics", HTTP_GET, [](AsyncWebServerRequest* request) {
    static int ds;       // section index
    static uint16_t dp;  // point index for batched arrays
    ds = 0;
    dp = 0;

    AsyncWebServerResponse* response = request->beginChunkedResponse(
      "application/json",
      [](uint8_t* buffer, size_t maxLen, size_t /* index */) -> size_t {
        char* b = (char*)buffer;
        int n = 0;
        if (maxLen < 64) return 0;

        switch (ds) {
          case 0: { // Header + config
            n = snprintf(b, maxLen,
              "{\"device\":\"%s\",\"firmware\":\"%s\","
              "\"timestamp\":%u,\"uptime\":%lu,\"freeHeap\":%u,\"heapMin\":%u,"
              "\"config\":{"
              "\"titration_vol\":%.2f,\"sample_vol\":%.1f,"
              "\"correction_factor\":%.3f,\"hcl_molarity\":%.4f,"
              "\"hcl_volume\":%.1f,\"cal_units\":%d,"
              "\"fast_ph\":%.1f,\"endpoint_method\":%d,"
              "\"min_start_ph\":%.1f,\"stab_timeout\":%d,\"gran_mix_delay\":%d,"
              "\"drop_ul\":%.1f,\"titration_rpm\":%.1f,\"fast_phase_rpm\":%.1f,\"prefill_ul\":%.1f,"
              "\"max_acid_ml\":%.1f,\"fast_step_ul\":%d,"
              "\"cal_v4\":%.2f,\"cal_v7\":%.2f,\"cal_v10\":%.2f,"
              "\"schedule_mode\":%d,\"interval_hours\":%d,\"anchor_time\":%d"
              "},",
              deviceName, FW_VERSION,
              (uint32_t)time(nullptr), millis() / 1000, ESP.getFreeHeap(), heapMin,
              configStore.getTitrationVolume(), configStore.getSampleVolume(),
              configStore.getCorrectionFactor(), configStore.getHClMolarity(),
              configStore.getHClVolume(), configStore.getCalUnits(),
              configStore.getFastTitrationPH(), (int)configStore.getEndpointMethod(),
              configStore.getMinStartPH(), configStore.getStabilizationTimeout(), configStore.getGranMixDelay(),
              configStore.getDropVolumeUL(), configStore.getTitrationRPM(), configStore.getFastPhaseRPM(), configStore.getPrefillVolumeUL(),
              configStore.getMaxAcidML(), configStore.getFastStepUL(),
              voltage_4PH, voltage_7PH, voltage_10PH,
              (int)configStore.getScheduleMode(),
              (int)configStore.getIntervalHours(),
              (int)configStore.getAnchorTime());
            ds = 1;
            break;
          }
          case 1: { // Constants
            n = snprintf(b, maxLen,
              "\"constants\":{"
              "\"gran_region_ph\":%.1f,\"gran_stop_ph\":%.1f,"
              "\"endpoint_ph\":%.1f,\"fixed_stop_ph\":%.1f,"
              "\"min_gran_points\":%d,\"gran_min_r2\":%.2f,"
              "\"steps_per_rev\":%d,\"motor_steps_per_unit\":%d,"
              "\"titration_step_size\":%d,\"medium_step_mult\":%d,"
              "\"gran_step_mult\":%d,\"fast_batch_max\":%d,"
              "\"fast_batch_min\":%d,\"nernst_factor\":%.5f,"
              "\"meas_temp_c\":%.1f,\"cal_temp_c\":%.1f,\"ph_amp_gain\":%.1f,"
              "\"water_temp\":%.1f,\"temp_sensor\":%s,"
              "\"tmc_detected\":%s},",
              GRAN_REGION_PH, GRAN_STOP_PH,
              ENDPOINT_PH, FIXED_ENDPOINT_STOP_PH,
              MIN_GRAN_POINTS, GRAN_MIN_R2,
              STEPS_PER_REVOLUTION, MOTOR_STEPS_PER_UNIT,
              TITRATION_STEP_SIZE, MEDIUM_STEP_MULTIPLIER,
              GRAN_STEP_MULTIPLIER, FAST_BATCH_MAX,
              FAST_BATCH_MIN, NERNST_FACTOR,
              configStore.getMeasTempC(), configStore.getCalTempC(), PH_AMP_GAIN,
              getWaterTemperatureC(),
              hasTemperatureSensor() ? "true" : "false",
              isTMCDetected() ? "true" : "false");
            ds = 2;
            break;
          }
          case 2: { // Measurement result
            if (!hasLastKHResult) {
              n = snprintf(b, maxLen, "\"measurement\":{\"available\":false},");
            } else {
              const KHResult& r = lastKHResult;
              char sv[16],sg[16],se[16],sp[16],sh[16],sr[16],ep[16],co[16],cv[16];
              diagFloat(sv, 16, r.khValue, 2);
              diagFloat(sg, 16, r.khGran, 2);
              diagFloat(se, 16, r.khEndpoint, 2);
              diagFloat(sp, 16, r.startPH, 2);
              diagFloat(sh, 16, r.hclUsed, 3);
              diagFloat(sr, 16, r.granR2, 5);
              diagFloat(ep, 16, r.endpointPH, 2);
              diagFloat(co, 16, r.confidence, 2);
              diagFloat(cv, 16, r.crossValDiff, 2);
              n = snprintf(b, maxLen,
                "\"measurement\":{\"available\":true,"
                "\"timestamp\":%u,"
                "\"kh_value\":%s,\"kh_gran\":%s,\"kh_endpoint\":%s,"
                "\"start_ph\":%s,\"hcl_used_ml\":%s,"
                "\"gran_r2\":%s,\"endpoint_ph\":%s,"
                "\"used_gran\":%s,\"confidence\":%s,"
                "\"data_points\":%d,\"stab_timeouts\":%d,"
                "\"elapsed_sec\":%lu,\"cross_val_diff\":%s,"
                "\"rssi_min\":%d,\"rssi_max\":%d},",
                lastMeasTimestamp,
                sv, sg, se, sp, sh, sr, ep,
                r.usedGran ? "true" : "false",
                co, r.dataPointCount, r.stabTimeouts,
                r.elapsedSec, cv,
                (int)r.rssiMin, (int)r.rssiMax);
            }
            ds = 3;
            break;
          }
          case 3: { // Noise metrics
            if (hasLastKHResult) {
              const KHResult& r = lastKHResult;
              float reversalPct = (r.granStepCount > 1)
                ? (r.phReversals * 100.0f / (r.granStepCount - 1)) : 0;
              // Estimate pump noise: sqrt(step_noise^2 - (probe_noise_mv * acidSlope_ph_per_mv)^2)
              float acidSl = getAcidSlope();
              float phPerMv = (!isnan(acidSl) && fabsf(acidSl) > 1.0f)
                ? 1.0f / fabsf(acidSl) : 0.00575f;
              float probeNoisePh = r.probeNoiseMv * phPerMv;
              float pumpNoise = 0;
              if (r.stepNoisePh > probeNoisePh) {
                pumpNoise = sqrtf(r.stepNoisePh * r.stepNoisePh
                                  - probeNoisePh * probeNoisePh);
              }
              char pn[16], sn[16], pmp[16], mn[16];
              diagFloat(pn, 16, r.probeNoiseMv, 2);
              diagFloat(sn, 16, r.stepNoisePh, 4);
              diagFloat(pmp, 16, pumpNoise, 4);
              diagFloat(mn, 16, r.maxNoiseMv, 2);
              n = snprintf(b, maxLen,
                "\"noise\":{\"probe_noise_mv\":%s,\"max_noise_mv\":%s,"
                "\"high_noise_count\":%d,"
                "\"step_noise_ph\":%s,"
                "\"ph_reversals\":%d,\"gran_steps\":%d,"
                "\"reversal_rate_pct\":%.1f,"
                "\"estimated_pump_noise_ph\":%s},",
                pn, mn, r.highNoiseCount, sn, r.phReversals, r.granStepCount,
                reversalPct, pmp);
            } else {
              n = 0;  // No noise data without measurement
            }
            ds = 4;
            break;
          }
          case 4: { // Probe health
            char reason[96];
            const char* health = getProbeHealthDetail(reason, sizeof(reason));
            char ae[16], al[16], ay[16], as[16], ls[16];
            diagFloat(ae, 16, getAcidEfficiency(), 1);
            diagFloat(al, 16, getAlkalineEfficiency(), 1);
            diagFloat(ay, 16, getProbeAsymmetry(), 1);
            diagFloat(as, 16, getAcidSlope(), 3);
            diagFloat(ls, 16, getAlkalineSlope(), 3);
            n = snprintf(b, maxLen,
              "\"probe\":{\"health\":\"%s\",\"reason\":\"%s\","
              "\"acid_eff\":%s,\"alk_eff\":%s,\"asymmetry\":%s,"
              "\"acid_slope\":%s,\"alk_slope\":%s,"
              "\"nernst_ideal\":%.3f,"
              "\"response_ms\":%lu,\"cal_ts\":%u},",
              health, reason,
              ae, al, ay, as, ls,
              NERNST_FACTOR * (273.15f + configStore.getCalTempC()),
              getLastStabilizationMs(),
              configStore.getCalTimestamp());
            ds = 5;
            break;
          }
          case 5: { // Gran scatter (header + all points)
            int p = snprintf(b, maxLen,
              "\"gran_scatter\":{\"r2\":%.4f,\"eq_ml\":%.3f,"
              "\"used\":%s,\"win_low\":%.2f,\"win_high\":%.2f,"
              "\"count\":%d,\"points\":[",
              granBufR2, granBufEqML,
              granBufUsed ? "true" : "false",
              lastKHResult.granWinLow, lastKHResult.granWinHigh,
              (int)granBufCount);
            for (uint8_t i = 0; i < granBufCount && p < (int)maxLen - 30; i++) {
              if (i > 0) p += snprintf(b + p, maxLen - p, ",");
              p += snprintf(b + p, maxLen - p, "[%.3f,%.6f]",
                           granBuffer[i].ml, granBuffer[i].f);
            }
            p += snprintf(b + p, maxLen - p, "]},");
            n = p;
            ds = 6;
            break;
          }
          case 6: { // Analysis data points header
            n = snprintf(b, maxLen,
              "\"analysis_points\":{\"count\":%d,"
              "\"fields\":[\"units\",\"pH\",\"mV\",\"stabMs\",\"phase\",\"flags\"],"
              "\"points\":[", analysisCount);
            dp = 0;
            ds = (analysisCount > 0) ? 7 : 8;
            break;
          }
          case 7: { // Analysis points (batched)
            int p = 0;
            while (dp < (uint16_t)analysisCount && p < (int)maxLen - 50) {
              if (dp > 0) p += snprintf(b + p, maxLen - p, ",");
              p += snprintf(b + p, maxLen - p, "[%.0f,%.3f,%.1f,%u,%u,%u]",
                           analysisBuf[dp].units, analysisBuf[dp].pH,
                           analysisBuf[dp].mV, analysisBuf[dp].stabMs,
                           analysisBuf[dp].phase, analysisBuf[dp].flags);
              dp++;
            }
            n = p;
            if (dp >= (uint16_t)analysisCount) ds = 8;
            break;
          }
          case 8: { // Close analysis + titration curve header
            n = snprintf(b, maxLen,
              "]},\"titration_curve\":{\"count\":%d,\"points\":[", mesCount);
            dp = 0;
            ds = (mesCount > 0) ? 9 : 10;
            break;
          }
          case 9: { // Titration points (batched)
            int p = 0;
            while (dp < mesCount && p < (int)maxLen - 40) {
              if (dp > 0) p += snprintf(b + p, maxLen - p, ",");
              p += snprintf(b + p, maxLen - p, "[%.3f,%.2f,%.1f]",
                           mesBuffer[dp].ml, mesBuffer[dp].ph, mesBuffer[dp].mV);
              dp++;
            }
            n = p;
            if (dp >= mesCount) ds = 10;
            break;
          }
          case 10: { // Close titration + event log header
            n = snprintf(b, maxLen, "]},\"event_log\":[");
            dp = 0;
            ds = (logCount > 0) ? 11 : 12;
            break;
          }
          case 11: { // Event log entries (batched)
            int p = 0;
            uint8_t st = (logCount < LOG_BUF_MAX) ? 0 : logHead;
            while (dp < logCount && p < (int)maxLen - 160) {
              uint8_t idx = (st + dp) % LOG_BUF_MAX;
              if (dp > 0) p += snprintf(b + p, maxLen - p, ",");
              // JSON-escape quotes and backslashes in log text
              char esc[200];
              int e = 0;
              for (int j = 0; logBuffer[idx].text[j] && e < 198; j++) {
                char c = logBuffer[idx].text[j];
                if (c == '"' || c == '\\') esc[e++] = '\\';
                esc[e++] = c;
              }
              esc[e] = '\0';
              p += snprintf(b + p, maxLen - p,
                "{\"type\":\"%s\",\"text\":\"%s\",\"ts\":%u}",
                logBuffer[idx].type == 'e' ? "error" : "msg",
                esc, logBuffer[idx].ts);
              dp++;
            }
            n = p;
            if (dp >= logCount) ds = 12;
            break;
          }
          case 12: { // Close event log + close JSON
            n = snprintf(b, maxLen, "]}");
            ds = 13;
            break;
          }
          default:
            return (size_t)0;
        }

        if (n >= (int)maxLen) n = (int)maxLen - 1;
        return (size_t)n;
      });

    char cdHeader[80];
    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);
    snprintf(cdHeader, sizeof(cdHeader),
      "attachment; filename=\"kh_diagnostics_%04d%02d%02d_%02d%02d.json\"",
      tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min);
    response->addHeader("Content-Disposition", cdHeader);
    request->send(response);
  });

  // Fallback API for state
  server.on("/api/state", HTTP_GET, [](AsyncWebServerRequest* request) {
    JsonDocument doc;
    doc["ph"] = pH;
    doc["hclVol"] = configStore.getHClVolume();
    doc["rssi"] = wifiManager.getRSSI();
    doc["uptime"] = millis() / 1000;
    char buf[256];
    serializeJson(doc, buf, sizeof(buf));
    request->send(200, "application/json", buf);
  });

  // Hardware diagnostics JSON download
  server.on("/api/hwdiag", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!isHWDiagReportReady()) {
      request->send(404, "application/json", "{\"error\":\"No diagnostics report available. Run diagnostics first.\"}");
      return;
    }

    static int hwds;
    hwds = 0;

    AsyncWebServerResponse* response = request->beginChunkedResponse(
      "application/json",
      [](uint8_t* buffer, size_t maxLen, size_t /* index */) -> size_t {
        if (maxLen < 64) return 0;
        if (hwds >= getHWDiagSectionCount()) return 0;  // done

        size_t written = 0;
        serveHWDiagChunk(hwds, (char*)buffer, maxLen, &written);
        hwds++;
        return written;
      });
    char cdHeader[120];
    time_t now = time(nullptr);
    struct tm tm;
    localtime_r(&now, &tm);
    snprintf(cdHeader, sizeof(cdHeader),
      "attachment; filename=\"%s_hw_diagnostics_%04d%02d%02d_%02d%02d.json\"",
      deviceName, tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min);
    response->addHeader("Content-Disposition", cdHeader);
    request->send(response);
  });

  registerOTAUploadHandler();

  server.begin();
  Serial.println("Web server started on port 80");
}

// OTA firmware/filesystem upload via HTTP POST
void registerOTAUploadHandler() {
  server.on("/api/update", HTTP_POST,
    // Response handler (after upload completes)
    [](AsyncWebServerRequest* request) {
      bool success = !Update.hasError();
      request->send(200, "application/json",
        success ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"Update failed\"}");
      if (success) {
        delay(1000);
        ESP.restart();
      }
    },
    // Upload handler (called per chunk)
    [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
      if (index == 0) {
        String typeParam = request->hasParam("type") ? request->getParam("type")->value() : "firmware";
        if (typeParam != "firmware" && typeParam != "filesystem") {
          Serial.printf("OTA rejected: unknown type '%s'\n", typeParam.c_str());
          return;
        }
        int updateType = (typeParam == "filesystem") ? U_SPIFFS : U_FLASH;
        size_t updateSize = (updateType == U_SPIFFS) ? 0xF0000 : ESP.getFreeSketchSpace();
        Serial.printf("OTA upload start: %s (%u bytes max)\n", typeParam.c_str(), updateSize);
        if (!Update.begin(updateSize, updateType)) {
          Update.printError(Serial);
        }
      }
      if (Update.isRunning()) {
        if (Update.write(data, len) != len) {
          Update.printError(Serial);
        }
      }
      if (final) {
        if (!Update.end(true)) {
          Update.printError(Serial);
        } else {
          Serial.printf("OTA upload complete: %u bytes\n", index + len);
        }
      }
    }
  );
}

// Minimal web server for AP mode (captive portal + setup)
void setupAPWebServer() {
  // Serve the captive portal setup page
  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", SETUP_PAGE);
  });

  // Return current config (for pre-filling the form)
  server.on("/api/setup", HTTP_GET, [](AsyncWebServerRequest* request) {
    char mqttSrv[65], mqttUser[33];
    configStore.getMqttServer(mqttSrv, sizeof(mqttSrv));
    configStore.getMqttUsername(mqttUser, sizeof(mqttUser));
    char buf[256];
    snprintf(buf, sizeof(buf),
      "{\"device_name\":\"%s\",\"mqtt_server\":\"%s\",\"mqtt_port\":%d,\"mqtt_user\":\"%s\"}",
      deviceName, mqttSrv, configStore.getMqttPort(), mqttUser);
    request->send(200, "application/json", buf);
  });

  // Save WiFi + MQTT credentials
  server.on("/api/setup", HTTP_POST, [](AsyncWebServerRequest* request) {},
    NULL,
    [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
      // Accumulate body (small JSON, single chunk expected)
      if (index + len > 512) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"Too large\"}");
        return;
      }
      if (index == 0 && (index + len) == total) {
        // Complete body in one chunk
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, (const char*)data, len);
        if (err) {
          request->send(400, "application/json", "{\"ok\":false,\"error\":\"Invalid JSON\"}");
          return;
        }
        const char* ssid = doc["ssid"] | "";
        const char* password = doc["password"] | "";
        if (strlen(ssid) == 0) {
          request->send(400, "application/json", "{\"ok\":false,\"error\":\"SSID required\"}");
          return;
        }
        configStore.setWifiCredentials(ssid, password);

        // MQTT config (optional)
        const char* mqttSrv = doc["mqtt_server"] | "homeassistant.local";
        int mqttPort = doc["mqtt_port"] | 1883;
        const char* mqttUser = doc["mqtt_user"] | "";
        const char* mqttPass = doc["mqtt_pass"] | "";
        configStore.setMqttConfig(mqttSrv, mqttPort, mqttUser, mqttPass);

        request->send(200, "application/json", "{\"ok\":true}");
        Serial.println("WiFi credentials saved. Restarting...");
        delay(1000);
        ESP.restart();
      }
    }
  );

  // WiFi network scan
  server.on("/api/scan", HTTP_GET, [](AsyncWebServerRequest* request) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_FAILED) {
      WiFi.scanNetworks(true);  // Start async scan
      request->send(200, "application/json", "[]");
      return;
    }
    if (n == WIFI_SCAN_RUNNING) {
      request->send(200, "application/json", "[]");
      return;
    }
    // Scan complete — build JSON response
    String json = "[";
    for (int i = 0; i < n; i++) {
      if (i > 0) json += ",";
      json += "{\"ssid\":\"";
      // Escape quotes in SSID
      String ssid = WiFi.SSID(i);
      ssid.replace("\"", "\\\"");
      json += ssid;
      json += "\",\"rssi\":";
      json += WiFi.RSSI(i);
      json += ",\"open\":";
      json += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "true" : "false";
      json += "}";
    }
    json += "]";
    WiFi.scanDelete();
    request->send(200, "application/json", json);
  });

  // OTA upload also available in AP mode
  registerOTAUploadHandler();

  // Captive portal redirects — redirect all unknown requests to /setup
  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/setup");
  });
  server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/setup");
  });
  server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/setup");
  });
  server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/setup");
  });
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/setup");
  });

  server.begin();
  Serial.println("AP web server started (captive portal)");
}
