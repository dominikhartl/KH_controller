#include <Arduino.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <config.h>
#include <ArduinoJson.h>
#include <pins.h>

#include "motors.h"
#include "stirrer.h"
#include "measurement.h"
#include "config_store.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "scheduler.h"
#include "ha_discovery.h"
#include "web_server.h"

// --- Global state ---
int drops = 0;
float startPH = 0;
bool discoveryPublished = false;
unsigned long lastDiagnosticsTime = 0;
unsigned long lastBroadcastTime = 0;

// MQTT topic buffers
char topicCmd[50];
char MQmsg[50];
char MQerr[50];
char MQKH[50];
char MQstartpH[50];
char MQmespH[50];
char MQkhValue[50];

// --- Publish helpers ---

void publishError(const char* errorMessage) {
  mqttManager.publish(MQerr, errorMessage);
  broadcastError(errorMessage);
}

void publishMessage(const char* message) {
  mqttManager.publish(MQmsg, message);
  broadcastMessage(message);
}

// --- HCl tracking helper ---
void subtractHCl(int dropsUsed) {
  float calDrops = (float)configStore.getCalDrops();
  if (calDrops <= 0) {
    publishError("Error: calDrops is zero, cannot track HCl!");
    return;
  }
  float titVol = configStore.getTitrationVolume();
  float hclVol = configStore.getHClVolume();
  float used = ((float)dropsUsed / calDrops) * titVol;
  float remaining = hclVol - used;
  if (remaining < 0) remaining = 0;
  configStore.setHClVolume(remaining);
}

// --- Pump calibration ---

void calibrateTitrationPump() {
  publishMessage("Calibrating pump");
  drops = 0;
  // Fast phase: 200-drop batches (matches measureKH fast phase)
  const int FAST_BATCH = 200;
  int fastTarget = CALIBRATION_TARGET_DROPS / 2;
  while (drops < fastTarget) {
    int batch = min(FAST_BATCH, fastTarget - drops);
    titrate(batch, TITRATION_SPEED);
    drops += batch;
    delay(TITRATION_MIX_DELAY_FAST_MS);
    mqttManager.loop();
    ArduinoOTA.handle();
    if (drops % 1000 == 0) {
      char dropsBuf[48];
      snprintf(dropsBuf, sizeof(dropsBuf), "Drops: %d / %d", drops, CALIBRATION_TARGET_DROPS);
      publishMessage(dropsBuf);
    }
  }
  // Slow phase: 2-drop steps (matches measureKH precise phase)
  while (drops < CALIBRATION_TARGET_DROPS) {
    titrate(TITRATION_STEP_SIZE, TITRATION_SPEED);
    delay(TITRATION_MIX_DELAY_MS);
    drops += TITRATION_STEP_SIZE;
    mqttManager.loop();
    if (drops % 1000 == 0) {
      char dropsBuf[48];
      snprintf(dropsBuf, sizeof(dropsBuf), "Drops: %d / %d", drops, CALIBRATION_TARGET_DROPS);
      publishMessage(dropsBuf);
    }
  }
  subtractHCl(CALIBRATION_TARGET_DROPS);
  drops = 0;
  digitalWrite(EN_PIN2, HIGH);
  publishMessage("Pump calibration done");
}

// --- MeasureKH ---

void measureKH() {
  // Re-entrancy guard: prevent concurrent measurements
  static bool measuring = false;
  if (measuring) {
    publishError("Measurement already in progress");
    return;
  }
  measuring = true;

  broadcastTitrationStart();  // Signal dashboard to clear live pH chart
  int errorflag = 0;
  drops = 0;
  const char* errorMessage = "";
  publishError("");  // Clear previous error

  // Validate calibration before starting
  if (!isCalibrationValid()) {
    publishError("Error: pH calibration invalid. Re-calibrate with pH 4/7/10 buffers.");
    measuring = false;
    return;
  }

  titrate(FILL_VOLUME, PREFILL_SPEED);
  digitalWrite(EN_PIN2, HIGH);
  publishMessage("Taking sample");
  washSample(1.2, 1.0);
  delay(100);
  startStirrer();
  delay(STIRRER_WARMUP_MS);  // Wait for solution to homogenize
  measurePH(100);
  if (isnan(pH)) {
    errorMessage = "Error: pH probe not working";
    stopStirrer();
    digitalWrite(EN_PIN2, HIGH);
    errorflag = 1;
  } else if (pH < ENDPOINT_PH + 1.0) {
    startPH = pH;
    errorMessage = "Error: Starting pH too low";
    stopStirrer();
    digitalWrite(EN_PIN2, HIGH);
    errorflag = 1;
  }

  if (errorflag == 0) {
    startPH = pH;
    mqttManager.loop();

    float fastPH = configStore.getFastTitrationPH();

    // --- Fast phase: pump 200-drop batches until pH drops below fastPH ---
    const int FAST_BATCH = 200;
    float lastFastPH = startPH;
    int stallCount = 0;
    publishMessage("Fast titration");

    while (pH > fastPH && drops < MAX_TITRATION_DROPS && errorflag == 0) {
      titrate(FAST_BATCH, TITRATION_SPEED);
      drops += FAST_BATCH;
      delay(TITRATION_MIX_DELAY_FAST_MS);
      measurePHFast(5);
      broadcastTitrationPH(pH, drops);
      mqttManager.loop();
      ArduinoOTA.handle();

      if (isnan(pH)) {
        errorMessage = "Error: pH probe not working!";
        errorflag = 1;
      } else if (pH > lastFastPH - 0.05) {
        stallCount++;
        if (stallCount >= 10) {
          errorMessage = "Error: insufficient pH change";
          errorflag = 1;
        }
      } else {
        stallCount = 0;
        lastFastPH = pH;
      }
    }

    // --- Precise phase: adaptive steps with endpoint interpolation ---
    float prevPH = pH;
    int prevDrops = drops;
    int lowcnt = 0;

    if (errorflag == 0) {
      char buf[32];
      snprintf(buf, sizeof(buf), "Precise phase (pH %.1f)", pH);
      publishMessage(buf);
    }

    while (!isnan(pH) && (pH > (ENDPOINT_PH - 0.1)) && (drops < MAX_TITRATION_DROPS) && (lowcnt < 3) && (errorflag == 0)) {
      // Track previous reading for endpoint interpolation
      prevPH = pH;
      prevDrops = drops;

      // Adaptive step size and measurement precision
      int stepVol;
      if (pH > (ENDPOINT_PH + 0.5)) {
        stepVol = TITRATION_STEP_SIZE * 2;  // 4 drops
        titrate(stepVol, TITRATION_SPEED);
        delay(TITRATION_MIX_DELAY_FAST_MS);
        measurePHFast(10);
      } else if (pH > ENDPOINT_PH) {
        stepVol = TITRATION_STEP_SIZE;      // 2 drops
        titrate(stepVol, TITRATION_SPEED);
        delay(TITRATION_MIX_DELAY_MS);
        measurePH(20);
      } else {
        // Below endpoint: confirmation with high precision
        lowcnt++;
        stepVol = TITRATION_STEP_SIZE;
        titrate(stepVol, TITRATION_SPEED);
        delay(TITRATION_MIX_DELAY_MS);
        measurePH(30);
      }
      drops += stepVol;

      mqttManager.loop();
      ArduinoOTA.handle();
      ws.cleanupClients();
      mqttManager.publish(MQmespH, String(pH).c_str());
      broadcastTitrationPH(pH, drops);

      if (isnan(pH)) {
        errorMessage = "Error: pH probe not working!";
        errorflag = 1;
      }
      if (drops >= MAX_TITRATION_DROPS - stepVol) {
        errorMessage = "Error: reached acid max!";
        errorflag = 1;
      }
    }

    mqttManager.loop();
    if (errorflag == 0) {
      // Interpolate exact endpoint crossing for better accuracy
      float exactDrops = (float)drops;
      if (prevPH > ENDPOINT_PH && pH <= ENDPOINT_PH && prevPH > pH) {
        float fraction = (prevPH - ENDPOINT_PH) / (prevPH - pH);
        exactDrops = (float)prevDrops + fraction * (float)(drops - prevDrops);
      }

      publishMessage(("Drops: " + String((int)exactDrops)).c_str());
      mqttManager.publish(MQKH, String((int)exactDrops).c_str());
      mqttManager.publish(MQstartpH, String(startPH).c_str());

      // Calculate KH on device
      float calDrops = (float)configStore.getCalDrops();
      float titVol = configStore.getTitrationVolume();
      float samVol = configStore.getSampleVolume();
      float corrF = configStore.getCorrectionFactor();
      float hclMol = configStore.getHClMolarity();

      if (calDrops <= 0 || samVol <= 0) {
        errorMessage = "Error: invalid calibration (calDrops or samVol is zero)";
        errorflag = 1;
        publishError(errorMessage);
      } else {
        float hclUsed = (exactDrops / calDrops) * titVol;
        float khValue = (hclUsed / samVol) * 2800.0f * hclMol * corrF;
        subtractHCl(drops + FILL_VOLUME); // titration drops + prefill

        // HCl low-level warning
        float remainingHCl = configStore.getHClVolume();
        if (remainingHCl <= 0) {
          publishError("Warning: HCl supply empty! Refill needed.");
        } else if (remainingHCl < HCL_LOW_THRESHOLD_ML) {
          char warnBuf[64];
          snprintf(warnBuf, sizeof(warnBuf), "Warning: HCl low (%.0f mL remaining)", remainingHCl);
          publishError(warnBuf);
        }

        mqttManager.publish(MQkhValue, String(khValue, 2).c_str(), true);

        // Save last measurement for persistent dashboard display
        configStore.setLastKH(khValue);
        configStore.setLastStartPH(startPH);

        // Store history for web dashboard charts
        appendHistory("kh", khValue);
        appendHistory("ph", startPH);
      }
    } else {
      publishError(errorMessage);
    }
  }
  digitalWrite(EN_PIN2, HIGH);
  stopStirrer();
  washSample(1.5, 1);
  if (errorflag == 0) {
    publishMessage("Done!");
  } else {
    publishMessage("Measurement finished with errors");
  }
  measuring = false;
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
  pinMode(EN_PIN1, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(EN_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STIRRER_PIN, OUTPUT);
  pinMode(PH_PIN, INPUT);
  Serial.begin(115200);

  // Initialize calibrated ADC
  initADC();

  // Initialize config store (NVS) and migrate from EEPROM if needed
  configStore.begin();

  // Keep MQTT/OTA alive during long motor operations (washSample takes ~16 min)
  setMotorYieldCallback([]() {
    mqttManager.loop();
    ArduinoOTA.handle();
    ws.cleanupClients();
  });

  // Report wash progress to web dashboard (WebSocket only, no MQTT during motor ops)
  setMotorProgressCallback([](int percent) {
    broadcastProgress(percent);
  });

  // Build MQTT topic strings
  snprintf(topicCmd, sizeof(topicCmd), "%s/cmd", DEVICE_NAME);
  snprintf(MQmsg, sizeof(MQmsg), "%s/message", DEVICE_NAME);
  snprintf(MQerr, sizeof(MQerr), "%s/error", DEVICE_NAME);
  snprintf(MQKH, sizeof(MQKH), "%s/KH", DEVICE_NAME);
  snprintf(MQstartpH, sizeof(MQstartpH), "%s/startPH", DEVICE_NAME);
  snprintf(MQmespH, sizeof(MQmespH), "%s/mes_pH", DEVICE_NAME);
  snprintf(MQkhValue, sizeof(MQkhValue), "%s/kh_value", DEVICE_NAME);

  // Non-blocking WiFi
  wifiManager.begin(WIFI_SSID, WIFI_PASSWORD);

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
  ArduinoOTA.setHostname(DEVICE_NAME);
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

  // Load pH calibration from NVS
  voltage_4PH = configStore.getVoltage4PH();
  voltage_7PH = configStore.getVoltage7PH();
  voltage_10PH = configStore.getVoltage10PH();

  // Compute linear fit from calibration values
  updateCalibrationFit();

  // Web server + WebSocket dashboard
  setupWebServer();

  // Scheduler with NTP
  scheduler.begin();
  scheduler.onMeasurementDue([]() {
    mqttManager.publish(MQmsg, "Scheduled measurement starting");
    measureKH();
  });
}

// --- Main loop ---

void loop() {
  wifiManager.loop();
  mqttManager.loop();
  ArduinoOTA.handle();
  scheduler.loop();
  ws.cleanupClients();

  // Publish HA Discovery on first MQTT connect or after HA restart
  if (mqttManager.isConnected() && !discoveryPublished) {
    publishAllDiscovery();
    publishAllConfigStates();

    // Re-publish last measurement values so HA has them immediately
    float lastKH = configStore.getLastKH();
    float lastSPH = configStore.getLastStartPH();
    if (lastKH > 0) mqttManager.publish(MQkhValue, String(lastKH, 2).c_str(), true);
    if (lastSPH > 0) mqttManager.publish(MQstartpH, String(lastSPH, 2).c_str(), true);

    discoveryPublished = true;
  }

  // Broadcast state to WebSocket clients every 2s
  if (millis() - lastBroadcastTime > 2000) {
    lastBroadcastTime = millis();
    broadcastState();
  }

  // Publish diagnostics every 60s
  if (mqttManager.isConnected() && millis() - lastDiagnosticsTime > 60000) {
    lastDiagnosticsTime = millis();
    publishDiagnostics();
  }

}
