#include <Arduino.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include <config.h>
#include <ArduinoJson.h>
#include <pins.h>
#include <time.h>

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
int units = 0;
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
void subtractHCl(int unitsUsed) {
  float calUnits = (float)configStore.getCalUnits();
  if (calUnits <= 0) {
    publishError("Error: calUnits is zero, cannot track HCl!");
    return;
  }
  float titVol = configStore.getTitrationVolume();
  float hclVol = configStore.getHClVolume();
  float used = ((float)unitsUsed / calUnits) * titVol;
  float remaining = hclVol - used;
  if (remaining < 0) remaining = 0;
  configStore.setHClVolume(remaining);
}

// --- Pump calibration ---

void calibrateTitrationPump() {
  publishMessage("Calibrating pump");
  units = 0;
  // Pump CALIBRATION_TARGET_UNITS in 200-unit batches
  // Peristaltic pump delivers fixed volume per revolution regardless of speed
  const int BATCH = 200;
  while (units < CALIBRATION_TARGET_UNITS) {
    int batch = min(BATCH, CALIBRATION_TARGET_UNITS - units);
    titrate(batch, TITRATION_SPEED);
    units += batch;
    delay(TITRATION_MIX_DELAY_FAST_MS);
    mqttManager.loop();
    ArduinoOTA.handle();
    ws.cleanupClients();
    if (units % 1000 == 0) {
      char buf[48];
      snprintf(buf, sizeof(buf), "Units: %d / %d", units, CALIBRATION_TARGET_UNITS);
      publishMessage(buf);
    }
  }
  subtractHCl(CALIBRATION_TARGET_UNITS);
  units = 0;
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
  units = 0;
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
  } else if (pH < 7.0) {
    startPH = pH;
    errorMessage = "Error: Starting pH too low (possible carryover)";
    stopStirrer();
    digitalWrite(EN_PIN2, HIGH);
    errorflag = 1;
  }

  if (errorflag == 0) {
    startPH = pH;
    mqttManager.loop();

    // Check probe health and calibration age
    char reason[96];
    const char* health = getProbeHealthDetail(reason, sizeof(reason));
    if (strcmp(health, "Good") != 0) {
      char hBuf[128];
      snprintf(hBuf, sizeof(hBuf), "Warning: Probe %s — %s", health, reason);
      publishError(hBuf);
    }
    uint32_t calTs = configStore.getCalTimestamp();
    time_t now = time(nullptr);
    if (calTs > 0 && now > 1000000000) {
      int calAgeDays = (int)((now - calTs) / 86400);
      if (calAgeDays > CALIBRATION_AGE_WARNING_DAYS) {
        char aBuf[64];
        snprintf(aBuf, sizeof(aBuf), "Warning: Calibration is %d days old", calAgeDays);
        publishError(aBuf);
      }
    }

    float fastPH = configStore.getFastTitrationPH();

    // --- Fast phase: pump 200-unit batches until pH falls below fastPH ---
    const int FAST_BATCH = 200;
    float lastFastPH = startPH;
    int stallCount = 0;
    publishMessage("Fast titration");

    while (pH > fastPH && units < MAX_TITRATION_UNITS && errorflag == 0) {
      titrate(FAST_BATCH, TITRATION_SPEED);
      units += FAST_BATCH;
      delay(TITRATION_MIX_DELAY_FAST_MS);
      measurePHFast(5);
      broadcastTitrationPH(pH, units);
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

    // --- Precise phase: adaptive steps with Gran transformation ---
    static TitrationPoint dataPoints[MAX_TITRATION_POINTS];
    int nPoints = 0;
    int granCount = 0;

    if (errorflag == 0) {
      char buf[32];
      snprintf(buf, sizeof(buf), "Precise phase (pH %.1f)", pH);
      publishMessage(buf);
    }

    // Store data points only near the endpoint (pH < 5.0) to avoid filling the buffer
    // with useless high-pH coarse-step data
    static const float DATA_STORE_PH = 5.0f;

    while (!isnan(pH) && pH > GRAN_STOP_PH && units < MAX_TITRATION_UNITS
           && granCount < MAX_GRAN_POINTS && errorflag == 0) {
      int stepVol;
      if (pH > DATA_STORE_PH) {
        // Approach: large steps, just get near the endpoint
        stepVol = TITRATION_STEP_SIZE * 10; // 20 units
        titrate(stepVol, TITRATION_SPEED);
        delay(TITRATION_MIX_DELAY_FAST_MS);
        measurePHFast(3);
      } else if (pH > GRAN_REGION_PH) {
        // Near endpoint: medium steps, fast measurement, store for interpolation
        stepVol = TITRATION_STEP_SIZE * 4;  // 8 units
        titrate(stepVol, TITRATION_SPEED);
        delay(TITRATION_MIX_DELAY_FAST_MS);
        measurePHFast(5);
      } else {
        // Gran region: fine steps, precise measurement (stabilization handles equilibration)
        stepVol = TITRATION_STEP_SIZE;      // 2 units
        titrate(stepVol, TITRATION_SPEED);
        delay(TITRATION_MIX_DELAY_FAST_MS);
        measurePH(10);
      }
      units += stepVol;

      // Store data points near the endpoint for Gran analysis and interpolation
      if (pH < DATA_STORE_PH && nPoints < MAX_TITRATION_POINTS) {
        dataPoints[nPoints++] = {(float)units, pH};
      }
      if (pH < GRAN_REGION_PH) granCount++;

      mqttManager.loop();
      ArduinoOTA.handle();
      ws.cleanupClients();
      mqttManager.publish(MQmespH, String(pH).c_str());
      broadcastTitrationPH(pH, units);

      if (isnan(pH)) {
        errorMessage = "Error: pH probe not working!";
        errorflag = 1;
      }
      if (units >= MAX_TITRATION_UNITS - stepVol) {
        errorMessage = "Error: reached acid max!";
        errorflag = 1;
      }
    }

    mqttManager.loop();
    if (errorflag == 0) {
      // Get calibration parameters
      float calUnits = (float)configStore.getCalUnits();
      float titVol = configStore.getTitrationVolume();
      float samVol = configStore.getSampleVolume();
      float corrF = configStore.getCorrectionFactor();
      float hclMol = configStore.getHClMolarity();

      if (calUnits <= 0 || samVol <= 0) {
        errorMessage = "Error: invalid calibration (calUnits or samVol is zero)";
        errorflag = 1;
        publishError(errorMessage);
      } else {
        // Determine equivalence point via Gran transformation
        float granR2 = 0;
        float exactUnits;
        bool usedGran = false;

        if (granCount >= MIN_GRAN_POINTS) {
          exactUnits = granAnalysis(dataPoints, nPoints, samVol, titVol, calUnits, &granR2);
          if (!isnan(exactUnits)) {
            usedGran = true;
          } else {
            exactUnits = interpolateAtPH(dataPoints, nPoints, ENDPOINT_PH);
            publishMessage("Gran analysis failed, using interpolation");
          }
        } else {
          exactUnits = interpolateAtPH(dataPoints, nPoints, ENDPOINT_PH);
          publishMessage("Insufficient Gran data, using interpolation");
        }

        if (isnan(exactUnits)) {
          errorMessage = "Error: could not determine equivalence point";
          errorflag = 1;
          publishError(errorMessage);
        } else {
          float hclUsed = (exactUnits / calUnits) * titVol;
          float khValue = (hclUsed / samVol) * 2800.0f * hclMol * corrF;
          subtractHCl(units + FILL_VOLUME);

          // Publish volume and endpoint info
          char volBuf[64];
          if (usedGran) {
            snprintf(volBuf, sizeof(volBuf), "Endpoint: %.2f mL (Gran, R²=%.3f)", hclUsed, granR2);
          } else {
            snprintf(volBuf, sizeof(volBuf), "Endpoint: %.2f mL (interpolation at pH %.1f)", hclUsed, ENDPOINT_PH);
          }
          publishMessage(volBuf);
          mqttManager.publish(MQKH, String(hclUsed, 2).c_str());
          mqttManager.publish(MQstartpH, String(startPH).c_str());

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
      }
    } else {
      publishError(errorMessage);
    }
  }
  // Anti-suckback: small reverse to prevent drip from titration nozzle
  digitalWrite(DIR_PIN2, HIGH);  // Reverse
  for (int i = 0; i < ANTI_SUCKBACK_STEPS; i++) {
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds((unsigned int)MOTOR_START_SPEED);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds((unsigned int)MOTOR_START_SPEED);
  }
  digitalWrite(DIR_PIN2, LOW);   // Restore forward
  digitalWrite(EN_PIN2, HIGH);

  stopStirrer();

  // Double rinse to eliminate acid carryover
  washSample(1.5, 1.0);
  delay(2000);
  washSample(1.2, 1.0);

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
