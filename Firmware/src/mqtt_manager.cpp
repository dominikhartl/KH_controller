#include "mqtt_manager.h"
#include "config_store.h"
#include <config.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

extern char deviceName[];

MQTTManager mqttManager;

// MQTT topic buffers
char topicAvailability[50];

// Static buffers for MQTT credentials (PubSubClient stores pointers, not copies)
static char mqttServerBuf[65];
static char mqttUserBuf[33];
static char mqttPassBuf[33];

void MQTTManager::begin() {
  client.setClient(espClient);

  // Load MQTT config from NVS
  configStore.getMqttServer(mqttServerBuf, sizeof(mqttServerBuf));
  int mqttPort = configStore.getMqttPort();
  client.setServer(mqttServerBuf, mqttPort);
  client.setBufferSize(768);
  client.setSocketTimeout(2);  // 2s TCP timeout (default 15s can trigger watchdog)

  // Build fixed client ID from MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(clientId, sizeof(clientId), "KHv3-%02X%02X%02X", mac[3], mac[4], mac[5]);

  snprintf(topicAvailability, sizeof(topicAvailability), "%s/availability", deviceName);
}

void MQTTManager::loop() {
  if (!wifiManager.isConnected()) {
    if (wasConnected) {
      wasConnected = false;
      if (onDisconnectCb) onDisconnectCb();
    }
    wifiWasDown = true;
    return;
  }

  // Reset backoff when WiFi comes back (so MQTT reconnects quickly after brief dropout)
  if (wifiWasDown) {
    wifiWasDown = false;
    currentReconnectInterval = RECONNECT_INTERVAL_MS;
    lastReconnectAttempt = 0;
  }

  if (client.connected()) {
    client.loop();
    currentReconnectInterval = RECONNECT_INTERVAL_MS; // Reset backoff on success

    // Periodic availability heartbeat every 25s (under 30s keepalive)
    // Also serves as connection health check - if publish fails, next loop detects disconnect
    unsigned long now = millis();
    if (now - lastHeartbeat > 25000) {
      lastHeartbeat = now;
      client.publish(topicAvailability, "online", true);
    }
    return;
  }

  // Detect transition from connected to disconnected
  if (wasConnected) {
    wasConnected = false;
    Serial.println("MQTT disconnected");
    if (onDisconnectCb) onDisconnectCb();
  }

  // Non-blocking reconnect with exponential backoff
  unsigned long now = millis();
  unsigned long interval = motorMode ? MOTOR_RECONNECT_INTERVAL_MS : currentReconnectInterval;
  if (now - lastReconnectAttempt < interval) {
    return;
  }
  // In motor mode: allow reconnect at reduced frequency, but give up after repeated failures
  if (motorMode && motorReconnectFails >= MOTOR_MAX_FAILS) return;
  lastReconnectAttempt = now;

  // Use shorter socket timeout during motor ops to limit blocking
  if (motorMode) client.setSocketTimeout(1);
  esp_task_wdt_reset();  // Feed watchdog before potentially blocking TCP connect

  if (tryConnect()) {
    Serial.println("MQTT connected");
    wasConnected = true;
    lastHeartbeat = millis();
    motorReconnectFails = 0;

    // Publish online availability
    client.publish(topicAvailability, "online", true);

    // Re-subscribe to all tracked topics
    for (uint8_t i = 0; i < subscriptionCount; i++) {
      client.subscribe(subscriptions[i]);
    }

    needsResubscribe = false;
    currentReconnectInterval = RECONNECT_INTERVAL_MS;
  } else {
    Serial.print("MQTT connect failed, state: ");
    Serial.println(client.state());
    if (motorMode) {
      motorReconnectFails++;
    }
    // Exponential backoff (only in normal mode; motor mode uses fixed interval)
    if (!motorMode) {
      currentReconnectInterval = min(currentReconnectInterval * 2, MAX_RECONNECT_INTERVAL_MS);
    }
  }

  // Restore normal socket timeout after motor-mode attempt
  if (motorMode) client.setSocketTimeout(2);
}

bool MQTTManager::tryConnect() {
  // Load MQTT credentials from NVS (refreshed on each connect attempt)
  configStore.getMqttUsername(mqttUserBuf, sizeof(mqttUserBuf));
  configStore.getMqttPassword(mqttPassBuf, sizeof(mqttPassBuf));

  // Connect with LWT (Last Will and Testament) using fixed client ID
  return client.connect(
    clientId,
    mqttUserBuf,
    mqttPassBuf,
    topicAvailability,  // LWT topic
    0,                  // LWT QoS
    true,               // LWT retain
    "offline"           // LWT payload
  );
}

bool MQTTManager::isConnected() {
  return client.connected();
}

bool MQTTManager::publish(const char* topic, const char* payload, bool retained) {
  if (!client.connected()) {
    return false;
  }
  bool ok = client.publish(topic, payload, retained);
  if (!ok) {
    Serial.printf("MQTT publish failed: %s\n", topic);
  }
  return ok;
}

void MQTTManager::subscribe(const char* topic) {
  // Track for re-subscription after reconnect (dedup by pointer)
  bool found = false;
  for (uint8_t i = 0; i < subscriptionCount; i++) {
    if (subscriptions[i] == topic) { found = true; break; }
  }
  if (!found && subscriptionCount < MAX_SUBSCRIPTIONS) {
    subscriptions[subscriptionCount++] = topic;
  }
  if (client.connected()) {
    client.subscribe(topic);
  }
}

void MQTTManager::setCallback(MQTT_CALLBACK_SIGNATURE) {
  client.setCallback(callback);
}

PubSubClient& MQTTManager::getClient() {
  return client;
}
