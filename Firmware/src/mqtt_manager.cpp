#include "mqtt_manager.h"
#include <config.h>
#include <WiFi.h>

MQTTManager mqttManager;

// MQTT topic buffers
char topicAvailability[50];

void MQTTManager::begin() {
  client.setClient(espClient);
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(768);

  // Build fixed client ID from MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(clientId, sizeof(clientId), "KHv3-%02X%02X%02X", mac[3], mac[4], mac[5]);

  snprintf(topicAvailability, sizeof(topicAvailability), "%s/availability", DEVICE_NAME);
}

void MQTTManager::loop() {
  if (!wifiManager.isConnected()) {
    if (wasConnected) {
      wasConnected = false;
      if (onDisconnectCb) onDisconnectCb();
    }
    return;
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
  if (now - lastReconnectAttempt < currentReconnectInterval) {
    return;
  }
  lastReconnectAttempt = now;

  if (tryConnect()) {
    Serial.println("MQTT connected");
    wasConnected = true;
    lastHeartbeat = millis();

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
    // Exponential backoff
    currentReconnectInterval = min(currentReconnectInterval * 2, MAX_RECONNECT_INTERVAL_MS);
  }
}

bool MQTTManager::tryConnect() {
  // Connect with LWT (Last Will and Testament) using fixed client ID
  return client.connect(
    clientId,
    mqtt_username,
    mqtt_password,
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
  return client.publish(topic, payload, retained);
}

void MQTTManager::subscribe(const char* topic) {
  // Track for re-subscription after reconnect
  if (subscriptionCount < MAX_SUBSCRIPTIONS) {
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
