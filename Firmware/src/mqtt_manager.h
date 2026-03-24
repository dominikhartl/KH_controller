#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <PubSubClient.h>
#include "wifi_manager.h"

typedef void (*MQTTDisconnectCallback)();

class MQTTManager {
public:
  void begin();
  void loop();
  bool isConnected();
  bool publish(const char* topic, const char* payload, bool retained = false);
  void subscribe(const char* topic);
  void setCallback(MQTT_CALLBACK_SIGNATURE);
  void onDisconnect(MQTTDisconnectCallback cb) { onDisconnectCb = cb; }
  // Motor mode: allows reconnect attempts at reduced frequency (instead of full suppression)
  void setMotorMode(bool active) {
    motorMode = active;
    if (!active) {
      motorReconnectFails = 0;
      // Reset backoff so MQTT reconnects immediately after motor ops
      lastReconnectAttempt = 0;
      currentReconnectInterval = RECONNECT_INTERVAL_MS;
    }
  }
  PubSubClient& getClient();

private:
  WiFiClient espClient;
  PubSubClient client;
  bool needsResubscribe = false;
  bool wasConnected = false;
  bool wifiWasDown = false;
  unsigned long lastReconnectAttempt = 0;
  unsigned long lastHeartbeat = 0;
  char clientId[20] = {};

  static const unsigned long RECONNECT_INTERVAL_MS = 5000;
  static const unsigned long MAX_RECONNECT_INTERVAL_MS = 60000;
  static const unsigned long MOTOR_RECONNECT_INTERVAL_MS = 10000;  // Slower retries during motor ops
  static const uint8_t MOTOR_MAX_FAILS = 3;  // Stop trying after 3 failures during motor ops
  unsigned long currentReconnectInterval = RECONNECT_INTERVAL_MS;

  MQTTDisconnectCallback onDisconnectCb = nullptr;

  // Track subscriptions for re-subscribing after reconnect
  static const uint8_t MAX_SUBSCRIPTIONS = 32;
  const char* subscriptions[MAX_SUBSCRIPTIONS] = {};
  uint8_t subscriptionCount = 0;

  bool motorMode = false;
  uint8_t motorReconnectFails = 0;
  bool tryConnect();
};

extern MQTTManager mqttManager;

#endif // MQTT_MANAGER_H
