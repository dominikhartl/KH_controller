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
  PubSubClient& getClient();

private:
  WiFiClient espClient;
  PubSubClient client;
  bool needsResubscribe = false;
  bool wasConnected = false;
  unsigned long lastReconnectAttempt = 0;
  unsigned long lastHeartbeat = 0;
  char clientId[20] = {};

  static const unsigned long RECONNECT_INTERVAL_MS = 5000;
  static const unsigned long MAX_RECONNECT_INTERVAL_MS = 60000;
  unsigned long currentReconnectInterval = RECONNECT_INTERVAL_MS;

  MQTTDisconnectCallback onDisconnectCb = nullptr;

  // Track subscriptions for re-subscribing after reconnect
  static const uint8_t MAX_SUBSCRIPTIONS = 24;
  const char* subscriptions[MAX_SUBSCRIPTIONS];
  uint8_t subscriptionCount = 0;

  bool tryConnect();
};

extern MQTTManager mqttManager;

#endif // MQTT_MANAGER_H
