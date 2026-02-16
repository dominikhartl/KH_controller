#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>

class WifiManager {
public:
  void begin(const char* ssid, const char* password);
  void loop();
  bool isConnected();
  int8_t getRSSI();

private:
  enum State { DISCONNECTED, CONNECTING, CONNECTED };
  State state = DISCONNECTED;
  const char* ssid = nullptr;
  const char* password = nullptr;
  unsigned long connectStartTime = 0;
  unsigned long lastReconnectAttempt = 0;

  static const unsigned long CONNECT_TIMEOUT_MS = 15000;
  static const unsigned long RECONNECT_INTERVAL_MS = 30000;

  void startConnection();
};

extern WifiManager wifiManager;

#endif // WIFI_MANAGER_H
