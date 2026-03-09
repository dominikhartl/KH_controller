#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <DNSServer.h>

class WifiManager {
public:
  void begin(const char* ssid, const char* password);
  void beginAP(const char* apSSID);
  void loop();
  bool isConnected();
  bool isAPMode() const;
  int8_t getRSSI();

private:
  enum State { DISCONNECTED, CONNECTING, CONNECTED, AP_ACTIVE };
  State state = DISCONNECTED;
  const char* ssid = nullptr;
  const char* password = nullptr;
  unsigned long connectStartTime = 0;
  unsigned long lastReconnectAttempt = 0;

  DNSServer dnsServer;

  static const unsigned long CONNECT_TIMEOUT_MS = 15000;
  static const unsigned long RECONNECT_INTERVAL_MS = 30000;

  void startConnection();
};

extern WifiManager wifiManager;

#endif // WIFI_MANAGER_H
