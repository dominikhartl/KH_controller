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
  volatile State state = DISCONNECTED;  // volatile: written from WiFi event task
  const char* ssid = nullptr;
  const char* password = nullptr;
  unsigned long connectStartTime = 0;
  unsigned long lastReconnectAttempt = 0;
  unsigned long disconnectedSince = 0;  // For WiFi watchdog (full stack reset)

  DNSServer dnsServer;

  static const unsigned long CONNECT_TIMEOUT_MS = 15000;
  static const unsigned long RECONNECT_INTERVAL_MS = 5000;    // Was 30s — too slow for recovery
  static const unsigned long WIFI_WATCHDOG_MS = 120000;       // Force full stack reset after 2 min

  void startConnection();
  void startMDNS();
  static void onWiFiEvent(WiFiEvent_t event);
};

extern WifiManager wifiManager;

#endif // WIFI_MANAGER_H
