#include "wifi_manager.h"
#include <ESPmDNS.h>
#include <config.h>

WifiManager wifiManager;

void WifiManager::begin(const char* ssid, const char* password) {
  this->ssid = ssid;
  this->password = password;
  WiFi.mode(WIFI_STA);
  startConnection();
}

void WifiManager::startConnection() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  state = CONNECTING;
  connectStartTime = millis();
}

void WifiManager::loop() {
  switch (state) {
    case CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        state = CONNECTED;
        Serial.print("WiFi connected. IP: ");
        Serial.println(WiFi.localIP());

        // Start mDNS so device is reachable at khcontrollerv3.local
        String hostname = String(DEVICE_NAME);
        hostname.toLowerCase();
        if (MDNS.begin(hostname.c_str())) {
          Serial.print("mDNS started: ");
          Serial.print(hostname);
          Serial.println(".local");
          MDNS.addService("http", "tcp", 80);
        }
      } else if (millis() - connectStartTime > CONNECT_TIMEOUT_MS) {
        Serial.println("WiFi connection timeout - continuing offline");
        state = DISCONNECTED;
        lastReconnectAttempt = millis();
      }
      break;

    case CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected");
        state = DISCONNECTED;
        lastReconnectAttempt = millis();
      }
      break;

    case DISCONNECTED:
      if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
        startConnection();
      }
      break;
  }
}

bool WifiManager::isConnected() {
  return state == CONNECTED && WiFi.status() == WL_CONNECTED;
}

int8_t WifiManager::getRSSI() {
  if (isConnected()) {
    return WiFi.RSSI();
  }
  return 0;
}
