#include "wifi_manager.h"
#include <ESPmDNS.h>
#include <esp_wifi.h>

extern char deviceName[];

WifiManager wifiManager;

void WifiManager::begin(const char* ssid, const char* password) {
  this->ssid = ssid;
  this->password = password;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Disable modem sleep — critical at weak RSSI (-79 dBm)
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Max transmit power
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);  // 802.11b only — better range at cost of speed (fine for small JSON payloads)
  startConnection();
}

void WifiManager::beginAP(const char* apSSID) {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(apSSID);
  Serial.print("AP mode started. SSID: ");
  Serial.println(apSSID);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // DNS server redirects all domains to our IP (captive portal)
  dnsServer.start(53, "*", IPAddress(192,168,4,1));
  state = AP_ACTIVE;
}

void WifiManager::startConnection() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  String hostname = String(deviceName);
  hostname.toLowerCase();
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(ssid, password);
  state = CONNECTING;
  connectStartTime = millis();
}

void WifiManager::loop() {
  switch (state) {
    case AP_ACTIVE:
      dnsServer.processNextRequest();
      break;

    case CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        state = CONNECTED;
        Serial.print("WiFi connected. IP: ");
        Serial.println(WiFi.localIP());

        // Start mDNS so device is reachable at <deviceName>.local
        String hostname = String(deviceName);
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

bool WifiManager::isAPMode() const {
  return state == AP_ACTIVE;
}

int8_t WifiManager::getRSSI() {
  if (isConnected()) {
    return WiFi.RSSI();
  }
  return 0;
}
