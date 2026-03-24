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
  WiFi.setAutoReconnect(true);  // Let ESP-IDF handle transient drops automatically
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);  // 802.11b only — better range at cost of speed (fine for small JSON payloads)

  // Register event handlers for immediate state transitions (faster than polling)
  WiFi.onEvent(onWiFiEvent);

  startConnection();
}

void WifiManager::onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      if (wifiManager.state == CONNECTED) {
        Serial.println("WiFi event: disconnected");
      }
      if (wifiManager.state != AP_ACTIVE && wifiManager.state != CONNECTING) {
        wifiManager.state = DISCONNECTED;
        if (wifiManager.disconnectedSince == 0) {
          wifiManager.disconnectedSince = millis();
        }
        // Shorten wait so manual reconnect fires quickly if auto-reconnect fails
        wifiManager.lastReconnectAttempt = millis() - RECONNECT_INTERVAL_MS + 2000;
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("WiFi event: got IP ");
      Serial.println(WiFi.localIP());
      wifiManager.state = CONNECTED;
      wifiManager.disconnectedSince = 0;
      wifiManager.startMDNS();
      break;

    default:
      break;
  }
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

void WifiManager::startMDNS() {
  String hostname = String(deviceName);
  hostname.toLowerCase();
  if (MDNS.begin(hostname.c_str())) {
    Serial.print("mDNS started: ");
    Serial.print(hostname);
    Serial.println(".local");
    MDNS.addService("http", "tcp", 80);
  }
}

void WifiManager::loop() {
  switch (state) {
    case AP_ACTIVE:
      dnsServer.processNextRequest();
      break;

    case CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        state = CONNECTED;
        disconnectedSince = 0;
        Serial.print("WiFi connected. IP: ");
        Serial.println(WiFi.localIP());
        startMDNS();
      } else if (millis() - connectStartTime > CONNECT_TIMEOUT_MS) {
        Serial.println("WiFi connection timeout - continuing offline");
        state = DISCONNECTED;
        lastReconnectAttempt = millis();
        if (disconnectedSince == 0) disconnectedSince = millis();
      }
      break;

    case CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected (poll)");
        state = DISCONNECTED;
        lastReconnectAttempt = millis();
        if (disconnectedSince == 0) disconnectedSince = millis();
      }
      break;

    case DISCONNECTED:
      // WiFi watchdog: force full stack reset after prolonged disconnect
      if (disconnectedSince > 0 && millis() - disconnectedSince > WIFI_WATCHDOG_MS) {
        Serial.println("WiFi watchdog: forcing full stack reset");
        WiFi.disconnect(true);
        delay(100);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        WiFi.setAutoReconnect(true);
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
        disconnectedSince = millis();  // Reset watchdog timer
        startConnection();
        break;
      }
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
