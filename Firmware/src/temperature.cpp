#include "temperature.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "pins.h"
#include "config.h"

extern void publishMessage(const char* message);

static OneWire* oneWire = nullptr;
static DallasTemperature* sensors = nullptr;
static bool sensorFound = false;
static constexpr float SENSOR_ERROR_FALLBACK_C = 21.0f;

void initTemperature() {
  oneWire = new OneWire(TEMP_SENSOR_PIN);
  if (!oneWire) { Serial.println("FATAL: OneWire alloc failed"); return; }
  sensors = new DallasTemperature(oneWire);
  if (!sensors) { Serial.println("FATAL: DallasTemp alloc failed"); return; }
  constexpr int kBootProbeAttempts = 5;
  constexpr unsigned long kBootProbeDelayMs = 200;
  for (int attempt = 1; attempt <= kBootProbeAttempts; ++attempt) {
    sensors->begin();
    if (sensors->getDeviceCount() > 0) {
      sensorFound = true;
      if (attempt > 1) {
        Serial.printf("DS18B20 detected on attempt %d/%d\n", attempt, kBootProbeAttempts);
      }
      break;
    }
    if (attempt < kBootProbeAttempts) {
      Serial.printf("DS18B20 not found on attempt %d/%d, retrying...\n", attempt, kBootProbeAttempts);
      delay(kBootProbeDelayMs);
    }
  }
  if (sensorFound) {
    sensors->setResolution(12);  // 0.0625°C precision
    sensors->setWaitForConversion(true);
    sensors->requestTemperatures();
    float t = sensors->getTempCByIndex(0);
    // 85.0°C is the DS18B20 power-on reset default — discard it
    if (t == 85.0f) {
      delay(800);  // 12-bit conversion takes ~750ms
      sensors->requestTemperatures();
      t = sensors->getTempCByIndex(0);
    }
    Serial.printf("DS18B20 temperature sensor found: %.2f °C (12-bit)\n", t);
    publishMessage("Temperature sensor detected");
  } else {
    Serial.printf("No temperature sensor found, using default %.1f °C\n",
                  DEFAULT_MEASUREMENT_TEMP_C);
    publishMessage("No temperature sensor, using default 21°C");
  }
}

float getWaterTemperatureC() {
  if (!sensors) return SENSOR_ERROR_FALLBACK_C;
  if (!sensorFound) {
    static unsigned long lastReprobeMs = 0;
    unsigned long now = millis();
    if (lastReprobeMs == 0 || now - lastReprobeMs >= 30000UL) {
      lastReprobeMs = now;
      sensors->begin();
      if (sensors->getDeviceCount() > 0) {
        sensorFound = true;
        sensors->setResolution(12);
        sensors->setWaitForConversion(true);
        Serial.println("DS18B20 temperature sensor detected (recovered)");
        publishMessage("Temperature sensor detected (recovered)");
      }
    }
    if (!sensorFound) return SENSOR_ERROR_FALLBACK_C;
  }
  sensors->requestTemperatures();
  float t = sensors->getTempCByIndex(0);
  if (t <= DEVICE_DISCONNECTED_C + 1.0f || t == 85.0f) return SENSOR_ERROR_FALLBACK_C;
  if (t < 18.0f || t > 27.0f) {
    char buf[80];
    snprintf(buf, sizeof(buf), "WARNING: Water temperature %.1f °C outside 18-27 °C range, clamping", t);
    publishMessage(buf);
    if (t < 18.0f) return 18.0f;
    return 27.0f;
  }
  return t;
}

bool hasTemperatureSensor() {
  return sensorFound;
}
