#include "temperature.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "pins.h"
#include "config.h"

extern void publishMessage(const char* message);

static OneWire* oneWire = nullptr;
static DallasTemperature* sensors = nullptr;
static bool sensorFound = false;

void initTemperature() {
  oneWire = new OneWire(TEMP_SENSOR_PIN);
  sensors = new DallasTemperature(oneWire);
  sensors->begin();
  sensorFound = sensors->getDeviceCount() > 0;
  if (sensorFound) {
    sensors->setResolution(12);  // 0.0625°C precision
    sensors->requestTemperatures();
    float t = sensors->getTempCByIndex(0);
    Serial.printf("DS18B20 temperature sensor found: %.2f °C (12-bit)\n", t);
    publishMessage("Temperature sensor detected");
  } else {
    Serial.printf("No temperature sensor found, using default %.1f °C\n",
                  DEFAULT_MEASUREMENT_TEMP_C);
    publishMessage("No temperature sensor, using default 21°C");
  }
}

float getWaterTemperatureC() {
  if (!sensorFound || !sensors) return DEFAULT_MEASUREMENT_TEMP_C;
  sensors->requestTemperatures();
  float t = sensors->getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) return DEFAULT_MEASUREMENT_TEMP_C;
  return t;
}

bool hasTemperatureSensor() {
  return sensorFound;
}
