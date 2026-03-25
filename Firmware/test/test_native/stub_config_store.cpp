#include "config_store.h"
#include <cstring>
#include <cstdio>
#include <cmath>

ConfigStore configStore;

// --- Test-controllable values ---
static float _granMinR2 = 0.995f;
static float _bufPH4 = 4.0f;
static float _bufPH7 = 7.0f;
static float _bufPH10 = 10.0f;
static uint8_t _intervalHours = 6;
static uint16_t _anchorTime = 0;
static uint8_t _schedMode = 1;
static uint8_t _schedCount = 4;
static uint16_t _schedTimes[8] = {120, 480, 840, 1200, 0, 0, 0, 0};

// --- Test setter functions (declared in test_helpers.h or extern'd in tests) ---
extern "C" {
  void testSetGranMinR2(float v) { _granMinR2 = v; }
  void testSetBufferPH4(float v) { _bufPH4 = v; }
  void testSetBufferPH7(float v) { _bufPH7 = v; }
  void testSetBufferPH10(float v) { _bufPH10 = v; }
  void testSetIntervalHours(uint8_t h) { _intervalHours = h; }
  void testSetAnchorTime(uint16_t m) { _anchorTime = m; }
  void testSetScheduleMode(uint8_t m) { _schedMode = m; }
  void testSetScheduleCount(uint8_t c) { _schedCount = c; }
  void testSetScheduleTime(uint8_t idx, uint16_t mins) { if (idx < 8) _schedTimes[idx] = mins; }

  void testResetConfig() {
    _granMinR2 = 0.995f;
    _bufPH4 = 4.0f;
    _bufPH7 = 7.0f;
    _bufPH10 = 10.0f;
    _intervalHours = 6;
    _anchorTime = 0;
    _schedMode = 1;
    _schedCount = 4;
    uint16_t defaults[] = {120, 480, 840, 1200, 0, 0, 0, 0};
    memcpy(_schedTimes, defaults, sizeof(_schedTimes));
  }
}

// --- ConfigStore method implementations ---

void ConfigStore::begin() {}

// Gran analysis
float ConfigStore::getGranMinR2() { return _granMinR2; }

// Buffer pH values
float ConfigStore::getBufferPH4() { return _bufPH4; }
float ConfigStore::getBufferPH7() { return _bufPH7; }
float ConfigStore::getBufferPH10() { return _bufPH10; }

// Scheduler
uint8_t ConfigStore::getIntervalHours() { return _intervalHours; }
uint16_t ConfigStore::getAnchorTime() { return _anchorTime; }
uint8_t ConfigStore::getScheduleMode() { return _schedMode; }
uint8_t ConfigStore::getScheduleCount() { return _schedCount; }
uint16_t ConfigStore::getScheduleTime(uint8_t i) { return (i < 8) ? _schedTimes[i] : 0; }
const char* ConfigStore::getTimezone() { return "CET-1CEST,M3.5.0/2,M10.5.0/3"; }

// --- Stubs for methods declared in config_store.h but not used by tested code ---
// These exist only to satisfy the linker if any code path references them.

void ConfigStore::getDeviceName(char* buf, size_t len) { if (buf && len > 0) snprintf(buf, len, "KHpro"); }
void ConfigStore::setDeviceName(const char*) {}
float ConfigStore::getVoltage4PH() { return 2032.0f; }
float ConfigStore::getVoltage7PH() { return 1500.0f; }
float ConfigStore::getVoltage10PH() { return 984.0f; }
void ConfigStore::setVoltage4PH(float) {}
void ConfigStore::setVoltage7PH(float) {}
void ConfigStore::setVoltage10PH(float) {}
float ConfigStore::getTitrationVolume() { return 10.0f; }
float ConfigStore::getSampleVolume() { return 80.0f; }
float ConfigStore::getCorrectionFactor() { return 1.0f; }
float ConfigStore::getHClMolarity() { return 0.02f; }
float ConfigStore::getHClVolume() { return 5000.0f; }
int ConfigStore::getCalUnits() { return 6000; }
float ConfigStore::getFastTitrationPH() { return 5.0f; }
uint8_t ConfigStore::getEndpointMethod() { return 0; }
float ConfigStore::getMinStartPH() { return 7.5f; }
int ConfigStore::getStabilizationTimeout() { return 2000; }
int ConfigStore::getGranMixDelay() { return 2500; }
void ConfigStore::setTitrationVolume(float) {}
void ConfigStore::setSampleVolume(float) {}
void ConfigStore::setCorrectionFactor(float) {}
void ConfigStore::setHClMolarity(float) {}
void ConfigStore::setHClVolume(float) {}
void ConfigStore::setCalUnits(int) {}
void ConfigStore::setFastTitrationPH(float) {}
void ConfigStore::setEndpointMethod(uint8_t) {}
void ConfigStore::setMinStartPH(float) {}
void ConfigStore::setStabilizationTimeout(int) {}
void ConfigStore::setGranMixDelay(int) {}
void ConfigStore::setGranMinR2(float v) { _granMinR2 = v; }
float ConfigStore::getDropVolumeUL() { return 26.0f; }
void ConfigStore::setDropVolumeUL(float) {}
float ConfigStore::getTitrationRPM() { return 47.0f; }
void ConfigStore::setTitrationRPM(float) {}
float ConfigStore::getGranBurstRPM() { return 250.0f; }
void ConfigStore::setGranBurstRPM(float) {}
uint32_t ConfigStore::getGranBurstAccel() { return 200000; }
void ConfigStore::setGranBurstAccel(uint32_t) {}
float ConfigStore::getFastPhaseRPM() { return 50.0f; }
void ConfigStore::setFastPhaseRPM(float) {}
float ConfigStore::getPrefillVolumeUL() { return 100.0f; }
void ConfigStore::setPrefillVolumeUL(float) {}
float ConfigStore::getMaxAcidML() { return 16.0f; }
void ConfigStore::setMaxAcidML(float) {}
int ConfigStore::getFastStepUL() { return 330; }
void ConfigStore::setFastStepUL(int) {}
float ConfigStore::getMeasTempC() { return 21.0f; }
void ConfigStore::setMeasTempC(float) {}
float ConfigStore::getCalTempC() { return 25.0f; }
void ConfigStore::setCalTempC(float) {}
void ConfigStore::setBufferPH4(float v) { _bufPH4 = v; }
void ConfigStore::setBufferPH7(float v) { _bufPH7 = v; }
void ConfigStore::setBufferPH10(float v) { _bufPH10 = v; }
int ConfigStore::getSlopeWindowHours() { return 72; }
void ConfigStore::setSlopeWindowHours(int) {}
int ConfigStore::getStirrerSpeed() { return 90; }
void ConfigStore::setStirrerSpeed(int) {}
float ConfigStore::getSamplePumpRPM() { return 80.0f; }
void ConfigStore::setSamplePumpRPM(float) {}
float ConfigStore::getSampleCalRevsPerML() { return 4.55f; }
void ConfigStore::setSampleCalRevsPerML(float) {}
int ConfigStore::getSampleCalRevolutions() { return 350; }
void ConfigStore::setSampleCalRevolutions(int) {}
int ConfigStore::getNumWashes() { return 2; }
void ConfigStore::setNumWashes(int) {}
uint8_t ConfigStore::getPhSensorType() { return 0; }
void ConfigStore::setPhSensorType(uint8_t) {}
float ConfigStore::getVoltage4PHExt() { return 2032.0f; }
float ConfigStore::getVoltage7PHExt() { return 1500.0f; }
float ConfigStore::getVoltage10PHExt() { return 984.0f; }
void ConfigStore::setVoltage4PHExt(float) {}
void ConfigStore::setVoltage7PHExt(float) {}
void ConfigStore::setVoltage10PHExt(float) {}
bool ConfigStore::getSampleSpreadCycle() { return false; }
void ConfigStore::setSampleSpreadCycle(bool) {}
bool ConfigStore::getTitrateSpreadCycle() { return false; }
void ConfigStore::setTitrateSpreadCycle(bool) {}
float ConfigStore::getSampleMaxRPM() { return 0; }
void ConfigStore::setSampleMaxRPM(float) {}
float ConfigStore::getTitrateMaxRPM() { return 0; }
void ConfigStore::setTitrateMaxRPM(float) {}
bool ConfigStore::hasWifiCredentials() { return false; }
void ConfigStore::getWifiSSID(char* buf, size_t len) { if (buf && len > 0) buf[0] = '\0'; }
void ConfigStore::getWifiPassword(char* buf, size_t len) { if (buf && len > 0) buf[0] = '\0'; }
void ConfigStore::setWifiCredentials(const char*, const char*) {}
void ConfigStore::clearWifiCredentials() {}
void ConfigStore::getMqttServer(char* buf, size_t len) { if (buf && len > 0) buf[0] = '\0'; }
int ConfigStore::getMqttPort() { return 1883; }
void ConfigStore::getMqttUsername(char* buf, size_t len) { if (buf && len > 0) buf[0] = '\0'; }
void ConfigStore::getMqttPassword(char* buf, size_t len) { if (buf && len > 0) buf[0] = '\0'; }
void ConfigStore::setMqttConfig(const char*, int, const char*, const char*) {}
uint8_t ConfigStore::getBootCount() { return 0; }
void ConfigStore::setBootCount(uint8_t) {}
uint32_t ConfigStore::getLastBootTime() { return 0; }
void ConfigStore::setLastBootTime(uint32_t) {}
void ConfigStore::clearBootCount() {}
float ConfigStore::getLastKH() { return 0; }
float ConfigStore::getLastStartPH() { return 0; }
void ConfigStore::setLastKH(float) {}
void ConfigStore::setLastStartPH(float) {}
void ConfigStore::setScheduleCount(uint8_t) {}
void ConfigStore::setScheduleTime(uint8_t, uint16_t) {}
void ConfigStore::setScheduleMode(uint8_t) {}
bool ConfigStore::setIntervalHours(uint8_t) { return true; }
void ConfigStore::setAnchorTime(uint16_t) {}
uint32_t ConfigStore::getCalTimestamp() { return 0; }
void ConfigStore::setCalTimestamp(uint32_t) {}
uint32_t ConfigStore::getSampleCalTimestamp() { return 0; }
void ConfigStore::setSampleCalTimestamp(uint32_t) {}
uint32_t ConfigStore::getTitrationCalTimestamp() { return 0; }
void ConfigStore::setTitrationCalTimestamp(uint32_t) {}
void ConfigStore::setTimezone(const char*) {}
int ConfigStore::getSlopeHistory(SlopeEntry*, int) { return 0; }
void ConfigStore::addSlopeEntry(uint32_t, float, float) {}
int ConfigStore::getGranReadings() { return 10; }
void ConfigStore::setGranReadings(int) {}
float ConfigStore::getKHEMA() { return NAN; }
void ConfigStore::setKHEMA(float) {}
float ConfigStore::getKHEMAAlpha() { return 0.3f; }
void ConfigStore::setKHEMAAlpha(float) {}
