#include "config_store.h"
#include <config.h>
#include <EEPROM.h>

namespace {
inline int      clampi(int v, int lo, int hi)               { return v < lo ? lo : (v > hi ? hi : v); }
inline float    clampf(float v, float lo, float hi)         { return v < lo ? lo : (v > hi ? hi : v); }
inline uint32_t clampu32(uint32_t v, uint32_t lo, uint32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline uint8_t  clampu8(uint8_t v, uint8_t lo, uint8_t hi)  { return v < lo ? lo : (v > hi ? hi : v); }
inline uint16_t clampu16(uint16_t v, uint16_t lo, uint16_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

ConfigStore configStore;

void ConfigStore::begin() {
  prefs.begin("khctrl", false);

  // Migrate from EEPROM on first boot after update
  if (!prefs.getBool("migrated", false)) {
    migrateFromEEPROM();
    prefs.putBool("migrated", true);
  }

  // One-time migration: import hardcoded WiFi/MQTT credentials into NVS
  if (!prefs.getBool("wifi_migrated", false)) {
    prefs.putString("wifi_ssid", WIFI_SSID);
    prefs.putString("wifi_pass", WIFI_PASSWORD);
    prefs.putString("mqtt_srv", mqtt_server);
    prefs.putUShort("mqtt_port", mqtt_port);
    prefs.putString("mqtt_user", mqtt_username);
    prefs.putString("mqtt_pass", mqtt_password);
    prefs.putBool("wifi_migrated", true);
  }
}

void ConfigStore::migrateFromEEPROM() {
  EEPROM.begin(512);
  float v4, v7, v10;
  EEPROM.get(0, v4);
  EEPROM.get(sizeof(float), v7);
  EEPROM.get(2 * sizeof(float), v10);

  // Only migrate if values look valid (not NaN or obviously wrong)
  if (!isnan(v4) && v4 > 0 && v4 < 4096) {
    prefs.putFloat("v4ph", v4);
  }
  if (!isnan(v7) && v7 > 0 && v7 < 4096) {
    prefs.putFloat("v7ph", v7);
  }
  if (!isnan(v10) && v10 > 0 && v10 < 4096) {
    prefs.putFloat("v10ph", v10);
  }
}

// Device name
void ConfigStore::getDeviceName(char* buf, size_t len) {
  String s = prefs.getString("dev_name", DEFAULT_DEVICE_NAME);
  strncpy(buf, s.c_str(), len - 1);
  buf[len - 1] = '\0';
}

void ConfigStore::setDeviceName(const char* name) {
  prefs.putString("dev_name", name);
}

// Timezone (POSIX TZ string)
static char tzBuf[48] = "CET-1CEST,M3.5.0/2,M10.5.0/3";

const char* ConfigStore::getTimezone() {
  String s = prefs.getString("timezone", "CET-1CEST,M3.5.0/2,M10.5.0/3");
  strncpy(tzBuf, s.c_str(), sizeof(tzBuf) - 1);
  tzBuf[sizeof(tzBuf) - 1] = '\0';
  return tzBuf;
}

void ConfigStore::setTimezone(const char* tz) {
  prefs.putString("timezone", tz);
}

// pH calibration voltages
float ConfigStore::getVoltage4PH() { return prefs.getFloat("v4ph", 1812.0); }
float ConfigStore::getVoltage7PH() { return prefs.getFloat("v7ph", 1292.0); }
float ConfigStore::getVoltage10PH() { return prefs.getFloat("v10ph", 900.0); }
void ConfigStore::setVoltage4PH(float v) { prefs.putFloat("v4ph", v); }
void ConfigStore::setVoltage7PH(float v) { prefs.putFloat("v7ph", v); }
void ConfigStore::setVoltage10PH(float v) { prefs.putFloat("v10ph", v); }

// KH calculation parameters
float ConfigStore::getTitrationVolume() { return clampf(prefs.getFloat("tit_vol", 9.8), 1.0f, 100.0f); }
float ConfigStore::getSampleVolume()    { return clampf(prefs.getFloat("sam_vol", 77.0), 10.0f, 500.0f); }
float ConfigStore::getCorrectionFactor() { return prefs.getFloat("corr_f", 1.0); }
float ConfigStore::getHClMolarity() { return prefs.getFloat("hcl_mol", 0.024); }
float ConfigStore::getHClVolume() { return prefs.getFloat("hcl_vol", 5000.0); }
int ConfigStore::getCalUnits() {
  // Floor at 100 to prevent division-by-zero from NVS corruption. Default 6000 if missing.
  int v = prefs.getInt("cal_drops", 6000);
  return (v < 100) ? 6000 : v;
}
float ConfigStore::getFastTitrationPH() { return prefs.getFloat("fast_ph", FAST_TITRATION_PH_DEFAULT); }
void ConfigStore::setTitrationVolume(float v) { prefs.putFloat("tit_vol", v); }
void ConfigStore::setSampleVolume(float v) { prefs.putFloat("sam_vol", v); }
void ConfigStore::setCorrectionFactor(float v) { prefs.putFloat("corr_f", v); }
void ConfigStore::setHClMolarity(float v) { prefs.putFloat("hcl_mol", v); }
void ConfigStore::setHClVolume(float v) { prefs.putFloat("hcl_vol", v); }
void ConfigStore::setCalUnits(int v) { prefs.putInt("cal_drops", v); }
void ConfigStore::setFastTitrationPH(float v) { prefs.putFloat("fast_ph", v); }
uint8_t ConfigStore::getEndpointMethod() { return prefs.getUChar("ep_method", 0); }
void ConfigStore::setEndpointMethod(uint8_t m) { prefs.putUChar("ep_method", m); }
float ConfigStore::getMinStartPH() { return prefs.getFloat("min_sph", MIN_START_PH_DEFAULT); }
void ConfigStore::setMinStartPH(float v) { prefs.putFloat("min_sph", v); }
int ConfigStore::getStabilizationTimeout()       { return clampi(prefs.getInt("stab_ms", STABILIZATION_TIMEOUT_MS), 500, 5000); }
void ConfigStore::setStabilizationTimeout(int ms){ prefs.putInt("stab_ms", clampi(ms, 500, 5000)); }
int ConfigStore::getMixDelay()                   { return clampi(prefs.getInt("gran_mix", TITRATION_MIX_DELAY_GRAN_MS), 500, 5000); }
void ConfigStore::setMixDelay(int ms)            { prefs.putInt("gran_mix", clampi(ms, 500, 5000)); }

float ConfigStore::getGranMinR2()                { return clampf(prefs.getFloat("gran_r2", GRAN_MIN_R2), 0.990f, 1.000f); }
void ConfigStore::setGranMinR2(float v)          { prefs.putFloat("gran_r2", clampf(v, 0.990f, 1.000f)); }

float ConfigStore::getDropVolumeUL()             { return clampf(prefs.getFloat("drop_ul", 15.0), 5.0f, 200.0f); }
void ConfigStore::setDropVolumeUL(float ul)      { prefs.putFloat("drop_ul", clampf(ul, 5.0f, 200.0f)); }
float ConfigStore::getTitrationRPM()             { return clampf(prefs.getFloat("tit_rpm", TITRATION_RPM), 10.0f, 200.0f); }
void ConfigStore::setTitrationRPM(float rpm)     { prefs.putFloat("tit_rpm", clampf(rpm, 10.0f, 200.0f)); }
float ConfigStore::getGranBurstRPM()             { return clampf(prefs.getFloat("gran_brpm", GRAN_BURST_RPM), 80.0f, 250.0f); }
void ConfigStore::setGranBurstRPM(float rpm)     { prefs.putFloat("gran_brpm", clampf(rpm, 80.0f, 250.0f)); }
uint32_t ConfigStore::getGranBurstAccel()        { return clampu32(prefs.getUInt("gran_baccel", GRAN_BURST_ACCEL), 50000, 500000); }
void ConfigStore::setGranBurstAccel(uint32_t accel) { prefs.putUInt("gran_baccel", clampu32(accel, 50000, 500000)); }
float ConfigStore::getFastPhaseRPM()             { return clampf(prefs.getFloat("fast_rpm", 50.0), 10.0f, 150.0f); }
void ConfigStore::setFastPhaseRPM(float rpm)     { prefs.putFloat("fast_rpm", clampf(rpm, 10.0f, 150.0f)); }

float ConfigStore::getPrefillVolumeUL()          { return clampf(prefs.getFloat("prefill_ul", 100.0), 10.0f, 500.0f); }
void ConfigStore::setPrefillVolumeUL(float ul)   { prefs.putFloat("prefill_ul", clampf(ul, 10.0f, 500.0f)); }

// Max acid volume per measurement (mL)
float ConfigStore::getMaxAcidML()                { return clampf(prefs.getFloat("max_acid_ml", 16.0f), 5.0f, 100.0f); }
void ConfigStore::setMaxAcidML(float ml)         { prefs.putFloat("max_acid_ml", clampf(ml, 5.0f, 100.0f)); }

// Fast phase max step volume (µL)
int ConfigStore::getFastStepUL()                 { return clampi(prefs.getInt("fast_step_ul", 330), 50, 2000); }
void ConfigStore::setFastStepUL(int ul)          { prefs.putInt("fast_step_ul", clampi(ul, 50, 2000)); }

// Measurement temperature
float ConfigStore::getMeasTempC()                { return clampf(prefs.getFloat("meas_temp", DEFAULT_MEASUREMENT_TEMP_C), 0.0f, 40.0f); }
void ConfigStore::setMeasTempC(float t)          { prefs.putFloat("meas_temp", clampf(t, 0.0f, 40.0f)); }

// Calibration temperature (temperature at which pH buffers were measured)
float ConfigStore::getCalTempC()                 { return clampf(prefs.getFloat("cal_temp", DEFAULT_MEASUREMENT_TEMP_C), 0.0f, 40.0f); }
void ConfigStore::setCalTempC(float t)           { prefs.putFloat("cal_temp", clampf(t, 0.0f, 40.0f)); }

// Buffer pH reference values at 25°C (user-settable, from bottle labels)
float ConfigStore::getBufferPH4()  { return prefs.getFloat("buf_ph4", DEFAULT_BUFFER_PH_4); }
void  ConfigStore::setBufferPH4(float v)  { prefs.putFloat("buf_ph4", v); }
float ConfigStore::getBufferPH7()  { return prefs.getFloat("buf_ph7", DEFAULT_BUFFER_PH_7); }
void  ConfigStore::setBufferPH7(float v)  { prefs.putFloat("buf_ph7", v); }
float ConfigStore::getBufferPH10() { return prefs.getFloat("buf_ph10", DEFAULT_BUFFER_PH_10); }
void  ConfigStore::setBufferPH10(float v) { prefs.putFloat("buf_ph10", v); }

// KH slope lookback window
int ConfigStore::getSlopeWindowHours()           { return clampi(prefs.getInt("slope_hrs", 72), 24, 168); }
void ConfigStore::setSlopeWindowHours(int h)     { prefs.putInt("slope_hrs", clampi(h, 24, 168)); }

// Stirrer speed (percent)
int ConfigStore::getStirrerSpeed()               { return clampi(prefs.getInt("stir_spd", 90), 80, 100); }
void ConfigStore::setStirrerSpeed(int pct)       { prefs.putInt("stir_spd", clampi(pct, 80, 100)); }

// Sample pump speed
float ConfigStore::getSamplePumpRPM()            { return clampf(prefs.getFloat("samp_rpm", MOTOR_TARGET_RPM), 20.0f, 250.0f); }
void ConfigStore::setSamplePumpRPM(float rpm)    { prefs.putFloat("samp_rpm", clampf(rpm, 20.0f, 250.0f)); }

// Sample pump calibration factor (revolutions per mL)
float ConfigStore::getSampleCalRevsPerML()       { return clampf(prefs.getFloat("samp_cal", 4.55f /* 350 revs / 77.0 mL */), 0.5f, 100.0f); }
void ConfigStore::setSampleCalRevsPerML(float v) { prefs.putFloat("samp_cal", clampf(v, 0.5f, 100.0f)); }

// Sample pump calibration revolutions (how many revs during calibration run)
int ConfigStore::getSampleCalRevolutions()       { return clampi(prefs.getInt("samp_cal_rev", SAMPLE_CAL_REVOLUTIONS), 50, 2200); }
void ConfigStore::setSampleCalRevolutions(int v) { prefs.putInt("samp_cal_rev", clampi(v, 50, 2200)); }

int ConfigStore::getNumWashes()                  { return clampi(prefs.getInt("num_washes", 2), 1, 5); }
void ConfigStore::setNumWashes(int n)            { prefs.putInt("num_washes", clampi(n, 1, 5)); }

// Gran zone pH readings per step
int ConfigStore::getGranReadings()               { return clampi(prefs.getInt("gran_read", 10), 3, 50); }
void ConfigStore::setGranReadings(int n)         { prefs.putInt("gran_read", clampi(n, 3, 50)); }

// EMA-smoothed KH
float ConfigStore::getKHEMA() {
  float v = prefs.getFloat("kh_ema", NAN);
  return v;
}
void ConfigStore::setKHEMA(float v) {
  prefs.putFloat("kh_ema", v);
}
float ConfigStore::getKHEMAAlpha()               { return clampf(prefs.getFloat("kh_ema_a", 0.3f), 0.1f, 1.0f); }
void ConfigStore::setKHEMAAlpha(float a)         { prefs.putFloat("kh_ema_a", clampf(a, 0.1f, 1.0f)); }

// pH sensor type: 0=auto, 1=internal, 2=ADS1115, 3=EZO
uint8_t ConfigStore::getPhSensorType() {
  // Migrate from old use_ads boolean on first access.
  // Old default was true (ADS1115 enabled) so absence of the key also means ADS1115.
  if (!prefs.getBool("ph_mig", false)) {
    bool useAds = prefs.getBool("use_ads", true);  // default true = ADS1115
    prefs.putUChar("ph_sensor", useAds ? PH_SENSOR_ADS1115 : PH_SENSOR_INTERNAL);
    prefs.remove("use_ads");
    prefs.putBool("ph_mig", true);
    prefs.putBool("ph_mig2", true);  // mark second migration done too
  }
  // Second migration: devices that ran the first migration before the fix above landed
  // ended up with ph_sensor=AUTO (0) because use_ads was never stored (default was true).
  // Upgrade AUTO → ADS1115 to restore the original behaviour.
  if (!prefs.getBool("ph_mig2", false)) {
    if (prefs.getUChar("ph_sensor", PH_SENSOR_AUTO) == PH_SENSOR_AUTO) {
      prefs.putUChar("ph_sensor", PH_SENSOR_ADS1115);
    }
    prefs.putBool("ph_mig2", true);
  }
  return prefs.getUChar("ph_sensor", PH_SENSOR_AUTO);
}
void ConfigStore::setPhSensorType(uint8_t t) {
  if (t > PH_SENSOR_EZO) t = PH_SENSOR_AUTO;
  prefs.putUChar("ph_sensor", t);
}
float ConfigStore::getVoltage4PHExt() { return prefs.getFloat("v4ph_ext", NAN); }
float ConfigStore::getVoltage7PHExt() { return prefs.getFloat("v7ph_ext", NAN); }
float ConfigStore::getVoltage10PHExt() { return prefs.getFloat("v10ph_ext", NAN); }
void ConfigStore::setVoltage4PHExt(float v) { prefs.putFloat("v4ph_ext", v); }
void ConfigStore::setVoltage7PHExt(float v) { prefs.putFloat("v7ph_ext", v); }
void ConfigStore::setVoltage10PHExt(float v) { prefs.putFloat("v10ph_ext", v); }

// TMC2209 per-motor chopper mode
bool ConfigStore::getSampleSpreadCycle() { return prefs.getBool("samp_sc", false); }
void ConfigStore::setSampleSpreadCycle(bool v) { prefs.putBool("samp_sc", v); }
bool ConfigStore::getTitrateSpreadCycle() { return prefs.getBool("titr_sc", false); }
void ConfigStore::setTitrateSpreadCycle(bool v) { prefs.putBool("titr_sc", v); }

// Last measurement results
float ConfigStore::getLastKH() { return prefs.getFloat("last_kh", 0); }
float ConfigStore::getLastStartPH() { return prefs.getFloat("last_sph", 0); }
void ConfigStore::setLastKH(float v) { prefs.putFloat("last_kh", v); }
void ConfigStore::setLastStartPH(float v) { prefs.putFloat("last_sph", v); }

// Schedule
uint8_t ConfigStore::getScheduleCount() { return prefs.getUChar("sched_cnt", 4); }

uint16_t ConfigStore::getScheduleTime(uint8_t index) {
  char key[10];
  snprintf(key, sizeof(key), "sched_%d", index);
  // Defaults: 02:00, 08:00, 14:00, 20:00
  uint16_t defaults[] = {120, 480, 840, 1200, 0, 0, 0, 0};
  return prefs.getUShort(key, (index < 8) ? defaults[index] : 0);
}

void ConfigStore::setScheduleCount(uint8_t count) {
  if (count > 8) count = 8;
  prefs.putUChar("sched_cnt", count);
}

void ConfigStore::setScheduleTime(uint8_t index, uint16_t minutesFromMidnight) {
  if (index >= 8) return;
  if (minutesFromMidnight > 1439) minutesFromMidnight = 1439;
  char key[10];
  snprintf(key, sizeof(key), "sched_%d", index);
  prefs.putUShort(key, minutesFromMidnight);
}

// Schedule mode
uint8_t ConfigStore::getScheduleMode() { return prefs.getUChar("sched_mode", 1); }
void ConfigStore::setScheduleMode(uint8_t mode) {
  if (mode > 2) mode = 0;  // 0=custom, 1=interval, 2=never
  prefs.putUChar("sched_mode", mode);
}

// Interval mode parameters
uint8_t ConfigStore::getIntervalHours() { return prefs.getUChar("sched_intv", 6); }
bool ConfigStore::setIntervalHours(uint8_t h) {
  const uint8_t valid[] = {1, 2, 3, 4, 6, 8, 12, 24};
  bool ok = false;
  for (int i = 0; i < 8; i++) { if (h == valid[i]) { ok = true; break; } }
  if (!ok) return false;
  prefs.putUChar("sched_intv", h);
  return true;
}

uint16_t ConfigStore::getAnchorTime() { return prefs.getUShort("sched_anch", 1200); }
void ConfigStore::setAnchorTime(uint16_t mins) {
  if (mins > 1439) mins = 1439;
  prefs.putUShort("sched_anch", mins);
}

// Calibration timestamps
uint32_t ConfigStore::getCalTimestamp() { return prefs.getULong("cal_ts", 0); }
void ConfigStore::setCalTimestamp(uint32_t ts) { prefs.putULong("cal_ts", ts); }
uint32_t ConfigStore::getSampleCalTimestamp() { return prefs.getULong("samp_cal_ts", 0); }
void ConfigStore::setSampleCalTimestamp(uint32_t ts) { prefs.putULong("samp_cal_ts", ts); }
uint32_t ConfigStore::getTitrationCalTimestamp() { return prefs.getULong("tit_cal_ts", 0); }
void ConfigStore::setTitrationCalTimestamp(uint32_t ts) { prefs.putULong("tit_cal_ts", ts); }

// Motor max speed (from config.h)
float ConfigStore::getSampleMaxRPM() { return SAMPLE_MAX_RPM; }
void ConfigStore::setSampleMaxRPM(float) {}
float ConfigStore::getTitrateMaxRPM() { return TITRATE_MAX_RPM; }
void ConfigStore::setTitrateMaxRPM(float) {}

// WiFi credentials
bool ConfigStore::hasWifiCredentials() {
  return prefs.getString("wifi_ssid", "").length() > 0;
}

void ConfigStore::getWifiSSID(char* buf, size_t len) {
  String s = prefs.getString("wifi_ssid", "");
  strncpy(buf, s.c_str(), len - 1);
  buf[len - 1] = '\0';
}

void ConfigStore::getWifiPassword(char* buf, size_t len) {
  String s = prefs.getString("wifi_pass", "");
  strncpy(buf, s.c_str(), len - 1);
  buf[len - 1] = '\0';
}

void ConfigStore::setWifiCredentials(const char* ssid, const char* password) {
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", password);
}

void ConfigStore::clearWifiCredentials() {
  prefs.putString("wifi_ssid", "");
  prefs.putString("wifi_pass", "");
}

// MQTT broker config
static char mqttServerBuf[65];
static char mqttUserBuf[33];
static char mqttPassBuf[33];

void ConfigStore::getMqttServer(char* buf, size_t len) {
  String s = prefs.getString("mqtt_srv", "homeassistant.local");
  strncpy(buf, s.c_str(), len - 1);
  buf[len - 1] = '\0';
}

int ConfigStore::getMqttPort() {
  return prefs.getUShort("mqtt_port", 1883);
}

void ConfigStore::getMqttUsername(char* buf, size_t len) {
  String s = prefs.getString("mqtt_user", "");
  strncpy(buf, s.c_str(), len - 1);
  buf[len - 1] = '\0';
}

void ConfigStore::getMqttPassword(char* buf, size_t len) {
  String s = prefs.getString("mqtt_pass", "");
  strncpy(buf, s.c_str(), len - 1);
  buf[len - 1] = '\0';
}

void ConfigStore::setMqttConfig(const char* server, int port, const char* user, const char* pass) {
  prefs.putString("mqtt_srv", server);
  prefs.putUShort("mqtt_port", (uint16_t)port);
  prefs.putString("mqtt_user", user);
  prefs.putString("mqtt_pass", pass);
}

// Boot counter for triple power-cycle detection
uint8_t ConfigStore::getBootCount() { return prefs.getUChar("boot_cnt", 0); }
void ConfigStore::setBootCount(uint8_t count) { prefs.putUChar("boot_cnt", count); }
uint32_t ConfigStore::getLastBootTime() { return prefs.getULong("last_boot", 0); }
void ConfigStore::setLastBootTime(uint32_t ms) { prefs.putULong("last_boot", ms); }
void ConfigStore::clearBootCount() {
  prefs.putUChar("boot_cnt", 0);
  prefs.putULong("last_boot", 0);
}

// Slope history
int ConfigStore::getSlopeHistory(SlopeEntry* entries, int maxEntries) {
  uint8_t count = prefs.getUChar("sl_cnt", 0);
  if (count > MAX_SLOPE_HISTORY) count = MAX_SLOPE_HISTORY;
  int toRead = (count < maxEntries) ? count : maxEntries;
  if (toRead > 0) {
    SlopeEntry all[MAX_SLOPE_HISTORY];
    size_t expected = count * sizeof(SlopeEntry);
    size_t read = prefs.getBytes("sl_data", all, expected);
    if (read != expected) return 0;  // NVS corrupt — return empty
    // Return the last toRead entries (newest)
    int offset = count - toRead;
    memcpy(entries, all + offset, toRead * sizeof(SlopeEntry));
  }
  return toRead;
}

void ConfigStore::addSlopeEntry(uint32_t timestamp, float slope, float asymmetry) {
  if (isnan(slope)) return;

  // Struct format changed (added asymmetry field) — detect old format by size mismatch
  if (!prefs.getBool("sl_v2", false)) {
    prefs.putUChar("sl_cnt", 0);  // reset history on format upgrade
    prefs.putBool("sl_v2", true);
  }

  SlopeEntry entries[MAX_SLOPE_HISTORY];
  uint8_t count = prefs.getUChar("sl_cnt", 0);
  if (count > MAX_SLOPE_HISTORY) count = MAX_SLOPE_HISTORY;
  if (count > 0) {
    size_t read = prefs.getBytes("sl_data", entries, count * sizeof(SlopeEntry));
    if (read != count * sizeof(SlopeEntry)) count = 0;  // corrupt NVS — reset
  }

  // Dedup: if last entry is within 1 hour, update it (same calibration session)
  if (count > 0 && timestamp >= entries[count - 1].timestamp
      && (timestamp - entries[count - 1].timestamp) < 3600) {
    entries[count - 1].slope = slope;
    entries[count - 1].asymmetry = asymmetry;
    entries[count - 1].timestamp = timestamp;
  } else {
    // Shift oldest out if full
    if (count >= MAX_SLOPE_HISTORY) {
      memmove(entries, entries + 1, (MAX_SLOPE_HISTORY - 1) * sizeof(SlopeEntry));
      count = MAX_SLOPE_HISTORY - 1;
    }
    entries[count].timestamp = timestamp;
    entries[count].slope = slope;
    entries[count].asymmetry = asymmetry;
    count++;
    prefs.putUChar("sl_cnt", count);
  }
  prefs.putBytes("sl_data", entries, count * sizeof(SlopeEntry));
}
