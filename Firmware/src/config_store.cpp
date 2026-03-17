#include "config_store.h"
#include <config.h>
#include <EEPROM.h>

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
float ConfigStore::getTitrationVolume() {
  float v = prefs.getFloat("tit_vol", 9.8);
  if (v < 1.0) v = 1.0;
  if (v > 100.0) v = 100.0;
  return v;
}
float ConfigStore::getSampleVolume() {
  float v = prefs.getFloat("sam_vol", 77.0);
  if (v < 10.0) v = 10.0;
  if (v > 500.0) v = 500.0;
  return v;
}
float ConfigStore::getCorrectionFactor() { return prefs.getFloat("corr_f", 1.0); }
float ConfigStore::getHClMolarity() { return prefs.getFloat("hcl_mol", 0.024); }
float ConfigStore::getHClVolume() { return prefs.getFloat("hcl_vol", 5000.0); }
int ConfigStore::getCalUnits() { return prefs.getInt("cal_drops", 6000); }
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
int ConfigStore::getStabilizationTimeout() {
  int ms = prefs.getInt("stab_ms", STABILIZATION_TIMEOUT_MS);
  if (ms < 500) ms = 500;
  if (ms > 5000) ms = 5000;
  return ms;
}
void ConfigStore::setStabilizationTimeout(int ms) {
  if (ms < 500) ms = 500;
  if (ms > 5000) ms = 5000;
  prefs.putInt("stab_ms", ms);
}
int ConfigStore::getGranMixDelay() {
  int ms = prefs.getInt("gran_mix", TITRATION_MIX_DELAY_GRAN_MS);
  if (ms < 500) ms = 500;
  if (ms > 5000) ms = 5000;
  return ms;
}
void ConfigStore::setGranMixDelay(int ms) {
  if (ms < 500) ms = 500;
  if (ms > 5000) ms = 5000;
  prefs.putInt("gran_mix", ms);
}

float ConfigStore::getDropVolumeUL() {
  float ul = prefs.getFloat("drop_ul", 26.0);
  if (ul < 5.0) ul = 5.0;
  if (ul > 200.0) ul = 200.0;
  return ul;
}
void ConfigStore::setDropVolumeUL(float ul) {
  if (ul < 5.0) ul = 5.0;
  if (ul > 200.0) ul = 200.0;
  prefs.putFloat("drop_ul", ul);
}
float ConfigStore::getTitrationRPM() {
  float rpm = prefs.getFloat("tit_rpm", TITRATION_RPM);
  if (rpm < 10.0) rpm = 10.0;
  if (rpm > 200.0) rpm = 200.0;
  return rpm;
}
void ConfigStore::setTitrationRPM(float rpm) {
  if (rpm < 10.0) rpm = 10.0;
  if (rpm > 200.0) rpm = 200.0;
  prefs.putFloat("tit_rpm", rpm);
}
float ConfigStore::getGranBurstRPM() {
  float rpm = prefs.getFloat("gran_brpm", GRAN_BURST_RPM);
  if (rpm < 80.0f)  rpm = 80.0f;
  if (rpm > 250.0f) rpm = 250.0f;
  return rpm;
}
void ConfigStore::setGranBurstRPM(float rpm) {
  if (rpm < 80.0f)  rpm = 80.0f;
  if (rpm > 250.0f) rpm = 250.0f;
  prefs.putFloat("gran_brpm", rpm);
}
uint32_t ConfigStore::getGranBurstAccel() {
  uint32_t a = prefs.getUInt("gran_baccel", GRAN_BURST_ACCEL);
  if (a < 50000)  a = 50000;
  if (a > 500000) a = 500000;
  return a;
}
void ConfigStore::setGranBurstAccel(uint32_t accel) {
  if (accel < 50000)  accel = 50000;
  if (accel > 500000) accel = 500000;
  prefs.putUInt("gran_baccel", accel);
}
float ConfigStore::getFastPhaseRPM() {
  float rpm = prefs.getFloat("fast_rpm", 50.0);
  if (rpm < 10.0) rpm = 10.0;
  if (rpm > 150.0) rpm = 150.0;
  return rpm;
}
void ConfigStore::setFastPhaseRPM(float rpm) {
  if (rpm < 10.0) rpm = 10.0;
  if (rpm > 150.0) rpm = 150.0;
  prefs.putFloat("fast_rpm", rpm);
}

float ConfigStore::getPrefillVolumeUL() {
  float ul = prefs.getFloat("prefill_ul", 100.0);
  if (ul < 10.0) ul = 10.0;
  if (ul > 500.0) ul = 500.0;
  return ul;
}
void ConfigStore::setPrefillVolumeUL(float ul) {
  if (ul < 10.0) ul = 10.0;
  if (ul > 500.0) ul = 500.0;
  prefs.putFloat("prefill_ul", ul);
}

// Max acid volume per measurement (mL)
float ConfigStore::getMaxAcidML() {
  float v = prefs.getFloat("max_acid_ml", 16.0f);
  if (v < 5.0f) v = 5.0f;
  if (v > 100.0f) v = 100.0f;
  return v;
}
void ConfigStore::setMaxAcidML(float ml) {
  if (ml < 5.0f) ml = 5.0f;
  if (ml > 100.0f) ml = 100.0f;
  prefs.putFloat("max_acid_ml", ml);
}

// Fast phase max step volume (µL)
int ConfigStore::getFastStepUL() {
  int v = prefs.getInt("fast_step_ul", 330);
  if (v < 50) v = 50;
  if (v > 2000) v = 2000;
  return v;
}
void ConfigStore::setFastStepUL(int ul) {
  if (ul < 50) ul = 50;
  if (ul > 2000) ul = 2000;
  prefs.putInt("fast_step_ul", ul);
}

// Measurement temperature
float ConfigStore::getMeasTempC() {
  float t = prefs.getFloat("meas_temp", DEFAULT_MEASUREMENT_TEMP_C);
  if (t < 0.0f) t = 0.0f;
  if (t > 40.0f) t = 40.0f;
  return t;
}
void ConfigStore::setMeasTempC(float t) {
  if (t < 0.0f) t = 0.0f;
  if (t > 40.0f) t = 40.0f;
  prefs.putFloat("meas_temp", t);
}

// Calibration temperature (temperature at which pH buffers were measured)
float ConfigStore::getCalTempC() {
  float t = prefs.getFloat("cal_temp", DEFAULT_MEASUREMENT_TEMP_C);
  if (t < 0.0f) t = 0.0f;
  if (t > 40.0f) t = 40.0f;
  return t;
}
void ConfigStore::setCalTempC(float t) {
  if (t < 0.0f) t = 0.0f;
  if (t > 40.0f) t = 40.0f;
  prefs.putFloat("cal_temp", t);
}

// Buffer pH reference values at 25°C (user-settable, from bottle labels)
float ConfigStore::getBufferPH4()  { return prefs.getFloat("buf_ph4", DEFAULT_BUFFER_PH_4); }
void  ConfigStore::setBufferPH4(float v)  { prefs.putFloat("buf_ph4", v); }
float ConfigStore::getBufferPH7()  { return prefs.getFloat("buf_ph7", DEFAULT_BUFFER_PH_7); }
void  ConfigStore::setBufferPH7(float v)  { prefs.putFloat("buf_ph7", v); }
float ConfigStore::getBufferPH10() { return prefs.getFloat("buf_ph10", DEFAULT_BUFFER_PH_10); }
void  ConfigStore::setBufferPH10(float v) { prefs.putFloat("buf_ph10", v); }

// KH slope lookback window
int ConfigStore::getSlopeWindowHours() {
  int h = prefs.getInt("slope_hrs", 72);
  if (h < 24) h = 24;
  if (h > 168) h = 168;
  return h;
}
void ConfigStore::setSlopeWindowHours(int h) {
  if (h < 24) h = 24;
  if (h > 168) h = 168;
  prefs.putInt("slope_hrs", h);
}

// Stirrer speed (percent)
int ConfigStore::getStirrerSpeed() {
  int pct = prefs.getInt("stir_spd", 90);
  if (pct < 80) pct = 80;
  if (pct > 100) pct = 100;
  return pct;
}
void ConfigStore::setStirrerSpeed(int pct) {
  if (pct < 80) pct = 80;
  if (pct > 100) pct = 100;
  prefs.putInt("stir_spd", pct);
}

// Sample pump speed
float ConfigStore::getSamplePumpRPM() {
  float rpm = prefs.getFloat("samp_rpm", MOTOR_TARGET_RPM);
  if (rpm < 20.0) rpm = 20.0;
  if (rpm > 250.0) rpm = 250.0;
  return rpm;
}
void ConfigStore::setSamplePumpRPM(float rpm) {
  if (rpm < 20.0) rpm = 20.0;
  if (rpm > 250.0) rpm = 250.0;
  prefs.putFloat("samp_rpm", rpm);
}

// Sample pump calibration factor (revolutions per mL)
float ConfigStore::getSampleCalRevsPerML() {
  float v = prefs.getFloat("samp_cal", 4.55f);  // default: SAMPLE_PUMP_VOLUME / 77.0
  if (v < 0.5f) v = 0.5f;
  if (v > 100.0f) v = 100.0f;
  return v;
}
void ConfigStore::setSampleCalRevsPerML(float v) {
  if (v < 0.5f) v = 0.5f;
  if (v > 100.0f) v = 100.0f;
  prefs.putFloat("samp_cal", v);
}

// Sample pump calibration revolutions (how many revs during calibration run)
int ConfigStore::getSampleCalRevolutions() {
  int v = prefs.getInt("samp_cal_rev", SAMPLE_CAL_REVOLUTIONS);
  if (v < 50) v = 50;
  if (v > 2200) v = 2200;
  return v;
}
void ConfigStore::setSampleCalRevolutions(int v) {
  if (v < 50) v = 50;
  if (v > 2200) v = 2200;
  prefs.putInt("samp_cal_rev", v);
}
int ConfigStore::getNumWashes() {
  int v = prefs.getInt("num_washes", 2);
  if (v < 1) v = 1;
  if (v > 5) v = 5;
  return v;
}
void ConfigStore::setNumWashes(int n) {
  if (n < 1) n = 1;
  if (n > 5) n = 5;
  prefs.putInt("num_washes", n);
}

// pH sensor type: 0=auto, 1=internal, 2=ADS1115, 3=EZO
uint8_t ConfigStore::getPhSensorType() {
  // Migrate from old use_ads boolean on first access
  if (!prefs.getBool("ph_mig", false)) {
    if (prefs.isKey("use_ads")) {
      bool useAds = prefs.getBool("use_ads", true);
      prefs.putUChar("ph_sensor", useAds ? PH_SENSOR_ADS1115 : PH_SENSOR_INTERNAL);
      prefs.remove("use_ads");
    }
    prefs.putBool("ph_mig", true);
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

// TMC2209 per-motor stall detection settings
bool ConfigStore::getSampleSpreadCycle() { return prefs.getBool("samp_sc", false); }
void ConfigStore::setSampleSpreadCycle(bool v) { prefs.putBool("samp_sc", v); }
int ConfigStore::getSampleStallSG() {
  int v = prefs.getInt("samp_sg", 0);
  if (v < 0) v = 0;
  if (v > 500) v = 500;
  return v;
}
void ConfigStore::setSampleStallSG(int v) {
  if (v < 0) v = 0;
  if (v > 500) v = 500;
  prefs.putInt("samp_sg", v);
}
bool ConfigStore::getTitrateSpreadCycle() { return prefs.getBool("titr_sc", false); }
void ConfigStore::setTitrateSpreadCycle(bool v) { prefs.putBool("titr_sc", v); }
int ConfigStore::getTitrateStallSG() {
  int v = prefs.getInt("titr_sg", 0);
  if (v < 0) v = 0;
  if (v > 500) v = 500;
  return v;
}
void ConfigStore::setTitrateStallSG(int v) {
  if (v < 0) v = 0;
  if (v > 500) v = 500;
  prefs.putInt("titr_sg", v);
}

// SG baselines (new-tube reference for wear detection)
int ConfigStore::getSampleSGBaseline() { return prefs.getInt("samp_sg_bl", 0); }
void ConfigStore::setSampleSGBaseline(int v) { prefs.putInt("samp_sg_bl", v); }
int ConfigStore::getTitrateSGBaseline() { return prefs.getInt("titr_sg_bl", 0); }
void ConfigStore::setTitrateSGBaseline(int v) { prefs.putInt("titr_sg_bl", v); }

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
