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

// pH calibration voltages
float ConfigStore::getVoltage4PH() { return prefs.getFloat("v4ph", 1812.0); }
float ConfigStore::getVoltage7PH() { return prefs.getFloat("v7ph", 1292.0); }
float ConfigStore::getVoltage10PH() { return prefs.getFloat("v10ph", 900.0); }
void ConfigStore::setVoltage4PH(float v) { prefs.putFloat("v4ph", v); }
void ConfigStore::setVoltage7PH(float v) { prefs.putFloat("v7ph", v); }
void ConfigStore::setVoltage10PH(float v) { prefs.putFloat("v10ph", v); }

// KH calculation parameters
float ConfigStore::getTitrationVolume() { return prefs.getFloat("tit_vol", 13.4); }
float ConfigStore::getSampleVolume() { return prefs.getFloat("sam_vol", 82.0); }
float ConfigStore::getCorrectionFactor() { return prefs.getFloat("corr_f", 1.0); }
float ConfigStore::getHClMolarity() { return prefs.getFloat("hcl_mol", 0.02); }
float ConfigStore::getHClVolume() { return prefs.getFloat("hcl_vol", 5000.0); }
int ConfigStore::getCalDrops() { return prefs.getInt("cal_drops", 6000); }
float ConfigStore::getFastTitrationPH() { return prefs.getFloat("fast_ph", FAST_TITRATION_PH_DEFAULT); }
void ConfigStore::setTitrationVolume(float v) { prefs.putFloat("tit_vol", v); }
void ConfigStore::setSampleVolume(float v) { prefs.putFloat("sam_vol", v); }
void ConfigStore::setCorrectionFactor(float v) { prefs.putFloat("corr_f", v); }
void ConfigStore::setHClMolarity(float v) { prefs.putFloat("hcl_mol", v); }
void ConfigStore::setHClVolume(float v) { prefs.putFloat("hcl_vol", v); }
void ConfigStore::setCalDrops(int v) { prefs.putInt("cal_drops", v); }
void ConfigStore::setFastTitrationPH(float v) { prefs.putFloat("fast_ph", v); }

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
