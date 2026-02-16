#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <Preferences.h>

class ConfigStore {
public:
  void begin();

  // pH calibration voltages
  float getVoltage4PH();
  float getVoltage7PH();
  float getVoltage10PH();
  void setVoltage4PH(float v);
  void setVoltage7PH(float v);
  void setVoltage10PH(float v);

  // KH calculation parameters
  float getTitrationVolume();   // mL per calDrops drops (default 13.4)
  float getSampleVolume();      // mL (default 82.0)
  float getCorrectionFactor();  // unitless (default 1.0)
  float getHClMolarity();       // mol/L (default 0.02)
  float getHClVolume();         // mL remaining (default 5000)
  int getCalDrops();            // drops per calibration run (default 6000)
  float getFastTitrationPH();   // pH threshold: fast→precise (default 5.8)
  void setTitrationVolume(float v);
  void setSampleVolume(float v);
  void setCorrectionFactor(float v);
  void setHClMolarity(float v);
  void setHClVolume(float v);
  void setCalDrops(int v);
  void setFastTitrationPH(float v);

  // Last measurement results (persistent across reboots)
  float getLastKH();
  float getLastStartPH();
  void setLastKH(float v);
  void setLastStartPH(float v);

  // Schedule (minutes from midnight, up to 8 slots)
  uint8_t getScheduleCount();
  uint16_t getScheduleTime(uint8_t index);
  void setScheduleCount(uint8_t count);
  void setScheduleTime(uint8_t index, uint16_t minutesFromMidnight);

private:
  Preferences prefs;
  void migrateFromEEPROM();
};

extern ConfigStore configStore;

#endif // CONFIG_STORE_H
