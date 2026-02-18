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
  float getTitrationVolume();   // mL per calUnits (default 13.4)
  float getSampleVolume();      // mL (default 82.0)
  float getCorrectionFactor();  // unitless (default 1.0)
  float getHClMolarity();       // mol/L (default 0.02)
  float getHClVolume();         // mL remaining (default 5000)
  int getCalUnits();             // units per calibration run (default 6000)
  float getFastTitrationPH();   // pH threshold: fast→precise (default 5.0)
  uint8_t getEndpointMethod();  // 0=Gran, 1=Fixed (default 0)
  void setTitrationVolume(float v);
  void setSampleVolume(float v);
  void setCorrectionFactor(float v);
  void setHClMolarity(float v);
  void setHClVolume(float v);
  void setCalUnits(int v);
  void setFastTitrationPH(float v);
  void setEndpointMethod(uint8_t m);

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

  // Schedule mode (0=custom, 1=interval)
  uint8_t getScheduleMode();
  void setScheduleMode(uint8_t mode);

  // Interval mode parameters
  uint8_t getIntervalHours();        // one of {1,2,3,4,6,8,12,24}
  void setIntervalHours(uint8_t h);
  uint16_t getAnchorTime();          // minutes from midnight
  void setAnchorTime(uint16_t mins);

  // Calibration timestamp (Unix epoch)
  uint32_t getCalTimestamp();
  void setCalTimestamp(uint32_t ts);

  // Slope history (up to 10 entries, newest last)
  static const int MAX_SLOPE_HISTORY = 10;
  struct SlopeEntry {
    uint32_t timestamp;
    float slope;
  };
  int getSlopeHistory(SlopeEntry* entries, int maxEntries);
  void addSlopeEntry(uint32_t timestamp, float slope);

private:
  Preferences prefs;
  void migrateFromEEPROM();
};

extern ConfigStore configStore;

#endif // CONFIG_STORE_H
