#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <Preferences.h>

class ConfigStore {
public:
  void begin();

  // Device name (mDNS hostname, MQTT prefix, UI title)
  void getDeviceName(char* buf, size_t len);
  void setDeviceName(const char* name);

  // pH calibration voltages
  float getVoltage4PH();
  float getVoltage7PH();
  float getVoltage10PH();
  void setVoltage4PH(float v);
  void setVoltage7PH(float v);
  void setVoltage10PH(float v);

  // KH calculation parameters
  float getTitrationVolume();   // mL per calUnits (default 9.8)
  float getSampleVolume();      // mL (default 82.0)
  float getCorrectionFactor();  // unitless (default 1.0)
  float getHClMolarity();       // mol/L (default 0.02)
  float getHClVolume();         // mL remaining (default 5000)
  int getCalUnits();             // units per calibration run (default 6000)
  float getFastTitrationPH();   // pH threshold: fast→precise (default 5.0)
  uint8_t getEndpointMethod();  // 0=Gran, 1=Fixed (default 0)
  float getMinStartPH();        // Minimum acceptable starting pH (default 7.5)
  int getStabilizationTimeout(); // Stabilization timeout in ms (default 2000, max 5000)
  int getGranMixDelay();         // Gran zone mixing delay in ms (default TITRATION_MIX_DELAY_GRAN_MS)
  void setTitrationVolume(float v);
  void setSampleVolume(float v);
  void setCorrectionFactor(float v);
  void setHClMolarity(float v);
  void setHClVolume(float v);
  void setCalUnits(int v);
  void setFastTitrationPH(float v);
  void setEndpointMethod(uint8_t m);
  void setMinStartPH(float v);
  void setStabilizationTimeout(int ms);
  void setGranMixDelay(int ms);
  float getDropVolumeUL();       // Gran zone drop volume in µL (default 26.0)
  void setDropVolumeUL(float ul);
  float getTitrationRPM();       // Cal/prefill/fill pump speed in RPM (default TITRATION_RPM)
  void setTitrationRPM(float rpm);
  float getGranBurstRPM();       // Gran zone burst ejection speed in RPM (default GRAN_BURST_RPM)
  void setGranBurstRPM(float rpm);
  uint32_t getGranBurstAccel();  // Gran zone burst acceleration in steps/s² (default GRAN_BURST_ACCEL)
  void setGranBurstAccel(uint32_t accel);
  float getFastPhaseRPM();       // Fast titration phase speed in RPM (default 50)
  void setFastPhaseRPM(float rpm);
  float getPrefillVolumeUL();    // Prefill volume in µL (default 100.0)
  void setPrefillVolumeUL(float ul);
  float getMaxAcidML();           // Max acid volume per measurement in mL (default 16.0)
  void setMaxAcidML(float ml);
  int getFastStepUL();             // Fast phase max step volume in µL (default 330, range 50-2000)
  void setFastStepUL(int ul);
  float getMeasTempC();           // Measurement temperature in °C (default 21.0)
  void setMeasTempC(float t);
  float getCalTempC();            // Temperature at which pH calibration was performed
  void setCalTempC(float t);
  float getBufferPH4();           // Buffer pH 4 value at 25°C (as printed on bottle)
  void setBufferPH4(float v);
  float getBufferPH7();           // Buffer pH 7 value at 25°C
  void setBufferPH7(float v);
  float getBufferPH10();          // Buffer pH 10 value at 25°C
  void setBufferPH10(float v);
  int getSlopeWindowHours();      // KH slope lookback window in hours (default 72, range 24-168)
  void setSlopeWindowHours(int h);
  int getStirrerSpeed();           // Stirrer speed in percent (default 90, range 80-100)
  void setStirrerSpeed(int pct);
  float getSamplePumpRPM();        // Sample pump speed in RPM (default MOTOR_TARGET_RPM, range 20-400)
  void setSamplePumpRPM(float rpm);
  float getSampleCalRevsPerML();   // Sample pump calibration: revolutions per mL (default ~4.55)
  void setSampleCalRevsPerML(float v);
  int getSampleCalRevolutions();   // Revolutions used during sample pump calibration run (default 350)
  void setSampleCalRevolutions(int v);
  int getNumWashes();              // Number of washes before measurement (default 2, range 1-5)
  void setNumWashes(int n);

  // ADS1115 external ADC
  bool getUseADS1115();             // Use external ADC for pH (default false)
  void setUseADS1115(bool v);
  float getVoltage4PHExt();         // External ADC calibration voltages
  float getVoltage7PHExt();
  float getVoltage10PHExt();
  void setVoltage4PHExt(float v);
  void setVoltage7PHExt(float v);
  void setVoltage10PHExt(float v);

  // TMC2209 per-motor stall detection settings
  bool getSampleSpreadCycle();        // Use SpreadCycle for sample pump (default false)
  void setSampleSpreadCycle(bool v);
  int getSampleStallSG();             // SG stall threshold for sample pump (default 30)
  void setSampleStallSG(int v);
  bool getTitrateSpreadCycle();       // Use SpreadCycle for titration pump (default false)
  void setTitrateSpreadCycle(bool v);
  int getTitrateStallSG();            // SG stall threshold for titration pump (default 100)
  void setTitrateStallSG(int v);
  int getSampleSGBaseline();           // New-tube SG baseline for sample pump (default 0 = not set)
  void setSampleSGBaseline(int v);
  int getTitrateSGBaseline();          // New-tube SG baseline for titration pump (default 0 = not set)
  void setTitrateSGBaseline(int v);

  // Motor max speed from diagnostics (RPM, 0 = not tested)
  float getSampleMaxRPM();
  void setSampleMaxRPM(float rpm);
  float getTitrateMaxRPM();
  void setTitrateMaxRPM(float rpm);

  // WiFi credentials (stored in NVS, migrated from credentials.h on first boot)
  bool hasWifiCredentials();
  void getWifiSSID(char* buf, size_t len);
  void getWifiPassword(char* buf, size_t len);
  void setWifiCredentials(const char* ssid, const char* password);
  void clearWifiCredentials();

  // MQTT broker config (stored in NVS, migrated from config.h defaults on first boot)
  void getMqttServer(char* buf, size_t len);
  int getMqttPort();
  void getMqttUsername(char* buf, size_t len);
  void getMqttPassword(char* buf, size_t len);
  void setMqttConfig(const char* server, int port, const char* user, const char* pass);

  // Boot counter for triple power-cycle WiFi reset detection
  uint8_t getBootCount();
  void setBootCount(uint8_t count);
  uint32_t getLastBootTime();
  void setLastBootTime(uint32_t ms);
  void clearBootCount();

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

  // Schedule mode (0=custom, 1=interval, 2=never)
  uint8_t getScheduleMode();
  void setScheduleMode(uint8_t mode);

  // Interval mode parameters
  uint8_t getIntervalHours();        // one of {1,2,3,4,6,8,12,24}
  bool setIntervalHours(uint8_t h);  // returns false if invalid
  uint16_t getAnchorTime();          // minutes from midnight
  void setAnchorTime(uint16_t mins);

  // Calibration timestamps (Unix epoch)
  uint32_t getCalTimestamp();
  void setCalTimestamp(uint32_t ts);
  uint32_t getSampleCalTimestamp();
  void setSampleCalTimestamp(uint32_t ts);
  uint32_t getTitrationCalTimestamp();
  void setTitrationCalTimestamp(uint32_t ts);

  // Timezone (POSIX TZ string, e.g. "CET-1CEST,M3.5.0/2,M10.5.0/3")
  const char* getTimezone();
  void setTimezone(const char* tz);

  // Slope history (up to 10 entries, newest last)
  static const int MAX_SLOPE_HISTORY = 10;
  struct SlopeEntry {
    uint32_t timestamp;
    float slope;
    float asymmetry;
  };
  int getSlopeHistory(SlopeEntry* entries, int maxEntries);
  void addSlopeEntry(uint32_t timestamp, float slope, float asymmetry);

private:
  Preferences prefs;
  void migrateFromEEPROM();
};

extern ConfigStore configStore;

#endif // CONFIG_STORE_H
