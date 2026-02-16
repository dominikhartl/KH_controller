#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>
#include "config_store.h"

class Scheduler {
public:
  typedef void (*MeasurementCallback)();

  void begin();
  void loop();
  bool isTimeSynced();
  void onMeasurementDue(MeasurementCallback cb);
  void resetDailyFlags();

  // Compute effective time slots for interval mode
  uint8_t computeIntervalSlots(uint16_t* outSlots, uint8_t maxSlots);

  // Get formatted time strings for dashboard
  String getNextMeasurementTime();
  String getCurrentTime();

private:
  MeasurementCallback callback = nullptr;
  bool timeSynced = false;

  // Fallback interval-based scheduling
  unsigned long lastMeasurementTime = 0;
  static const unsigned long FALLBACK_INTERVAL_MS = 21600000; // 6 hours

  // Schedule state (24 slots max for 1h interval mode)
  bool alreadyRanToday[24] = {};
  uint8_t lastDay = 0;

  // Build effective slot list based on current mode
  uint8_t buildEffectiveSlots(uint16_t* slots, uint8_t maxSlots);
};

extern Scheduler scheduler;

#endif // SCHEDULER_H
