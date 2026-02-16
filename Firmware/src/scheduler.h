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

  // Get formatted time strings for dashboard
  String getNextMeasurementTime();
  String getCurrentTime();

private:
  MeasurementCallback callback = nullptr;
  bool timeSynced = false;

  // Fallback interval-based scheduling
  unsigned long lastMeasurementTime = 0;
  static const unsigned long FALLBACK_INTERVAL_MS = 21600000; // 6 hours

  // Schedule state
  bool alreadyRanToday[8] = {};
  uint8_t lastDay = 0;
};

extern Scheduler scheduler;

#endif // SCHEDULER_H
