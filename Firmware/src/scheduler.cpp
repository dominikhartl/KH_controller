#include "scheduler.h"
#include <config.h>
#include <time.h>

Scheduler scheduler;

void Scheduler::begin() {
  // Configure NTP with timezone from config (defaults to CET/CEST for Europe/Zurich)
  const char* tz = configStore.getTimezone();
  configTzTime(tz, "pool.ntp.org", "time.google.com");
  lastMeasurementTime = millis();
}

uint8_t Scheduler::computeIntervalSlots(uint16_t* outSlots, uint8_t maxSlots) {
  uint8_t interval = configStore.getIntervalHours();
  uint16_t anchor = configStore.getAnchorTime();
  if (interval == 0) interval = 6;

  uint16_t intervalMins = (uint16_t)interval * 60;
  uint16_t first = anchor % intervalMins;

  uint8_t count = 0;
  for (uint16_t t = first; t < 1440 && count < maxSlots; t += intervalMins) {
    outSlots[count++] = t;
  }
  return count;
}

uint8_t Scheduler::buildEffectiveSlots(uint16_t* slots, uint8_t maxSlots) {
  uint8_t mode = configStore.getScheduleMode();
  if (mode == 2) return 0;  // Never — scheduling disabled
  if (mode == 1) {
    return computeIntervalSlots(slots, maxSlots);
  }
  // Custom mode
  uint8_t count = configStore.getScheduleCount();
  if (count > 8) count = 8;
  if (count > maxSlots) count = maxSlots;
  for (uint8_t i = 0; i < count; i++) {
    slots[i] = configStore.getScheduleTime(i);
  }
  return count;
}

void Scheduler::resetDailyFlags() {
  memset(alreadyRanToday, 0, sizeof(alreadyRanToday));
}

void Scheduler::loop() {
  if (!callback) return;

  // Check if NTP has synced
  if (!timeSynced) {
    time_t now = time(nullptr);
    if (now > MIN_VALID_EPOCH) {
      timeSynced = true;
      Serial.println("NTP time synced");

      struct tm timeinfo;
      getLocalTime(&timeinfo);
      Serial.printf("Current time: %02d:%02d:%02d\n",
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }
  }

  if (timeSynced) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return;

    uint16_t currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    uint8_t currentDay = timeinfo.tm_mday;

    // Reset daily flags at midnight
    if (currentDay != lastDay) {
      memset(alreadyRanToday, 0, sizeof(alreadyRanToday));
      lastDay = currentDay;
    }

    uint16_t slots[24];
    uint8_t count = buildEffectiveSlots(slots, 24);

    for (uint8_t i = 0; i < count; i++) {
      // Within a 2-minute window of schedule time, and not yet run today
      if (!alreadyRanToday[i] &&
          currentMinutes >= slots[i] &&
          currentMinutes < slots[i] + 2) {
        alreadyRanToday[i] = true;
        // Deduplication: skip if last measurement was less than 5 minutes ago
        if (millis() - lastMeasurementTime < 300000 && lastMeasurementTime > 0) {
          Serial.printf("Skipping schedule slot %d — last measurement was <5min ago\n", i);
          continue;
        }
        Serial.printf("Scheduled measurement %d triggered at %02d:%02d\n",
                       i, timeinfo.tm_hour, timeinfo.tm_min);
        lastMeasurementTime = millis();
        callback();
      }
    }
  } else if (configStore.getScheduleMode() != 2) {
    // Fallback: interval-based (skip if scheduling disabled)
    if (millis() - lastMeasurementTime >= FALLBACK_INTERVAL_MS) {
      Serial.println("Fallback interval measurement triggered");
      lastMeasurementTime = millis();
      callback();
    }
  }
}

bool Scheduler::isTimeSynced() {
  return timeSynced;
}

void Scheduler::onMeasurementDue(MeasurementCallback cb) {
  callback = cb;
}

String Scheduler::getNextMeasurementTime() {
  if (configStore.getScheduleMode() == 2) return "disabled";
  if (!timeSynced) return "NTP not synced";

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "unknown";

  uint16_t currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  uint16_t slots[24];
  uint8_t count = buildEffectiveSlots(slots, 24);
  uint16_t nextTime = 0xFFFF;

  // Find next schedule time today (skip already-ran slots)
  for (uint8_t i = 0; i < count; i++) {
    if (!alreadyRanToday[i] && slots[i] > currentMinutes && slots[i] < nextTime) {
      nextTime = slots[i];
    }
  }

  // If none found today, get the earliest tomorrow
  if (nextTime == 0xFFFF) {
    for (uint8_t i = 0; i < count; i++) {
      if (slots[i] < nextTime) {
        nextTime = slots[i];
      }
    }
  }

  if (nextTime == 0xFFFF) return "none";

  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", nextTime / 60, nextTime % 60);
  return String(buf);
}

uint8_t Scheduler::getNextScheduledTimestamps(uint32_t* outTimestamps, uint8_t maxCount) {
  if (!timeSynced || maxCount == 0) return 0;
  if (configStore.getScheduleMode() == 2) return 0;  // scheduling disabled

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return 0;

  uint16_t currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  uint16_t slots[24];
  uint8_t slotCount = buildEffectiveSlots(slots, 24);
  if (slotCount == 0) return 0;

  // Sort slots ascending
  for (uint8_t i = 0; i < slotCount - 1; i++)
    for (uint8_t j = i + 1; j < slotCount; j++)
      if (slots[j] < slots[i]) { uint16_t tmp = slots[i]; slots[i] = slots[j]; slots[j] = tmp; }

  // Get current midnight as Unix timestamp
  struct tm midnight = timeinfo;
  midnight.tm_hour = 0; midnight.tm_min = 0; midnight.tm_sec = 0;
  time_t midnightTs = mktime(&midnight);

  uint8_t out = 0;
  // First pass: remaining slots today
  for (uint8_t i = 0; i < slotCount && out < maxCount; i++) {
    if (slots[i] > currentMinutes) {
      outTimestamps[out++] = (uint32_t)midnightTs + (uint32_t)slots[i] * 60;
    }
  }
  // Subsequent days until we have enough
  for (uint8_t day = 1; out < maxCount && day <= 7; day++) {
    uint32_t dayBase = (uint32_t)midnightTs + (uint32_t)day * 86400;
    for (uint8_t i = 0; i < slotCount && out < maxCount; i++) {
      outTimestamps[out++] = dayBase + (uint32_t)slots[i] * 60;
    }
  }
  return out;
}

String Scheduler::getCurrentTime() {
  if (!timeSynced) return "";

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "";

  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return String(buf);
}
