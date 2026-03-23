#ifndef PREFERENCES_H_STUB
#define PREFERENCES_H_STUB

#include <cstdint>
#include <cstddef>

class Preferences {
public:
  bool begin(const char*, bool = false) { return true; }
  void end() {}
  unsigned long getULong(const char*, unsigned long def = 0) { return def; }
  size_t putULong(const char*, unsigned long) { return 4; }
  bool getBool(const char*, bool def = false) { return def; }
  size_t putBool(const char*, bool) { return 1; }
  float getFloat(const char*, float def = 0) { return def; }
  size_t putFloat(const char*, float) { return 4; }
  int32_t getInt(const char*, int32_t def = 0) { return def; }
  size_t putInt(const char*, int32_t) { return 4; }
  uint8_t getUChar(const char*, uint8_t def = 0) { return def; }
  size_t putUChar(const char*, uint8_t) { return 1; }
  uint16_t getUShort(const char*, uint16_t def = 0) { return def; }
  size_t putUShort(const char*, uint16_t) { return 2; }
  size_t getString(const char*, char* buf, size_t len) { if (buf && len > 0) buf[0] = '\0'; return 0; }
  size_t putString(const char*, const char*) { return 0; }
  size_t putBytes(const char*, const void*, size_t) { return 0; }
  size_t getBytes(const char*, void*, size_t) { return 0; }
};

#endif // PREFERENCES_H_STUB
