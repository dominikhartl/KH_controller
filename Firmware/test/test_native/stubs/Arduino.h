#ifndef ARDUINO_H_STUB
#define ARDUINO_H_STUB

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>

// Arduino-style String class (minimal stub)
class String {
public:
  String() : _buf(nullptr), _len(0) {}
  String(const char* s) {
    if (s) { _len = strlen(s); _buf = (char*)malloc(_len + 1); strcpy(_buf, s); }
    else { _buf = nullptr; _len = 0; }
  }
  String(const String& o) {
    if (o._buf) { _len = o._len; _buf = (char*)malloc(_len + 1); strcpy(_buf, o._buf); }
    else { _buf = nullptr; _len = 0; }
  }
  String& operator=(const String& o) {
    if (this != &o) { free(_buf); if (o._buf) { _len = o._len; _buf = (char*)malloc(_len + 1); strcpy(_buf, o._buf); } else { _buf = nullptr; _len = 0; } }
    return *this;
  }
  ~String() { free(_buf); }
  const char* c_str() const { return _buf ? _buf : ""; }
  size_t length() const { return _len; }
  bool operator==(const char* s) const { return strcmp(c_str(), s ? s : "") == 0; }
  bool operator!=(const char* s) const { return !(*this == s); }
private:
  char* _buf;
  size_t _len;
};

// Serial stub
struct SerialStub {
  void begin(unsigned long) {}
  void print(const char*) {}
  void print(int) {}
  void println(const char* = "") {}
  void println(int) {}
  void printf(const char*, ...) {}
};
static SerialStub Serial;

// Time stubs
static unsigned long _stub_millis = 0;
inline unsigned long millis() { return _stub_millis; }
inline void setStubMillis(unsigned long v) { _stub_millis = v; }
inline void delay(unsigned long) {}

// NTP/time stubs
struct tm;
inline void configTzTime(const char*, const char* = nullptr, const char* = nullptr) {}
inline bool getLocalTime(struct tm* t) {
  memset(t, 0, sizeof(struct tm));
  return false;
}

#endif // ARDUINO_H_STUB
