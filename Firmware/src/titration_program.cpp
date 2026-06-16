#include "titration_program.h"

// Little-endian field accessors — keep the wire format portable across the ESP32
// target and the native (host) test build.
static inline void wr16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static inline void wr32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

size_t titrationProgramSerialize(uint8_t* out, size_t outCap,
                                 const TitrationStep* steps, int count, uint32_t srcTs) {
  if (count < 0) return 0;
  size_t need = TITR_PROG_HEADER_BYTES + (size_t)count * TITR_STEP_WIRE_BYTES;
  if (outCap < need) return 0;

  wr32(out + 0, TITR_PROG_MAGIC);
  wr16(out + 4, TITR_PROG_VERSION);
  wr16(out + 6, (uint16_t)count);
  wr32(out + 8, srcTs);

  uint8_t* p = out + TITR_PROG_HEADER_BYTES;
  for (int i = 0; i < count; i++) {
    wr16(p, steps[i].units);   p += 2;
    *p++ = steps[i].mode;
    wr16(p, steps[i].dwellMs); p += 2;
  }
  return need;
}

int titrationProgramParse(const uint8_t* buf, size_t len,
                          TitrationStep* outSteps, int maxSteps, uint32_t* outSrcTs) {
  if (len < (size_t)TITR_PROG_HEADER_BYTES) return -1;
  if (rd32(buf + 0) != TITR_PROG_MAGIC)   return -1;
  if (rd16(buf + 4) != TITR_PROG_VERSION) return -1;

  int count = rd16(buf + 6);
  if (count > maxSteps) return -1;
  size_t need = TITR_PROG_HEADER_BYTES + (size_t)count * TITR_STEP_WIRE_BYTES;
  if (len < need) return -1;

  if (outSrcTs) *outSrcTs = rd32(buf + 8);
  const uint8_t* p = buf + TITR_PROG_HEADER_BYTES;
  for (int i = 0; i < count; i++) {
    outSteps[i].units   = rd16(p); p += 2;
    outSteps[i].mode    = *p++;
    outSteps[i].dwellMs = rd16(p); p += 2;
  }
  return count;
}
