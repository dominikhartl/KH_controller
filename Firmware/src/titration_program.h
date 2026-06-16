#ifndef TITRATION_PROGRAM_H
#define TITRATION_PROGRAM_H

#include <stdint.h>
#include <stddef.h>

// One recorded titration step: a single titrate() call plus the non-pumping dwell
// that followed it (mix delay + stabilization + reading). Recorded during measureKH
// and replayed verbatim by titration-pump calibration, so the units->mL factor is
// produced under the same fast/gran mode-mix the device actually uses when measuring.
struct TitrationStep {
  uint16_t units;    // stepVol dosed
  uint8_t  mode;     // 0=fast, 1=medium, 2=gran (drives rpm/accel at replay)
  uint16_t dwellMs;  // non-pumping wall-time of this step
};

static const uint8_t TITR_MODE_FAST   = 0;
static const uint8_t TITR_MODE_MEDIUM = 1;
static const uint8_t TITR_MODE_GRAN   = 2;

// Max steps a recorded/replayed program can hold. Worst-case measurement step count
// (fast ramp + Gran tail) is a few hundred; 512 leaves headroom. ~3 KB BSS per buffer.
static const int MAX_PROGRAM_STEPS = 512;

// Serialized format (little-endian):
//   header: magic(4) version(2) count(2) srcUnixTs(4) = 12 bytes
//   body:   count x [ units(2) mode(1) dwellMs(2) ]    = 5 bytes/step
// Field-wise LE (not a struct memcpy) so layout is padding/endianness independent.
static const uint32_t TITR_PROG_MAGIC   = 0x4B485450u;  // "KHTP"
static const uint16_t TITR_PROG_VERSION = 1;
#define TITR_PROG_HEADER_BYTES 12
#define TITR_STEP_WIRE_BYTES   5

// Serialize `count` steps into out[]. Returns bytes written, or 0 if outCap is too
// small (or count < 0).
size_t titrationProgramSerialize(uint8_t* out, size_t outCap,
                                 const TitrationStep* steps, int count, uint32_t srcTs);

// Parse a serialized program: fills outSteps (up to maxSteps) and *outSrcTs.
// Returns the step count (>= 0) on success, or -1 if the buffer is invalid
// (bad magic/version, truncated, or more steps than maxSteps can hold).
int titrationProgramParse(const uint8_t* buf, size_t len,
                          TitrationStep* outSteps, int maxSteps, uint32_t* outSrcTs);

#endif // TITRATION_PROGRAM_H
