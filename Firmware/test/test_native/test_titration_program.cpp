#include <unity.h>
#include "titration_program.h"
#include "default_titration_program.h"
#include <string.h>

// --- Serialize / parse round-trip ---

void test_program_round_trip(void) {
  TitrationStep in[3] = {
    {1035, TITR_MODE_FAST,   500},
    {52,   TITR_MODE_MEDIUM, 6000},
    {33,   TITR_MODE_GRAN,   5168},
  };
  uint8_t buf[64];
  size_t n = titrationProgramSerialize(buf, sizeof(buf), in, 3, 1781206788u);
  TEST_ASSERT_EQUAL_size_t(TITR_PROG_HEADER_BYTES + 3 * TITR_STEP_WIRE_BYTES, n);

  TitrationStep out[8];
  uint32_t srcTs = 0;
  int count = titrationProgramParse(buf, n, out, 8, &srcTs);
  TEST_ASSERT_EQUAL_INT(3, count);
  TEST_ASSERT_EQUAL_UINT32(1781206788u, srcTs);
  for (int i = 0; i < 3; i++) {
    TEST_ASSERT_EQUAL_UINT16(in[i].units, out[i].units);
    TEST_ASSERT_EQUAL_UINT8(in[i].mode, out[i].mode);
    TEST_ASSERT_EQUAL_UINT16(in[i].dwellMs, out[i].dwellMs);
  }
}

// --- Rejects a buffer with the wrong magic ---

void test_program_parse_rejects_bad_magic(void) {
  TitrationStep in[1] = {{33, TITR_MODE_GRAN, 5000}};
  uint8_t buf[32];
  size_t n = titrationProgramSerialize(buf, sizeof(buf), in, 1, 100u);
  buf[0] ^= 0xFF;  // corrupt magic
  TitrationStep out[1];
  uint32_t srcTs = 0;
  TEST_ASSERT_EQUAL_INT(-1, titrationProgramParse(buf, n, out, 1, &srcTs));
}

// --- Rejects a truncated buffer (count says more steps than bytes present) ---

void test_program_parse_rejects_truncated(void) {
  TitrationStep in[3] = {
    {1035, TITR_MODE_FAST, 500},
    {52, TITR_MODE_MEDIUM, 6000},
    {33, TITR_MODE_GRAN, 5168},
  };
  uint8_t buf[64];
  size_t n = titrationProgramSerialize(buf, sizeof(buf), in, 3, 1u);
  TitrationStep out[8];
  uint32_t srcTs = 0;
  // Chop off the last step's bytes — header still claims 3 steps.
  TEST_ASSERT_EQUAL_INT(-1, titrationProgramParse(buf, n - TITR_STEP_WIRE_BYTES, out, 8, &srcTs));
}

// --- Serialize fails (returns 0) when the output buffer is too small ---

void test_program_serialize_buffer_too_small(void) {
  TitrationStep in[3] = {
    {1035, TITR_MODE_FAST, 500},
    {52, TITR_MODE_MEDIUM, 6000},
    {33, TITR_MODE_GRAN, 5168},
  };
  uint8_t small[TITR_PROG_HEADER_BYTES + 2 * TITR_STEP_WIRE_BYTES];  // room for 2, need 3
  TEST_ASSERT_EQUAL_size_t(0, titrationProgramSerialize(small, sizeof(small), in, 3, 1u));
}

// --- Parse rejects a program with more steps than the caller's buffer can hold ---

void test_program_parse_rejects_overflow(void) {
  TitrationStep in[3] = {
    {1035, TITR_MODE_FAST, 500},
    {52, TITR_MODE_MEDIUM, 6000},
    {33, TITR_MODE_GRAN, 5168},
  };
  uint8_t buf[64];
  size_t n = titrationProgramSerialize(buf, sizeof(buf), in, 3, 1u);
  TitrationStep out[2];  // too small for 3 steps
  uint32_t srcTs = 0;
  TEST_ASSERT_EQUAL_INT(-1, titrationProgramParse(buf, n, out, 2, &srcTs));
}

// --- Embedded default program (real captured run) sanity ---

void test_default_program_length(void) {
  TEST_ASSERT_EQUAL_INT(95, DEFAULT_TITRATION_PROGRAM_LEN);
}

void test_default_program_has_no_medium_steps(void) {
  // With fast_ph == GRAN_REGION_PH the measurement emits only fast + gran steps.
  for (int i = 0; i < DEFAULT_TITRATION_PROGRAM_LEN; i++) {
    TEST_ASSERT_NOT_EQUAL_UINT8(TITR_MODE_MEDIUM, DEFAULT_TITRATION_PROGRAM[i].mode);
  }
}

void test_default_program_modes_valid(void) {
  for (int i = 0; i < DEFAULT_TITRATION_PROGRAM_LEN; i++) {
    uint8_t m = DEFAULT_TITRATION_PROGRAM[i].mode;
    TEST_ASSERT_TRUE(m == TITR_MODE_FAST || m == TITR_MODE_GRAN);
  }
}

void test_default_program_total_units_in_range(void) {
  long total = 0;
  for (int i = 0; i < DEFAULT_TITRATION_PROGRAM_LEN; i++) {
    total += DEFAULT_TITRATION_PROGRAM[i].units;
  }
  // Captured run summed to ~14,731 units (~11.4 mL).
  TEST_ASSERT_TRUE(total > 14000 && total < 15500);
}

void run_titration_program_tests(void) {
  RUN_TEST(test_program_round_trip);
  RUN_TEST(test_program_parse_rejects_bad_magic);
  RUN_TEST(test_program_parse_rejects_truncated);
  RUN_TEST(test_program_serialize_buffer_too_small);
  RUN_TEST(test_program_parse_rejects_overflow);
  RUN_TEST(test_default_program_length);
  RUN_TEST(test_default_program_has_no_medium_steps);
  RUN_TEST(test_default_program_modes_valid);
  RUN_TEST(test_default_program_total_units_in_range);
}
