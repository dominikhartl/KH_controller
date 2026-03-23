#include <unity.h>

// Test group runners (defined in each test file)
extern void run_gran_tests(void);
extern void run_interpolate_tests(void);
extern void run_buffer_tests(void);
extern void run_volume_tests(void);
extern void run_scheduler_tests(void);

// Test helper
extern "C" void testResetConfig(void);

void setUp(void) {
  testResetConfig();
}

void tearDown(void) {}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  run_gran_tests();
  run_interpolate_tests();
  run_buffer_tests();
  run_volume_tests();
  run_scheduler_tests();
  return UNITY_END();
}
