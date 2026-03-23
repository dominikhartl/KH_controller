#include <unity.h>
#include "gran_analysis.h"
#include <math.h>

extern "C" {
  void testSetBufferPH4(float v);
  void testSetBufferPH7(float v);
  void testSetBufferPH10(float v);
  void testResetConfig();
}

void test_ph4_at_25C(void) {
  testSetBufferPH4(4.0f);
  float result = bufferPHAtTemp(4, 25.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.000f, result);
}

void test_ph4_at_20C(void) {
  testSetBufferPH4(4.0f);
  // BUF4_TEMPCO = +0.001, dT = -5 => 4.0 + 0.001*(-5) = 3.995
  float result = bufferPHAtTemp(4, 20.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.995f, result);
}

void test_ph7_at_30C(void) {
  testSetBufferPH7(7.0f);
  // BUF7_TEMPCO = -0.002, dT = +5 => 7.0 + (-0.002)*5 = 6.990
  float result = bufferPHAtTemp(7, 30.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.990f, result);
}

void test_ph10_at_15C(void) {
  testSetBufferPH10(10.0f);
  // BUF10_TEMPCO = -0.010, dT = -10 => 10.0 + (-0.010)*(-10) = 10.100
  float result = bufferPHAtTemp(10, 15.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.100f, result);
}

void test_unknown_nominal(void) {
  // Nominal=5 not in switch -> returns (float)5
  float result = bufferPHAtTemp(5, 25.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, result);
}

void test_custom_buffer_ph4(void) {
  testSetBufferPH4(4.01f);
  float result = bufferPHAtTemp(4, 25.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.01f, result);
}

void test_extreme_cold_temp(void) {
  testSetBufferPH4(4.0f);
  // dT = 0 - 25 = -25. Result = 4.0 + 0.001*(-25) = 3.975
  float result = bufferPHAtTemp(4, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.975f, result);
}

void test_extreme_hot_temp(void) {
  testSetBufferPH10(10.0f);
  // dT = 40 - 25 = 15. Result = 10.0 + (-0.010)*15 = 9.850
  float result = bufferPHAtTemp(10, 40.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.850f, result);
}

void run_buffer_tests(void) {
  RUN_TEST(test_ph4_at_25C);
  RUN_TEST(test_ph4_at_20C);
  RUN_TEST(test_ph7_at_30C);
  RUN_TEST(test_ph10_at_15C);
  RUN_TEST(test_unknown_nominal);
  RUN_TEST(test_custom_buffer_ph4);
  RUN_TEST(test_extreme_cold_temp);
  RUN_TEST(test_extreme_hot_temp);
}
