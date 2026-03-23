#include <unity.h>
#include "gran_analysis.h"
#include <math.h>

void test_interpolate_basic(void) {
  TitrationPoint pts[] = {
    {100, 5.0f, 0, 0, 0, 0},
    {200, 4.0f, 0, 0, 0, 0},
  };
  float result = interpolateAtPH(pts, 2, 4.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, result);
}

void test_interpolate_multi_point(void) {
  TitrationPoint pts[] = {
    {0,   8.0f, 0, 0, 0, 0},
    {100, 7.0f, 0, 0, 0, 0},
    {200, 5.0f, 0, 0, 0, 0},
    {300, 4.0f, 0, 0, 0, 0},
  };
  // Target 7.5 falls between pts[0] (8.0) and pts[1] (7.0)
  float result = interpolateAtPH(pts, 4, 7.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, result);

  // Target 4.5 falls between pts[2] (5.0) and pts[3] (4.0)
  float result2 = interpolateAtPH(pts, 4, 4.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 250.0f, result2);
}

void test_interpolate_no_crossing(void) {
  TitrationPoint pts[] = {
    {100, 5.0f, 0, 0, 0, 0},
    {200, 4.8f, 0, 0, 0, 0},
  };
  float result = interpolateAtPH(pts, 2, 4.0f);
  TEST_ASSERT_TRUE(isnan(result));
}

void test_interpolate_single_point(void) {
  TitrationPoint pts[] = {
    {100, 5.0f, 0, 0, 0, 0},
  };
  float result = interpolateAtPH(pts, 1, 4.5f);
  TEST_ASSERT_TRUE(isnan(result));
}

void test_interpolate_empty(void) {
  float result = interpolateAtPH(nullptr, 0, 4.5f);
  TEST_ASSERT_TRUE(isnan(result));
}

void test_interpolate_equal_ph_skip(void) {
  TitrationPoint pts[] = {
    {100, 5.0f, 0, 0, 0, 0},
    {200, 5.0f, 0, 0, 0, 0},
  };
  // Both points have same pH — division guard should skip
  float result = interpolateAtPH(pts, 2, 5.0f);
  TEST_ASSERT_TRUE(isnan(result));
}

void test_interpolate_exact_boundary(void) {
  TitrationPoint pts[] = {
    {100, 5.0f, 0, 0, 0, 0},
    {200, 4.5f, 0, 0, 0, 0},
  };
  // Target exactly at second point's pH (condition: prev > target && curr <= target)
  float result = interpolateAtPH(pts, 2, 4.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, result);
}

void run_interpolate_tests(void) {
  RUN_TEST(test_interpolate_basic);
  RUN_TEST(test_interpolate_multi_point);
  RUN_TEST(test_interpolate_no_crossing);
  RUN_TEST(test_interpolate_single_point);
  RUN_TEST(test_interpolate_empty);
  RUN_TEST(test_interpolate_equal_ph_skip);
  RUN_TEST(test_interpolate_exact_boundary);
}
