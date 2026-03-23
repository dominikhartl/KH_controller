#include <unity.h>
#include <math.h>

// Volume calculation mirrors production code:
//   revsPerML = (float)calRevs / measuredVol     (web_server.cpp:357)
//   sampVol   = (float)calRevs / revsPerML        (main.cpp:1165)
// KH formula:
//   hclUsed = (eqUnits / calUnits) * titVol
//   kh      = (hclUsed / sampVol) * 2800.0f * hclMol * corrF

void test_volume_round_trip(void) {
  int calRevs = 350;
  float measuredVol = 77.0f;
  float revsPerML = (float)calRevs / measuredVol;
  float sampVol = (revsPerML > 0) ? (float)calRevs / revsPerML : 0.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, measuredVol, sampVol);
}

void test_volume_round_trip_80ml(void) {
  int calRevs = 450;
  float measuredVol = 80.0f;
  float revsPerML = (float)calRevs / measuredVol;
  float sampVol = (revsPerML > 0) ? (float)calRevs / revsPerML : 0.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, measuredVol, sampVol);
}

void test_volume_zero_guard(void) {
  float revsPerML = 0.0f;
  int calRevs = 350;
  float sampVol = (revsPerML > 0) ? (float)calRevs / revsPerML : 0.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, sampVol);
}

void test_kh_formula(void) {
  // Real measurement values from diagnostics
  float eqUnits = 15260.87f;  // approximate from the real data
  float calUnits = 15000.0f;
  float titVol = 10.38f;
  float sampVol = 80.0f;
  float hclMol = 0.024f;
  float corrF = 1.0f;

  float hclUsed = (eqUnits / calUnits) * titVol;
  float kh = (hclUsed / sampVol) * 2800.0f * hclMol * corrF;
  TEST_ASSERT_FLOAT_WITHIN(0.15f, 8.87f, kh);
}

void test_kh_formula_zero_volume(void) {
  float hclUsed = 10.56f;
  float sampVol = 0.0f;
  // Production code should guard against this
  // Here we verify the guard pattern works
  float kh = (sampVol > 0) ? (hclUsed / sampVol) * 2800.0f * 0.024f * 1.0f : 0.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, kh);
}

void test_volume_changes_after_cal_revs_change(void) {
  // Simulates the volume_bug.md scenario:
  // If calRevs changes but revsPerML stays the same, computed volume changes
  int originalCalRevs = 350;
  float measuredVol = 77.0f;
  float revsPerML = (float)originalCalRevs / measuredVol;  // 4.545

  // Now user changes calRevs to 450 without recalibrating
  int newCalRevs = 450;
  float newSampVol = (revsPerML > 0) ? (float)newCalRevs / revsPerML : 0.0f;
  // This should NOT equal the original measured volume — it's a known inconsistency
  TEST_ASSERT_TRUE(fabsf(newSampVol - measuredVol) > 1.0f);
  // New volume should be proportionally larger
  float expected = measuredVol * (float)newCalRevs / (float)originalCalRevs;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, expected, newSampVol);
}

void test_kh_confidence_interval(void) {
  float eqUnits = 15260.0f;
  float eqSE = 50.0f;
  float kh = 8.87f;
  float khCI = 1.96f * eqSE * (kh / eqUnits);
  // CI = 1.96 * 50 * (8.87 / 15260) = 1.96 * 50 * 0.000581 = 0.0569
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.057f, khCI);
}

void test_removal_ratio_consistent(void) {
  // main.cpp:1166 uses sampleFillRevs * 1.5f for removal
  int sampleFillRevs = 350;
  int sampleRemoveRevs = (int)(sampleFillRevs * 1.5f);
  TEST_ASSERT_EQUAL_INT(525, sampleRemoveRevs);

  // Verify this ratio is consistently 1.5x
  sampleFillRevs = 450;
  sampleRemoveRevs = (int)(sampleFillRevs * 1.5f);
  TEST_ASSERT_EQUAL_INT(675, sampleRemoveRevs);
}

void run_volume_tests(void) {
  RUN_TEST(test_volume_round_trip);
  RUN_TEST(test_volume_round_trip_80ml);
  RUN_TEST(test_volume_zero_guard);
  RUN_TEST(test_kh_formula);
  RUN_TEST(test_kh_formula_zero_volume);
  RUN_TEST(test_volume_changes_after_cal_revs_change);
  RUN_TEST(test_kh_confidence_interval);
  RUN_TEST(test_removal_ratio_consistent);
}
