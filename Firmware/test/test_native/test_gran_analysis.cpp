#include <unity.h>
#include "gran_analysis.h"
#include <math.h>
#include <string.h>

// Test helpers declared in stub_config_store.cpp
extern "C" {
  void testSetGranMinR2(float v);
  void testResetConfig();
}

// Real titration data from kh_diagnostics_20260315_2206.json (108 points)
// Config: sampleVol=80.0, titVol=10.38, calUnits=15000, granMinR2=0.99
static TitrationPoint realData[] = {
  {14255,4.971f,1858,0,0,0},
  {14277,4.949f,1862,167,2,0},
  {14299,4.935f,1864,167,2,0},
  {14321,4.929f,1865,166,2,0},
  {14343,4.919f,1867,167,2,0},
  {14365,4.908f,1869,167,2,0},
  {14387,4.902f,1870,166,2,0},
  {14409,4.894f,1871,167,2,0},
  {14431,4.884f,1872,167,2,0},
  {14453,4.880f,1874,167,2,0},
  {14475,4.872f,1875,167,2,0},
  {14497,4.861f,1877,167,2,0},
  {14519,4.851f,1878,167,2,0},
  {14541,4.846f,1879,167,2,0},
  {14563,4.836f,1881,166,2,0},
  {14585,4.827f,1883,167,2,0},
  {14607,4.816f,1885,167,2,0},
  {14629,4.809f,1886,167,2,0},
  {14651,4.803f,1887,167,2,0},
  {14673,4.791f,1889,166,2,0},
  {14695,4.780f,1891,167,2,0},
  {14717,4.776f,1891,167,2,0},
  {14739,4.763f,1894,166,2,0},
  {14761,4.751f,1897,167,2,0},
  {14783,4.745f,1897,167,2,0},
  {14805,4.732f,1899,167,2,0},
  {14827,4.714f,1902,167,2,0},
  {14849,4.704f,1904,167,2,0},
  {14871,4.700f,1904,167,2,0},
  {14893,4.689f,1907,167,2,0},
  {14915,4.674f,1909,166,2,0},
  {14937,4.666f,1911,167,2,0},
  {14959,4.651f,1914,166,2,0},
  {14981,4.632f,1916,167,2,0},
  {15003,4.623f,1918,167,2,0},
  {15025,4.607f,1921,166,2,0},
  {15047,4.595f,1923,167,2,0},
  {15069,4.584f,1925,167,2,0},
  {15091,4.571f,1927,167,2,0},
  {15113,4.552f,1931,167,2,0},
  {15135,4.517f,1936,166,2,0},
  {15157,4.526f,1935,167,2,0},
  {15179,4.512f,1938,167,2,0},
  {15201,4.505f,1939,166,2,0},
  {15223,4.492f,1941,167,2,0},
  {15245,4.470f,1945,167,2,0},
  {15267,4.464f,1946,167,2,0},
  {15289,4.447f,1949,167,2,0},
  {15311,4.426f,1953,168,2,0},
  {15333,4.413f,1954,167,2,0},
  {15355,4.399f,1957,167,2,0},
  {15377,4.377f,1961,167,2,0},
  {15399,4.367f,1963,167,2,0},
  {15421,4.352f,1965,167,2,0},
  {15443,4.332f,1969,167,2,0},
  {15465,4.312f,1972,167,2,0},
  {15487,4.297f,1975,167,2,0},
  {15509,4.276f,1978,167,2,0},
  {15531,4.267f,1980,167,2,0},
  {15553,4.251f,1983,165,2,0},
  {15575,4.233f,1986,167,2,0},
  {15597,4.218f,1989,167,2,0},
  {15619,4.204f,1991,166,2,0},
  {15641,4.191f,1994,167,2,0},
  {15663,4.171f,1997,167,2,0},
  {15685,4.158f,1999,167,2,0},
  {15707,4.139f,2002,167,2,0},
  {15729,4.127f,2005,167,2,0},
  {15751,4.112f,2007,167,2,0},
  {15773,4.099f,2009,167,2,0},
  {15795,4.081f,2013,167,2,0},
  {15817,4.078f,2013,167,2,0},
  {15839,4.059f,2016,167,2,0},
  {15861,4.044f,2019,167,2,0},
  {15883,4.034f,2020,167,2,0},
  {15905,4.020f,2023,167,2,0},
  {15927,4.002f,2026,167,2,0},
  {15949,3.987f,2029,167,2,0},
  {15971,3.977f,2030,167,2,0},
  {15993,3.962f,2033,167,2,0},
  {16015,3.955f,2034,167,2,0},
  {16037,3.947f,2036,167,2,0},
  {16059,3.929f,2039,167,2,0},
  {16081,3.915f,2041,167,2,0},
  {16103,3.906f,2043,167,2,0},
  {16125,3.894f,2045,167,2,0},
  {16147,3.883f,2047,167,2,0},
  {16169,3.872f,2049,167,2,0},
  {16191,3.860f,2051,167,2,0},
  {16213,3.849f,2053,167,2,0},
  {16235,3.841f,2054,167,2,0},
  {16257,3.832f,2056,167,2,0},
  {16279,3.824f,2057,167,2,0},
  {16301,3.813f,2059,167,2,0},
  {16323,3.803f,2061,167,2,0},
  {16345,3.791f,2063,167,2,0},
  {16367,3.791f,2063,167,2,0},
  {16389,3.778f,2065,167,2,0},
  {16411,3.767f,2067,167,2,0},
  {16433,3.766f,2067,168,2,0},
  {16455,3.755f,2069,167,2,0},
  {16477,3.743f,2071,167,2,0},
  {16499,3.732f,2073,167,2,0},
  {16521,3.732f,2073,167,2,0},
  {16543,3.718f,2075,167,2,0},
  {16565,3.710f,2077,167,2,0},
  {16587,3.702f,2078,167,2,0},
  {16609,3.696f,2080,167,2,0},
};
static const int REAL_DATA_COUNT = sizeof(realData) / sizeof(realData[0]);
static const float REAL_SAMPLE_VOL = 80.0f;
static const float REAL_TIT_VOL = 10.38f;
static const float REAL_CAL_UNITS = 15000.0f;
static const float REAL_HCL_MOL = 0.024f;

// --- Gran analysis with real data ---

void test_gran_real_data_returns_valid(void) {
  testSetGranMinR2(0.99f);
  float r2 = 0;
  float eq = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                          REAL_TIT_VOL, REAL_CAL_UNITS, &r2);
  TEST_ASSERT_FALSE(isnan(eq));
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.999f, r2);  // R² should be ~0.999
}

void test_gran_real_data_kh_value(void) {
  testSetGranMinR2(0.99f);
  float r2 = 0;
  float eq = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                          REAL_TIT_VOL, REAL_CAL_UNITS, &r2);
  TEST_ASSERT_FALSE(isnan(eq));
  float hclUsed = (eq / REAL_CAL_UNITS) * REAL_TIT_VOL;
  float kh = (hclUsed / REAL_SAMPLE_VOL) * 2800.0f * REAL_HCL_MOL * 1.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.15f, 8.87f, kh);  // Expected KH ~8.87
}

void test_gran_real_data_window_bounds(void) {
  testSetGranMinR2(0.99f);
  float r2 = 0, winLow = 0, winHigh = 0;
  float eq = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                          REAL_TIT_VOL, REAL_CAL_UNITS, &r2, &winLow, &winHigh);
  TEST_ASSERT_FALSE(isnan(eq));
  TEST_ASSERT_TRUE(winLow >= 3.5f && winLow <= 3.8f);
  TEST_ASSERT_TRUE(winHigh >= 4.0f && winHigh <= 4.5f);
}

void test_gran_real_data_eq_se(void) {
  testSetGranMinR2(0.99f);
  float r2 = 0, eqSE = 0;
  float eq = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                          REAL_TIT_VOL, REAL_CAL_UNITS, &r2,
                          nullptr, nullptr, nullptr, 0, nullptr, nullptr,
                          nullptr, nullptr, &eqSE);
  TEST_ASSERT_FALSE(isnan(eq));
  TEST_ASSERT_TRUE(eqSE > 0);
  TEST_ASSERT_TRUE(eqSE < 200.0f);
}

void test_gran_real_data_slope_positive(void) {
  testSetGranMinR2(0.99f);
  float r2 = 0, slope = 0, intercept = 0;
  float eq = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                          REAL_TIT_VOL, REAL_CAL_UNITS, &r2,
                          nullptr, nullptr, nullptr, 0, &slope, &intercept);
  TEST_ASSERT_FALSE(isnan(eq));
  TEST_ASSERT_TRUE(slope > 0);
}

void test_gran_real_data_windows_populated(void) {
  testSetGranMinR2(0.99f);
  float r2 = 0;
  GranWindowResult windows[MAX_GRAN_WINDOWS];
  int nWindows = 0;
  float eq = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                          REAL_TIT_VOL, REAL_CAL_UNITS, &r2,
                          nullptr, nullptr, nullptr, 0, nullptr, nullptr,
                          windows, &nWindows);
  TEST_ASSERT_FALSE(isnan(eq));
  TEST_ASSERT_TRUE(nWindows > 0);
  // At least one window should be valid
  bool anyValid = false;
  for (int i = 0; i < nWindows; i++) {
    if (windows[i].valid) anyValid = true;
  }
  TEST_ASSERT_TRUE(anyValid);
}

// --- Edge cases ---

void test_gran_too_few_points(void) {
  TitrationPoint pts[2] = {{100, 4.5f, 0, 0, 0, 0}, {200, 4.0f, 0, 0, 0, 0}};
  float r2 = 0;
  char reason[64] = "";
  float eq = granAnalysis(pts, 2, 80.0f, 10.0f, 15000.0f, &r2,
                          nullptr, nullptr, reason, sizeof(reason));
  TEST_ASSERT_TRUE(isnan(eq));
  TEST_ASSERT_EQUAL_STRING("Too few data points", reason);
}

void test_gran_zero_cal_units(void) {
  float r2 = 0;
  char reason[64] = "";
  float eq = granAnalysis(realData, REAL_DATA_COUNT, 80.0f, 10.0f, 0.0f, &r2,
                          nullptr, nullptr, reason, sizeof(reason));
  TEST_ASSERT_TRUE(isnan(eq));
  TEST_ASSERT_EQUAL_STRING("Invalid calibration units", reason);
}

void test_gran_no_points_in_region(void) {
  // All points have pH > 5.0 — none in Gran region
  TitrationPoint pts[10];
  for (int i = 0; i < 10; i++) {
    pts[i] = {(float)(1000 + i * 100), 6.0f + i * 0.1f, 0, 0, TITRATION_PHASE_GRAN, 0};
  }
  testSetGranMinR2(0.99f);
  float r2 = 0;
  char reason[64] = "";
  float eq = granAnalysis(pts, 10, 80.0f, 10.0f, 15000.0f, &r2,
                          nullptr, nullptr, reason, sizeof(reason));
  TEST_ASSERT_TRUE(isnan(eq));
}

void test_gran_outlier_rejection(void) {
  testSetGranMinR2(0.99f);
  // Get clean result
  float r2Clean = 0;
  float eqClean = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                               REAL_TIT_VOL, REAL_CAL_UNITS, &r2Clean);
  TEST_ASSERT_FALSE(isnan(eqClean));

  // Copy data and corrupt one point in the Gran region
  TitrationPoint corrupted[108];
  memcpy(corrupted, realData, sizeof(realData));
  // Point at index 80 is pH ~3.96 — in the Gran region. Set pH to 4.5 (big outlier)
  corrupted[80].pH = 4.5f;

  float r2Dirty = 0;
  float eqDirty = granAnalysis(corrupted, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                               REAL_TIT_VOL, REAL_CAL_UNITS, &r2Dirty);
  TEST_ASSERT_FALSE(isnan(eqDirty));
  // Result should be within 2% of clean due to outlier rejection
  float pctDiff = fabsf(eqDirty - eqClean) / eqClean * 100.0f;
  TEST_ASSERT_TRUE(pctDiff < 2.0f);
}

void test_gran_r2_threshold_respected(void) {
  testSetGranMinR2(0.9999f);  // Unrealistically high — no window will pass
  float r2 = 0;
  char reason[64] = "";
  float eq = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                          REAL_TIT_VOL, REAL_CAL_UNITS, &r2,
                          nullptr, nullptr, reason, sizeof(reason));
  TEST_ASSERT_TRUE(isnan(eq));
  // Window selection filters by R² threshold, so no valid window is found
  TEST_ASSERT_NOT_NULL(strstr(reason, "No valid Gran window"));
}

// --- granRegression direct tests ---

void test_gran_regression_min_points(void) {
  // Exactly 7 points in range (below MIN_GRAN_POINTS=8) -> false
  TitrationPoint pts[7];
  float k = REAL_TIT_VOL / REAL_CAL_UNITS;
  for (int i = 0; i < 7; i++) {
    pts[i] = {(float)(15500 + i * 50), 3.8f + i * 0.05f, 0, 0, TITRATION_PHASE_GRAN, 0};
  }
  bool excluded[7] = {};
  float slope, intercept, r2, ssRes;
  int count;
  bool ok = granRegression(pts, 7, 80.0f, k, excluded, 3.5f, 4.5f,
                           &slope, &intercept, &r2, &ssRes, &count);
  TEST_ASSERT_FALSE(ok);
}

void test_gran_regression_at_min_points(void) {
  // 8 points in range (== MIN_GRAN_POINTS) -> should succeed
  TitrationPoint pts[8];
  float k = REAL_TIT_VOL / REAL_CAL_UNITS;
  for (int i = 0; i < 8; i++) {
    pts[i] = {(float)(15500 + i * 50), 3.8f + i * 0.05f, 0, 0, TITRATION_PHASE_GRAN, 0};
  }
  bool excluded[8] = {};
  float slope, intercept, r2, ssRes;
  int count;
  bool ok = granRegression(pts, 8, 80.0f, k, excluded, 3.5f, 4.5f,
                           &slope, &intercept, &r2, &ssRes, &count);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT(8, count);
}

void test_gran_negative_slope_nan(void) {
  // Construct data where Gran function DECREASES with units (impossible in reality)
  // This should cause slope <= 0 -> NAN
  TitrationPoint pts[10];
  float k = 10.0f / 15000.0f;
  for (int i = 0; i < 10; i++) {
    // pH INCREASES as units increase -> Gran fn decreases (wrong direction)
    pts[i] = {(float)(15000 + i * 100), 3.5f + i * 0.1f, 0, 0, TITRATION_PHASE_GRAN, 0};
  }
  testSetGranMinR2(0.5f);  // Very low threshold to not mask the negative slope
  float r2 = 0;
  float eq = tryGranWindow(pts, 10, 80.0f, k, 3.4f, 4.6f, &r2);
  TEST_ASSERT_TRUE(isnan(eq));
}

void test_gran_phase_filter_excludes_unstabilized(void) {
  // Fast/medium-phase points (unstabilized, ±0.3 pH) inside the Gran window
  // must NOT affect the regression — only phase-2 points participate.
  testSetGranMinR2(0.99f);
  float r2Clean = 0;
  float eqClean = granAnalysis(realData, REAL_DATA_COUNT, REAL_SAMPLE_VOL,
                               REAL_TIT_VOL, REAL_CAL_UNITS, &r2Clean);
  TEST_ASSERT_FALSE(isnan(eqClean));

  // Append garbage fast-phase (0) and medium-phase (1) points inside the window
  TitrationPoint withJunk[REAL_DATA_COUNT + 4];
  memcpy(withJunk, realData, sizeof(realData));
  withJunk[REAL_DATA_COUNT + 0] = {15600.0f, 4.30f, 0, 0, TITRATION_PHASE_FAST, 0};
  withJunk[REAL_DATA_COUNT + 1] = {15800.0f, 4.45f, 0, 0, TITRATION_PHASE_FAST, 0};
  withJunk[REAL_DATA_COUNT + 2] = {16000.0f, 3.60f, 0, 0, TITRATION_PHASE_MEDIUM, 0};
  withJunk[REAL_DATA_COUNT + 3] = {16200.0f, 4.10f, 0, 0, TITRATION_PHASE_MEDIUM, 0};

  float r2Junk = 0;
  float eqJunk = granAnalysis(withJunk, REAL_DATA_COUNT + 4, REAL_SAMPLE_VOL,
                              REAL_TIT_VOL, REAL_CAL_UNITS, &r2Junk);
  TEST_ASSERT_FALSE(isnan(eqJunk));
  // Identical result: the junk points are invisible to the fit
  TEST_ASSERT_FLOAT_WITHIN(0.5f, eqClean, eqJunk);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, r2Clean, r2Junk);
}

void test_gran_regression_only_phase2_counted(void) {
  // 8 phase-2 points + 4 in-window phase-0 points -> regression count must be 8
  TitrationPoint pts[12];
  float k = REAL_TIT_VOL / REAL_CAL_UNITS;
  for (int i = 0; i < 8; i++) {
    pts[i] = {(float)(15500 + i * 50), 3.8f + i * 0.05f, 0, 0, TITRATION_PHASE_GRAN, 0};
  }
  for (int i = 8; i < 12; i++) {
    pts[i] = {(float)(15500 + i * 50), 3.9f, 0, 0, TITRATION_PHASE_FAST, 0};
  }
  bool excluded[12] = {};
  float slope, intercept, r2, ssRes;
  int count;
  bool ok = granRegression(pts, 12, 80.0f, k, excluded, 3.5f, 4.5f,
                           &slope, &intercept, &r2, &ssRes, &count);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_INT(8, count);
}

// --- Test runner ---
void run_gran_tests(void) {
  RUN_TEST(test_gran_real_data_returns_valid);
  RUN_TEST(test_gran_real_data_kh_value);
  RUN_TEST(test_gran_real_data_window_bounds);
  RUN_TEST(test_gran_real_data_eq_se);
  RUN_TEST(test_gran_real_data_slope_positive);
  RUN_TEST(test_gran_real_data_windows_populated);
  RUN_TEST(test_gran_too_few_points);
  RUN_TEST(test_gran_zero_cal_units);
  RUN_TEST(test_gran_no_points_in_region);
  RUN_TEST(test_gran_outlier_rejection);
  RUN_TEST(test_gran_r2_threshold_respected);
  RUN_TEST(test_gran_regression_min_points);
  RUN_TEST(test_gran_regression_at_min_points);
  RUN_TEST(test_gran_negative_slope_nan);
  RUN_TEST(test_gran_phase_filter_excludes_unstabilized);
  RUN_TEST(test_gran_regression_only_phase2_counted);
}
