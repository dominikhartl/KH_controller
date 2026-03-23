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
  {14255,4.971f,1858.4f,0,0,0},
  {14277,4.949f,1861.9f,167,2,0},
  {14299,4.935f,1864.3f,167,2,0},
  {14321,4.929f,1864.7f,166,2,0},
  {14343,4.919f,1866.9f,167,2,0},
  {14365,4.908f,1868.6f,167,2,0},
  {14387,4.902f,1869.6f,166,2,0},
  {14409,4.894f,1871.0f,167,2,0},
  {14431,4.884f,1872.5f,167,2,0},
  {14453,4.880f,1873.5f,167,2,0},
  {14475,4.872f,1874.8f,167,2,0},
  {14497,4.861f,1876.7f,167,2,0},
  {14519,4.851f,1878.5f,167,2,0},
  {14541,4.846f,1879.4f,167,2,0},
  {14563,4.836f,1880.9f,166,2,0},
  {14585,4.827f,1882.6f,167,2,0},
  {14607,4.816f,1884.6f,167,2,0},
  {14629,4.809f,1886.1f,167,2,0},
  {14651,4.803f,1886.9f,167,2,0},
  {14673,4.791f,1888.8f,166,2,0},
  {14695,4.780f,1890.7f,167,2,0},
  {14717,4.776f,1891.3f,167,2,0},
  {14739,4.763f,1893.9f,166,2,0},
  {14761,4.751f,1896.6f,167,2,0},
  {14783,4.745f,1897.2f,167,2,0},
  {14805,4.732f,1899.2f,167,2,0},
  {14827,4.714f,1902.3f,167,2,0},
  {14849,4.704f,1904.3f,167,2,0},
  {14871,4.700f,1904.5f,167,2,0},
  {14893,4.689f,1906.6f,167,2,0},
  {14915,4.674f,1909.4f,166,2,0},
  {14937,4.666f,1910.6f,167,2,0},
  {14959,4.651f,1913.6f,166,2,0},
  {14981,4.632f,1916.3f,167,2,0},
  {15003,4.623f,1918.4f,167,2,0},
  {15025,4.607f,1920.8f,166,2,0},
  {15047,4.595f,1922.9f,167,2,0},
  {15069,4.584f,1925.0f,167,2,0},
  {15091,4.571f,1927.3f,167,2,0},
  {15113,4.552f,1930.6f,167,2,0},
  {15135,4.517f,1935.8f,166,2,0},
  {15157,4.526f,1935.0f,167,2,0},
  {15179,4.512f,1937.6f,167,2,0},
  {15201,4.505f,1938.6f,166,2,0},
  {15223,4.492f,1940.8f,167,2,0},
  {15245,4.470f,1944.7f,167,2,0},
  {15267,4.464f,1945.7f,167,2,0},
  {15289,4.447f,1948.9f,167,2,0},
  {15311,4.426f,1952.7f,168,2,0},
  {15333,4.413f,1954.5f,167,2,0},
  {15355,4.399f,1957.0f,167,2,0},
  {15377,4.377f,1961.3f,167,2,0},
  {15399,4.367f,1962.7f,167,2,0},
  {15421,4.352f,1965.4f,167,2,0},
  {15443,4.332f,1968.7f,167,2,0},
  {15465,4.312f,1972.3f,167,2,0},
  {15487,4.297f,1974.7f,167,2,0},
  {15509,4.276f,1978.3f,167,2,0},
  {15531,4.267f,1980.3f,167,2,0},
  {15553,4.251f,1982.6f,165,2,0},
  {15575,4.233f,1986.0f,167,2,0},
  {15597,4.218f,1988.6f,167,2,0},
  {15619,4.204f,1991.2f,166,2,0},
  {15641,4.191f,1993.5f,167,2,0},
  {15663,4.171f,1996.7f,167,2,0},
  {15685,4.158f,1998.9f,167,2,0},
  {15707,4.139f,2002.4f,167,2,0},
  {15729,4.127f,2004.7f,167,2,0},
  {15751,4.112f,2007.1f,167,2,0},
  {15773,4.099f,2009.4f,167,2,0},
  {15795,4.081f,2012.7f,167,2,0},
  {15817,4.078f,2012.7f,167,2,0},
  {15839,4.059f,2016.5f,167,2,0},
  {15861,4.044f,2018.8f,167,2,0},
  {15883,4.034f,2020.5f,167,2,0},
  {15905,4.020f,2023.2f,167,2,0},
  {15927,4.002f,2026.5f,167,2,0},
  {15949,3.987f,2028.6f,167,2,0},
  {15971,3.977f,2030.5f,167,2,0},
  {15993,3.962f,2033.1f,167,2,0},
  {16015,3.955f,2034.4f,167,2,0},
  {16037,3.947f,2036.3f,167,2,0},
  {16059,3.929f,2038.8f,167,2,0},
  {16081,3.915f,2041.2f,167,2,0},
  {16103,3.906f,2042.8f,167,2,0},
  {16125,3.894f,2044.8f,167,2,0},
  {16147,3.883f,2046.6f,167,2,0},
  {16169,3.872f,2048.6f,167,2,0},
  {16191,3.860f,2050.7f,167,2,0},
  {16213,3.849f,2052.7f,167,2,0},
  {16235,3.841f,2054.2f,167,2,0},
  {16257,3.832f,2055.8f,167,2,0},
  {16279,3.824f,2057.2f,167,2,0},
  {16301,3.813f,2058.8f,167,2,0},
  {16323,3.803f,2060.6f,167,2,0},
  {16345,3.791f,2062.7f,167,2,0},
  {16367,3.791f,2062.6f,167,2,0},
  {16389,3.778f,2065.2f,167,2,0},
  {16411,3.767f,2067.0f,167,2,0},
  {16433,3.766f,2067.0f,168,2,0},
  {16455,3.755f,2069.3f,167,2,0},
  {16477,3.743f,2071.3f,167,2,0},
  {16499,3.732f,2072.9f,167,2,0},
  {16521,3.732f,2073.0f,167,2,0},
  {16543,3.718f,2075.4f,167,2,0},
  {16565,3.710f,2076.7f,167,2,0},
  {16587,3.702f,2078.5f,167,2,0},
  {16609,3.696f,2079.5f,167,2,0},
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
    pts[i] = {(float)(1000 + i * 100), 6.0f + i * 0.1f, 0, 0, 0, 0};
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
    pts[i] = {(float)(15500 + i * 50), 3.8f + i * 0.05f, 0, 0, 0, 0};
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
    pts[i] = {(float)(15500 + i * 50), 3.8f + i * 0.05f, 0, 0, 0, 0};
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
    pts[i] = {(float)(15000 + i * 100), 3.5f + i * 0.1f, 0, 0, 0, 0};
  }
  testSetGranMinR2(0.5f);  // Very low threshold to not mask the negative slope
  float r2 = 0;
  float eq = tryGranWindow(pts, 10, 80.0f, k, 3.4f, 4.6f, &r2);
  TEST_ASSERT_TRUE(isnan(eq));
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
}
