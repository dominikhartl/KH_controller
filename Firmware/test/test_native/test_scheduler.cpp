#include <unity.h>
#include "scheduler.h"

extern "C" {
  void testSetIntervalHours(uint8_t h);
  void testSetAnchorTime(uint16_t m);
  void testResetConfig();
}

void test_6h_anchor_0(void) {
  testSetIntervalHours(6);
  testSetAnchorTime(0);
  uint16_t slots[24];
  uint8_t count = scheduler.computeIntervalSlots(slots, 24);
  TEST_ASSERT_EQUAL_UINT8(4, count);
  TEST_ASSERT_EQUAL_UINT16(0, slots[0]);
  TEST_ASSERT_EQUAL_UINT16(360, slots[1]);
  TEST_ASSERT_EQUAL_UINT16(720, slots[2]);
  TEST_ASSERT_EQUAL_UINT16(1080, slots[3]);
}

void test_6h_anchor_120(void) {
  testSetIntervalHours(6);
  testSetAnchorTime(120);
  uint16_t slots[24];
  uint8_t count = scheduler.computeIntervalSlots(slots, 24);
  TEST_ASSERT_EQUAL_UINT8(4, count);
  TEST_ASSERT_EQUAL_UINT16(120, slots[0]);
  TEST_ASSERT_EQUAL_UINT16(480, slots[1]);
  TEST_ASSERT_EQUAL_UINT16(840, slots[2]);
  TEST_ASSERT_EQUAL_UINT16(1200, slots[3]);
}

void test_1h_anchor_0(void) {
  testSetIntervalHours(1);
  testSetAnchorTime(0);
  uint16_t slots[24];
  uint8_t count = scheduler.computeIntervalSlots(slots, 24);
  TEST_ASSERT_EQUAL_UINT8(24, count);
  TEST_ASSERT_EQUAL_UINT16(0, slots[0]);
  TEST_ASSERT_EQUAL_UINT16(60, slots[1]);
  TEST_ASSERT_EQUAL_UINT16(1380, slots[23]);
}

void test_24h_anchor_480(void) {
  testSetIntervalHours(24);
  testSetAnchorTime(480);
  uint16_t slots[24];
  uint8_t count = scheduler.computeIntervalSlots(slots, 24);
  TEST_ASSERT_EQUAL_UINT8(1, count);
  TEST_ASSERT_EQUAL_UINT16(480, slots[0]);
}

void test_interval_0_defaults_6(void) {
  testSetIntervalHours(0);
  testSetAnchorTime(0);
  uint16_t slots[24];
  uint8_t count = scheduler.computeIntervalSlots(slots, 24);
  // interval=0 defaults to 6h -> 4 slots
  TEST_ASSERT_EQUAL_UINT8(4, count);
  TEST_ASSERT_EQUAL_UINT16(0, slots[0]);
  TEST_ASSERT_EQUAL_UINT16(360, slots[1]);
}

void test_max_slots_limit(void) {
  testSetIntervalHours(1);
  testSetAnchorTime(0);
  uint16_t slots[3];
  uint8_t count = scheduler.computeIntervalSlots(slots, 3);
  TEST_ASSERT_EQUAL_UINT8(3, count);
  TEST_ASSERT_EQUAL_UINT16(0, slots[0]);
  TEST_ASSERT_EQUAL_UINT16(60, slots[1]);
  TEST_ASSERT_EQUAL_UINT16(120, slots[2]);
}

void test_3h_anchor_1200(void) {
  testSetIntervalHours(3);
  testSetAnchorTime(1200);
  uint16_t slots[24];
  uint8_t count = scheduler.computeIntervalSlots(slots, 24);
  // interval=180min, first = 1200 % 180 = 120
  // slots: 120, 300, 480, 660, 840, 1020, 1200, 1380
  TEST_ASSERT_EQUAL_UINT8(8, count);
  TEST_ASSERT_EQUAL_UINT16(120, slots[0]);
  TEST_ASSERT_EQUAL_UINT16(300, slots[1]);
  TEST_ASSERT_EQUAL_UINT16(1380, slots[7]);
}

void run_scheduler_tests(void) {
  RUN_TEST(test_6h_anchor_0);
  RUN_TEST(test_6h_anchor_120);
  RUN_TEST(test_1h_anchor_0);
  RUN_TEST(test_24h_anchor_480);
  RUN_TEST(test_interval_0_defaults_6);
  RUN_TEST(test_max_slots_limit);
  RUN_TEST(test_3h_anchor_1200);
}
