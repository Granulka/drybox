#include <unity.h>
#include "safety.h"
#include <math.h>

SafetyMonitor safety;

void setUp(void) {
  safety = SafetyMonitor();
}

void tearDown(void) {
  // Cleanup
}

void test_safety_initialization() {
  SafetyMonitor testSafety;

  TEST_ASSERT_FALSE(testSafety.isThermistorFaulted());
  TEST_ASSERT_EQUAL_UINT8(0, testSafety.getFaultStreak());
}

void test_safety_valid_temperature() {
  bool fault = safety.checkTemperatureFault(25.0);

  TEST_ASSERT_FALSE(fault);
  TEST_ASSERT_FALSE(safety.isThermistorFaulted());
  TEST_ASSERT_EQUAL_UINT8(0, safety.getFaultStreak());
}

void test_safety_overtemperature() {
  // Test temperature above MAX_SAFE_TEMPERATURE (90°C)
  bool fault = safety.checkTemperatureFault(95.0);

  TEST_ASSERT_TRUE(fault);
}

void test_safety_boundary_temperature() {
  // Test exactly at limit
  bool fault1 = safety.checkTemperatureFault(90.0);
  TEST_ASSERT_FALSE(fault1);

  // Test just above limit
  bool fault2 = safety.checkTemperatureFault(90.1);
  TEST_ASSERT_TRUE(fault2);

  // Test just below limit
  safety.reset();
  bool fault3 = safety.checkTemperatureFault(89.9);
  TEST_ASSERT_FALSE(fault3);
}

void test_safety_nan_temperature_single() {
  // Single NAN reading should not trigger fault
  bool fault = safety.checkTemperatureFault(NAN);

  TEST_ASSERT_FALSE(fault);
  TEST_ASSERT_FALSE(safety.isThermistorFaulted());
  TEST_ASSERT_EQUAL_UINT8(1, safety.getFaultStreak());
}

void test_safety_nan_temperature_streak() {
  // Multiple NAN readings should trigger fault after threshold

  // First 3 readings - no fault yet
  for (int i = 0; i < 3; i++) {
    bool fault = safety.checkTemperatureFault(NAN);
    TEST_ASSERT_FALSE(fault);
  }

  // 4th reading should trigger fault
  bool fault = safety.checkTemperatureFault(NAN);
  TEST_ASSERT_TRUE(fault);
  TEST_ASSERT_TRUE(safety.isThermistorFaulted());
  TEST_ASSERT_EQUAL_UINT8(4, safety.getFaultStreak());
}

void test_safety_fault_recovery() {
  // Trigger fault
  for (int i = 0; i < 4; i++) {
    safety.checkTemperatureFault(NAN);
  }
  TEST_ASSERT_TRUE(safety.isThermistorFaulted());

  // Valid reading should clear fault
  bool fault = safety.checkTemperatureFault(25.0);

  TEST_ASSERT_FALSE(fault);
  TEST_ASSERT_FALSE(safety.isThermistorFaulted());
  TEST_ASSERT_EQUAL_UINT8(0, safety.getFaultStreak());
}

void test_safety_fault_streak_reset() {
  // Build up fault streak
  safety.checkTemperatureFault(NAN);
  safety.checkTemperatureFault(NAN);
  TEST_ASSERT_EQUAL_UINT8(2, safety.getFaultStreak());

  // Valid reading resets streak
  safety.checkTemperatureFault(50.0);
  TEST_ASSERT_EQUAL_UINT8(0, safety.getFaultStreak());

  // Start new streak
  safety.checkTemperatureFault(NAN);
  TEST_ASSERT_EQUAL_UINT8(1, safety.getFaultStreak());
}

void test_safety_reset() {
  // Trigger fault
  for (int i = 0; i < 4; i++) {
    safety.checkTemperatureFault(NAN);
  }

  // Reset
  safety.reset();

  TEST_ASSERT_FALSE(safety.isThermistorFaulted());
  TEST_ASSERT_EQUAL_UINT8(0, safety.getFaultStreak());
}

void test_safety_is_temperature_safe_static() {
  // Test static method
  TEST_ASSERT_TRUE(SafetyMonitor::isTemperatureSafe(25.0));
  TEST_ASSERT_TRUE(SafetyMonitor::isTemperatureSafe(89.9));
  TEST_ASSERT_TRUE(SafetyMonitor::isTemperatureSafe(0.0));

  TEST_ASSERT_FALSE(SafetyMonitor::isTemperatureSafe(90.1));
  TEST_ASSERT_FALSE(SafetyMonitor::isTemperatureSafe(100.0));
  TEST_ASSERT_FALSE(SafetyMonitor::isTemperatureSafe(NAN));
  TEST_ASSERT_FALSE(SafetyMonitor::isTemperatureSafe(-1.0));
}

void test_safety_multiple_faults() {
  // Test that once faulted, subsequent NAN readings keep fault state
  for (int i = 0; i < 10; i++) {
    bool fault = safety.checkTemperatureFault(NAN);
    if (i >= 3) {
      TEST_ASSERT_TRUE(fault);
      TEST_ASSERT_TRUE(safety.isThermistorFaulted());
    }
  }
}

void test_safety_overtemp_doesnt_affect_streak() {
  // Overtemperature should trigger fault immediately
  // without affecting fault streak counter

  bool fault = safety.checkTemperatureFault(95.0);

  TEST_ASSERT_TRUE(fault);
  // Fault streak should still be 0 (overtemp is different from thermistor fault)
  TEST_ASSERT_EQUAL_UINT8(0, safety.getFaultStreak());
}

void test_safety_alternating_readings() {
  // Alternate between valid and invalid readings
  safety.checkTemperatureFault(NAN);
  TEST_ASSERT_EQUAL_UINT8(1, safety.getFaultStreak());

  safety.checkTemperatureFault(25.0);
  TEST_ASSERT_EQUAL_UINT8(0, safety.getFaultStreak());

  safety.checkTemperatureFault(NAN);
  TEST_ASSERT_EQUAL_UINT8(1, safety.getFaultStreak());

  safety.checkTemperatureFault(30.0);
  TEST_ASSERT_EQUAL_UINT8(0, safety.getFaultStreak());

  // Should never fault with alternating readings
  TEST_ASSERT_FALSE(safety.isThermistorFaulted());
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_safety_initialization);
  RUN_TEST(test_safety_valid_temperature);
  RUN_TEST(test_safety_overtemperature);
  RUN_TEST(test_safety_boundary_temperature);
  RUN_TEST(test_safety_nan_temperature_single);
  RUN_TEST(test_safety_nan_temperature_streak);
  RUN_TEST(test_safety_fault_recovery);
  RUN_TEST(test_safety_fault_streak_reset);
  RUN_TEST(test_safety_reset);
  RUN_TEST(test_safety_is_temperature_safe_static);
  RUN_TEST(test_safety_multiple_faults);
  RUN_TEST(test_safety_overtemp_doesnt_affect_streak);
  RUN_TEST(test_safety_alternating_readings);

  return UNITY_END();
}
