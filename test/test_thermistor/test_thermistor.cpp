#include <unity.h>
#include "thermistor.h"
#include <math.h>

void setUp(void) {
  // Set up code here, runs before each test
}

void tearDown(void) {
  // Clean up code here, runs after each test
}

// Test thermistor resistance calculation
// Divider: 5V -- R_FIXED -- A2 -- NTC -- GND, so R_ntc = R_FIXED * adc / (1023 - adc)
// ADC reading the divider produces for a given thermistor resistance
static int adcFor(float rNtc) {
  return (int)(ADC_MAX_VAL * rNtc / (R_FIXED_RESISTOR + rNtc) + 0.5);
}

void test_thermistor_resistance_calculation() {
  // 100k (25°C nominal), within ADC quantisation
  TEST_ASSERT_FLOAT_WITHIN(1500.0, 100000.0, calculateThermistorResistance(adcFor(100000.0)));
  // Equal resistances at mid-scale
  TEST_ASSERT_FLOAT_WITHIN(R_FIXED_RESISTOR * 0.01, R_FIXED_RESISTOR, calculateThermistorResistance(512));
}

void test_thermistor_resistance_invalid_adc() {
  // Test boundary conditions
  TEST_ASSERT_EQUAL_FLOAT(-1.0, calculateThermistorResistance(0));
  TEST_ASSERT_EQUAL_FLOAT(-1.0, calculateThermistorResistance(1023));
  TEST_ASSERT_EQUAL_FLOAT(-1.0, calculateThermistorResistance(-1));
  TEST_ASSERT_EQUAL_FLOAT(-1.0, calculateThermistorResistance(1024));
}

void test_resistance_to_temperature() {
  // Test with nominal resistance (100k at 25°C)
  float temp = resistanceToTemperature(100000.0);
  TEST_ASSERT_FLOAT_WITHIN(0.5, 25.0, temp);

  // Test with lower resistance (higher temp)
  // At 50°C: R = R0 * exp(B * (1/T - 1/T0))
  // R = 100000 * exp(3950 * (1/323.15 - 1/298.15))
  // R = 100000 * exp(3950 * (0.003095 - 0.003354))
  // R = 100000 * exp(3950 * -0.000259)
  // R = 100000 * exp(-1.023) = 100000 * 0.359 = 35900 ohms
  float temp_50c = resistanceToTemperature(35900.0);
  TEST_ASSERT_FLOAT_WITHIN(2.0, 50.0, temp_50c);

  // Test invalid resistance
  TEST_ASSERT_TRUE(isnan(resistanceToTemperature(0.0)));
  TEST_ASSERT_TRUE(isnan(resistanceToTemperature(-100.0)));
}

void test_adc_to_temperature_room_temp() {
  // 100k NTC at 25°C
  TEST_ASSERT_FLOAT_WITHIN(1.0, 25.0, adcToTemperature(adcFor(100000.0)));
  // Hotter thermistor -> lower resistance -> lower ADC (≈ 9.34k at 90°C)
  TEST_ASSERT_FLOAT_WITHIN(1.0, 90.0, adcToTemperature(adcFor(9340.0)));
  // Calibration point measured on the board: ADC 923 with the DHT11 at 22°C
  TEST_ASSERT_FLOAT_WITHIN(2.0, 22.0, adcToTemperature(923));
}

void test_adc_to_temperature_invalid() {
  // Test invalid ADC values
  TEST_ASSERT_TRUE(isnan(adcToTemperature(0)));
  TEST_ASSERT_TRUE(isnan(adcToTemperature(1023)));
  TEST_ASSERT_TRUE(isnan(adcToTemperature(-5)));
}

void test_temperature_range_validation() {
  // Working range of the divider produces reasonable temperatures
  for (int adc = 300; adc <= 1000; adc += 100) {
    float temp = adcToTemperature(adc);
    TEST_ASSERT_FALSE(isnan(temp));
    TEST_ASSERT_GREATER_THAN(-50.0, temp);
    TEST_ASSERT_LESS_THAN(200.0, temp);
  }
}

void test_thermistor_monotonic_behavior() {
  // NTC on the low side: higher ADC = higher resistance = lower temperature
  TEST_ASSERT_GREATER_THAN(calculateThermistorResistance(400), calculateThermistorResistance(600));
  TEST_ASSERT_GREATER_THAN(calculateThermistorResistance(600), calculateThermistorResistance(800));
  TEST_ASSERT_LESS_THAN(adcToTemperature(600), adcToTemperature(800));
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_thermistor_resistance_calculation);
  RUN_TEST(test_thermistor_resistance_invalid_adc);
  RUN_TEST(test_resistance_to_temperature);
  RUN_TEST(test_adc_to_temperature_room_temp);
  RUN_TEST(test_adc_to_temperature_invalid);
  RUN_TEST(test_temperature_range_validation);
  RUN_TEST(test_thermistor_monotonic_behavior);

  return UNITY_END();
}
