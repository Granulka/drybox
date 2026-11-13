#include <unity.h>
#include "pid_controller.h"

SimplePID pid;

void setUp(void) {
  // Reset PID before each test
  pid = SimplePID();
  pid.setOutputLimits(0, 255);
}

void tearDown(void) {
  // Cleanup after each test
}

void test_pid_initialization() {
  SimplePID testPid;

  TEST_ASSERT_EQUAL_DOUBLE(0.0, testPid.sp);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, testPid.integral);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, testPid.lastOutput);
  TEST_ASSERT_FALSE(testPid.initialized);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, testPid.outMin);
  TEST_ASSERT_EQUAL_DOUBLE(255.0, testPid.outMax);
}

void test_pid_set_tunings() {
  pid.setTunings(10.0, 0.5, 5.0);

  TEST_ASSERT_EQUAL_DOUBLE(10.0, pid.Kp);
  TEST_ASSERT_EQUAL_DOUBLE(0.5, pid.Ki);
  TEST_ASSERT_EQUAL_DOUBLE(5.0, pid.Kd);
}

void test_pid_set_output_limits() {
  pid.setOutputLimits(50.0, 200.0);

  TEST_ASSERT_EQUAL_DOUBLE(50.0, pid.outMin);
  TEST_ASSERT_EQUAL_DOUBLE(200.0, pid.outMax);
}

void test_pid_first_computation() {
  pid.sp = 50.0;
  double output;
  unsigned long time = 1000;

  bool computed = pid.compute(25.0, output, time);

  TEST_ASSERT_TRUE(computed);
  TEST_ASSERT_TRUE(pid.initialized);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, output);  // First call returns 0
}

void test_pid_proportional_response() {
  pid.sp = 50.0;
  pid.Kp = 2.0;
  pid.Ki = 0.0;  // Disable integral
  pid.Kd = 0.0;  // Disable derivative

  double output;

  // First call initializes
  pid.compute(30.0, output, 1000);

  // Second call should have proportional response
  // Error = 50 - 30 = 20
  // Output = Kp * error = 2.0 * 20 = 40
  pid.compute(30.0, output, 2000);

  TEST_ASSERT_EQUAL_DOUBLE(40.0, output);
}

void test_pid_integral_accumulation() {
  pid.sp = 50.0;
  pid.Kp = 0.0;  // Disable proportional
  pid.Ki = 0.1;
  pid.Kd = 0.0;  // Disable derivative

  double output;

  // Initialize
  pid.compute(40.0, output, 1000);

  // After 1 second with error of 10
  // Integral = Ki * error * dt = 0.1 * 10 * 1.0 = 1.0
  pid.compute(40.0, output, 2000);
  TEST_ASSERT_FLOAT_WITHIN(0.1, 1.0, output);

  // After another second
  // Integral = 1.0 + 0.1 * 10 * 1.0 = 2.0
  pid.compute(40.0, output, 3000);
  TEST_ASSERT_FLOAT_WITHIN(0.1, 2.0, output);
}

void test_pid_derivative_on_measurement() {
  pid.sp = 50.0;
  pid.Kp = 0.0;  // Disable proportional
  pid.Ki = 0.0;  // Disable integral
  pid.Kd = 10.0;

  double output;

  // Initialize at 30°C
  pid.compute(30.0, output, 1000);

  // Temperature rises to 35°C in 1 second
  // dInput = (35 - 30) / 1.0 = 5.0
  // Output = -Kd * dInput = -10.0 * 5.0 = -50.0 (clamped to 0)
  pid.compute(35.0, output, 2000);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, output);  // Clamped to minimum
}

void test_pid_output_clamping() {
  pid.sp = 100.0;
  pid.Kp = 20.0;  // High gain to force clamping
  pid.Ki = 0.0;
  pid.Kd = 0.0;

  double output;

  // Initialize
  pid.compute(10.0, output, 1000);

  // Error = 90, Output = 20 * 90 = 1800 (should clamp to 255)
  pid.compute(10.0, output, 2000);
  TEST_ASSERT_EQUAL_DOUBLE(255.0, output);
}

void test_pid_integral_anti_windup() {
  pid.sp = 100.0;
  pid.Kp = 0.0;
  pid.Ki = 100.0;  // Very high Ki to cause windup
  pid.Kd = 0.0;
  pid.setOutputLimits(0, 255);

  double output;

  // Initialize
  pid.compute(50.0, output, 1000);

  // Large error for multiple iterations
  for (int i = 1; i <= 10; i++) {
    pid.compute(50.0, output, 1000 + i * 1000);
  }

  // Integral should be clamped to output max
  TEST_ASSERT_LESS_OR_EQUAL(255.0, pid.getIntegral());
  TEST_ASSERT_EQUAL_DOUBLE(255.0, output);
}

void test_pid_reset() {
  pid.sp = 50.0;
  double output;

  // Run PID for a bit
  pid.compute(30.0, output, 1000);
  pid.compute(30.0, output, 2000);
  pid.compute(30.0, output, 3000);

  // Reset
  pid.reset(40.0, 100.0);

  TEST_ASSERT_FALSE(pid.initialized);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, pid.getIntegral());
  TEST_ASSERT_EQUAL_DOUBLE(40.0, pid.lastInput);
  TEST_ASSERT_EQUAL_DOUBLE(100.0, pid.getLastOutput());
}

void test_pid_sample_time_enforcement() {
  pid.sp = 50.0;
  pid.sampleTimeMs = 1000;

  double output;

  // Initialize
  pid.compute(30.0, output, 1000);
  double firstOutput = output;

  // Try to compute before sample time elapsed
  bool computed = pid.compute(35.0, output, 1500);

  TEST_ASSERT_FALSE(computed);
  TEST_ASSERT_EQUAL_DOUBLE(firstOutput, output);  // Should return last output
}

void test_pid_combined_response() {
  // Test realistic PID behavior with all terms
  pid.sp = 50.0;
  pid.Kp = 5.0;
  pid.Ki = 0.1;
  pid.Kd = 10.0;

  double output;
  double temp = 25.0;

  // Initialize
  pid.compute(temp, output, 0);

  // Simulate heating over time
  for (int t = 1; t <= 10; t++) {
    // Simulate temperature rising slowly
    temp += 0.5;
    pid.compute(temp, output, t * 1000);

    // Output should be positive (heating)
    TEST_ASSERT_GREATER_OR_EQUAL(0.0, output);
    TEST_ASSERT_LESS_OR_EQUAL(255.0, output);
  }
}

void test_pid_reaches_setpoint() {
  // Simplified test: as we approach setpoint, output should decrease
  pid.sp = 50.0;
  pid.Kp = 8.0;
  pid.Ki = 0.04;
  pid.Kd = 30.0;

  double output1, output2, output3;

  pid.compute(30.0, output1, 0);     // Initialize
  pid.compute(30.0, output1, 1000);  // Far from setpoint
  pid.compute(45.0, output2, 2000);  // Closer to setpoint
  pid.compute(49.0, output3, 3000);  // Very close to setpoint

  // Output should generally decrease as we approach setpoint
  // (Though derivative term can cause temporary increases)
  TEST_ASSERT_GREATER_THAN(output3, output1);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_pid_initialization);
  RUN_TEST(test_pid_set_tunings);
  RUN_TEST(test_pid_set_output_limits);
  RUN_TEST(test_pid_first_computation);
  RUN_TEST(test_pid_proportional_response);
  RUN_TEST(test_pid_integral_accumulation);
  RUN_TEST(test_pid_derivative_on_measurement);
  RUN_TEST(test_pid_output_clamping);
  RUN_TEST(test_pid_integral_anti_windup);
  RUN_TEST(test_pid_reset);
  RUN_TEST(test_pid_sample_time_enforcement);
  RUN_TEST(test_pid_combined_response);
  RUN_TEST(test_pid_reaches_setpoint);

  return UNITY_END();
}
