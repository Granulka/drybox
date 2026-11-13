#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * Simple PID Controller
 * Implements a PID controller with integral anti-windup
 * and derivative on measurement to reduce setpoint kick
 */
class SimplePID {
public:
  double sp = 0.0;     // setpoint
  double Kp = 8.0;     // proportional gain
  double Ki = 0.04;    // integral gain
  double Kd = 30.0;    // derivative gain
  double outMin = 0.0;
  double outMax = 255.0;

  double integral = 0.0;
  double lastInput = 0.0;
  double lastOutput = 0.0;
  bool initialized = false;
  unsigned long lastTime = 0;
  unsigned long sampleTimeMs = 1000;

  /**
   * Set output limits for clamping
   */
  void setOutputLimits(double mn, double mx) {
    outMin = mn;
    outMax = mx;
  }

  /**
   * Set PID tuning parameters
   */
  void setTunings(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
  }

  /**
   * Compute PID output
   * @param input Current process variable
   * @param out Output variable (PWM value)
   * @param currentTime Current time in milliseconds
   * @return true if output was computed (sample time elapsed)
   */
  bool compute(double input, double &out, unsigned long currentTime) {
    if (!initialized) {
      lastTime = currentTime;
      lastInput = input;
      lastOutput = 0.0;
      initialized = true;
      out = lastOutput;
      return true;
    }

    unsigned long dtMs = currentTime - lastTime;
    if (dtMs < sampleTimeMs) {
      out = lastOutput;
      return false;
    }

    double dt = dtMs / 1000.0;
    double error = sp - input;

    // Integral with anti-windup
    integral += (Ki * error * dt);
    if (integral > outMax) integral = outMax;
    if (integral < outMin) integral = outMin;

    // Derivative on measurement (reduces kick)
    double dInput = (input - lastInput) / dt;

    double output = Kp * error + integral - Kd * dInput;

    // Clamp output
    if (output > outMax) output = outMax;
    if (output < outMin) output = outMin;

    lastOutput = output;
    lastInput = input;
    lastTime = currentTime;
    out = output;
    return true;
  }

  /**
   * Reset PID state
   * @param input Current process variable
   * @param currentOutput Initial output value
   */
  void reset(double input, double currentOutput = 0.0) {
    initialized = false;
    integral = 0.0;
    lastInput = input;
    lastOutput = currentOutput;
  }

  /**
   * Get current integral value (for testing/debugging)
   */
  double getIntegral() const { return integral; }

  /**
   * Get last output value
   */
  double getLastOutput() const { return lastOutput; }
};

#endif // PID_CONTROLLER_H
