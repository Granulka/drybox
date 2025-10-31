#ifndef SAFETY_H
#define SAFETY_H

#include <math.h>

// Safety Configuration
const float MAX_SAFE_TEMPERATURE = 90.0;  // °C
const unsigned FAULT_SAMPLE_THRESHOLD = 4;

/**
 * Safety monitor for temperature control
 */
class SafetyMonitor {
private:
  uint8_t faultStreak;
  bool thermistorFault;

public:
  SafetyMonitor() : faultStreak(0), thermistorFault(false) {}

  /**
   * Check if temperature reading indicates a fault
   * @param tempC Temperature reading
   * @return true if fault detected
   */
  bool checkTemperatureFault(float tempC) {
    // Check for invalid reading
    if (isnan(tempC)) {
      if (++faultStreak >= FAULT_SAMPLE_THRESHOLD) {
        thermistorFault = true;
        return true;
      }
      return thermistorFault;
    }

    // Valid reading, reset fault streak
    faultStreak = 0;
    thermistorFault = false;

    // Check for overtemperature
    if (tempC > MAX_SAFE_TEMPERATURE) {
      return true;
    }

    return false;
  }

  /**
   * Check if thermistor has faulted
   */
  bool isThermistorFaulted() const {
    return thermistorFault;
  }

  /**
   * Get current fault streak count
   */
  uint8_t getFaultStreak() const {
    return faultStreak;
  }

  /**
   * Reset safety monitor state
   */
  void reset() {
    faultStreak = 0;
    thermistorFault = false;
  }

  /**
   * Check if a temperature is within safe operating range
   */
  static bool isTemperatureSafe(float tempC) {
    if (isnan(tempC)) return false;
    return tempC <= MAX_SAFE_TEMPERATURE && tempC >= 0.0;
  }
};

#endif // SAFETY_H
