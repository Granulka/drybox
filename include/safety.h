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

// Heater response watch (thermal runaway / detached thermistor)
#ifndef HEATER_WATCH_WINDOW_MS
#define HEATER_WATCH_WINDOW_MS 600000UL  // 10 min at full power...
#endif
const float HEATER_WATCH_MIN_RISE_C = 2.0;  // ...must raise the reading by this much
const float HEATER_WATCH_BAND_C = 5.0;      // not checked this close to the setpoint

/**
 * Detects a heater that runs at full power without the measured temperature
 * responding (thermistor fell off the heater, heater/MOSFET open, no 12V).
 */
class HeaterWatch {
private:
  bool active;
  unsigned long windowStart;
  float startTemp;

public:
  HeaterWatch() : active(false), windowStart(0), startTemp(0.0) {}

  void reset() { active = false; }

  /**
   * Call once per control step.
   * @param fullPower true when the heater output is saturated
   * @param tempC measured temperature
   * @param setC setpoint
   * @param now current time in milliseconds
   * @return true if the heater is not responding
   */
  bool update(bool fullPower, float tempC, float setC, unsigned long now) {
    if (!fullPower || isnan(tempC) || tempC >= setC - HEATER_WATCH_BAND_C) {
      active = false;
      return false;
    }
    if (!active || tempC - startTemp >= HEATER_WATCH_MIN_RISE_C) {
      active = true;
      windowStart = now;
      startTemp = tempC;
      return false;
    }
    return (now - windowStart) >= HEATER_WATCH_WINDOW_MS;
  }
};

#endif // SAFETY_H
