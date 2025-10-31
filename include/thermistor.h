#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <math.h>

// Thermistor Configuration
const float R_FIXED_RESISTOR = 6800.0;   // series resistor (ohms)
const float R_NOMINAL_THERM  = 100000.0; // thermistor at 25°C (ohms)
const float T_NOMINAL_THERM  = 25.0;     // nominal temp (°C)
const float BETA_THERM       = 3950.0;   // beta coefficient
const float ADC_MAX_VAL      = 1023.0;   // 10-bit ADC

/**
 * Calculate thermistor resistance from ADC reading
 * @param adc ADC reading (0-1023)
 * @return Resistance in ohms, or -1.0 if invalid
 */
inline float calculateThermistorResistance(int adc) {
  if (adc <= 0 || adc >= 1023) {
    return -1.0;  // Invalid reading
  }
  // Voltage divider: 5V -- R_FIXED -- A2 -- NTC -- GND
  return R_FIXED_RESISTOR * (ADC_MAX_VAL / adc - 1.0);
}

/**
 * Convert thermistor resistance to temperature using Beta equation
 * @param resistance Resistance in ohms
 * @return Temperature in Celsius, or NAN if invalid
 */
inline float resistanceToTemperature(float resistance) {
  if (resistance <= 0.0) {
    return NAN;
  }

  // Steinhart-Hart / Beta equation
  float steinhart = resistance / R_NOMINAL_THERM;
  steinhart = log(steinhart);
  steinhart /= BETA_THERM;
  steinhart += 1.0 / (T_NOMINAL_THERM + 273.15);
  steinhart = 1.0 / steinhart;

  return steinhart - 273.15;  // K to C
}

/**
 * Convert ADC reading directly to temperature
 * @param adc ADC reading (0-1023)
 * @return Temperature in Celsius, or NAN if invalid
 */
inline float adcToTemperature(int adc) {
  float resistance = calculateThermistorResistance(adc);
  if (resistance < 0.0) {
    return NAN;
  }
  return resistanceToTemperature(resistance);
}

#endif // THERMISTOR_H
