#include "BarometricSensor.h"
#include <math.h>

/**
 * @brief Recalibrates the ground pressure and temperature references (P0, T0).
 * @details This function reads the current pressure and temperature from the sensor and updates
 *          the ground pressure and temperature references.
 * @param numReadings The number of readings to average for the calibration.
 */
void BarometricSensor::calibrateGroundReference(int numReadings) {
    long accumulatedPressure = 0;
    float accumulatedTemperature = 0.0f;

    for (int i = 0; i < numReadings; i++) {
        float p = getPressurePa();
        accumulatedPressure += p;
        accumulatedTemperature += getTemperatureC();
        delay(20);
    }

    if (numReadings > 0) {
        groundPressureP0_Pa = (float)accumulatedPressure / numReadings;
        groundTemperatureT0_K = (accumulatedTemperature / numReadings) + 273.15f;
    }
}

/**
 * @brief Calculates altitude from pressure using the barometric formula.
 * @details Uses the international standard atmosphere model to convert pressure readings
 *          to altitude above sea level.
 * @param pressure_pa The current pressure reading in Pascals.
 * @return The calculated altitude in meters.
 */
float BarometricSensor::altitudeFromPressure(float pressure_pa) {
    if (pressure_pa <= 0 || groundPressureP0_Pa <= 0) return -9999.0f;

    float pressureRatio = pressure_pa / groundPressureP0_Pa;
    if (pressureRatio <= 0) return -9999.0f;

    // Barometric Equation (International Standard Atmosphere)
    return (1.0f - pow(pressureRatio, ISA_EXPONENT)) * (groundTemperatureT0_K / L_ISA);
}

/**
 * @brief Reads the current pressure from the sensor and calculates altitude.
 * @details Calculates altitude based on the current pressure reading from the sensor
 *          and the stored ground pressure reference (P0).
 * @return The calculated altitude in meters based on the current pressure reading.
 */
float BarometricSensor::readAltitude() {
    float currentPressure = getPressurePa();
    if (currentPressure <= 0 || isnan(currentPressure)) {
        return -9999.0f; // Error value
    }
    return altitudeFromPressure(currentPressure);
}

/**
 * @brief Recalibrates the ground pressure reference (P0) using a low-pass filter.
 * @details This function reads the current pressure and updates
 *          the ground pressure reference (P0) using a simple exponential moving average
 *         filter to smooth out short-term fluctuations.
 */
void BarometricSensor::recalibrateGroundPressure(float currentPressure_pa) {
    if (currentPressure_pa <= 0 || isnan(currentPressure_pa)) {
        return; 
    }
    
    if (groundPressureP0_Pa == 101325.0f) { 
        // If we still have the boot default value, set it directly
        groundPressureP0_Pa = currentPressure_pa;
    } else {
        // Low-pass filter to slowly update ground pressure reference to account for weather drift
        groundPressureP0_Pa = (ALPHA_MOVING_AVERAGE * currentPressure_pa) + 
                              (1.0f - ALPHA_MOVING_AVERAGE) * groundPressureP0_Pa;
    }
}

/**
 * @brief Sets the ground pressure reference (P0) for the module.
 * @details This function allows updating the ground pressure reference used
 *         in altitude calculations.
 * @param p0 The new ground pressure reference in Pascals.
 */
void BarometricSensor::setGroundPressureP0(float p0) {
    if (p0 > 0) {
        groundPressureP0_Pa = p0;
    }
}

/**
 * @brief Sets the ground temperature reference (T0) for the module.
 * @details This function allows updating the ground temperature reference used
 *         in altitude calculations.
 * @param t0 The new ground temperature reference in Kelvin.
 */
void BarometricSensor::setGroundTemperatureT0(float t0) {
    if (t0 > 0) {
        groundTemperatureT0_K = t0;
    }
}

/**
 * @brief Gets the current ground pressure reference (P0) for the module.
 * @return The ground pressure reference in Pascals.
 */
float BarometricSensor::getGroundPressureP0() const {
    return groundPressureP0_Pa;
}

/**
 * @brief Gets the current ground temperature reference (T0) for the module.
 * @return The ground temperature reference in Kelvin.
 */
float BarometricSensor::getGroundTemperatureT0() const {
    return groundTemperatureT0_K;
}
