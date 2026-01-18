#include "Funcoes_BMP.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>  
#include <Adafruit_BMP280.h>  
#include "Config_voo.h"       
#include <math.h>             
#include <Arduino.h>

// Variables and BMP280 Object
static Adafruit_BMP280 bmp_sensor_obj;              // I2C0
// static Adafruit_BMP280 bmp_sensor_obj(&WireN);   // I2CN
// WireN.begin();                                   // Use this if using a different I2C bus
const uint8_t BMP280_I2C_ADAFRUIT_ADDRESS = 0x76;
static float bmp_module_groundPressureP0 = 93690.00f; // Valor tem que ser alterado
static float bmp_module_groundTemperatureT0 = 296.9f;  // Valor tem que ser alterado

// Constants for altitude calculation
static const float L_ISA_BMP = 0.0065f;
static const float G_ACCEL_BMP = 9.80665f;
static const float M_AIR_BMP = 0.0289644f;
static const float R_GAS_BMP = 8.31447f;
static const float ISA_EXPONENT_BMP = (R_GAS_BMP * L_ISA_BMP) / (G_ACCEL_BMP * M_AIR_BMP);

// Parameter for low-pass filtering of ground pressure
const float ALPHA_MOVING_AVERAGE = 0.01f; 

/**
 * @brief Configures and initializes the BMP280 sensor.
 * @details Sets up the BMP280 sensor with specified sampling settings,
 *          and collects reference readings for ground pressure (P0) and temperature (T0).
 **/
bool setupBMP() {
    DEBUG_PRINTLN_F("SETUP_BMP: CONFIGURING BMP280..."); 
    if (!bmp_sensor_obj.begin(BMP280_I2C_ADAFRUIT_ADDRESS)) {
        DEBUG_PRINTLN("ERROR: Could not find BMP280!");
        return false;
    }
    DEBUG_PRINTLN("SETUP_BMP: BMP280 initialized successfully.");

    bmp_sensor_obj.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                               Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                               Adafruit_BMP280::SAMPLING_X4,     /* Pressure oversampling */
                               Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                               Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */

    DEBUG_PRINTLN("SETUP_BMP: Collecting reference readings from BMP280 (P0 and T0 at ground level)...");
    long accumulatedPressure = 0;
    float accumulatedTemperature = 0.0f;
    const int numReadingsForReference = 100;

    for (int i = 0; i < numReadingsForReference; i++) {
        uint32_t p = bmp_sensor_obj.readPressure();
        if (p == 0) { /* ... error treatment ... */ }
        accumulatedPressure += p;
        accumulatedTemperature += bmp_sensor_obj.readTemperature();
        delay(20);
    }

    if (numReadingsForReference > 0) {
        bmp_module_groundPressureP0 = (float)accumulatedPressure / numReadingsForReference;
        bmp_module_groundTemperatureT0 = (accumulatedTemperature / numReadingsForReference) + 273.15f;
    } else { /* ... error treatment ... */  }
    DEBUG_PRINT_F("SETUP_BMP: P0 (BMP):");
    DEBUG_PRINT(bmp_module_groundPressureP0);
    DEBUG_PRINT_F(" | T0 (BMP):");
    DEBUG_PRINTLN((bmp_module_groundTemperatureT0 - 273.15f));
    return true;
}

/**
 * @brief Calculates altitude from pressure using the barometric formula.
 * @details Uses the international standard atmosphere model to convert pressure readings
 *          to altitude above sea level.
 * * @param pressao_pa The current pressure reading in Pascals.
 * @return The calculated altitude in meters.
 */
float altitudeFromPressure(float pressure_pa) {
    if (pressure_pa <= 0 || bmp_module_groundPressureP0 <= 0) return -9999.0f;

    float pressureRatio = pressure_pa / bmp_module_groundPressureP0;
    if (pressureRatio <= 0) return -9999.0f;

    // Fórmula barométrica
    return (1.0f - pow(pressureRatio, ISA_EXPONENT_BMP)) * (bmp_module_groundTemperatureT0 / L_ISA_BMP); // *Try other methods for a more accurate altitude calculation*
}

/**
 * @brief Reads the current pressure from the BMP280 sensor and calculates altitude.
 * @details Calculates altitude based on the current pressure reading from the BMP280
 *          and the stored ground pressure reference (P0).
 * @return The calculated altitude in meters based on the current pressure reading.
 */
float readAltitude() {
    float currentPressure = bmp_sensor_obj.readPressure(); 
    if (currentPressure <= 0 || isnan(currentPressure)) {
        DEBUG_PRINTLN_F("Failed to read pressure from BMP280!");
        return -9999.0f; // Error value
    }
    return altitudeFromPressure(currentPressure);
}

/**
 * @brief Sets the ground pressure reference (P0) for the BMP280 module.
 * @details This function allows updating the ground pressure reference used
 *         in altitude calculations.
 * @param p0 The new ground pressure reference in Pascals.
 */
void setGroundPressureP0_BMP(float p0) {
    if (p0 > 0) {
        bmp_module_groundPressureP0 = p0;
    }
}

/**
 * @brief Sets the ground temperature reference (T0) for the BMP280 module.
 * @details This function allows updating the ground temperature reference used
 *         in altitude calculations.
 * @param t0 The new ground temperature reference in Kelvin.
 */
void setGroundTemperatureT0_BMP(float t0) {
    if (t0 > 0) {
        bmp_module_groundTemperatureT0 = t0;
    }
}

/**
 * @brief Recalibrates the ground pressure reference (P0) for the BMP280 using a low-pass filter.
 * @details This function reads the current pressure from the BMP280 and updates
 *          the ground pressure reference (P0) using a simple exponential moving average
 *         filter to smooth out short-term fluctuations.
 */
void recalibrateGroundPressure() {

    float currentPressure_pa = bmp_sensor_obj.readPressure();
    if (currentPressure_pa <= 0 || isnan(currentPressure_pa)) {
        return; 
    }
    if (bmp_module_groundPressureP0 == 101325.0f) {
        bmp_module_groundPressureP0 = currentPressure_pa;
    } else {
        // Low-pass filter to update ground pressure reference
        bmp_module_groundPressureP0 = (ALPHA_MOVING_AVERAGE * currentPressure_pa) + (1.0f - ALPHA_MOVING_AVERAGE) * bmp_module_groundPressureP0;
    }
}

/**
 * @brief Gets the current ground pressure reference (P0) for the BMP280 module.
 * @return The ground pressure reference in Pascals.
*/
float getGroundPressureP0_BMP() { return bmp_module_groundPressureP0; }

/**
 * @brief Gets the current ground temperature reference (T0) for the BMP280 module.
 * @return The ground temperature reference in Kelvin.
*/
float getGroundTemperatureT0_BMP() { return bmp_module_groundTemperatureT0; }

/**
 * @brief Gets the current pressure (P) for the BMP280 module.
 * @return The pressure in Pascals.
*/
float getPressaoBMPAtual() { return bmp_sensor_obj.readPressure(); }

/**
 * @brief Gets the current temperature (T) for the BMP280 module.
 * @return The temperature in Kelvin.
*/
float getTemperaturaBMPAtual() { return bmp_sensor_obj.readTemperature(); }

