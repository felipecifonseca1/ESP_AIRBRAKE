#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>

/**
 * @class IMUSensor
 * @brief Abstract base class defining the hardware interface for an Inertial Measurement Unit.
 * @details This contract promises raw, unfiltered data from the Accelerometer, Gyroscope, and Magnetometer.
 *          It does NOT contain any orientation mathematics (e.g. Madgwick, tilt, acceleration).
 */
class IMUSensor {
public:
    virtual ~IMUSensor() = default;

    // --- Hardware Control ---
    /**
     * @brief Initializes the IMU hardware and configures settings (scales, bandwidth).
     * @return True if initialization was successful, false otherwise.
     */
    virtual bool init() = 0;

    /**
     * @brief Polls the hardware for the latest data. Must be called rapidly in the main loop.
     * @return True if new data was successfully read.
     */
    virtual bool update() = 0;
    
    // --- Calibration Triggers ---
    virtual void calibrateAccel() = 0;
    virtual void calibrateGyro() = 0;
    virtual void calibrateMag() = 0;

    // --- Raw Accelerometer Data (g) ---
    virtual float getAccX() const = 0;
    virtual float getAccY() const = 0;
    virtual float getAccZ() const = 0;

    // --- Raw Gyroscope Data (rad/s) ---
    // Note: Most IMUs return deg/s natively. The HAL wrapper must convert this to rad/s for the Madgwick filter.
    virtual float getGyroX_rads() const = 0;
    virtual float getGyroY_rads() const = 0;
    virtual float getGyroZ_rads() const = 0;

    // --- Raw Magnetometer Data (uT) ---
    virtual float getMagX() const = 0;
    virtual float getMagY() const = 0;
    virtual float getMagZ() const = 0;
    
    // --- Temperature (°C) ---
    virtual float getTemperature() const = 0;
    
    // --- Calibration Getters/Setters (Optional depending on hardware) ---
    virtual float getAccelBiasX() const = 0;
    virtual float getAccelBiasY() const = 0;
    virtual float getAccelBiasZ() const = 0;
    
    virtual float getGyroBiasX() const = 0;
    virtual float getGyroBiasY() const = 0;
    virtual float getGyroBiasZ() const = 0;

    // --- Device Information ---
    virtual String getDeviceName() const = 0;
};

#endif // IMU_SENSOR_H
