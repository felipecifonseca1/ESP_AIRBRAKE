#ifndef MPU9250_HAL_H
#define MPU9250_HAL_H

#include <Wire.h>
#include "IMUSensor.h"
#include "MPU9250/MPU9250.h" 
#include "Config_voo.h"

/**
 * @class MPU9250_HAL
 * @brief Concrete implementation of IMUSensor for the custom I2C MPU9250 library.
 * @details This wrapper configures the IMU using the specific MPU9250Setting struct 
 *          that matches the original flight configuration. It also handles MPU9250
 *          specific EEPROM persistence and fine-tuning calibration algorithms.
 */
class MPU9250_HAL : public IMUSensor {
private:
    MPU9250 _mpu;
    uint8_t _i2cAddr;
    
    // Original configuration settings used to revert back after fine-tuning tests
    MPU9250Setting _mpuConfig;

    // Constants for fine-tuning
    static constexpr float ACCEL_CALIB_SENSITIVITY_FS = 16384.0f; // LSB/g ( Range for calib = +/-2g)
    static constexpr float GYRO_CALIB_SENSITIVITY_FS  = 131.0f;   // LSB/(deg/s) ( Range for calib = +/-250dps)

public:
    /**
     * @brief Initialize with the I2C Address.
     * @param i2cAddr I2C Address of the sensor (usually 0x68).
     */
    MPU9250_HAL(uint8_t i2cAddr = 0x68);

    // Virtual Overrides
    bool init(bool verbose = true, bool autoCalibrate = false) override;
    bool update(float dt = 0.02f) override;
    void injectData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) override;

    void calibrateAccel() override;
    void calibrateGyro() override;
    void calibrateMag() override;
    void calibrateMagVisual();
    bool isMagDataNew();

    // --- Raw Data Getters (Standardized to Logical Z-Up Frame internally) ---
    // If HIL is active, we bypass the physical mounting flip so the simulation is detached.
    // NOTE: If physicalZAxisDown is true, the sensor is upside down natively (reads -1g at rest).
    // We normalize to Logical Z-Up (+1g at rest) for the filters.
    float getAccX() const override { return _mpu.getAccX(); } 
    float getAccY() const override { return PHYSICAL_Z_AXIS_DOWN ? -_mpu.getAccY() : _mpu.getAccY(); }
    float getAccZ() const override { return PHYSICAL_Z_AXIS_DOWN ? -_mpu.getAccZ() : _mpu.getAccZ(); } 

    float getGyroX_rads() const override { return _mpu.getGyroX() * DEG_TO_RAD; }
    float getGyroY_rads() const override { return PHYSICAL_Z_AXIS_DOWN ? -(_mpu.getGyroY() * DEG_TO_RAD) : _mpu.getGyroY() * DEG_TO_RAD; }
    float getGyroZ_rads() const override { return PHYSICAL_Z_AXIS_DOWN ? -(_mpu.getGyroZ() * DEG_TO_RAD) : _mpu.getGyroZ() * DEG_TO_RAD; }

    // Magnetometer Alignment (AK8963 vs MPU-9250 frame)
    // Standard (Upright): hx=my, hy=mx, hz=-mz. We flip Y/Z into logical Rocket Frame.
    float getMagX() const override { return _mpu.getMagY(); }
    float getMagY() const override { return PHYSICAL_Z_AXIS_DOWN ? -_mpu.getMagX() : _mpu.getMagX(); }
    float getMagZ() const override { return PHYSICAL_Z_AXIS_DOWN ? _mpu.getMagZ() : -_mpu.getMagZ(); }

    float getTemperature() const override { return _mpu.getTemperature(); }

    // --- Specific MPU9250 Methods ---
    /**
     * @brief Check if calibrated data exists in EEPROM.
     * @return true if valid magic number is found.
     */
    bool hasCalibrationData();
    static constexpr uint8_t CALIBRATION_MAGIC = 0xAA; ///< Magic byte to verify calibration validity

    void saveCalibration(bool printDebug = false); ///< Save hardware registers and mag params to EEPROM
    void clearHardwareOffsets();                    ///< Clear all internal bias registers to zero
    void loadCalibration(bool printDebug = false); ///< Load calibration from EEPROM to hardware
    /**
     * @enum CalibEraseType
     * @brief Specifies which portion of the calibration to wipe.
     */
    enum class CalibEraseType {
        ALL,        ///< Wipes everything and invalidates the EEPROM magic
        ACCEL_GYRO, ///< Wipes only Accel/Gyro biases from EEPROM
        MAG         ///< Wipes only Magnetometer biases/scales from EEPROM
    };

    void eraseCalibration(CalibEraseType type = CalibEraseType::ALL); ///< Wipe calibration data from EEPROM

    /**
     * @brief Performs a full IMU calibration sequence (Accel, Gyro, Mag)
     */
    void runFullCalibration(bool printDebug = false, bool performFineTuning = false, bool physicalZAxisDown = true);

    /**
     * @brief Iteratively adjusts the IMU calibration biases.
     * @return true If calibration converged within tolerances.
     */
    bool adjustCalibrationIteratively(int samples_per_iteration, bool print_debug, float accel_tol_g, float gyro_tol_dps, int max_iter, bool physicalZAxisDown);

    // --- Calibration Getters (Returns totals in physical units) ---
    float getAccelBiasX() const override { return _mpu.getAccBiasX(); }
    float getAccelBiasY() const override { return _mpu.getAccBiasY(); }
    float getAccelBiasZ() const override { return _mpu.getAccBiasZ(); }

    float getGyroBiasX() const override { return _mpu.getGyroBiasX() * DEG_TO_RAD; }
    float getGyroBiasY() const override { return _mpu.getGyroBiasY() * DEG_TO_RAD; }
    float getGyroBiasZ() const override { return _mpu.getGyroBiasZ() * DEG_TO_RAD; }

    // --- Metadata ---
    String getDeviceName() const override { return "Custom_MPU9250_I2C"; }
    
    // Pass-through
    MPU9250& getRawDevice() { return _mpu; }

private:
    bool _magDataFresh = false;
    void forceBypass();
    void printVector(const char* label, const float values[], float scale, int decimals, const char* unit_label);
    void collectBiasErrors(int samples, float result_accel_g[3], float result_gyro_dps[3], bool physicalZAxisDown);
};

#endif // MPU9250_HAL_H
