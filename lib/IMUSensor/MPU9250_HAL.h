#ifndef MPU9250_HAL_H
#define MPU9250_HAL_H

#include "IMUSensor.h"
#include "MPU9250/MPU9250.h" 

/**
 * @class MPU9250_HAL
 * @brief Concrete implementation of IMUSensor for the custom I2C MPU9250 library.
 * @details This wrapper configures the IMU using the specific MPU9250Setting struct 
 *          that matches the original flight configuration.
 */
class MPU9250_HAL : public IMUSensor {
private:
    MPU9250 _mpu;
    uint8_t _i2cAddr;
    
public:
    // Initialize with the I2C Address (defaults to 0x68)
    MPU9250_HAL(uint8_t i2cAddr = 0x68) : _i2cAddr(i2cAddr) {}

    bool init() override {
        // Define the exact flight configuration that was in Funcoes_suporte_IMU.cpp
        MPU9250Setting mpuConfig;
        mpuConfig.accel_fs_sel = ACCEL_FS_SEL::A16G;
        mpuConfig.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
        mpuConfig.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
        mpuConfig.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
        mpuConfig.gyro_fchoice = 0x03;
        mpuConfig.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
        mpuConfig.accel_fchoice = 0x01;
        mpuConfig.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

        return _mpu.setup(_i2cAddr, mpuConfig);
    }

    bool update() override {
        // The custom library's update() takes dt and runs madgwick internally too,
        // but it still fulfills our rule of reading raw data.
        return _mpu.update(); 
    }

    void calibrateAccel() override { _mpu.calibrateAccelGyro(); }
    void calibrateGyro() override { _mpu.calibrateAccelGyro(); }
    void calibrateMag() override { _mpu.calibrateMag(); }

    // --- Raw Data Getters ---
    // The custom library getters like getAccX() return values mapped up to 16G based on resolution
    float getAccX() const override { return _mpu.getAccX(); } 
    float getAccY() const override { return _mpu.getAccY(); }
    float getAccZ() const override { return _mpu.getAccZ(); }

    // The custom library returns degrees/sec natively.
    float getGyroX_rads() const override { return _mpu.getGyroX() * DEG_TO_RAD; }
    float getGyroY_rads() const override { return _mpu.getGyroY() * DEG_TO_RAD; }
    float getGyroZ_rads() const override { return _mpu.getGyroZ() * DEG_TO_RAD; }

    // The custom library returns milliGauss
    float getMagX() const override { return _mpu.getMagX(); }
    float getMagY() const override { return _mpu.getMagY(); }
    float getMagZ() const override { return _mpu.getMagZ(); }

    float getTemperature() const override { return _mpu.getTemperature(); }

    // --- Calibration Getters ---
    float getAccelBiasX() const override { return _mpu.getAccBiasX(); }
    float getAccelBiasY() const override { return _mpu.getAccBiasY(); }
    float getAccelBiasZ() const override { return _mpu.getAccBiasZ(); }

    float getGyroBiasX() const override { return _mpu.getGyroBiasX() * DEG_TO_RAD; }
    float getGyroBiasY() const override { return _mpu.getGyroBiasY() * DEG_TO_RAD; }
    float getGyroBiasZ() const override { return _mpu.getGyroBiasZ() * DEG_TO_RAD; }

    // --- Metadata ---
    String getDeviceName() const override { return "Custom_MPU9250_I2C"; }
    
    // Pass-through to the raw object so we can still access the built-in Madgwick filter 
    // during this intermediate phase before we rip OrientationEngine out.
    MPU9250& getRawDevice() { return _mpu; }
};

#endif // MPU9250_HAL_H
