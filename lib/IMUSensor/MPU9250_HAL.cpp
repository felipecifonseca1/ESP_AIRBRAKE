#include "MPU9250_HAL.h"
#include <EEPROM.h>

/**
 * @brief Constructor.
 * @param i2cAddr I2C address of the MPU9250 (default 0x68).
 */
MPU9250_HAL::MPU9250_HAL(uint8_t i2cAddr) : _i2cAddr(i2cAddr) {
    // Configuration will be set in init()
}

/**
 * @brief Initialize the hardware and load saved calibration if available.
 * @return true if communication and setup were successful.
 */
bool MPU9250_HAL::init(bool verbose, bool autoCalibrate) {
    if (verbose) DEBUG_PRINTLN_F("Initializing MPU from HAL...");
    // Record the flight configuration
    _mpuConfig.accel_fs_sel = ACCEL_FS_SEL::A16G;
    _mpuConfig.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    _mpuConfig.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    _mpuConfig.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    _mpuConfig.gyro_fchoice = 0x03;
    _mpuConfig.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    _mpuConfig.accel_fchoice = 0x01;
    _mpuConfig.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    bool success = _mpu.setup(_i2cAddr, _mpuConfig);
    if (success) {
        if (autoCalibrate) {
            if (ERASE_CALIB_ON_STARTUP) {
                eraseCalibration();
                DEBUG_PRINTLN_F("BOOT: Erasing IMU calibration from EEPROM...");
            }

            if (ERASE_CALIB_ON_STARTUP || !hasCalibrationData()) {
                runFullCalibration(PRINT_IMU_PARAMS, PERFORM_FINE_TUNING, PHYSICAL_Z_AXIS_DOWN);
            } else {
                if (PERFORM_FINE_TUNING) {
                    DEBUG_PRINTLN_F("Performing iterative Fine-Tuning update...");
                    adjustCalibrationIteratively(50, true, CALIBRATION_ACCEL_TOL_G, CALIBRATION_GYRO_TOL_DPS, CALIBRATION_MAX_ITERATIONS, PHYSICAL_Z_AXIS_DOWN);
                }
            }
            
            // Final re-init to seat registers properly (warm-up)
            return init(false, false); 
        }

        clearHardwareOffsets(); // Ensure hardware starts fresh
        if (hasCalibrationData()) {
            loadCalibration(false);
        }
    }
    return success;
}

/**
 * @brief Clears the hardware offset registers of the MPU9250.
 * This effectively sets the internal bias to zero.
 */
void MPU9250_HAL::clearHardwareOffsets() {
    _mpu.setAccBias(0, 0, 0);
    _mpu.setGyroBias(0, 0, 0);
}

/**
 * @brief Poll the latest data from the sensor.
 * @return true if a new sample was available and read.
 */
bool MPU9250_HAL::update() {
    return _mpu.update(); 
}

/**
 * @brief Injects external sensor data for HIL (Hardware-In-the-Loop) simulations.
 */
void MPU9250_HAL::injectData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    // We use the Ts from Config_voo.h and assume motor_on=false for the filters during injection
    // Coordinate mapping is already handled by external parameters in MPU9250::update
    _mpu.update(Ts, false, false, ax, ay, az, gx, gy, gz, mx, my, mz);
}

/**
 * @brief Trigger the library's internal accelerometer/gyroscope calibration.
 */
void MPU9250_HAL::calibrateAccel() {
    _mpu.calibrateAccelGyro(true); // Default to Z-Down for internal library calib
}

/**
 * @brief Trigger the library's internal accelerometer/gyroscope calibration.
 */
void MPU9250_HAL::calibrateGyro() {
    _mpu.calibrateAccelGyro(true);
}

/**
 * @brief Trigger the library's internal magnetometer calibration.
 */
void MPU9250_HAL::calibrateMag() {
    _mpu.calibrateMag();
}

// =========================================================================
// SPECIFIC MPU9250 CALIBRATION METHODS
// =========================================================================

/**
 * @brief Helper to print a labeled 3-element vector.
 * @param label Label for the vector.
 * @param values Array of 3 float values.
 * @param scale Scaling factor to apply to values before printing.
 * @param decimals Number of decimal places to print.
 * @param unit_label Optional unit label to append.
 */
void MPU9250_HAL::printVector(const char* label, const float values[], float scale, int decimals, const char* unit_label) {
    DEBUG_PRINT_F(label); 
    DEBUG_PRINT_F(": ");
    for (int i = 0; i < 3; i++) {
        if (isnan(values[i])) {
            DEBUG_PRINT_F("nan");
        } else {
            DEBUG_PRINT(values[i] * scale, decimals);
        }
        if (i < 2) {
            DEBUG_PRINT_F(", ");
        }
    }
    if (unit_label != nullptr && strlen(unit_label) > 0) {
        DEBUG_PRINT_F(" [");
        DEBUG_PRINT(unit_label); 
        DEBUG_PRINT_F("]");
    }
    DEBUG_PRINTLN_F(""); 
}

/**
 * @brief Save current hardware biases and magnetometer params to EEPROM.
 * @param printDebug If true, prints the values being saved.
 */
void MPU9250_HAL::saveCalibration(bool printDebug) {
    if (printDebug) DEBUG_PRINTLN_F("Saving IMU calibration to EEPROM...");
    
    EEPROM.write(0, CALIBRATION_MAGIC);
    
    // Accel Biases (float -> 4 bytes each)
    float abx = _mpu.getAccBiasX();
    float aby = _mpu.getAccBiasY();
    float abz = _mpu.getAccBiasZ();
    EEPROM.put(1, abx);
    EEPROM.put(5, aby);
    EEPROM.put(9, abz);
    
    // Gyro Biases
    float gbx = _mpu.getGyroBiasX();
    float gby = _mpu.getGyroBiasY();
    float gbz = _mpu.getGyroBiasZ();
    EEPROM.put(13, gbx);
    EEPROM.put(17, gby);
    EEPROM.put(21, gbz);
    
    // Mag Calib (Bias and Scale)
    EEPROM.put(25, _mpu.getMagBiasX());
    EEPROM.put(29, _mpu.getMagBiasY());
    EEPROM.put(33, _mpu.getMagBiasZ());
    EEPROM.put(37, _mpu.getMagScaleX());
    EEPROM.put(41, _mpu.getMagScaleY());
    EEPROM.put(45, _mpu.getMagScaleZ());
    
    EEPROM.commit();
    if (printDebug) DEBUG_PRINTLN_F("IMU calibration saved successfully.");
}

/**
 * @brief Invalidate calibration in EEPROM.
 */
void MPU9250_HAL::eraseCalibration() {
    DEBUG_PRINTLN_F("BOOT: Erasing IMU calibration from EEPROM...");
    EEPROM.write(0, 0xFF); // Invalidate magic number
    EEPROM.commit();
}

/**
 * @brief Load calibration data from EEPROM and apply to hardware registers.
 * @param printDebug If true, prints the loaded values.
 */
void MPU9250_HAL::loadCalibration(bool printDebug) {
    if (!hasCalibrationData()) {
        if (printDebug) DEBUG_PRINTLN_F("No valid calibration data in EEPROM.");
        return;
    }

    if (printDebug) DEBUG_PRINTLN_F("Loading IMU calibration from EEPROM...");
    
    clearHardwareOffsets(); // Start from clean slate before applying saved total

    float abx, aby, abz, gbx, gby, gbz;
    float mbx, mby, mbz, msx, msy, msz;

    EEPROM.get(1, abx); EEPROM.get(5, aby); EEPROM.get(9, abz);
    EEPROM.get(13, gbx); EEPROM.get(17, gby); EEPROM.get(21, gbz);
    
    EEPROM.get(25, mbx); EEPROM.get(29, mby); EEPROM.get(33, mbz);
    EEPROM.get(37, msx); EEPROM.get(41, msy); EEPROM.get(45, msz);

    _mpu.setAccBias(abx, aby, abz);
    _mpu.setGyroBias(gbx, gby, gbz);
    _mpu.setMagBias(mbx, mby, mbz);
    _mpu.setMagScale(msx, msy, msz);

    if (printDebug) {
        float ab[3] = {abx, aby, abz};
        float gb[3] = {gbx, gby, gbz};
        float mb[3] = {mbx, mby, mbz};
        float ms[3] = {msx, msy, msz};
        printVector("  Loaded Accel Bias", ab, 1.0f, 2, "LSB");
        printVector("  Loaded Gyro Bias", gb, 1.0f, 2, "LSB");
        printVector("  Loaded Mag Bias", mb, 1.0f, 2, "LSB");
        printVector("  Loaded Mag Scale", ms, 1.0f, 2, "");
        DEBUG_PRINTLN_F("IMU calibration loaded and applied.");
    }
}

/**
 * @brief Checks if valid calibration data exists in EEPROM.
 * @return true if calibration magic number is found and valid.
 */
bool MPU9250_HAL::hasCalibrationData() {
    uint8_t magic = EEPROM.read(0);
    return (magic == CALIBRATION_MAGIC);
}

/**
 * @brief Samples the IMU multiple times to find the average error.
 * @param samples Number of samples to average.
 * @param result_accel_g Output array for accel error [g].
 * @param result_gyro_dps Output array for gyro error [dps].
 * @param physicalZAxisDown Rocket mounting orientation (true if Z-axis points down, false if up).
 */
void MPU9250_HAL::collectBiasErrors(int samples, float result_accel_g[3], float result_gyro_dps[3], bool physicalZAxisDown) {
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    uint32_t prev_time_micros = micros();
    const uint32_t sample_interval_micros = 8000; 

    for (int i = 0; i < samples; i++) {
        while (micros() - prev_time_micros < sample_interval_micros) {}
        prev_time_micros = micros(); 

        if (update()) { 
            accel_sum[0] += _mpu.getAccX(); 
            accel_sum[1] += _mpu.getAccY(); 
            accel_sum[2] += _mpu.getAccZ(); 
            
            gyro_sum[0] += _mpu.getGyroX(); 
            gyro_sum[1] += _mpu.getGyroY(); 
            gyro_sum[2] += _mpu.getGyroZ(); 
            
            if (physicalZAxisDown) {
                accel_sum[2] += 1.0f; // Sensor sees -1g, error is (Reading - (-1g)) = Reading + 1
            } else {
                accel_sum[2] -= 1.0f; // Sensor sees +1g, error is (Reading - 1g)
            }
        } else {
            i--; 
            delay(1); 
        }
    }
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        result_accel_g[axis] = accel_sum[axis] / samples;
        result_gyro_dps[axis] = gyro_sum[axis] / samples;
    }
}

/**
 * @brief The "Step 3" iterative fine-tuning algorithm.
 * @details Matching legacy proven robustness: This method resets the hardware 
 *          completely on every loop to prevent hardware bias contamination.
 *          It nudges the biases until the stationary reading is perfectly zeroed.
 * @param samples_per_iteration Number of samples to collect for each error calculation.
 * @param print_debug If true, prints detailed debug information for each iteration.
 * @param accel_tol_g Accelerometer tolerance in g.
 * @param gyro_tol_dps Gyroscope tolerance in degrees per second.
 * @param max_iter Maximum number of iterations.
 * @param physicalZAxisDown Rocket mounting orientation (true if Z-axis points down, false if up).
 * @return true if convergence was achieved.
 */
bool MPU9250_HAL::adjustCalibrationIteratively(int samples_per_iteration, bool print_debug, float accel_tol_g, float gyro_tol_dps, int max_iter, bool physicalZAxisDown) {
    bool calibrated = false;
    
    // 1. Configure for high precision (A2G is 8x more sensitive than A16G)
    MPU9250Setting calibFineTuneConfig = _mpuConfig; // Start with current config
    calibFineTuneConfig.accel_fs_sel = ACCEL_FS_SEL::A2G; 
    calibFineTuneConfig.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
    calibFineTuneConfig.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;

    DEBUG_PRINT_F("Fine Tuning: ");

    for (int iter = 0; iter < max_iter; ++iter) {
        float error_accel_g[3], error_gyro_dps[3];

        // Match Legacy: Full device re-init to clear hardware buffers and registers
        if (!_mpu.setup(_i2cAddr, calibFineTuneConfig)) {
             DEBUG_PRINTLN_F("ERROR: HW Reset failed during fine-tuning.");
             return false;
        }
        _mpu.verbose(false);
        loadCalibration(false); // Restore current total into hardware

        collectBiasErrors(samples_per_iteration, error_accel_g, error_gyro_dps, physicalZAxisDown);

        bool accel_ok = true, gyro_ok = true;
        for (int i = 0; i < 3; i++) {
            if (abs(error_accel_g[i]) > accel_tol_g) accel_ok = false;
            if (abs(error_gyro_dps[i]) > gyro_tol_dps) gyro_ok = false;
        }

        if (print_debug) {
            DEBUG_PRINTF("\nFine tuning iteration: %d / %d\n", (iter + 1), max_iter);
            printVector("  Error Accel (g)", error_accel_g, 1.0f, 5, "g");
            printVector("  Error Gyro (dps)", error_gyro_dps, 1.0f, 5, "dps");
            
            float current_acc[3] = {_mpu.getAccBiasX(), _mpu.getAccBiasY(), _mpu.getAccBiasZ()};
            float current_gyro[3] = {_mpu.getGyroBiasX(), _mpu.getGyroBiasY(), _mpu.getGyroBiasZ()};
            printVector("  Current Acc Bias (LSB)", current_acc, 1.0f, 2, "");
            printVector("  Current Gyro Bias (LSB)", current_gyro, 1.0f, 2, "");
        } else {
            DEBUG_PRINT_F("."); 
        }

        if (accel_ok && gyro_ok) {
            calibrated = true;
            break; 
        }

        // Apply nudges to the absolute state. 
        // Logic: Output = Raw - Bias. If Output is too high (Error > 0), we INCREASE Bias.
        float Kp_accel = 0.15f; 
        float Kp_gyro  = 0.5f;

        float current_acc[3] = {_mpu.getAccBiasX(), _mpu.getAccBiasY(), _mpu.getAccBiasZ()};
        float current_gyro[3] = {_mpu.getGyroBiasX(), _mpu.getGyroBiasY(), _mpu.getGyroBiasZ()};

        _mpu.setAccBias(
            current_acc[0] + (error_accel_g[0] * 16384.0f) * Kp_accel,
            current_acc[1] + (error_accel_g[1] * 16384.0f) * Kp_accel,
            current_acc[2] + (error_accel_g[2] * 16384.0f) * Kp_accel
        );
        _mpu.setGyroBias(
            current_gyro[0] + (error_gyro_dps[0] * 131.0f) * Kp_gyro,
            current_gyro[1] + (error_gyro_dps[1] * 131.0f) * Kp_gyro,
            current_gyro[2] + (error_gyro_dps[2] * 131.0f) * Kp_gyro
        );

        saveCalibration(false); 
    }

    DEBUG_PRINTLN_F(" Done.");
    init(false); // Restore final flight config silently
    return calibrated;
}

/**
 * @brief High-level runner for the interactive calibration sequence.
 * @param printDebug Enable serial prompts.
 * @param performFineTuning If true, runs the iterative Step 3.
 * @param physicalZAxisDown Orientation parameter for gravity removal.
 */
void MPU9250_HAL::runFullCalibration(bool printDebug, bool performFineTuning, bool physicalZAxisDown) {
    // ALWAYS print progress for full calibration to avoid user confusion
    DEBUG_PRINTLN_F("\n--- [STEP 1/3] BASE IMU CALIBRATION ---");
    DEBUG_PRINTLN_F("Keep rocket STILL and LEVEL on the pad...");
    
    clearHardwareOffsets(); // Start from absolute zero
    delay(2000);

    _mpu.verbose(printDebug);
    _mpu.calibrateAccelGyro(physicalZAxisDown); // Combined library calib
    _mpu.verbose(false);

    float base_acc[3] = {_mpu.getAccBiasX(), _mpu.getAccBiasY(), _mpu.getAccBiasZ()};
    float base_gyro[3] = {_mpu.getGyroBiasX(), _mpu.getGyroBiasY(), _mpu.getGyroBiasZ()};
    printVector("  Step 1 Result - Accel Bias", base_acc, 1.0f, 2, "LSB");
    printVector("  Step 1 Result - Gyro Bias", base_gyro, 1.0f, 2, "LSB");

    DEBUG_PRINTLN_F("\n--- [STEP 2/3] MAGNETOMETER CALIBRATION ---");
    DEBUG_PRINTLN_F("Move rocket in FIGURE-EIGHT pattern (Starting in 2s)...");
    delay(2000);
    calibrateMag();
    
    saveCalibration(true); 
    DEBUG_PRINTLN_F("Base calibration (Step 1 & 2) saved to EEPROM.");

    if (performFineTuning) {
        DEBUG_PRINTLN_F("\n--- [STEP 3/3] ITERATIVE FINE TUNING ---");
        DEBUG_PRINTLN_F("Keep STILL again for high-precision bias tracking...");
        delay(2000);
        float accel_tol_g = CALIBRATION_ACCEL_TOL_G;
        float gyro_tol_dps = CALIBRATION_GYRO_TOL_DPS;
        adjustCalibrationIteratively(50, true, accel_tol_g, gyro_tol_dps, CALIBRATION_MAX_ITERATIONS, physicalZAxisDown);
    }
}
