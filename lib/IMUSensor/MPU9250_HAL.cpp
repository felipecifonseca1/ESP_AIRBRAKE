#include "MPU9250_HAL.h"
#include <EEPROM.h>
#include "Config_voo.h"

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

    forceBypass();

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
    if (!success) {
        DEBUG_PRINTLN_F("!!!! ERROR !!!!: MPU9250 Magnetometer (AK8963) setup FAILED! Mag data will be 0.00.");
    }
    
    if (success) {
        if (autoCalibrate) {
            if (ERASE_CALIB_ON_STARTUP) {
                eraseCalibration(CalibEraseType::ALL);
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

            if (FORCED_MAG_CALIBRATION) {
               calibrateMagVisual();
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
 * @param dt Integration time step [s]
 * @return true if a new sample was available and read.
 */
bool MPU9250_HAL::update(float dt) {
    _magDataFresh = true; // Always allow filtering in the 50Hz loop for smoother interpolation
    return _mpu.update(dt); 
}

/**
 * @brief Injects external sensor data for HIL (Hardware-In-the-Loop) simulations.
 */
void MPU9250_HAL::injectData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    // We use the Ts from Config_voo.h and assume motor_on=false for the filters during injection
    // Update internal MPU object so that raw getters return HIL data
    // We pass Gs directly because MPU9250 library getters return the stored 'a' values.
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
/**
 * @brief Trigger the library's internal magnetometer calibration.
 */
void MPU9250_HAL::calibrateMag() {
    _mpu.calibrateMag();
}

/**
 * @brief Performs a 30-second "Figure-8" visual calibration for the Magnetometer.
 * Provides real-time feedback and saves result to EEPROM.
 */
void MPU9250_HAL::calibrateMagVisual() {
    DEBUG_PRINTLN_F("\n--- MAGNETOMETER VISUAL CALIBRATION ---");
    
    // [FORCE] Ensure I2C Bypass is active
    forceBypass();


    // [CHECK] Verify Magnetometer is actually there
    Wire.beginTransmission(0x0C); // AK8963 Address
    Wire.write(0x00); // WHO_AM_I
    uint8_t err = Wire.endTransmission(false);
    
    uint8_t magId = 0xFF;
    if (err == 0) {
        Wire.requestFrom(0x0C, (uint8_t)1);
        magId = Wire.available() ? Wire.read() : 0xFF;
    } else {
        DEBUG_PRINTF("I2C ERROR AK8963 Bypass Check: %d\n", err);
    }
    
    DEBUG_PRINT_F("AK8963 WHO_AM_I: 0x"); DEBUG_PRINTLN(magId, HEX);
    
    if (magId != 0x48) {
        DEBUG_PRINTLN_F("!!!! ERROR: AK8963 not found. Bypass FAILED! Mag will be zero.");
    }

    // [MODE] Force 100Hz 16-bit Continuous Mode (CNTL1 = 0x16)
    Wire.beginTransmission(0x0C);
    Wire.write(0x0A); Wire.write(0x16);
    Wire.endTransmission();
    delay(100);

    // [WAIT] 2-second prep window so the user has time to grab the rocket
    DEBUG_PRINTLN_F("GET READY... HOLD STILL (2s)");
    delay(2000);

    // [RESET] Clear any old biases in the library object so we get RAW data
    _mpu.setMagBias(0, 0, 0);
    _mpu.setMagScale(1, 1, 1);

    float m_min[3], m_max[3];
    bool firstSample = true;
    
    DEBUG_PRINTLN_F("CALIBRATING... SPIN THE ROCKET NOW (30s)!");
    
    uint32_t startTime = millis();
    uint32_t lastPrint = 0;
    
    while (millis() - startTime < 30000) {
        _mpu.update_mag(); 
        
        float mx = _mpu.getMagX();
        float my = _mpu.getMagY();
        float mz = _mpu.getMagZ();
        
        // Skip junk zeros at startup
        if (abs(mx) < 0.001f && abs(my) < 0.001f && abs(mz) < 0.001f) {
            delay(10);
            continue;
        }

        if (firstSample) {
            m_min[0] = m_max[0] = mx;
            m_min[1] = m_max[1] = my;
            m_min[2] = m_max[2] = mz;
            firstSample = false;
        } else {
            // Update Min/Max bounds
            if (mx < m_min[0]) m_min[0] = mx;
            if (mx > m_max[0]) m_max[0] = mx;
            if (my < m_min[1]) m_min[1] = my;
            if (my > m_max[1]) m_max[1] = my;
            if (mz < m_min[2]) m_min[2] = mz;
            if (mz > m_max[2]) m_max[2] = mz;
        }

        if (millis() - lastPrint > 200) {
            int progress = (millis() - startTime) / 300;
            DEBUG_PRINT_F("P:"); DEBUG_PRINT(progress); DEBUG_PRINT_F("% | ");
            DEBUG_PRINT_F("X("); DEBUG_PRINT((int)m_min[0]); DEBUG_PRINT_F(","); DEBUG_PRINT((int)m_max[0]);
            DEBUG_PRINT_F(") Y("); DEBUG_PRINT((int)m_min[1]); DEBUG_PRINT_F(","); DEBUG_PRINT((int)m_max[1]);
            DEBUG_PRINT_F(") Z("); DEBUG_PRINT((int)m_min[2]); DEBUG_PRINT_F(","); DEBUG_PRINT((int)m_max[2]);
            DEBUG_PRINTLN_F(")");
            lastPrint = millis();
        }
        delay(10);
        yield();
    }

    // Safety check: if no data was collected, don't overwrite with 0s
    if (m_max[0] < m_min[0] + 1.0f) {
        DEBUG_PRINTLN_F("!!!! ERROR: No Mag data collected. Check Wiring/Power! Calibration aborted.");
        return;
    }

    // Calculate Bias (Hard Iron)
    float mbx = (m_max[0] + m_min[0]) / 2.0f;
    float mby = (m_max[1] + m_min[1]) / 2.0f;
    float mbz = (m_max[2] + m_min[2]) / 2.0f;

    // Calculate Scale (Soft Iron)
    float deltaX = m_max[0] - m_min[0];
    float deltaY = m_max[1] - m_min[1];
    float deltaZ = m_max[2] - m_min[2];
    float avgDelta = (deltaX + deltaY + deltaZ) / 3.0f;

    float msx = (deltaX > 0.1f) ? (avgDelta / deltaX) : 1.0f;
    float msy = (deltaY > 0.1f) ? (avgDelta / deltaY) : 1.0f;
    float msz = (deltaZ > 0.1f) ? (avgDelta / deltaZ) : 1.0f;

    DEBUG_PRINTLN_F("\nCALIBRATION COMPLETE!");
    _mpu.setMagBias(mbx, mby, mbz);
    _mpu.setMagScale(msx, msy, msz);

    // Save to EEPROM
    float abx, aby, abz, gbx, gby, gbz;
    EEPROM.get(1, abx); EEPROM.get(5, aby); EEPROM.get(9, abz);
    EEPROM.get(13, gbx); EEPROM.get(17, gby); EEPROM.get(21, gbz);

    EEPROM.write(0, CALIBRATION_MAGIC);
    EEPROM.put(1, abx); EEPROM.put(5, aby); EEPROM.put(9, abz);
    EEPROM.put(13, gbx); EEPROM.put(17, gby); EEPROM.put(21, gbz);
    EEPROM.put(25, mbx); EEPROM.put(29, mby); EEPROM.put(33, mbz);
    EEPROM.put(37, msx); EEPROM.put(41, msy); EEPROM.put(45, msz);
    EEPROM.commit();

    DEBUG_PRINT_F("Saved Mag Bias: ");  DEBUG_PRINT(mbx); DEBUG_PRINT_F(", "); DEBUG_PRINT(mby); DEBUG_PRINT_F(", "); DEBUG_PRINTLN(mbz);
    DEBUG_PRINT_F("Saved Mag Scale: "); DEBUG_PRINT(msx); DEBUG_PRINT_F(", "); DEBUG_PRINT(msy); DEBUG_PRINT_F(", "); DEBUG_PRINTLN(msz);
}

/**
 * @brief Force MPU9250 into I2C Bypass mode.
 * Disables I2C Master and enables Bypass bit to allow direct access to AK8963.
 */
void MPU9250_HAL::forceBypass() {
    // 0x37 = INT_PIN_CFG, 0x02 = BYPASS_EN
    Wire.beginTransmission(_i2cAddr);
    Wire.write(0x37); Wire.write(0x02);
    Wire.endTransmission();
    
    // 0x6A = USER_CTRL, 0x00 = Disable I2C Master
    Wire.beginTransmission(_i2cAddr);
    Wire.write(0x6A); Wire.write(0x00);
    Wire.endTransmission();
    delay(10);
}

/**
 * @brief Checks if the latest update() call included a fresh magnetic sample.
 */
bool MPU9250_HAL::isMagDataNew() {
    return _magDataFresh;
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
 * @brief Wipes selected portion of calibration data from EEPROM.
 * @param type Portions to wipe: ALL, ACCEL_GYRO, or MAG.
 */
void MPU9250_HAL::eraseCalibration(CalibEraseType type) {
    if (type == CalibEraseType::ALL) {
        DEBUG_PRINTLN_F("BOOT: Erasing ALL IMU calibration from EEPROM...");
        EEPROM.write(0, 0xFF); // Invalidate magic number
        EEPROM.commit();
        return;
    }

    // For partial erases, we MUST have existing valid data to preserve the other part
    if (!hasCalibrationData()) {
        DEBUG_PRINTLN_F("BOOT: Partial erase requested but no valid data exists. Skipping.");
        return;
    }

    // 1. Load the valid totals from EEPROM into the _mpu object
    loadCalibration(false); 

    if (type == CalibEraseType::ACCEL_GYRO) {
        DEBUG_PRINTLN_F("BOOT: Erasing Accel/Gyro calibration ONLY...");
        _mpu.setAccBias(0, 0, 0);
        _mpu.setGyroBias(0, 0, 0);
    } 
    else if (type == CalibEraseType::MAG) {
        DEBUG_PRINTLN_F("BOOT: Erasing Magnetometer calibration ONLY...");
        _mpu.setMagBias(0, 0, 0);
        _mpu.setMagScale(1.0f, 1.0f, 1.0f);
    }

    // 2. Commit the new hybrid state (with selected parts zeroed) back to EEPROM
    // Note: This keeps the CALIBRATION_MAGIC intact so the remaining part is still valid.
    saveCalibration(true);
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
            accel_sum[0] += getAccX(); 
            accel_sum[1] += getAccY(); 
            accel_sum[2] += getAccZ(); 
            
            gyro_sum[0] += (getGyroX_rads() * RAD_TO_DEG); 
            gyro_sum[1] += (getGyroY_rads() * RAD_TO_DEG); 
            gyro_sum[2] += (getGyroZ_rads() * RAD_TO_DEG); 
            
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
        forceBypass();
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
            current_acc[2] - (error_accel_g[2] * 16384.0f) * Kp_accel
        );
        _mpu.setGyroBias(
            current_gyro[0] + (error_gyro_dps[0] * 131.0f) * Kp_gyro,
            current_gyro[1] + (error_gyro_dps[1] * 131.0f) * Kp_gyro,
            current_gyro[2] - (error_gyro_dps[2] * 131.0f) * Kp_gyro
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
        delay(3000);
        float accel_tol_g = CALIBRATION_ACCEL_TOL_G;
        float gyro_tol_dps = CALIBRATION_GYRO_TOL_DPS;
        adjustCalibrationIteratively(50, true, accel_tol_g, gyro_tol_dps, CALIBRATION_MAX_ITERATIONS, physicalZAxisDown);
    }
}
