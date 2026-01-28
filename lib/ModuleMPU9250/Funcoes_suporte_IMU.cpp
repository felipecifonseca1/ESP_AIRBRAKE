#include "Funcoes_suporte_IMU.h"
#include <Wire.h>
#include "MPU9250.h" 
#include <EEPROM.h>
#include "Config_voo.h"  
#include <Arduino.h>
#include "Sinalizacao.h"


MPU9250 mpu;  // Global MPU9250 object
MPU9250Setting mpuConfig;

const float ACCEL_CALIB_SENSITIVITY_FS = 16384.0f; // LSB/g ( Range for calib = +/-2g)
const float GYRO_CALIB_SENSITIVITY_FS  = 131.0f;   // LSB/(deg/s) ( Range for calib = +/-250dps)


/**
 * @brief Saves the IMU calibration parameters to EEPROM.
 * @details Stores the biases of the accelerometer, gyroscope and magnetometer.
 * This allows the rocket to "remember" the calibration even after turning off the battery.
 * * @param print If true, prints the saved values to Serial Monitor for debugging.
 */
void saveCalibration(bool print) {
  float accel_bias[] = {mpu.getAccBiasX(), mpu.getAccBiasY(), mpu.getAccBiasZ()};
  float gyro_bias[]  = {mpu.getGyroBiasX(), mpu.getGyroBiasY(), mpu.getGyroBiasZ()};
  float mag_bias[]   = {mpu.getMagBiasX(), mpu.getMagBiasY(), mpu.getMagBiasZ()};
  float mag_scale[]  = {mpu.getMagScaleX(), mpu.getMagScaleY(), mpu.getMagScaleZ()};

  EEPROM.put(0, accel_bias);
  EEPROM.put(sizeof(accel_bias), gyro_bias);
  EEPROM.put(sizeof(accel_bias) + sizeof(gyro_bias), mag_bias);
  EEPROM.put(sizeof(accel_bias) + sizeof(gyro_bias) + sizeof(mag_bias), mag_scale);

  if (EEPROM.commit()) {
      if (print) {
        DEBUG_PRINTLN_F("--- CALIBRATION SAVED IN EEPROM ---");
        print_vector("Acc Bias", accel_bias);
        print_vector("Gyro Bias", gyro_bias);
        print_vector("Mag Bias", mag_bias);
        print_vector("Mag Scale", mag_scale);}
  } else {
      DEBUG_PRINTLN("ERROR: Failed to save in EEPROM!");
  }
}

/**
 * @brief Erases the IMU calibration data from EEPROM.
 * @details Writes zeros to the memory to force a new calibration on next initialization.
 **/
void eraseCalibration() {
    float empty_data[3] = {NAN, NAN, NAN};  // Uses NAN to indicate no calibration

    EEPROM.put(0, empty_data);  // Deletes accel_bias
    EEPROM.put(sizeof(empty_data), empty_data);  // Deletes gyro_bias
    EEPROM.put(sizeof(empty_data) * 2, empty_data);  // Deletes mag_bias
    EEPROM.put(sizeof(empty_data) * 3, empty_data);  // Deletes mag_scale


    if (EEPROM.commit()) {
        DEBUG_PRINTLN("Calibration erased from EEPROM.");
    } else {
        DEBUG_PRINTLN("ERROR: Failed to erase EEPROM!");
    }
}

/**
 * @brief Loads the IMU calibration parameters from EEPROM to the sensor.
 * @details Reads the saved values and applies them to the 'mpu' object, avoiding
 * having to recalibrate the rocket at the launch pad.
 * * @param print If true, prints the loaded values.
 */
void loadCalibration(bool print) {
  float accel_bias[3], gyro_bias[3], mag_bias[3], mag_scale[3];

  EEPROM.get(0, accel_bias);
  EEPROM.get(sizeof(accel_bias), gyro_bias);
  EEPROM.get(sizeof(accel_bias) + sizeof(gyro_bias), mag_bias);
  EEPROM.get(sizeof(accel_bias) + sizeof(gyro_bias) + sizeof(mag_bias), mag_scale);

  // Aplies values to the MPU9250 object
  mpu.setAccBias(accel_bias[0], accel_bias[1], accel_bias[2]);
  mpu.setGyroBias(gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  mpu.setMagBias(mag_bias[0], mag_bias[1], mag_bias[2]);
  mpu.setMagScale(mag_scale[0], mag_scale[1], mag_scale[2]);

  if (print){
    DEBUG_PRINTLN("Calibration loaded from EEPROM");

    // Show calibration parameters
    DEBUG_PRINTLN("< Calibration parameters >");
    print_vector("accel bias [mili_g]", accel_bias, 1000.0 / MPU9250::CALIB_ACCEL_SENSITIVITY );
    print_vector("gyro bias [deg/s]", gyro_bias, 1.0 / MPU9250::CALIB_GYRO_SENSITIVITY);
    print_vector("mag bias [mG]", mag_bias, 1);
    print_vector("mag scale []", mag_scale, 1);
  }
}

/**
 * @brief Checks if valid calibration data is saved.
 * @return true if the signature (byte 123) is found.
 * @return false if the memory is uninitialized or corrupted.
 */
bool hasCalibrationDataIMU() {
    float test_value;
    EEPROM.get(0, test_value);
    return !isnan(test_value);  
}

/**
 * @brief Auxiliary function to print 3D vector values to Serial.
 * @param label Label to identify the vector.
 * @param values Array of 3 float values representing the vector.
 * @param scale Scaling factor to apply to each value before printing.
 * @param decimals Number of decimal places to print.
 * @param unit_label Optional unit label to append after the vector values.
 */
void print_vector(const char* label, const float values[], float scale, int decimals, const char* unit_label) {
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
 * @brief Writes a byte to a specific register of the IMU via I2C.
 * @param address I2C address of the IMU.
 * @param subAddress Register address to write to.
 * @param data Byte value to write.
 */
void write_byte_local(uint8_t address, uint8_t subAddress, uint8_t data) {
     Wire.beginTransmission(address);
     Wire.write(subAddress);
     Wire.write(data);
     Wire.endTransmission();
}

uint8_t read_byte_local(uint8_t address, uint8_t subAddress) {
    uint8_t data = 0;
    
    // Step 1: Tell the MPU which register you want to read
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false); // Send a restart, keeping the connection alive

    // Step 2: Request 1 byte from the device
    Wire.requestFrom(address, (uint8_t)1);
    
    if (Wire.available()) {
        data = Wire.read();
    }
    
    return data;
}

/**
 * @brief Collects IMU data to compute bias errors.
 * @param samples Number of samples to collect.
 * @param result_accel_g Output array to store accelerometer bias errors in 'g'.
 * @param result_gyro_dps Output array to store gyroscope bias errors in 'deg/s'.
 * @details Collects 'samples' readings from the IMU at ~125Hz, computes
 * the average bias error for each axis, and outputs the results.
 */
void test_bias_for_adjust(int samples, float result_accel_g[3], float result_gyro_dps[3]) {
    static bool headerPrinted_test_bias = false; // For printing header only once
    
    // Prints header once
    if (!headerPrinted_test_bias && samples > 0) {
        DEBUG_PRINT_F("Time_ms,RawErrAx_g,RawErrAy_g,RawErrAz_g,RawErrGx_dps,RawErrGy_dps,RawErrGz_dps\n");
        headerPrinted_test_bias = true;
    }
    if (samples == 0) { // If no samples are requested, zero the results and return
        for(int i=0; i<3; ++i) { result_accel_g[i] = 0.0f; result_gyro_dps[i] = 0.0f; }
        return;
    }

    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    
    uint32_t prev_time_micros = micros();
    const uint32_t sample_interval_micros = 8000; 

    if (!mpu.update()) {
        DEBUG_PRINTLN_F("FUNCTIONS_IMU : Failed to update IMU.");
        // Zero results on failure
        for(int i=0; i<3; ++i) { result_accel_g[i] = 0.0f; result_gyro_dps[i] = 0.0f; }
        return;
    }

    for (int i = 0; i < samples; i++) {
        while (micros() - prev_time_micros < sample_interval_micros) {
        }
        prev_time_micros = micros(); 

        if (mpu.update()) { 
            accel_sum[0] += mpu.getAccX(); // Unit: 'g'
            accel_sum[1] += mpu.getAccY(); // Unit: 'g'
            accel_sum[2] += mpu.getAccZ(); // Unit: 'g' 
            
            gyro_sum[0] += mpu.getGyroX(); // Unit: 'dps'
            gyro_sum[1] += mpu.getGyroY(); // Unit: 'dps'
            gyro_sum[2] += mpu.getGyroZ(); // Unit: 'dps'
            
            // Gravity compensation on Z axis
            if (PHYSICAL_Z_AXIS_DOWN) {
                accel_sum[2] -= 1.0f; 
            } else {
                accel_sum[2] += 1.0f; 
            }
        } else {
            DEBUG_PRINTLN_F("FUNCTIONS_IMU : Failed to update IMU inside sampling loop.");
            i--; 
            delay(1); 
        }
    }
    // Calculates the averages to obtain the bias errors    
    for (uint8_t axis = 0; axis < 3; axis++) {
        result_accel_g[axis] = accel_sum[axis] / samples;
        result_gyro_dps[axis] = gyro_sum[axis] / samples;
    }

    DEBUG_PRINT(millis()); 
    DEBUG_PRINT_F(",");
    for (int i = 0; i < 3; i++) { 
        DEBUG_PRINT(result_accel_g[i], 5); 
        DEBUG_PRINT_F(","); 
    }
    for (int i = 0; i < 3; i++) { 
        DEBUG_PRINT(result_gyro_dps[i], 5); 
        if (i < 2) DEBUG_PRINT_F(",");
    }
    DEBUG_PRINTLN_F(""); 
}

/**
 * @brief Iteratively adjusts the IMU calibration biases.
 * @param samples_per_iteration Number of samples to collect per iteration.
 * @param print_debug If true, prints debug information to Serial.
 * @param accel_tol_g Tolerance for the accelerometer error in 'g'.
 * @param gyro_tol_dps Tolerance for the gyroscope error in 'deg/s'.
 * @param max_iter Maximum number of iterations to avoid infinite loops.
 * @details Measures the residual error and adjusts the offsets using a proportional gain.
 * @return true If calibration converged within tolerances.
 */
bool adjustCalibrationIteratively(int samples_per_iteration, bool print_debug, float accel_tol_g, float gyro_tol_dps, int max_iter) {
    DEBUG_PRINTLN_F("FUNCTIONS_IMU: Starting iterative fine-tuning...");
    bool calibrated = false;

    // Change MPU configuration to high sensitivity for fine-tuning
    MPU9250Setting calibFineTuneConfig;
    calibFineTuneConfig.accel_fs_sel = ACCEL_FS_SEL::A16G;
    calibFineTuneConfig.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
    // Keep other settings from main configuration
    calibFineTuneConfig.fifo_sample_rate = mpuConfig.fifo_sample_rate;
    calibFineTuneConfig.gyro_dlpf_cfg = mpuConfig.gyro_dlpf_cfg;
    calibFineTuneConfig.accel_dlpf_cfg = mpuConfig.accel_dlpf_cfg;
    calibFineTuneConfig.gyro_fchoice = mpuConfig.gyro_fchoice;
    calibFineTuneConfig.accel_fchoice = mpuConfig.accel_fchoice;

    for (int iter = 0; iter < max_iter; ++iter) {
        // Guarantee MPU is set to fine-tuning configuration
        if (!mpu.setup(0x68, calibFineTuneConfig)) {
            DEBUG_PRINTLN_F("FUNCTIONS_IMU: Failed to configure MPU for fine-tuning!");
            return false;
        }
        loadCalibration(false); // Load current biases for this configuration

        if (print_debug) {
            DEBUG_PRINTF("Fine tuning iteration: %d / %d\n", (iter + 1), max_iter);
        }

        float current_accel_bias_LSB[3], current_gyro_bias_LSB[3];
        float error_accel_g[3], error_gyro_dps[3];
        float new_accel_bias_LSB[3], new_gyro_bias_LSB[3];

        float Kp_accel = 0.15;
        float Kp_gyro  = 0.5;

        current_accel_bias_LSB[0] = mpu.getAccBiasX();
        current_accel_bias_LSB[1] = mpu.getAccBiasY();
        current_accel_bias_LSB[2] = mpu.getAccBiasZ();
        current_gyro_bias_LSB[0] = mpu.getGyroBiasX();
        current_gyro_bias_LSB[1] = mpu.getGyroBiasY();
        current_gyro_bias_LSB[2] = mpu.getGyroBiasZ();

        test_bias_for_adjust(samples_per_iteration, error_accel_g, error_gyro_dps);

        if (print_debug) {
            print_vector("  Error Accel (g)", error_accel_g, 1.0f, 5, "g");
            print_vector("  Error Gyro (dps)", error_gyro_dps, 1.0f, 5, "dps");
        }

        bool accel_ok = true;
        for (int i = 0; i < 3; i++) if (abs(error_accel_g[i]) > accel_tol_g) accel_ok = false;
        bool gyro_ok = true;
        for (int i = 0; i < 3; i++) if (abs(error_gyro_dps[i]) > gyro_tol_dps) gyro_ok = false;

        if (accel_ok && gyro_ok) {
            if (print_debug) DEBUG_PRINTLN_F("Fine tuning completed! Tolerance achieved.");
            calibrated = true;
            break; 
        }

        if (print_debug) DEBUG_PRINTLN_F("  Adjusting biases...");
        for (int i = 0; i < 3; i++) {
            float error_accel_in_LSB = error_accel_g[i] * ACCEL_CALIB_SENSITIVITY_FS;
            new_accel_bias_LSB[i] = current_accel_bias_LSB[i] + (error_accel_in_LSB * Kp_accel);

            float error_gyro_in_LSB = error_gyro_dps[i] * GYRO_CALIB_SENSITIVITY_FS;
            new_gyro_bias_LSB[i]  = current_gyro_bias_LSB[i]  + (error_gyro_in_LSB * Kp_gyro);
        }

        mpu.setAccBias(new_accel_bias_LSB[0], new_accel_bias_LSB[1], new_accel_bias_LSB[2]);
        mpu.setGyroBias(new_gyro_bias_LSB[0], new_gyro_bias_LSB[1], new_gyro_bias_LSB[2]);
        saveCalibration(false); 
        
        if (print_debug) DEBUG_PRINTLN_F("  --- End of Fine Tuning Iteration ---");
    }

    if (!calibrated && print_debug) {
        DEBUG_PRINTLN_F("FUNCTIONS_IMU: Maximum number of fine-tuning iterations reached.");
    }
    return calibrated;
}

/**
 * @brief Performs a full IMU calibration sequence.
 * @details Calibrates the accelerometer, gyroscope and magnetometer using
 * the MPU9250 library functions, then saves the results to EEPROM.
 * @warning The IMU must be kept still and level during accelerometer/gyroscope calibration,
 * and moved in a figure-eight pattern during magnetometer calibration.
 * @param print If true, prints the progress.
 * @param perform_fine_tuning If true, performs fine tuning after the main calibration.
 */
void calibration_IMU(bool print_debug, bool perform_fine_tuning) {
    DEBUG_PRINTLN_F("FUNCTIONS_IMU: Starting full IMU calibration...");
    mpu.verbose(false);

    DEBUG_PRINTLN_F("Calibrating Accel/Gyro (MPU9250 lib)... Keep still and level.");
    if(print_debug) mpu.verbose(true);
    delay(2000);
    mpu.calibrateAccelGyro(PHYSICAL_Z_AXIS_DOWN); 

    DEBUG_PRINTLN_F("Calibrating Magnetometer (MPU9250 lib)... Move in a figure-eight pattern.");
    delay(2000);
    mpu.calibrateMag();
    if(print_debug) mpu.verbose(false);

    saveCalibration(print_debug); 
    DEBUG_PRINTLN_F("Calibration with MPU9250 lib finished and saved to EEPROM.");

    if (perform_fine_tuning) {
        DEBUG_PRINTLN_F("Fine tuning - keep the sensor still again");
        delay(3000);
        float accel_tol_g = 0.0025;
        float gyro_tol_dps = 0.025;
        int max_iter = 30;
        adjustCalibrationIteratively(50, print_debug, accel_tol_g, gyro_tol_dps, max_iter);
    }
    DEBUG_PRINTLN_F("FUNCTIONS_IMU: Calibration process completed.");
}

/**
 * @brief Initializes the IMU hardware and configures flight parameters.
 * * @details Configures I2C, sets scales (Acc: 16G, Gyro: 2000dps), and Digital Low Pass 
 * Filter (DLPF) bandwidth to reduce motor vibration.
 * * @param calibrate_if_needed If true, calibrates if the EEPROM is empty.
 * @param perform_fine_tuning_on_calib If true, runs an iterative fine-tuning algorithm.
 * @param print_params If true, prints the configurations.
 * @return true If the MPU9250 is detected and initialized.
 */
bool setup_IMU(bool calibrate_if_needed, bool perform_fine_tuning_on_calib, bool print_params) {
    DEBUG_PRINTLN_F("SETUP_IMU: Initializing MPU9250..."); 

    // Define the final FLIGHT configuration
    mpuConfig.accel_fs_sel = ACCEL_FS_SEL::A16G;
    mpuConfig.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    mpuConfig.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    mpuConfig.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    mpuConfig.gyro_fchoice = 0x03;
    mpuConfig.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    mpuConfig.accel_fchoice = 0x01;
    mpuConfig.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
    // Configure magnetic declination (examples: Sp:-21.46, Pira:-21.47, Midland: 5.32, Munchen: 4.27)
    // CHANGE FOR COMPETITION if needed
    mpu.setMagneticDeclination(4.6); // Adjust according to your actual launch location

    // Apply the initial flight configuration
    if (!mpu.setup(0x68, mpuConfig)) { 
        DEBUG_PRINTLN_F("SETUP_IMU: ERROR - MPU failed to connect or initial setup.");
        return false;
    }
    DEBUG_PRINTLN_F("SETUP_IMU: MPU9250 connected and flight configuration applied.");

    // Check if calibration data exists or if a new calibration is needed
    if (hasCalibrationDataIMU()) {
        DEBUG_PRINTLN_F("SETUP_IMU: Existing calibration found in EEPROM.");
        if (perform_fine_tuning_on_calib) {
            DEBUG_PRINTLN_F("SETUP_IMU: Performing fine-tuning on existing calibration...");
            adjustCalibrationIteratively(50, print_params, 0.0025, 0.025, 30);
        }
    } else {
        DEBUG_PRINTLN_F("SETUP_IMU: No IMU calibration found.");
        if (calibrate_if_needed) {
            DEBUG_PRINTLN_F("SETUP_IMU: Performing full calibration...");
            calibration_IMU(print_params, perform_fine_tuning_on_calib);
        } else {
            DEBUG_PRINTLN_F("SETUP_IMU: Proceeding without calibration (will use zero biases).");
        }
    }

    // Ensure the MPU is set to the flight configuration
    DEBUG_PRINTLN_F("SETUP_IMU: Reapplying flight configuration and loading final biases...");
    if (!mpu.setup(0x68, mpuConfig)) { 
        DEBUG_PRINTLN_F("SETUP_IMU: ERROR - Failed to reapply flight config to MPU.");
        return false;
    }
    loadCalibration(print_params); // Load latest biases from EEPROM and apply to the mpu object
    reset_orientation(PHYSICAL_Z_AXIS_DOWN);
    DEBUG_PRINTLN_F("SETUP_IMU: Completed successfully and ready for flight.");
    return true;
}

/**
 * @brief Resets the orientation (Quaternions) to default.
 * * @param PHYSICAL_Z_AXIS_DOWN If true, considers the sensor's Z-axis pointing downwards.
 * Useful if the filter diverges significantly before launch.
 */
void reset_orientation(bool PHYSICAL_Z_AXIS_DOWN){
    mpu.resetOrientation(PHYSICAL_Z_AXIS_DOWN);
}

/**
 * @brief Enables or disables continuous drift learning.
 * * @details When enabled, the filter slowly adjusts gyroscope biases over time 
 * to compensate for drift. This is achieved using the filter's zeta parameter.
 * * @param enabled If true, activates continuous drift learning.
 * Useful for long periods before launch.
 */
void setDriftLearning(bool enabled) {
    mpu.setDriftLearning(enabled);
}

/**
 * @brief Adjusts the Madgwick filter beta parameter.
 * * @details The beta parameter controls the filter's convergence speed.
 * * @param errorDegPerSec Expected error value in deg/s.
 * Higher values make the filter respond faster but may introduce more noise.
 */
void setFilterBeta(float errorDegPerSec) {
    mpu.setFilterBeta(errorDegPerSec);
}

/**
 * @brief Prints the current orientation (Euler Angles) for debugging.
 */
void print_roll_pitch_yaw() {
    DEBUG_PRINTF("Yaw: %.2f | Pitch: %.2f | Roll: %.2f  ",mpu.getYaw(),mpu.getPitch(),mpu.getRoll() );
}

/**
 * @brief Calculates the rocket's tilt relative to the vertical.
 * * @details Uses Quaternions to calculate the angle between the body's Z-axis 
 * and the gravity vector. Essential for safety (e.g., preventing airbrake deployment if tilted).
 * * @note 0° = Body Z-axis aligned with World Z (up or down, depending on PHYSICAL_Z_AXIS_DOWN).
 * 90° = Rocket is horizontal.
 * @return float Tilt angle in degrees [0° to 180°].
 */
float calcTilt() { 

    float qx = mpu.getQuaternionX();
    float qy = mpu.getQuaternionY();

    // Avoid reading QW and QZ to reduce math operations
    float cos_theta = 1.0f - 2.0f * (qx * qx + qy * qy);

    // Essential to prevent floating-point errors producing NaN in acosf
    if (cos_theta > 1.0f) {
        cos_theta = 1.0f;
    } else if (cos_theta < -1.0f) {
        cos_theta = -1.0f;
    }

    float tilt_rad = acosf(cos_theta);

    float tilt_deg = tilt_rad * RAD_TO_DEG;

    if (!PHYSICAL_Z_AXIS_DOWN){
        tilt_deg = (180.0f - tilt_deg);
    }
    
    return tilt_deg;
}

/**
 * @brief Calculates the net vertical acceleration of the rocket in the Earth frame.
 * * @details Performs a reference frame transformation of the Z acceleration vector, 
 * taking raw accelerometer data (body frame) and rotating it to the 
 * World Frame (NED) using the current Quaternions.
 * * Math: Projects the acceleration vector [ax, ay, az] onto the gravity direction 
 * and subtracts 1G to obtain pure motion acceleration.
 * * @note Assumes mpu.update() has already been called in the main loop.
 * * @return float Vertical acceleration in [m/s²].
 * Positive = Ascending (accelerating upwards).
 * Zero = Stationary or constant velocity.
 */
float computeNetAcceleration(bool print_debug, float ax_input, float ay_input, float az_input, bool autoUpdate) {

    // Access quaternion values from the mpu object
    float qx = mpu.getQuaternionX();
    float qy = mpu.getQuaternionY();
    float qz = mpu.getQuaternionZ();
    float qw = mpu.getQuaternionW();

    
  
    // Raw accelerometer data in 'g'
    float ax_g, ay_g, az_g;

    if (!autoUpdate) {
        ax_g = ax_input;      ay_g = ay_input;      az_g = az_input;
    } else {
        ax_g = mpu.getAccX(); ay_g = mpu.getAccY(); az_g = mpu.getAccZ();
    }

   

    if (print_debug){
        Serial.print("qx: ");
        Serial.print(qx);
        Serial.print(" | qy: ");
        Serial.print(qy);
        Serial.print(" | qz: ");
        Serial.print(qz);
        Serial.print(" | qw: ");
        Serial.print(qw);
        Serial.print(" | ax_body: ");
        Serial.print(ax_g);
        Serial.print(" | ay_body: ");
        Serial.print(ay_g);
        Serial.print(" | az_body: ");
        Serial.println(az_g);
    }
    
    // Rotate the specific force vector (measured acceleration) from the body
    // frame into the Earth frame (NED) and extract the Z (down) component.
    // Third row components of the body-to-world rotation matrix (R_bw) for NED:
    // R_bw[2][0] (for ax_body) = 2.0f * (qx * qz - qw * qy)
    // R_bw[2][1] (for ay_body) = 2.0f * (qy * qz + qw * qx)
    // R_bw[2][2] (for az_body) = qw*qw - qx*qx - qy*qy + qz*qz

    // Pre-calc common terms
    float qxqz = qx * qz;
    float qwqy = qw * qy;
    float qyqz = qy * qz;
    float qwqx = qw * qx;
    float qx2  = qx * qx;
    float qy2  = qy * qy;

    // Vertical acceleration in World frame (in 'g'), Z axis down (Down)
    float worldZAcceleration = 
        (2.0f * (qxqz - qwqy)) * ax_g +
        (2.0f * (qyqz + qwqx)) * ay_g +
        (1.0f - 2.0f * (qx2 + qy2)) * az_g;

    // Remove gravity and convert to m/s^2
    float netVerticalAcceleration_ms2 = -(worldZAcceleration - 1.0f) * G_GRAVITATIONAL_CONSTANT;
    
    // Remove small vibrations
    if (abs(netVerticalAcceleration_ms2) < 0.2f) { 
        netVerticalAcceleration_ms2 = 0.0f;
    }

    return netVerticalAcceleration_ms2;
}

/**
 * @brief Wrapper function to execute the fine-tuning test.
 */
void test_fine_tuning(){
    DEBUG_PRINTLN_F("\n--- STARTING ITERATIVE IMU FINE-TUNING TEST ---");
    float accel_tol_g = 0.005;  // Desired tolerance in 'g'
    float gyro_tol_dps = 0.05; // Desired tolerance in 'dps'
    int maxIterations = 25; // Maximum number of adjustment attempts
    bool fineTuningDone = false;

    // Single call to the function that runs the internal iterative loop:
    fineTuningDone = adjustCalibrationIteratively(
                                    50,    // samples_per_iteration for test_bias_for_adjust
                                    false,   // print_debug
                                    accel_tol_g,
                                    gyro_tol_dps,
                                    maxIterations // max_iter for the internal adjustCalibrationIteratively loop
                                );


    if (fineTuningDone) {
        signalSuccessfullModule("IMU Fine Tune");
        DEBUG_PRINTLN_F("--- ITERATIVE IMU FINE-TUNING TEST COMPLETED SUCCESSFULLY ---");
        delay(5000);
    } else {
        signalFailedModule("IMU Fine Tune");
        DEBUG_PRINTLN_F("--- ITERATIVE IMU FINE-TUNING TEST: TOLERANCE NOT REACHED AFTER MAX ITERATIONS ---");
        delay(5000);
    }
    
}

