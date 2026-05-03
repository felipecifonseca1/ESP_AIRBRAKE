#include "MPU9250_HAL.h"
#include "Config_voo.h"
#include <Preferences.h>


// NVS namespace and magic value for calibration validity
static constexpr char NVS_NS[] = "imu_calib";
static constexpr uint32_t NVS_MAGIC = 0xCAFEBABEu;

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
  if (verbose)
    DEBUG_PRINTLN_F("Initializing MPU from HAL...");

  forceBypass();

  // Record the flight configuration
  _mpuConfig.accel_fs_sel = ACCEL_FS_SEL::A16G;
  _mpuConfig.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  _mpuConfig.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  _mpuConfig.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  _mpuConfig.gyro_fchoice = 0x03;
  _mpuConfig.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
  _mpuConfig.accel_fchoice = 0x01;
  _mpuConfig.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_21HZ;

  bool success = _mpu.setup(_i2cAddr, _mpuConfig);
  if (!success) {
    DEBUG_PRINTLN_F("!!!! ERROR !!!!: MPU9250 Magnetometer (AK8963) setup "
                    "FAILED! Mag data will be 0.00.");
  }

  if (success) {
    if (autoCalibrate) {
      if (ERASE_CALIB_ON_STARTUP) {
        eraseCalibration(CalibEraseType::ALL);
        DEBUG_PRINTLN_F("BOOT: Erasing IMU calibration from EEPROM...");
      }

      if (ERASE_CALIB_ON_STARTUP || !hasCalibrationData()) {
        runFullCalibration(PRINT_IMU_PARAMS, PERFORM_FINE_TUNING,
                           PHYSICAL_Z_AXIS_DOWN);
      } else {
        if (PERFORM_FINE_TUNING) {
          DEBUG_PRINTLN_F("Performing iterative Fine-Tuning update...");
          adjustCalibrationIteratively(
              50, true, CALIBRATION_ACCEL_TOL_G, CALIBRATION_GYRO_TOL_DPS,
              CALIBRATION_MAX_ITERATIONS, PHYSICAL_Z_AXIS_DOWN);
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
 * @return true if a new, non-stale sample was available and read.
 */
bool MPU9250_HAL::update(float dt) {
  _magDataFresh = true; // Always allow filtering in the 50Hz loop for smoother
                        // interpolation
  bool fresh = _mpu.update(dt);

  // Stale-data watchdog: 5 consecutive identical accel readings = sensor frozen
  float ax = _mpu.getAccX(), ay = _mpu.getAccY(), az = _mpu.getAccZ();
  bool identical = (ax == _lastAx) && (ay == _lastAy) && (az == _lastAz);
  _lastAx = ax;
  _lastAy = ay;
  _lastAz = az;

  if (identical) {
    if (_staleCount < 255)
      _staleCount++;
  } else {
    _staleCount = 0;
  }
  _dataStale = (_staleCount >= 5);

  if (fresh || _magDataFresh) {
    float cal_out[3];
    // The underlying library outputs milliGauss (mG). Multiply by 0.1 to
    // convert to uT.
    float mx_uT = _mpu.getMagX() * 0.1f;
    float my_uT = _mpu.getMagY() * 0.1f;
    float mz_uT = _mpu.getMagZ() * 0.1f;
    applyMagCalibration(mx_uT, my_uT, mz_uT, cal_out);
    _magCalX = cal_out[0];
    _magCalY = cal_out[1];
    _magCalZ = cal_out[2];
  }

  (void)fresh;
  return !_dataStale;
}

/**
 * @brief Injects external sensor data for HIL (Hardware-In-the-Loop)
 * simulations.
 */
void MPU9250_HAL::injectData(float ax, float ay, float az, float gx, float gy,
                             float gz, float mx, float my, float mz) {

  _mpu.update(Ts, false, false, ax, ay, az, gx * RAD_TO_DEG, gy * RAD_TO_DEG,
              gz * RAD_TO_DEG, mx, my, mz);
  // For HIL simulation, the injected mx, my, mz are already calibrated.
  _magCalX = _mpu.getMagX();
  _magCalY = _mpu.getMagY();
  _magCalZ = _mpu.getMagZ();
}

/**
 * @brief Trigger the library's internal accelerometer/gyroscope calibration.
 */
void MPU9250_HAL::calibrateAccel() {
  _mpu.calibrateAccelGyro(PHYSICAL_Z_AXIS_DOWN);
}

/**
 * @brief Trigger the library's internal accelerometer/gyroscope calibration.
 */
void MPU9250_HAL::calibrateGyro() {
  _mpu.calibrateAccelGyro(PHYSICAL_Z_AXIS_DOWN);
}

/**
 * @brief Trigger the library's internal magnetometer calibration.
 */
/**
 * @brief Trigger the library's internal magnetometer calibration.
 */
void MPU9250_HAL::calibrateMag() {
  _mpu.calibrateMag();

  // Capture the library's calculated scalar bias/scale into our 3x3 framework
  // Since the library calculates bias in mG, we convert it to uT
  _magBias[0] = _mpu.getMagBiasX() * 0.1f;
  _magBias[1] = _mpu.getMagBiasY() * 0.1f;
  _magBias[2] = _mpu.getMagBiasZ() * 0.1f;

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      _magScaleMatrix[r][c] = 0.0f;
    }
  }
  _magScaleMatrix[0][0] = _mpu.getMagScaleX();
  _magScaleMatrix[1][1] = _mpu.getMagScaleY();
  _magScaleMatrix[2][2] = _mpu.getMagScaleZ();

  // Reset library internals so calibration isn't applied twice
  _mpu.setMagBias(0, 0, 0);
  _mpu.setMagScale(1, 1, 1);
}

// -------------------------------------------------------------------------
// Helper matrix solvers for Ellipsoid fit
// -------------------------------------------------------------------------
static bool gauss_jordan9(float a[9][9], float b[9], float x[9]) {
  float m[9][10];
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++)
      m[i][j] = a[i][j];
    m[i][9] = b[i];
  }
  for (int i = 0; i < 9; i++) {
    int max_row = i;
    for (int k = i + 1; k < 9; k++)
      if (fabsf(m[k][i]) > fabsf(m[max_row][i]))
        max_row = k;
    if (fabsf(m[max_row][i]) < 1e-6f)
      return false;
    if (max_row != i) {
      for (int j = i; j < 10; j++) {
        float tmp = m[i][j];
        m[i][j] = m[max_row][j];
        m[max_row][j] = tmp;
      }
    }
    float div = m[i][i];
    for (int j = i; j < 10; j++)
      m[i][j] /= div;
    for (int k = 0; k < 9; k++) {
      if (k != i) {
        float factor = m[k][i];
        for (int j = i; j < 10; j++)
          m[k][j] -= factor * m[i][j];
      }
    }
  }
  for (int i = 0; i < 9; i++)
    x[i] = m[i][9];
  return true;
}

static bool gauss_jordan3(float a[3][3], float b[3], float x[3]) {
  float m[3][4];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      m[i][j] = a[i][j];
    m[i][3] = b[i];
  }
  for (int i = 0; i < 3; i++) {
    int max_row = i;
    for (int k = i + 1; k < 3; k++)
      if (fabsf(m[k][i]) > fabsf(m[max_row][i]))
        max_row = k;
    if (fabsf(m[max_row][i]) < 1e-6f)
      return false;
    if (max_row != i) {
      for (int j = i; j < 4; j++) {
        float tmp = m[i][j];
        m[i][j] = m[max_row][j];
        m[max_row][j] = tmp;
      }
    }
    float div = m[i][i];
    for (int j = i; j < 4; j++)
      m[i][j] /= div;
    for (int k = 0; k < 3; k++) {
      if (k != i) {
        float factor = m[k][i];
        for (int j = i; j < 4; j++)
          m[k][j] -= factor * m[i][j];
      }
    }
  }
  for (int i = 0; i < 3; i++)
    x[i] = m[i][3];
  return true;
}

/**
 * @brief High-speed application of the 3x3 Soft Iron matrix and 3x1 Hard Iron
 * bias
 */
void MPU9250_HAL::applyMagCalibration(float raw_x, float raw_y, float raw_z,
                                      float *cal_out) const {
  float x = raw_x - _magBias[0];
  float y = raw_y - _magBias[1];
  float z = raw_z - _magBias[2];

  cal_out[0] = _magScaleMatrix[0][0] * x + _magScaleMatrix[0][1] * y +
               _magScaleMatrix[0][2] * z;
  cal_out[1] = _magScaleMatrix[1][0] * x + _magScaleMatrix[1][1] * y +
               _magScaleMatrix[1][2] * z;
  cal_out[2] = _magScaleMatrix[2][0] * x + _magScaleMatrix[2][1] * y +
               _magScaleMatrix[2][2] * z;
}

/**
 * @brief Performs a 30-second "Figure-8" visual calibration for the
 * Magnetometer. Uses a least-squares ellipsoid fit to calculate a 3x3 Soft Iron
 * and 3x1 Hard Iron matrix.
 */
void MPU9250_HAL::calibrateMagVisual() {
  DEBUG_PRINTLN_F("\n--- MAGNETOMETER VISUAL CALIBRATION ---");

  // Ensure I2C Bypass is active
  forceBypass();

  // Verify Magnetometer is actually there
  Wire.beginTransmission(0x0C); // AK8963 Address
  Wire.write(0x00);             // WHO_AM_I
  uint8_t err = Wire.endTransmission(false);

  uint8_t magId = 0xFF;
  if (err == 0) {
    Wire.requestFrom(0x0C, (uint8_t)1);
    magId = Wire.available() ? Wire.read() : 0xFF;
  } else {
    DEBUG_PRINTF("I2C ERROR AK8963 Bypass Check: %d\n", err);
  }

  DEBUG_PRINT_F("AK8963 WHO_AM_I: 0x");
  DEBUG_PRINTLN(magId, HEX);

  if (magId != 0x48) {
    DEBUG_PRINTLN_F(
        "!!!! ERROR: AK8963 not found. Bypass FAILED! Mag will be zero.");
  }

  // Force 100Hz 16-bit Continuous Mode (CNTL1 = 0x16)
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x16);
  Wire.endTransmission();
  delay(100);

  DEBUG_PRINTLN_F("GET READY... HOLD STILL (2s)");
  delay(2000);

  // Bypass library-level scaling/bias so we get purely raw data
  _mpu.setMagBias(0, 0, 0);
  _mpu.setMagScale(1, 1, 1);

  // DTD and DT1 accumulators for 9x9 normal equations: D^T D * theta = D^T X
  float DTD[9][9] = {0};
  float DT1[9] = {0};

  float m_min[3] = {32767.0f, 32767.0f, 32767.0f};
  float m_max[3] = {-32768.0f, -32768.0f, -32768.0f};
  bool firstSample = true;
  uint32_t numSamples = 0;

  DEBUG_PRINTLN_F("CALIBRATING... SPIN THE ROCKET IN ALL DIRECTIONS (30s)!");

  uint32_t startTime = millis();
  uint32_t lastPrint = 0;

  while (millis() - startTime < 30000) {
    if (_mpu.update_mag()) {
      // Convert library mG output to uT natively
      float mx_raw = _mpu.getMagX() * 0.1f;
      float my_raw = _mpu.getMagY() * 0.1f;
      float mz_raw = _mpu.getMagZ() * 0.1f;

      // Skip junk zeros at startup
      if (abs(mx_raw) > 0.001f || abs(my_raw) > 0.001f ||
          abs(mz_raw) > 0.001f) {
        // Scale data to prevent float precision overflow in x^2, y^2
        float scale_factor = 1000.0f;
        float mx = mx_raw / scale_factor;
        float my = my_raw / scale_factor;
        float mz = mz_raw / scale_factor;

        // Vector v = [y^2, z^2, 2xy, 2xz, 2yz, 2x, 2y, 2z, 1]
        float v[9] = {my * my,        mz * mz,        2.0f * mx * my,
                      2.0f * mx * mz, 2.0f * my * mz, 2.0f * mx,
                      2.0f * my,      2.0f * mz,      1.0f};

        // Equation: a y^2 + b z^2 + 2c xy + 2d xz + 2e yz + 2f x + 2g y + 2h z
        // + i = -x^2
        float target = -(mx * mx);

        for (int i = 0; i < 9; i++) {
          DT1[i] += v[i] * target;
          for (int j = 0; j < 9; j++) {
            DTD[i][j] += v[i] * v[j];
          }
        }

        if (firstSample) {
          m_min[0] = m_max[0] = mx_raw;
          m_min[1] = m_max[1] = my_raw;
          m_min[2] = m_max[2] = mz_raw;
          firstSample = false;
        } else {
          if (mx_raw < m_min[0])
            m_min[0] = mx_raw;
          if (mx_raw > m_max[0])
            m_max[0] = mx_raw;
          if (my_raw < m_min[1])
            m_min[1] = my_raw;
          if (my_raw > m_max[1])
            m_max[1] = my_raw;
          if (mz_raw < m_min[2])
            m_min[2] = mz_raw;
          if (mz_raw > m_max[2])
            m_max[2] = mz_raw;
        }
        numSamples++;
      }
    }

    if (millis() - lastPrint > 500) {
      int progress = (int)((millis() - startTime) * 100 / 30000);
      DEBUG_PRINT_F("P:");
      DEBUG_PRINT(progress);
      DEBUG_PRINT_F("% | ");
      if (firstSample) {
        DEBUG_PRINTLN_F("Searching for Magnetometer signal...");
      } else {
        DEBUG_PRINT_F("Samples: ");
        DEBUG_PRINT(numSamples);
        DEBUG_PRINTLN_F(" (Keep Rotating)");
      }
      lastPrint = millis();
    }
    yield();
  }

  if (numSamples < 100) {
    DEBUG_PRINTLN_F(
        "!!!! ERROR: Not enough Mag data collected. Calibration aborted.");
    return;
  }

  // Solve for theta
  float theta[9];
  if (!gauss_jordan9(DTD, DT1, theta)) {
    DEBUG_PRINTLN_F("!!!! ERROR: Ellipsoid fit failed (Singular matrix). Did "
                    "you rotate in all 3D directions?");
    return;
  }

  // theta = [a, b, c, d, e, f, g, h, i]
  float a = theta[0], b = theta[1], c = theta[2];
  float d = theta[3], e = theta[4];
  float f = theta[5], g = theta[6], h = theta[7];
  float i_coeff = theta[8];

  // Q matrix
  float Q[3][3] = {{1.0f, c, d}, {c, a, e}, {d, e, b}};
  // U vector
  float U[3] = {f, g, h};

  // Find Bias V_bias = -Q^-1 * U
  // To do this, solve Q * V_bias = -U
  float negU[3] = {-f, -g, -h};
  float V_bias[3];
  if (!gauss_jordan3(Q, negU, V_bias)) {
    DEBUG_PRINTLN_F("!!!! ERROR: Matrix Q singular.");
    return;
  }

  // Unscale the bias
  _magBias[0] = V_bias[0] * 1000.0f;
  _magBias[1] = V_bias[1] * 1000.0f;
  _magBias[2] = V_bias[2] * 1000.0f;

  float k = -(U[0] * V_bias[0] + U[1] * V_bias[1] + U[2] * V_bias[2] + i_coeff);

  // M = Q / k
  float M[3][3];
  for (int r = 0; r < 3; r++) {
    for (int c_idx = 0; c_idx < 3; c_idx++) {
      M[r][c_idx] = Q[r][c_idx] / k;
    }
  }

  // Target magnetic field magnitude radius (preserves local muT strength
  // instead of normalizing to 1)
  float deltaX = m_max[0] - m_min[0];
  float deltaY = m_max[1] - m_min[1];
  float deltaZ = m_max[2] - m_min[2];
  float target_R = (deltaX + deltaY + deltaZ) / 6.0f;

  // Cholesky decomposition of M to find L (M = L * L^T)
  bool cholesky_ok = true;
  float L[3][3] = {0};

  if (M[0][0] <= 0.0f)
    cholesky_ok = false;
  else {
    L[0][0] = sqrtf(M[0][0]);
    L[1][0] = M[1][0] / L[0][0];
    L[2][0] = M[2][0] / L[0][0];

    float M11_sub = M[1][1] - L[1][0] * L[1][0];
    if (M11_sub <= 0.0f)
      cholesky_ok = false;
    else {
      L[1][1] = sqrtf(M11_sub);
      L[2][1] = (M[2][1] - L[2][0] * L[1][0]) / L[1][1];

      float M22_sub = M[2][2] - L[2][0] * L[2][0] - L[2][1] * L[2][1];
      if (M22_sub <= 0.0f)
        cholesky_ok = false;
      else {
        L[2][2] = sqrtf(M22_sub);
      }
    }
  }

  float W_scale = target_R / 1000.0f;
  bool extreme_scale = false;
  if (cholesky_ok) {
    // If the soft iron scale distorts the radius by more than 3x, the rotation
    // was incomplete
    if (L[0][0] * W_scale > 3.0f || L[0][0] * W_scale < 0.33f)
      extreme_scale = true;
    if (L[1][1] * W_scale > 3.0f || L[1][1] * W_scale < 0.33f)
      extreme_scale = true;
    if (L[2][2] * W_scale > 3.0f || L[2][2] * W_scale < 0.33f)
      extreme_scale = true;
  }

  bool bad_center = false;
  // If the fitted center is wildly outside the min/max bounds, it's a
  // hyperboloid not an ellipsoid
  if (_magBias[0] < m_min[0] - deltaX || _magBias[0] > m_max[0] + deltaX)
    bad_center = true;
  if (_magBias[1] < m_min[1] - deltaY || _magBias[1] > m_max[1] + deltaY)
    bad_center = true;
  if (_magBias[2] < m_min[2] - deltaZ || _magBias[2] > m_max[2] + deltaZ)
    bad_center = true;

  if (!cholesky_ok || extreme_scale || bad_center) {
    DEBUG_PRINTLN_F("!!!! WARNING: 3D Ellipsoid fit unstable (Insufficient 3D "
                    "rotation or Non-Positive Definite).");
    DEBUG_PRINTLN_F(
        "!!!! FALLING BACK to legacy scalar Min/Max calibration...");

    // Calculate Bias (Hard Iron)
    _magBias[0] = (m_max[0] + m_min[0]) / 2.0f;
    _magBias[1] = (m_max[1] + m_min[1]) / 2.0f;
    _magBias[2] = (m_max[2] + m_min[2]) / 2.0f;

    // Calculate Scale (Soft Iron)
    float avgDelta = (deltaX + deltaY + deltaZ) / 3.0f;
    float msx = (deltaX > 0.1f) ? (avgDelta / deltaX) : 1.0f;
    float msy = (deltaY > 0.1f) ? (avgDelta / deltaY) : 1.0f;
    float msz = (deltaZ > 0.1f) ? (avgDelta / deltaZ) : 1.0f;

    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        _magScaleMatrix[r][c] = 0.0f;
      }
    }
    _magScaleMatrix[0][0] = msx;
    _magScaleMatrix[1][1] = msy;
    _magScaleMatrix[2][2] = msz;
  } else {
    // Soft iron calibration matrix W is L^T, scaled by the target radius /
    // 1000.0f
    _magScaleMatrix[0][0] = L[0][0] * W_scale;
    _magScaleMatrix[0][1] = L[1][0] * W_scale;
    _magScaleMatrix[0][2] = L[2][0] * W_scale;
    _magScaleMatrix[1][0] = 0.0f;
    _magScaleMatrix[1][1] = L[1][1] * W_scale;
    _magScaleMatrix[1][2] = L[2][1] * W_scale;
    _magScaleMatrix[2][0] = 0.0f;
    _magScaleMatrix[2][1] = 0.0f;
    _magScaleMatrix[2][2] = L[2][2] * W_scale;
  }

  DEBUG_PRINTLN_F("\nCALIBRATION COMPLETE!");
  saveCalibration(false);

  DEBUG_PRINT_F("Saved Mag Bias: ");
  DEBUG_PRINT(_magBias[0]);
  DEBUG_PRINT_F(", ");
  DEBUG_PRINT(_magBias[1]);
  DEBUG_PRINT_F(", ");
  DEBUG_PRINTLN(_magBias[2]);
  DEBUG_PRINT_F("Saved Mag Matrix [Row 0]: ");
  DEBUG_PRINT(_magScaleMatrix[0][0], 4);
  DEBUG_PRINT_F(", ");
  DEBUG_PRINT(_magScaleMatrix[0][1], 4);
  DEBUG_PRINT_F(", ");
  DEBUG_PRINTLN(_magScaleMatrix[0][2], 4);
  DEBUG_PRINT_F("Saved Mag Matrix [Row 1]: ");
  DEBUG_PRINT(_magScaleMatrix[1][0], 4);
  DEBUG_PRINT_F(", ");
  DEBUG_PRINT(_magScaleMatrix[1][1], 4);
  DEBUG_PRINT_F(", ");
  DEBUG_PRINTLN(_magScaleMatrix[1][2], 4);
  DEBUG_PRINT_F("Saved Mag Matrix [Row 2]: ");
  DEBUG_PRINT(_magScaleMatrix[2][0], 4);
  DEBUG_PRINT_F(", ");
  DEBUG_PRINT(_magScaleMatrix[2][1], 4);
  DEBUG_PRINT_F(", ");
  DEBUG_PRINTLN(_magScaleMatrix[2][2], 4);
}

/**
 * @brief Force MPU9250 into I2C Bypass mode.
 * Disables I2C Master and enables Bypass bit to allow direct access to AK8963.
 */
void MPU9250_HAL::forceBypass() {
  // 0x37 = INT_PIN_CFG, 0x02 = BYPASS_EN
  Wire.beginTransmission(_i2cAddr);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();

  // 0x6A = USER_CTRL, 0x00 = Disable I2C Master
  Wire.beginTransmission(_i2cAddr);
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
}

/**
 * @brief Checks if the latest update() call included a fresh magnetic sample.
 */
bool MPU9250_HAL::isMagDataNew() { return _magDataFresh; }

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
void MPU9250_HAL::printVector(const char *label, const float values[],
                              float scale, int decimals,
                              const char *unit_label) {
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
 * @brief Save current hardware biases and magnetometer params to NVS.
 * @param printDebug If true, prints the values being saved.
 */
void MPU9250_HAL::saveCalibration(bool printDebug) {
  if (printDebug)
    DEBUG_PRINTLN_F("Saving IMU calibration to NVS...");

  Preferences prefs;
  prefs.begin(NVS_NS, false); // read-write

  prefs.putUInt("magic", NVS_MAGIC);

  prefs.putFloat("abx", _mpu.getAccBiasX());
  prefs.putFloat("aby", _mpu.getAccBiasY());
  prefs.putFloat("abz", _mpu.getAccBiasZ());
  prefs.putFloat("gbx", _mpu.getGyroBiasX());
  prefs.putFloat("gby", _mpu.getGyroBiasY());
  prefs.putFloat("gbz", _mpu.getGyroBiasZ());

  prefs.putFloat("mbx", _magBias[0]);
  prefs.putFloat("mby", _magBias[1]);
  prefs.putFloat("mbz", _magBias[2]);
  prefs.putFloat("ms11", _magScaleMatrix[0][0]);
  prefs.putFloat("ms12", _magScaleMatrix[0][1]);
  prefs.putFloat("ms13", _magScaleMatrix[0][2]);
  prefs.putFloat("ms21", _magScaleMatrix[1][0]);
  prefs.putFloat("ms22", _magScaleMatrix[1][1]);
  prefs.putFloat("ms23", _magScaleMatrix[1][2]);
  prefs.putFloat("ms31", _magScaleMatrix[2][0]);
  prefs.putFloat("ms32", _magScaleMatrix[2][1]);
  prefs.putFloat("ms33", _magScaleMatrix[2][2]);

  prefs.end();
  if (printDebug)
    DEBUG_PRINTLN_F("IMU calibration saved successfully.");
}

/**
 * @brief Wipes selected portion of calibration data from NVS.
 * @param type Portions to wipe: ALL, ACCEL_GYRO, or MAG.
 */
void MPU9250_HAL::eraseCalibration(CalibEraseType type) {
  if (type == CalibEraseType::ALL) {
    DEBUG_PRINTLN_F("BOOT: Erasing ALL IMU calibration from NVS...");
    Preferences prefs;
    prefs.begin(NVS_NS, false);
    prefs.clear();
    prefs.end();
    return;
  }

  // For partial erases, we MUST have existing valid data to preserve the other
  // part
  if (!hasCalibrationData()) {
    DEBUG_PRINTLN_F(
        "BOOT: Partial erase requested but no valid data exists. Skipping.");
    return;
  }

  // 1. Load the valid totals from NVS into the _mpu object
  loadCalibration(false);

  if (type == CalibEraseType::ACCEL_GYRO) {
    DEBUG_PRINTLN_F("BOOT: Erasing Accel/Gyro calibration ONLY...");
    _mpu.setAccBias(0, 0, 0);
    _mpu.setGyroBias(0, 0, 0);
  } else if (type == CalibEraseType::MAG) {
    DEBUG_PRINTLN_F("BOOT: Erasing Magnetometer calibration ONLY...");
    _magBias[0] = _magBias[1] = _magBias[2] = 0.0f;
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        _magScaleMatrix[r][c] = (r == c) ? 1.0f : 0.0f;
      }
    }
  }

  // 2. Commit the new hybrid state back to NVS
  saveCalibration(true);
}

/**
 * @brief Load calibration data from NVS and apply to hardware registers.
 * @param printDebug If true, prints the loaded values.
 */
void MPU9250_HAL::loadCalibration(bool printDebug) {
  if (!hasCalibrationData()) {
    if (printDebug)
      DEBUG_PRINTLN_F("No valid calibration data in NVS.");
    return;
  }

  if (printDebug)
    DEBUG_PRINTLN_F("Loading IMU calibration from NVS...");

  clearHardwareOffsets(); // Start from clean slate before applying saved total

  Preferences prefs;
  prefs.begin(NVS_NS, true); // read-only

  float abx = prefs.getFloat("abx", 0.0f), aby = prefs.getFloat("aby", 0.0f),
        abz = prefs.getFloat("abz", 0.0f);
  float gbx = prefs.getFloat("gbx", 0.0f), gby = prefs.getFloat("gby", 0.0f),
        gbz = prefs.getFloat("gbz", 0.0f);

  _magBias[0] = prefs.getFloat("mbx", 0.0f);
  _magBias[1] = prefs.getFloat("mby", 0.0f);
  _magBias[2] = prefs.getFloat("mbz", 0.0f);

  _magScaleMatrix[0][0] = prefs.getFloat("ms11", 1.0f);
  _magScaleMatrix[0][1] = prefs.getFloat("ms12", 0.0f);
  _magScaleMatrix[0][2] = prefs.getFloat("ms13", 0.0f);
  _magScaleMatrix[1][0] = prefs.getFloat("ms21", 0.0f);
  _magScaleMatrix[1][1] = prefs.getFloat("ms22", 1.0f);
  _magScaleMatrix[1][2] = prefs.getFloat("ms23", 0.0f);
  _magScaleMatrix[2][0] = prefs.getFloat("ms31", 0.0f);
  _magScaleMatrix[2][1] = prefs.getFloat("ms32", 0.0f);
  _magScaleMatrix[2][2] = prefs.getFloat("ms33", 1.0f);

  prefs.end();

  _mpu.setAccBias(abx, aby, abz);
  _mpu.setGyroBias(gbx, gby, gbz);

  // We enforce raw data output from the library
  _mpu.setMagBias(0, 0, 0);
  _mpu.setMagScale(1, 1, 1);

  if (printDebug) {
    float ab[3] = {abx, aby, abz};
    float gb[3] = {gbx, gby, gbz};
    printVector("  Loaded Accel Bias", ab, 1.0f, 2, "LSB");
    printVector("  Loaded Gyro Bias", gb, 1.0f, 2, "LSB");
    printVector("  Loaded Mag Bias", _magBias, 1.0f, 2, "uT");
    DEBUG_PRINT_F("  Loaded Mag Scale Matrix [Row 0]: ");
    DEBUG_PRINT(_magScaleMatrix[0][0], 4);
    DEBUG_PRINT_F(", ");
    DEBUG_PRINT(_magScaleMatrix[0][1], 4);
    DEBUG_PRINT_F(", ");
    DEBUG_PRINTLN(_magScaleMatrix[0][2], 4);
    DEBUG_PRINT_F("  Loaded Mag Scale Matrix [Row 1]: ");
    DEBUG_PRINT(_magScaleMatrix[1][0], 4);
    DEBUG_PRINT_F(", ");
    DEBUG_PRINT(_magScaleMatrix[1][1], 4);
    DEBUG_PRINT_F(", ");
    DEBUG_PRINTLN(_magScaleMatrix[1][2], 4);
    DEBUG_PRINT_F("  Loaded Mag Scale Matrix [Row 2]: ");
    DEBUG_PRINT(_magScaleMatrix[2][0], 4);
    DEBUG_PRINT_F(", ");
    DEBUG_PRINT(_magScaleMatrix[2][1], 4);
    DEBUG_PRINT_F(", ");
    DEBUG_PRINTLN(_magScaleMatrix[2][2], 4);
    DEBUG_PRINTLN_F("IMU calibration loaded and applied.");
  }
}

/**
 * @brief Checks if valid calibration data exists in NVS.
 * @return true if calibration magic number is found and valid.
 */
bool MPU9250_HAL::hasCalibrationData() {
  Preferences prefs;
  // Open read-write so the namespace is created on first access.
  prefs.begin(NVS_NS, false);
  uint32_t magic = prefs.getUInt("magic", 0);
  prefs.end();
  return (magic == NVS_MAGIC);
}

/**
 * @brief Samples the IMU multiple times to find the average error.
 * @param samples Number of samples to average.
 * @param result_accel_g Output array for accel error [g].
 * @param result_gyro_dps Output array for gyro error [dps].
 * @param physicalZAxisDown Rocket mounting orientation (true if Z-axis points
 * down, false if up).
 */
void MPU9250_HAL::collectBiasErrors(int samples, float result_accel_g[3],
                                    float result_gyro_dps[3],
                                    bool physicalZAxisDown) {
  float accel_sum[3] = {0.0f, 0.0f, 0.0f};
  float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
  uint32_t prev_time_micros = micros();
  const uint32_t sample_interval_micros = 8000;

  for (int i = 0; i < samples; i++) {
    while (micros() - prev_time_micros < sample_interval_micros) {
    }
    prev_time_micros = micros();

    if (update()) {
      accel_sum[0] += _mpu.getAccX();
      accel_sum[1] += _mpu.getAccY();
      accel_sum[2] += _mpu.getAccZ();

      gyro_sum[0] += _mpu.getGyroX();
      gyro_sum[1] += _mpu.getGyroY();
      gyro_sum[2] += _mpu.getGyroZ();

      // Auto-detect physical gravity polarity to calculate error correctly
      if (_mpu.getAccZ() < 0.0f) {
          accel_sum[2] += 1.0f; // Sensor sees -1g, error is (Reading - (-1g))
      } else {
          accel_sum[2] -= 1.0f; // Sensor sees +1g, error is (Reading - (+1g))
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
 *          It nudges the biases until the stationary reading is perfectly
 * zeroed.
 * @param samples_per_iteration Number of samples to collect for each error
 * calculation.
 * @param print_debug If true, prints detailed debug information for each
 * iteration.
 * @param accel_tol_g Accelerometer tolerance in g.
 * @param gyro_tol_dps Gyroscope tolerance in degrees per second.
 * @param max_iter Maximum number of iterations.
 * @param physicalZAxisDown Rocket mounting orientation (true if Z-axis points
 * down, false if up).
 * @return true if convergence was achieved.
 */
bool MPU9250_HAL::adjustCalibrationIteratively(int samples_per_iteration,
                                               bool print_debug,
                                               float accel_tol_g,
                                               float gyro_tol_dps, int max_iter,
                                               bool physicalZAxisDown) {
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

    collectBiasErrors(samples_per_iteration, error_accel_g, error_gyro_dps,
                      physicalZAxisDown);

    bool accel_ok = true, gyro_ok = true;
    for (int i = 0; i < 3; i++) {
      if (abs(error_accel_g[i]) > accel_tol_g)
        accel_ok = false;
      if (abs(error_gyro_dps[i]) > gyro_tol_dps)
        gyro_ok = false;
    }

    if (print_debug) {
      DEBUG_PRINTF("\nFine tuning iteration: %d / %d\n", (iter + 1), max_iter);
      printVector("  Error Accel (g)", error_accel_g, 1.0f, 5, "g");
      printVector("  Error Gyro (dps)", error_gyro_dps, 1.0f, 5, "dps");

      float current_acc[3] = {_mpu.getAccBiasX(), _mpu.getAccBiasY(),
                              _mpu.getAccBiasZ()};
      float current_gyro[3] = {_mpu.getGyroBiasX(), _mpu.getGyroBiasY(),
                               _mpu.getGyroBiasZ()};
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
    // Logic: Output = Raw - Bias. If Output is too high (Error > 0), we
    // INCREASE Bias.
    float Kp_accel = 0.15f;
    float Kp_gyro = 0.5f;

    float current_acc[3] = {_mpu.getAccBiasX(), _mpu.getAccBiasY(),
                            _mpu.getAccBiasZ()};
    float current_gyro[3] = {_mpu.getGyroBiasX(), _mpu.getGyroBiasY(),
                             _mpu.getGyroBiasZ()};

    _mpu.setAccBias(current_acc[0] + (error_accel_g[0] * 16384.0f) * Kp_accel,
                    current_acc[1] + (error_accel_g[1] * 16384.0f) * Kp_accel,
                    current_acc[2] + (error_accel_g[2] * 16384.0f) * Kp_accel);
    _mpu.setGyroBias(current_gyro[0] + (error_gyro_dps[0] * 131.0f) * Kp_gyro,
                     current_gyro[1] + (error_gyro_dps[1] * 131.0f) * Kp_gyro,
                     current_gyro[2] + (error_gyro_dps[2] * 131.0f) * Kp_gyro);

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
void MPU9250_HAL::runFullCalibration(bool printDebug, bool performFineTuning,
                                     bool physicalZAxisDown) {
  // ALWAYS print progress for full calibration to avoid user confusion
  DEBUG_PRINTLN_F("\n--- [STEP 1/3] BASE IMU CALIBRATION ---");
  DEBUG_PRINTLN_F("Keep rocket STILL and LEVEL on the pad...");

  clearHardwareOffsets(); // Start from absolute zero
  delay(2000);

  _mpu.verbose(printDebug);
  
  // Auto-detect physical gravity polarity to prevent 2g bias overflow
  _mpu.update();
  bool sensor_reads_negative_gravity = (_mpu.getAccZ() < 0.0f);
  _mpu.calibrateAccelGyro(sensor_reads_negative_gravity);
  
  _mpu.verbose(false);

  float base_acc[3] = {_mpu.getAccBiasX(), _mpu.getAccBiasY(),
                       _mpu.getAccBiasZ()};
  float base_gyro[3] = {_mpu.getGyroBiasX(), _mpu.getGyroBiasY(),
                        _mpu.getGyroBiasZ()};
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
    adjustCalibrationIteratively(50, true, accel_tol_g, gyro_tol_dps,
                                 CALIBRATION_MAX_ITERATIONS, physicalZAxisDown);
  }
}
