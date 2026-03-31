#include "FlightController.h"
#include "DataManager.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern QueueHandle_t flightDataQueue;
#include "AltitudeSpeedTable.hh"
#include "DragCoefficientTable.hh"
#include "MPU9250_HAL.h"
#include "Sinalizacao.h"
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <math.h>


using namespace Eigen;

// --- RTC Memory Backup ---
static RTC_NOINIT_ATTR FlightRecoveryData _rtcBackup;
static const uint32_t RTC_MAGIC = 0xABBA1234;

/**
 * @brief Gets the singleton instance of the FlightController.
 * @attention The very first time this is called in setup(), the
 * BarometricSensor pointer must be provided.
 * @return Reference to the FlightController instance.
 */
FlightController &FlightController::getInstance(BarometricSensor *b,
                                                AttitudeEstimator *estimator) {
  static FlightController instance(b, estimator);
  if (b != nullptr) {
    instance.baro = b;
  }
  if (estimator != nullptr) {
    instance._attitudeEstimator = estimator;
  }
  return instance;
}

/**
 * @brief Constructor. Initializes the PID controller and resets buffers.
 */
FlightController::FlightController(BarometricSensor *b,
                                   AttitudeEstimator *estimator)
    : baro(b), _attitudeEstimator(estimator),
      _controller(PID_KP, PID_KI, PID_KD, ROCKET_MASS_KG, AIRBRAKE_REF_AREA_M2,
                  Ts) { // Initialize PID Controller using global constants

  // Initialize buffers
  for (int i = 0; i < _burnoutWindowSize; i++)
    _burnoutBufferAcc[i] = 0.0f;
  for (int i = 0; i < _regressionWindowSize; i++) {
    _regressionTimeBuffer[i] = 0.0f;
    _regressionAltBuffer[i] = 0.0f;
  }

  _flightState = FlightState::SENSOR_CALIBRATION; // Initial State
  _stateEntryTime = 0;
  _launchDetectedTime = 0;
  _apogeeDetectedTime = 0;
}

/**
 * @brief Configures the PID controller limits and parameters.
 */
void FlightController::setupController() {
  _controller.setLimits(0, 1);

  // Apply default filter selection from config
  AttitudeFilterSel selected = static_cast<AttitudeFilterSel>(DEFAULT_ATTITUDE_FILTER);
  _attitudeEstimator->selectFilter(selected);

  DEBUG_PRINT_F("FlightLogic: Attitude Filter -> ");
  switch (selected) {
    case AttitudeFilterSel::NONE:     DEBUG_PRINTLN_F("NONE"); break;
    case AttitudeFilterSel::MADGWICK: DEBUG_PRINTLN_F("MADGWICK"); break;
    case AttitudeFilterSel::MAHONY:   DEBUG_PRINTLN_F("MAHONY"); break;
    case AttitudeFilterSel::EKF:      DEBUG_PRINTLN_F("EKF (FUTURE)"); break;
    case AttitudeFilterSel::MEKF:     DEBUG_PRINTLN_F("MEKF"); break;
    case AttitudeFilterSel::NAV_MEKF: DEBUG_PRINTLN_F("NAV_MEKF"); break;
  }

  DEBUG_PRINTLN_F("Controller initialized.");
}

/**
 * @brief Initializes the Kalman Filter matrices and parameters.
 */
void FlightController::setupKalman() {
  if (_attitudeEstimator->getCurrentFilter() == AttitudeFilterSel::NAV_MEKF) {
    Matrix<float, 3, 1> initPos(0, 0, 0); // Ground altitude is 0 AGL
    Matrix<float, 3, 1> initVel(0, 0, 0);
    // AttitudeEstimator initializes orientation from gravity in setup, we can pull the quat
    Matrix<float, 4, 1> initQuat = { _attitudeEstimator->getQuaternionW(), _attitudeEstimator->getQuaternionX(), _attitudeEstimator->getQuaternionY(), _attitudeEstimator->getQuaternionZ() };
    
    // --- TUNING: Tighter Gyro and Accel process noise for less erratic behavior ---
    _navMekf.init(initPos, initVel, initQuat, 0.1f, 1e-4f, 1e-6f, 5e-3f, 1e-5f);
    DEBUG_PRINTLN_F("FlightController: NavMEKF initialized.");
  } else {
    _F_kf << 1.0f, Ts, 0.0f, 1.0f;
    _G_kf << 0.5 * Ts * Ts, Ts;
    _H_kf << 1, 0, 0, 1; // ZUKF velocity

    float var_proc_pos = KALMAN_VAR_PROC_POS;
    float var_proc_vel = KALMAN_VAR_PROC_VEL;
    _Q_kf << var_proc_pos * (Ts * Ts * Ts * Ts) / 4.0,
        var_proc_pos * (Ts * Ts * Ts) / 2, var_proc_pos * (Ts * Ts * Ts) / 2,
        var_proc_vel * Ts * Ts;

    float var_med_alt =
        KALMAN_VAR_MEAS_ALT; // Standard variance for altitude measurement
    float var_zupt_vel =
        KALMAN_VAR_ZUPT_VEL; // Very low variance for ZUPT velocity measurement

    _R_kf << var_med_alt, 0, 0, var_zupt_vel;

    _P0_kf << 1, 0, // Altitude uncertainty
        0, 1;       // Velocity uncertainty

    _X0_kf << 0,
        0; // AGL altitude and vertical speed initial estimates

    _kf.init(_F_kf, _G_kf, _H_kf, _Q_kf, _R_kf, _P0_kf, _X0_kf);
    DEBUG_PRINTLN_F(
        "FlightController: Kalman filter initialized internal estimation.");
  }
}

/**
 * @brief Runs the state estimation logic (IMU update + Kalman Filter).
 * Can handle HIL simulation data if HIL_MODE_ACTIVE is true.
 */
bool FlightController::runStateEstimator() {
  uint32_t start_us = micros();
  _diagnostics.flightState = (uint8_t)_flightState;
  uint32_t step_us;
  float filter_dt = Ts;
  bool motor_on = (_flightState == FlightState::MOTOR_ON);

  RawFlightData data;
  HILSimulationData hilData;


  if (HIL_MODE_ACTIVE) {
    // --- HIL Mode: Read data from CSV file ---
    step_us = micros();
    hilData = DataManager::getInstance().readHILStep();
    _diagnostics.sensorRead_us = micros() - step_us;

    if (!hilData.valid) {
      if (DataManager::getInstance().isLoggingActive()) {
        DataManager::getInstance().stopLogging();
        DEBUG_PRINTLN_F("End of HIL simulation. Logging stopped.");
      }
      _telemetryEnabled = false;
      return false; // Simulation finished
    }

    step_us = micros();
    _barometricPressure = hilData.barometricPressure_Pa;

    if (hilData.hasFullIMU) {      // Extract basic HIL variables
      // INJECT: 1:1 HIL Data (Assuming standard Z-longitudinal sim frame)
      float ax = hilData.accX_ms2 / G_GRAVITATIONAL_CONSTANT; 
      float ay = hilData.accY_ms2 / G_GRAVITATIONAL_CONSTANT; 
      float az = hilData.accZ_ms2 / G_GRAVITATIONAL_CONSTANT; 
      
      float gx = hilData.gyroX_rads;
      float gy = hilData.gyroY_rads;
      float gz = hilData.gyroZ_rads;

      float mx = USE_MAGNETOMETER ? hilData.magX_T : 0.0f; 
      float my = USE_MAGNETOMETER ? hilData.magY_T : 0.0f;
      float mz = USE_MAGNETOMETER ? hilData.magZ_T : 0.0f;

      // Assign sim-time to data row for correct log playback
      data.timestamp = (uint32_t)(hilData.time_s * 1000.0f);

      _attitudeEstimator->getIMU()->injectData(ax, ay, az, gx, gy, gz, mx, my, mz);
    } else {
      _netVerticalAcceleration = hilData.netVerticalAcceleration_ms2;
      _tilt = 90 - hilData.tilt;
    }
    _diagnostics.imuFilter_us = micros() - step_us;
    _diagnostics.navCalc_us = 0; // Grouped above

    step_us = micros(); // Reset for Kalman section
  } else {
    // --- Real Mode: Read data from sensors ---
    step_us = micros();

    // Calculate Dynamic DT based on actual loop timing
    uint32_t nowUs = micros();
    float dt_actual = Ts; // Initial default
    if (_lastEstimatorLoopUs > 0) {
      dt_actual = (nowUs - _lastEstimatorLoopUs) / 1000000.0f;
      if (dt_actual > 0.1f)
        dt_actual = Ts; // Guard against first run or massive stalls
    }
    _lastEstimatorLoopUs = nowUs;
    filter_dt = dt_actual;

    _attitudeEstimator->getIMU()->update(dt_actual);
    _barometricPressure = baro->getPressurePa();
    _diagnostics.sensorRead_us = micros() - step_us;

    // Pad-level baseline tracking: Update ground pressure if not in flight
    if (_flightState == FlightState::HEALTH_CHECK || _flightState == FlightState::WAIT_LAUNCH) {
        baro->recalibrateGroundPressure(_barometricPressure);
    }

    step_us = micros();


    if (_flightState == FlightState::MOTOR_ON) {
        _attitudeEstimator->setFilterBeta(0.1f); // Smooth during thrust
    }
  }

  // --- Filter and State Estimation Execution ---
  // --- Sensor Transformation and Preparation ---
  // We run the AttitudeEstimator update first to populate aligned sensor data,
  // but its internal filter (Madgwick/Mahony) will skip execution if currentFilter == NAV_MEKF.
  _attitudeEstimator->update(filter_dt, motor_on);
  
  AttitudeFilterSel currentFilter = _attitudeEstimator->getCurrentFilter();
  step_us = micros();
  float measuredAltitude_m = baro->altitudeFromPressure(_barometricPressure);

  if (currentFilter == AttitudeFilterSel::NAV_MEKF) {
      // Use "Aligned" sensor data from the AttitudeEstimator to ensure frame consistency
      Matrix<float, 3, 1> gyro(_attitudeEstimator->getTransformedGyroX() * DEG_TO_RAD, 
                               _attitudeEstimator->getTransformedGyroY() * DEG_TO_RAD, 
                               _attitudeEstimator->getTransformedGyroZ() * DEG_TO_RAD);
      
      Matrix<float, 3, 1> acc(_attitudeEstimator->getTransformedAccX() * G_GRAVITATIONAL_CONSTANT, 
                              _attitudeEstimator->getTransformedAccY() * G_GRAVITATIONAL_CONSTANT, 
                              _attitudeEstimator->getTransformedAccZ() * G_GRAVITATIONAL_CONSTANT);
      
      // 1. Prediction (IMU Integration)
      _navMekf.predict(gyro, acc, filter_dt);

      // 2. Accelerometer Update (Gravity Fusion / Leveling)
      // Only fuse gravity if we are not under high dynamic acceleration (e.g., stationary on pad)
      float acc_norm = acc.norm();
      if (abs(acc_norm - G_GRAVITATIONAL_CONSTANT) < 2.0f) { // Allowing 0.2G deviation for leveling
          Matrix<float, 3, 3> R_acc = Matrix<float, 3, 3>::Identity() * 1.0f;  
          _navMekf.updateAccel(acc / G_GRAVITATIONAL_CONSTANT, R_acc);
      }

      // 3. Barometer Update
      if (measuredAltitude_m > -9000.0f) {
           _navMekf.updateBaro(measuredAltitude_m, KALMAN_VAR_MEAS_ALT);
      }

      // 4. Magnetometer Update
      if (_attitudeEstimator->getUseMagnetometer()) {
          IMUSensor* imu = _attitudeEstimator->getIMU();
          // Align magnetometer to body frame (matches AK8963 rotation in AttitudeEstimator)
          float mx = imu->getMagY(); 
          float my = imu->getMagX(); 
          float mz = imu->getMagZ();
          
          Matrix<float, 3, 1> mag_meas(mx, my, mz);
          float ref_x, ref_y, ref_z;
          _attitudeEstimator->getMagneticReference(ref_x, ref_y, ref_z);
          Matrix<float, 3, 1> mag_ref(ref_x, ref_y, ref_z);
          
          Matrix<float, 3, 3> R_mag = Matrix<float, 3, 3>::Identity() * 5.0f; // Mag Covariance
          _navMekf.updateMag(mag_meas, mag_ref, R_mag);
      }

      // 5. Zero-Velocity Update (ZUPT) - The new ZUKF
      // Only runs when the rocket is known to be stationary (on the pad)
      if (_flightState == FlightState::HEALTH_CHECK || _flightState == FlightState::WAIT_LAUNCH) {
          Matrix<float, 3, 1> zero_vel(0.0f, 0.0f, 0.0f);
          Matrix<float, 3, 3> R_zupt = Matrix<float, 3, 3>::Identity() * 0.01f; // High confidence in zero velocity
          _navMekf.updateVelocity(zero_vel, R_zupt);
      }

      // 6. GPS Timing Simulation
      if (SIMULATE_GPS_TIMING) {
          static uint8_t gpsSimCounter = 0;
          if (++gpsSimCounter >= 10) { // 10Hz simulation (100Hz / 10)
              gpsSimCounter = 0;
              Matrix<float, 3, 1> mock_pos(0.0f, 0.0f, 0.0f);
              Matrix<float, 3, 1> mock_vel(0.0f, 0.0f, 0.0f);
              Matrix<float, 6, 6> R_gps = Matrix<float, 6, 6>::Identity() * 0.1f; 
              _navMekf.updateGPS(mock_pos, mock_vel, R_gps);
          }
      }

      // Sync state back to AttitudeEstimator for common math getters to work (tilt, netAccel)
      Matrix<float, 4, 1> q = _navMekf.getQuaternion();
      _attitudeEstimator->setQuaternion(q(0), q(1), q(2), q(3));
      
      _tilt = _attitudeEstimator->getTilt();
      _netVerticalAcceleration = _attitudeEstimator->getNetVerticalAcceleration();
      _filteredAltitude = _navMekf.getPosition()(2);
      _filteredVerticalVelocity = _navMekf.getVelocity()(2);
      
      _diagnostics.imuFilter_us = micros() - step_us; // MEKF is the filter now
  } else {
      // Legacy Mode: Filter is already updated in the prep step above
      _diagnostics.imuFilter_us = micros() - step_us;

      _tilt = _attitudeEstimator->getTilt();
      _netVerticalAcceleration = _attitudeEstimator->getNetVerticalAcceleration();

      // Legacy Mode: 1D Kalman Filter (Altitude/Velocity)
      step_us = micros();
      Eigen::Matrix<float, 1, 1> U_kf;
      U_kf << _netVerticalAcceleration;
      _kf.Predict(U_kf);

      if (measuredAltitude_m > -9000.0f) {
        Eigen::Matrix<float, 2, 1> Z_kf;
        Z_kf << measuredAltitude_m, 0.0f;
        _kf.Update(Z_kf, _R_kf);
      } else {
        DEBUG_PRINTLN_F("WARNING: Invalid altitude measurement. Kalman Update skipped.");
      }

      Matrix<float, 2, 1> stateEstimate = _kf.getPosterioriState();
      _filteredAltitude = stateEstimate(0, 0);
      _filteredVerticalVelocity = stateEstimate(1, 0);
  }
  
  _diagnostics.kalmanUpdate_us = micros() - step_us;

  // --- Update Loop Diagnostics ---
  _diagnostics.totalExecute_us = micros() - start_us;
  _diagnostics.totalCycles++;
  if (_diagnostics.totalExecute_us > 20000) {
    _diagnostics.cyclesExceeded++;
  }
  if (_diagnostics.totalExecute_us > _diagnostics.peakExecution_us) {
    _diagnostics.peakExecution_us = _diagnostics.totalExecute_us;
  }

  // Populate RawFlightData for logging/telemetry
  data.timestamp = HIL_MODE_ACTIVE ? (uint32_t)(hilData.time_s * 1000.0f) : millis();
  
  IMUSensor* imu = _attitudeEstimator->getIMU();
  data.accX = imu->getAccX();
  data.accY = imu->getAccY();
  data.accZ = imu->getAccZ();
  data.gyroX = imu->getGyroX_rads() * (180.0f / PI);
  data.gyroY = imu->getGyroY_rads() * (180.0f / PI);
  data.gyroZ = imu->getGyroZ_rads() * (180.0f / PI);
  data.magX = imu->getMagX();
  data.magY = imu->getMagY();
  data.magZ = imu->getMagZ();
  
  data.qW = _attitudeEstimator->getQuaternionW();
  data.qX = _attitudeEstimator->getQuaternionX();
  data.qY = _attitudeEstimator->getQuaternionY();
  data.qZ = _attitudeEstimator->getQuaternionZ();
  
  data.filteredAltitude = _filteredAltitude;
  data.filteredVerticalVelocity = _filteredVerticalVelocity;
  data.netVerticalAcceleration = _netVerticalAcceleration;
  data.tilt = _tilt;
  data.barometricPressure = _barometricPressure;
  data.airbrakeDeployment = (int16_t)(_airbrakeDeployment * 100.0f);
  data.pid_gain = _pid_gain;
  data.cd_gain = _cd_gain;
  data.flightState = (uint8_t)_flightState;

  return true; // Data valid
}


void FlightController::resetDiagnostics() {
  uint64_t total = _diagnostics.totalCycles;
  uint64_t exceeded = _diagnostics.cyclesExceeded;
  _diagnostics = {0};
}

/**
 * @brief Updates the provided data structure with the current flight state and
 * sensor readings.
 * @param data Reference to the RawFlightData structure to populate.
 */
void FlightController::updateLogger(RawFlightData &data) {
  data.timestamp = millis();
  data.accX = _attitudeEstimator->getIMU()->getAccX();
  data.accY = _attitudeEstimator->getIMU()->getAccY();
  data.accZ = _attitudeEstimator->getIMU()->getAccZ();
  data.gyroX = _attitudeEstimator->getIMU()->getGyroX_rads() * RAD_TO_DEG;
  data.gyroY = _attitudeEstimator->getIMU()->getGyroY_rads() * RAD_TO_DEG;
  data.gyroZ = _attitudeEstimator->getIMU()->getGyroZ_rads() * RAD_TO_DEG;
  data.magX = _attitudeEstimator->getIMU()->getMagX() / 10.0;
  data.magY = _attitudeEstimator->getIMU()->getMagY() / 10.0;
  data.magZ = _attitudeEstimator->getIMU()->getMagZ() / 10.0;
  data.qW = _attitudeEstimator->getQuaternionW();
  data.qX = _attitudeEstimator->getQuaternionX();
  data.qY = _attitudeEstimator->getQuaternionY();
  data.qZ = _attitudeEstimator->getQuaternionZ();
  data.filteredAltitude = _filteredAltitude;
  data.filteredVerticalVelocity = _filteredVerticalVelocity;
  data.netVerticalAcceleration = _netVerticalAcceleration;
  data.tilt = _tilt;
  data.barometricPressure = _barometricPressure;
  data.airbrakeDeployment = _airbrakeDeployment * 100;
  data.pid_gain = _pid_gain;
  data.cd_gain = _cd_gain;
  data.flightState = static_cast<uint8_t>(_flightState);
}

/**
 * @brief Main update loop. Runs the estimator and executes the logic for the
 * current flight state.
 */
void FlightController::update() {

  if (!runStateEstimator()) {
    return; // Stop processing if HIL simulation finished
  }

  // --- Recovery System Update ---
  static FlightState lastState = FlightState::SENSOR_CALIBRATION;
  static uint32_t lastRTCSave = 0;

  // Save on state change
  if (_flightState != lastState) {
    saveStateToRTC();

    // Selective Force Sync for critical data persistence (Apogee/Landing)
    if (_flightState == FlightState::DESCENT ||
        _flightState == FlightState::LANDING) {
      DataManager::getInstance().forceSync();
    }

    lastState = _flightState;
  }
  // Save periodically (every 100ms)
  else if (millis() - lastRTCSave > 100) {
    saveStateToRTC();
    lastRTCSave = millis();
  }
  // ------------------------------

  switch (_flightState) {
  case FlightState::SENSOR_CALIBRATION:
    calibrationCheckLoop();
    break;
  case FlightState::HEALTH_CHECK:
    healthCheckLoop();
    break;
  case FlightState::WAIT_LAUNCH:
    waitLaunchLoop();
    break;
  case FlightState::MOTOR_ON:
    motorOnLoop();
    break;
  case FlightState::BURNOUT:
    burnoutLoop();
    break;
  case FlightState::AIRBRAKE_DEPLOYMENT:
    airbrakeDeploymentLoop();
    break;
  case FlightState::APOGEE:
    apogeeLoop();
    break;
  case FlightState::DESCENT:
    descentLoop();
    break;
  case FlightState::LANDING:
    landingLoop();
    break;
  default:
    break;
  }
}

// --- Internal Loops ---

/**
 * @brief Loop for SENSOR_CALIBRATION state. Checks for calibration data.
 */
void FlightController::calibrationCheckLoop() {
  // Throttle: only print once per second (50 cycles at 20ms)
  static uint8_t calPrintCtr = 0;
  if (++calPrintCtr >= 50) {
    DEBUG_PRINTLN_F("STATE: Sensor Calibration");
    calPrintCtr = 0;
  }

  MPU9250_HAL *hal = static_cast<MPU9250_HAL *>(_attitudeEstimator->getIMU());
  if (hal->hasCalibrationData() && baro->getGroundPressureP0() != 101325.0f) {
    DEBUG_PRINTLN_F("FlightLogic: Sensors Ready (Calibrated).");
    _flightState = FlightState::HEALTH_CHECK;
    _stateEntryTime = millis();
  }
}

/**
 * @brief Loop for HEALTH_CHECK state. Verifies sensor health before launch.
 */
void FlightController::healthCheckLoop() {
  // Throttle: only print once per second (50 cycles at 20ms)
  static uint8_t hcPrintCtr = 0;
  if (++hcPrintCtr >= 50) {
    DEBUG_PRINTLN_F("State: Health Check");
    hcPrintCtr = 0;
  }

  if (checkFlightSystemHealth(_filteredAltitude, _filteredVerticalVelocity)) {
    _healthCheckCount++;
    DEBUG_PRINT_F("Component health OK this iteration. Count: ");
    DEBUG_PRINTLN(_healthCheckCount);

    if (_healthCheckCount >= _reqHealthChecks) {
      DEBUG_PRINTLN_F("HEALTH CHECK: Consecutive checks OK. Transitioning...");

      _flightState = FlightState::WAIT_LAUNCH;
      DataManager::getInstance().startLogging(); // Activate data logging

      // System settings for wait-for-launch
      _attitudeEstimator->setFilterBeta(10.0f); // High initial gain for fast alignment on pad
      DEBUG_PRINTLN_F("FlightLogic: Ready for Launch -> WAIT_LAUNCH.");
      _R_kf(1, 1) = 0.000001f; // Reduce velocity measurement variance for ZUKF
      DataManager::getInstance().setDecimationFactor(10);
      _stateEntryTime = millis();
      _healthCheckCount = 0;
      _launchCounter = 0; // Reset launch counter
    }
  } else {
    DEBUG_PRINTLN_F("ERROR: Component health failure detected by checkFlightSystemHealth()!");
    signalFailedModule("Health Check Iteration");
    _healthCheckCount = 0; // Reset count if any check fails
  }
}

/**
 * @brief Loop for WAIT_LAUNCH state. Monitors for launch acceleration.
 */
void FlightController::waitLaunchLoop() {
  uint32_t timeOnPad = millis() - _stateEntryTime;
  if (timeOnPad > 5000) {
    _attitudeEstimator->setFilterBeta(0.05f); // Rock solid stable on pad
  } else {
    _attitudeEstimator->setFilterBeta(10.0f); // Faster initial convergence (snaps to gravity)
  }
  if (detectLaunch(_netVerticalAcceleration, _filteredAltitude)) {

    // System settings for flight
    _attitudeEstimator->setDriftLearning(false); // Disable drift learning on IMU after launch
    _R_kf(1, 1) = 1000000000.0f; // Set high variance on velocity measurement to ignore zero input during flight
    DataManager::getInstance().setDecimationFactor(1); // Save every 20ms during flight

    _flightState = FlightState::MOTOR_ON;
    _stateEntryTime = millis();
    _launchDetectedTime = millis();
  }
}

/**
 * @brief Loop for MOTOR_ON state. Monitors ascent and checks for burnout.
 */
void FlightController::motorOnLoop() {
  unsigned long tempoDesdeLancamento = millis() - _launchDetectedTime;
  if (detectBurnout(_netVerticalAcceleration, tempoDesdeLancamento)) {
    _attitudeEstimator->setFilterBeta(0.1f);
    _flightState = FlightState::BURNOUT;
    _stateEntryTime = millis();
  }
}

/**
 * @brief Loop for BURNOUT state. Waits for conditions to deploy airbrakes.
 */
void FlightController::burnoutLoop() {
  // Check for airbrakes actuation window
  if (detectAirbrakesActuation(_filteredAltitude, _filteredVerticalVelocity)) {
    _flightState = FlightState::AIRBRAKE_DEPLOYMENT;
    _stateEntryTime = millis();
  }
}

/**
 * @brief Loop for AIRBRAKE_DEPLOYMENT state. Controls airbrakes and monitors
 * for apogee.
 */
void FlightController::airbrakeDeploymentLoop() {
  // Airbrake control logic
  _delta_V_ms = lookUpSpeed(_filteredAltitude) - _filteredVerticalVelocity;

  _pid_gain = _controller.computePID(0, _delta_V_ms);
  _cd_gain = _controller.computeCd(apoggeTargetAltitude_m, _filteredAltitude,
                                   _filteredVerticalVelocity,
                                   -G_GRAVITATIONAL_CONSTANT, RHO_AIR);
  _controlInput = _pid_gain + _cd_gain;

  if (_tilt < maxTiltAngle) {
    _airbrakeDeployment = getNearestActuation(
        (_filteredVerticalVelocity / MACH_VELOCITY), _controlInput);
  } else {
    _airbrakeDeployment = 0.0;
  }

  commandAirbrakes(_airbrakeDeployment);

  // Apogee Detection
  if (detectApogeeByRegression(_filteredAltitude, millis())) {
    _flightState = FlightState::APOGEE;
    _attitudeEstimator->setFilterBeta(1.0f);
    _stateEntryTime = millis();
    _apogeeDetectedTime = millis();
  }
}

/**
 * @brief Loop for APOGEE state. Retracts airbrakes and transitions to descent.
 */
void FlightController::apogeeLoop() {
  retractAirbrakes();
  DEBUG_PRINTLN_F("Airbrakes retracted at apogee.");

  _flightState = FlightState::DESCENT;
  DataManager::getInstance().setDecimationFactor(
      10); // Save every 10x20ms = 200ms
  _stateEntryTime = millis();
}

/**
 * @brief Loop for DESCENT state. Monitors for landing conditions.
 */
void FlightController::descentLoop() {
  unsigned long tempoDesdeApogeu = millis() - _apogeeDetectedTime;
  if (detectLanding(_filteredVerticalVelocity, _filteredAltitude,
                    tempoDesdeApogeu)) {
    DEBUG_PRINTLN_F("LANDING DETECTED!");
    _flightState = FlightState::LANDING;
    DataManager::getInstance().setDecimationFactor(
        50); // Save every 50x20ms = 1s
    _stateEntryTime = millis();
  }
  // Timeout for LANDING detection if not detected after long time
  else if (tempoDesdeApogeu > _maxWaitTimeLanding) {
    DEBUG_PRINTLN_F("Timeout for LANDING detection.");
    _flightState = FlightState::LANDING; // Force LANDING state
    DataManager::getInstance().setDecimationFactor(
        50); // Save every 50x20ms = 1s
    _stateEntryTime = millis();
  }
}

/**
 * @brief Loop for LANDING state. Handles post-flight logging and cleanup.
 */
void FlightController::landingLoop() {
  DEBUG_PRINTLN_F("STATE: LANDING");
  // Continue saving final data for a short period (120000ms - 2min)
  DataManager &logger = DataManager::getInstance();

  // Stop logging after 2 minutes to save space/time

  if (logger.isLoggingActive() && (millis() - _stateEntryTime) >= 120000) {
    logger.stopLogging(); // Stop logging after 2 mins
    DEBUG_PRINTLN_F("Operation finished. Awaiting recovery.");
  }

  // Non-blocking slow-down: use timestamp instead of delay(10000)
  static uint32_t _landingLastPrint = 0;
  if (millis() - _landingLastPrint < 10000)
    return;
  _landingLastPrint = millis();
  DEBUG_PRINTLN_F("STATE: LANDING - idle");
}

// --- Specific Methods ---

/**
 * @brief Forces the flight controller into a specific state. Useful for
 * debugging or HIL.
 * @param newState The target FlightState.
 */
void FlightController::forceState(FlightState newState) {
  _flightState = newState;
  _stateEntryTime = millis();
  DEBUG_PRINT_F("FlightState Forced to: ");
  DEBUG_PRINTLN((int)_flightState);
  saveStateToRTC(); // Update backup

  if (_flightState >= FlightState::WAIT_LAUNCH) {
    DataManager::getInstance().startLogging();
    DataManager::getInstance().setDecimationFactor(
        1); // Default to 1 for high-res HIL

    if (_flightState == FlightState::WAIT_LAUNCH) {
      DataManager::getInstance().setDecimationFactor(
          10); // Throttle back slightly
      if (_attitudeEstimator) {
        _attitudeEstimator->setDriftLearning(false); // [FIX] Disabled Integral Windup: Trust the static Gyro Calibration instead.
        _attitudeEstimator->resetEstimatorState();   // [DIAGNOSTIC] Clear any biases from startup
      }
      _attitudeEstimator->setFilterBeta(10.0f);
      _R_kf(1, 1) = 0.000001f;
      _healthCheckCount = 0;
      _launchCounter = 0; // Reset launch counter
    }
  }
}

/**
 * @brief Sets up the servo for airbrakes.
 * @return true if servo setup is successful, false otherwise.
 */
bool FlightController::setupServo() {
  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  delay(1500); // Massive delay to let LEDC HW settle after tones

  DEBUG_PRINTLN_F("SETUP_SERVO: Initializing ESP32Servo...");

  _airbrakeServo.setPeriodHertz(50);
  if (_airbrakeServo.attach(_pinServoAirbrake, _servoMinPulse, _servoMaxPulse) == 0) {
    DEBUG_PRINTLN_F("ERROR: ESP32Servo attach failed!");
    return false;
  }

  DEBUG_PRINTLN_F("SETUP_SERVO: Servo configuration complete.");
  commandAirbrakes(0); // Set to zero

  return true;
}

/**
 * @brief Commands the airbrakes to a desired position.
 * @param desiredPosition 0.0 to 1.0
 */
void FlightController::commandAirbrakes(float desiredPosition) {
  // Convert desired position to pulse width (500us to 2500us nominally for
  // 0-180) Our config: _servoMinPulse = 560, _servoMaxPulse = 1520 (for 0-90)

  float pulse_us =
      _servoMinPulse + (desiredPosition * (_servoMaxPulse - _servoMinPulse));

  _airbrakeServo.writeMicroseconds(pulse_us);
}

/**
 * @brief Commands the airbrakes to fully retracted position.
 */
void FlightController::retractAirbrakes() { commandAirbrakes(0); }

/**
 * @brief Checks the health status of the flight logic system.
 * @param filteredAltitude Filtered altitude in meters.
 * @param filteredVerticalVelocity Filtered vertical velocity in m/s.
 * @return true if the system is healthy, false otherwise.
 */
bool FlightController::checkFlightSystemHealth(float filteredAltitude,
                                               float filteredVerticalVelocity) {
  bool healthOk = true;

  // IMU Health Check
  float ax = _attitudeEstimator->getIMU()->getAccX();
  float ay = _attitudeEstimator->getIMU()->getAccY();
  float az = _attitudeEstimator->getIMU()->getAccZ();
  float accel_mag = sqrt(ax * ax + ay * ay + az * az);

  if (accel_mag < (1.0f - HEALTH_ACCEL_MAG_TOLERANCE) ||
      accel_mag >
          (1.0f +
           HEALTH_ACCEL_MAG_TOLERANCE)) { // Tolerance defined in Config_voo
    DEBUG_PRINTLN_F("FlightLogic: Health Failure - Accel Magnitude.");
    healthOk = false;
  }

  if (abs(_attitudeEstimator->getIMU()->getGyroX_rads()) >
          (HEALTH_GYRO_TOLERANCE_DPS * DEG_TO_RAD) ||
      abs(_attitudeEstimator->getIMU()->getGyroY_rads()) >
          (HEALTH_GYRO_TOLERANCE_DPS * DEG_TO_RAD) ||
      abs(_attitudeEstimator->getIMU()->getGyroZ_rads()) >
          (HEALTH_GYRO_TOLERANCE_DPS *
           DEG_TO_RAD)) { // Tolerance defined in Config_voo
    DEBUG_PRINTLN_F("FlightLogic: Health Failure - High Gyro.");
    healthOk = false;
  }

  // Kalman Filter Health Check
  if (abs(filteredAltitude) >
      HEALTH_ALT_TOLERANCE_M) { // Tolerance defined in Config_voo
    DEBUG_PRINT_F("FlightLogic: Health Failure - Altitude Kalman: ");
    DEBUG_PRINTLN(filteredAltitude);
    healthOk = false;
  }
  if (abs(filteredVerticalVelocity) >
      HEALTH_VEL_TOLERANCE_MS) { // Tolerance defined in Config_voo
    DEBUG_PRINT_F("FlightLogic: Health Failure - Velocity Kalman: ");
    DEBUG_PRINTLN(filteredVerticalVelocity);
    healthOk = false;
  }
  // ! Add back when a servo is physically connected and attach() succeeds
  // // Servo Health Check
  if (!_airbrakeServo.attached()) {
    DEBUG_PRINTLN_F("FlightLogic: Health Failure - Servo connection.");
    healthOk = false;
  }

  if (_testServo) {
    if (millis() - _servoTestLastTime >= 500) {
      _servoTestLastTime = millis();
      _servoTestStep++;

      switch (_servoTestStep) {
      case 1:
        // Step 1: Go to 0
        _airbrakeServo.write(0);
        DEBUG_PRINTLN_F("SERVO TEST: 0 deg");
        break;
      case 2:
        // Step 2: Go to 45
        _airbrakeServo.write(45);
        DEBUG_PRINTLN_F("SERVO TEST: 45 deg");
        break;
      case 3:
        // Step 3: Go to 90
        _airbrakeServo.write(90);
        DEBUG_PRINTLN_F("SERVO TEST: 90 deg");
        break;
      case 4:
        // Step 4: Go to 45
        _airbrakeServo.write(45);
        DEBUG_PRINTLN_F("SERVO TEST: 45 deg");
        break;
      case 5:
        // Step 5: Go to 0 and Finish
        _airbrakeServo.write(0);
        DEBUG_PRINTLN_F("SERVO TEST: 0 deg (Done)");
        _testServo = false;
        _servoTestStep = 0;
        return healthOk; // Return actual health status now that test is done
        break;
      }
    }
    // While testing, return false to keep system in HEALTH_CHECK state
    return false;
  }

  return healthOk;
}

/**
 * @brief Detects launch based on vertical acceleration and altitude.
 * @param verticalAcceleration Vertical acceleration in m/s^2.
 * @param filteredAltitude Filtered altitude in meters.
 * @return true if launch is detected, false otherwise.
 */
bool FlightController::detectLaunch(float verticalAcceleration,
                                    float filteredAltitude) {
  // REQUIREMENT: Positive vertical acceleration (upward) or altitude threshold
  // Only trigger on positive vertical acceleration to prevent downward shocks from falsely triggering lift-off.
  if (verticalAcceleration > _accelLimitLaunch ||filteredAltitude > _heightLimitLaunch) {
    _launchCounter++;
  } else {
    _launchCounter = 0;
  }

  // Confirm launch with multiple consecutive readings 
  if (_launchCounter >= _launchConfirmationCount) {
    DEBUG_PRINTLN_F("FLIGHT_LOGIC: Launch confirmed!");
    return true;
  }
  return false;
}

/**
 * @brief Detects motor burnout based on vertical acceleration and time since
 * launch.
 * @param verticalAcceleration Vertical acceleration in m/s^2.
 * @param timeSinceLaunch Time since launch in milliseconds.
 * @return true if burnout is detected, false otherwise.
 */
bool FlightController::detectBurnout(float verticalAcceleration,
                                     unsigned long timeSinceLaunch) {
  // Update moving average buffer
  _burnoutMovingSum -= _burnoutBufferAcc[_burnoutIndexHead];
  _burnoutBufferAcc[_burnoutIndexHead] = verticalAcceleration;
  _burnoutMovingSum += _burnoutBufferAcc[_burnoutIndexHead];

  // Move head index
  _burnoutIndexHead = (_burnoutIndexHead + 1) % _burnoutWindowSize;

  // Check if buffer is full
  if (!_burnoutFullBuffer && _burnoutIndexHead == 0)
    _burnoutFullBuffer = true;

  // Calculate moving average
  float movingAverageAcc;
  if (_burnoutFullBuffer) {
    movingAverageAcc = _burnoutMovingSum / _burnoutWindowSize;
  } else {
    int amostrasAtuais = (_burnoutIndexHead == 0) ? 1 : _burnoutIndexHead;
    movingAverageAcc = _burnoutMovingSum / (float)amostrasAtuais;
  }

  // Check burnout condition
  if (timeSinceLaunch > _minMotorBurnTime &&
      abs(movingAverageAcc) < _accelLimitBurnout) {
    _burnoutCounter++;
  } else {
    _burnoutCounter = 0;
  }

  if (_burnoutCounter >= _burnoutConfirmationCount) {
    DEBUG_PRINTLN_F("FLIGHT_LOGIC: Burnout detected.");
    return true;
  }
  return false;
}

/**
 * @brief Detects airbrakes actuation based on altitude and vertical velocity.
 * @param filteredAltitude Filtered altitude in meters.
 * @param filteredVerticalVelocity Filtered vertical velocity in m/s.
 * @return true if airbrakes actuation is detected, false otherwise.
 */
bool FlightController::detectAirbrakesActuation(
    float filteredAltitude, float filteredVerticalVelocity) {
  if (filteredAltitude > _minActuationHeight &&
      abs(filteredVerticalVelocity) < _velLimitActuation) {
    DEBUG_PRINTLN_F("FLIGHT_LOGIC: Conditions for airbrake actuation met.");
    return true;
  }
  return false;
}

/**
 * @brief Detects apogee based on vertical velocity and altitude.
 * @param filteredVerticalVelocity Filtered vertical velocity in m/s.
 * @param filteredAltitude Filtered altitude in meters.
 * @return true if apogee is detected, false otherwise.
 */
bool FlightController::detectApogee(float filteredVerticalVelocity,
                                    float filteredAltitude) {
  // Update max recorded height
  if (filteredAltitude > _maxRecordedHeight) {
    _maxRecordedHeight = filteredAltitude;
  }
  // Increment counter if condition is met
  if (filteredVerticalVelocity < _velLimitApogee) {
    _consecutiveApogeeCount++;
    DEBUG_PRINT_F("Apogee condition met. Count: ");
    DEBUG_PRINTLN(_consecutiveApogeeCount);

  } else {
    _consecutiveApogeeCount = 0;
  }

  // Check if required consecutive readings are met
  if (_consecutiveApogeeCount >= _readingsForApogeeConfirmation) {
    DEBUG_PRINTLN_F("FLIGHT_LOGIC: APOGEE DETECTED AND CONFIRMED!");
    DEBUG_PRINT_F("Maximum Recorded Altitude: ");
    DEBUG_PRINT(_maxRecordedHeight, 2);
    DEBUG_PRINTLN_F("m");

    _consecutiveApogeeCount = 0;
    return true;
  }
  return false;
}

/**
 * @brief Detects apogee by regression analysis based on altitude and time.
 * @param filteredAltitude Filtered altitude in meters.
 * @param currentTime_ms Current time in milliseconds.
 * @return true if apogee is detected, false otherwise.
 */
bool FlightController::detectApogeeByRegression(float filteredAltitude,
                                                unsigned long currentTime_ms) {
  // If already detected, lock state and return true
  if (_regressionApogeeConfirmed)
    return true;

  // 1. Add new sample to Circular Buffer
  // Convert ms to seconds to avoid overflow in power calculations (t^4)
  _regressionTimeBuffer[_regressionHeadIndex] = currentTime_ms / 1000.0f;
  _regressionAltBuffer[_regressionHeadIndex] = filteredAltitude;

  _regressionHeadIndex = (_regressionHeadIndex + 1) % _regressionWindowSize;

  // Only start calculating when the buffer is full
  if (_regressionSampleCount < _regressionWindowSize) {
    _regressionSampleCount++;
    return false;
  }

  // 2. Time Normalization (Crucial for float precision)
  // Use time relative to the oldest sample in the window (t0)
  int oldestIndex = _regressionHeadIndex; // In a full buffer, head points to
                                          // the oldest element
  float t0 = _regressionTimeBuffer[oldestIndex];

  // 3. Accumulate Sums for Least Squares (Use double for precision)
  double sum_t = 0, sum_t2 = 0, sum_t3 = 0, sum_t4 = 0;
  double sum_y = 0, sum_ty = 0, sum_t2y = 0;

  for (int i = 0; i < _regressionWindowSize; i++) {
    int idx = (oldestIndex + i) % _regressionWindowSize;

    double t = (double)(_regressionTimeBuffer[idx] - t0); // Relative time
    double y = (double)_regressionAltBuffer[idx];
    double t2 = t * t;

    sum_t += t;
    sum_t2 += t2;
    sum_t3 += t2 * t;
    sum_t4 += t2 * t2;

    sum_y += y;
    sum_ty += t * y;
    sum_t2y = sum_t2y + (t2 * y);
  }

  // 4. Setup and Solve the Linear System
  // Matrix A * x = B
  Matrix3d A;
  A << (double)_regressionWindowSize, sum_t, sum_t2, sum_t, sum_t2, sum_t3,
      sum_t2, sum_t3, sum_t4;

  Vector3d B;
  B << sum_y, sum_ty, sum_t2y;

  // Solve for x = [c, b, a] (where y = at^2 + bt + c)
  // LDLT decomposition is faster than full inversion
  Vector3d x = A.ldlt().solve(B);

  float a = (float)x[2]; // Quadratic coefficient (Concavity / Acceleration)
  float b = (float)x[1]; // Linear coefficient (Initial velocity)
  // float c = x[0]; // Initial altitude (not used for decision)

  // 5. Coefficient Analysis

  bool isConcave = (a < APOGEE_REGRESSION_CONCAVITY_THRESHOLD);

  // B. Estimated Velocity at the CURRENT instant (End of Window):
  // Regression smooths out the velocity. v(t) = 2*a*t + b
  float t_final =
      _regressionTimeBuffer[(_regressionHeadIndex - 1 + _regressionWindowSize) %
                            _regressionWindowSize] -
      t0;
  float estimatedVelocity = 2.0f * a * t_final + b;

  bool isDescending =
      (estimatedVelocity < APOGEE_REGRESSION_DESCENDING_THRESHOLD);

  if (isConcave && isDescending) {
    _regressionApogeeConfirmed = true;
    // DEBUG_PRINTLN_F("APOGEE (Pure Regression) Confirmed!");
    return true;
  }

  return false;
}

/**
 * @brief Detects landing based on vertical velocity, altitude, and time since
 * apogee.
 * @param filteredVerticalVelocity Filtered vertical velocity in m/s.
 * @param filteredAltitude Filtered altitude in meters.
 * @param timeSinceApogee Time since apogee in milliseconds.
 * @return true if landing is detected, false otherwise.
 */
bool FlightController::detectLanding(float filteredVerticalVelocity,
                                     float filteredAltitude,
                                     unsigned long timeSinceApogee) {
  if (timeSinceApogee < _minTimeAfterApogeeLanding) {
    return false;
  }

  if (abs(filteredVerticalVelocity) < _velLimitLanding &&
      filteredAltitude < _altLimitLanding) {
    if (_firstGroundDetectionTime == 0) {
      _firstGroundDetectionTime = millis();
    }
    if (millis() - _firstGroundDetectionTime >= _minTimeLanding) {
      DEBUG_PRINTLN_F("FLIGHT_LOGIC: Landing detected!");
      return true;
    }
  } else {
    _firstGroundDetectionTime =
        0; // Reset counter if leaving landing conditions
  }
  return false;
}

/**
 * @brief Reads the current tilt angle.
 * @return Current tilt angle in degrees.
 */
float FlightController::readCurrentTilt() {
  return _attitudeEstimator->getTilt();
}

// --- Recovery System Implementation ---
void FlightController::saveStateToRTC() {
  _rtcBackup.magicNumber = RTC_MAGIC;
  _rtcBackup.state = _flightState;
  _rtcBackup.basePressure = baro->getGroundPressureP0();
  _rtcBackup.baseTemperature = baro->getGroundTemperatureT0();
  _rtcBackup.maxAltitude = _maxRecordedHeight;
  _rtcBackup.stateEntryTime = _stateEntryTime;
  _rtcBackup.timestamp = millis();
}

void FlightController::resetRecoveryData() {
  _rtcBackup.magicNumber = 0; // Invalidate
  _rtcBackup.state = FlightState::WAIT_LAUNCH;
}

bool FlightController::attemptRecovery() {
  DEBUG_PRINTLN_F("RECOVERY: Checking for valid RTC backup...");

  if (_rtcBackup.magicNumber == RTC_MAGIC) {
    DEBUG_PRINTLN_F("RECOVERY: Valid backup found!");

    // Restore Critical References FIRST
    // Only restore P0/T0 if not in calibration/health check states
    if (_flightState != FlightState::SENSOR_CALIBRATION &&
        _flightState != FlightState::HEALTH_CHECK) {
      baro->setGroundPressureP0(_rtcBackup.basePressure);
      baro->setGroundTemperatureT0(_rtcBackup.baseTemperature);
      DEBUG_PRINT_F("RECOVERY: Restored P0: ");
      DEBUG_PRINT(_rtcBackup.basePressure);
      DEBUG_PRINT_F(" Pa | T0: ");
      DEBUG_PRINT(_rtcBackup.baseTemperature);
      DEBUG_PRINTLN_F(" C");
    } else {
      DEBUG_PRINTLN_F("RECOVERY: Skipping P0/T0 restore due to current state.");
    }

    // Restore State
    _flightState = _rtcBackup.state;
    _maxRecordedHeight = _rtcBackup.maxAltitude;

    // Adjust entry time relative to new millis()
    // Original duration = stored_timestamp - stored_entryTime
    uint32_t durationInState = _rtcBackup.timestamp - _rtcBackup.stateEntryTime;
    if (durationInState > millis()) {
      // If we were in state longer than current uptime, wrap around logic
      _stateEntryTime = millis() - durationInState;
    } else {
      _stateEntryTime = millis() - durationInState;
    }

    DEBUG_PRINTLN_F("RECOVERY: State restored successfully.");
    return true;
  }
  DEBUG_PRINTLN_F("RECOVERY: No valid backup or Magic mismatch.");
  return false;
}

/**
 * @brief Prints the full telemetry payload to standard serial for plotting
 */
void FlightController::printFullTelemetry(const RawFlightData &data) {
  if (!_telemetryEnabled)
    return;

#if USE_TELEPLOT == 0
  // Standard Monitor
  switch ((FlightState)data.flightState) {
  case FlightState::SENSOR_CALIBRATION:
    PRINT_STATE("CALIBRATION");
    break;
  case FlightState::HEALTH_CHECK:
    PRINT_STATE("HEALTH_CHECK");
    break;
  case FlightState::WAIT_LAUNCH:
    PRINT_STATE("WAIT_LAUNCH");
    PLOT_VAR("Alt", data.filteredAltitude);
    PLOT_VAR("VelZ", data.filteredVerticalVelocity);
    PLOT_VAR("AccelZ", data.netVerticalAcceleration);
    PLOT_VAR("Tilt", data.tilt);
    break;
  case FlightState::MOTOR_ON:
    PRINT_STATE("MOTOR_ON");
    PLOT_VAR("Alt", data.filteredAltitude);
    PLOT_VAR("VelZ", data.filteredVerticalVelocity);
    PLOT_VAR("Tilt", data.tilt);
    break;
  case FlightState::BURNOUT:
    PRINT_STATE("BURNOUT");
    PLOT_VAR("Alt", data.filteredAltitude);
    PLOT_VAR("VelZ", data.filteredVerticalVelocity);
    PLOT_VAR("Tilt", data.tilt);
    break;
  case FlightState::AIRBRAKE_DEPLOYMENT:
    PRINT_STATE("AIRBRAKE_DEPLOYMENT");
    PLOT_VAR("Alt", data.filteredAltitude);
    PLOT_VAR("VelZ", data.filteredVerticalVelocity);
    PLOT_VAR("Tilt", data.tilt);
    PLOT_VAR("Deflection", data.airbrakeDeployment);
    break;
  case FlightState::APOGEE:
    PRINT_STATE("APOGEE");
    PLOT_VAR("Max_Alt", _maxRecordedHeight);
    break;
  case FlightState::DESCENT:
    PRINT_STATE("DESCENT");
    PLOT_VAR("Alt", data.filteredAltitude);
    PLOT_VAR("VelZ", data.filteredVerticalVelocity);
    break;
  case FlightState::LANDING:
    PRINT_STATE("LANDING");
    break;
  default:
    PRINT_STATE("UNKNOWN");
    break;
  }
  DEBUG_PRINTLN(); // Newline for the summary row
#else
  // Teleplot: Full Data Stream for High-Res Graphing
  PLOT_VAR("AccX", data.accX);
  PLOT_VAR("AccY", data.accY);
  PLOT_VAR("AccZ", data.accZ);
  PLOT_VAR("GyroX", data.gyroX);
  PLOT_VAR("GyroY", data.gyroY);
  PLOT_VAR("GyroZ", data.gyroZ);
  PLOT_VAR("MagX", data.magX);
  PLOT_VAR("MagY", data.magY);
  PLOT_VAR("MagZ", data.magZ);
  PLOT_VAR("qW", data.qW);
  PLOT_VAR("qX", data.qX);
  PLOT_VAR("qY", data.qY);
  PLOT_VAR("qZ", data.qZ);
  PLOT_VAR("Alt", data.filteredAltitude);
  PLOT_VAR("VelZ", data.filteredVerticalVelocity);
  PLOT_VAR("AccelZ", data.netVerticalAcceleration);
  PLOT_VAR("Alt", data.filteredAltitude);
  PLOT_VAR("VelZ", data.filteredVerticalVelocity);
  PLOT_VAR("AccelZ", data.netVerticalAcceleration);
  PLOT_VAR("Max_Alt",_maxRecordedHeight); 
  PLOT_VAR("Tilt", data.tilt);
  PLOT_VAR("TiltLimit", maxTiltAngle);
  PLOT_VAR("Max", maxTiltAngle);
  PLOT_VAR("Press", data.barometricPressure);      // Pa
  PLOT_VAR("Deflection", data.airbrakeDeployment); // %
  PLOT_VAR("PID_Gain", data.pid_gain);
  PLOT_VAR("Cd_Gain", data.cd_gain);
  PLOT_VAR("FlightState", data.flightState);
#endif
}
