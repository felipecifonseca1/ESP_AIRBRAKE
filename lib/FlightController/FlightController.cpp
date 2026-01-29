#include "FlightController.h"
#include "AltitudeSpeedTable.hh"
#include "DataManager.h"
#include "FlightController.h"
#include "Funcoes_BMP.h"
#include "DataManager.h" 
#include "DragCoefficientTable.hh"
#include <Arduino.h>
#include "Funcoes_suporte_IMU.h"
#include "Sinalizacao.h"
#include <ArduinoEigenDense.h>
#include <math.h>

using namespace Eigen;

/**
 * @brief Gets the singleton instance of the FlightController.
 * @return Reference to the FlightController instance.
 */
FlightController &FlightController::getInstance() {
  static FlightController instance;
  return instance;
}

/**
 * @brief Constructor. Initializes the PID controller and resets buffers.
 */
FlightController::FlightController(): _controller(_Kp, _Ki, _Kd, _mass_kg, _area_m2, _Ts){ // Initialize PID Controller

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
  DEBUG_PRINTLN_F("Controller initialized.");
}

/**
 * @brief Initializes the Kalman Filter matrices and parameters.
 */
void FlightController::setupKalman() {
  _F_kf << 1.0f, Ts,
           0.0f, 1.0f;
  _G_kf << 0.5 * Ts * Ts, Ts;
  _H_kf << 1, 0,
           0, 1; // ZUKF velocity 

  float var_proc_pos = KALMAN_VAR_PROC_POS;
  float var_proc_vel = KALMAN_VAR_PROC_VEL;
  _Q_kf << var_proc_pos * (Ts * Ts * Ts * Ts) / 4.0, var_proc_pos * (Ts * Ts * Ts) / 2, 
           var_proc_pos * (Ts * Ts * Ts) / 2,        var_proc_vel * Ts * Ts;

  float var_med_alt = KALMAN_VAR_MEAS_ALT; // Standard variance for altitude measurement
  float var_zupt_vel = KALMAN_VAR_ZUPT_VEL; // Very low variance for ZUPT velocity measurement

  _R_kf << var_med_alt, 0, 0, var_zupt_vel;

  _P0_kf << 1, 0, // Altitude uncertainty
            0, 1; // Velocity uncertainty

  _X0_kf << 0, 
            0; // AGL altitude and vertical speed initial estimates

  _kf.init(_F_kf, _G_kf, _H_kf, _Q_kf, _R_kf, _P0_kf, _X0_kf);
  DEBUG_PRINTLN_F("FlightController: Kalman filter initialized internal estimation.");
}

/**
 * @brief Runs the state estimation logic (IMU update + Kalman Filter).
 * Can handle HIL simulation data if HIL_MODE_ACTIVE is true.
 */
void FlightController::runStateEstimator() {
  bool motor_on = (_flightState == FlightState::MOTOR_ON);
  
  // Local variable for HIL data
  HILSimulationData hilData;

  if (HIL_MODE_ACTIVE) {
    // --- HIL Mode: Read data from CSV file ---
    hilData = DataManager::getInstance().readHILStep();

    if (!hilData.valid) {
      if (DataManager::getInstance().isLoggingActive()) {
        DataManager::getInstance().stopLogging();
        Serial.println("End of HIL simulation. Logging stopped.");
      }
      return;
    }

    _barometricPressure = hilData.barometricPressure_Pa;

    if (hilData.hasFullIMU) {
      float accX_g = hilData.accX_ms2 / G_GRAVITATIONAL_CONSTANT;
      float accY_g = hilData.accY_ms2 / G_GRAVITATIONAL_CONSTANT;
      float accZ_g = hilData.accZ_ms2 / G_GRAVITATIONAL_CONSTANT;

      float gyroX_degs = hilData.gyroX_rads * RAD_TO_DEG;
      float gyroY_degs = hilData.gyroY_rads * RAD_TO_DEG;
      float gyroZ_degs = hilData.gyroZ_rads * RAD_TO_DEG;

      float magX_mG = hilData.magX_T * 1e7;
      float magY_mG = hilData.magY_T * 1e7;
      float magZ_mG = hilData.magZ_T * 1e7;

      if (PHYSICAL_Z_AXIS_DOWN) {
        if (!mpu.update(Ts, motor_on, false, accX_g, accY_g, -accZ_g,
                        -gyroX_degs, -gyroY_degs, gyroZ_degs, magX_mG, -magY_mG,
                        -magZ_mG)) {
          DEBUG_PRINTLN_F("Failed to update simulated IMU data!");
          return;
        }
        _netVerticalAcceleration = computeNetAcceleration(false, accX_g, accY_g, -accZ_g, false);
        _tilt = readCurrentTilt();
      } else {
        if (!mpu.update(Ts, motor_on, false, -accX_g, accY_g, accZ_g,
                        gyroX_degs, -gyroY_degs, -gyroZ_degs, -magX_mG,
                        -magY_mG, magZ_mG)) {
          DEBUG_PRINTLN_F("Failed to update simulated IMU data!");
          return;
        }
        _netVerticalAcceleration = computeNetAcceleration(false, -accX_g, accY_g, accZ_g, false);
        _tilt = readCurrentTilt();
      }
    } else {
      // Simple HIL mode
      _netVerticalAcceleration = hilData.netVerticalAcceleration_ms2;
      _tilt = 90 - hilData.tilt; // Convert tilt to match system definition
    }
  } else {
    // --- Real Mode: Read data from sensors ---
    if (!mpu.update(Ts, motor_on)) {
      DEBUG_PRINTLN_F("Failed to read IMU data!");
      return;
    }

    _netVerticalAcceleration = computeNetAcceleration(false);
    _barometricPressure = getPressaoBMPAtual();
    _tilt = readCurrentTilt();
  }

  // --- Kalman Filter Execution ---
  Eigen::Matrix<float, 1, 1> U_kf;
  U_kf << _netVerticalAcceleration;
  _kf.Predict(U_kf);

  float measuredAltitude_m = altitudeFromPressure(_barometricPressure);

  if (measuredAltitude_m > -9000.0f) {
    Eigen::Matrix<float, 2, 1> Z_kf;
    Z_kf << measuredAltitude_m, 0.0f; // Zero velocity measurement for ZUKF
    _kf.Update(Z_kf, _R_kf);
  } else {
    DEBUG_PRINTLN_F("WARNING: Invalid altitude measurement. Kalman Update skipped.");
  }

  // Get the posteriori state estimate
  Matrix<float, 2, 1> stateEstimate = _kf.getPosterioriState();
  _filteredAltitude = stateEstimate(0, 0);
  _filteredVerticalVelocity = stateEstimate(1, 0);
}

/**
 * @brief Updates the provided data structure with the current flight state and sensor readings.
 * @param data Reference to the RawFlightData structure to populate.
 */
void FlightController::updateLogger(RawFlightData &data) {
  data.timestamp = millis(); 
  data.accX = mpu.getAccX();
  data.accY = mpu.getAccY();
  data.accZ = mpu.getAccZ();
  data.gyroX = mpu.getGyroX();
  data.gyroY = mpu.getGyroY();
  data.gyroZ = mpu.getGyroZ();
  data.magX = mpu.getMagX() / 10.0;
  data.magY = mpu.getMagY() / 10.0;
  data.magZ = mpu.getMagZ() / 10.0;
  data.qW = mpu.getQuaternionW();
  data.qX = mpu.getQuaternionX();
  data.qY = mpu.getQuaternionY();
  data.qZ = mpu.getQuaternionZ();
  data.filteredAltitude = _filteredAltitude;
  data.filteredVerticalVelocity = _filteredVerticalVelocity;
  data.netVerticalAcceleration = _netVerticalAcceleration;
  data.tilt = _tilt;
  data.barometricPressure = _barometricPressure; 
  data.airbrakeDeployment = _airbrakeDeployment * 100;
  data.gain1 = _controlGain1;
  data.gain2 = _controlGain2;
  data.flightState = static_cast<uint8_t>(_flightState);
}

/**
 * @brief Main update loop. Runs the estimator and executes the logic for the current flight state.
 */
void FlightController::update() {
  
  runStateEstimator(); // Get full state estimation from Kalman Filter and Madgwick Filter

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
  DEBUG_PRINTLN_F("STATE: Sensor Calibration");

  if (hasCalibrationDataIMU() && getGroundPressureP0_BMP() != 101324.0f) {
    DEBUG_PRINTLN_F("Initial calibration (IMU/BMP P0) done in setup.");
    _flightState = FlightState::HEALTH_CHECK;
    _stateEntryTime = millis();
  }
}

/**
 * @brief Loop for HEALTH_CHECK state. Verifies sensor health before launch.
 */
void FlightController::healthCheckLoop() {
  DEBUG_PRINTLN_F("State: Health Check");

  recalibrateGroundPressure();

  if (checkFlightSystemHealth(_filteredAltitude, _filteredVerticalVelocity)) {
    _healthCheckCount++;
    DEBUG_PRINT_F("Component health OK this iteration. Count: ");
    DEBUG_PRINTLN(_healthCheckCount);

    if (_healthCheckCount >=
        _reqHealthChecks) { 
      Serial.println("HEALTH CHECK: Consecutive checks OK. Transitioning...");
      signalSuccessfullModule("Health Check Complete");

      _flightState = FlightState::WAIT_LAUNCH;
      DataManager::getInstance().startLogging(); // Activate data logging

      // System settings for wait-for-launch
      setDriftLearning(true); // Enable drift learning on the IMU
      setFilterBeta(2.5f); // Increase filter beta for faster response during wait
      _R_kf(1, 1) = 0.000001f; // Reduce velocity measurement variance for ZUKF
      
      _stateEntryTime = millis();
      _healthCheckCount = 0;
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
  _loopPrintCounter++;

  recalibrateGroundPressure();

  if (_loopPrintCounter > _printCountLimit) {
    DEBUG_PRINT_F("STATE: WAIT_LAUNCH | Alt: ");
    DEBUG_PRINT(_filteredAltitude);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(_filteredVerticalVelocity);
    DEBUG_PRINT_F("m/s | AccelZ: ");
    DEBUG_PRINT(_netVerticalAcceleration);
    DEBUG_PRINT_F("m/s^2 | Tilt: ");
    DEBUG_PRINT(_tilt);
    DEBUG_PRINTLN_F("deg");
    _loopPrintCounter = 0;
  }

  if (detectLaunch(_netVerticalAcceleration, _filteredAltitude)) {
    DEBUG_PRINTLN_F("LAUNCH DETECTED!");

    // System settings for flight
    setDriftLearning(false); // Disable drift learning on IMU after launch
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
  _loopPrintCounter++;
  if (_loopPrintCounter > _printCountLimit) {
    DEBUG_PRINT_F("STATE: MOTOR_ON | Alt: ");
    DEBUG_PRINT(_filteredAltitude);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(_filteredVerticalVelocity);
    DEBUG_PRINT_F("m/s | Tilt: ");
    DEBUG_PRINT(_tilt);
    DEBUG_PRINTLN_F("deg");
    _loopPrintCounter = 0;
  }

  unsigned long tempoDesdeLancamento = millis() - _launchDetectedTime;
  if (detectBurnout(_netVerticalAcceleration, tempoDesdeLancamento)) {
    DEBUG_PRINTLN_F("BURNOUT DETECTED!");
    setFilterBeta(0.75f);
    _flightState = FlightState::BURNOUT;
    _stateEntryTime = millis();
  }
}

/**
 * @brief Loop for BURNOUT state. Waits for conditions to deploy airbrakes.
 */
void FlightController::burnoutLoop() {
  DEBUG_PRINT_F("STATE: Burnout | Alt: ");
  DEBUG_PRINT(_filteredAltitude);
  DEBUG_PRINT_F("m | VelZ: ");
  DEBUG_PRINT(_filteredVerticalVelocity);
  DEBUG_PRINTLN_F("m/s");

  if (detectAirbrakesActuation(_filteredAltitude, _filteredVerticalVelocity)) {
    DEBUG_PRINTLN_F("Appropriate speed detected, initiating airbrake actuation.");
    _flightState = FlightState::AIRBRAKE_DEPLOYMENT;
    _stateEntryTime = millis();
  }
}

/**
 * @brief Loop for AIRBRAKE_DEPLOYMENT state. Controls airbrakes and monitors for apogee.
 */
void FlightController::airbrakeDeploymentLoop() {
  _loopPrintCounter++;
  if (_loopPrintCounter > _printCountLimit) {
    DEBUG_PRINT_F("STATE: AIRBRAKE_DEPLOYMENT | Alt: ");
    DEBUG_PRINT(_filteredAltitude);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(_filteredVerticalVelocity);
    DEBUG_PRINT_F("m/s | Tilt: ");
    DEBUG_PRINT(_tilt);
    DEBUG_PRINT_F("deg | Computed Deflection: ");
    DEBUG_PRINTLN(_airbrakeDeployment);
    _loopPrintCounter = 0;
  }

  // Airbrake control logic
  _delta_V_ms = lookUpSpeed(_filteredAltitude) - _filteredVerticalVelocity;

  _controlGain1 = _controller.computePID(0, _delta_V_ms);
  _controlGain2 = _controller.computeCd(apoggeTargetAltitude_m, _filteredAltitude, _filteredVerticalVelocity, -G_GRAVITATIONAL_CONSTANT, RHO_AIR);
  _controlInput = _controlGain1 + _controlGain2;

  if (_tilt < maxTiltAngle) {
    _airbrakeDeployment = getNearestActuation((_filteredVerticalVelocity / MACH_VELOCITY), _controlInput);
  } else {
    _airbrakeDeployment = 0.0;
  }

  commandAirbrakes(_airbrakeDeployment);

  // Apogee Detection
  if (detectApogeeByRegression(_filteredAltitude, millis())) {
    DEBUG_PRINTLN_F("APOGEE DETECTED!");
    _flightState = FlightState::APOGEE;
    setFilterBeta(1.0f);
    _stateEntryTime = millis();
    _apogeeDetectedTime = millis();
  }
}

/**
 * @brief Loop for APOGEE state. Retracts airbrakes and transitions to descent.
 */
void FlightController::apogeeLoop() {
  DEBUG_PRINT_F("STATE: APOGEE | Max Alt: ");
  DEBUG_PRINT(_filteredAltitude);
  DEBUG_PRINTLN_F("m");

  retractAirbrakes();
  DEBUG_PRINTLN_F("Airbrakes retracted at apogee.");

  _flightState = FlightState::DESCENT;
  DataManager::getInstance().setDecimationFactor(10); // Save every 10x20ms = 200ms
  _stateEntryTime = millis();
}

/**
 * @brief Loop for DESCENT state. Monitors for landing conditions.
 */
void FlightController::descentLoop() {
  _loopPrintCounter++;
  if (_loopPrintCounter > _printCountLimit) {
    DEBUG_PRINT_F("STATE: DESCENT | Alt: ");
    DEBUG_PRINT(_filteredAltitude);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(_filteredVerticalVelocity);
    DEBUG_PRINTLN_F("m/s");
    _loopPrintCounter = 0;
  }

  unsigned long tempoDesdeApogeu = millis() - _apogeeDetectedTime;
  if (detectLanding(_filteredVerticalVelocity, _filteredAltitude, tempoDesdeApogeu)) {
    DEBUG_PRINTLN_F("LANDING DETECTED!");
    _flightState = FlightState::LANDING;
    DataManager::getInstance().setDecimationFactor(50); // Save every 50x20ms = 1s
    _stateEntryTime = millis();
  }
  // Timeout for LANDING detection if not detected after long time
  else if (tempoDesdeApogeu > _maxWaitTimeLanding) {
    DEBUG_PRINTLN_F("Timeout for LANDING detection.");
    _flightState = FlightState::LANDING; // Force LANDING state
    DataManager::getInstance().setDecimationFactor(50); // Save every 50x20ms = 1s
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

  delay(10000); // Slow down loop in landing
}

// --- Specific Methods ---

/**
 * @brief Forces the flight controller into a specific state. Useful for debugging or HIL.
 * @param newState The target FlightState.
 */
void FlightController::forceState(FlightState newState) {
    _flightState = newState;
    _stateEntryTime = millis();
    DEBUG_PRINT_F("FlightState Forced to: ");
    DEBUG_PRINTLN((int)_flightState);

    if (_flightState == FlightState::WAIT_LAUNCH) {
        DataManager::getInstance().startLogging(); 
        DataManager::getInstance().setDecimationFactor(10);
        setDriftLearning(true); 
        setFilterBeta(2.5f);
        _R_kf(1, 1) = 0.000001f; 
        _healthCheckCount = 0;
    }
}

/**
 * @brief Sets up the servo for airbrakes.
 * @return true if servo setup is successful, false otherwise.
 */
bool FlightController::setupServo() {
  DEBUG_PRINTLN_F("SETUP_SERVO: Initializing servo");
  DEBUG_PRINT_F("SETUP_SERVO: Configuring Servo on pin ");
  DEBUG_PRINTLN(_pinServoAirbrake);

  _airbrakeServo.attach(_pinServoAirbrake, _servoMinPulse, _servoMaxPulse);
  _airbrakeServo.write(0);

  if (_airbrakeServo.attached()) {
    DEBUG_PRINTLN_F("SETUP_SERVO: Servo attached successfully.");
    return true;
  } else {
    DEBUG_PRINT_F("ERROR: Failed to attach servo on pin ");
    DEBUG_PRINTLN(_pinServoAirbrake);
    return false;
  }
}

/**
 * @brief Commands the airbrakes to a desired position.
 * @param desiredPosition 0.0 to 1.0
 */
void FlightController::commandAirbrakes(float desiredPosition) {
  // Convert desired position to servo angle - 0 = 0° | 1.0 = 90(45°)° (full opening)
  int anguloServo = map(desiredPosition * 100, 0, 100, 0, 90);
  _airbrakeServo.write(anguloServo);
  // DEBUG_PRINT_F("FLIGHT_LOGIC: Commanding airbrakes to position (0-1): ");
  // DEBUG_PRINTLN(desiredPosition);
}

/**
 * @brief Commands the airbrakes to fully retracted position.
 */
void FlightController::retractAirbrakes() {
  _airbrakeServo.write(0);
  // DEBUG_PRINTLN_F("FLIGHT_LOGIC: Airbrakes fully retracted.");
}

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
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float accel_mag = sqrt(ax * ax + ay * ay + az * az);

  if (accel_mag < 0.85f || accel_mag > 1.15f) { // Tolerance of +/- 0.15g
    DEBUG_PRINTLN_F("FlightLogic: Health Failure - Accel Magnitude.");
    healthOk = false;
  }

  if (abs(mpu.getGyroX()) > 3.0f || abs(mpu.getGyroY()) > 3.0f ||
      abs(mpu.getGyroZ()) > 3.0f) { // Tolerance of 3 dps
    DEBUG_PRINTLN_F("FlightLogic: Health Failure - High Gyro.");
    healthOk = false;
  }

  // Kalman Filter Health Check
  if (abs(filteredAltitude) > 4.0f) { // Tolerance of +/- 4m on ground
    DEBUG_PRINT_F("FlightLogic: Health Failure - Altitude Kalman: ");
    DEBUG_PRINTLN(filteredAltitude);
    healthOk = false;
  }
  if (abs(filteredVerticalVelocity) > 1.0f) { // Tolerance of +/- 1m/s
    DEBUG_PRINT_F("FlightLogic: Health Failure - Velocity Kalman: ");
    DEBUG_PRINTLN(filteredVerticalVelocity);
    healthOk = false;
  }

  // Servo Health Check
  if (!_airbrakeServo.attached()) {
    DEBUG_PRINTLN_F("FlightLogic: Health Failure - Servo connection.");
    healthOk = false;
  }

  if (_testServo) {
    _airbrakeServo.write(0);
    delay(500);
    _airbrakeServo.write(45);
    delay(500);
    _airbrakeServo.write(90);
    delay(500);
    _airbrakeServo.write(45);
    delay(500);
    _airbrakeServo.write(0);
    _testServo = false;
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
  if (verticalAcceleration > _accelLimitLaunch ||
      filteredAltitude > _heightLimitLaunch) {
    DEBUG_PRINTLN_F("FLIGHT_LOGIC: Launch detected!");
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
      movingAverageAcc < _accelLimitBurnout) {
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
  if (filteredAltitude > _minActuationHeight && filteredVerticalVelocity < _velLimitActuation) {
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

  // 3. Accumulate Sums for Least Squares
  float sum_t = 0, sum_t2 = 0, sum_t3 = 0, sum_t4 = 0;
  float sum_y = 0, sum_ty = 0, sum_t2y = 0;

  for (int i = 0; i < _regressionWindowSize; i++) {
    int idx = (oldestIndex + i) % _regressionWindowSize;

    float t = _regressionTimeBuffer[idx] - t0; // Relative time
    float y = _regressionAltBuffer[idx];
    float t2 = (float)t * t;

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
  Matrix3f A;
  A << _regressionWindowSize, sum_t, sum_t2, sum_t, sum_t2, sum_t3, sum_t2,
      sum_t3, sum_t4;

  Vector3f B;
  B << sum_y, sum_ty, sum_t2y;

  // Solve for x = [c, b, a] (where y = at^2 + bt + c)
  // LDLT decomposition is faster than full inversion
  Vector3f x = A.ldlt().solve(B);

  float a = x[2]; // Quadratic coefficient (Concavity / Acceleration)
  float b = x[1]; // Linear coefficient (Initial velocity)
  // float c = x[0]; // Initial altitude (not used for decision)

  // 5. Coefficient Analysis

  // A. Curve Concavity (2*a):
  // Must be significantly negative to indicate a ballistic trajectory.
  bool isConcave = (a < -0.05f);

  // B. Estimated Velocity at the CURRENT instant (End of Window):
  // Regression smooths out the velocity. v(t) = 2*a*t + b
  float t_final =
      _regressionTimeBuffer[(_regressionHeadIndex - 1 + _regressionWindowSize) %
                            _regressionWindowSize] -
      t0;
  float estimatedVelocity = 2.0f * a * t_final + b;

  // Must be descending (< 0) with a small hysteresis margin
  bool isDescending = (estimatedVelocity < -0.5f);

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
  return calcTilt(); // From Funcoes_suporte_IMU.h
}
