#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "Config_voo.h"
#include "Controller.hh" // Include PID Controller
#include "KalmanFilter.hh" // Include Kalman Filter
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <ESP32Servo.h>
#include <cstdint>



// Forward Declaration
struct RawFlightData;

// Enum for Flight States
enum class FlightState {
  SENSOR_CALIBRATION,
  HEALTH_CHECK,
  WAIT_LAUNCH,
  MOTOR_ON,
  BURNOUT,
  AIRBRAKE_DEPLOYMENT,
  APOGEE,
  DESCENT,
  LANDING
};

class FlightController {
public:
  // Singleton access
  static FlightController &getInstance();

  // --- Main Update Loop ---
  /**
   * @brief Main update method called every control cycle.
   * @param alt Current filtered altitude (m)
   * @param vel Current filtered vertical velocity (m/s)
   * @param acc Current net vertical acceleration (m/s^2)
   * @param tilt Current tilt angle (degrees)
   * @param pressure Current barometric pressure (Pa)
   **/
  void update();

  /**
   * @brief Fills the provided data structure with current flight data.
   */
  void updateLogger(RawFlightData &data);

  // --- Kalman & Estimator ---
  void setupKalman();
  void runStateEstimator();

  // --- State Detection Functions (Public as requested) ---
  bool detectLaunch(float verticalAcceleration, float filteredAltitude);
  bool detectBurnout(float verticalAcceleration, unsigned long timeSinceLaunch);
  bool detectAirbrakesActuation(float filteredAltitude,
                                float filteredVerticalVelocity);
  bool detectApogee(float filteredVerticalVelocity, float filteredAltitude);
  bool detectApogeeByRegression(float filteredAltitude,
                                unsigned long currentTime_ms);
  bool detectLanding(float filteredVerticalVelocity, float filteredAltitude,
                     unsigned long timeSinceApogee);
  bool checkFlightSystemHealth(float filteredAltitude,
                               float filteredVerticalVelocity);

  // --- Actuation & Sensor Functions ---
  float readCurrentTilt();
  bool setupServo();
  void commandAirbrakes(float desiredPosition);
  void retractAirbrakes();

  // --- Getters for Main/Logging ---
  FlightState getFlightState() const { return _flightState; }
  float getAirbrakeDeployment() const { return _airbrakeDeployment; }
  float getControlGain1() const { return _controlGain1; }

  float getControlGain2() const { return _controlGain2; }

  // --- Debug / HIL ---
  void forceState(FlightState newState);

  // --- Setup ---
  void setupController(); // Setup PID limits etc.

private:
  // Private constructor for Singleton
  FlightController();

  // Delete copy constructor and assignment operator
  FlightController(const FlightController &) = delete;
  void operator=(const FlightController &) = delete;

  // --- Internal Loops ---
  void calibrationCheckLoop();
  void healthCheckLoop();
  void waitLaunchLoop();
  void motorOnLoop();
  void burnoutLoop();
  void airbrakeDeploymentLoop();
  void apogeeLoop();
  void descentLoop();
  void landingLoop();

  // --- Internal State & Constants ---

  // Current State
  FlightState _flightState;

  // Data Cache 
  float _filteredAltitude;
  float _filteredVerticalVelocity;
  float _netVerticalAcceleration;
  float _tilt;
  float _barometricPressure;

  // Timers
  uint32_t _stateEntryTime;
  uint32_t _launchDetectedTime;
  uint32_t _apogeeDetectedTime;

  // PID Controller
  Controller _controller;
  float _controlGain1 = 0.0f;
  float _controlGain2 = 0.0f;
  float _controlInput = 0.0f;
  float _airbrakeDeployment = 0.0f;
  float _delta_V_ms = 0.0f;

  // Controller Constants 
  const float _Kp = 0.025f;
  const float _Ki = 0.075f;
  const float _Kd = 0.02f;
  const float _mass_kg = 30.605f;
  const float _area_m2 = 0.02097f;
  const float _Ts_ms = 20.0f;
  const float _Ts = 0.020f; // 20ms / 1000

  // Servo
  Servo _airbrakeServo;
  const uint8_t _pinServoAirbrake = 27;
  const uint16_t _servoMinPulse = 560;  // 0 deg
  const uint16_t _servoMaxPulse = 1520; // 90 deg
  bool _testServo = false;

  // Launch Detection Constants
  const float _accelLimitLaunch = 1.5f * 9.80665f;
  const int8_t _heightLimitLaunch = 4;

  // Burnout Detection
  static const int _burnoutWindowSize = 20;
  const uint16_t _minMotorBurnTime = 5000;
  const float _accelLimitBurnout = -0.5f;
  const int _burnoutConfirmationCount = 5;

  // Burnout Buffer State
  float _burnoutBufferAcc[_burnoutWindowSize];
  int _burnoutIndexHead = 0;
  float _burnoutMovingSum = 0.0f;
  bool _burnoutFullBuffer = false;
  int _burnoutCounter = 0;

  // Airbrake Actuation Constants
  const uint16_t _minActuationHeight = 500;
  const float _velLimitActuation = 0.7f * 335.0f; // 0.7 Mach

  // Apogee Detection
  const float _velLimitApogee = 0.5f;
  const uint8_t _readingsForApogeeConfirmation = 10;

  // Apogee State
  float _maxRecordedHeight = 0.0f;
  uint8_t _consecutiveApogeeCount = 0;

  // Apogee Regression
  static const int _regressionWindowSize = 30;
  float _regressionTimeBuffer[_regressionWindowSize];
  float _regressionAltBuffer[_regressionWindowSize];
  int _regressionHeadIndex = 0;
  int _regressionSampleCount = 0;
  bool _regressionApogeeConfirmed = false;

  // Landing Detection
  const float _velLimitLanding = 0.5f;
  const uint8_t _altLimitLanding = 10;
  const uint32_t _minTimeAfterApogeeLanding = 90000;
  const uint16_t _minTimeLanding = 5000;
  const uint32_t _maxWaitTimeLanding = 600000;

  // Landing State
  uint32_t _firstGroundDetectionTime = 0;

  // Health Check
  uint8_t _healthCheckCount = 0;
  const uint8_t _reqHealthChecks = 5;

  // Loop Print Counter
  uint8_t _loopPrintCounter = 0;
  const uint8_t _printCountLimit = 5;

  // --- Kalman Filter & Estimator State ---
  KalmanFilter _kf;
  Eigen::Matrix<float, 2, 2> _F_kf;
  Eigen::Matrix<float, 2, 1> _G_kf;
  Eigen::Matrix<float, 2, 2> _H_kf;
  Eigen::Matrix<float, 2, 2> _Q_kf;
  Eigen::Matrix<float, 2, 2> _R_kf;
  Eigen::Matrix<float, 2, 2> _P0_kf;
  Eigen::Matrix<float, 2, 1> _X0_kf;

};

#endif // FLIGHT_CONTROLLER_H
