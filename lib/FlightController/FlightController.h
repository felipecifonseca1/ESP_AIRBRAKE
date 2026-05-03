#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "Config_voo.h"
#include "Controller.hh" // Include PID Controller
#include "KalmanFilter.hh" // Include Kalman Filter
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <ESP32Servo.h>
#include "BarometricSensor.h"
#include "AttitudeEstimator.h"
#include "NavMEKF.h"

// --- Data Structures ---
struct RawFlightData;

struct LoopDiagnostics {
    uint32_t sensorRead_us;
    uint32_t imuFilter_us;
    uint32_t navCalc_us;
    uint32_t kalmanUpdate_us;
    uint32_t totalExecute_us;
    uint32_t loopInterval_us;
    uint32_t queueSend_us;
    uint8_t flightState;
    
    // Statistics
    uint64_t totalCycles;
    uint64_t cyclesExceeded;
    uint32_t peakExecution_us;
};

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

// Recovery Data Structure (RTC)
struct FlightRecoveryData {
    uint32_t magicNumber;       // 0xDEADBEEF
    FlightState state;          
    float basePressure;         // P0
    float baseTemperature;      // T0
    float maxAltitude;          
    uint32_t stateEntryTime;    
    uint32_t timestamp;         // Last save time
};

class FlightController {
public:
  // Singleton access
  static FlightController &getInstance(BarometricSensor* b = nullptr, AttitudeEstimator* estimator = nullptr);

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
  bool runStateEstimator();

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
  float getPIDGain() const { return _pid_gain; }
  float getCdGain() const { return _cd_gain; }
  const LoopDiagnostics& getDiagnostics() const { return _diagnostics; }
  LoopDiagnostics& getDiagnosticsMutable() { return _diagnostics; }
  AttitudeEstimator* getAttitudeEstimator() { return _attitudeEstimator; }
  void resetDiagnostics();

  // --- Telemetry ---
  void printFullTelemetry(const RawFlightData& data);
  void setTelemetryEnabled(bool enabled) { _telemetryEnabled = enabled; }
  bool isTelemetryEnabled() const { return _telemetryEnabled; }

  // --- Debug / HIL ---
  void forceState(FlightState newState);
  void resetAttitudeEstimator() { if (_attitudeEstimator) _attitudeEstimator->resetEstimatorState(); }

  // --- Recovery System ---
  void saveStateToRTC();
  bool attemptRecovery();
  void resetRecoveryData();

  // --- Setup ---
  void setupController(); // Setup PID limits etc.

private:
  // Private constructor for Singleton
  FlightController(BarometricSensor* b, AttitudeEstimator* estimator);
  BarometricSensor* baro;
  AttitudeEstimator* _attitudeEstimator;

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
  uint32_t _stateEntryTime = 0;
  uint32_t _lastEstimatorLoopUs = 0;
  uint32_t _launchDetectedTime = 0;
  uint32_t _apogeeDetectedTime;

  // PID Controller
  Controller _controller;
  float _pid_gain = 0.0f;
  float _cd_gain = 0.0f;
  float _controlInput = 0.0f;

  // Diagnostics
  LoopDiagnostics _diagnostics = {0};

  // State Detection Functions
  float _airbrakeDeployment = 0.0f;
  float _delta_V_ms = 0.0f;
  bool _tiltLockout = false; // Latches true once tilt >= maxTiltAngle; stays retracted to avoid chatter

  // Controller Constants 
  const float _Kp = PID_KP;
  const float _Ki = PID_KI;
  const float _Kd = PID_KD;
  const float _mass_kg = ROCKET_MASS_KG;
  const float _area_m2 = AIRBRAKE_REF_AREA_M2;
  const float _Ts_ms = Ts_ms;
  const float _Ts = Ts;

  // Servo
  Servo _airbrakeServo;
  const uint8_t _pinServoAirbrake = PIN_SERVO;
  const uint16_t _servoMinPulse = SERVO_MIN_PULSE;  // 0 deg
  const uint16_t _servoMaxPulse = SERVO_MAX_PULSE; // 90 deg
  bool _testServo = TESTE_SERVO;
  uint8_t _servoTestStep = 0;
  uint32_t _servoTestLastTime = 0;

  // Launch Detection Constants
  const float _accelLimitLaunch = LAUNCH_ACCEL_THRESHOLD_G * G_GRAVITATIONAL_CONSTANT;
  const float _heightLimitLaunch = LAUNCH_HEIGHT_THRESHOLD_M;
  const uint8_t _launchConfirmationCount = LAUNCH_CONFIRMATION_COUNT;
  int _launchCounter = 0;

  // Burnout Detection
  static const int _burnoutWindowSize = BURNOUT_WINDOW_SIZE;
  const uint16_t _minMotorBurnTime = BURNOUT_MIN_MOTOR_TIME_MS;
  const float _accelLimitBurnout = BURNOUT_ACCEL_THRESHOLD_G * G_GRAVITATIONAL_CONSTANT;
  const int _burnoutConfirmationCount = BURNOUT_CONFIRMATION_COUNT;

  // Burnout Buffer State
  float _burnoutBufferAcc[_burnoutWindowSize];
  int _burnoutIndexHead = 0;
  float _burnoutMovingSum = 0.0f;
  bool _burnoutFullBuffer = false;
  int _burnoutCounter = 0;

  // Airbrake Actuation Constants
  const float _minActuationHeight = ACTUATION_MIN_HEIGHT_M;
  const float _velLimitActuation = ACTUATION_VEL_LIMIT_MACH * MACH_VELOCITY;

  // Apogee Detection
  const float _velLimitApogee = APOGEE_VEL_THRESHOLD_MS;
  const uint8_t _readingsForApogeeConfirmation = APOGEE_READINGS_CONFIRMATION;

  // Apogee State
  float _maxRecordedHeight = 0.0f;
  uint8_t _consecutiveApogeeCount = 0;

  // Apogee Regression
  static const int _regressionWindowSize = APOGEE_REGRESSION_WINDOW;
  float _regressionTimeBuffer[_regressionWindowSize];
  float _regressionAltBuffer[_regressionWindowSize];
  int _regressionHeadIndex = 0;
  int _regressionSampleCount = 0;
  bool _regressionApogeeConfirmed = false;

  // Landing Detection
  const float _velLimitLanding = LANDING_VEL_THRESHOLD_MS;
  const float _altLimitLanding = LANDING_ALT_THRESHOLD_M;
  const uint32_t _minTimeAfterApogeeLanding = LANDING_MIN_TIME_AFTER_APOGEE_MS;
  const uint16_t _minTimeLanding = LANDING_STABLE_TIME_MS;
  const uint32_t _maxWaitTimeLanding = LANDING_MAX_WAIT_TIME_MS; 

  // Landing State
  uint32_t _firstGroundDetectionTime = 0;

  // Health Check
  uint8_t _healthCheckCount = 0;
  const uint8_t _reqHealthChecks = HEALTH_CHECK_REQUIRED_COUNT;

  // Loop Print Counter
  uint8_t _loopPrintCounter = 0;
  const uint8_t _printCountLimit = TELEMETRY_LOGGING_DECIMATION;
  bool _telemetryEnabled = ENABLE_TELEMETRY;

  // --- Kalman Filter & Estimator State ---
  KalmanFilter _kf;
  NavMEKF _navMekf;
  Eigen::Matrix<float, 2, 2> _F_kf;
  Eigen::Matrix<float, 2, 1> _G_kf;
  Eigen::Matrix<float, 2, 2> _H_kf;
  Eigen::Matrix<float, 2, 2> _Q_kf;
  Eigen::Matrix<float, 2, 2> _R_kf;
  Eigen::Matrix<float, 2, 2> _P0_kf;
  Eigen::Matrix<float, 2, 1> _X0_kf;

};

#endif // FLIGHT_CONTROLLER_H
