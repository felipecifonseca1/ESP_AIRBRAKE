#include "Logica_voo.h"
#include "Config_voo.h"
#include <ESP32Servo.h> // Use ESP32Servo library on ESP32
#include <math.h>
#include <Arduino.h>
#include "Funcoes_suporte_IMU.h"

#define MACH_VELOCITY 335

// Servo configurations
Servo airbrakeServo; // Servo object
const uint8_t PIN_SERVO_AIRBRAKE = 27; 
// Config 90 degrees max ---> actual angle = angle/2
// *Need to adjust min and max values according to the servo used*
const uint16_t min_pulse = 560;  // 0°  - write(0)
const uint16_t max_pulse = 1520; // 90° - write(180)
bool test_servo = false;   // Servo sweep

// Parameters for launch detection
const float ACCEL_LIMIT_LAUNCH = 1.5f * G_GRAVITATIONAL_CONSTANT; // m/s^2 
const int8_t HEIGHT_LIMIT_LAUNCH = 4; // m

// Parameters for burnout detection
const uint16_t MIN_MOTOR_BURN_TIME = 5000; // ms 
const float ACCEL_LIMIT_BURNOUT = -0.5; // m/s^2 

// Parameters for airbrake actuation
const uint16_t MIN_ACTUATION_HEIGHT = 500; // m
const float VEL_LIMIT_ACTUACTION = 0.7 * MACH_VELOCITY;    // m/s

// Parameters for apogee detection
const float VEL_LIMIT_APOGEE = 1.0; // m/s 
static float maxRecordedHeight = 0; // To help confirm apogee
static uint8_t consecutiveApogeeCount = 0;
const uint8_t READINGS_FOR_APOGEE_CONFIRMATION = 25;

// Parameters for landing detection
const float VEL_LIMIT_LANDING = 0.5;  // m/s
const uint8_t ALT_LIMIT_LANDING = 10; // m
const uint32_t MIN_TIME_AFTER_APOGEE_LANDING = 90000; // ms (1.5 min after apogee to consider landing)
static uint32_t firstGroundDetectionTime = 0;
const uint16_t MIN_TIME_LANDING = 5000; // ms (5s of stable landing conditions)

/**
 * @brief Initializes the servo for airbrake actuation.
 * @return true if the servo was successfully initialized, false otherwise.
 */
bool setupServo() { 
    DEBUG_PRINTLN_F("SETUP_SERVO: Initializing servo");
    DEBUG_PRINT_F("SETUP_SERVO: Configuring Servo on pin ");
    DEBUG_PRINTLN(PIN_SERVO_AIRBRAKE);
    airbrakeServo.attach(PIN_SERVO_AIRBRAKE, min_pulse, max_pulse);
    airbrakeServo.write(0);

    if (airbrakeServo.attached()) {
        DEBUG_PRINTLN_F("SETUP_SERVO: Servo attached successfully.");
        return true;
    } else {
        DEBUG_PRINT_F("ERROR: Failed to attach servo on pin ");
        DEBUG_PRINTLN(PIN_SERVO_AIRBRAKE);
        return false; 
    }
}

/**
 * @brief Commands the airbrakes to a desired position.
 * @details The position is given as a normalized value between 0.0 (fully retracted) and 1.0 (fully extended). 
 *          The function converts this value to the corresponding servo angle and sends the command to the servo.
 * @param desiredPosition A float value between 0.0 and 1.0 representing the desired airbrake position.
 */
void commandAirbrakes(float desiredPosition) {

    // Convert desired position to servo angle - 0 = 0° | 1.0 = 90(45°)° (full opening)
    int anguloServo = map(desiredPosition * 100, 0, 100, 0, 90); 
    airbrakeServo.write(anguloServo);
    // DEBUG_PRINT_F("FLIGHT_LOGIC: Commanding airbrakes to position (0-1): "); 
    // DEBUG_PRINTLN(desiredPosition);
}

/**
 * @brief Commands the airbrakes to fully retracted position.
 */
void retractAirbrakes() {
    airbrakeServo.write(0); 
    // DEBUG_PRINTLN_F("FLIGHT_LOGIC: Airbrakes fully retracted.");
}

/**
 * @brief Checks the health status of the flight logic system.
 * @details This function performs various health checks on the IMU, Kalman filter estimates, and servo connection.
 *          It checks for reasonable acceleration magnitudes, gyro rates, and filtered altitude/velocity values.
 * @param filteredAltitude Current filtered altitude from Kalman filter (m).
 * @param filteredVerticalVelocity Current filtered vertical velocity from Kalman filter (m/s)
 * @return true if the flight logic system is healthy, false otherwise.
 */
bool checkFlightSystemHealth(float filteredAltitude, float filteredVerticalVelocity) {

    bool healthOk = true; 

    // IMU Health Check
    float ax = mpu.getAccX(); float ay = mpu.getAccY(); float az = mpu.getAccZ();
    float accel_mag = sqrt(ax*ax + ay*ay + az*az);
    if (accel_mag < 0.85f || accel_mag > 1.15f) { // Tolerance of +/- 0.15g
        DEBUG_PRINTLN_F("FlightLogic: Health Failure - Accel Magnitude.");
        healthOk = false;
    }
    if (abs(mpu.getGyroX()) > 3.0f || abs(mpu.getGyroY()) > 3.0f || abs(mpu.getGyroZ()) > 3.0f) { // Tolerance of 3 dps
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
    if (!airbrakeServo.attached()){
        DEBUG_PRINTLN_F("FlightLogic: Health Failure - Servo connection.");
        healthOk = false;
    }

    if (test_servo){
        airbrakeServo.write(0);
        delay(500);
        airbrakeServo.write(45);
        delay(500);
        airbrakeServo.write(90);
        delay(500);
        airbrakeServo.write(45);
        delay(500);
        airbrakeServo.write(0);
        test_servo = false;
    }

    return healthOk;
}

/**
 * @brief Detects launch based on vertical acceleration and altitude.
 * @details Launch is detected if the vertical acceleration exceeds a defined threshold
 *         or if the altitude exceeds a certain limit.
 * @param verticalAcceleration Current vertical acceleration (m/s^2).
 * @param filteredAltitude Current filtered altitude from Kalman filter (m).
 * @return true if the launch is detected, false otherwise.
 */
bool detectLaunch(float verticalAcceleration, float filteredAltitude) {
    if (verticalAcceleration > ACCEL_LIMIT_LAUNCH || filteredAltitude > HEIGHT_LIMIT_LAUNCH) {
        DEBUG_PRINTLN_F("FLIGHT_LOGIC: Launch detected!");
        return true;
    }
    return false;
}

/**
 * @brief Detects motor burnout based on vertical acceleration and time since launch.
 * @details Burnout is detected if the vertical acceleration drops below a defined threshold
 *         and a minimum time has passed since launch.
 * @param aceleracaoZVerticalAtual Current vertical acceleration (m/s^2).
 * @param timeSinceLaunch Time elapsed since launch (ms).
 * @return true if motor burnout is detected, false otherwise.
 */
bool detectBurnout(float verticalAcceleration, unsigned long timeSinceLaunch) {
    if (timeSinceLaunch > MIN_MOTOR_BURN_TIME && verticalAcceleration < ACCEL_LIMIT_BURNOUT) {
        DEBUG_PRINTLN_F("FLIGHT_LOGIC: Burnout detected.");
        return true;
    }
    return false;
}

/**
 * @brief Detects if airbrakes should be activated based on altitude and vertical velocity.
 * @details Airbrakes are activated if the altitude exceeds a minimum threshold
 *         and the vertical velocity is below a defined limit.
 * @param filteredAltitude Current filtered altitude from Kalman filter (m).
 * @param filteredVerticalVelocity Current filtered vertical velocity (m/s).
 * @return true if airbrakes should be activated, false otherwise.
 */
bool detectAirbrakesActuation(float filteredAltitude, float filteredVerticalVelocity) {
    if (filteredAltitude > MIN_ACTUATION_HEIGHT && filteredVerticalVelocity < VEL_LIMIT_ACTUACTION) {
        DEBUG_PRINTLN_F("FLIGHT_LOGIC: Conditions for airbrake actuation met.");
        return true;
    }
    return false;
}

/**
 * @brief Detects apogee based on velocity and altitude.
 * @details Apogee is detected if the vertical velocity drops below a defined threshold
 *         for a certain number of consecutive readings.
 * @param filteredVerticalVelocity Current filtered vertical velocity (m/s).
 * @param filteredAltitude Current filtered altitude from Kalman filter (m).
 * @return true if apogee is detected, false otherwise.
 */
bool detectApogee(float filteredVerticalVelocity, float filteredAltitude) {
    
    if (filteredAltitude > maxRecordedHeight) {
        maxRecordedHeight = filteredAltitude;
    }
    // Increment counter if condition is met
    if (filteredVerticalVelocity < VEL_LIMIT_APOGEE) {
        consecutiveApogeeCount++;
        DEBUG_PRINT_F("Apogee condition met. Count: ");
        DEBUG_PRINTLN(consecutiveApogeeCount);

    } else {
        consecutiveApogeeCount = 0;
    }

    // Check if required consecutive readings are met
    if (consecutiveApogeeCount >= READINGS_FOR_APOGEE_CONFIRMATION) {
        DEBUG_PRINTLN_F("FLIGHT_LOGIC: APOGEE DETECTED AND CONFIRMED!");
        DEBUG_PRINT_F("Maximum Recorded Altitude: ");
        DEBUG_PRINT(maxRecordedHeight, 2);
        DEBUG_PRINTLN_F("m");
        
        consecutiveApogeeCount = 0;     
        return true; 
    }
    return false;
}

/**
 * @brief Detects landing based on vertical velocity, altitude, and time since apogee.
 * @details Landing is detected if the vertical velocity drops below a defined threshold
 *         and a minimum time has passed since apogee.
 * @param filteredVerticalVelocity Current filtered vertical velocity (m/s).
 * @param filteredAltitude Current filtered altitude from Kalman filter (m).
 * @param timeSinceApogee Time elapsed since apogee (ms).
 * @return true if landing is detected, false otherwise.
 */
bool detectLanding(float filteredVerticalVelocity, float filteredAltitude, unsigned long timeSinceApogee) {

    if (timeSinceApogee < MIN_TIME_AFTER_APOGEE_LANDING) {
        return false; 
    }

    if (abs(filteredVerticalVelocity) < VEL_LIMIT_LANDING && filteredAltitude < ALT_LIMIT_LANDING) {
        if (firstGroundDetectionTime == 0) {
            firstGroundDetectionTime = millis();
        }
        if (millis() - firstGroundDetectionTime >= MIN_TIME_LANDING) {
            DEBUG_PRINTLN_F("FLIGHT_LOGIC: Landing detected!");
            return true;
        }
    } else {
        firstGroundDetectionTime = 0; // Reset counter if leaving landing conditions
    }
    return false;
}

/**
 * @brief Reads the current tilt angle using IMU measurements.
 * @return The current tilt angle in degrees.
 */
float readCurrentTilt() {
    return calcTilt(); 
}

