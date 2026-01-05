#include "Logica_voo.h"
#include "Config_voo.h"
#include <ESP32Servo.h> // Use ESP32Servo library on ESP32
#include <math.h>
#include <Arduino.h>
#include "Funcoes_suporte_IMU.h"
#include <ArduinoEigenDense.h> 

using namespace Eigen;

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
const int BURNOUT_CONFIRMATION_COUNT = 5; // Number of consecutive readings
const int MOVING_AVERAGE_WINDOW_SIZE = 20;

// Parameters for airbrake actuation
const uint16_t MIN_ACTUATION_HEIGHT = 500; // m
const float VEL_LIMIT_ACTUACTION = 0.7 * MACH_VELOCITY;    // m/s

// Parameters for apogee detection
const float VEL_LIMIT_APOGEE = 0.5; // m/s 
const uint8_t READINGS_FOR_APOGEE_CONFIRMATION = 10; // Number of consecutive readings
const int REGRESSION_WINDOW_SIZE = 30; // Size of the window for altitude regression

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

    static float bufferAcc[MOVING_AVERAGE_WINDOW_SIZE];
    static int indexHead = 0;
    static float movingSum = 0.0f;
    static bool fullBuffer = false;
    static int burnoutCounter = 0;

    // Update moving average buffer
    movingSum -= bufferAcc[indexHead];
    bufferAcc[indexHead] = verticalAcceleration;
    movingSum += bufferAcc[indexHead];

    // Move head index
    indexHead = (indexHead + 1) % MOVING_AVERAGE_WINDOW_SIZE;
    
    // Check if buffer is full
    if (!fullBuffer && indexHead == 0) fullBuffer = true;

    // Calculate moving average
    float movingAverageAcc;
    if (fullBuffer) {
        movingAverageAcc = movingSum / MOVING_AVERAGE_WINDOW_SIZE;
    } else {
        int amostrasAtuais = (indexHead == 0) ? 1 : indexHead; 
        movingAverageAcc = movingSum / (float)amostrasAtuais; 
    }

    // Check burnout condition
    if (timeSinceLaunch > MIN_MOTOR_BURN_TIME && movingAverageAcc < ACCEL_LIMIT_BURNOUT) {
        burnoutCounter++;
    }
    else {
        burnoutCounter = 0;
    }

    if (burnoutCounter >= BURNOUT_CONFIRMATION_COUNT) {
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
 * @param netVerticalAcceleration Current net vertical acceleration (m/s^2).
 * @return true if apogee is detected, false otherwise.
 */
bool detectApogee(float filteredVerticalVelocity, float filteredAltitude) {
    static float maxRecordedHeight = 0; // To help confirm apogee
    static uint8_t consecutiveApogeeCount = 0;
    // Update max recorded height
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
 * @brief Detects apogee by calculating a parabolic fit to altitude data.
 * @details Uses the Least Squares method to solve h(t) = at^2 + bt + c.
 * Apogee is confirmed if the curve is concave down (a < 0) and the 
 * estimated velocity (derivative at current time) is negative.
 * * @param filteredAltitude Current altitude (barometric or filtered) [m]
 * @param currentTime_ms Current timestamp [ms]
 * @return true if the vertex of the parabola has been passed.
 */
bool detectApogeeByRegression(float filteredAltitude, unsigned long currentTime_ms) {
    // --- INTERNAL STATE ---
    static float timeBuffer[REGRESSION_WINDOW_SIZE];
    static float altBuffer[REGRESSION_WINDOW_SIZE];
    static int headIndex = 0;
    static int sampleCount = 0;
    static bool apogeeConfirmed = false;

    // If already detected, lock state and return true
    if (apogeeConfirmed) return true;

    // 1. Add new sample to Circular Buffer
    // Convert ms to seconds to avoid overflow in power calculations (t^4)
    timeBuffer[headIndex] = currentTime_ms / 1000.0f; 
    altBuffer[headIndex] = filteredAltitude;
    
    headIndex = (headIndex + 1) % REGRESSION_WINDOW_SIZE;
    
    // Only start calculating when the buffer is full
    if (sampleCount < REGRESSION_WINDOW_SIZE) {
        sampleCount++;
        return false; 
    }

    // 2. Time Normalization (Crucial for float precision)
    // Use time relative to the oldest sample in the window (t0)
    int oldestIndex = headIndex; // In a full buffer, head points to the oldest element
    float t0 = timeBuffer[oldestIndex];

    // 3. Accumulate Sums for Least Squares
    double sum_t = 0, sum_t2 = 0, sum_t3 = 0, sum_t4 = 0;
    double sum_y = 0, sum_ty = 0, sum_t2y = 0;

    for (int i = 0; i < REGRESSION_WINDOW_SIZE; i++) {
        int idx = (oldestIndex + i) % REGRESSION_WINDOW_SIZE;
        
        float t = timeBuffer[idx] - t0; // Relative time 
        float y = altBuffer[idx];
        double t2 = (double)t * t;
        
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
    A << REGRESSION_WINDOW_SIZE, sum_t,  sum_t2,
         sum_t,                  sum_t2, sum_t3,
         sum_t2,                 sum_t3, sum_t4;

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
    // If a > 0, the rocket would be curving upwards (impossible at apogee).
    bool isConcave = (a < -0.05f);

    // B. Estimated Velocity at the CURRENT instant (End of Window):
    // Regression smooths out the velocity. v(t) = 2*a*t + b
    float t_final = timeBuffer[(headIndex - 1 + REGRESSION_WINDOW_SIZE) % REGRESSION_WINDOW_SIZE] - t0;
    float estimatedVelocity = 2.0f * a * t_final + b;

    // Must be descending (< 0) with a small hysteresis margin
    bool isDescending = (estimatedVelocity < -0.5f);

    if (isConcave && isDescending) {
        apogeeConfirmed = true;
        // DEBUG_PRINTLN_F("APOGEE (Pure Regression) Confirmed!");
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

