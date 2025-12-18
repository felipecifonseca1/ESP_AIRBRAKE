#include "Logica_voo.h"
#include "Config_voo.h"
#include <ESP32Servo.h> // Use ESP32Servo library on ESP32
#include <math.h>
#include <Arduino.h>
#include "Funcoes_suporte_IMU.h"

#define MACH_VELOCITY 335

// Configuracoes do servo
Servo airbrakeServo; // Servo object
const uint8_t PIN_SERVO_AIRBRAKE = 27; 
// Config 90 graus max ---> angulo real = angulo/2
// *Necessario ajustar os valores min e max conforme o servo utilizado*
const uint16_t min_pulse = 560;  // 0°  - write(0)
const uint16_t max_pulse = 1520; // 90° - write(180)
bool test_servo = false;   // Sweep do servo

// Parametros para deteccao lancamento
const float ACCEL_LIMIT_LAUNCH = 1.5f * G_CONSTANTE_GRAVITACIONAL_MS2; // m/s^2 
const int8_t HEIGHT_LIMIT_LAUNCH = 4; // m

// Parametros para deteccao burnout
const uint16_t MIN_MOTOR_BURN_TIME = 5000; // ms 
const float ACCEL_LIMIT_BURNOUT = -0.5; // m/s^2 

// Parametros para atuacao airbrakes
const uint16_t MIN_ACTUATION_HEIGHT = 500; // m
const float VEL_LIMIT_ACTUACTION = 0.7 * MACH_VELOCITY;    // m/s

// Parametros para deteccao apogeu
const float VEL_LIMIT_APOGEE = 1.0; // m/s 
static float maxRecordedHeight = 0; // Para ajudar a confirmar
static uint8_t consecutiveApogeeCount = 0;
const uint8_t READINGS_FOR_APOGEE_CONFIRMATION = 25;

// Parametros para deteccao pouso
const float VEL_LIMIT_LANDING = 0.5;  // m/s
const uint8_t ALT_LIMIT_LANDING = 10; // m
const uint32_t MIN_TIME_AFTER_APOGEE_LANDING = 90000; // ms (1.5 min após apogeu para considerar pouso)
static uint32_t firstGroundDetectionTime = 0;
const uint16_t MIN_TIME_LANDING = 5000; // ms (5s de condições estáveis de pouso)

/**
 * @brief Initializes the servo for airbrake actuation.
 * @return true if the servo was successfully initialized, false otherwise.
 */
bool setupServo() { 
    DEBUG_PRINTLN_F("SETUP_SERVO: Inicializando servo ");
    DEBUG_PRINT_F("SINALIZACAO: Configurando Servo no pino ");
    DEBUG_PRINTLN(PIN_SERVO_AIRBRAKE);
    airbrakeServo.attach(PIN_SERVO_AIRBRAKE, min_pulse, max_pulse);
    airbrakeServo.write(0);

    if (airbrakeServo.attached()) {
        DEBUG_PRINTLN_F("SINALIZACAO: Servo anexado com sucesso.");
        return true;
    } else {
        DEBUG_PRINT_F("ERRO: Falha ao anexar o servo no pino ");
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

    // Converte a posição desejada para um ângulo de servo - 0 = 0° | 1.0 = 90(45°)° (abertura completa)
    int anguloServo = map(desiredPosition * 100, 0, 100, 0, 90); 
    airbrakeServo.write(anguloServo);
    // DEBUG_PRINT_F("LOGICA_VOO: Comandando airbrakes para posicao (0-1): "); 
    // DEBUG_PRINTLN(desiredPosition);
}

/**
 * @brief Commands the airbrakes to fully retracted position.
 */
void retractAirbrakes() {
    airbrakeServo.write(0); 
    // DEBUG_PRINTLN_F("LOGICA_VOO: Airbrakes totalmente retraídos.");
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

    bool saudeOk = true; 

    //Checagem IMU
    float ax = mpu.getAccX(); float ay = mpu.getAccY(); float az = mpu.getAccZ();
    float accel_mag = sqrt(ax*ax + ay*ay + az*az);
    if (accel_mag < 0.85f || accel_mag > 1.15f) { // Tolerância de +/- 0.15g
        DEBUG_PRINTLN_F("LogicaVoo: Falha Saude - Magnitude Accel.");
        saudeOk = false;
    }
    if (abs(mpu.getGyroX()) > 3.0f || abs(mpu.getGyroY()) > 3.0f || abs(mpu.getGyroZ()) > 3.0f) { // Tolerância de 3 dps
        DEBUG_PRINTLN_F("LogicaVoo: Falha Saude - Gyro alto.");
        saudeOk = false;
    }

    // Checagem Kalman 
    if (abs(filteredAltitude) > 8.0f) { // Tolerância de +/- 8m no solo
        DEBUG_PRINT_F("LogicaVoo: Falha Saude - Altitude Kalman: ");
        DEBUG_PRINTLN(filteredAltitude);
        saudeOk = false;
    }
    if (abs(filteredVerticalVelocity) > 1.0f) { // Tolerância de +/- 1m/s
        DEBUG_PRINT_F("LogicaVoo: Falha Saude - Velocidade Kalman: ");
        DEBUG_PRINTLN(filteredVerticalVelocity);
        saudeOk = false;
    }
    
    // Checagem Servo
    if (!airbrakeServo.attached()){
        DEBUG_PRINTLN_F("LogicaVoo: Falha Saude - Conexão servo.");
        saudeOk = false;
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

    return saudeOk;
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
        DEBUG_PRINTLN_F("LOGICA_VOO: Lançamento detectado!");
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
        DEBUG_PRINTLN_F("LOGICA_VOO: Burnout detectado.");
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
        DEBUG_PRINTLN_F("LOGICA_VOO: Condicoes para atuacao dos airbrakes atendidas.");
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
    // Incrementa o contador caso condicao seja atendida
    if (filteredVerticalVelocity < VEL_LIMIT_APOGEE) {
        consecutiveApogeeCount++;
        DEBUG_PRINT_F("Condicao de apogeu atendida. Contagem: ");
        DEBUG_PRINTLN(consecutiveApogeeCount);

    } else {
        consecutiveApogeeCount = 0;
    }

    // Verifica o número de leituras consecutivas necessarias 
    if (consecutiveApogeeCount >= READINGS_FOR_APOGEE_CONFIRMATION) {
        DEBUG_PRINTLN_F("LOGICA_VOO: APOGEU DETECTADO E CONFIRMADO!");
        DEBUG_PRINT_F("Altitude Maxima Registrada: ");
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
            DEBUG_PRINTLN_F("LOGICA_VOO: Pouso detectado!");
            return true;
        }
    } else {
        firstGroundDetectionTime = 0; // Reseta o contador se sair das condições de pouso
    }
    return false;
}

/**
 * @brief Reads the current tilt angle using IMU measurements.
 * @return The current tilt angle in degrees.
 */
float readCurrentTilt() {
    return calcTilt(); // Função de Funcoes_suporte_IMU.h
}

