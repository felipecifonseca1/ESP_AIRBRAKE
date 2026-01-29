#ifndef CONFIG_VOO_H
#define CONFIG_VOO_H
#include <cstdint>

// CONTROLE GLOBAL DE DEBUG SERIAL
#define DEBUG_SERIAL_ENABLED 1 // 1 - debug | 0 - voo

// --- Pin Configuration ---
constexpr uint8_t PIN_BUZZER = 14;
constexpr uint8_t PIN_LED_1  = 12;
constexpr uint8_t PIN_LED_2  = 2;  
constexpr uint8_t PIN_SD_CS   = 5;
constexpr uint8_t PIN_SD_SCK  = 18;
constexpr uint8_t PIN_SD_MOSI = 23;
constexpr uint8_t PIN_SD_MISO = 19;
constexpr uint8_t PIN_FLASH_CS = 4; 
constexpr uint8_t PIN_SERVO  = 27;

// --- Physics Parameters ---
constexpr float G_GRAVITATIONAL_CONSTANT = 9.80665f; // Gravity acceleration m/s^2
constexpr float MACH_VELOCITY = 335.0f; // MACH velocity m/s
constexpr float RHO_AIR = 1.293f; // Air density kg/m^3
constexpr float ROCKET_MASS_KG = 30.605f;
constexpr float ROCKET_AREA_M2 = 0.02097f;

// --- System Settings ---
constexpr float Ts_ms = 20.0f;                    // Loop time in ms (50Hz)
constexpr float Ts = Ts_ms / 1000.0f;             // Loop time in seconds
constexpr float apoggeTargetAltitude_m = 3254.0f; // Apogee target altitude in meters
constexpr float maxTiltAngle = 20.0f;             // Maximum tilt angle in degrees
constexpr uint8_t LOG_SYNC_INTERVAL = 1; // 1  = Safe Mode | 10 = Balanced | 50 = Fast

// --- IMU Configuration ---
constexpr bool CALIBRATE_IMU_ON_STARTUP = false;
constexpr bool PRINT_IMU_PARAMS = false;
constexpr bool PERFORM_FINE_TUNING = false;
constexpr bool PHYSICAL_Z_AXIS_DOWN = true; // Defines the orientation of the physical IMU Z axis
constexpr bool ERASE_CALIBRATION_ON_STARTUP = false;

// --- HIL Simulation ---
constexpr bool HIL_MODE_ACTIVE = true;
constexpr char HIL_FILENAME[] = "/Teste_HIL_Sensors_no_bias.csv";

// --- Servo Configuration ---
constexpr uint16_t SERVO_MIN_PULSE = 560;
constexpr uint16_t SERVO_MAX_PULSE = 1520;

// --- Controller Constants ---
constexpr float PID_KP = 0.025f;
constexpr float PID_KI = 0.075f;
constexpr float PID_KD = 0.02f;

// --- Kalman Filter Tuning ---
constexpr float KALMAN_VAR_PROC_POS = 1.0f;
constexpr float KALMAN_VAR_PROC_VEL = 3.0f;
constexpr float KALMAN_VAR_MEAS_ALT = 1.0f;
constexpr float KALMAN_VAR_ZUPT_VEL = 0.000001f;

// --- Events Detection ---
constexpr float LAUNCH_ACCEL_THRESHOLD_G = 1.5f; 
constexpr uint16_t BURNOUT_MIN_MOTOR_TIME_MS = 5000;
constexpr float    BURNOUT_ACCEL_THRESHOLD_G = 0.5f;
constexpr float APOGEE_VEL_THRESHOLD_MS = 0.5f;


// Macros Serial Debug
#if DEBUG_SERIAL_ENABLED == 1
    // Se o debug estiver ATIVADO:
    #include <Arduino.h> // Necessário para Serial e F()

    #define DEBUG_PRINT(...)         Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...)       Serial.println(__VA_ARGS__)

    #define DEBUG_PRINT_F(str)     Serial.print(F(str))
    #define DEBUG_PRINTLN_F(str)   Serial.println(F(str))
    
    // Para placas como Teensy que suportam Serial.printf 
    // Adicione outras definições de placas se necessário 
    #if defined(__IMXRT1062__) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40) || defined(ESP32)
        #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
    #else
        #define DEBUG_PRINTF(fmt, ...) \
            do { \
                Serial.print(F("[printf no dispo] ")); \
                Serial.print(F(fmt)); \
            } while(0)
        #define DEBUG_PRINTF(fmt, ...) // Alternativa: compila para nada
    #endif

#else
    // Se o debug estiver DESATIVADO:
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
    #define DEBUG_PRINT_F(str)
    #define DEBUG_PRINTLN_F(str)
    #define DEBUG_PRINTF(fmt, ...)
#endif // DEBUG_SERIAL_ENABLED


#endif // CONFIG_VOO_H
