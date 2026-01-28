#ifndef CONFIG_VOO_H
#define CONFIG_VOO_H


// CONTROLE GLOBAL DE DEBUG SERIAL
#define DEBUG_SERIAL_ENABLED 1 // 1 - debug | 0 - voo

// --- Physics Constants ---
constexpr float G_GRAVITATIONAL_CONSTANT = 9.80665f; // Gravity acceleration m/s^2
constexpr float MACH_VELOCITY = 335.0f; // MACH velocity m/s
constexpr float RHO_AIR = 1.293f; // Air density kg/m^3

// --- System Settings ---
constexpr float Ts_ms = 20.0f;                    // Loop time in ms (50Hz)
constexpr float Ts = Ts_ms / 1000.0f;             // Loop time in seconds
constexpr float apoggeTargetAltitude_m = 3254.0f; // Apogee target altitude in meters
constexpr float maxTiltAngle = 20.0f;             // Maximum tilt angle in degrees
// Logging Configuration
// 1  = Safe Mode (Save every line, slowest ~9.6ms)
// 10 = Balanced (Save every 10 lines, fast ~2.0ms)
// 50 = Fast (Save every 50 lines, very fast ~1.1ms)
constexpr int LOG_SYNC_INTERVAL = 10; 


// --- IMU Configuration ---
constexpr bool CALIBRATE_IMU_ON_STARTUP = false;
constexpr bool PRINT_IMU_PARAMS = false;
constexpr bool PERFORM_FINE_TUNING = false;
constexpr bool PHYSICAL_Z_AXIS_DOWN = true; // Defines the orientation of the physical IMU Z axis

// --- HIL Simulation ---
constexpr bool HIL_MODE_ACTIVE = true;
constexpr char HIL_FILENAME[] = "/Teste_HIL_Sensors_no_bias.csv";


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
