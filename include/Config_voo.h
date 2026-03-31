#ifndef CONFIG_VOO_H
#define CONFIG_VOO_H

#include <cstdint>
#include <Arduino.h>

// =========================================================================
// SYSTEM DEBUG & TELEMETRY
// =========================================================================

#define DEBUG_SERIAL_ENABLED 1 // 1: Debug (Detailed Serial) | 0: Flight (Silent)
#define USE_TELEPLOT 2 // 2: Full Teleplot | 1: Basic Teleplot | 0: Standard Monitor

// =========================================================================
// HARDWARE PINOUT & I2C ADDRESSES
// =========================================================================

constexpr uint8_t PIN_BUZZER              = 39;   // (JTAG Conflict)
constexpr uint8_t PIN_LED_1               = 10;  
constexpr uint8_t PIN_LED_2               = 48;  
constexpr uint8_t PIN_SERVO               = 42;   // (JTAG Conflict)
constexpr uint8_t PIN_SDA                 = 1;    // I2C Data
constexpr uint8_t PIN_SCL                 = 2;    // I2C Clock

// SDIO 4-bit Pins
constexpr uint8_t PIN_SDIO_CLK            = 12;
constexpr uint8_t PIN_SDIO_CMD            = 9;
constexpr uint8_t PIN_SDIO_D0             = 13;
constexpr uint8_t PIN_SDIO_D1             = 14;
constexpr uint8_t PIN_SDIO_D2             = 8;
constexpr uint8_t PIN_SDIO_D3             = 46; 
constexpr uint8_t PIN_SDIO_DET            = 11; 

// //! Conflict with internal PSRAM
// constexpr uint8_t PIN_FLASH_CS            = 36; //! Conflict with PSRAM
// constexpr uint8_t PIN_FLASH_MOSI          = 35; //! Conflict with PSRAM
// constexpr uint8_t PIN_FLASH_MISO          = 37; //! Conflict with PSRAM
// constexpr uint8_t PIN_FLASH_SCK           = 17;
// constexpr uint8_t PIN_FLASH_WP            = 38;
// constexpr uint8_t PIN_FLASH_HOLD          = 16;

constexpr uint8_t PIN_FLASH_CS            = 255; 
constexpr uint8_t PIN_FLASH_MOSI          = 255; 
constexpr uint8_t PIN_FLASH_MISO          = 255; 
constexpr uint8_t PIN_FLASH_SCK           = 255;
constexpr uint8_t PIN_FLASH_WP            = 255;
constexpr uint8_t PIN_FLASH_HOLD          = 255;

constexpr uint8_t I2C_ADDRESS_IMU         = 0x68;      // MPU9250 I2C address
constexpr uint8_t I2C_ADDRESS_BARO        = 0x76;      // BMP280 I2C address

// =========================================================================
// PHYSICAL CONSTANTS & ROCKET SPECS
// =========================================================================

constexpr float G_GRAVITATIONAL_CONSTANT = 9.80665f;   // Standard gravity [m/s^2]
constexpr float MACH_VELOCITY            = 335.0f;     // Speed of sound at sea level [m/s]
constexpr float RHO_AIR                  = 1.293f;     // Standard air density [kg/m^3]
constexpr float ROCKET_MASS_KG           = 30.605f;    // Current rocket mass [kg]
constexpr float AIRBRAKE_REF_AREA_M2     = 0.02097f;   // Total cross-sectional area of 4 petals [m^2]

// =========================================================================
// SYSTEM OPERATIONAL SETTINGS
// =========================================================================

constexpr float Ts_ms                     = 10.0f;     // Control loop period [ms] 
constexpr float Ts                        = 0.010f;    // Control loop period [s]
constexpr bool SIMULATE_GPS_TIMING        = true;      // GPS: Simulation timing once every 10 cycles
constexpr float apoggeTargetAltitude_m    = 3254.0f;   // Target mission apogee [m]
constexpr float maxTiltAngle              = 60.0f;     // Max safety tilt for actuation [deg]
constexpr float NET_ACC_THRESHOLD         = 0.2f;      // Vertical noise floor [m/s^2]
constexpr uint32_t WDT_TIMEOUT_MS         = 5000;      // Watchdog timeout in milliseconds
constexpr bool useRecovery                = false;     // Enable recovery sequence logic
constexpr bool runBusScan                 = false;     // Run I2C bus scan at startup

// --- IMU & Orientation ---
constexpr bool PHYSICAL_Z_AXIS_DOWN       = true;     // IMU Mounting   : true: Z-Down | false: Z-Up
constexpr bool USE_MAGNETOMETER           = false;      // Filter: Enable Mag for drift correction
constexpr float MAGNETOMETER_FUSION_WEIGHT = 0.01f;    // Mag Authority
constexpr float ATTITUDE_GYRO_CUTOFF_DPS  = 0.00f;     // Gyro Deadband: Ignore rotations < this value [dps]

constexpr bool CALIBRATE_IMU_ON_STARTUP   = true;      // Run library calib ONLY if data is missing
constexpr bool PRINT_IMU_PARAMS           = false;     // Print biases to Serial at boot
constexpr bool PERFORM_FINE_TUNING        = false;     // Run iterative bias tweak on every boot
constexpr bool FORCED_MAG_CALIBRATION     = false;     // Trigger the 30s visual spin routine
constexpr bool ERASE_CALIB_ON_STARTUP     = false;      // Force delete all saved IMU data
constexpr float CALIBRATION_ACCEL_TOL_G   = 0.0025f;   // Iterative target for Accel [g]
constexpr float CALIBRATION_GYRO_TOL_DPS  = 0.025f;    // Iterative target for Gyro [dps]
constexpr int   CALIBRATION_MAX_ITERATIONS  = 30;      // Safety limit for iterative calibration

// Acceleration magnitude window to trust gravity for orientation correction.
constexpr float ORIENTATION_MASK_MIN_G     = 0.85f;     // TRUST accel only if > 0.85g
constexpr float ORIENTATION_MASK_MAX_G     = 1.15f;     // TRUST accel only if < 1.15g

namespace AttitudeFilter {
    constexpr uint8_t NONE     = 0;
    constexpr uint8_t MADGWICK = 1;
    constexpr uint8_t MAHONY   = 2;
    constexpr uint8_t EKF      = 3;
    constexpr uint8_t MEKF     = 4;
    constexpr uint8_t NAV_MEKF = 5;
}
constexpr uint8_t DEFAULT_ATTITUDE_FILTER = AttitudeFilter::NAV_MEKF;

namespace MagLocation {
    constexpr uint8_t CUSTOM        = 0;
    constexpr uint8_t SAO_PAULO     = 1;
    constexpr uint8_t PIRASSUNUNGA  = 2;
    constexpr uint8_t MUNICH        = 3;
    constexpr uint8_t MIDLAND_TX    = 4;
}
constexpr uint8_t DEFAULT_MAG_LOCATION = MagLocation::SAO_PAULO;

// --- SD Card & Logging ---
constexpr bool ENABLE_DATA_LOGGING        = true;      // Master switch for all logging
constexpr bool ENABLE_SD_LOGGING          = false;      // Toggle SD (CSV)
constexpr bool ENABLE_INTERNAL_LOGGING    = false;      // Toggle Internal Flash (Binary)
constexpr bool ENABLE_EXTERNAL_LOGGING    = false;     // Toggle External Flash (Reserved)
constexpr bool ENABLE_TELEMETRY           = true;      // Enable real-time monitor prints
constexpr uint8_t TELEMETRY_LOGGING_DECIMATION = 5;    // Cycles between telemetry prints (10Hz) to save APB bandwidth

constexpr char     SD_LOG_FOLDER[]        = "/REG_VOO"; 
constexpr char     SD_LOG_BASENAME[]      = "VOO_";
constexpr uint16_t SD_MAX_LOG_FILES       = 1000; 
constexpr uint32_t SD_WRITE_TIMEOUT_MS    = 100;        // Max time allowed for single write [ms]
constexpr uint8_t  LOG_BUFFER_SIZE_SD     = 5;          // Records to buffer in RAM before SD write
constexpr uint16_t LOG_BUFFER_SIZE_INT    = 15000;      // Binary records to buffer in PSRAM (~660KB)
constexpr uint16_t LOG_PAD_FLUSH_SIZE     = 50;         // Pad Mode: Records to buffer before Flash write
constexpr uint16_t LOG_CHUNK_SIZE_DESCENT = 50;         // Descent: Records to trickle-write per cycle
constexpr uint16_t LOG_SYNC_INTERVAL_SD   = 10;         // Sync every buffer flush
constexpr uint16_t LOG_SYNC_INTERVAL_INT  = 15000;      // Sync Internal Flash 

// --- HIL (Hardware-In-the-Loop) ---
constexpr bool HIL_MODE_ACTIVE            = false;       // Enable  sensor simulation
constexpr char HIL_FILENAME[]             = "/Teste_HIL_Sensors_ZUp.csv";
constexpr uint32_t HIL_STABILIZATION_MS   = 20000;       // Time to wait for estimator to settle [ms]

// --- Servo Specs ---
constexpr bool TESTE_SERVO                = false;      // Enable servo test
constexpr uint16_t SERVO_MIN_PULSE        = 560;        // Pulse length for 0 deg (retracted) [us]
constexpr uint16_t SERVO_MAX_PULSE        = 1520;       // Pulse length for 90 deg (extended) [us]

// =========================================================================
// FLIGHT CONTROLLER TUNING
// =========================================================================

// --- PID Coefficients ---
constexpr float PID_KP                    = 0.025f;     // Proportional gain
constexpr float PID_KI                    = 0.075f;     // Integral gain
constexpr float PID_KD                    = 0.02f;      // Derivative gain

// --- Kalman Filter ---
constexpr float KALMAN_VAR_PROC_POS       = 1.0f;       // Process variance: Position
constexpr float KALMAN_VAR_PROC_VEL       = 3.0f;       // Process variance: Velocity
constexpr float KALMAN_VAR_MEAS_ALT       = 1.0f;       // Measurement variance: Baro Altitude
constexpr float KALMAN_VAR_ZUPT_VEL       = 0.000001f;  // Zero-velocity update variance

// =========================================================================
// FLIGHT EVENTS DETECTION
// =========================================================================

// --- Launch Detection ---
constexpr float LAUNCH_ACCEL_THRESHOLD_G   = 1.5f;     // Acceleration threshold for lift-off [g]
constexpr float LAUNCH_HEIGHT_THRESHOLD_M  = 4.0f;     // Minimum AGL altitude to confirm launch [m]
constexpr uint8_t LAUNCH_CONFIRMATION_COUNT = 10;      // Consecutive cycles to confirm launch (~200ms)

// --- Burnout Detection ---
constexpr uint16_t BURNOUT_MIN_MOTOR_TIME_MS = 5000;   // Wait time before allowing burnout check [ms]
constexpr float    BURNOUT_ACCEL_THRESHOLD_G = 0.5f;   // Acceleration drop threshold [g]
constexpr uint8_t  BURNOUT_WINDOW_SIZE       = 20;     // Gravity compensation window size
constexpr uint8_t  BURNOUT_CONFIRMATION_COUNT = 5;     // Cycles required for state lock

// --- Airbrake Actuation ---
constexpr float ACTUATION_MIN_HEIGHT_M     = 500.0f;   // Start control only above this altitude [m]
constexpr float ACTUATION_VEL_LIMIT_MACH   = 0.7f;     // Maximum allowed speed for deployment [Mach]

// --- Apogee Detection ---
constexpr float   APOGEE_VEL_THRESHOLD_MS  = 0.5f;     // Vertical speed threshold for peak [m/s]
constexpr uint8_t APOGEE_READINGS_CONFIRMATION = 10;   // Readings required to confirm apogee
constexpr uint8_t APOGEE_REGRESSION_WINDOW  = 30;      // Moving window for polynomial regression
constexpr float   APOGEE_REGRESSION_CONCAVITY_THRESHOLD = -0.05f; // Curve shape threshold
constexpr float   APOGEE_REGRESSION_DESCENDING_THRESHOLD = -0.5f; // Confirmed descent velocity [m/s]

// --- Landing Detection ---
constexpr float    LANDING_VEL_THRESHOLD_MS          = 0.5f;   // Max velocity for ground detect [m/s]
constexpr float    LANDING_ALT_THRESHOLD_M           = 10.0f;  // Max altitude for ground detect [m]
constexpr uint32_t LANDING_MIN_TIME_AFTER_APOGEE_MS  = 90000;  // Absolute safety timeout after apogee [ms]
constexpr uint16_t LANDING_STABLE_TIME_MS            = 5000;   // Stability duration for lock [ms]
constexpr uint32_t LANDING_MAX_WAIT_TIME_MS          = 600000; // Ultimate timeout (10 min) [ms]

// =========================================================================
// SYSTEM HEALTH & DIAGNOSTICS
// =========================================================================

constexpr uint8_t HEALTH_CHECK_REQUIRED_COUNT = 5;     // Success cycles to return "Health OK"
constexpr float   HEALTH_ACCEL_MAG_TOLERANCE  = 0.15f; // Stationary 1g test tolerance [g]
constexpr float   HEALTH_GYRO_TOLERANCE_DPS   = 1.0f;  // Stationary drift tolerance [dps]
constexpr float   HEALTH_ALT_TOLERANCE_M      = 4.0f;  // Baro variance tolerance at pad [m]
constexpr float   HEALTH_VEL_TOLERANCE_MS     = 1.0f;  // Kalman velocity variance at pad [m/s]

// =========================================================================
// DEBUG MACROS
// =========================================================================

#if DEBUG_SERIAL_ENABLED == 1

#define SERIAL_CHECK (Serial.availableForWrite() > 0)

#define DEBUG_PRINT(...)    if (SERIAL_CHECK) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...)  if (SERIAL_CHECK) Serial.println(__VA_ARGS__)
#define DEBUG_PRINT_F(str)  if (SERIAL_CHECK) Serial.print(F(str))
#define DEBUG_PRINTLN_F(str) if (SERIAL_CHECK) Serial.println(F(str))

#if defined(__IMXRT1062__) || defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40) || defined(ESP32)
#define DEBUG_PRINTF(fmt, ...) if (SERIAL_CHECK) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...) \
    do { \
        if (SERIAL_CHECK) { \
            Serial.print(F("[printf not available] ")); \
            Serial.print(F(fmt)); \
        } \
    } while (0)
#endif
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT_F(str)
#define DEBUG_PRINTLN_F(str)
#define DEBUG_PRINTF(fmt, ...)
#endif

// --- Teleplot Macros ---
#if USE_TELEPLOT >= 1
    #define PRINT_STATE(stateName) do { if (_telemetryEnabled) { Serial.print(F(">STATE:")); Serial.println(stateName); } } while (0)
    #define PLOT_VAR(name, val) \
        do { if (SERIAL_CHECK && _telemetryEnabled) { Serial.print(F(">")); Serial.print(name); Serial.print(F(":")); Serial.println(val); } } while (0)
#else
    #define PRINT_STATE(stateName) do { if (_telemetryEnabled) { Serial.print(F("STATE: ")); Serial.print(stateName); Serial.print(F(" | ")); } } while (0)
    #define PLOT_VAR(name, val) \
        do { if (SERIAL_CHECK && _telemetryEnabled) { Serial.print(name); Serial.print(F(": ")); Serial.print(val); Serial.print(F(" | ")); } } while (0)
#endif

#endif // CONFIG_VOO_H
