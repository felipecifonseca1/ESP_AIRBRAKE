/**
 * @file main.cpp
 * @brief Main flight software for ESP32 Airbrake System.
 * @details Integrates IMU, Barometer, Kalman Filter, PID Control, and Data Logging.
 ** @author Felipe Fonseca
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
// #include <WiFi.h>
// #include <esp_bt.h>

// Modules
#include "Funcoes_suporte_IMU.h" 
#include "Funcoes_BMP.h"         
#include "Logica_voo.h"           
#include "KalmanFilter.hh"
#include "Controller.hh"
#include "AltitudeSpeedTable.hh"
#include "DragCoefficientTable.hh"
#include <ArduinoEigenDense.h>  
#include "Sinalizacao.h"
#include "Config_voo.h"
#include "DataManager.h"
#define EEPROM_SIZE 256 // Define EEPROM size

// --- Global Variables ---
bool calibrate_imu_on_startup = false;                     // Calibrate IMU on startup
bool print_imu_params = false;                            // Print IMU parameters after calibration
bool perform_fine_tuning = false;                         // Fine tuning flag for IMU
const float Ts_ms = 20.0f;                                 // Loop time in ms (50Hz)
const float Ts = Ts_ms / 1000.0f;                         // Loop time in seconds
uint32_t previousLoopTime = 0;                            // Stores previous loop time
const uint32_t MAX_WAIT_TIME_LANDING = 600000;            // 10 minutes max wait time for landing 
const uint32_t LANDING_TIMEOUT = 120000;                  // 2 minutes safety timeout after landing detection
bool serialCommActive = true;                             // Serial communication flag
const float G_GRAVITATIONAL_CONSTANT = 9.80665f;          // Gravity acceleration in m/s^2
u_int8_t print_count = 5; 
const int WDT_TIMEOUT_MS = 15000;                          // Watchdog timeout in milliseconds

esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS, 
    .idle_core_mask = (1 << 0), 
    .trigger_panic = true
}; 

// Flight State & Data
RawFlightData flightData; 
HILSimulationData hilData;

// --- Flight Mode Flags ---
const bool HIL_MODE_ACTIVE =  false; // true - HIL mode | false - Flight mode
const char* HIL_FILENAME = "/Teste_HIL_Sensors_no_bias.csv";
const bool HIL_MODE_Z_DOWN = true; // true - Z axis down | false - Z axis up

// Matrices and Kalman Filter Initialization
Eigen::Matrix<float, 2, 2> F_kf;
Eigen::Matrix<float, 2, 1> G_kf;
Eigen::Matrix<float, 2, 2> H_kf;
Eigen::Matrix<float, 2, 2> Q_kf;
Eigen::Matrix<float, 2, 2> R_kf;
Eigen::Matrix<float, 2, 2> P0_kf;
Eigen::Matrix<float, 2, 1> X0_kf;
KalmanFilter kf;

// Variables to store flight parameters
float filteredAltitude_m = 0.0f;
float filteredVerticalVelocity_ms = 0.0f;
float netVerticalAcceleration_ms2 = 0.0f; 
float barometricPressure_Pa = 0.0f;
float tilt_deg = 0.0f;
float delta_V_ms = 0.0f;
float controlGain1 = 0.0f;
float controlGain2 = 0.0f;
float controlInput = 0.0f;
float airbrakeDeployment = 0.0f;

// Variables of the Flight State Machine
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

FlightState flightState = FlightState::SENSOR_CALIBRATION;
uint32_t stateEntryTime = 0;
uint32_t launchDetectedTime = 0; 
uint32_t apogeeDetectedTime = 0;

// ---  Kalman Filter Setup ---
void setupKalman() {
    

    F_kf << 1.0f, Ts,
            0.0f, 1.0f;

    G_kf << 0.5 * Ts * Ts,
            Ts;

    H_kf << 1, 0
          , 0, 1; // ZUKF velocity

    // Adjust process noise covariance Q_kf and measurement noise covariance R_kf
    float var_proc_pos = 1.0f; 
    float var_proc_vel = 3.0f;  
    Q_kf << var_proc_pos*(Ts*Ts*Ts*Ts)/4.0, var_proc_pos*(Ts*Ts*Ts)/2,
            var_proc_pos*(Ts*Ts*Ts)/2, var_proc_vel*Ts*Ts;

    float var_med_alt = 1.0f; // Standard variance for altitude measurement
    float var_zupt_vel = 0.000001f; // Very low variance for ZUPT velocity measurement

    R_kf << var_med_alt, 0,
            0, var_zupt_vel;  

    P0_kf << 1, 0,  // Altitude uncertainty
             0, 1;  // Velocity uncertainty

    X0_kf << 0, 0; // AGL altitude and vertical speed initial estimates

    kf.init(F_kf, G_kf, H_kf, Q_kf, R_kf, P0_kf, X0_kf);
    DEBUG_PRINTLN_F("Kalman filter initialized.");
}

// PID gains (obtained through simulation) and Controller Initialization
float Kp = 0.025f;
float Ki = 0.075f;
float Kd = 0.02f;

float mass_kg = 30.605f; // Rocket mass in kg
float area_m2 = 0.02097f; // Reference area of the control surface in m²
Controller controller = Controller(Kp, Ki, Kd, mass_kg, area_m2, Ts);

void setupController() {
    controller.setLimits(0, 1);
    DEBUG_PRINTLN_F("Controller initialized.");
}

// uint32_t kalmanTime = 0;
// uint32_t attitudeTime = 0;

bool motor_on;
void fullStateEstimateUpdate() {
    // kalmanTime = millis();
    // attitudeTime = millis();

    flightState == FlightState::MOTOR_ON ? motor_on = true : motor_on = false;

    if (HIL_MODE_ACTIVE) {
        // --- HIL Mode: Read data from CSV file ---
        hilData = DataManager::getInstance().readHILStep();
        
        if (!hilData.valid) {
            // End of HIL simulation
            if (DataManager::getInstance().isLoggingActive()) {
                DataManager::getInstance().stopLogging();
                Serial.println("End of HIL simulation. Logging stopped.");
            }
            return; 
        }

        // Data from HIL
        barometricPressure_Pa = hilData.barometricPressure_Pa;
        
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


            if (HIL_MODE_Z_DOWN) {
                if (!mpu.update(Ts, motor_on, false, accX_g, accY_g, -accZ_g, -gyroX_degs, -gyroY_degs, gyroZ_degs, magX_mG, -magY_mG, -magZ_mG)) {
                        DEBUG_PRINTLN_F("Failed to update simulated IMU data!");
                        return; 
                    }
                
                
                netVerticalAcceleration_ms2 = computeNetAcceleration(true, accX_g, accY_g, accZ_g, false);
                tilt_deg = readCurrentTilt();
            } else{
                if (!mpu.update(Ts, motor_on, false, accX_g, accY_g, accZ_g, gyroX_degs, gyroY_degs, gyroZ_degs, magX_mG, magY_mG, magZ_mG)) {
                        DEBUG_PRINTLN_F("Failed to update simulated IMU data!");
                        return; 
                    }
                
                
                netVerticalAcceleration_ms2 = computeNetAcceleration(true, accX_g, accY_g, accZ_g, false);
                tilt_deg = readCurrentTilt();
            }

        } else {
            // Modo Simples 
            netVerticalAcceleration_ms2 = hilData.netVerticalAcceleration_ms2;
            tilt_deg = 90 - hilData.tilt; // Convert tilt to match system definition
        }
        

    } else {
        // --- Real Mode: Read data from sensors ---
        if (!mpu.update(Ts, motor_on)) {
            DEBUG_PRINTLN_F("Failed to read IMU data!");
            return; 
        }

        netVerticalAcceleration_ms2 = computeNetAcceleration(false);
        // DEBUG_PRINT_F("Attitude time:");
        // DEBUG_PRINTLN(millis() - attitudeTime);
        barometricPressure_Pa = getPressaoBMPAtual(); 
        tilt_deg = readCurrentTilt();
    }

    // --- Kalman Filter Execution ---
    
    // Control Input U for the Prediction Step
    Eigen::Matrix<float, 1, 1> U_kf;
    U_kf << netVerticalAcceleration_ms2;
    
    // Prediction Step
    kf.Predict(U_kf);

    // Altitude Measurement from Barometer
    float measuredAltitude_m = altitudeFromPressure(barometricPressure_Pa);
    
    if (measuredAltitude_m > -9000.0f) { // Check for valid altitude measurement
        Eigen::Matrix<float, 2, 1> Z_kf;
        Z_kf << measuredAltitude_m, 0.0f; // Zero velocity measurement for ZUKF
        kf.Update(Z_kf, R_kf);
    } else {
        DEBUG_PRINTLN_F("WARNING: Invalid altitude measurement. Kalman Update skipped.");
    }

    // Get the posteriori state estimate
    Matrix<float, 2, 1> stateEstimate = kf.getPosterioriState();
    filteredAltitude_m = stateEstimate(0, 0);
    filteredVerticalVelocity_ms = stateEstimate(1, 0);
    // DEBUG_PRINT_F("Kalman time:");
    // DEBUG_PRINTLN(millis() - kalmanTime);
}

void updateLogger(){
    
    flightData.timestamp = millis();                                    // Timestamp [ms]
    flightData.accX = mpu.getAccX();                                    // Body frame x axis acceleration [m/s²]
    flightData.accY = mpu.getAccY();                                    // Body frame y axis acceleration [m/s²]
    flightData.accZ = mpu.getAccZ();                                    // Body frame z axis acceleration [m/s²]
    flightData.gyroX = mpu.getGyroX();                                  // Body frame x axis rotacional velocity [°/s]
    flightData.gyroY = mpu.getGyroY();                                  // Body frame y axis rotacional velocity [°/s]
    flightData.gyroZ = mpu.getGyroZ();                                  // Body frame z axis rotacional velocity [°/s]
    flightData.magX = mpu.getMagX()/10.0;                               // Body frame x axis magnetic field [uT]
    flightData.magY = mpu.getMagY()/10.0;                               // Body frame y axis magnetic field [uT]
    flightData.magZ = mpu.getMagZ()/10.0;                               // Body frame z axis magnetic field [uT]
    flightData.qW = mpu.getQuaternionW();                               // W quaternion component [-]
    flightData.qX = mpu.getQuaternionX();                               // X quaternion component [-]
    flightData.qY = mpu.getQuaternionY();                               // Y quaternion component [-]
    flightData.qZ = mpu.getQuaternionZ();                               // Z quaternion component [-]
    flightData.filteredAltitude = filteredAltitude_m;                   // Inertial frame filtered altitude (z) [m]
    flightData.filteredVerticalVelocity = filteredVerticalVelocity_ms;  // Inertial frame filtered velocity (vz) [m/s]
    flightData.netVerticalAcceleration = netVerticalAcceleration_ms2;   // Inertial frame net acceleration (az) [m/s²]
    flightData.tilt = tilt_deg;                                         // Z axis tilt angle [°]
    flightData.barometricPressure = barometricPressure_Pa;              // Barometric pressure [Pa]
    flightData.airbrakeDeployment = airbrakeDeployment*100;             // Airbrake deployment [%]
    flightData.gain1 = controlGain1;                                    // PID gain [-]
    flightData.gain2 = controlGain2;                                    // Cd gain [-]
    flightData.flightState = int(flightState);                          // Flight state [-]
}

// --- State Machine Functions ---

void calibrationCheckLoop() {
    DEBUG_PRINTLN_F("STATE: Sensor Calibration");

    if (hasCalibrationDataIMU() && getGroundPressureP0_BMP() != 101324.0f) { 
        DEBUG_PRINTLN_F("Initial calibration (IMU/BMP P0) done in setup.");
        flightState = FlightState::HEALTH_CHECK;
        stateEntryTime = millis();
    }
}

static u_int8_t healthCheckCount = 0; 
const u_int8_t REQ_HEALTH_CHECKS = 5; 

void healthCheckLoop() {
    DEBUG_PRINTLN_F("State: Health Check");

    fullStateEstimateUpdate();
    recalibrateGroundPressure(); 

    if (checkFlightSystemHealth(filteredAltitude_m, filteredVerticalVelocity_ms) || HIL_MODE_ACTIVE) { 
        healthCheckCount++;
        DEBUG_PRINT_F("Component health OK this iteration. Count: ");
        DEBUG_PRINTLN(healthCheckCount);

        if (healthCheckCount >= REQ_HEALTH_CHECKS || HIL_MODE_ACTIVE) {
            Serial.println("HEALTH CHECK: Consecutive checks OK. Transitioning...");
            signalSuccessfullModule("Health Check Complete"); 

            flightState = FlightState::WAIT_LAUNCH;
            DataManager::getInstance().startLogging(); // Activate data logging
        
            // System settings for wait-for-launch
            DataManager::getInstance().setDecimationFactor(10); // Saves every 10x20ms = 200ms
            setDriftLearning(true); // Enable drift learning on the IMU
            setFilterBeta(2.5f); // Increase filter beta for faster response during wait
            R_kf(1,1) = 0.000001f; // Reduce velocity measurement variance for ZUKF

            stateEntryTime = millis();         
            healthCheckCount = 0;    
        }
    } else {
        DEBUG_PRINTLN_F("ERROR: Component health failure detected by checkFlightSystemHealth()!");
        signalFailedModule("Health Check Iteration"); 
        healthCheckCount = 0; // Reset count if any check fails   
    }
}
 
u_int8_t contador = 0;
void waitLaunchLoop() {
    contador ++;
    // Continuously update ground pressure reference to compensate drift
    recalibrateGroundPressure();
    fullStateEstimateUpdate();
    DataManager& logger = DataManager::getInstance();
    if (contador > print_count){
        DEBUG_PRINT_F("STATE: WAIT_LAUNCH | Alt: ");
        DEBUG_PRINT(filteredAltitude_m);
        DEBUG_PRINT_F("m | VelZ: ");
        DEBUG_PRINT(filteredVerticalVelocity_ms);
        DEBUG_PRINT_F("m/s | AccelZ: ");
        DEBUG_PRINT(netVerticalAcceleration_ms2);
        DEBUG_PRINT_F("m/s^2 | Tilt: ");
        DEBUG_PRINT(tilt_deg);
        DEBUG_PRINTLN_F("°");
        contador = 0;
    }

    if (logger.isLoggingActive()) {
        updateLogger();
        logger.logDataSD(flightData); 
    }

    if (detectLaunch(netVerticalAcceleration_ms2,filteredAltitude_m)) { 
        DEBUG_PRINTLN_F("LAUNCH DETECTED!");
        
        // System settings for flight
        setDriftLearning(false); // Disable drift learning on IMU after launch
        setFilterBeta(0.0f);
        R_kf(1,1) = 1000000000.0f; // Set high variance on velocity measurement to ignore zero input during flight
        logger.setDecimationFactor(1); // Save every 20ms during flight

        flightState = FlightState::MOTOR_ON;
        stateEntryTime = millis();
        launchDetectedTime = millis(); 
    }
}

void motorOnLoop() {
    contador ++;
    if (contador > print_count){
        DEBUG_PRINT_F("STATE: MOTOR_ON | Alt: ");
        DEBUG_PRINT(filteredAltitude_m);
        DEBUG_PRINT_F("m | VelZ: ");
        DEBUG_PRINT(filteredVerticalVelocity_ms);
        DEBUG_PRINT_F("m/s | Tilt: ");
        DEBUG_PRINT(tilt_deg);
        DEBUG_PRINTLN_F("°");
        contador = 0;
    }

    DataManager& logger = DataManager::getInstance();
    fullStateEstimateUpdate();

    if (logger.isLoggingActive()) {
        updateLogger();
        logger.logDataSD(flightData); 
    }

    unsigned long tempoDesdeLancamento = millis() - launchDetectedTime;
    if (detectBurnout(netVerticalAcceleration_ms2, tempoDesdeLancamento)) { 
        DEBUG_PRINTLN_F("BURNOUT DETECTED!");
        setFilterBeta(0.5f);
        flightState = FlightState::BURNOUT; 
        stateEntryTime = millis();
    }
}

void loopBurnout() {
    DEBUG_PRINT_F("STATE: Burnout | Alt: ");
    DEBUG_PRINT(filteredAltitude_m);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(filteredVerticalVelocity_ms);
    DEBUG_PRINTLN_F("m/s");
    DataManager& logger = DataManager::getInstance();

    fullStateEstimateUpdate();
    if (logger.isLoggingActive()) {
        updateLogger();
        logger.logDataSD(flightData); 
    }

    if (detectAirbrakesActuation(filteredAltitude_m, filteredVerticalVelocity_ms)) {
        DEBUG_PRINTLN_F("Appropriate speed detected, initiating airbrake actuation.");
        flightState = FlightState::AIRBRAKE_DEPLOYMENT;
        stateEntryTime = millis();
    }
}

void airbrakeDeploymentLoop() {
    contador ++;
    if (contador > print_count){
        DEBUG_PRINT_F("STATE: AIRBRAKE_DEPLOYMENT | Alt: ");
        DEBUG_PRINT(filteredAltitude_m);
        DEBUG_PRINT_F("m | VelZ: ");
        DEBUG_PRINT(filteredVerticalVelocity_ms);
        DEBUG_PRINT_F("m/s | Tilt: ");
        DEBUG_PRINT(tilt_deg);
        DEBUG_PRINT_F("° | Computed Deflection: ");
        DEBUG_PRINTLN(airbrakeDeployment);
        contador = 0;
    }
    DataManager& logger = DataManager::getInstance();

    fullStateEstimateUpdate(); 

    if (logger.isLoggingActive()) {
        updateLogger();
        logger.logDataSD(flightData); 
    }

    // AIRBRAKE CONTROL LOGIC
    delta_V_ms = lookUpSpeed(filteredAltitude_m) - filteredVerticalVelocity_ms;
    controlGain1 = controller.computePID(0, delta_V_ms);
    controlGain2 = controller.computeCd(3260, filteredAltitude_m, filteredVerticalVelocity_ms, -G_GRAVITATIONAL_CONSTANT, 1.293);
    controlInput = controlGain1 + controlGain2;

    if (tilt_deg < 20){
        // Convert speed to Mach ---> vel/335

        airbrakeDeployment = getNearestActuation((filteredVerticalVelocity_ms/335), controlInput); 
    }
    else{
        airbrakeDeployment = 0.0; 
    }

    commandAirbrakes(airbrakeDeployment);

    // if (detectApogee(filteredVerticalVelocity_ms, filteredAltitude_m)) { 
    //     DEBUG_PRINTLN_F("APOGEE DETECTED!");
    //     flightState = FlightState::APOGEE;
    //     stateEntryTime = millis();
    //     apogeeDetectedTime = millis();
    // }

     if (detectApogeeByRegression(filteredAltitude_m, millis())) { 
        DEBUG_PRINTLN_F("APOGEE DETECTED!");
        flightState = FlightState::APOGEE;
        setFilterBeta(1.0);
        stateEntryTime = millis();
        apogeeDetectedTime = millis();
    }
     
}

void apogeeLoop() {
    DEBUG_PRINT_F("STATE: APOGEE | Max Alt: ");
    DEBUG_PRINT(filteredAltitude_m);
    DEBUG_PRINTLN_F("m");
    fullStateEstimateUpdate(); 

    DataManager& logger = DataManager::getInstance();
    if (logger.isLoggingActive()) {
        updateLogger();
        logger.logDataSD(flightData); 
    }
    retractAirbrakes(); 
    DEBUG_PRINTLN_F("Airbrakes retracted at apogee.");
    
    flightState = FlightState::DESCENT; 
    logger.setDecimationFactor(10); // Save every 10x20ms = 200ms
    stateEntryTime = millis();
}

void descentLoop() {

    contador ++;
    if (contador > print_count){
        DEBUG_PRINT_F("STATE: DESCENT | Alt: ");
        DEBUG_PRINT(filteredAltitude_m);
        DEBUG_PRINT_F("m | VelZ: ");
        DEBUG_PRINT(filteredVerticalVelocity_ms);
        DEBUG_PRINTLN_F("m/s");
        contador = 0;
    }
 

    fullStateEstimateUpdate();
    DataManager& logger = DataManager::getInstance();
    if (logger.isLoggingActive()) {
        updateLogger();
        logger.logDataSD(flightData); 
    }

    unsigned long tempoDesdeApogeu = millis() - apogeeDetectedTime;
    if (detectLanding(filteredVerticalVelocity_ms, filteredAltitude_m, tempoDesdeApogeu)) { 
        DEBUG_PRINTLN_F("LANDING DETECTED!");
        flightState = FlightState::LANDING;
        logger.setDecimationFactor(50); // Save every 50x20ms = 1s
        stateEntryTime = millis();
    }

     // Timeout for LANDING detection if not detected after long time
     else if (tempoDesdeApogeu > MAX_WAIT_TIME_LANDING) {
         DEBUG_PRINTLN_F("Timeout for LANDING detection.");
         flightState = FlightState::LANDING; // Force LANDING state
         logger.setDecimationFactor(50); // Save every 50x20ms = 1s
         stateEntryTime = millis();
     }
}

void landingLoop() {
    DEBUG_PRINTLN_F("STATE: LANDING");
    // Continue saving final data for a short period (120000ms - 2min)
    DataManager& logger = DataManager::getInstance();
    if (logger.isLoggingActive() && (millis() - stateEntryTime)< 120000 ) {
        updateLogger();
        logger.logDataSD(flightData); 
    }
    else{
        logger.stopLogging(); // Desliga o data logging
    }
    DEBUG_PRINTLN_F("Operation finished. Awaiting recovery.");
    delay(10000);
}

// --- SETUP E LOOP PRINCIPAIS ---
void setup() {
    Serial.begin(115200);
    // Wait for Serial to connect, with timeout
    unsigned long serialStartTime = millis();
    while (!Serial && (millis() - serialStartTime < 4000));

    // // Turn off WIFI 
    // WiFi.disconnect(true);
    // WiFi.mode(WIFI_OFF);

    // // Turn off Bluetooth
    // esp_bt_controller_disable();
    // esp_bt_controller_deinit();
    
    if (!EEPROM.begin(EEPROM_SIZE)) {
        DEBUG_PRINTLN_F("CRITICAL ERROR: Failed to initialize EEPROM!");
    } else {
        DEBUG_PRINTLN_F("EEPROM initialized successfully.");
    }

    // Initialize basics: I2C, SPI and signaling system
    Wire.begin();
    Wire.setClock(400000); // Set I2C to 400kHz - test different values if necessary ---> wire length influences
    
    setupSinalizacao();
    signalStartupStart();

    // eraseCalibration(); // First time calibration

    DEBUG_PRINTLN_F("==== INITIALIZING AIRBRAKE SYSTEM ====");

    // Storage setup
    DEBUG_PRINTLN_F("Initializing SD card...");
    DataManager& logger = DataManager::getInstance();
    if (!logger.setupSD()) {
        signalFailedModule("Log SD Card Setup");
        DEBUG_PRINTLN_F("FATAL ERROR: Failed to initialize SD card.");
        buzzerBeeps(10,300,150, 1200);
    } else {
        signalSuccessfullModule("Log SD Card Setup");
    }

    if (!setup_IMU(calibrate_imu_on_startup, perform_fine_tuning, print_imu_params)) {
        signalFailedModule("IMU");
        DEBUG_PRINTLN_F("FATAL ERROR: IMU setup failed. System halted.");
        while (1) { ledBlink(PIN_LED_STATUS_2, 1, 500, 500); delay(100); }
    } else {
        signalSuccessfullModule("IMU");
    }

    // Sensors or HIL setup
    if (HIL_MODE_ACTIVE) {
        DEBUG_PRINTLN_F("**** HIL MODE ACTIVATED ****");
        if (logger.initHIL(HIL_FILENAME)) {
            
            
            HILSimulationData firstSample = logger.readHILStep();
            
            if (firstSample.valid) {
                float pressaoInicial = firstSample.barometricPressure_Pa;
                setGroundPressureP0_BMP(pressaoInicial);

                DEBUG_PRINT_F("HIL: P0 set to: ");
                DEBUG_PRINT(pressaoInicial);
                DEBUG_PRINTLN_F(" Pa");
                
                logger.resetHIL(); 
                
            } else {
                DEBUG_PRINTLN_F("HIL: Error - Empty file!");
                while(1); 
            }
            
            signalSuccessfullModule("HIL Init");
        } else {
            DEBUG_PRINTLN_F("HIL FATAL ERROR: File not found!");
            while(1);
        }

    } else {
        DEBUG_PRINTLN_F("Initializing BMP (setup_BMP)...");
        if (!setupBMP()) { 
            signalFailedModule("BMP280");
            DEBUG_PRINTLN_F("FATAL ERROR: BMP280 setup failed. System halted.");
            while (1) { ledBlink(PIN_LED_STATUS_2, 1, 500, 500); delay(100); }
        } else {
            signalSuccessfullModule("BMP280");
        }
        DEBUG_PRINTLN_F("**** REAL FLIGHT MODE ACTIVATED ****");
        }
     
    if (setupServo()) { 
        signalSuccessfullModule("Servo");
        retractAirbrakes(); 
    } else {
        signalFailedModule("Servo");
        DEBUG_PRINTLN_F("FATAL ERROR: Servo setup failed.");
        while (1) { ledBlink(PIN_LED_STATUS_2, 1, 250, 250); delay(100); }
    }

    delay(500); // Time to stabilize data

    DEBUG_PRINTLN_F("Initializing Controller...");
    setupController(); 
    signalSuccessfullModule("Controller");
    delay(100);

    // Setup Kalman and Controller 
    DEBUG_PRINTLN_F("Initializing Kalman Filter...");
    setupKalman(); 

    Serial.flush();

    stateEntryTime = millis();
    previousLoopTime = millis();
    if (!HIL_MODE_ACTIVE){
        flightState = FlightState::HEALTH_CHECK; 
    }
    else {
        flightState =  FlightState::WAIT_LAUNCH; 
        logger.startLogging();
    }

    DEBUG_PRINTLN("WDT: Initializing Watchdog...");
    esp_task_wdt_init(&twdt_config); // Timeout of 3 seconds
    esp_task_wdt_add(NULL);; // Adds the current task (main loop) to the WDT
    DEBUG_PRINTLN("WDT: Active.");

    DEBUG_PRINTLN_F("All optional setups and tests completed.");
    signalStartupComplete(); 

    DEBUG_PRINT_F("Transitioning to initial state: "); 
    DEBUG_PRINTLN((int)flightState); 
}

void loop() {
    // Pega o tempo atual no início de cada passagem pelo loop
    unsigned long tempoAtual = millis();

    if (tempoAtual - previousLoopTime >= Ts_ms) {
        previousLoopTime += Ts_ms;
        
        switch (flightState) {
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
                loopBurnout();
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
                    DEBUG_PRINTLN_F("ERROR: Unknown STATE! Restarting...");
                flightState = FlightState::HEALTH_CHECK;
                stateEntryTime = millis();
                break;
            
        }
        
        // DEBUG_PRINT(millis() - previousLoopTime);
        // DEBUG_PRINTLN_F(" ms to execute!");
        // Serial.print(millis() - previousLoopTime);
        // Serial.println(" ms to execute!");

        if (millis() > previousLoopTime + Ts_ms) { 
            Serial.print("WARNING: Main loop overrun! Execution time: ");
            Serial.print(millis() - previousLoopTime);
            Serial.println(" ms!");
        }
    }
    if (serialCommActive){
        if (Serial.available()) {
        char cmd = Serial.read();
        DataManager& logger = DataManager::getInstance();
        esp_task_wdt_reset();
        
            if (cmd == 'd' || cmd == 'D') { // 'd' for Dump 
                Serial.println("Command received: Dump Log");
                logger.dumpCurrentLog();
                logger.stopLogging(); // Close active logs for safety
            }
            if (cmd == 'h' || cmd == 'H') { // 'H' de HIL
                // Pause the flight loop temporarily to receive the file
                logger.stopLogging(); // Close active logs for safety
                logger.receiveHILFile(HIL_FILENAME);
            }
            
            if (cmd == 'l' || cmd == 'L') { // 'l' to List
                logger.listFiles();
            }

            if (cmd == 'c' || cmd == 'C') {
                logger.clearAllLogs();
            }

            if (cmd == 'p' || cmd == 'P') {
                logger.stopLogging(); 
            }
        }
    }
    esp_task_wdt_reset(); // Resets the watchdog timer
}
