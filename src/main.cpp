/**
 * @file main.cpp
 * @brief Main flight software for ESP32 Airbrake System.
 * @details Integrates IMU, Barometer, Kalman Filter, PID Control, and Data Logging.
 * * @author Felipe Fonseca
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>

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
bool calibrate_imu_on_startup = true;                     // Calibrate IMU on startup
bool print_imu_params = false;                            // Print IMU parameters after calibration
bool perform_fine_tuning = false;                         // Fine tuning flag for IMU
const float Ts_ms = 20.0;                                 // Loop time in ms (50Hz)
const float Ts = Ts_ms / 1000.0f;                         // Loop time in seconds
uint32_t previousLoopTime = 0;                            // Stores previous loop time
const uint32_t MAX_WAIT_TIME_LANDING = 600000;            // 10 minutes max wait time for landing 
const uint32_t LANDING_TIMEOUT = 120000;                  // 2 minutes safety timeout after landing detection
bool serialCommActive = true;                             // Serial communication flag
const float G_GRAVITATIONAL_CONSTANT = 9.80665f;          // Gravity acceleration in m/s^2
const u_int8_t WDT_TIMEOUT_S = 5;                         // Watchdog timeout in seconds

// Flight State & Data
RawFlightData flightData; // Struct from DataManager.h
HILSimulationData hilData;

// --- Flight Mode Flags ---
const bool HIL_MODE_ACTIVE = true; // true - HIL mode | false - Flight mode
const char* HIL_FILENAME = "/Teste_HIL.csv";

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
float filteredAltitude_m = 0.0;
float filteredVerticalVelocity_ms = 0.0;
float netVerticalAcceleration_ms2 = 0.0; 
float barometricPressure_Pa = 0.0f;
float tilt_deg = 0.0;
float delta_V_ms = 0.0;
float controlGain1 = 0.0;
float controlGain2 = 0.0;
float controlInput = 0.0;
float airbrakeDeployment = 0.0;

// Variables of the Flight State Machine
enum class FlightState {
    SENSOR_CALIBRATION,
    HEALTH_CHECK,
    WAIT_LAUNCH,
    FLIGHT,
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
    float var_proc_pos = 1.0; 
    float var_proc_vel = 3.0;  
    Q_kf << var_proc_pos*(Ts*Ts*Ts*Ts)/4.0, var_proc_pos*(Ts*Ts*Ts)/2,
            var_proc_pos*(Ts*Ts*Ts)/2, var_proc_vel*Ts*Ts;

    float var_med_alt = 1.0; // Standard variance for altitude measurement
    float var_zupt_vel = 0.000001; // Very low variance for ZUPT velocity measurement

    R_kf << var_med_alt, 0,
            0, var_zupt_vel;  

    P0_kf << 1, 0,  // Altitude uncertainty
             0, 1;  // Velocity uncertainty

    X0_kf << 0, 0; // AGL altitude and vertical speed initial estimates

    kf.init(F_kf, G_kf, H_kf, Q_kf, R_kf, P0_kf, X0_kf);
    DEBUG_PRINTLN_F("Kalman filter initialized.");
}

// PID gains (obtained through simulation) and Controller Initialization
float Kp = 0.025;
float Ki = 0.075;
float Kd = 0.02;

float mass_kg = 30.605; // Rocket mass in kg
float area_m2 = 0.02097; // Reference area of the control surface in m²
Controller controller = Controller(Kp, Ki, Kd, mass_kg, area_m2, Ts);

void setupController() {
    controller.setLimits(0, 1);
    DEBUG_PRINTLN_F("Controller initialized.");
}

// uint32_t kalmanTime = 0;
// uint32_t attitudeTime = 0;

void fullStateEstimateUpdate() {
    // kalmanTime = millis();
    // attitudeTime = millis();
    if (HIL_MODE_ACTIVE) {
        // --- HIL Mode: Read data from CSV file ---
        hilData = DataManager::getInstance().readHILStep();
        
        if (!hilData.dadosValidos) {
            // End of HIL simulation
            if (DataManager::getInstance().isLoggingActive()) {
                DataManager::getInstance().stopLogging();
                Serial.println("End of HIL simulation. Logging stopped.");
            }
            return; 
        }
        
        // Data from HIL
        netVerticalAcceleration_ms2 = hilData.netVerticalAcceleration_ms2;
        barometricPressure_Pa = hilData.barometricPressure_Pa;
        tilt_deg = 90 - hilData.tilt; // Convert tilt to match system definition
    } else {
        // --- Real Mode: Read data from sensors ---
        if (!mpu.update()) {
            DEBUG_PRINTLN_F("Failed to read IMU data!");
            return; 
        }
        
        netVerticalAcceleration_ms2 = computeNetAcceleration();
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
    float measuredAltitude = altitudeFromPressure(barometricPressure_Pa);
    
    if (measuredAltitude > -9000.0f) { // Check for valid altitude measurement
        Eigen::Matrix<float, 2, 1> Z_kf;
        Z_kf << measuredAltitude, 0.0f; // Zero velocity measurement for ZUKF
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
    
    flightData.timestamp = millis();
    flightData.accX = mpu.getAccX();
    flightData.accY = mpu.getAccY();
    flightData.accZ = mpu.getAccZ();
    flightData.gyroX = mpu.getGyroX();
    flightData.gyroY = mpu.getGyroY();
    flightData.gyroZ = mpu.getGyroZ();
    flightData.magX = mpu.getMagX(); 
    flightData.magY = mpu.getMagY();
    flightData.magZ = mpu.getMagZ();
    flightData.qW = mpu.getQuaternionW();
    flightData.qX = mpu.getQuaternionX();
    flightData.qY = mpu.getQuaternionY();
    flightData.qZ = mpu.getQuaternionZ();
    flightData.filteredAltitude = filteredAltitude_m; 
    flightData.filteredVerticalVelocity = filteredVerticalVelocity_ms; 
    flightData.netVerticalAcceleration = netVerticalAcceleration_ms2; 
    flightData.tilt = tilt_deg; // lertilt()
    flightData.barometricPressure = barometricPressure_Pa;
    // flightData.barometricPressure =  getGroundPressureP0_BMP(); // For debugging
    flightData.airbrakeDeployment = airbrakeDeployment; 
    flightData.gain1 = controlGain1; 
    flightData.gain2 = controlGain2; 
    flightData.flightState = u_int8_t(flightState);
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

    if (checkFlightSystemHealth(filteredAltitude_m, filteredVerticalVelocity_ms)) { 
        healthCheckCount++;
        DEBUG_PRINT_F("Component health OK this iteration. Count: ");
        DEBUG_PRINTLN(healthCheckCount);

        if (healthCheckCount >= REQ_HEALTH_CHECKS) {
            Serial.println("HEALTH CHECK: Consecutive checks OK. Transitioning...");
            signalSuccessfullModule("Health Check Complete"); 

            flightState = FlightState::WAIT_LAUNCH;
            DataManager::getInstance().startLogging(); // Activate data logging
        
            // System settings for wait-for-launch
            DataManager::getInstance().setDecimationFactor(10); // Saves every 10x20ms = 200ms
            setDriftLearning(true); // Enable drift learning on the IMU
            setFilterBeta(30.0f); // Increase filter beta for faster response during wait
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

void waitLaunchLoop() {

    // Continuously update ground pressure reference to compensate drift
    recalibrateGroundPressure();
    fullStateEstimateUpdate();
    DataManager& logger = DataManager::getInstance();

    DEBUG_PRINT_F("STATE: WAIT_LAUNCH | Alt: ");
    DEBUG_PRINT(filteredAltitude_m);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(filteredVerticalVelocity_ms);
    DEBUG_PRINT_F("m/s | AccelZ: ");
    DEBUG_PRINT(netVerticalAcceleration_ms2);
    DEBUG_PRINTLN_F("m/s^2");
    // DEBUG_PRINT_F(",Alt:");
    // DEBUG_PRINT(filteredAltitude_m);
    // DEBUG_PRINT_F(",VelZ:");
    // DEBUG_PRINTLN(filteredVerticalVelocity_ms);
    
    if (logger.isLoggingActive()) {
        updateLogger();
        logger.logDataSD(flightData); 
    }

    if (detectLaunch(netVerticalAcceleration_ms2,filteredAltitude_m)) { 
        DEBUG_PRINTLN_F("LAUNCH DETECTED!");
        
        // System settings for flight
        setDriftLearning(false); // Disable drift learning on IMU after launch
        setFilterBeta(3.0f);
        R_kf(1,1) = 1000000000.0f; // Set high variance on velocity measurement to ignore zero input during flight
        logger.setDecimationFactor(1); // Save every 20ms during flight

        flightState = FlightState::FLIGHT;
        stateEntryTime = millis();
        launchDetectedTime = millis(); 
    }
}

void flightLoop() {
    DEBUG_PRINT_F("STATE: FLIGHT | Alt: ");
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

    unsigned long tempoDesdeLancamento = millis() - launchDetectedTime;
    if (detectBurnout(netVerticalAcceleration_ms2, tempoDesdeLancamento)) { 
        DEBUG_PRINTLN_F("BURNOUT DETECTED!");
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
    DEBUG_PRINT_F("STATE: AIRBRAKE_DEPLOYMENT | Alt: ");
    DEBUG_PRINT(filteredAltitude_m);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(filteredVerticalVelocity_ms);
    DEBUG_PRINT_F("m/s | Tilt: ");
    DEBUG_PRINT(tilt_deg);
    DEBUG_PRINT_F("° | Computed Deflection: ");
    DEBUG_PRINTLN(airbrakeDeployment);
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

    if (detectApogee(filteredVerticalVelocity_ms, filteredAltitude_m)) { 
        DEBUG_PRINTLN_F("APOGEE DETECTED!");
        flightState = FlightState::APOGEE;
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

    if (HIL_MODE_ACTIVE == false) {
        DEBUG_PRINT_F("STATE: DESCENT | Alt: ");
        DEBUG_PRINT(filteredAltitude_m);
        DEBUG_PRINT_F("m | VelZ:  ");
        DEBUG_PRINT(filteredVerticalVelocity_ms);
        DEBUG_PRINTLN_F("m/s");
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

    
    if (!EEPROM.begin(EEPROM_SIZE)) {
        DEBUG_PRINTLN_F("CRITICAL ERROR: Failed to initialize EEPROM!");
    } else {
        DEBUG_PRINTLN_F("EEPROM initialized successfully.");
    }

    // Initialize basics: I2C, SPI and signaling system
    Wire.begin();
    Wire.setClock(400000); // Set I2C to 400kHz - test different values if necessary ---> wire length influences
    
    SPI.begin();
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

    // Sensors or HIL setup
    if (HIL_MODE_ACTIVE) {
        DEBUG_PRINTLN_F("**** HIL MODE ACTIVATED ****");
        if (logger.initHIL(HIL_FILENAME)) {
            
            
            HILSimulationData firstSample = logger.readHILStep();
            
            if (firstSample.dadosValidos) {
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
        DEBUG_PRINTLN_F("**** REAL FLIGHT MODE ACTIVATED ****");
        if (!setup_IMU(calibrate_imu_on_startup, perform_fine_tuning, print_imu_params)) {
            signalFailedModule("IMU");
            DEBUG_PRINTLN_F("FATAL ERROR: IMU setup failed. System halted.");
            while (1) { ledBlink(PIN_LED_STATUS_2, 1, 500, 500); delay(100); }
        } else {
            signalSuccessfullModule("IMU");
        }

        DEBUG_PRINTLN_F("Initializing BMP (setup_BMP)...");
        if (!setupBMP()) { 
            signalFailedModule("BMP280");
            DEBUG_PRINTLN_F("FATAL ERROR: BMP280 setup failed. System halted.");
            while (1) { ledBlink(PIN_LED_STATUS_2, 1, 500, 500); delay(100); }
        } else {
            signalSuccessfullModule("BMP280");
        }
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
    esp_task_wdt_init(WDT_TIMEOUT_S, true); // Timeout of 3 seconds
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
            case FlightState::FLIGHT:
                flightLoop();
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
            DEBUG_PRINT_F("WARNING: Main loop overrun! Execution time: ");
            DEBUG_PRINT(millis() - previousLoopTime);
            DEBUG_PRINTLN_F(" ms!");
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