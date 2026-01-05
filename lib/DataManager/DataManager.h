/**
 * @file DataManager.h
 * @brief Data Manager for logging and HIL simulation.
 * @details This class handles data logging to SD card and SPI Flash,
 * as well as managing Hardware-in-the-Loop (HIL) simulation files.
 */

#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <SPIFlash.h>
#include "Config_voo.h"

// --- Data Structures ---

struct ScaledFlightData {
    unsigned long timestamp_ms;
    int16_t accX_scaled, accY_scaled, accZ_scaled;
    int16_t gyroX_scaled, gyroY_scaled, gyroZ_scaled;
    int16_t magX_scaled, magY_scaled, magZ_scaled;
    int16_t qW_scaled, qX_scaled, qY_scaled, qZ_scaled;
    int16_t filteredAltitude_scaled;
    int16_t filteredVerticalVelocity_scaled;
    int16_t netVerticalAcceleration_scaled;
    int16_t tilt_scaled; 
    float barometricPressure_scaled;
    int16_t airbrakeDeployment_scaled;
    int16_t gain1_scaled;
    int16_t gain2_scaled;
    uint8_t flightState;
}; 

struct RawFlightData {
    u_int32_t timestamp;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float qW, qX, qY, qZ;
    float filteredAltitude;
    float filteredVerticalVelocity;
    float netVerticalAcceleration;
    float tilt; 
    float barometricPressure;
    float airbrakeDeployment;
    float gain1, gain2;
    u_int8_t flightState;
};

struct HILSimulationData {
    bool valid = false; 
    float time_s = 0.0f;

    // Processed data
    float barometricPressure_Pa = 0.0f;
    float netVerticalAcceleration_ms2 = 0.0f;
    float tilt = 0.0f;

    // Full Sensor Mode Data 
    float accX_ms2 = 0.0f, accY_ms2 = 0.0f, accZ_ms2 = 0.0f;
    float gyroX_rads = 0.0f, gyroY_rads = 0.0f, gyroZ_rads = 0.0f;
    float magX_T = 0.0f, magY_T = 0.0f, magZ_T = 0.0f;
    
    // Flags to tell the main loop what data is available
    bool hasFullIMU = false; 
};

enum class HILMode {
    NONE,
    SIMPLE, // 3 Columns: Time, Pressure, NetAcc
    FULL    // 11 Columns: Time, Acc(3), Gyro(3), Mag(3), Pressure
};

class DataManager {
public:
    // Access to Singleton
    static DataManager& getInstance();

    // --- Initialization ---

    bool setupSD();
    bool setupFlash(bool erase = false);
    
    // --- Logging control ---

    void startLogging();
    void stopLogging();
    bool isLoggingActive() const { return _loggingActive; }
    void setDecimationFactor(uint16_t factor);
    void closeSDCard(); 

    // --- Core logging logic ---

    void logDataSD(const RawFlightData& data);
    void logDataFlash(const RawFlightData& data);
    
    // --- HIL Simulation ---

    bool initHIL(const char* filename);
    HILSimulationData readHILStep();
    void stopHIL();
    void resetHIL();

    // --- Auxilary serial tools ---

    void listFiles();
    void dumpCurrentLog();
    void clearAllLogs();
    void receiveHILFile(const char* HILFileName);

    // --- State getters ---

    String getCurrentFileName() const { return _currentSDFileName; }
    uint32_t getFlashAddress() const { return _flashAddr; }

    // Avoid copying the singleton
    DataManager(const DataManager&) = delete;
    void operator=(const DataManager&) = delete;

private:
    DataManager(); // Private constructor for singleton

    // Data directories
    const char* _logFolder = "/REG_VOO"; 
    const char* _logBasename = "VOO_";
    const uint16_t _maxLogFiles = 1000; 
    
    // State variables
    bool _loggingActive;
    bool _HILLoggingActive;
    HILMode _currentHILMode = HILMode::NONE;
    uint16_t _decimationFactor;
    uint16_t _decimationCounter;

    // SD Card
    File _logFile;
    File _hilFile;
    String _currentSDFileName;
    bool _sdAvailable;
    uint8_t _sdRecordCounter;
    const uint8_t _sdFlushLimit = 50;

    // Pins VSPI
    const u_int8_t _pinCS_SD = 5; 
    const u_int8_t _pinSCK_SD = 18; 
    const u_int8_t _pinMOSI_SD = 23; 
    const u_int8_t _pinMISO_SD = 19; 

    // Flash SPI
    SPIFlash _flash;
    uint32_t _flashAddr;
    bool _flashAvailable;
    const u_int8_t _pinCS_Flash = 15;
   
    // Helpers
    bool ensureSDConnection();
    String generateFileName();
    HILMode detectHILFormat(String headerLine);
};

#endif // DATA_MANAGER_H