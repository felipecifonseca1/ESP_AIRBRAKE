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
    int flightState;
};

struct HILSimulationData {
    bool dadosValidos = false; 
    float time_s = 0.0f;
    float barometricPressure_Pa = 0.0f;
    float netVerticalAcceleration_ms2 = 0.0f;
    float tilt = 0.0f;
};

class DataManager {
public:
    // Access to Singleton
    static DataManager& getInstance();

    // --- INITIALIZATION ---

    bool setupSD();
    bool setupFlash(bool erase = false);
    
    // --- LOGGING CONTROL ---

    void startLogging();
    void stopLogging();
    bool isLoggingActive() const { return _loggingActive; }
    void setDecimationFactor(uint16_t factor);
    void closeSDCard(); 

    // --- CORE LOGGING LOGIC ---

    void logDataSD(const RawFlightData& data);
    void logDataFlash(const RawFlightData& data);
    
    // --- HIL SIMULATION ---

    bool initHIL(const char* filename);
    HILSimulationData readHILStep();
    void stopHIL();
    void resetHIL();

    // --- AUXILIARY METHODS ---

    void listFiles();
    void dumpCurrentLog();
    void clearAllLogs();
    void receiveHILFile(const char* HILFileName);

    // --- STATE GETTERS ---

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
    uint16_t _decimationFactor;
    uint16_t _decimationCounter;

    // SD Card
    File _logFile;
    File _hilFile;
    String _currentSDFileName;
    bool _sdAvailable;
    uint8_t _sdRecordCounter;
    const uint8_t _sdFlushLimit = 50;
    const u_int8_t _pinCS_SD = 5; 
    const u_int8_t _pinSCK_SD = 18; 
    const u_int8_t _pinMOSI_SD = 23; 
    const u_int8_t _pinMISO_SD = 19; 

    // Flash SPI
    SPIFlash _flash;
    uint32_t _flashAddr;
    bool _flashAvailable;
    const u_int8_t _pinCS_Flash = 15;
   
    bool ensureSDConnection();
    String generateFileName();
};

#endif // DATA_MANAGER_H