/**
 * @file DataManager.h
 * @brief Data Manager for logging and HIL simulation.
 * @details This class handles data logging to SD card and SPI Flash,
 * as well as managing Hardware-in-the-Loop (HIL) simulation files.
 */

#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <FFat.h>
#include <SPIMemory.h>
#include "Config_voo.h"

// --- Data Structures ---

// (BinaryFlightRecord removed in favor of ScaledFlightData)

struct __attribute__((packed)) ScaledFlightData {
    uint32_t timestamp_ms;          // 4 bytes
    int16_t accX_scaled, accY_scaled, accZ_scaled; // 6 bytes
    int16_t gyroX_scaled, gyroY_scaled, gyroZ_scaled; // 6 bytes
    int16_t magX_scaled, magY_scaled, magZ_scaled; // 6 bytes
    int16_t qW_scaled, qX_scaled, qY_scaled, qZ_scaled; // 8 bytes
    int16_t altitude_scaled;       // 2 bytes (dm)
    int16_t velocity_scaled;       // 2 bytes (cm/s)
    int16_t accel_net_scaled;      // 2 bytes (cm/s2)
    int16_t tilt_scaled;           // 2 bytes (centi-deg)
    uint32_t pressure_scaled;      // 4 bytes (Pa)
    uint8_t airbrake_scaled;       // 1 byte (0-100%)
    int16_t gain1_scaled;          // 2 bytes
    int16_t gain2_scaled;          // 2 bytes
    uint8_t flightState;           // 1 byte
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
    int16_t airbrakeDeployment;
    float pid_gain, cd_gain;
    uint8_t flightState;
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

struct IODiagnostics {
    uint32_t sdWrite_us;
    uint32_t sdWriteAvg_us;
    uint32_t internalFlashWrite_us;
    uint32_t internalFlashWriteAvg_us;
    uint32_t telemetryPrint_us;
    uint32_t totalTaskCycle_us;
    uint32_t maxSdWrite_us;
    uint32_t maxInternalFlashWrite_us;
    uint16_t ffatBufferCount;
};

class DataManager {
public:
    // Access to Singleton
    static DataManager& getInstance();

    // --- Initialization ---

    bool setupSD();
    bool setupInternalFlash(bool formatIfFailed = true);
    bool setupFlash(bool erase = false);
    
    void forceSync();

    // --- Logging control ---
    
    void startLogging();
    void stopLogging();
    bool isLoggingActive() const { return _loggingActive; }
    void setDecimationFactor(uint16_t factor);
    void closeSDCard(); 
    
    // Target Toggles
    void setSDLogging(bool enabled) { _sdEnabled = enabled; }
    void setInternalLogging(bool enabled) { _internalEnabled = enabled; }
    void setExternalLogging(bool enabled) { _externalEnabled = enabled; }
    
    bool isSDEnabled() const { return _sdEnabled; }
    bool isInternalEnabled() const { return _internalEnabled; }
    bool isExternalEnabled() const { return _externalEnabled; }
    
    // --- Core logging logic ---

    void logFlightData(const RawFlightData& data);
    
    // --- HIL Simulation ---

    bool initHIL(const char* filename);
    HILSimulationData readHILStep();
    void stopHIL();
    void resetHIL();

    // --- Auxilary tools ---

    void listSDFiles();
    void listInternalFiles();
    void dumpSDCurrentLog();
    void dumpInternalFlash(int fileIndex = -1); // Default to latest
    int getLatestInternalLogIndex() const;
    void clearSDAllLogs();
    void clearInternalFlash();
    void receiveHILFile(const char* HILFileName);
    void runFrequencyTest(uint32_t freq, u_int16_t flushLimit, u_int16_t numberOfRecords, bool onlyPrintf = true, bool oneBitMode = false);
    void runStrategyBenchmark(uint32_t freq, u_int16_t numberOfRecords);

    // --- State getters ---

    String getCurrentFileName() const { return _currentSDFileName; }
    uint32_t getFlashAddress() const { return _flashAddr; }
    
    // --- Diagnostics ---
    const IODiagnostics& getIODiagnostics() const { return _ioDiag; }
    IODiagnostics& getIODiagnosticsMutable() { return _ioDiag; }
    void resetIODiagnostics() { _ioDiag = {0}; }

    // Avoid copying the singleton
    DataManager(const DataManager&) = delete;
    void operator=(const DataManager&) = delete;

private:
    DataManager(); // Private constructor for singleton
    // State initialization
    void initStates() {
        _loggingActive = false;
        _stopRequested = false;
        _HILLoggingActive = false;
        _decimationFactor = 1;
        _decimationCounter = 0;
        _sdAvailable = false;
        _sdEnabled = true;       
        _internalEnabled = true; 
        _externalEnabled = false; 
        _sdRecordCounter = 0;
        _sdLEDCounter = 0;
        // ... (Flash/Helpers init is in constructor)
    }

    // Data directories
    const char* _logFolder = SD_LOG_FOLDER; 
    const char* _logBasename = SD_LOG_BASENAME;
    const uint16_t _maxLogFiles = SD_MAX_LOG_FILES; 
    
    // State variables
    bool _loggingActive;
    volatile bool _stopRequested;
    bool _HILLoggingActive;
    HILMode _currentHILMode = HILMode::NONE;
    uint16_t _decimationFactor;
    uint16_t _decimationCounter;

    // SD Card
    File _logFile;
    File _hilFile;
    String _currentSDFileName;
    bool _sdAvailable;
    bool _sdEnabled; 
    uint8_t _sdRecordCounter;
    uint8_t _sdLEDCounter;
    String  _sdBuffer;      // Memory buffer for CSV rows
    uint8_t _sdBufferCount; // Current number of records in buffer
    const uint32_t PRE_ALLOC_SIZE = 20 * 1024 * 1024; // TODO: implement pre-allocation of 20 Mb files 
    const uint32_t _sdWriteTimeoutMs = SD_WRITE_TIMEOUT_MS;

    // Pins SDIO
    const u_int8_t _pinSDIO_CLK = PIN_SDIO_CLK; 
    const u_int8_t _pinSDIO_CMD = PIN_SDIO_CMD; 
    const u_int8_t _pinSDIO_D0  = PIN_SDIO_D0; 
    const u_int8_t _pinSDIO_D1  = PIN_SDIO_D1; 
    const u_int8_t _pinSDIO_D2  = PIN_SDIO_D2; 
    const u_int8_t _pinSDIO_D3  = PIN_SDIO_D3; 
    const u_int8_t _pinSDIO_DET = PIN_SDIO_DET; 
    
    // --- Diagnostics ---
    IODiagnostics _ioDiag = {0};

    // Internal Flash (FFat)
    File _internalLogFile;
    String _currentInternalFileName;
    bool _ffatAvailable;
    bool _internalEnabled; 
    uint32_t _ffatRecordCounter; // Records since last file close
    ScaledFlightData _ffatBuffer[LOG_BUFFER_SIZE_INT]; // RAM buffer for binary records
    uint16_t _ffatBufferCount;   // Current records in RAM buffer
    
    // Flash SPI
    const u_int8_t _pinCS_Flash = PIN_FLASH_CS;
    SPIFlash *_flash;
    uint32_t _flashAddr;
    bool _flashAvailable;
    bool _externalEnabled; 

    // Stabilization variables
    bool _hilStabilizing;             // Flag to indicate stabilization state
    unsigned long _hilStartTimeMS;   
    const unsigned long _hilStabilizationDurationMS = HIL_STABILIZATION_MS; 
    HILSimulationData _staticHILFrame; // Data from the first line
   
    // Helpers
    void writeToSD(const RawFlightData& data);
    void writeToInternal(const RawFlightData& data);
    void writeToExternal(const RawFlightData& data);
    bool ensureSDConnection();
    String generateFileName();
    HILMode detectHILFormat(String headerLine);
    HILSimulationData parseHILLine(String line);
};

#endif // DATA_MANAGER_H