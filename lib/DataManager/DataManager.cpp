#include "DataManager.h"
#include <esp_task_wdt.h>

// Singleton Instance Getter
DataManager &DataManager::getInstance() {
  static DataManager instance;
  return instance;
}

// Constructor: Initializes internal members
DataManager::DataManager()
    : _loggingActive(false), _stopRequested(false), _HILLoggingActive(false),
      _decimationFactor(1), _decimationCounter(0), _sdAvailable(false),
      _sdRecordCounter(0), _sdLEDCounter(0), _sdBuffer(""), _sdBufferCount(0),
      _ffatRecordCounter(0), _ffatBuffer(nullptr), _ffatBufferCount(0), _ffatWriteIndex(0),
      _pinSDIO_DET(PIN_SDIO_DET),
      _pinCS_Flash(PIN_FLASH_CS), _flash(nullptr), _flashAddr(0),
      _flashAvailable(false), _currentHILMode(HILMode::NONE),
      _hilStabilizing(false), _ffatAvailable(false),
      _sdEnabled(ENABLE_SD_LOGGING), 
      _internalEnabled(ENABLE_INTERNAL_LOGGING), 
      _externalEnabled(ENABLE_EXTERNAL_LOGGING) {
          
      // Allocate massive PSRAM buffer
      _ffatBuffer = (ScaledFlightData*) heap_caps_malloc(LOG_BUFFER_SIZE_INT * sizeof(ScaledFlightData), MALLOC_CAP_SPIRAM);
      if (!_ffatBuffer) {
          // Fallback to minimal SRAM buffer
          _ffatBuffer = (ScaledFlightData*) malloc(100 * sizeof(ScaledFlightData));
      }
}

// --- Initialization ---

/**
 * @brief Sets up logging on the SD card by creating a new log file.
 * @details This function initializes the SD card, ensures the log directory
 * exists, and creates a new log file with a unique name. It writes the CSV
 * header to the file.
 * @return true if the setup was successful, false otherwise.
 */
bool DataManager::setupSD() {

  if (!ENABLE_DATA_LOGGING || !ENABLE_SD_LOGGING) {
    DEBUG_PRINTLN_F("SD: Logging disabled by configuration.");
    _sdAvailable = false;
    return true;
  }

  if (!ensureSDConnection()) {
    DEBUG_PRINTLN_F("SD: ERROR - Failed to mount card!");
    _sdAvailable = false;
    return false;
  }
  // SD_MMC is initialized in ensureSDConnection()

  if (!SD_MMC.exists(_logFolder))
    SD_MMC.mkdir(_logFolder);

  _currentSDFileName = generateFileName();
  DEBUG_PRINT_F("SD: Creating file: ");
  DEBUG_PRINTLN(_currentSDFileName);
  _logFile = SD_MMC.open(_currentSDFileName, FILE_WRITE);

  if (_logFile) {
    // CSV Header
    _logFile.println(
        "Time[ms],AccX[g],AccY[g],AccZ[g],GyroX[°/s],GyroY[°/s],GyroZ[°/"
        "s],MagX[uT],MagY[uT],MagZ[uT],qW,qX,qY,qZ,AltFilt[m],VelFilt[m/"
        "s],AccVert[m/"
        "s^2],Tilt[°],PressBMP[Pa],Servo[%],PID_Gain,Cd_Gain,Estado");

    if (LOG_SYNC_INTERVAL_SD <= 1) {
      _logFile.close();
      DEBUG_PRINTLN_F("SD: File created and closed (Safe Mode).");
    } else {
      _logFile.flush();
      DEBUG_PRINTLN_F("SD: File open (Batch Mode).");
    }

    _sdAvailable = true;
    return true;
  }

  _sdAvailable = false;
  return false;
}

/**
 * @brief Sets up internal FFat storage for binary logging.
 */
bool DataManager::setupInternalFlash(bool formatIfFailed) {
    if (!ENABLE_DATA_LOGGING || !ENABLE_INTERNAL_LOGGING) {
        DEBUG_PRINTLN_F("FFAT: Internal logging disabled by configuration.");
        _ffatAvailable = false;
        return true;
    }
    DEBUG_PRINTLN_F("FFAT: Initializing Internal Flash...");
    
    if (!FFat.begin(false)) {
        if (formatIfFailed) {
            DEBUG_PRINTLN_F("FFAT: Mount failed. Formatting...");
            if (FFat.format()) {
                DEBUG_PRINTLN_F("FFAT: Format success. Mounting...");
                if (!FFat.begin()) return false;
            } else {
                DEBUG_PRINTLN_F("FFAT: Format FAILED.");
                return false;
            }
        } else {
            return false;
        }
    }

    DEBUG_PRINT_F("FFAT: Total: ");
    DEBUG_PRINT(FFat.totalBytes() / 1024);
    DEBUG_PRINT_F("KB, Free: ");
    DEBUG_PRINTLN(FFat.freeBytes() / 1024);

    // Generate indexing for files
    char buf[32];
    int index = 0;
    while (index < 999) {
        snprintf(buf, sizeof(buf), "/log_%03d.bin", index);
        if (!FFat.exists(buf)) break;
        index++;
    }
    _ffatRecordCounter = index; // Store latest index
    _currentInternalFileName = String(buf);
    DEBUG_PRINT_F("FFAT: Current file: ");
    DEBUG_PRINTLN(_currentInternalFileName);

    _ffatAvailable = true;
    return true;
}

/**
 * @brief Initializes the SPI Flash chip.
 */
bool DataManager::setupFlash(bool eraseArea) {
  DEBUG_PRINTLN_F("FLASH: Initializing SPI Flash...");
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ESP32S3)
  // CRITICAL WARNING: These pins are part of the internal Flash/PSRAM bus on N16R8.
  if (_pinCS_Flash == 36 || _pinCS_Flash == 17 || _pinCS_Flash == 16) {
    DEBUG_PRINTLN_F("FLASH: CRITICAL WARNING - INTERNAL PIN CONFLICT (N16R8)!");
  }
#endif

  if (!_flash) {
    _flash = new SPIFlash(_pinCS_Flash, &SPI);
  }

  if (_flash && _flash->begin()) {
    uint32_t jedec = _flash->getJEDECID();
    DEBUG_PRINT_F("FLASH: Module Found! ID: 0x");
    DEBUG_PRINTLN(jedec, HEX);

    if (jedec == 0x000000 || jedec == 0xFFFFFF) {
      DEBUG_PRINTLN_F("FLASH: ERROR - Invalid ID. Check wiring/pins.");
      _flashAvailable = false;
      return false;
    }

    _flashAvailable = true;
    _flashAddr = 0x000000;
    if (eraseArea) {
      DEBUG_PRINTLN_F("FLASH: Erasing chip (please wait)...");
      _flash->eraseChip();
      DEBUG_PRINTLN_F("FLASH: Erase complete.");
    }
    return true;
  }
  _flashAvailable = false;
  return false;
}

// --- Logging control ---

/**
 * @brief Starts general logging.
 */
void DataManager::startLogging() {
  if (ENABLE_DATA_LOGGING) {
    _loggingActive = true;
    DEBUG_PRINTLN_F("LOG: Initialized.");
  } else {
    DEBUG_PRINTLN_F("LOG: Disabled by configuration.");
  }
}

/**
 * @brief Stops the general logging.
 */
void DataManager::stopLogging() {
  _stopRequested = true;
  DEBUG_PRINTLN_F("LOG: Stop requested. Waiting for logging loop...");

  // Wait for the logging loop to pick up the request and close safely
  uint32_t startWait = millis();
  while (_loggingActive && (millis() - startWait < 200)) {
    delay(5);
  }

  if (_loggingActive) {
    // If timed out, force close (safety net)
    DEBUG_PRINTLN_F("LOG: Timeout waiting for loop. Force closing.");
    closeSDCard();
  }

  _stopRequested = false;
  digitalWrite(PIN_LED_2, LOW);
  DEBUG_PRINTLN_F("LOG: Stopped.");
}

/**
 * @brief Sets the decimation factor for SD card logging.
 * @details This function allows adjustment of the logging frequency by setting
 * a decimation factor. A factor of 1 logs every data point, 2 logs every second
 * data point, etc.
 * @warning Setting a high decimation factor may result in loss of important
 * data.
 * @param fator The decimation factor for logging.
 */
void DataManager::setDecimationFactor(uint16_t factor) {
  _decimationFactor = (factor < 1) ? 1 : factor;
  DEBUG_PRINT_F("LOG: Decimation factor set to ");
  DEBUG_PRINTLN(_decimationFactor);
}

/**
 * @brief Stops logging and finalizes the SD card file.
 * @details This function flushes any remaining data to the SD card,
 * closes the log file, and updates the logging status.
 **/
void DataManager::closeSDCard() {
  if (_sdAvailable) {
    if (_logFile) {
      if (_sdBuffer.length() > 0) {
          _logFile.print(_sdBuffer);
          _sdBuffer = "";
          _sdBufferCount = 0;
      }
      _logFile.flush();
      _logFile.close();
    }
  }

  if (_ffatAvailable && _ffatBufferCount > _ffatWriteIndex) {
      if (!_internalLogFile) {
          _internalLogFile = FFat.open(_currentInternalFileName, FILE_APPEND);
      }
      
      if (_internalLogFile) {
          uint16_t remaining = _ffatBufferCount - _ffatWriteIndex;
          DEBUG_PRINT_F("FFAT: Flushing final ");
          DEBUG_PRINT(remaining);
          DEBUG_PRINTLN_F(" records before closing.");
          _internalLogFile.write((uint8_t*)&_ffatBuffer[_ffatWriteIndex], remaining * sizeof(ScaledFlightData));
          _ffatWriteIndex = _ffatBufferCount;
          _ioDiag.ffatBufferCount = 0;
      }
  }

  if (_internalLogFile) {
      _internalLogFile.close();
  }

  DEBUG_PRINTLN_F("LOG: SD card closed.");
  _loggingActive = false;
}

/**
 * @brief Forces a flush of all RAM buffers and closes/reopens files to ensure 
 * data persistence. Safe to call during non-critical flight phases (e.g., Apogee).
 */
void DataManager::forceSync() {
  if (!_loggingActive) return;

  // 1. Flush and sync SD Card
  if (_sdAvailable && _logFile) {
      if (_sdBuffer.length() > 0) {
          _logFile.print(_sdBuffer);
          _sdBuffer = "";
          _sdBufferCount = 0;
      }
      _logFile.flush();
      _logFile.close();
      
      // Reopen to continue logging
      _logFile = SD_MMC.open(_currentSDFileName, FILE_APPEND);
      _sdRecordCounter = 0;
  }

  // 2. Flush and sync Internal Flash
  if (_ffatAvailable && _internalLogFile) {
      if (_ffatBufferCount > 0) {
          _internalLogFile.write((uint8_t*)_ffatBuffer, _ffatBufferCount * sizeof(ScaledFlightData));
          _ffatBufferCount = 0;
      }
      _internalLogFile.close();
      
      // Reopen to continue logging
      _internalLogFile = FFat.open(_currentInternalFileName, FILE_APPEND);
      _ffatRecordCounter = 0;
  }

  DEBUG_PRINTLN_F("LOG: Force sync completed.");
}

// --- Core logging logic ---

/**
 * @brief Dispatcher for flight data logging across enabled targets.
 * @param data Flight data to log.
 */
void DataManager::logFlightData(const RawFlightData& data) {
  // Handle Stop Request synchronously
  if (_stopRequested) {
    if (_loggingActive) closeSDCard();
    return;
  }

  if (!_loggingActive || _hilStabilizing) return;

  // Global decimation check
  _decimationCounter++;
  if (_decimationCounter < _decimationFactor) {
    return; // Skip this data point
  }
  _decimationCounter = 0; // Reset and save now

  // 1. SD Card Logging
  if (_sdAvailable && _sdEnabled) {
      writeToSD(data);
  }

  // 2. Internal Flash Logging
  if (_ffatAvailable && _internalEnabled) {
      writeToInternal(data);
  }

  // 3. External SPI Flash Logging
  if (_flashAvailable && _externalEnabled) {
      writeToExternal(data);
  }
}

/**
 * @brief Logic for writing CSV data to SD card.
 */
void DataManager::writeToSD(const RawFlightData& data) {
  uint32_t startTime = millis();

  // Ensure file is open
  if (!_logFile) {
    _logFile = SD_MMC.open(_currentSDFileName, FILE_APPEND);
    if (!_logFile) {
      DEBUG_PRINTLN_F("SD: Error opening file!");
      return;
    }
  }

  // Format row to local buffer first
  char rowBuf[256];
  snprintf(rowBuf, sizeof(rowBuf), "%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%d\n",
                  data.timestamp, data.accX, data.accY, data.accZ, data.gyroX,
                  data.gyroY, data.gyroZ, data.magX, data.magY, data.magZ,
                  data.qW, data.qX, data.qY, data.qZ, data.filteredAltitude,
                  data.filteredVerticalVelocity, data.netVerticalAcceleration,
                  data.tilt, data.barometricPressure, data.airbrakeDeployment,
                  data.pid_gain, data.cd_gain, (int)data.flightState);

  _sdBuffer += rowBuf;
  _sdBufferCount++;

  // Write to SD in batches to reduce bus contention
  if (_sdBufferCount >= LOG_BUFFER_SIZE_SD) {
    uint32_t writeStart = micros();
    _logFile.print(_sdBuffer);
    _ioDiag.sdWrite_us = micros() - writeStart;
    
    // Low-pass filter for average (Alpha = 0.1)
    if (_ioDiag.sdWriteAvg_us == 0) _ioDiag.sdWriteAvg_us = _ioDiag.sdWrite_us;
    else _ioDiag.sdWriteAvg_us = (_ioDiag.sdWriteAvg_us * 9 + _ioDiag.sdWrite_us) / 10;

    if (_ioDiag.sdWrite_us > _ioDiag.maxSdWrite_us) {
      _ioDiag.maxSdWrite_us = _ioDiag.sdWrite_us;
    }

    _sdBuffer = "";
    _sdBufferCount = 0;
    
    // Increment sync record counter only on actual writes
    _sdRecordCounter += LOG_BUFFER_SIZE_SD; 
  }

  // Periodic Sync/Close for stability
  if (_sdRecordCounter >= LOG_SYNC_INTERVAL_SD) {
    _logFile.close();

    _sdLEDCounter++;
    if (_sdLEDCounter >= 1) {
      _sdLEDCounter = 0;
      digitalWrite(PIN_LED_2, !digitalRead(PIN_LED_2));
    }

    // static uint32_t lastSaveTime = 0;
    // uint32_t now = millis();
    // DEBUG_PRINT_F("SD: Data saved. dt: ");  DEBUG_PRINT(now - lastSaveTime);
    // DEBUG_PRINTLN_F(" ms"); lastSaveTime = now;
    _sdRecordCounter = 0;
  }

  // Check for Timeout
  uint32_t duration = millis() - startTime;
  if (duration > _sdWriteTimeoutMs) {
    DEBUG_PRINT_F("SD: Write Timeout! Duration: ");
    DEBUG_PRINT(duration);
    DEBUG_PRINTLN_F(" ms");
    DEBUG_PRINTLN_F("SD: Disabling Logging for Safety.");

    if (_logFile) _logFile.close();
    // We don't disable global logging, just the SD target
    _sdAvailable = false; 
    _sdEnabled = false;
  }
}

/**
 * @brief Logic for writing binary data to Internal Flash (FFat).
 */
void DataManager::writeToInternal(const RawFlightData& data) {
    ScaledFlightData record;
    record.timestamp_ms = data.timestamp;
    
    record.accX_scaled = (int16_t)(data.accX * 1000.0f);
    record.accY_scaled = (int16_t)(data.accY * 1000.0f);
    record.accZ_scaled = (int16_t)(data.accZ * 1000.0f);
    
    record.gyroX_scaled = (int16_t)(data.gyroX * 100.0f);
    record.gyroY_scaled = (int16_t)(data.gyroY * 100.0f);
    record.gyroZ_scaled = (int16_t)(data.gyroZ * 100.0f);
    
    record.magX_scaled = (int16_t)(data.magX * 10.0f);
    record.magY_scaled = (int16_t)(data.magY * 10.0f);
    record.magZ_scaled = (int16_t)(data.magZ * 10.0f);
    
    record.qW_scaled = (int16_t)(data.qW * 10000.0f);
    record.qX_scaled = (int16_t)(data.qX * 10000.0f);
    record.qY_scaled = (int16_t)(data.qY * 10000.0f);
    record.qZ_scaled = (int16_t)(data.qZ * 10000.0f);
    
    record.altitude_scaled = (int16_t)(data.filteredAltitude * 10.0f);
    record.velocity_scaled = (int16_t)(data.filteredVerticalVelocity * 100.0f);
    record.accel_net_scaled = (int16_t)(data.netVerticalAcceleration * 100.0f);
    record.tilt_scaled = (int16_t)(data.tilt * 100.0f);
    
    record.pressure_scaled = (uint32_t)data.barometricPressure;
    record.airbrake_scaled = (uint8_t)data.airbrakeDeployment;
    record.gain1_scaled = (int16_t)(data.pid_gain * 100.0f);
    record.gain2_scaled = (int16_t)(data.cd_gain * 100.0f);
    record.flightState = data.flightState;

    // FlightState enum mapping: WAIT_LAUNCH is 2 or less, DESCENT is 7, LANDING is 8
    bool isPad = (data.flightState <= 2);
    bool isDescent = (data.flightState == 7 || data.flightState == 8);
    bool isAscent = (!isPad && !isDescent);

    if (isPad) {
        // Pad Phase: Buffer small amounts and flush to flash regularly
        if (_ffatBuffer && _ffatBufferCount < LOG_BUFFER_SIZE_INT) {
            _ffatBuffer[_ffatBufferCount] = record;
            _ffatBufferCount++;
            _ioDiag.ffatBufferCount = _ffatBufferCount;
        }

        if (_ffatBufferCount >= LOG_PAD_FLUSH_SIZE) {
            if (!_internalLogFile) {
                _internalLogFile = FFat.open(_currentInternalFileName, FILE_APPEND);
            }
            if (_internalLogFile) {
                uint32_t writeStart = micros();
                _internalLogFile.write((const uint8_t*)_ffatBuffer, _ffatBufferCount * sizeof(ScaledFlightData));
                _ioDiag.internalFlashWrite_us = micros() - writeStart;
                
                if (_ioDiag.internalFlashWriteAvg_us == 0) _ioDiag.internalFlashWriteAvg_us = _ioDiag.internalFlashWrite_us;
                else _ioDiag.internalFlashWriteAvg_us = (_ioDiag.internalFlashWriteAvg_us * 9 + _ioDiag.internalFlashWrite_us) / 10;

                if (_ioDiag.internalFlashWrite_us > _ioDiag.maxInternalFlashWrite_us) {
                    _ioDiag.maxInternalFlashWrite_us = _ioDiag.internalFlashWrite_us;
                }
                
                // Because we flushed from the beginning of RAM, reset buffer count
                _ffatBufferCount = 0;
                _ioDiag.ffatBufferCount = 0;
                
                // Periodic sync
                _ffatRecordCounter += LOG_PAD_FLUSH_SIZE;
                if (_ffatRecordCounter >= LOG_SYNC_INTERVAL_INT) {
                    _internalLogFile.close();
                    _ffatRecordCounter = 0;
                }
            }
        }
    } else if (isAscent) {
        // Ascent Phase: Massive PSRAM buffer only, NO flash writes!
        if (_ffatBuffer && _ffatBufferCount < LOG_BUFFER_SIZE_INT) {
            _ffatBuffer[_ffatBufferCount] = record;
            _ffatBufferCount++;
            _ioDiag.ffatBufferCount = _ffatBufferCount;
        }
    } else if (isDescent) {
        // Descent Phase: Stop accumulating. Trickle dump the accumulated PSRAM buffer to Flash.
        if (_ffatWriteIndex < _ffatBufferCount) {
            if (!_internalLogFile) {
                _internalLogFile = FFat.open(_currentInternalFileName, FILE_APPEND);
            }

            if (_internalLogFile) {
                uint32_t writeStart = micros();
                
                uint16_t recordsRemaining = _ffatBufferCount - _ffatWriteIndex;
                uint16_t chunkSize = (recordsRemaining > LOG_CHUNK_SIZE_DESCENT) ? LOG_CHUNK_SIZE_DESCENT : recordsRemaining;
                
                _internalLogFile.write((const uint8_t*)&_ffatBuffer[_ffatWriteIndex], chunkSize * sizeof(ScaledFlightData));
                
                _ioDiag.internalFlashWrite_us = micros() - writeStart;
                
                if (_ioDiag.internalFlashWriteAvg_us == 0) _ioDiag.internalFlashWriteAvg_us = _ioDiag.internalFlashWrite_us;
                else _ioDiag.internalFlashWriteAvg_us = (_ioDiag.internalFlashWriteAvg_us * 9 + _ioDiag.internalFlashWrite_us) / 10;

                if (_ioDiag.internalFlashWrite_us > _ioDiag.maxInternalFlashWrite_us) {
                    _ioDiag.maxInternalFlashWrite_us = _ioDiag.internalFlashWrite_us;
                }

                _ffatWriteIndex += chunkSize;
                _ioDiag.ffatBufferCount = _ffatBufferCount - _ffatWriteIndex; // Update UI to show remaining to flush
                
                if (_ffatWriteIndex >= _ffatBufferCount) {
                    _internalLogFile.close(); // Sync when all data is finally dumped
                }
            }
        }
    }
}

/**
 * @brief Logic for writing data to External SPI Flash.
 * @note Implementation pending hardware-specific SPIFlash library calls.
 */
void DataManager::writeToExternal(const RawFlightData& data) {
    if (!_flashAvailable) return;
    // TODO: Implement binary serialization for external flash
}

/**
 * @brief Lists all files in the internal flash memory.
 */
void DataManager::listInternalFiles() {
  if (!_ffatAvailable) {
    DEBUG_PRINTLN_F("FFAT: Internal flash not available.");
    return;
  }
  DEBUG_PRINTLN_F("\n--- Internal Flash Files ---");
  File root = FFat.open("/");
  File file = root.openNextFile();
  while (file) {
    DEBUG_PRINT_F("File: ");
    DEBUG_PRINT(file.name());
    DEBUG_PRINT_F("  Size: ");
    DEBUG_PRINTLN(file.size());
    file = root.openNextFile();
    
    // Safety
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  DEBUG_PRINT_F("FFAT Free: ");
  DEBUG_PRINTLN(FFat.freeBytes() / 1024);
}

void DataManager::dumpInternalFlash(int fileIndex) {
  if (!_ffatAvailable) return;
  
  if (fileIndex == -1) {
    fileIndex = getLatestInternalLogIndex();
    if (fileIndex < 0) {
      DEBUG_PRINTLN_F("FFAT: No logs found to dump.");
      return;
    }
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "/log_%03d.bin", fileIndex);
  
  File f = FFat.open(buf, FILE_READ);
  if (!f) {
    DEBUG_PRINTLN_F("FFAT: File not found.");
    return;
  }

  DEBUG_PRINT_F("--- DECODING INTERNAL LOG: ");
  DEBUG_PRINTLN(buf);
  
  // Print CSV Header for the decoded data
  DEBUG_PRINTLN_F("timestamp_ms,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,qW,qX,qY,qZ,alt,vel,acc_net,tilt,pres,brake,state");

  ScaledFlightData record;
  uint32_t recordCount = 0;
  while (f.read((uint8_t*)&record, sizeof(record)) == sizeof(record)) {
    Serial.printf("%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.1f,%.2f,%.2f,%.2f,%u,%u,%d\n",
        record.timestamp_ms,
        record.accX_scaled / 1000.0f, record.accY_scaled / 1000.0f, record.accZ_scaled / 1000.0f,
        record.gyroX_scaled / 100.0f, record.gyroY_scaled / 100.0f, record.gyroZ_scaled / 100.0f,
        record.magX_scaled / 10.0f, record.magY_scaled / 10.0f, record.magZ_scaled / 10.0f,
        record.qW_scaled / 10000.0f, record.qX_scaled / 10000.0f, record.qY_scaled / 10000.0f, record.qZ_scaled / 10000.0f,
        record.altitude_scaled / 10.0f,
        record.velocity_scaled / 100.0f,
        record.accel_net_scaled / 100.0f,
        record.tilt_scaled / 100.0f,
        record.pressure_scaled,
        (uint16_t)record.airbrake_scaled, // Print as number
        record.flightState
    );

    // Feed the watchdog to prevent reset during large files
    if (++recordCount % 20 == 0) {
        esp_task_wdt_reset();
        vTaskDelay(1); // Yield to other tasks
    }
  }
  DEBUG_PRINTLN_F("--- END OF LOG ---");
  f.close();
}

void DataManager::clearInternalFlash() {
    DEBUG_PRINTLN_F("FFAT: Formatting internal flash...");
    if (FFat.format()) {
        DEBUG_PRINTLN_F("FFAT: Format complete.");
    } else {
        DEBUG_PRINTLN_F("FFAT: Format FAILED.");
    }
}

// --- HIL Simulation ---

/**
 * @brief Helper to detect CSV format based on column count (commas).
 * @param headerLine Header line of the HIL .csv file.
 */
HILMode DataManager::detectHILFormat(String headerLine) {
  int commaCount = 0;
  for (char c : headerLine) {
    if (c == ',')
      commaCount++;
  }

  // Simple Mode: time, pressure, acc_net, tilt (3 commas)
  if (commaCount == 3) {
    DEBUG_PRINTLN_F("HIL: Detected SIMPLE format (4 cols).");
    return HILMode::SIMPLE;
  }

  // Full Mode: time, acc(3), gyro(3), mag(3), press (10 commas)
  if (commaCount >= 10) {
    DEBUG_PRINTLN_F("HIL: Detected FULL SENSOR format (11+ cols).");
    return HILMode::FULL;
  }

  DEBUG_PRINTLN_F("HIL: Unknown format.");
  return HILMode::NONE;
}

/**
 * @brief Parses a single line from the HIL file.
 * @param line The line to parse.
 * @return A HILSimulationData struct containing the parsed data.
 */
HILSimulationData DataManager::parseHILLine(String line) {
  HILSimulationData data;
  data.valid = false;
  line.trim();
  if (line.length() == 0)
    return data;

  // Format: time, pressure, net_accel, tilt
  if (_currentHILMode == HILMode::SIMPLE) {
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);

    if (p1 != -1 && p2 != -1 && p3 != -1) {
      data.time_s = line.substring(0, p1).toFloat();
      data.barometricPressure_Pa = line.substring(p1 + 1, p2).toFloat();
      data.netVerticalAcceleration_ms2 = line.substring(p2 + 1, p3).toFloat();
      data.tilt = line.substring(p3 + 1).toFloat();

      data.hasFullIMU = false;
      data.valid = true;
    }
  }

  // Format: time, ax, ay, az, gx, gy, gz, mx, my, mz, pressure
  else if (_currentHILMode == HILMode::FULL) {

    float values[11];
    int currentIdx = 0;
    int pStart = 0;
    int pEnd = 0;

    for (int i = 0; i < 11; i++) {
      pEnd = line.indexOf(',', pStart);
      if (pEnd == -1)
        pEnd = line.length();

      String val = line.substring(pStart, pEnd);
      values[i] = val.toFloat();

      pStart = pEnd + 1;
      if (pStart > line.length())
        break; // Safety break
    }

    data.time_s = values[0];

    // IMU Data
    data.accX_ms2 = values[1];
    data.accY_ms2 = values[2];
    data.accZ_ms2 = values[3];
    data.gyroX_rads = values[4];
    data.gyroY_rads = values[5];
    data.gyroZ_rads = values[6];
    data.magX_T = values[7];
    data.magY_T = values[8];
    data.magZ_T = values[9];

    // Baro
    data.barometricPressure_Pa = values[10];

    data.hasFullIMU = true;
    data.valid = true;
  }

  return data;
}

/**
 * @brief Initiates a Hardware-in-the-Loop (HIL) simulation by loading data from
 * a specified file on the SD card.
 * @details This function checks for the existence of the specified file on the
 * SD card, attempts to open it, and prepares it for reading simulation data. It
 * handles various scenarios, including missing files and read errors.
 * @param filename The name of the file on the SD card containing HIL simulation
 * data
 **/
bool DataManager::initHIL(const char *filename) {
  if (!_sdAvailable) {
    if (!ensureSDConnection()) {
      DEBUG_PRINTLN_F("HIL ERROR: Failed to connect to SD.");
      return false;
    }
  }
  String HILFilePath = "";
  // Tries to open the file as given
  if (SD_MMC.exists(filename)) {
    HILFilePath = String(filename);
    DEBUG_PRINT_F("HIL: File found on root: ");
    DEBUG_PRINTLN(HILFilePath);
  }
  // Tries to open the file with leading slash
  else if (SD_MMC.exists("/" + String(filename))) {
    HILFilePath = "/" + String(filename);
    DEBUG_PRINT_F("HIL: File found (with leading slash): ");
    DEBUG_PRINTLN(HILFilePath);
  }
  // Tries inside the logs folder
  else {
    String pathTemp = String(_logFolder) + "/" + String(filename);
    if (SD_MMC.exists(pathTemp)) {
      HILFilePath = pathTemp;
      DEBUG_PRINT_F("HIL: File found in logs folder: ");
      DEBUG_PRINTLN(HILFilePath);
    } else {
      DEBUG_PRINT_F("HIL ERROR: File NOT found: ");
      DEBUG_PRINTLN(filename);
      return false;
    }
  }

  _hilFile = SD_MMC.open(filename, FILE_READ);
  if (_hilFile) {
    DEBUG_PRINTLN_F("HIL: File opened successfully.");
    if (_hilFile.available()) {
      String header = _hilFile.readStringUntil('\n');
      _currentHILMode = detectHILFormat(header);

      DEBUG_PRINT_F("HIL: Read header: ");
      DEBUG_PRINTLN(header.substring(0, 60) + "...");

      if (_currentHILMode == HILMode::NONE) {
        _hilFile.close();
        return false;
      }

      String firstLineStr = _hilFile.readStringUntil('\n');
      _staticHILFrame = parseHILLine(firstLineStr);

      if (!_staticHILFrame.valid) {
        DEBUG_PRINTLN_F("HIL ERROR: Empty file or invalid first line.");
        _hilFile.close();
        return false;
      }

      // --- Orientation Guard ---
      if (_staticHILFrame.hasFullIMU) {
        bool csvSeemsZDown = (_staticHILFrame.accZ_ms2 < -5.0f);
        if (csvSeemsZDown != PHYSICAL_Z_AXIS_DOWN) {
          DEBUG_PRINTF(
              "[HIL WARNING] Orientation mismatch! CSV:%s | Config:%s\n",
              csvSeemsZDown ? "Z-DOWN" : "Z-UP",
              PHYSICAL_Z_AXIS_DOWN ? "Z-DOWN" : "Z-UP");
        }
      }

      // Forces gyro to zero to avoid drift
      _staticHILFrame.gyroX_rads = 0.0f;
      _staticHILFrame.gyroY_rads = 0.0f;
      _staticHILFrame.gyroZ_rads = 0.0f;

      _hilStabilizing = true;
      _hilStartTimeMS = millis();

      // Returns to the start
      _hilFile.seek(0);
      _hilFile.readStringUntil('\n');

      DEBUG_PRINT_F("HIL: Initialized. Entering stabilization period: ");
      DEBUG_PRINT(_hilStabilizationDurationMS);
      DEBUG_PRINTLN_F("ms.");
      return true;

    } else {
      DEBUG_PRINTLN_F("HIL: Warning - File is empty.");
      _hilFile.close();
      return false;
    }
  } else {
    DEBUG_PRINTLN_F("HIL: Error - SD.open failed.");
    return false;
  }
}

/**
 * @brief Reads the next simulation step from the HIL simulation file.
 * @details This function reads the next line from the currently active HIL
 * simulation file on the SD card, parses the data, and returns it in a
 * HILSimulationData struct. If the simulation is not active or the file is not
 * available, it returns a struct with valid set to false.
 * @return A HILSimulationData struct containing the parsed simulation data.
 **/
HILSimulationData DataManager::readHILStep() {
  // Stabilization phase
  if (_hilStabilizing) {
    unsigned long elapsed = millis() - _hilStartTimeMS;

    if (elapsed < _hilStabilizationDurationMS) {
      HILSimulationData frame = _staticHILFrame;

      // Artificial negative time
      float progress = (float)elapsed / 1000.0f;
      frame.time_s = -((float)_hilStabilizationDurationMS / 1000.0f) + progress;

      return frame;
    } else {
      _hilStabilizing = false;
      DEBUG_PRINTLN("HIL: End of stabilization. Reading CSV data.");
    }
  }

  if (!_hilFile || !_hilFile.available()) {
    HILSimulationData data;
    data.valid = false;
    return data;
  }

  String line = _hilFile.readStringUntil('\n');
  return parseHILLine(line);
}

/**
 * @brief Resets the HIL simulation file to the beginning for a new run.
 */
void DataManager::resetHIL() {
  if (_hilFile) {
    _hilFile.seek(0);
    _hilFile.readStringUntil('\n');

    _hilStabilizing = true;
    _hilStartTimeMS = millis();
  }
  DEBUG_PRINTLN_F("HIL: File reset.");
}

/**
 * @brief Stops the HIL simulation and closes the associated file.
 */
void DataManager::stopHIL() {
  if (_hilFile)
    _hilFile.close();
}

// --- Auxilary tools ---

/**
 * @brief Ensures the SD card is connected and mounted.
 * @return true if the SD card is accessible, false otherwise.
 */
bool DataManager::ensureSDConnection() {

  /*
  pinMode(_pinCS_Flash, OUTPUT);
  digitalWrite(_pinCS_Flash, HIGH);
  */

  delay(200); // Give some time for power to stabilize

  if (SD_MMC.cardType() != CARD_NONE)
    return true;

  static bool sdmmcInitialized = false;
  if (!sdmmcInitialized) {
    DEBUG_PRINTLN_F("SD: Configuring pins and pull-ups...");

    // Manual pull-up configuration to help if external resistors are
    // missing/weak SD_MMC requirements: CMD and D0-D3 should have pull-ups.
    pinMode(_pinSDIO_CMD, INPUT_PULLUP);
    pinMode(_pinSDIO_D0, INPUT_PULLUP);
    pinMode(_pinSDIO_D1, INPUT_PULLUP);
    pinMode(_pinSDIO_D2, INPUT_PULLUP);
    pinMode(_pinSDIO_D3, INPUT_PULLUP);

    // ESP32-S3 SD_MMC.setPins(clk, cmd, d0, d1, d2, d3)
    if (!SD_MMC.setPins(_pinSDIO_CLK, _pinSDIO_CMD, _pinSDIO_D0, _pinSDIO_D1,
                        _pinSDIO_D2, _pinSDIO_D3)) {
      DEBUG_PRINTLN_F("SD_MMC: Failed to set pins!");
      return false;
    }

    // --- Multi-stage Initialization Strategy ---
    delay(500); // Give the card time to internally reset after power-on
    DEBUG_PRINTLN_F("SD: Starting mount sequence...");

    // // 1. Attempt 4-bit at 20MHz (Standard Performance)
    // DEBUG_PRINTLN_F("SD: Attempting 4-bit, 8MHz...");
    // if (SD_MMC.begin("/sdcard", false, false, 8000)) {
    //   DEBUG_PRINTLN_F("SD: Mounted successfully (4-bit, 8MHz).");
    // }

    // }


    // 2. Fallback to 1-bit at 20MHz (Less pins, more stable)
    DEBUG_PRINTLN_F("SD: Attempting 1-bit, 20MHz...");
    if (SD_MMC.begin("/sdcard", true, false, 20000)) {
      DEBUG_PRINTLN_F("SD: Mounted successfully (1-bit, 20MHz).");
      sdmmcInitialized = true;
      return true;
    }

    // 3. Last resort: 1-bit at 10MHz (Maximum stability)
    DEBUG_PRINTLN_F("SD: Last resort attempting 1-bit, 10MHz...");
    if (SD_MMC.begin("/sdcard", true, false, 10000)) {
      DEBUG_PRINTLN_F("SD: Mounted successfully (1-bit, 10MHz).");
      sdmmcInitialized = true;
      return true;
    }

    DEBUG_PRINTLN_F("SD: All mount attempts failed.");
    return false;
  }
  return true;
}


/**
 * @brief Generates the next available filename for logging on the SD card.
 * @details This function checks for existing files in the log directory and
 * generates a new filename by incrementing a counter until an unused name is
 * found.
 * @return A String containing the next available filename.
 */
String DataManager::generateFileName() {
  char buf[64];
  for (int i = 0; i < _maxLogFiles; i++) {
    snprintf(buf, sizeof(buf), "%s/%s%03d.csv", _logFolder, _logBasename, i);
    if (!SD_MMC.exists(buf))
      return String(buf);
  }
  snprintf(buf, sizeof(buf), "%s/%s%lu.%s", _logFolder, _logBasename,
           (unsigned long)millis(), "csv");
  return String(buf);
}

/**
 * @brief Dumps the current log file from the SD card to the Serial console.
 * @details This function reads the current log file stored on the SD card
 * and outputs its contents to the Serial console. It is useful for retrieving
 * log data without removing the SD card.
 **/
void DataManager::dumpSDCurrentLog() {
  if (_currentSDFileName == "") {
    DEBUG_PRINTLN_F("No file name registered to read.");
    return;
  }

  if (!ensureSDConnection()) {
    DEBUG_PRINTLN_F("Error: SD card not accessible.");
    return;
  }
  DEBUG_PRINT_F("\n--- Reading file: ");
  DEBUG_PRINTLN(_currentSDFileName);
  DEBUG_PRINTLN_F("---");
  File f = SD_MMC.open(_currentSDFileName);

  if (!f) {
    DEBUG_PRINTLN_F("Error opening file for reading.");
    return;
  }

  uint32_t byteCount = 0;

  while (f.available()) {
    uint8_t b = f.read();

    if (SERIAL_CHECK)
      Serial.write(b);
    if (++byteCount % 512 == 0) {
      esp_task_wdt_reset();
    }
  }
  f.close();
  DEBUG_PRINTLN_F("\n==========================================");
  DEBUG_PRINTLN_F("--- END OF FILE ---");
}

/**
 * @brief Lists all log files stored on the _logFolder (/REG_VOO) SD card.
 * @details This function accesses the SD card and lists all files in the
 * specified log directory. It prints the filenames and their sizes in KB to the
 * Serial console.
 **/
void DataManager::listSDFiles() {
  if (!ensureSDConnection()) {
    DEBUG_PRINTLN_F("Error: SD card not accessible.");
    return;
  }

  DEBUG_PRINT_F("\n--- List of files in ");
  DEBUG_PRINT(_logFolder);
  DEBUG_PRINTLN_F(" ---");
  File root = SD_MMC.open(_logFolder);
  if (!root || !root.isDirectory()) {
    DEBUG_PRINTLN_F("Log folder empty or non-existent.");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      DEBUG_PRINT_F("  ");
      DEBUG_PRINT(file.name());
      DEBUG_PRINT_F("\tSize: ");
      DEBUG_PRINT(file.size() / 1024.0);
      DEBUG_PRINTLN_F(" KB");
    }
    file = root.openNextFile();
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  DEBUG_PRINTLN_F("--------------------------------");
}

/**
 * @brief Deletes all log files stored on the SD card in the _logFolder
 * directory.
 * @details This function accesses the SD card and deletes all files in the
 * specified log directory. It prompts the user for confirmation before
 * proceeding with the deletion.
 **/
void DataManager::clearSDAllLogs() {
  if (!ensureSDConnection()) {
    DEBUG_PRINTLN_F("SD: Error connecting.");
    return;
  }

  DEBUG_PRINTLN_F("\n!!! Warning: Deleting files in 3 seconds !!!");
  delay(3000); // Delay to delete files

  File root = SD_MMC.open(_logFolder);
  if (!root || !root.isDirectory()) {
    DEBUG_PRINTLN_F("Log folder empty or non-existent.");
    return;
  }

  DEBUG_PRINTLN_F("Clearing logs...");

  File file = root.openNextFile();
  while (file) {
    String path = "";
    String nome = String(file.name());
    if (nome.startsWith("/")) {
      path = nome;
    } else {
      path = String(_logFolder) + "/" + nome;
    }

    file.close();

    DEBUG_PRINT_F("Deleting: ");
    DEBUG_PRINT(path);

    if (SD_MMC.remove(path)) {
      DEBUG_PRINTLN_F(" [OK]");
    } else {
      DEBUG_PRINTLN_F(" [FAILED]");
    }

    file = root.openNextFile();
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  root.close();

  DEBUG_PRINTLN_F("Clearing complete.");

  _sdAvailable = false;
  _currentSDFileName = "";
}

/**
 * @brief Receives a HIL simulation file via Serial and saves it to the SD card.
 * @details This function prompts the user to send a CSV file via Serial input.
 * It reads the incoming data, stores it in a buffer, and saves it to a
 * specified file on the SD card. The function handles timeouts and notifies the
 * user of success or errors.
 * @param HILFileName The name of the file to save the received HIL simulation
 * data.
 **/
void DataManager::receiveHILFile(const char *HILFileName) {
  if (!Serial)
    return; // Magic Bullet: Exit if no host listening
  if (!ensureSDConnection()) {
    DEBUG_PRINTLN_F("Error: SD not found.");
    return;
  }

  // Reserve buffer (200KB) *Change later to something configurable*
  String bufferRAM;
  if (!bufferRAM.reserve(200000)) {
    DEBUG_PRINTLN_F("Warning: Low memory.");
  }

  DEBUG_PRINTLN_F("\n==================================================");
  DEBUG_PRINTLN_F("FILE UPLOAD MODE TO RECEIVE HIL SIMULATION DATA");
  DEBUG_PRINTLN_F("==================================================");
  DEBUG_PRINTLN_F("1. Copy the CSV content.");
  DEBUG_PRINTLN_F("2. Paste ALL and send.");
  DEBUG_PRINTLN_F("3. Type 'END' and send.");
  DEBUG_PRINTLN_F("--- READY ---");

  bool recebendo = true;
  unsigned long ultimaAtividade = millis();

  while (recebendo) {
    if (Serial.available()) {
      char c = Serial.read();
      bufferRAM += c;
      ultimaAtividade = millis();

      if (bufferRAM.length() > 200000) {
        if (Serial)
          Serial.println("\nError: Buffer full!");
        recebendo = false;
      }
      // Feed watchdog during large serial uploads
      esp_task_wdt_reset();
    }

    // Check for END
    if (bufferRAM.endsWith("END") || bufferRAM.endsWith("END\n") ||
        bufferRAM.endsWith("END\r\n")) {
      recebendo = false;
      int posFim = bufferRAM.lastIndexOf("END");
      if (posFim != -1)
        bufferRAM = bufferRAM.substring(0, posFim);
      if (Serial)
        Serial.println("\nEND command detected.");
    }

    if (millis() - ultimaAtividade > 10000) { // Timeout of 10 seconds
      if (Serial)
        Serial.println("\nTimeout. Saving...");
      recebendo = false;
    }
  }
  esp_task_wdt_reset();

  if (bufferRAM.indexOf('\n') == -1) {
    DEBUG_PRINTLN_F(
        "Warning: Single-line text detected. Repairing formatting...");

    int mudancas = 0;
    int len = bufferRAM.length();

    for (int i = 1; i < len - 1; i++) {
      if (bufferRAM[i] == ' ') {
        char prev = bufferRAM[i - 1];
        char next = bufferRAM[i + 1];

        // If previous char is a digit AND next char is digit or negative sign
        if (isDigit(prev) && (isDigit(next) || next == '-')) {
          bufferRAM[i] = '\n'; // Replace space with newline
          mudancas++;
        }
      }
    }
    DEBUG_PRINT_F("Break lines added: ");
    DEBUG_PRINTLN(mudancas);
  }

  // Gravação
  SD_MMC.remove(HILFileName);
  File fileHIL = SD_MMC.open(HILFileName, FILE_WRITE);
  if (fileHIL) {
    fileHIL.print(bufferRAM);
    fileHIL.flush();
    fileHIL.close();
    DEBUG_PRINTLN_F("SUCCESS: File saved to SD.");
  } else {
    DEBUG_PRINTLN_F("ERROR: Failed to save file.");
  }

  bufferRAM = "";
  DEBUG_PRINTLN_F("==================================================");
}

/**
 * @brief Runs a frequency test on the SD card.
 * @param freq The frequency to test.
 * @param flushLimit The number of records to flush.
 * @param numberOfRecords The number of records to test.
 * @param onlyPrintf Whether to only print the data.
 * @param oneBitMode Whether to use 1-bit mode.
 */
void DataManager::runFrequencyTest(uint32_t freq, u_int16_t flushLimit,
                                   u_int16_t numberOfRecords, bool onlyPrintf, bool oneBitMode) {
  if (!Serial)
    return; // Exit if no host
  RawFlightData data = {123456, 1.1,  -2.2,     9.8, 0.1,  0.2,  0.3,   15.0,
                        20.0,   25.0, 1.0,      0.0, 0.0,  0.0,  100.5, 50.2,
                        1.5,    5.0,  101325.0, 45,  0.02, 0.07, 2};
  DEBUG_PRINTLN_F("\n=======================================");
  DEBUG_PRINTF("STABILITY TEST AT %d MHz (%s mode)\n", freq / 1000000, oneBitMode ? "1-bit" : "4-bit");

  // Fallback: SD_MMC uses setPins already, but frequency test might want to
  if (!SD_MMC.begin("/sdcard", oneBitMode, false, freq / 1000)) {
    DEBUG_PRINTLN_F("SD_MMC Initialization Failed!");
    return;
  }

  auto benchmark = [&](bool usePrintf, const char *filename) {
    File file = SD_MMC.open(filename, FILE_WRITE);
    uint32_t totalWriteUs = 0;
    uint32_t totalFlushUs = 0;
    int flushCount = 0;

    for (int i = 1; i <= numberOfRecords; i++) {
      uint32_t startWrite = micros();
      if (usePrintf) {
        file.printf("%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,"
                    "%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%d\n",
                    data.timestamp, data.accX, data.accY, data.accZ, data.gyroX,
                    data.gyroY, data.gyroZ, data.magX, data.magY, data.magZ,
                    data.qW, data.qX, data.qY, data.qZ, data.filteredAltitude,
                    data.filteredVerticalVelocity, data.netVerticalAcceleration,
                    data.tilt, data.barometricPressure, data.airbrakeDeployment,
                    data.pid_gain, data.cd_gain, data.flightState);
      } else {
        file.print(data.timestamp);
        file.print(',');
        file.print(data.accX, 3);
        file.print(',');
        file.print(data.accY, 3);
        file.print(',');
        file.print(data.accZ, 3);
        file.print(',');
        file.print(data.gyroX, 3);
        file.print(',');
        file.print(data.gyroY, 3);
        file.print(',');
        file.print(data.gyroZ, 3);
        file.print(',');
        file.print(data.magX, 3);
        file.print(',');
        file.print(data.magY, 3);
        file.print(',');
        file.print(data.magZ, 3);
        file.print(',');
        file.print(data.qW, 4);
        file.print(',');
        file.print(data.qX, 4);
        file.print(',');
        file.print(data.qY, 4);
        file.print(',');
        file.print(data.qZ, 4);
        file.print(',');
        file.print(data.filteredAltitude, 2);
        file.print(',');
        file.print(data.filteredVerticalVelocity, 2);
        file.print(',');
        file.print(data.netVerticalAcceleration, 2);
        file.print(',');
        file.print(data.tilt, 2);
        file.print(',');
        file.print(data.barometricPressure, 1);
        file.print(',');
        file.print(data.airbrakeDeployment);
        file.print(',');
        file.print(data.pid_gain, 2);
        file.print(',');
        file.print(data.cd_gain, 2);
        file.print(',');
        file.println(data.flightState);
      }
      totalWriteUs += (micros() - startWrite);

      if (i % flushLimit == 0) {
        uint32_t startFlush = micros();
        file.flush();
        totalFlushUs += (micros() - startFlush);
        flushCount++;
      }
    }
    file.close();
    SD_MMC.remove(filename);

    DEBUG_PRINTF("[%s]\n", usePrintf ? "PRINTF" : "PRINTS");
    DEBUG_PRINTF("Avg Write: %.3f ms\n",
                 (totalWriteUs / float(numberOfRecords)) / 1000.0);
    DEBUG_PRINTF("Avg Flush: %.3f ms\n",
                 (totalFlushUs / (float)flushCount) / 1000.0);
    DEBUG_PRINTF("Worst Case (Write+Flush): %.3f ms\n",
                 ((totalWriteUs / float(numberOfRecords)) +
                  (totalFlushUs / (float)flushCount)) /
                     1000.0);
  };

  if (onlyPrintf) {
    benchmark(true, "/test_printf.csv");
  } else {
    benchmark(false, "/test1.csv");
    benchmark(true, "/test2.csv");
  }
}

/**
 * @brief Runs a strategy benchmark on the SD card.
 * @param freq The frequency to test.
 * @param numberOfRecords The number of records to test.
 */
void DataManager::runStrategyBenchmark(uint32_t freq,u_int16_t numberOfRecords) {
  if (!Serial)
    return; // Exit if no host listening
  RawFlightData data = {123456, 1.1,  -2.2,     9.8, 0.1,  0.2,  0.3,   15.0,
                        20.0,   25.0, 1.0,      0.0, 0.0,  0.0,  100.5, 50.2,
                        1.5,    5.0,  101325.0, 45,  0.02, 0.07, 2};
  DEBUG_PRINTLN_F("\n=======================================");
  DEBUG_PRINTF("LOGGING STRATEGY BENCHMARK (%d Records)\n", numberOfRecords);

  if (!SD_MMC.begin("/sdcard", false, false, freq / 1000)) {
    DEBUG_PRINTLN_F("SD_MMC Initialization Failed!");
    return;
  }

  auto runScenario = [&](const char *name, int flushInterval,
                         int closeInterval) -> float {
    char filename[32];
    snprintf(filename, 32, "/bench_%d_%d.csv", flushInterval, closeInterval);

    uint32_t totalTimeUs = 0;
    File file; // Keep scope if needed
    bool fileIsOpen = false;

    // Ensure clean start
    if (SD_MMC.exists(filename))
      SD_MMC.remove(filename);

    if (closeInterval == 0) {
      file = SD_MMC.open(filename, FILE_WRITE);
      fileIsOpen = true;
    }

    for (int i = 1; i <= numberOfRecords; i++) {
      uint32_t start = micros();

      if (!fileIsOpen) {
        file = SD_MMC.open(filename, FILE_APPEND);
        fileIsOpen = true;
      }

      file.printf("%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%."
                  "4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%d\n",
                  data.timestamp, data.accX, data.accY, data.accZ, data.gyroX,
                  data.gyroY, data.gyroZ, data.magX, data.magY, data.magZ,
                  data.qW, data.qX, data.qY, data.qZ, data.filteredAltitude,
                  data.filteredVerticalVelocity, data.netVerticalAcceleration,
                  data.tilt, data.barometricPressure, data.airbrakeDeployment,
                  data.pid_gain, data.cd_gain, data.flightState);

      if (flushInterval > 0 && (i % flushInterval == 0)) {
        file.flush();
      }
      if (closeInterval > 0 && (i % closeInterval == 0)) {
        file.close();
        fileIsOpen = false;
      }

      totalTimeUs += (micros() - start);
    }

    if (fileIsOpen)
      file.close();
    SD_MMC.remove(filename);

    float avgTime = (totalTimeUs / (float)numberOfRecords) / 1000.0f;
    DEBUG_PRINTF("%-25s | Flush: %-3d  | Close: %-3d  | Avg: %.3f ms\n", name,
                 flushInterval, closeInterval, avgTime);
    return avgTime;
  };

  DEBUG_PRINTLN_F(
      "\nStrategy                  | Settings    | Action      | Result");
  DEBUG_PRINTLN_F(
      "--------------------------|-------------|-------------|-----------");

  runScenario("50 flush (default)", 50, 0);
  runScenario("25 flush", 25, 0);
  runScenario("10 flush", 10, 0);
  runScenario("5 flush", 5, 0);
  runScenario("1 flush", 1, 0);
  runScenario("50 close", 0, 50);
  runScenario("25 close", 0, 25);
  runScenario("10 close", 0, 10);
  runScenario("5 close", 0, 5);
  runScenario("1 close", 0, 1);
  DEBUG_PRINTLN_F(
      "------------------------------------------------------------------");
}

/**
 * @brief Returns the index of the latest existing internal log.
 * @return Latest index, or -1 if no logs exist.
 */
int DataManager::getLatestInternalLogIndex() const {
  if (!_ffatAvailable || _ffatRecordCounter == 0) return -1;
  return (int)_ffatRecordCounter - 1;
}

// Benchmark the SD card
// DataManager::getInstance().runFrequencyTest(16000000, 50, 1000, false);
// DataManager::getInstance().runStrategyBenchmark(16000000, 1000);

// // SD Comparative Benchmarks (1-bit vs 4-bit)
// for (uint32_t f : {1000000, 4000000, 8000000, 16000000, 20000000}) {
//   DataManager::getInstance().runFrequencyTest(f, 50, 1000, false, true);  // 1-bit
//   DataManager::getInstance().runFrequencyTest(f, 50, 1000, false, false); // 4-bit
// }