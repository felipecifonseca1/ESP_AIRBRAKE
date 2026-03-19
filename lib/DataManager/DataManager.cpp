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
      _sdRecordCounter(0), _sdLEDCounter(0), _pinSDIO_DET(PIN_SDIO_DET),
      _pinCS_Flash(PIN_FLASH_CS), _flash(nullptr), _flashAddr(0),
      _flashAvailable(false), _currentHILMode(HILMode::NONE),
      _hilStabilizing(false) {}

// --- Initialization ---

/**
 * @brief Sets up logging on the SD card by creating a new log file.
 * @details This function initializes the SD card, ensures the log directory
 * exists, and creates a new log file with a unique name. It writes the CSV
 * header to the file.
 * @return true if the setup was successful, false otherwise.
 */
bool DataManager::setupSD() {

  if (!ENABLE_DATA_LOGGING) {
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

    if (LOG_SYNC_INTERVAL <= 1) {
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
 * @brief Initializes the SPI Flash chip.
 */
bool DataManager::setupFlash(bool eraseArea) {
  if (_pinCS_Flash == 255) {
    DEBUG_PRINTLN_F("FLASH: Disabled (PIN 255). Skipping setup.");
    _flashAvailable = false;
    return true; // Consider "success" if disabled
  }

  if (!_flash) {
    _flash = new SPIFlash(_pinCS_Flash, &SPI);
  }

  if (_flash && _flash->begin()) {
    _flashAvailable = true;
    _flashAddr = 0x000000;
    if (eraseArea)
      _flash->eraseChip();
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
      _logFile.flush();
      _logFile.close();
    }
    DEBUG_PRINTLN_F("SD: SD_MMC card closed.");
    // _sdAvailable = false; // Don't mark SD as unavailable, just the file
    // closed.
    _loggingActive = false;
  }
}

// --- Core logging logic ---

/**
 * @brief Records flight data to the SD card.
 * @details This function logs the provided flight data to the SD card. It uses
 * a decimation factor to reduce the amount of data saved without altering the
 * main loop speed. The data is saved in CSV format.
 * @param data A RawFlightData struct containing the flight data to be logged.
 **/
void DataManager::logDataSD(const RawFlightData &data) {
  // Handle Stop Request synchronously
  if (_stopRequested) {
    if (_loggingActive)
      closeSDCard();
    return;
  }

  if (!_sdAvailable || !_loggingActive)
    return;

  // Safety check: Logic handles opening if closed
  // if (!CLOSE_LOG_FILE_AFTER_EACH_WRITE && !_logFile) return;

  _decimationCounter++;

  if (_decimationCounter < _decimationFactor) {
    return; // Skip this data point
  }

  _decimationCounter = 0; // Reset and save now

  uint32_t startTime = millis();

  // Ensure file is open
  if (!_logFile) {
    _logFile = SD_MMC.open(_currentSDFileName, FILE_APPEND);
    if (!_logFile) {
      DEBUG_PRINTLN_F("SD: Error opening file!");
      return;
    }
  }

  _logFile.printf("%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%."
                  "4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%d\n",
                  data.timestamp, data.accX, data.accY, data.accZ, data.gyroX,
                  data.gyroY, data.gyroZ, data.magX, data.magY, data.magZ,
                  data.qW, data.qX, data.qY, data.qZ, data.filteredAltitude,
                  data.filteredVerticalVelocity, data.netVerticalAcceleration,
                  data.tilt, data.barometricPressure, data.airbrakeDeployment,
                  data.pid_gain, data.cd_gain, data.flightState);

  _sdRecordCounter++;
  if (_sdRecordCounter >= LOG_SYNC_INTERVAL) {
    _logFile.close();

    _sdLEDCounter++;
    if (_sdLEDCounter >= 5) {
      _sdLEDCounter = 0;
      // Non-blocking toggle
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

    // Force close if still open to try and save state, then disable
    if (_logFile)
      _logFile.close();
    _loggingActive = false;
    _sdAvailable = false;
  }
}

/**
 * @brief Skeleton for SPI Flash logging (To be implemented with scaling logic).
 * @param data A RawFlightData struct containing the flight data to be logged.
 */
void DataManager::logDataFlash(const RawFlightData &data) {
  // Implement scale and write logic here
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

    // // 1. Attempt 4-bit at 20MHz (Standard Performance)
    // DEBUG_PRINTLN_F("SD: Attempting 4-bit, 8MHz...");
    // if (SD_MMC.begin("/sdcard", false, false, 8000)) {
    //   DEBUG_PRINTLN_F("SD: Mounted successfully (4-bit, 8MHz).");
    //   sdmmcInitialized = true;
    //   return true;
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
void DataManager::dumpCurrentLog() {
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
void DataManager::listFiles() {
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
void DataManager::clearAllLogs() {
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

// Benchmark the SD card
// DataManager::getInstance().runFrequencyTest(16000000, 50, 1000, false);
// DataManager::getInstance().runStrategyBenchmark(16000000, 1000);

// // SD Comparative Benchmarks (1-bit vs 4-bit)
// for (uint32_t f : {1000000, 4000000, 8000000, 16000000, 20000000}) {
//   DataManager::getInstance().runFrequencyTest(f, 50, 1000, false, true);  // 1-bit
//   DataManager::getInstance().runFrequencyTest(f, 50, 1000, false, false); // 4-bit
// }