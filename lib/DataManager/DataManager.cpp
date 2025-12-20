#include "DataManager.h"
#include <esp_task_wdt.h>

// Singleton Instance Getter
DataManager& DataManager::getInstance() {
    static DataManager instance;
    return instance;
}

// Constructor: Initializes internal members
DataManager::DataManager() : 
    _loggingActive(false),
    _HILLoggingActive(false),
    _decimationFactor(1),
    _decimationCounter(0),
    _sdAvailable(false),
    _sdRecordCounter(0),
    _flash(_pinCS_Flash, 0xEF40), // Generic JEDEC ID for W25Q128
    _flashAddr(0),
    _flashAvailable(false) 
{}

// --- INITIALIZATION ---

/**
 * @brief Sets up logging on the SD card by creating a new log file.
 * @details This function initializes the SD card, ensures the log directory exists,
 * and creates a new log file with a unique name. It writes the CSV header to the file.
* @return true if the setup was successful, false otherwise.
 */
bool DataManager::setupSD() {
    if (!ensureSDConnection()) {
        DEBUG_PRINTLN_F("SD: ERROR - Falha ao montar cartao!");
        _sdAvailable = false;
        return false;
    }
    SD.begin(_pinCS_SD, SPI, 4000000);

    if (!SD.exists(_logFolder)) SD.mkdir(_logFolder);

    _currentSDFileName = generateFileName();
    DEBUG_PRINT_F("SD: Criando arquivo: ");
    DEBUG_PRINTLN(_currentSDFileName);
    _logFile = SD.open(_currentSDFileName, FILE_WRITE);

    if (_logFile) {
        // CSV Header
        _logFile.println("Time,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,QW,QX,QY,QZ,AltFilt,VelFilt,AccVertLiq,Tilt,Pressao,Servo,G1,G2,Estado");
        _logFile.flush();
        _sdAvailable = true;
        DEBUG_PRINTLN_F("SD: Pronto.");
        return true;
    }

    _sdAvailable = false;
    return false;
}

/**
 * @brief Initializes the SPI Flash chip.
 */
bool DataManager::setupFlash(bool eraseArea) {
    if (_flash.initialize()) {
        _flashAvailable = true;
        _flashAddr = 0x000000;
        if (eraseArea) _flash.chipErase();
        return true;
    }
    _flashAvailable = false;
    return false;
}

// --- LOGGING CONTROL ---

/**
 * @brief Starts general logging.
 */
void DataManager::startLogging() {
    _loggingActive = true;
    DEBUG_PRINTLN_F("LOG: Iniciado.");
}

/**
 * @brief Stops the general logging.
 */
void DataManager::stopLogging() {
    _loggingActive = false;
    if (_sdAvailable && _logFile) {
        closeSDCard();
    }
    DEBUG_PRINTLN_F("LOG: Parado.");
}

/**
 * @brief Sets the decimation factor for SD card logging.
 * @details This function allows adjustment of the logging frequency by setting
 * a decimation factor. A factor of 1 logs every data point, 2 logs every second data point, etc.
 * @warning Setting a high decimation factor may result in loss of important data.
 * @param fator The decimation factor for logging. 
 */
void DataManager::setDecimationFactor(uint16_t factor) {
    _decimationFactor = (factor < 1) ? 1 : factor;
     DEBUG_PRINT_F("LOG: Fator de decimacao ajustado para "); 
     DEBUG_PRINTLN(_decimationFactor);
}

/**
 * @brief Stops logging and finalizes the SD card file.
 * @details This function flushes any remaining data to the SD card,
 * closes the log file, and updates the logging status.
 **/
void DataManager::closeSDCard() {
    if (_sdAvailable && _logFile) {
        _logFile.flush();
        _logFile.close();
        DEBUG_PRINTLN_F("SD: Cartao SD fechado.");
        _sdAvailable = false;
        _loggingActive = false;
    }
}

// --- CORE LOGGING LOGIC ---

/**
 * @brief Records flight data to the SD card.
 * @details This function logs the provided flight data to the SD card. It uses a decimation factor
 * to reduce the amount of data saved without altering the main loop speed. The data is saved in CSV format.
 * @param data A RawFlightData struct containing the flight data to be logged.
 **/
void DataManager::logDataSD(const RawFlightData& data) {
    if (!_sdAvailable || !_logFile || !_loggingActive) return;

    // Isso reduz a quantidade de dados salvos sem mudar o loop principal
   
    _decimationCounter++;

    if (_decimationCounter < _decimationFactor) {
        return; // Skip this data point
    }

    _decimationCounter = 0; // Reset and save now
    /*    Escaled data logging (commented out for future use)
    // _logFile.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%d,%d,%d,%d\n",
    //     data.timestamp,
    //     (int)(data.accX * 1000), (int)(data.accY * 1000), (int)(data.accZ * 1000),
    //     (int)(data.gyroX * 10),  (int)(data.gyroY * 10),  (int)(data.gyroZ * 10),
    //     (int)(data.magX),        (int)(data.magY),        (int)(data.magZ),
    //     (int)(data.qW * 10000),  (int)(data.qX * 10000),  (int)(data.qY * 10000), (int)(data.qZ * 10000),
    //     (int)(data.filteredAltitude * 100),
    //     (int)(data.filteredVerticalVelocity * 100),
    //     (int)(data.netVerticalAcceleration * 1000),
    //     (int)(data.tilt * 10),
    //     (long)(data.barometricPressure),
    //     (int)(data.servoAtuacao_percent * 10),
    //     (int)(data.gain1 * 1000), (int)(data.gain2 * 1000),
    //     data.flightState
    // );
    */
    _logFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.1f,%.1f,%.2f,%.3f,%.3f,%d\n",
        data.timestamp,
        data.accX, data.accY, data.accZ,
        data.gyroX, data.gyroY, data.gyroZ,
        data.magX, data.magY, data.magZ,
        data.qW, data.qX, data.qY, data.qZ,
        data.filteredAltitude,
        data.filteredVerticalVelocity,
        data.netVerticalAcceleration,
        data.tilt,
        data.barometricPressure,
        data.servoAtuacao_percent,
        data.gain1,
        data.gain2,
        data.flightState
    );

    // Periodic flush to prevent data loss on crash
    _sdRecordCounter++;
    if (_sdRecordCounter >= _sdFlushLimit) {
        _logFile.flush();
        // Serial.println("SD: Dados salvos."); 
        _sdRecordCounter = 0;
    }
}

/**
 * @brief Skeleton for SPI Flash logging (To be implemented with scaling logic).
 */
void DataManager::logDataFlash(const RawFlightData& data) {
    // Implement scale and write logic here
}

// --- HIL SIMULATION ---

/**
 * @brief Initiates a Hardware-in-the-Loop (HIL) simulation by loading data from a specified file on the SD card.
 * @details This function checks for the existence of the specified file on the SD card,
 * attempts to open it, and prepares it for reading simulation data. It handles various
 * scenarios, including missing files and read errors.
 * @param filename The name of the file on the SD card containing HIL simulation data
 **/
bool DataManager::initHIL(const char* filename) {
    if (!ensureSDConnection()) {
        DEBUG_PRINTLN_F("HIL ERRO: Failed to conect to SD.");
        return false;
    }
    String HILFilePath = "";
    // Tries to open the file as given
    if (SD.exists(filename)) {
        HILFilePath = String(filename);
        DEBUG_PRINT_F("HIL: File found on root: ");
        DEBUG_PRINTLN(HILFilePath);
    } 
    // Tries to open the file with leading slash
    else if (SD.exists("/" + String(filename))) {
        HILFilePath = "/" + String(filename);
        DEBUG_PRINT_F("HIL: File found (with leading slash): ");
        DEBUG_PRINTLN(HILFilePath);
    }
     // Tries inside the logs folder
    else {
         String pathTemp = String(_logFolder) + "/" + String(filename);
         if (SD.exists(pathTemp)) {
             HILFilePath = pathTemp;
             DEBUG_PRINT_F("HIL: File found in logs folder: ");
             DEBUG_PRINTLN(HILFilePath);
         } else {
             DEBUG_PRINT_F("HIL ERRO: Arquivo NAO encontrado: ");
             DEBUG_PRINTLN(filename);
             return false;
         }
    }

    _hilFile = SD.open(filename, FILE_READ);
    if (_hilFile) {
        DEBUG_PRINTLN_F("HIL: File opened successfully.");
        if (_hilFile.available()) {
            String header = _hilFile.readStringUntil('\n'); 
            DEBUG_PRINT_F("HIL: Read header: ");
            DEBUG_PRINTLN(header.substring(0, 60) + "..."); 
        } else {
            DEBUG_PRINTLN_F("HIL: Warning - File is empty.");
            _hilFile.close();
            return false;
        }
        return true;
    } else {
        DEBUG_PRINTLN_F("HIL: Error - SD.open failed.");
        return false;
    }
}

/**
 * @brief Reads the next simulation step from the HIL simulation file.
 * @details This function reads the next line from the currently active HIL simulation file
 * on the SD card, parses the data, and returns it in a HILSimulationData struct. If the simulation
 * is not active or the file is not available, it returns a struct with dadosValidos set to false.
 * @return A HILSimulationData struct containing the parsed simulation data.
 **/
HILSimulationData DataManager::readHILStep() {
    HILSimulationData data;
    if (!_hilFile || !_hilFile.available()){
        data.dadosValidos = false;
        return data;
    } 
    
    String line = _hilFile.readStringUntil('\n');
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);

    if (p1 != -1 && p2 != -1 && p3 != -1) {
        data.tempo_s = line.substring(0, p1).toFloat();
        data.pressao_Pa = line.substring(p1+1, p2).toFloat();
        data.aceleracaoLiquida_ms2 = line.substring(p2+1, p3).toFloat();
        data.tilt = line.substring(p3+1).toFloat();
        data.dadosValidos = true;
    }
    return data;
}

/**
 * @brief Resets the HIL simulation file to the beginning for a new run.
 */
void DataManager::resetHIL() {
    if (_hilFile) {
        _hilFile.seek(0);
        _hilFile.readStringUntil('\n');
    }
    DEBUG_PRINTLN_F("HIL: File reset."); 
}

/**
 * @brief Stops the HIL simulation and closes the associated file.
 */
void DataManager::stopHIL() {
    if (_hilFile) _hilFile.close();
}

// --- AUXILIARY METHODS ---

/**
 * @brief Ensures the SD card is connected and mounted.
    * @return true if the SD card is accessible, false otherwise.
 */
bool DataManager::ensureSDConnection() {
    if (SD.cardType() != CARD_NONE) return true;
    SPI.begin(_pinSCK_SD, _pinMISO_SD, _pinMOSI_SD, _pinCS_SD); // Standard ESP32 VSPI pins
    if (!SD.begin(_pinCS_SD)) {
        return false;
    }
    return true;
}

/**
 * @brief Generates the next available filename for logging on the SD card.
 * @details This function checks for existing files in the log directory and
 * generates a new filename by incrementing a counter until an unused name is found.
 * @return A String containing the next available filename.
 */
String DataManager::generateFileName() {
    char buf[64];
    for (int i = 0; i < _maxLogFiles; i++) {
        snprintf(buf, sizeof(buf), "%s/%s%03d.csv", _logFolder, _logBasename, i);
        if (!SD.exists(buf)) return String(buf);
    }
    snprintf(buf, sizeof(buf), "%s/%s%lu.%s", _logFolder, _logBasename, (unsigned long)millis(), "csv");
    return String(buf);
}

/**
 * @brief Dumps the current log file from the SD card to the Serial console.
 * @details This function reads the current log file stored on the SD card
 * and outputs its contents to the Serial console. It is useful for retrieving
 * log data without removing the SD card.
 **/
void DataManager::dumpCurrentLog() {
    if (_currentSDFileName == ""){
        Serial.println("No file name registered to read.");
        return;
    } 
    if (!ensureSDConnection()) {
        Serial.println("Error: SD card not accessible.");
        return;
    }
    Serial.print("\n--- Reading file: ");
    Serial.print(_currentSDFileName);
    Serial.println(" ---");
    File f = SD.open(_currentSDFileName);
   
    if (!f) {
        Serial.println("Error opening file for reading.");
        return;
    }

    uint32_t byteCount = 0;

    while (f.available()){
        Serial.write(f.read());
        if (++byteCount % 512 == 0) {
            esp_task_wdt_reset();
        }
    } 
    f.close();
    Serial.println("\n==========================================");
    Serial.println("--- FIM DO ARQUIVO ---");
}

/**
 * @brief Lists all log files stored on the _logFolder (/REG_VOO) SD card.
 * @details This function accesses the SD card and lists all files in the
 * specified log directory. It prints the filenames and their sizes in KB to the Serial console.
 **/
void DataManager::listFiles() {
    if (!ensureSDConnection()) {
        Serial.println("Error: SD card not accessible.");
        return;
    }

    Serial.println("\n--- List of files in " + String(_logFolder) + " ---");
    File root = SD.open(_logFolder);
    if (!root || !root.isDirectory()) {
        Serial.println("Log folder empty or non-existent.");
        return;
    }
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            Serial.print("  ");
            Serial.print(file.name());
            Serial.print("\tSize: ");
            Serial.print(file.size() / 1024.0);
            Serial.println(" KB");
        }
        file = root.openNextFile();
    }
    Serial.println("--------------------------------");
}

/**
 * @brief Deletes all log files stored on the SD card in the _logFolder directory.
 * @details This function accesses the SD card and deletes all files in the specified
 * log directory. It prompts the user for confirmation before proceeding with the deletion.
 **/
void DataManager::clearAllLogs() {
    if (!ensureSDConnection()) {
        Serial.println("SD: Error to connect.");
        return;
    }

    Serial.println("\n!!! Warning: Deleting files in 3 seconds !!!");
    delay(3000); // Delay to delete files

    File root = SD.open(_logFolder);
    if (!root || !root.isDirectory()) {
        Serial.println("Log folder empty or non-existent.");
        return;
    }

    Serial.println("Clearing logs...");
    
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

        Serial.print("Deleting: ");
        Serial.print(path);

        if (SD.remove(path)) {
            Serial.println(" [OK]");
        } else {
            Serial.println(" [FAILED]");
        }

        file = root.openNextFile();
    }
    root.close();

    Serial.println("Clearing complete.");
    
    _sdAvailable = false;
    _currentSDFileName = "";
}

/**
 * @brief Receives a HIL simulation file via Serial and saves it to the SD card.
 * @details This function prompts the user to send a CSV file via Serial input.
 * It reads the incoming data, stores it in a buffer, and saves it to a specified
 * file on the SD card. The function handles timeouts and notifies the user of success or errors.
 * @param HILFileName The name of the file to save the received HIL simulation data.
 **/
void DataManager::receiveHILFile(const char* HILFileName) {
    if (!ensureSDConnection()) {
        Serial.println("Erro: SD not found.");
        return;
    }
    
    // Reserva Buffer (200KB) *Mudar depois para algo parametrizavel*
    String bufferRAM;
    if (!bufferRAM.reserve(200000)) {
        Serial.println("Warning: Low memory.");
    }

    Serial.println("\n==================================================");
    Serial.println("FILE UPLOAD MODE TO RECEIVE HIL SIMULATION DATA");
    Serial.println("==================================================");
    Serial.println("1. Copy the CSV content.");
    Serial.println("2. Paste ALL and send.");
    Serial.println("3. Type 'END' and send.");
    Serial.println("--- READY ---");

    bool recebendo = true;
    unsigned long ultimaAtividade = millis();

    while (recebendo) {
        if (Serial.available()) {
            char c = Serial.read();
            bufferRAM += c;
            ultimaAtividade = millis();
            
            if (bufferRAM.length() > 200000) {
                Serial.println("\nError: Buffer full!");
                recebendo = false;
            }
        }

        // Verifica FIM
        if (bufferRAM.endsWith("END") || bufferRAM.endsWith("END\n") || bufferRAM.endsWith("END\r\n")) {
            recebendo = false;
            int posFim = bufferRAM.lastIndexOf("END");
            if (posFim != -1) bufferRAM = bufferRAM.substring(0, posFim);
            Serial.println("\nEND command detected.");
        }
        
        if (millis() - ultimaAtividade > 10000) { // Timeout of 10 seconds
             Serial.println("\nTimeout. Saving...");
             recebendo = false;
        }
        esp_task_wdt_reset();
    }
    
    if (bufferRAM.indexOf('\n') == -1) {
        Serial.println("Warning: Single-line text detected. Repairing formatting...");
        
        int mudancas = 0;
        int len = bufferRAM.length();
        
        
        for (int i = 1; i < len - 1; i++) {
            if (bufferRAM[i] == ' ') {
                char prev = bufferRAM[i-1];
                char next = bufferRAM[i+1];
                
                // Se o anterior é um dígito E o próximo é dígito ou sinal negativo
                if (isDigit(prev) && (isDigit(next) || next == '-')) {
                    bufferRAM[i] = '\n'; // Substitui espaço por Enter
                    mudancas++;
                }
            }
        }
        Serial.print("Break lines added: ");
        Serial.println(mudancas);
    }

    // Gravação
    SD.remove(HILFileName);
    File fileHIL = SD.open(HILFileName, FILE_WRITE);
    if (fileHIL) {
        fileHIL.print(bufferRAM);
        fileHIL.flush();
        fileHIL.close();
        Serial.println("SUCCESS: File saved to SD.");
    } else {
        Serial.println("ERROR: Failed to save file.");
    }
    
    bufferRAM = ""; 
    Serial.println("==================================================");
}