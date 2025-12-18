// // Gerenciador_dados.cpp
#include "Gerenciador_dados.h" 
#include "Config_voo.h"     
#include <SPI.h>
#include <SPIFlash.h>       
#include <SD.h>             
#include <Arduino.h>

// Global variables for SD logging
bool gl_loggingAtivo = false;
u_int8_t gl_fatorDecimacao = 1; // Default: log every cycle

// --- Flash SPI ---
/* 

const uint8_t GL_FLASH_CS_PIN = 15; // Pino Chip Select ALTERAR
const uint16_t GL_W25QXX_JEDEC_ID = 0xEF40; 
static SPIFlash gl_flash_obj(GL_FLASH_CS_PIN, GL_W25QXX_JEDEC_ID);

static uint32_t gl_enderecoEscritaFlash = 0;
const uint32_t GL_TAMANHO_TOTAL_FLASH_BYTES = 16L * 1024L * 1024L; // Para W25Q128
// const uint32_t GL_TAMANHO_TOTAL_FLASH_BYTES = 8L * 1024L * 1024L; // Para W25Q64;
const uint32_t GL_INICIO_AREA_LOG_FLASH = 0x000000;

const uint32_t GL_FIM_AREA_LOG_FLASH_EXCLUSIVE = GL_TAMANHO_TOTAL_FLASH_BYTES; 
const uint32_t GL_TAMANHO_SETOR_FLASH = 4096; // 4KB
static uint32_t gl_ultimoSetorFlashApagado = 0xFFFFFFFF; 

// Fatores de Escala para Flash SPI
static const float GL_ACCEL_SCALE_FACTOR = 1000.0f;
static const float GL_GYRO_SCALE_FACTOR = 10.0f;
static const float GL_MAG_SCALE_FACTOR = 10.0f;
static const float GL_QUAT_SCALE_FACTOR = 30000.0f;
static const float GL_ALT_SCALE_FACTOR = 10.0f;
static const float GL_VEL_SCALE_FACTOR = 10.0f;
static const float GL_ACC_VERT_SCALE_FACTOR = 10.0f;
static const float GL_TILT_SCALE_FACTOR = 100.0f; 
static const float GL_PRESS_BMP_OFFSET_PA = 85000.0f;
static const float GL_SERVO_SCALE_FACTOR = 100.0f;
static const float GL_GAIN1_SCALE_FACTOR = 100.0f;
static const float GL_GAIN2_SCALE_FACTOR = 100.0f;

bool setupLogFlashSPI(bool apagarAreaLog) {
    DEBUG_PRINTLN_F("GL_FLASH: Inicializando Flash SPI...");

    if (!gl_flash_obj.initialize()) {
        DEBUG_PRINTLN_F("GL_FLASH: ERRO FATAL - Falha ao inicializar memoria flash SPI!");
        return false;
    }
    DEBUG_PRINTLN_F("GL_FLASH: Flash SPI inicializada.");
    DEBUG_PRINT_F("  GL_FLASH: JEDEC ID Lido: 0x"); 
    DEBUG_PRINTLN(gl_flash_obj.readDeviceId(), HEX);
    
    gl_enderecoEscritaFlash = GL_INICIO_AREA_LOG_FLASH;
    gl_ultimoSetorFlashApagado = 0xFFFFFFFF; // Força erase do primeiro setor a ser usado

    if (apagarAreaLog) { 
        DEBUG_PRINT_F("GL_FLASH: Apagando primeiro setor da flash SPI em 0x");
        DEBUG_PRINTLN(gl_enderecoEscritaFlash, HEX);
        
        gl_flash_obj.blockErase4K(gl_enderecoEscritaFlash); 
        
        DEBUG_PRINTLN_F("  GL_FLASH: Comando de apagar setor enviado. Aguardando conclusao...");
        unsigned long inicioErase = millis();
        bool eraseTimeout = false;
        while(gl_flash_obj.busy()){ 
            delay(50); 
            if(millis() - inicioErase > 2000){ 
                DEBUG_PRINTLN_F("\n  GL_FLASH: Timeout ao apagar setor!");
                eraseTimeout = true;
                break;
            }
        }
        if (!eraseTimeout) {
            DEBUG_PRINTLN_F("\n  GL_FLASH: Setor apagado.");
            gl_ultimoSetorFlashApagado = gl_enderecoEscritaFlash;
        } else {
            // Decide se isso é uma falha crítica para o setup
            // return false; 
        }
    }
    DEBUG_PRINTLN_F("GL_FLASH: Setup da Flash SPI concluido.");
    return true;
}

// Pega os dados recebidos, compacta e salva na flash
void gravarLogFlashSPI(const DadosVooBrutosParaLog& dados) {
    if (!gl_loggingAtivo) return;

    LogDataVooEscalonada entradaFlash;
    entradaFlash.timestamp_ms = dados.timestamp;
    entradaFlash.accX_scaled = (int16_t)(dados.accX * GL_ACCEL_SCALE_FACTOR);
    entradaFlash.accY_scaled = (int16_t)(dados.accY * GL_ACCEL_SCALE_FACTOR);
    entradaFlash.accZ_scaled = (int16_t)(dados.accZ * GL_ACCEL_SCALE_FACTOR);
    entradaFlash.gyroX_scaled = (int16_t)(dados.gyroX * GL_GYRO_SCALE_FACTOR);
    entradaFlash.gyroY_scaled = (int16_t)(dados.gyroY * GL_GYRO_SCALE_FACTOR);
    entradaFlash.gyroZ_scaled = (int16_t)(dados.gyroZ * GL_GYRO_SCALE_FACTOR);
    entradaFlash.magX_scaled = (int16_t)(dados.magX * GL_MAG_SCALE_FACTOR);
    entradaFlash.magY_scaled = (int16_t)(dados.magY * GL_MAG_SCALE_FACTOR);
    entradaFlash.magZ_scaled = (int16_t)(dados.magZ * GL_MAG_SCALE_FACTOR);
    entradaFlash.qW_scaled = (int16_t)(dados.qW * GL_QUAT_SCALE_FACTOR);
    entradaFlash.qX_scaled = (int16_t)(dados.qX * GL_QUAT_SCALE_FACTOR);
    entradaFlash.qY_scaled = (int16_t)(dados.qY * GL_QUAT_SCALE_FACTOR);
    entradaFlash.qZ_scaled = (int16_t)(dados.qZ * GL_QUAT_SCALE_FACTOR);
    entradaFlash.altitudeFiltrada_scaled = (int16_t)(dados.altitudeFiltrada * GL_ALT_SCALE_FACTOR);
    entradaFlash.velocidadeVerticalFiltrada_scaled = (int16_t)(dados.velocidadeVerticalFiltrada * GL_VEL_SCALE_FACTOR);
    entradaFlash.aceleracaoVerticalLiquida_scaled = (int16_t)(dados.aceleracaoVerticalLiquida * GL_ACC_VERT_SCALE_FACTOR);
    entradaFlash.tilt_scaled = (int16_t)(dados.tilt * GL_TILT_SCALE_FACTOR); 
    entradaFlash.pressaoBMP_scaled = (int16_t)(dados.pressaoBMP - GL_PRESS_BMP_OFFSET_PA);
    entradaFlash.gain1_scaled = (int16_t)(dados.gain1 * GL_GAIN1_SCALE_FACTOR);
    entradaFlash.gain2_scaled = (int16_t)(dados.gain2 * GL_GAIN1_SCALE_FACTOR);
    entradaFlash.estadoFoguete = (uint8_t)dados.estadoFoguete;

    uint32_t setorAtualParaEscritaFlash = (gl_enderecoEscritaFlash / GL_TAMANHO_SETOR_FLASH) * GL_TAMANHO_SETOR_FLASH;
    if (setorAtualParaEscritaFlash != gl_ultimoSetorFlashApagado && (gl_enderecoEscritaFlash % GL_TAMANHO_SETOR_FLASH == 0)) {
        if (gl_enderecoEscritaFlash >= GL_FIM_AREA_LOG_FLASH_EXCLUSIVE) {  }
        DEBUG_PRINT_F("GL_FLASH: Entrando em novo setor. Apagando setor em 0x"); DEBUG_PRINTLN(setorAtualParaEscritaFlash, HEX);

        unsigned long inicioEraseSetor = millis();
        gl_flash_obj.blockErase4K(setorAtualParaEscritaFlash); // Chamada direta

        bool eraseTimeout = false;
        unsigned long t_erase_loop = millis();
        while(gl_flash_obj.busy()) { 
            if (millis() - t_erase_loop > 800) { 
                DEBUG_PRINTLN_F("GL_FLASH: Timeout durante o erase do setor!"); 
                eraseTimeout = true;
                break; 
            } 
            delay(10); 
        }
        if (eraseTimeout) {
            pararLoggingGeral(); 
            return; 
        }
        gl_ultimoSetorFlashApagado = setorAtualParaEscritaFlash;
        DEBUG_PRINT_F("GL_FLASH: Setor apagado. Tempo decorrido: "); 
        DEBUG_PRINT(millis() - inicioEraseSetor); DEBUG_PRINTLN_F(" ms.");
    }

    if (gl_enderecoEscritaFlash + sizeof(LogDataVooEscalonada) > GL_FIM_AREA_LOG_FLASH_EXCLUSIVE) {
        DEBUG_PRINTLN_F("GL_FLASH: Memoria flash cheia! Log SPI interrompido.");
        return;
    }

    gl_flash_obj.writeBytes(gl_enderecoEscritaFlash, &entradaFlash, sizeof(LogDataVooEscalonada));
    gl_enderecoEscritaFlash += sizeof(LogDataVooEscalonada);
}

uint32_t getEnderecoAtualFlashSPI() {
    return gl_enderecoEscritaFlash;
}

*/

//  --- SD Card ---

// Pins VSPI for ESP32 DevKit V1
#define PIN_SD_SCK  18
#define PIN_SD_MISO 19
#define PIN_SD_MOSI 23
const u_int8_t GL_SD_CS_PIN = 5; // Pin Chip Select of SD Card

// === Configurações e Variáveis para Cartão SD ===
static File gl_dataFileSD; // Objeto de arquivo para o SD
static String gl_currentFileName = "";
const char* GL_FOLDER_LOG_SD = "/REG_VOO"; // Importante ter a barra inicial
const char* GL_BASENAME_LOG_SD = "VOO_";    
const u_int8_t GL_FLUSH_EVERY_N_REGISTER = 50; // Adjusted for performance - 50 (~1seg) 
static uint8_t gl_registerCounterSD = 0;
bool sdDisponivel = false;   

/**
 * @brief Finds the next available filename on the SD card for logging.
 * @details This function searches for the next available filename in the specified
 * directory on the SD card. It constructs filenames in the format:
 * /<folder>/<baseName>XXX.<ext>, where XXX is a zero-padded index.
 * It checks for existing files up to maxIndex and returns the first available name.
 * * @param folder The directory on the SD card where logs are stored.
 * @param baseName The base name for the log files. 
 * @param ext The file extension (e.g., "csv").
 * @param maxIndex The maximum index to check for existing files (default is 1000
 * to allow for filenames from 000 to 999).
 * @return A String containing the next available filename.
 */
String findNextAvailableName_SD(const char* folder, const char* baseName, const char* ext, u_int16_t maxIndex = 1000) {
    char buf[64];
    for (int i = 0; i < maxIndex; i++) {
        // Formats: /REG_VOO/VOO_000.csv
        snprintf(buf, sizeof(buf), "%s/%s%03d.%s", folder, baseName, i, ext);
        if (!SD.exists(buf)) {
            return String(buf);
        }
    }
    // Fallback if there are no available names
    snprintf(buf, sizeof(buf), "%s/%s%lu.%s", folder, baseName, (unsigned long)millis(), ext);
    return String(buf);
}

/**
 * @brief Ensures the SD card is connected and mounted.
    * @return true if the SD card is accessible, false otherwise.
 */
bool ensureConnectionSD() {
    // Se já estiver acessível, retorna true
    if (SD.cardType() != CARD_NONE) return true;

    // Se não, tenta inicializar
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, GL_SD_CS_PIN);
    if (!SD.begin(GL_SD_CS_PIN)) {
        return false;
    }
    return true;
}

/**
 * @brief Sets up logging on the SD card by creating a new log file.
 * @details This function initializes the SD card, ensures the log directory exists,
 * and creates a new log file with a unique name. It writes the CSV header to the file.
* @return true if the setup was successful, false otherwise.
 */
bool setupLogSDCard() {
    DEBUG_PRINTLN_F("SD: Iniciando setup...");

    if (!ensureConnectionSD()) {
        DEBUG_PRINTLN_F("SD: ERRO - Falha ao montar cartao!");
        sdDisponivel = false;
        return false;
    }

    if (!SD.exists(GL_FOLDER_LOG_SD)) {
        SD.mkdir(GL_FOLDER_LOG_SD);
    }

    gl_currentFileName = findNextAvailableName_SD(GL_FOLDER_LOG_SD, GL_BASENAME_LOG_SD, "csv");
    
    DEBUG_PRINT_F("SD: Criando arquivo: ");
    DEBUG_PRINTLN(gl_currentFileName);

    gl_dataFileSD = SD.open(gl_currentFileName, FILE_WRITE);

    if (gl_dataFileSD) {
        gl_dataFileSD.println("Time,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,QW,QX,QY,QZ,AltFilt,VelFilt,AccVertLiq,Tilt,Pressao,Servo,Gain1,Gain2,Estado");
        gl_dataFileSD.flush();
        sdDisponivel = true;
        DEBUG_PRINTLN_F("SD: Pronto.");
        return true;
    } else {
        sdDisponivel = false;
        return false;
    }
}

/**
 * @brief Lists all log files stored on the GL_FOLDER_LOG_SD (/REG_VOO) SD card.
 * @details This function accesses the SD card and lists all files in the
 * specified log directory. It prints the filenames and their sizes in KB to the Serial console.
 **/
void listarArquivosSD() {
    if (!ensureConnectionSD()) {
        Serial.println("Erro: Não foi possível acessar o SD para listar.");
        return;
    }
    
    Serial.println("\n--- Lista de arquivos em " + String(GL_FOLDER_LOG_SD) + " ---");
    File root = SD.open(GL_FOLDER_LOG_SD);
    if (!root || !root.isDirectory()) {
        Serial.println("Pasta de logs vazia ou inexistente.");
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
 * @brief Dumps the current log file from the SD card to the Serial console.
 * @details This function reads the current log file stored on the SD card
 * and outputs its contents to the Serial console. It is useful for retrieving
 * log data without removing the SD card.
 **/
void despejarLogAtualNaSerial() {
    if (gl_currentFileName == "") {
        Serial.println("Nenhum nome de arquivo registrado para ler.");
        return;
    }

    // Tenta reconectar hardware se necessário
    if (!ensureConnectionSD()) {
        Serial.println("Erro: Cartão SD não encontrado.");
        return;
    }

    // Se o arquivo ainda estiver aberto para escrita, salva tudo antes de ler
    if (sdDisponivel && gl_dataFileSD) {
        gl_dataFileSD.flush();
    }

    Serial.print("\n--- LENDO ARQUIVO: ");
    Serial.print(gl_currentFileName);
    Serial.println(" ---");
    Serial.println("COPIE O CONTEUDO ABAIXO E SALVE COMO .CSV:");
    Serial.println("==========================================");

    File fileRead = SD.open(gl_currentFileName, FILE_READ);
    if (fileRead) {
        uint8_t buffer[64];
        while (fileRead.available()) {
            int lidos = fileRead.read(buffer, sizeof(buffer));
            Serial.write(buffer, lidos);
        }
        fileRead.close();
    } else {
        Serial.println("Erro ao abrir arquivo para leitura.");
    }

    Serial.println("\n==========================================");
    Serial.println("--- FIM DO ARQUIVO ---");
}

/**
 * @brief Deletes all log files stored on the SD card in the GL_FOLDER_LOG_SD directory.
 * @details This function accesses the SD card and deletes all files in the specified
 * log directory. It prompts the user for confirmation before proceeding with the deletion.
 **/
void limparTodosLogs() {
    if (!ensureConnectionSD()) {
        Serial.println("Erro: SD não detectado.");
        return;
    }

    Serial.println("\n!!! ATENCAO: APAGANDO TODOS OS LOGS EM 3 SEGUNDOS !!!");
    delay(3000); // Dá tempo de se arrepender (desligar)

    File root = SD.open(GL_FOLDER_LOG_SD);
    if (!root || !root.isDirectory()) {
        Serial.println("Pasta de logs não encontrada.");
        return;
    }

    Serial.println("Iniciando limpeza...");
    
    File file = root.openNextFile();
    while (file) {
        String path = "";
        String nome = String(file.name());
        if (nome.startsWith("/")) {
            path = nome;
        } else {
            path = String(GL_FOLDER_LOG_SD) + "/" + nome;
        }

        file.close();

        Serial.print("Apagando: ");
        Serial.print(path);

        if (SD.remove(path)) {
            Serial.println(" [OK]");
        } else {
            Serial.println(" [FALHA]");
        }

        // Pega o próximo
        file = root.openNextFile();
    }
    root.close();

    Serial.println("Limpeza concluída. Reinicie o ESP32 para resetar o contador de arquivos.");
    
    // Reseta variaveis globais para forçar setup limpo na proxima vez
    sdDisponivel = false;
    gl_currentFileName = "";
}

/**
 * @brief Returns the current log filename being used on the SD card.
 * @return A String containing the current log filename.
 */
String getNomeArquivoSDLog() {
    return gl_currentFileName;
}

/**
 * @brief Sets the decimation factor for SD card logging.
 * @details This function allows adjustment of the logging frequency by setting
 * a decimation factor. A factor of 1 logs every data point, 2 logs every second data point, etc.
 * @warning Setting a high decimation factor may result in loss of important data.
 * @param fator The decimation factor for logging. 
 */
void setFatorDecimacaoLog(u_int16_t fator) {
    if (fator < 1) fator = 1;
    gl_fatorDecimacao = fator;
    DEBUG_PRINT_F("LOG: Fator de decimacao ajustado para "); DEBUG_PRINTLN(gl_fatorDecimacao);
}

/**
 * @brief Records flight data to the SD card.
 * @details This function logs the provided flight data to the SD card. It uses a decimation factor
 * to reduce the amount of data saved without altering the main loop speed. The data is saved in CSV format.
 * @param dados A DadosVooBrutosParaLog struct containing the flight data to be logged.
 **/
void gravarLogSDCard(DadosVooBrutosParaLog dados) {
    if (!sdDisponivel || !gl_dataFileSD || !gl_loggingAtivo) return;

    // Isso reduz a quantidade de dados salvos sem mudar o loop principal
    static uint8_t contadorDecimacao = 0;
    contadorDecimacao++;

    if (contadorDecimacao < gl_fatorDecimacao) {
        return; // Pula este ciclo de salvamento 
    }
    contadorDecimacao = 0; // Reseta e salva agora

    // * Testar uso de fatores de escala para inteiros economizando armazenamento *
    /*
    gl_dataFileSD.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%d,%d,%d,%d\n",
        dados.timestamp,
        (int)(dados.accX * 1000), (int)(dados.accY * 1000), (int)(dados.accZ * 1000),
        (int)(dados.gyroX * 10), (int)(dados.gyroY * 10), (int)(dados.gyroZ * 10),
        (int)(dados.magX), (int)(dados.magY), (int)(dados.magZ),
        (int)(dados.qW * 10000), (int)(dados.qX * 10000), (int)(dados.qY * 10000), (int)(dados.qZ * 10000),
        (int)(dados.altitudeFiltrada * 100), // Ex: 123.45m -> 12345
        (int)(dados.velocidadeVerticalFiltrada * 100),
        (int)(dados.aceleracaoVerticalLiquida * 1000),
        (int)(dados.tilt * 10),
        (long)(dados.pressaoBMP), // Pressão é grande (Pa), use long
        (int)(dados.servoAtuacao_percent * 10),
        (int)(dados.gain1 * 1000),
        (int)(dados.gain2 * 1000),
        dados.estadoFoguete
    );
    */

    gl_dataFileSD.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.1f,%.1f,%.2f,%.3f,%.3f,%d\n",
        dados.timestamp,
        dados.accX, dados.accY, dados.accZ,
        dados.gyroX, dados.gyroY, dados.gyroZ,
        dados.magX, dados.magY, dados.magZ,
        dados.qW, dados.qX, dados.qY, dados.qZ,
        dados.altitudeFiltrada,
        dados.velocidadeVerticalFiltrada,
        dados.aceleracaoVerticalLiquida,
        dados.tilt,
        dados.pressaoBMP,
        dados.servoAtuacao_percent,
        dados.gain1,
        dados.gain2,
        dados.estadoFoguete
    );

    // Controle de Flush (Também afeta a frequência de travamento)
    gl_registerCounterSD++;
    if (gl_registerCounterSD >= GL_FLUSH_EVERY_N_REGISTER) {
        gl_dataFileSD.flush();
        // Serial.println("SD: Dados salvos."); 
        gl_registerCounterSD = 0;
    }
}

/**
 * @brief Stops logging and finalizes the SD card file.
 * @details This function flushes any remaining data to the SD card,
 * closes the log file, and updates the logging status.
 **/
void finalizarSDCard() {
    if (sdDisponivel && gl_dataFileSD) {
        gl_dataFileSD.flush();
        gl_dataFileSD.close();
        DEBUG_PRINTLN_F("SD: Arquivo fechado e salvo.");
        sdDisponivel = false;
        gl_loggingAtivo = false;
    }
}

// =================================================================================
// --- Simulacao HIL via SD ---
// =================================================================================
File gl_arquivoSimulacaoHIL;
bool gl_simulacaoHIL_ativa = false;

/**
 * @brief Initiates a Hardware-in-the-Loop (HIL) simulation by loading data from a specified file on the SD card.
 * @details This function checks for the existence of the specified file on the SD card,
 * attempts to open it, and prepares it for reading simulation data. It handles various
 * scenarios, including missing files and read errors.
 * @param nomeArquivo The name of the file on the SD card containing HIL simulation data
 **/
bool iniciarSimulacaoHIL(const char* nomeArquivo) {

    // 1. Hardware Check
    if (!ensureConnectionSD()) {
        DEBUG_PRINTLN_F("HIL ERRO: Falha na conexao fisica com SD.");
        return false;
    }
    
    // 2. Verifica Existência (Tenta raiz e pasta logs)
    String caminhoFinal = "";
    
    // Tenta exatamente como passado (ex: "/Teste_HIL.csv")
    if (SD.exists(nomeArquivo)) {
        caminhoFinal = String(nomeArquivo);
        DEBUG_PRINT_F("HIL: Arquivo encontrado na raiz: ");
        DEBUG_PRINTLN(caminhoFinal);
    } 
    // Tenta adicionar barra se faltou (ex: "Teste_HIL.csv" -> "/Teste_HIL.csv")
    else if (SD.exists("/" + String(nomeArquivo))) {
        caminhoFinal = "/" + String(nomeArquivo);
        DEBUG_PRINT_F("HIL: Arquivo encontrado (com barra add): ");
        DEBUG_PRINTLN(caminhoFinal);
    }
    // Tenta dentro da pasta de logs
    else {
         String pathTemp = String(GL_FOLDER_LOG_SD) + "/" + String(nomeArquivo);
         if (SD.exists(pathTemp)) {
             caminhoFinal = pathTemp;
             DEBUG_PRINT_F("HIL: Arquivo encontrado na pasta logs: ");
             DEBUG_PRINTLN(caminhoFinal);
         } else {
             DEBUG_PRINT_F("HIL ERRO: Arquivo NAO encontrado: ");
             DEBUG_PRINTLN(nomeArquivo);
             return false;
         }
    }
    
    // 3. Tenta Abrir
    gl_arquivoSimulacaoHIL = SD.open(caminhoFinal, FILE_READ);
    
    if (gl_arquivoSimulacaoHIL) {
        gl_simulacaoHIL_ativa = true;
        DEBUG_PRINTLN_F("HIL: Arquivo aberto com sucesso.");
        
        // *Mudar para usar func de verificacao*
        if (gl_arquivoSimulacaoHIL.available()) {
             String header = gl_arquivoSimulacaoHIL.readStringUntil('\n'); 
             DEBUG_PRINT_F("HIL: Header lido: ");
             // Imprime só os primeiros 60 chars para não poluir
             DEBUG_PRINTLN(header.substring(0, 60) + "..."); 
        } else {
             DEBUG_PRINTLN_F("HIL AVISO: Arquivo vazio!");
             gl_arquivoSimulacaoHIL.close();
             return false;
        }
        return true;
    } else {
        DEBUG_PRINTLN_F("HIL ERRO: SD.open falhou (pode ser limite de arquivos abertos).");
        return false;
    }
}

/**
 * @brief Reads the next simulation step from the HIL simulation file.
 * @details This function reads the next line from the currently active HIL simulation file
 * on the SD card, parses the data, and returns it in a DadosSimulacaoHIL struct. If the simulation
 * is not active or the file is not available, it returns a struct with dadosValidos set to false.
 * @return A DadosSimulacaoHIL struct containing the parsed simulation data.
 **/
DadosSimulacaoHIL lerProximoPassoSimulacaoHIL() {
    DadosSimulacaoHIL dados;
    if (!gl_simulacaoHIL_ativa || !gl_arquivoSimulacaoHIL.available()) {
        dados.dadosValidos = false;
        return dados;
    }
    
    String linha = gl_arquivoSimulacaoHIL.readStringUntil('\n');
    int p1 = linha.indexOf(',');
    int p2 = linha.indexOf(',', p1 + 1);
    int p3 = linha.indexOf(',', p2 + 1);

    if (p1 != -1 && p2 != -1 && p3 != -1) {
        dados.tempo_s = linha.substring(0, p1).toFloat();
        dados.pressao_Pa = linha.substring(p1+1, p2).toFloat();
        dados.aceleracaoLiquida_ms2 = linha.substring(p2+1, p3).toFloat();
        dados.tilt = linha.substring(p3+1).toFloat();
        dados.dadosValidos = true;
    }
    return dados;
}

/**
 * @brief Receives a HIL simulation file via Serial and saves it to the SD card.
 * @details This function prompts the user to send a CSV file via Serial input.
 * It reads the incoming data, stores it in a buffer, and saves it to a specified
 * file on the SD card. The function handles timeouts and notifies the user of success or errors.
 **/
void receberArquivoHILViaSerial() {
    if (!ensureConnectionSD()) {
        Serial.println("ERRO: SD nao encontrado.");
        return;
    }

    const char* nomeArquivoHIL = "/Teste_HIL.csv"; // *Mudar depois para algo parametrizavel*
    
    // Reserva Buffer (200KB) *Mudar depois para algo parametrizavel*
    String bufferRAM;
    if (!bufferRAM.reserve(200000)) {
        Serial.println("AVISO: Memoria baixa. Arquivo pode ser cortado.");
    }

    Serial.println("\n==================================================");
    Serial.println("MODO DE UPLOAD DE ARQUIVO HIL VIA SERIAL");
    Serial.println("==================================================");
    Serial.println("1. Copie o conteudo do CSV.");
    Serial.println("2. Cole TUDO e envie.");
    Serial.println("3. Digite 'FIM' e envie.");
    Serial.println("--- PRONTO ---");

    bool recebendo = true;
    unsigned long ultimaAtividade = millis();

    while (recebendo) {
        if (Serial.available()) {
            char c = Serial.read();
            bufferRAM += c;
            ultimaAtividade = millis();
            
            if (bufferRAM.length() > 200000) {
                Serial.println("\nERRO: Buffer cheio!");
                recebendo = false;
            }
        }

        // Verifica FIM
        if (bufferRAM.endsWith("FIM") || bufferRAM.endsWith("FIM\n") || bufferRAM.endsWith("FIM\r\n")) {
            recebendo = false;
            int posFim = bufferRAM.lastIndexOf("FIM");
            if (posFim != -1) bufferRAM = bufferRAM.substring(0, posFim);
            Serial.println("\nComando FIM detectado.");
        }
        
        if (millis() - ultimaAtividade > 20000 && bufferRAM.length() > 0) {
             Serial.println("\nTimeout. Salvando...");
             recebendo = false;
        }
    }
    
    if (bufferRAM.indexOf('\n') == -1) {
        Serial.println("AVISO: Detectado texto em linha unica. Reparando formatacao...");
        
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
        Serial.print("Reparo concluido. Quebras de linha inseridas: ");
        Serial.println(mudancas);
    }

    // Gravação
    SD.remove(nomeArquivoHIL);
    File fileHIL = SD.open(nomeArquivoHIL, FILE_WRITE);
    if (fileHIL) {
        fileHIL.print(bufferRAM);
        fileHIL.flush();
        fileHIL.close();
        Serial.println("SUCESSO: Arquivo salvo.");
    } else {
        Serial.println("ERRO: Falha na gravacao.");
    }
    
    bufferRAM = ""; 
    Serial.println("==================================================");
}

/**
 * @brief Verifies and prints the content of the HIL simulation file stored on the SD card.
 * @details This function reads the first 100 bytes of the HIL simulation file
 * from the SD card and prints it to the Serial console for verification.
 **/
void checkContentFileHIL() {
    if (!ensureConnectionSD()) return;
    
    File f = SD.open("/Teste_HIL.csv", FILE_READ); // *Mudar depois para algo parametrizavel*
    if (!f) {
        Serial.println("Erro ao abrir arquivo.");
        return;
    }
    
    Serial.println("\n--- VERIFICANDO CONTEUDO (Primeiros 100 bytes) ---");
    int contador = 0;
    while (f.available() && contador < 100) {
        char c = f.read();
        if (c == '\n') Serial.print("[\\n]"); // Mostra visualmente onde tem enter
        else if (c == '\r') Serial.print("[\\r]");
        else Serial.print(c);
        contador++;
    }
    Serial.println("\n--------------------------------------------------");
    f.close();
}

/**
 * @brief Stops the HIL simulation and closes the associated file.
 */
void pararSimulacaoHIL() {
    if (gl_arquivoSimulacaoHIL) gl_arquivoSimulacaoHIL.close();
    gl_simulacaoHIL_ativa = false;
}

/**
 * @brief Resets the HIL simulation file to the beginning for a new run.
 */
void resetarSimulacaoHIL() {
    if (gl_arquivoSimulacaoHIL) {
        
        gl_arquivoSimulacaoHIL.seek(0);
        // Pula o header
        if (gl_arquivoSimulacaoHIL.available()) {
             gl_arquivoSimulacaoHIL.readStringUntil('\n'); 
        }
        DEBUG_PRINTLN_F("HIL: Arquivo rebobinado."); 
    }
}

/**
 * @brief Starts general logging.
 */
void iniciarLoggingGeral() {
    gl_loggingAtivo = true;
    DEBUG_PRINTLN_F("LOG: Iniciado.");
}

/**
 * @brief Stops the general logging.
 */
void pararLoggingGeral() {
    gl_loggingAtivo = false;
    finalizarSDCard();
    DEBUG_PRINTLN_F("LOG: Parado.");
}

/**
 * @brief Checks if general logging is active.
 * @return true if logging is active, false otherwise.
 */
bool isLoggingDeDadosAtivo() {
    return gl_loggingAtivo;
}