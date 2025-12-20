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
#include "Funcoes_suporte_IMU.h" // Para setup_IMU() e outras funcoes
#include "Funcoes_BMP.h"         // Para setupBMP() e lerAltitudeDoBMP280()
#include "Logica_voo.h"           // Para detectarLancamento(), lerTilt(), atuarAirbrakes(), etc.
#include "KalmanFilter.hh"
#include "Controller.hh"
#include "AltitudeSpeedTable.hh"
#include "DragCoefficientTable.hh"
#include <ArduinoEigenDense.h>   // Para Eigen::MatrixXd
#include "Sinalizacao.h"
#include "Config_voo.h"
#include "DataManager.h"
#define EEPROM_SIZE 256 // Definir o tamanho do EEPROM

// --- Variáveis Globais do Sketch Principal ---
bool calibrate_imu_on_startup = true;                     // Calibrar IMU na inicialização
bool print_imu_params = false;        
bool perform_fine_tuning = false;                          // Ajuste fino do acelererometro e giroscopio 
const unsigned long Ts_ms = 20.0;                         // Tempo de loop em ms (50Hz)
float Ts = (float)Ts_ms / 1000.0f;                        // Tempo de amostragem em segundos
unsigned long tempoAnteriorLoop = 0;                      // Armazena o tempo em que o último ciclo foi executado
const unsigned long MAX_TEMPO_ESPERA_POUSO = 600000;      // 10 minutos 
bool comunicacaoSerial = true;                            // Ativa funcoes de comunicacao serial
const float G_CONSTANTE_GRAVITACIONAL_MS2 = 9.80665f;     // Aceleração gravidade m/s^2
const u_int8_t WDT_TIMEOUT_S = 5;                        // Watchdog timeout in seconds

// Flight State & Data
RawFlightData flightData; // Struct from DataManager.h
HILSimulationData hilData;

// --- FLAG PARA CONTROLAR O MODO HIL ---
const bool HIL_MODE_ACTIVE = true; // true - Ativa o modo HIL | false - Voo real
const char* HIL_FILENAME = "/Teste_HIL.csv";

// Matrizes e Objeto para o Filtro de Kalman
Eigen::Matrix<float, 2, 2> F_kf;
Eigen::Matrix<float, 2, 1> G_kf;
Eigen::Matrix<float, 2, 2> H_kf;
Eigen::Matrix<float, 2, 2> Q_kf;
Eigen::Matrix<float, 2, 2> R_kf;
Eigen::Matrix<float, 2, 2> P0_kf;
Eigen::Matrix<float, 2, 1> X0_kf;
KalmanFilter kf;

// Variáveis para armazenar o estado estimado pelo Kalman
float filteredAltitude_m = 0.0;
float filteredVerticalVelocity_ms = 0.0;
float netVerticalAcceleration_ms2 = 0.0; 
float barometricPressure_Pa = 0.0f;
float tiltAtual_deg = 0.0;
float delta_V_ms = 0.0;
float controlGain1 = 0.0;
float controlGain2 = 0.0;
float controlInput = 0.0;
float deflexaoCalculada = 0.0;

// Variáveis da Máquina de Estados
enum class FaseFoguete {
    CALIBRACAO_SENSORES,
    CHECAGEM_SAUDE,
    ESPERA_LANCAMENTO,
    VOO,
    BURNOUT,
    ATUACAO_AIRBRAKES, 
    APOGEU,
    SALVAR_DADOS,
    POUSO
};

FaseFoguete flightState = FaseFoguete::CALIBRACAO_SENSORES;
unsigned long tempoEntradaNoEstado = 0;
unsigned long tempoLancamentoDetectado = 0; 
unsigned long tempoApogeuDetectado = 0;

// --- Funções de Configuração ---
void setupKalman() {
    

    F_kf << 1.0f, Ts,
            0.0f, 1.0f;

    G_kf << 0.5 * Ts * Ts,
            Ts;

    H_kf << 1, 0
          , 0, 1; // velocidade do ZUKF

    // Ajuste Q e R conforme a confiança nos seus modelos e sensores
    float var_proc_pos = 1.0; 
    float var_proc_vel = 3.0;  
    Q_kf << var_proc_pos*(Ts*Ts*Ts*Ts)/4.0, var_proc_pos*(Ts*Ts*Ts)/2,
            var_proc_pos*(Ts*Ts*Ts)/2, var_proc_vel*Ts*Ts;

    float var_med_alt = 1.0; // Ex: desvio padrão de sqr(5)m para o barômetro (1m^2)
    float var_zupt_vel = 0.000001; // Variância muito baixa para velocidade durante ZUPT

    R_kf << var_med_alt, 0,
            0, var_zupt_vel;  

    P0_kf << 1, 0,  // Incerteza na altitude
             0, 1;  // Incerteza na velocidade

    X0_kf << 0, 0; // Altitude AGL inicial e velocidade vertical inicial

    kf.init(F_kf, G_kf, H_kf, Q_kf, R_kf, P0_kf, X0_kf);
    DEBUG_PRINTLN_F("Filtro de Kalman inicializado.");
}

// Valores dos ganhos ajustados com base em simulacoes
float Kp = 0.025;
float Ki = 0.075;
float Kd = 0.02;

float mass = 30.605; // Massa do foguete (Mudar conforme a massa real )
float area = 0.02097; // Area total das petalas
Controller controller = Controller(Kp, Ki, Kd, mass, area, Ts);

void setupController() {
    controller.setLimits(0, 1);
    DEBUG_PRINTLN_F("Controlador inicializado.");
}

// uint32_t tempokalman = 0;
// uint32_t tempoorientacao = 0;

void atualizarEstimativaDeEstadoCompleta() {
    // tempokalman = millis();
    // tempoorientacao = millis();
    if (HIL_MODE_ACTIVE) {
        // --- Modo HIL: Pega dados do arquivo CSV ---
        hilData = DataManager::getInstance().readHILStep();
        
        if (!hilData.dadosValidos) {
            // Fim da simulação ou erro de leitura.
            if (DataManager::getInstance().isLoggingActive()) {
                DataManager::getInstance().stopLogging();
                DEBUG_PRINTLN_F("Fim da simulacao HIL. Logging parado.");
            }
            return; 
        }
        
        // Usa os dados do arquivo CSV
        netVerticalAcceleration_ms2 = hilData.aceleracaoLiquida_ms2;
        barometricPressure_Pa = hilData.pressao_Pa;
        // No modo HIL, não temos dados de sensor para tilt real 
        tiltAtual_deg = 90 - hilData.tilt; // Converter para ângulo de tilt usado no código
    } else {
        // --- Modo Real: Pega dados dos sensores ---
        if (!mpu.update()) {
            DEBUG_PRINTLN_F("Falha ao atualizar MPU!");
            return; // Sai se não conseguir ler a IMU
        }
        
        netVerticalAcceleration_ms2 = calcularAceleracaoZVerticalLiquida();
        // DEBUG_PRINT_F("Aceleracao Z Vertical Liquida calculada: ");
        // DEBUG_PRINTLN(millis() - tempoorientacao);
        barometricPressure_Pa = getPressaoBMPAtual(); 
        tiltAtual_deg = readCurrentTilt();
    }

    // Executar o Filtro de Kalman 
    
    // Entrada de Controle U para a etapa de Predição
    Eigen::Matrix<float, 1, 1> U_kf;
    U_kf << netVerticalAcceleration_ms2;
    
    // Etapa de Predição
    kf.Predict(U_kf);

    // Medição Z para a etapa de Atualização
    float altitudeMedida = altitudeFromPressure(barometricPressure_Pa);
    
    if (altitudeMedida > -9000.0f) { // Checa por valor de erro de altitudeFromPressure
        Eigen::Matrix<float, 2, 1> Z_kf;
        Z_kf << altitudeMedida, 0.0f; // Velocidade mandada como zero para ZUPT
        kf.Update(Z_kf, R_kf);
    } else {
        DEBUG_PRINTLN_F("AVISO: Medicao de altitude invalida. Kalman Update pulado.");
        // Se a medição falhar, o filtro prosseguirá usando apenas a predição.
    }

    // Obter saídas do filtro
    Matrix<float, 2, 1> estadoEstimado = kf.getPosterioriState();
    filteredAltitude_m = estadoEstimado(0, 0);
    filteredVerticalVelocity_ms = estadoEstimado(1, 0);
    // DEBUG_PRINT_F("Estimativa de estado atualizada pelo Kalman:");
    // DEBUG_PRINTLN(millis() - tempokalman);
}

void atualizarLogger(){
    // Preencher a struct DadosVooBrutosParaLog
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
    flightData.filteredAltitude = filteredAltitude_m; // Variavel global
    flightData.filteredVerticalVelocity = filteredVerticalVelocity_ms; // Variavel global
    flightData.netVerticalAcceleration = netVerticalAcceleration_ms2; // Variavel global
    flightData.tilt = tiltAtual_deg; // lertiltAtual()
    flightData.barometricPressure = barometricPressure_Pa;
    // flightData.barometricPressure =  getGroundPressureP0_BMP(); // Somente para testes sem o BMP
    flightData.servoAtuacao_percent = deflexaoCalculada; // Variavel global
    flightData.gain1 = controlGain1; // Variavel global
    flightData.gain2 = controlGain2; // Variavel global
    flightData.flightState = int(flightState);
}

// --- Funções da Máquina de Estados ---

void loopCalibracaoSensores() {
    DEBUG_PRINTLN_F("ESTADO: Calibracao Sensores");
    bool imuCalibrada = hasCalibrationData(); 
    bool bmpCalibrado = (getGroundPressureP0_BMP() != 101324.0f); // Checagem simples se P0 foi atualizado

    if (imuCalibrada && bmpCalibrado) { 
        DEBUG_PRINTLN_F("Calibracoes iniciais (IMU/BMP P0) concluidas no setup.");
        flightState = FaseFoguete::CHECAGEM_SAUDE;
        tempoEntradaNoEstado = millis();
    }
}

static int contagemChecksSaudeOK_Atual = 0; 
const int NUM_CHECKS_SAUDE_NECESSARIOS = 5; 

void loopChecagemSaude() {
    DEBUG_PRINTLN_F("State: Health Check");

    atualizarEstimativaDeEstadoCompleta();
    recalibrarPressaoDeSoloBMP(); 

    if (checkFlightSystemHealth(filteredAltitude_m, filteredVerticalVelocity_ms)) { 
        contagemChecksSaudeOK_Atual++;
        DEBUG_PRINT_F("Saude dos componentes OK nesta iteracao. Contagem: ");
        DEBUG_PRINTLN(contagemChecksSaudeOK_Atual);

        if (contagemChecksSaudeOK_Atual >= NUM_CHECKS_SAUDE_NECESSARIOS) {
            Serial.println("CHECAGEM DE SAUDE: Verificacoes consecutivas OK. Transicionando...");
            sinalizarSucessoModulo("Checagem Saude Geral"); 

            flightState = FaseFoguete::ESPERA_LANCAMENTO;
            DataManager::getInstance().startLogging(); // Ativa o data logging
        
            // Configurações do sistema para espera de lançamento
            DataManager::getInstance().setDecimationFactor(10); // Salva a cada 10x20ms = 200ms
            setDriftLearning(true); // Habilita aprendizado de drift na IMU
            setFilterBeta(30.0f); // Aumenta o ganho do filtro para resposta mais rápida durante a espera
            R_kf(1,1) = 0.000001f; // Reduz a variância da medição de velocidade para ZUPT mais forte

            tempoEntradaNoEstado = millis();         
            contagemChecksSaudeOK_Atual = 0;    
        }
    } else {
        DEBUG_PRINTLN_F("ERRO na saude dos componentes detectado por checarSaudeLogicaVoo()!");
        sinalizarFalhaModulo("Checagem Saude Iteracao"); 
        contagemChecksSaudeOK_Atual = 0; // Reset da contagem se qualquer verificação falhar   
    }
}

void loopEsperaLancamento() {

    // Atualiza continuamente a referência de pressão do solo para compensar o drift
    recalibrarPressaoDeSoloBMP();
    atualizarEstimativaDeEstadoCompleta();
    DataManager& logger = DataManager::getInstance();

    DEBUG_PRINT_F("ESTADO: Espera Lancamento | Alt: ");
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
        atualizarLogger();
        logger.logDataSD(flightData); 
    }

    // Lógica de detecção de lançamento 
    if (detectLaunch(netVerticalAcceleration_ms2,filteredAltitude_m)) { 
        Serial.println("LANCAMENTO DETECTADO!");
        
        // Configurações do sistema para o voo
        setDriftLearning(false); // Desabilita aprendizado de drift na IMU após o lançamento
        setFilterBeta(3.0f);
        R_kf(1,1) = 1000000000.0f; // Coloca alta variância na medição de velocidade para ignorar a entrada 0 durante o voo
        logger.setDecimationFactor(1); // Salva todo ciclo de 20ms durante o voo

        flightState = FaseFoguete::VOO;
        tempoEntradaNoEstado = millis();
        tempoLancamentoDetectado = millis(); 
    }
}

void loopVoo() {
    DEBUG_PRINT_F("ESTADO: Voo | Alt: ");
    DEBUG_PRINT(filteredAltitude_m);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(filteredVerticalVelocity_ms);
    DEBUG_PRINTLN_F("m/s");
    DataManager& logger = DataManager::getInstance();
    atualizarEstimativaDeEstadoCompleta();

    if (logger.isLoggingActive()) {
        atualizarLogger();
        logger.logDataSD(flightData); 
    }

    unsigned long tempoDesdeLancamento = millis() - tempoLancamentoDetectado;
    if (detectBurnout(netVerticalAcceleration_ms2, tempoDesdeLancamento)) { 
        Serial.println("BURNOUT DETECTADO!");
        flightState = FaseFoguete::BURNOUT; 
        tempoEntradaNoEstado = millis();
    }
}

void loopBurnout() {
    DEBUG_PRINT_F("ESTADO: Burnout | Alt: ");
    DEBUG_PRINT(filteredAltitude_m);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(filteredVerticalVelocity_ms);
    DEBUG_PRINTLN_F("m/s");
    DataManager& logger = DataManager::getInstance();

    atualizarEstimativaDeEstadoCompleta();
    if (logger.isLoggingActive()) {
        atualizarLogger();
        logger.logDataSD(flightData); 
    }
    if (detectAirbrakesActuation(filteredAltitude_m, filteredVerticalVelocity_ms)) {
        Serial.println("Velocidade adequada, iniciando atuacao dos airbrakes.");
        flightState = FaseFoguete::ATUACAO_AIRBRAKES;
        tempoEntradaNoEstado = millis();
    }
}

void loopAtuacaoAirbrakes() {
    DEBUG_PRINT_F("ESTADO: Atuacao Airbrakes | Alt: ");
    DEBUG_PRINT(filteredAltitude_m);
    DEBUG_PRINT_F("m | VelZ: ");
    DEBUG_PRINT(filteredVerticalVelocity_ms);
    DEBUG_PRINT_F("m/s | Tilt: ");
    DEBUG_PRINT(tiltAtual_deg);
    DEBUG_PRINT_F("° | Deflexao Calculada: ");
    DEBUG_PRINTLN(deflexaoCalculada);
    DataManager& logger = DataManager::getInstance();

    atualizarEstimativaDeEstadoCompleta(); 

    if (logger.isLoggingActive()) {
        atualizarLogger();
        logger.logDataSD(flightData); 
    }

    // LÓGICA DE CONTROLE DOS AIRBRAKES
    delta_V_ms = lookUpSpeed(filteredAltitude_m) - filteredVerticalVelocity_ms;
    controlGain1 = controller.computePID(0, delta_V_ms);
    controlGain2 = controller.computeCd(3260, filteredAltitude_m, filteredVerticalVelocity_ms, -G_CONSTANTE_GRAVITACIONAL_MS2, 1.293);
    controlInput = controlGain1 + controlGain2;

    if (tiltAtual_deg < 20){
        // Converte velocidade para mach ---> vel/335

        deflexaoCalculada = getNearestActuation((filteredVerticalVelocity_ms/335), controlInput); 
    }
    else{
        deflexaoCalculada = 0.0; 
    }

    commandAirbrakes(deflexaoCalculada);

    if (detectApogee(filteredVerticalVelocity_ms, filteredAltitude_m)) { 
        Serial.println("APOGEU DETECTADO!");
        flightState = FaseFoguete::APOGEU;
        tempoEntradaNoEstado = millis();
        tempoApogeuDetectado = millis();
    }
}

void loopApogeu() {
    DEBUG_PRINT_F("ESTADO: Apogeu | Altura Max: ");
    DEBUG_PRINT(filteredAltitude_m);
    DEBUG_PRINTLN_F("m");
    atualizarEstimativaDeEstadoCompleta(); 

    DataManager& logger = DataManager::getInstance();
    if (logger.isLoggingActive()) {
        atualizarLogger();
        logger.logDataSD(flightData); 
    }
    retractAirbrakes(); 
    Serial.println("Airbrakes retraidos no apogeu.");
    
    flightState = FaseFoguete::SALVAR_DADOS; 
    logger.setDecimationFactor(10); // Salva a cada 10x20ms = 200ms
    tempoEntradaNoEstado = millis();
}

// Representa o estado da descida do foguete
void loopSalvarDados() {

    if (HIL_MODE_ACTIVE == false) {
        DEBUG_PRINT_F("ESTADO: Salvar dados | Alt: ");
        DEBUG_PRINT(filteredAltitude_m);
        DEBUG_PRINT_F("m | VelZ:  ");
        DEBUG_PRINT(filteredVerticalVelocity_ms);
        DEBUG_PRINTLN_F("m/s");
    }
 

    atualizarEstimativaDeEstadoCompleta();
    DataManager& logger = DataManager::getInstance();
    if (logger.isLoggingActive()) {
        atualizarLogger();
        logger.logDataSD(flightData); 
    }

    unsigned long tempoDesdeApogeu = millis() - tempoApogeuDetectado;
    if (detectLanding(filteredVerticalVelocity_ms, filteredAltitude_m, tempoDesdeApogeu)) { 
        Serial.println("POUSO DETECTADO!");
        flightState = FaseFoguete::POUSO;
        logger.setDecimationFactor(50); // Salva a cada 50x20ms = 1s
        tempoEntradaNoEstado = millis();
    }

    // Timeout para pouso se não detectado após muito tempo
    else if (tempoDesdeApogeu > MAX_TEMPO_ESPERA_POUSO) {
       Serial.println("Timeout para detecção de pouso.");
       flightState = FaseFoguete::POUSO; // Força o estado de pouso
       logger.setDecimationFactor(50); // Salva a cada 50x20ms = 1s
       tempoEntradaNoEstado = millis();
    }
}

void loopPouso() {
    DEBUG_PRINTLN_F("ESTADO: Pouso");
    // Continuar salvando dados finais por um curto período (120000ms - 2min)
    DataManager& logger = DataManager::getInstance();
    if (logger.isLoggingActive() && (millis() - tempoEntradaNoEstado)< 120000 ) {
        atualizarLogger();
        logger.logDataSD(flightData); 
    }
    else{
        logger.stopLogging(); // Desliga o data logging
    }
    DEBUG_PRINTLN_F("Operacao finalizada. Aguardando recuperacao.");
    delay(10000);
}

// --- SETUP E LOOP PRINCIPAIS ---
void setup() {
    Serial.begin(115200);
    // Espera o Serial conectar, com timeout
    unsigned long HORA_INICIO_SERIAL = millis();
    while (!Serial && (millis() - HORA_INICIO_SERIAL < 4000));

    
    if (!EEPROM.begin(EEPROM_SIZE)) {
        DEBUG_PRINTLN_F("ERRO CRITICO: Falha ao iniciar a EEPROM!");
    } else {
        DEBUG_PRINTLN_F("EEPROM Inicializada com sucesso.");
    }

    // Inicializa o básico: I2C, SPI e sistema de sinalização
    Wire.begin();
    Wire.setClock(400000); // Configura I2C para 400kHz - testar valores diferentes se necessário ---> comprimento do fio influencia
    
    SPI.begin();
    setupSinalizacao();
    sinalizarInicioStartup();

    // eraseCalibration(); // Primeira vez de calibracao 

    DEBUG_PRINTLN_F("==== INITIALIZING AIRBRAKE SYSTEM ====");

    // Setup do Armazenamento 
    DEBUG_PRINTLN_F("Initializing SD card...");
    DataManager& logger = DataManager::getInstance();
    if (!logger.setupSD()) {
        sinalizarFalhaModulo("Log SD Card Setup");
        DEBUG_PRINTLN_F("FATAL ERRO: Failed to initialize SD card.");
        buzzerBeeps(10,300,150, 1200);
    } else {
        sinalizarSucessoModulo("Log SD Card Setup");
    }

    // Setup dos Sensores ou do HIL
    if (HIL_MODE_ACTIVE) {
        DEBUG_PRINTLN_F("**** MODO HIL ATIVADO ****");
        if (logger.initHIL(HIL_FILENAME)) {
            
            
            HILSimulationData firstSample = logger.readHILStep();
            
            if (firstSample.dadosValidos) {
                float pressaoInicial = firstSample.pressao_Pa;
                setGroundPressureP0_BMP(pressaoInicial);

                DEBUG_PRINT_F("HIL: P0 set to: ");
                DEBUG_PRINT(pressaoInicial);
                DEBUG_PRINTLN_F(" Pa");
                
                logger.resetHIL(); 
                
            } else {
                DEBUG_PRINTLN_F("HIL: Error - Empty file!");
                while(1); 
            }
            
            sinalizarSucessoModulo("HIL Init");
            
        } else {
            DEBUG_PRINTLN_F("HIL ERRO FATAL: Arquivo nao encontrado!");
            while(1);
        }

    } else {
        DEBUG_PRINTLN_F("**** MODO DE VOO REAL ATIVADO ****");
        if (!setup_IMU(calibrate_imu_on_startup, perform_fine_tuning, print_imu_params)) {
            sinalizarFalhaModulo("IMU");
            DEBUG_PRINTLN_F("ERRO FATAL: Setup da IMU falhou. Sistema travado.");
            while (1) { ledBlink(PINO_LED_STATUS_2, 1, 500, 500); delay(100); }
        } else {
            sinalizarSucessoModulo("IMU");
        }

        DEBUG_PRINTLN_F("Inicializando BMP (setup_BMP)...");
        if (!setupBMP()) { 
            sinalizarFalhaModulo("BMP280");
            DEBUG_PRINTLN_F("ERRO FATAL: Setup do BMP280 falhou. Sistema travado.");
            while (1) { ledBlink(PINO_LED_STATUS_2, 1, 500, 500); delay(100); }
        } else {
            sinalizarSucessoModulo("BMP280");
        }
    }
     
    if (setupServo()) { 
        sinalizarSucessoModulo("Servo");
        retractAirbrakes(); 
    } else {
        sinalizarFalhaModulo("Servo");
        DEBUG_PRINTLN_F("ERRO FATAL: Setup do Servo falhou.");
        while (1) { ledBlink(PINO_LED_STATUS_2, 1, 250, 250); delay(100); }
    }

    delay(500); // Tempo para estabilizacao dos dados
    
    DEBUG_PRINTLN_F("Todos os setups e testes opcionais concluidos.");
    sinalizarSistemaPronto(); 

    DEBUG_PRINTLN_F("Inicializando Controlador...");
    setupController(); 
    sinalizarSucessoModulo("Controlador");
    delay(100);

    // Setup do Kalman e do Controlador 
    DEBUG_PRINTLN_F("Inicializando Kalman Filter...");
    setupKalman(); 

    Serial.flush();

    tempoEntradaNoEstado = millis();
    tempoAnteriorLoop = millis();
    if (!HIL_MODE_ACTIVE){
        flightState = FaseFoguete::CHECAGEM_SAUDE; 
    }
    else {
        flightState =  FaseFoguete::ESPERA_LANCAMENTO; 
        logger.startLogging();
    }

    DEBUG_PRINTLN("WDT: Initializing Watchdog...");
    esp_task_wdt_init(WDT_TIMEOUT_S, true); // Tempo de timeout de 3 segundos
    esp_task_wdt_add(NULL);; // Adiciona a tarefa atual (loop principal) ao WDT
    DEBUG_PRINTLN("WDT: Active.");

    DEBUG_PRINT_F("Transitioning to initial state: "); 
    DEBUG_PRINTLN((int)flightState); 
}

void loop() {
    // Pega o tempo atual no início de cada passagem pelo loop
    unsigned long tempoAtual = millis();

    if (tempoAtual - tempoAnteriorLoop >= Ts_ms) {
        tempoAnteriorLoop += Ts_ms;
        
        switch (flightState) {
            case FaseFoguete::CALIBRACAO_SENSORES:
                loopCalibracaoSensores();
                break;
            case FaseFoguete::CHECAGEM_SAUDE:
                loopChecagemSaude();
                break;
            case FaseFoguete::ESPERA_LANCAMENTO:
                loopEsperaLancamento();
                break;
            case FaseFoguete::VOO:
                loopVoo();
                break;
            case FaseFoguete::BURNOUT:
                loopBurnout();
                break;
            case FaseFoguete::ATUACAO_AIRBRAKES:
                loopAtuacaoAirbrakes();
                break;
            case FaseFoguete::APOGEU:
                loopApogeu();
                break;
            case FaseFoguete::SALVAR_DADOS:
                loopSalvarDados();
                break;
            case FaseFoguete::POUSO:
                loopPouso();
                break;
            default:
                DEBUG_PRINTLN_F("ERRO: Estado desconhecido! Reiniciando...");
                flightState = FaseFoguete::CHECAGEM_SAUDE;
                tempoEntradaNoEstado = millis();
                break;
            
        }
        
        // DEBUG_PRINT(millis() - tempoAnteriorLoop);
        // DEBUG_PRINTLN_F(" ms para executar!");
        // Serial.print(millis() - tempoAnteriorLoop);
        // Serial.println(" ms para executar!");

        if (millis() > tempoAnteriorLoop + Ts_ms) { // Verifica se a execução atual já "atrasou" o próximo ciclo
            DEBUG_PRINT_F("WARNING: Main loop overrun! Execution time: ");
            DEBUG_PRINT(millis() - tempoAnteriorLoop);
            DEBUG_PRINTLN_F(" ms!");
        }
    }
    if (comunicacaoSerial){
        if (Serial.available()) {
        char cmd = Serial.read();
        DataManager& logger = DataManager::getInstance();
        esp_task_wdt_reset();
        
            if (cmd == 'd' || cmd == 'D') { // 'd' de Dump 
                Serial.println("Comando recebido: Despejar Log");
                logger.dumpCurrentLog();
                logger.stopLogging(); // Fecha logs ativos para segurança
            }
            if (cmd == 'h' || cmd == 'H') { // 'H' de HIL
                // Para o loop de voo momentaneamente para receber o arquivo
                logger.stopLogging(); // Fecha logs ativos para segurança
                logger.receiveHILFile(HIL_FILENAME);
            }
            
            if (cmd == 'l' || cmd == 'L') { // 'l' de Listar
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
    esp_task_wdt_reset(); // Reseta o watchdog timer
}