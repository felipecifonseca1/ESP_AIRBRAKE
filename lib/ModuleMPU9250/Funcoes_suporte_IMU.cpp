#include "Funcoes_suporte_IMU.h"
#include <Wire.h>
#include "MPU9250.h" 
#include <EEPROM.h>
#include "Config_voo.h"  
#include <Arduino.h>
#include "Sinalizacao.h"


MPU9250 mpu;  // Declaração de objeto global
MPU9250Setting mpuConfig;

const float ACCEL_CALIB_SENSITIVITY_FS = 16384.0f; // LSB/g (para FS = +/-2g)
const float GYRO_CALIB_SENSITIVITY_FS  = 131.0f;   // LSB/(deg/s) (para FS = +/-250dps)
bool z_axis_up = true; // Define se o z do sensor esta para cima ou para baixo (depende da montagem física)

/**
 * @brief Salva os parâmetros de calibração atuais na EEPROM.
 * @details Armazena os Biases do acelerómetro, giroscópio e magnetómetro.
 * Isso permite que o foguete "lembre" da calibração mesmo após desligar a bateria.
 * * @param print Se true, imprime os valores salvos no Monitor Serial para debug.
 */
void saveCalibration(bool print) {
  float accel_bias[] = {mpu.getAccBiasX(), mpu.getAccBiasY(), mpu.getAccBiasZ()};
  float gyro_bias[]  = {mpu.getGyroBiasX(), mpu.getGyroBiasY(), mpu.getGyroBiasZ()};
  float mag_bias[]   = {mpu.getMagBiasX(), mpu.getMagBiasY(), mpu.getMagBiasZ()};
  float mag_scale[]  = {mpu.getMagScaleX(), mpu.getMagScaleY(), mpu.getMagScaleZ()};

  EEPROM.put(0, accel_bias);
  EEPROM.put(sizeof(accel_bias), gyro_bias);
  EEPROM.put(sizeof(accel_bias) + sizeof(gyro_bias), mag_bias);
  EEPROM.put(sizeof(accel_bias) + sizeof(gyro_bias) + sizeof(mag_bias), mag_scale);

  if (EEPROM.commit()) {
      if (print) {
        DEBUG_PRINTLN_F("--- CALIBRACAO SALVA NA EEPROM ---");
        print_vector("Acc Bias", accel_bias);
        print_vector("Gyro Bias", gyro_bias);
        print_vector("Mag Bias", mag_bias);
        print_vector("Mag Scale", mag_scale);}
  } else {
      DEBUG_PRINTLN("ERRO: Falha ao commitar dados na EEPROM!");
  }
  // --------------------------
}

/**
 * @brief Apaga os dados de calibração da EEPROM.
 * @details Escreve zeros na memória para forçar uma nova calibração na proxima vez que for inicializado.
 */
void eraseCalibration() {
  float empty_data[3] = {NAN, NAN, NAN};  // Usa NaN para indicar valores não iniciais

  EEPROM.put(0, empty_data);  // Apaga accel_bias
  EEPROM.put(sizeof(empty_data), empty_data);  // Apaga gyro_bias
  EEPROM.put(sizeof(empty_data) * 2, empty_data);  // Apaga mag_bias
  EEPROM.put(sizeof(empty_data) * 3, empty_data);  // Apaga mag_scale


  if (EEPROM.commit()) {
      DEBUG_PRINTLN("Calibração apagada da Flash (Commit realizado).");
  } else {
      DEBUG_PRINTLN("ERRO: Falha ao apagar EEPROM!");
  }
}

/**
 * @brief Carrega os parâmetros de calibração da EEPROM para o sensor.
 * @details Lê os valores salvos e aplica-os ao objeto 'mpu', evitando
 * ter que recalibrar o foguete no pad de lançamento.
 * * @param print Se true, imprime os valores carregados.
 */
void loadCalibration(bool print) {
  float accel_bias[3], gyro_bias[3], mag_bias[3], mag_scale[3];

  EEPROM.get(0, accel_bias);
  EEPROM.get(sizeof(accel_bias), gyro_bias);
  EEPROM.get(sizeof(accel_bias) + sizeof(gyro_bias), mag_bias);
  EEPROM.get(sizeof(accel_bias) + sizeof(gyro_bias) + sizeof(mag_bias), mag_scale);

  // Aplica ao Driver do MPU9250
  mpu.setAccBias(accel_bias[0], accel_bias[1], accel_bias[2]);
  mpu.setGyroBias(gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  mpu.setMagBias(mag_bias[0], mag_bias[1], mag_bias[2]);
  mpu.setMagScale(mag_scale[0], mag_scale[1], mag_scale[2]);

  if (print){
    DEBUG_PRINTLN("Calibração carregada da EEPROM");

    // Mostra na tela os parametros de calibração
    DEBUG_PRINTLN("< Parâmetros de calibracao >");
    print_vector("accel bias [mili_g]", accel_bias, 1000.0 / MPU9250::CALIB_ACCEL_SENSITIVITY );
    print_vector("gyro bias [deg/s]", gyro_bias, 1.0 / MPU9250::CALIB_GYRO_SENSITIVITY);
    print_vector("mag bias [mG]", mag_bias, 1);
    print_vector("mag scale []", mag_scale, 1);
  }
}

/**
 * @brief Verifica se existe uma calibração válida salva.
 * @return true se a assinatura (byte 123) for encontrada.
 * @return false se a memória estiver virgem ou corrompida.
 */
bool hasCalibrationData() {
    float test_value;
    EEPROM.get(0, test_value);
    return !isnan(test_value);  
}

/**
 * @brief Função auxiliar para imprimir vetores no Serial.
 */
void print_vector(const char* label, const float values[], float scale, int decimals, const char* unit_label) {
    DEBUG_PRINT_F(label); 
    DEBUG_PRINT_F(": ");
    for (int i = 0; i < 3; i++) {
        if (isnan(values[i])) {
            DEBUG_PRINT_F("nan");
        } else {
            DEBUG_PRINT(values[i] * scale, decimals);
        }
        if (i < 2) {
            DEBUG_PRINT_F(", ");
        }
    }
    if (unit_label != nullptr && strlen(unit_label) > 0) {
        DEBUG_PRINT_F(" [");
        DEBUG_PRINT(unit_label); 
        DEBUG_PRINT_F("]");
    }
    DEBUG_PRINTLN_F(""); 
}

/**
 * @brief Escreve um byte num registrador I2C específico.
 * Utilizado para configurações de baixo nível se a biblioteca não suportar.
 */
void write_byte_local(uint8_t address, uint8_t subAddress, uint8_t data) {
     Wire.beginTransmission(address);
     Wire.write(subAddress);
     Wire.write(data);
     Wire.endTransmission();
}

/**
 * @brief Realiza leitura de N amostras para determinar o bias médio.
 * Usado pelo algoritmo de calibração iterativa.
 */
void test_bias_for_adjust(int samples, float result_accel_g[3], float result_gyro_dps[3]) {
    static bool headerPrinted_test_bias = false; // Para imprimir o cabeçalho apenas uma vez por execução do sketch
    
    // Imprime o cabeçalho para os dados brutos se for a primeira vez e houver amostras a coletar
    if (!headerPrinted_test_bias && samples > 0) {
        DEBUG_PRINT_F("Time_ms,RawErrAx_g,RawErrAy_g,RawErrAz_g,RawErrGx_dps,RawErrGy_dps,RawErrGz_dps\n");
        headerPrinted_test_bias = true;
    }
    if (samples == 0) { // Se nenhuma amostra for solicitada, zera os resultados e retorna
        for(int i=0; i<3; ++i) { result_accel_g[i] = 0.0f; result_gyro_dps[i] = 0.0f; }
        return;
    }

    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    
    uint32_t prev_time_micros = micros();
    const uint32_t sample_interval_micros = 8000; 

    if (!mpu.update()) {
        DEBUG_PRINTLN_F("FUNCOES_IMU (test_bias_for_adjust): Falha na primeira chamada a mpu.update().");
        // Zera os resultados em caso de falha ao atualizar o sensor
        for(int i=0; i<3; ++i) { result_accel_g[i] = 0.0f; result_gyro_dps[i] = 0.0f; }
        return;
    }

    for (int i = 0; i < samples; i++) {
        while (micros() - prev_time_micros < sample_interval_micros) {
        }
        prev_time_micros = micros(); 

        if (mpu.update()) { 
            accel_sum[0] += mpu.getAccX(); // Unidade: 'g'
            accel_sum[1] += mpu.getAccY(); // Unidade: 'g'
            accel_sum[2] += mpu.getAccZ(); // Unidade: 'g' 
            
            gyro_sum[0] += mpu.getGyroX(); // Unidade: 'dps'
            gyro_sum[1] += mpu.getGyroY(); // Unidade: 'dps'
            gyro_sum[2] += mpu.getGyroZ(); // Unidade: 'dps'
            
            // Compensação da gravidade
            accel_sum[2] -= 1.0f; 
        } else {
            DEBUG_PRINTLN_F("FUNCOES_IMU (test_bias_for_adjust): Falha mpu.update() dentro do loop de amostragem.");
            i--; 
            delay(1); 
        }
    }
    // Calcula as médias para obter os erros de bias
    for (uint8_t axis = 0; axis < 3; axis++) {
        result_accel_g[axis] = accel_sum[axis] / samples;
        result_gyro_dps[axis] = gyro_sum[axis] / samples;
    }

    DEBUG_PRINT(millis()); 
    DEBUG_PRINT_F(",");
    for (int i = 0; i < 3; i++) { 
        DEBUG_PRINT(result_accel_g[i], 5); 
        DEBUG_PRINT_F(","); 
    }
    for (int i = 0; i < 3; i++) { 
        DEBUG_PRINT(result_gyro_dps[i], 5); 
        if (i < 2) DEBUG_PRINT_F(",");
    }
    DEBUG_PRINTLN_F(""); 
}

/**
 * @brief Algoritmo iterativo para refinar a calibração automaticamente.
 * @param samples_per_iteration Número de amostras para coletar em cada iteração.
 * @param print_debug Se true, imprime informações detalhadas no Serial.
 * @param accel_tol_g Tolerância para o erro do acelerómetro em 'g'.
 * @param gyro_tol_dps Tolerância para o erro do giroscópio em 'deg/s'.
 * @param max_iter Número máximo de iterações para evitar loops infinitos.
 * @details Mede o erro residual e ajusta os offsets usando um ganho proporcional.
 * @return true Se a calibração convergiu dentro das tolerâncias.
 */
bool adjustCalibrationIteratively(int samples_per_iteration, bool print_debug, float accel_tol_g, float gyro_tol_dps, int max_iter) {
    DEBUG_PRINTLN_F("FUNCOES_IMU: Iniciando ajuste fino iterativo...");
    bool calibrated = false;

    // A configuração para calibração fina deve ser a mais sensível
    MPU9250Setting calibFineTuneConfig;
    calibFineTuneConfig.accel_fs_sel = ACCEL_FS_SEL::A2G;
    calibFineTuneConfig.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
    // Mantenha as outras configurações iguais às de voo
    calibFineTuneConfig.fifo_sample_rate = mpuConfig.fifo_sample_rate;
    calibFineTuneConfig.gyro_dlpf_cfg = mpuConfig.gyro_dlpf_cfg;
    calibFineTuneConfig.accel_dlpf_cfg = mpuConfig.accel_dlpf_cfg;
    calibFineTuneConfig.gyro_fchoice = mpuConfig.gyro_fchoice;
    calibFineTuneConfig.accel_fchoice = mpuConfig.accel_fchoice;

    for (int iter = 0; iter < max_iter; ++iter) {
        // Garante que o MPU está na configuração de alta sensibilidade 
        if (!mpu.setup(0x68, calibFineTuneConfig)) {
            DEBUG_PRINTLN_F("FUNCOES_IMU: Falha ao configurar MPU para ajuste!");
            return false;
        }
        loadCalibration(false); // Carrega os biases atuais para esta configuração

        if (print_debug) {
            DEBUG_PRINTF("Iteracao de Ajuste Fino: %d / %d\n", (iter + 1), max_iter);
        }

        float current_accel_bias_LSB[3], current_gyro_bias_LSB[3];
        float error_accel_g[3], error_gyro_dps[3];
        float new_accel_bias_LSB[3], new_gyro_bias_LSB[3];

        float Kp_accel = 0.15;
        float Kp_gyro  = 0.5;

        current_accel_bias_LSB[0] = mpu.getAccBiasX();
        current_accel_bias_LSB[1] = mpu.getAccBiasY();
        current_accel_bias_LSB[2] = mpu.getAccBiasZ();
        current_gyro_bias_LSB[0] = mpu.getGyroBiasX();
        current_gyro_bias_LSB[1] = mpu.getGyroBiasY();
        current_gyro_bias_LSB[2] = mpu.getGyroBiasZ();

        test_bias_for_adjust(samples_per_iteration, error_accel_g, error_gyro_dps);

        if (print_debug) {
            print_vector("  Erros Accel (g)", error_accel_g, 1.0f, 5, "g");
            print_vector("  Erros Gyro (dps)", error_gyro_dps, 1.0f, 5, "dps");
        }

        bool accel_ok = true;
        for (int i = 0; i < 3; i++) if (abs(error_accel_g[i]) > accel_tol_g) accel_ok = false;
        bool gyro_ok = true;
        for (int i = 0; i < 3; i++) if (abs(error_gyro_dps[i]) > gyro_tol_dps) gyro_ok = false;

        if (accel_ok && gyro_ok) {
            if (print_debug) DEBUG_PRINTLN_F("CALIBRACAO FINA CONCLUIDA! Tolerancia atingida.");
            calibrated = true;
            break; 
        }

        if (print_debug) DEBUG_PRINTLN_F("  Ajustando biases...");
        for (int i = 0; i < 3; i++) {
            float error_accel_in_LSB = error_accel_g[i] * ACCEL_CALIB_SENSITIVITY_FS;
            new_accel_bias_LSB[i] = current_accel_bias_LSB[i] + (error_accel_in_LSB * Kp_accel);

            float error_gyro_in_LSB = error_gyro_dps[i] * GYRO_CALIB_SENSITIVITY_FS;
            new_gyro_bias_LSB[i]  = current_gyro_bias_LSB[i]  + (error_gyro_in_LSB * Kp_gyro);
        }

        mpu.setAccBias(new_accel_bias_LSB[0], new_accel_bias_LSB[1], new_accel_bias_LSB[2]);
        mpu.setGyroBias(new_gyro_bias_LSB[0], new_gyro_bias_LSB[1], new_gyro_bias_LSB[2]);
        saveCalibration(false); 
        
        if (print_debug) DEBUG_PRINTLN_F("  --- Fim da Iteracao de Ajuste ---");
    }

    if (!calibrated && print_debug) {
        DEBUG_PRINTLN_F("FUNCOES_IMU: Maximo de iteracoes de ajuste fino atingido.");
    }
    return calibrated;
}

/**
 * @brief Executa a rotina de calibração interna do MPU9250, com possibilidade de ajuste fino, e salva os resultados na EEPROM
 * @warning O sensor DEVE estar perfeitamente imóvel e nivelado durante este processo.
 * @param print Se true, exibe o progresso.
 * @param perform_fine_tuning Se true, executa ajuste fino após a calibração principal.
 */
void calibration_IMU(bool print_debug, bool perform_fine_tuning) {
    DEBUG_PRINTLN_F("FUNCOES_IMU: Iniciando calibracao completa da IMU...");
    mpu.verbose(false);

    DEBUG_PRINTLN_F("Calibrando Accel/Gyro (biblioteca)... Mantenha PARADO e PLANO.");
    if(print_debug) mpu.verbose(true);
    delay(2000);
    mpu.calibrateAccelGyro(); 

    DEBUG_PRINTLN_F("Calibrando Magnetometro (biblioteca)... Mova em '8'.");
    delay(2000);
    mpu.calibrateMag();
    if(print_debug) mpu.verbose(false);

    saveCalibration(print_debug); 
    DEBUG_PRINTLN_F("Calibracao da biblioteca concluida e salva na EEPROM.");

    if (perform_fine_tuning) {
        DEBUG_PRINTLN_F("Calibracao da fina - deixe o sensor parado novamente");
        delay(3000);
        float accel_tol_g = 0.0025;
        float gyro_tol_dps = 0.025;
        int max_iter = 30;
        adjustCalibrationIteratively(50, print_debug, accel_tol_g, gyro_tol_dps, max_iter);
    }
    DEBUG_PRINTLN_F("FUNCOES_IMU: Processo de calibracao da IMU finalizado.");
}

/**
 * @brief Inicializa o hardware da IMU e configura os parâmetros de voo.
 * * @details Configura o I2C, define as escalas (Acc: 16G, Gyro: 2000dps) e a largura
 * de banda dos filtros (DLPF) para reduzir vibração do motor.
 * * @param calibrar_se_necessario Se true, calibra se a EEPROM estiver vazia.
 * @param perform_fine_tuning_on_calib Se true, roda algoritmo iterativo de ajuste fino.
 * @param print_params Se true, imprime configurações.
 * @return true Se o MPU9250 for detetado e iniciado.
 */
bool setup_IMU(bool calibrar_se_necessario, bool perform_fine_tuning_on_calib, bool print_params) {
    DEBUG_PRINTLN_F("SETUP_IMU: Inicializando MPU9250..."); 

    // Define a configuração de VOO final
    mpuConfig.accel_fs_sel = ACCEL_FS_SEL::A16G;
    mpuConfig.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    mpuConfig.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    mpuConfig.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    mpuConfig.gyro_fchoice = 0x03;
    mpuConfig.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    mpuConfig.accel_fchoice = 0x01;
    mpuConfig.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
    // Configura declinação magnética (Sp:-21.46 , Pira:-21.47, Midland: 5.32, Munchen: 4.77) )
    // MUDAR PARA COMPETICAO | MUDAR PARA COMPETICAO | MUDAR PARA COMPETICAO | MUDAR PARA COMPETICAO
    mpu.setMagneticDeclination(5.32); // Ajuste conforme sua localização real de lançamento

    // Aplica a configuração de voo inicial
    if (!mpu.setup(0x68, mpuConfig)) { 
        DEBUG_PRINTLN_F("SETUP_IMU: ERRO - MPU falha na conexao ou setup inicial.");
        return false;
    }
    DEBUG_PRINTLN_F("SETUP_IMU: MPU9250 conectado e configuracoes de VOO aplicadas.");

    // Verifica se os dados de calibração existem ou se uma nova calibração é necessária
    if (hasCalibrationData()) {
        DEBUG_PRINTLN_F("SETUP_IMU: Calibracao existente encontrada na EEPROM.");
        if (perform_fine_tuning_on_calib) {
            DEBUG_PRINTLN_F("SETUP_IMU: Executando ajuste fino sobre a calibracao existente...");
            adjustCalibrationIteratively(50, print_params, 0.0025, 0.025, 30);
        }
    } else {
        DEBUG_PRINTLN_F("SETUP_IMU: Nenhuma calibracao da IMU encontrada.");
        if (calibrar_se_necessario) {
            DEBUG_PRINTLN_F("SETUP_IMU: Executando calibracao completa...");
            calibration_IMU(print_params, perform_fine_tuning_on_calib);
        } else {
            DEBUG_PRINTLN_F("SETUP_IMU: Prosseguindo sem calibracao (usara biases zero).");
        }
    }

    //Garantir que o MPU está na configuração de VOO
    DEBUG_PRINTLN_F("SETUP_IMU: Reaplicando configuracoes de VOO e carregando biases finais...");
    if (!mpu.setup(0x68, mpuConfig)) { 
        DEBUG_PRINTLN_F("SETUP_IMU: ERRO - Falha ao reaplicar config de voo no MPU.");
        return false;
    }
    loadCalibration(print_params); // Carrega os biases mais recentes da EEPROM e aplica ao objeto mpu
    
    DEBUG_PRINTLN_F("SETUP_IMU: Concluido com sucesso e pronto para voo.");
    return true;
}

/**
 * @brief Reinicia a orientação (Quaternions) para o padrão.
 * * @param z_axis_up Se true, considera o eixo Z do sensor apontando para cima.
 * Útil se o filtro divergir muito antes do lançamento.
 */
void reset_orientation(bool z_axis_up){
    mpu.resetOrientation(z_axis_up);
}

/**
 * @brief Ativa ou desativa o aprendizado de drift contínuo.
 * @details Quando ativado, o filtro ajusta lentamente os biases do giroscópio
 * ao longo do tempo para compensar o drift. Isso é feito usando o parametro zeta do filtro Madgwick.
 * * @param enabled Se true, ativa o aprendizado de drift contínuo.
 * Útil para longos periodos antes do lancamento.
 */
void setDriftLearning(bool enabled) {
    mpu.setDriftLearning(enabled);
}

/**
 * @brief Ajusta o parâmetro beta do filtro Madgwick.
 * @details O parâmetro beta controla a velocidade de convergência do filtro.
 * * @param errorDegPerSec Valor do erro esperado em deg/s.
 * Valores maiores fazem o filtro responder mais rápido, mas podem introduzir mais ruído.
 */
void setFilterBeta(float errorDegPerSec) {
    mpu.setFilterBeta(errorDegPerSec);
}

/**
 * @brief Imprime a orientação atual (Euler Angles) para debug.
 */
void print_roll_pitch_yaw() {
    DEBUG_PRINTF("Yaw: %.2f | Pitch: %.2f | Roll: %.2f  ",mpu.getYaw(),mpu.getPitch(),mpu.getRoll() );
}

/**
 * @brief Calcula a inclinação (Tilt) do foguete em relação à vertical.
 * * @details Usa os Quaternions para calcular o ângulo entre o eixo Z do corpo 
 * e o vetor gravidade. Essencial para segurança (não abrir airbrakes se inclinado).
 * * @note 0° = Eixo Z do corpo alinhado com Z do mundo (para cima ou para baixo, depende de z_axis_up). 
 * 90° = Foguete horizontal.
 * @return float Ângulo de inclinação em graus [0° a 180°].
 */
float calcTilt() { 

    float qx = mpu.getQuaternionX();
    float qy = mpu.getQuaternionY();

    // Economiza ler QW e QZ e reduz operações matemáticas
    float cos_theta = 1.0f - 2.0f * (qx * qx + qy * qy);

    // Essencial para evitar que erros de float gerem NaN no acosf
    if (cos_theta > 1.0f) {
        cos_theta = 1.0f;
    } else if (cos_theta < -1.0f) {
        cos_theta = -1.0f;
    }

    float tilt_radianos = acosf(cos_theta);

    //  (180/PI ~= 57.29578) 
    float tilt_graus = tilt_radianos * 57.29578f;

    if (!z_axis_up){
        tilt_graus = (180.0f - tilt_graus);
    }
    // DEBUG_PRINT_F("| Tilt (Z_corpo vs Z_mundo_UP): ");
    // DEBUG_PRINT(tilt_graus);
    // DEBUG_PRINT_F(" graus. | ");
    // DEBUG_PRINT(">tilt:");
    // DEBUG_PRINT(tilt_graus);
    
    return tilt_graus;
}

/**
 * @brief  Calcula a aceleração vertical líquida do foguete no referencial da Terra.
 * * @details Esta função realiza a mudança de referencial do vetor de aceleracao em Z, pegando os dados 
 * crus do acelerômetro (referencial do corpo) e rotacionando-os para o 
 * referencial do mundo (NED) usando os Quaternions atuais.
 * * Matemática: Projeta o vetor de aceleração [ax, ay, az] na direção da gravidade 
 * e subtrai 1G para obter a aceleração de movimento pura.
 * * @note    Assume que mpu.update() já foi chamado no loop principal.
 * * @return  float Aceleração vertical em [m/s²]. 
 * Positivo = Subindo (acelerando para cima).
 * Zero = Parado ou velocidade constante.
 */
float calcularAceleracaoZVerticalLiquida() {

    // Acessamos as variáveis diretas da memória do objeto mpu
    float qx = mpu.getQuaternionX();
    float qy = mpu.getQuaternionY();
    float qz = mpu.getQuaternionZ();
    float qw = mpu.getQuaternionW();

    // Dados crus em 'g' 
    // Nota: O MPU9250 geralmente retorna em 'g' nos métodos getAcc.
    float ax_g = mpu.getAccX(); 
    float ay_g = -mpu.getAccY(); 
    float az_g = -mpu.getAccZ();

   // Rotacionar o vetor de FORÇA ESPECÍFICA (aceleração medida) do corpo 
    //    para o referencial da Terra (NED) e pegar a componente Z (Down).
    //    Componentes da 3ª LINHA da matriz de rotação Corpo-para-Mundo (R_bw) para NED:
    //    R_bw[2][0] (para ax_corpo) = 2.0f * (qx * qz - qw * qy)
    //    R_bw[2][1] (para ay_corpo) = 2.0f * (qy * qz + qw * qx)
    //    R_bw[2][2] (para az_corpo) = qw * qw - qx * qx - qy * qy + qz * qz
    
    // Pré-cálculo de termos comuns
    float qxqz = qx * qz;
    float qwqy = qw * qy;
    float qyqz = qy * qz;
    float qwqx = qw * qx;
    float qx2  = qx * qx;
    float qy2  = qy * qy;

    // Aceleração Vertical no Referencial Mundo (em 'g'), eixo Z para baixo (Down)
    float acc_Z_mundo_g = 
        (2.0f * (qxqz - qwqy)) * ax_g +
        (2.0f * (qyqz + qwqx)) * ay_g +
        (1.0f - 2.0f * (qx2 + qy2)) * az_g;

    // Remocao da gravidade e conversao para m/s²
    float acc_vertical_liquida_ms2 = -(acc_Z_mundo_g + 1.0f) * G_CONSTANTE_GRAVITACIONAL_MS2;
    // DEBUG_PRINT_F("| U_FINAL(m/s^2):"); 
    // DEBUG_PRINTLN(acc_vertical_liquida_ms2, 4);
    
    return acc_vertical_liquida_ms2;
}

/**
 * @brief Função wrapper para executar o teste de ajuste fino.
 */
void teste_fine_tuning(){
    DEBUG_PRINTLN_F("\n--- INICIANDO TESTE DE CALIBRACAO FINA ITERATIVA DA IMU ---");
    float accel_tol_g = 0.005;  // Tolerância desejada em 'g' 
    float gyro_tol_dps = 0.05; // Tolerância desejada em 'dps' 
    int max_iteracoes_ajuste = 25; // Número máximo de tentativas de ajuste
    bool calibracao_fina_concluida = false;

    // Chamada única para a função que faz o loop iterativo internamente:
    calibracao_fina_concluida = adjustCalibrationIteratively(
                                    50,    // samples_per_iteration para test_bias_for_adjust
                                    false,   // print_debug
                                    accel_tol_g,
                                    gyro_tol_dps,
                                    max_iteracoes_ajuste // max_iter para o loop interno de adjustCalibrationIteratively
                                );


    if (calibracao_fina_concluida) {
        sinalizarSucessoModulo("Ajuste Fino IMU");
        DEBUG_PRINTLN_F("--- TESTE DE CALIBRACAO FINA ITERATIVA DA IMU CONCLUIDO COM SUCESSO ---");
        delay(5000);
    } else {
        sinalizarFalhaModulo("Ajuste Fino IMU");
        DEBUG_PRINTLN_F("--- TESTE DE CALIBRACAO FINA ITERATIVA DA IMU: Tolerancia nao atingida apos max_iteracoes ---");
        delay(5000);
    }
    
}

