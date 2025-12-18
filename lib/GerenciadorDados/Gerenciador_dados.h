// Gerenciador_dados.h
#ifndef GERENCIADOR_DADOS_H
#define GERENCIADOR_DADOS_H

#include <Arduino.h>

// Estrutura para dados escalonados (para Flash SPI)
struct LogDataVooEscalonada {
    unsigned long timestamp_ms;
    int16_t accX_scaled, accY_scaled, accZ_scaled;
    int16_t gyroX_scaled, gyroY_scaled, gyroZ_scaled;
    int16_t magX_scaled, magY_scaled, magZ_scaled;
    int16_t qW_scaled, qX_scaled, qY_scaled, qZ_scaled;
    int16_t altitudeFiltrada_scaled;
    int16_t velocidadeVerticalFiltrada_scaled;
    int16_t aceleracaoVerticalLiquida_scaled;
    int16_t tilt_scaled; 
    float pressaoBMP_scaled;
    int16_t servoAtuacao_scaled;
    int16_t gain1_scaled;
    int16_t gain2_scaled;
    uint8_t estadoFoguete;
}; // Tamanho total: 47 bytes 

// Estrutura para passar os dados float brutos para as funções de log.
// Unidades time [ms], acc [m/s²], gyro [°/s], mag [uT], quaternion, altitude [m], vel vertical [m/s], acel vertical líquida [m/s²], tilt [°], pressão [Pa], servo [0-1], gains, estado
struct DadosVooBrutosParaLog {
    unsigned long timestamp;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float qW, qX, qY, qZ;
    float altitudeFiltrada;
    float velocidadeVerticalFiltrada;
    float aceleracaoVerticalLiquida;
    float tilt; 
    float pressaoBMP;
    float servoAtuacao_percent;
    float gain1;
    float gain2;
    int estadoFoguete;
};

struct DadosSimulacaoHIL {
    bool dadosValidos = false; 
    float tempo_s = 0.0f;
    float pressao_Pa = 0.0f;
    float aceleracaoLiquida_ms2 = 0.0f;
    float tilt = 0.0f;
};

// Inicia ou reinicia a leitura do arquivo de simulação HIL
bool iniciarSimulacaoHIL(const char* nomeArquivo);
// Lê a próxima linha do arquivo de simulação e retorna os dados
DadosSimulacaoHIL lerProximoPassoSimulacaoHIL();
// Para o HIL, se ele estiver ativo
void pararSimulacaoHIL();
void verificarConteudoArquivoHIL();
void resetarSimulacaoHIL(); 

// --- Funções Públicas do Gerenciador de Log ---

// Funções de Setup separadas
bool setupLogFlashSPI(bool apagarAreaLog = false);
bool setupLogSDCard(); // Cria novo arquivo CSV com cabeçalho
bool garantirConexaoSD(); // Tenta garantir que o SD está montado
void setFatorDecimacaoLog(u_int16_t fator);  // Define o fator de decimação do log (1 = tudo, 10 = 1 a cada 10, etc)

// Funções de Logging separadas
void gravarLogFlashSPI(const DadosVooBrutosParaLog& dados);
void gravarLogSDCard(DadosVooBrutosParaLog dados); 

// --- Dados via Serial ---
void listarArquivosSD();           // Mostra lista de arquivos no Monitor Serial
void despejarLogAtualNaSerial();   // Lê o arquivo atual e imprime no Monitor Serial
void receberArquivoHILViaSerial(); 
void limparTodosLogs(); // Apaga todos os logs do SD

// Controle global do estado de logging 
void iniciarLoggingGeral();
void pararLoggingGeral(); // Importante para fechar arquivo SD corretamente
bool isLoggingDeDadosAtivo();
void finalizarSDCard();

// Para debug ou informação
uint32_t getEnderecoAtualFlashSPI();
String getNomeArquivoSDLog();

#endif // GERENCIADOR_DADOS_H