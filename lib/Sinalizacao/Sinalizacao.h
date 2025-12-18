#ifndef SINALIZACAO_H
#define SINALIZACAO_H

#include <Arduino.h> 
// --- Constantes dos Pinos ---
const u_int8_t PINO_BUZZER = 14;
const u_int8_t PINO_LED_STATUS_1 = 12; // Ex: LED 1
const u_int8_t PINO_LED_STATUS_2 = 4; // Ex: LED 2

// Configuração inicial dos pinos
void setupSinalizacao();

// Funções básicas do Buzzer
void buzzerOn(unsigned int frequency = 1000); // Liga o buzzer com uma frequência (Hz)
void buzzerOff();
void buzzerBeep(unsigned int duracao_ms, unsigned int frequency = 1000);
void buzzerBeeps(int numero_beeps, unsigned int duracao_beep_ms, unsigned int duracao_pausa_ms, unsigned int frequency = 1000);

// Funções básicas dos LEDs
void ledOn(int pino_led);
void ledOff(int pino_led);
void ledBlink(int pino_led, int numero_piscadas, unsigned int duracao_aceso_ms, unsigned int duracao_apagado_ms);

// Sinais de Startup
void sinalizarInicioStartup(); // Ex: um beep longo e um LED aceso
void sinalizarSucessoModulo(const char* nome_modulo); // Ex: beep curto, LED 1 pisca
void sinalizarFalhaModulo(const char* nome_modulo);   // Ex: beeps de erro, LED 2 pisca
void sinalizarSistemaPronto(); // Ex: múltiplos beeps curtos, ambos LEDs piscam e depois 1 fica aceso

#endif // SINALIZACAO_H