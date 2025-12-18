#include "Sinalizacao.h"
#include "Config_voo.h"
#include <Arduino.h>

/**
 * @brief Initializes the signaling system by setting up the buzzer and LED pins.
 */
void setupSinalizacao() {
    pinMode(PINO_BUZZER, OUTPUT);
    pinMode(PINO_LED_STATUS_1, OUTPUT);
    pinMode(PINO_LED_STATUS_2, OUTPUT);
    digitalWrite(PINO_BUZZER, LOW);
    digitalWrite(PINO_LED_STATUS_1, LOW);
    digitalWrite(PINO_LED_STATUS_2, LOW);
}

/**
 * @brief Turns on the buzzer at a specific frequency.
 * @param frequency The frequency to set the buzzer to (in Hz).
 */
void buzzerOn(unsigned int frequency) {
    tone(PINO_BUZZER, frequency);
}

/**
 * @brief Turns off the buzzer.
 */
void buzzerOff() {
    noTone(PINO_BUZZER);
    digitalWrite(PINO_BUZZER, LOW);
}

/**
 * @brief Turns on the buzzer at a specific frequency.
 * @param duracao_ms The duration to keep the buzzer on (in milliseconds).
 * @param frequency The frequency to set the buzzer to (in Hz).
 */
void buzzerBeep(unsigned int duracao_ms, unsigned int frequency) {
    tone(PINO_BUZZER, frequency, 50); // Toca por 50ms 
}

/**
 * @brief Sounds multiple beeps with specified durations and pauses.
 * @param numero_beeps The number of beeps to sound.
 * @param duracao_beep_ms The duration of each beep (in milliseconds).
 * @param duracao_pausa_ms The duration of the pause between beeps (in milliseconds).
 * @param frequency The frequency to set the buzzer to (in Hz).
 * @warning A funcao esta quebrada
 */
void buzzerBeeps(int numero_beeps, unsigned int duracao_beep_ms, unsigned int duracao_pausa_ms, unsigned int frequency) {
   
    tone(PINO_BUZZER, frequency, 100); 
}

/**
 * @brief Turns on the specified LED.
 * @param pino_led The pin number of the LED to turn on.
 */
void ledOn(int pino_led) {
    digitalWrite(pino_led, HIGH);
}

/**
 * @brief Turns off the specified LED.
 * @param pino_led The pin number of the LED to turn off.
 */
void ledOff(int pino_led) {
    digitalWrite(pino_led, LOW);
}

/**
 * @brief Blinks the specified LED a certain number of times with given on/off durations.
 * @param pino_led The pin number of the LED to blink.
 * @param numero_piscadas The number of times to blink the LED.
 * @param duracao_aceso_ms The duration the LED stays on during each blink (in milliseconds).
 * @param duracao_apagado_ms The duration the LED stays off between blinks (in milliseconds).
 * @warning A funcao esta quebrada
 */
void ledBlink(int pino_led, int numero_piscadas, unsigned int duracao_aceso_ms, unsigned int duracao_apagado_ms) {
    digitalWrite(pino_led, HIGH);
 
}

/**
 * @brief Signals the start of the system startup with a long beep and LED indication.
 */
void sinalizarInicioStartup() {
    DEBUG_PRINTLN_F("SINALIZACAO: Iniciando Startup...");
    ledOn(PINO_LED_STATUS_1);
    buzzerBeep(0, 800); 
}

/**
 * @brief Signals the successful initialization of a module with a short beep.
 * @param nome_modulo The name of the module that was successfully initialized.
 */
void sinalizarSucessoModulo(const char* nome_modulo) {
    DEBUG_PRINT_F("SINALIZACAO: Modulo [");DEBUG_PRINT_F(nome_modulo); DEBUG_PRINTLN_F("] OK.");
    tone(PINO_BUZZER, 2000, 100); 
}

/**
 * @brief Signals the failed initialization of a module with a short beep.
 * @param nome_modulo The name of the module that was not successfully initialized.
 */
void sinalizarFalhaModulo(const char* nome_modulo) {
    DEBUG_PRINT_F("SINALIZACAO: FALHA no Modulo [");DEBUG_PRINT_F(nome_modulo); DEBUG_PRINTLN_F("]!");
    ledOn(PINO_LED_STATUS_2); 
    tone(PINO_BUZZER, 500, 1000); 
}

/**
 * @brief Signals that the system is ready with multiple short beeps and LED indications.
 */
void sinalizarSistemaPronto() {
    DEBUG_PRINTLN_F("SINALIZACAO: Sistema PRONTO.");
    ledOn(PINO_LED_STATUS_1);
    tone(PINO_BUZZER, 3000, 200); 
}