#include "Sinalizacao.h"
#include "Config_voo.h"
#include <Arduino.h>

/**
 * @brief Initializes the signaling system by setting up the buzzer and LED pins.
 */
void setupSinalizacao() {
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_STATUS_1, OUTPUT);
    pinMode(PIN_LED_STATUS_2, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_STATUS_1, LOW);
    digitalWrite(PIN_LED_STATUS_2, LOW);
}

/**
 * @brief Turns on the buzzer at a specific frequency.
 * @param frequency The frequency to set the buzzer to (in Hz).
 */
void buzzerOn(unsigned int frequency) {
    tone(PIN_BUZZER, frequency);
}

/**
 * @brief Turns off the buzzer.
 */
void buzzerOff() {
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
}

/**
 * @brief Turns on the buzzer at a specific frequency.
 * @param duration The duration to keep the buzzer on (in milliseconds).
 * @param frequency The frequency to set the buzzer to (in Hz).
 */
void buzzerBeep(unsigned int duration, unsigned int frequency) {
    tone(PIN_BUZZER, frequency, duration); 
}

/**
 * @brief Sounds multiple beeps with specified durations and pauses.
 * @param numberOfBeeps The number of beeps to sound.
 * @param durationBeeps The duration of each beep (in milliseconds).
 * @param durationPause The duration of the pause between beeps (in milliseconds).
 * @param frequency The frequency to set the buzzer to (in Hz).
 * @warning A funcao esta quebrada
 */
void buzzerBeeps(int numberOfBeeps, unsigned int durationBeeps, unsigned int durationPause, unsigned int frequency) {
   
    tone(PIN_BUZZER, frequency, durationBeeps); 
}

/**
 * @brief Turns on the specified LED.
 * @param pin_led The pin number of the LED to turn on.
 */
void ledOn(int pin_led) {
    digitalWrite(pin_led, HIGH);
}

/**
 * @brief Turns off the specified LED.
 * @param pin_led The pin number of the LED to turn off.
 */
void ledOff(int pin_led) {
    digitalWrite(pin_led, LOW);
}

/**
 * @brief Blinks the specified LED a certain number of times with given on/off durations.
 * @param pin_led The pin number of the LED to blink.
 * @param numberOfBlinks The number of times to blink the LED.
 * @param durationOn The duration the LED stays on during each blink (in milliseconds).
 * @param durationOff The duration the LED stays off between blinks (in milliseconds).
 * @warning A funcao esta quebrada
 */
void ledBlink(int pin_led, int numberOfBlinks, unsigned int durationOn, unsigned int durationOff) {
    digitalWrite(pin_led, HIGH);
 
}

/**
 * @brief Signals the start of the system startup with a long beep and LED indication.
 */
void signalStartupStart() {
    DEBUG_PRINTLN_F("ALERTS: Iniciando Startup...");
    ledOn(PIN_LED_STATUS_1);
    buzzerBeep(0, 800); 
}

/**
 * @brief Signals the successful initialization of a module with a short beep.
 * @param moduleName The name of the module that was successfully initialized.
 */
void signalSuccessfullModule(const char* moduleName) {
    DEBUG_PRINT_F("ALERTS: Module [");DEBUG_PRINT_F(moduleName); DEBUG_PRINTLN_F("] OK.");
    tone(PIN_BUZZER, 2000, 100); 
}

/**
 * @brief Signals the failed initialization of a module with a short beep.
 * @param moduleName The name of the module that was not successfully initialized.
 */
void signalFailedModule(const char* moduleName) {
    DEBUG_PRINT_F("ALERTS: FAILED Module [");DEBUG_PRINT_F(moduleName); DEBUG_PRINTLN_F("]!");
    ledOn(PIN_LED_STATUS_2); 
    tone(PIN_BUZZER, 500, 1000); 
}

/**
 * @brief Signals that the system is ready with multiple short beeps and LED indications.
 */
void signalStartupComplete() {
    DEBUG_PRINTLN_F("ALERTS: System READY.");
    ledOn(PIN_LED_STATUS_1);
    tone(PIN_BUZZER, 3000, 200); 
}