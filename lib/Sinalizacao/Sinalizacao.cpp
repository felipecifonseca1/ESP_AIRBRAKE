#include "Sinalizacao.h"
#include "Config_voo.h"
#include <Arduino.h>

/**
 * @brief Initializes the signaling system by setting up the buzzer and LED pins.
 */
void setupSinalizacao() {

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW); // Ensure silent on boot
    
    pinMode(PIN_LED_1, OUTPUT);
    digitalWrite(PIN_LED_1, LOW);
    
    pinMode(PIN_LED_2, OUTPUT);
    digitalWrite(PIN_LED_2, LOW);

    DEBUG_PRINTLN_F("Signaling: Hardware initialized.");
}

/**
 * @brief Turns on the buzzer at a specific frequency.
 * @param frequency The frequency to set the buzzer to (in Hz).
 */
void buzzerOn(uint16_t frequency) {
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
void buzzerBeep(uint16_t duration, uint16_t frequency) {
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
void buzzerBeeps(uint8_t numberOfBeeps, uint16_t durationBeeps, uint16_t durationPause, uint16_t frequency) {
    for (int i = 0; i < numberOfBeeps; i++) {
        tone(PIN_BUZZER, frequency); 
        delay(durationBeeps);
        noTone(PIN_BUZZER);
        if (i < numberOfBeeps - 1) delay(durationPause);
    }
}

/**
 * @brief Turns on the specified LED.
 * @param pin_led The pin number of the LED to turn on.
 */
void ledOn(uint8_t pin_led) {
    digitalWrite(pin_led, HIGH);
}

/**
 * @brief Turns off the specified LED.
 * @param pin_led The pin number of the LED to turn off.
 */
void ledOff(uint8_t pin_led) {
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
void ledBlink(uint8_t pin_led, uint8_t numberOfBlinks, uint16_t durationOn, uint16_t durationOff) {
    for (int i = 0; i < numberOfBlinks; i++) {
        digitalWrite(pin_led, HIGH);
        delay(durationOn);
        digitalWrite(pin_led, LOW);
        if (i < numberOfBlinks - 1) delay(durationOff);
    }
}

/**
 * @brief Signals the start of the system startup with a long beep and LED indication.
 */
void signalStartupStart() {
    DEBUG_PRINTLN_F("ALERTS: Starting Startup Sequence...");
    ledOn(PIN_LED_1);
    // Rising boot chime - Long and deliberate
    tone(PIN_BUZZER, 800);  delay(500);
    tone(PIN_BUZZER, 1000); delay(500);
    tone(PIN_BUZZER, 1200); delay(1000);
    noTone(PIN_BUZZER);
    delay(400); // Wait before proceeding
}

/**
 * @brief Signals the successful initialization of a module with a short beep.
 * @param moduleName The name of the module that was successfully initialized.
 */
void signalSuccessfullModule(const char* moduleName) {
    DEBUG_PRINT_F("ALERTS: Module [");DEBUG_PRINT_F(moduleName); DEBUG_PRINTLN_F("] OK.");
    // Clearer double beep - Long pulses
    tone(PIN_BUZZER, 2000); delay(600);
    noTone(PIN_BUZZER);     delay(300);
    tone(PIN_BUZZER, 2500); delay(600);
    noTone(PIN_BUZZER);
    delay(300); // Give user time to hear it
}

/**
 * @brief Signals the failed initialization of a module with a short beep.
 * @param moduleName The name of the module that was not successfully initialized.
 */
void signalFailedModule(const char* moduleName) {
    DEBUG_PRINT_F("ALERTS: FAILED Module [");DEBUG_PRINT_F(moduleName); DEBUG_PRINTLN_F("]!");
    ledOn(PIN_LED_2); 
    // Powerful falling error tone - Very long
    tone(PIN_BUZZER, 1000); delay(500);
    tone(PIN_BUZZER, 600);  delay(1000);
    noTone(PIN_BUZZER);
}

/**
 * @brief Signals that the system is ready with multiple short beeps and LED indications.
 */
void signalStartupComplete() {
    DEBUG_PRINTLN_F("ALERTS: System READY.");
    // Success multiple beeps - Grand finale
    tone(PIN_BUZZER, 1500); delay(600);
    tone(PIN_BUZZER, 2000); delay(600);
    tone(PIN_BUZZER, 3000); delay(1000);
    noTone(PIN_BUZZER);
    ledOn(PIN_LED_1);
}