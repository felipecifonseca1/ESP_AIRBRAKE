#ifndef SINALIZACAO_H
#define SINALIZACAO_H

#include <Arduino.h> 
// --- Constantes dos Pinos ---
const u_int8_t PIN_BUZZER = 14;
const u_int8_t PIN_LED_STATUS_1 = 12; // Ex: LED 1
const u_int8_t PIN_LED_STATUS_2 = 4; // Ex: LED 2

// Initial pin setup for signaling system
void setupSinalizacao();

// Basic buzzer functions
void buzzerOn(unsigned int frequency = 1000); 
void buzzerOff();
void buzzerBeep(unsigned int duration, unsigned int frequency = 1000);
void buzzerBeeps(int numberOfBeeps, unsigned int durationBeeps, unsigned int durationPause, unsigned int frequency = 1000);

// Basic LED functions
void ledOn(int pin_led);
void ledOff(int pin_led);
void ledBlink(int pin_led, int numberOfBlinks, unsigned int durationOn, unsigned int durationOff);

// Startup Signals
void signalStartupStart(); 
void signalSuccessfullModule(const char* moduleName); 
void signalFailedModule(const char* moduleName  );   
void signalStartupComplete(); 

#endif // SINALIZACAO_H