#ifndef SINALIZACAO_H
#define SINALIZACAO_H

#include <Arduino.h> 
#include "Config_voo.h"

// Initial pin setup for signaling system
void setupSinalizacao();

// Basic buzzer functions
void buzzerOn(uint16_t frequency = 1000); 
void buzzerOff();
void buzzerBeep(uint16_t duration, uint16_t frequency = 1000);
void buzzerBeeps(uint8_t numberOfBeeps, uint16_t durationBeeps, uint16_t durationPause, uint16_t frequency = 1000);

// Basic LED functions
void ledOn(uint8_t pin_led);
void ledOff(uint8_t pin_led);
void ledBlink(uint8_t pin_led, uint8_t numberOfBlinks, uint16_t durationOn, uint16_t durationOff);

// Startup Signals
void signalStartupStart(); 
void signalSuccessfullModule(const char* moduleName); 
void signalFailedModule(const char* moduleName  );   
void signalStartupComplete(); 

#endif // SINALIZACAO_H