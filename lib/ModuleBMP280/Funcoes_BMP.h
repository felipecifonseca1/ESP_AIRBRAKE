// Funcoes_BMP.h
#ifndef FUNCOES_BMP_H
#define FUNCOES_BMP_H

#include <Arduino.h> 

// --- BMP280 Module Functions ---
bool setupBMP();
float readAltitude();
float altitudeFromPressure(float pressao_pa);
void recalibrateGroundPressure(float currentPressure_pa);
void setGroundPressureP0_BMP(float p0);
void setGroundTemperatureT0_BMP(float t0);

// Getters 
float getGroundPressureP0_BMP();
float getGroundTemperatureT0_BMP();

float getTemperaturaBMPAtual();
float getPressaoBMPAtual();



#endif // FUNCOES_BMP_H