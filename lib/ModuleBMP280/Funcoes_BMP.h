// Funcoes_BMP.h
#ifndef FUNCOES_BMP_H
#define FUNCOES_BMP_H

#include <Arduino.h> // Para tipos e funções padrão do Arduino

// Declarações das funções que estarão em Funcoes_BMP.cpp
bool setupBMP();
float lerAltitudeDoBMP280();
float altitudeFromPressure(float pressao_pa);
void recalibrarPressaoDeSoloBMP();
void setGroundPressureP0_BMP(float p0);
void setGroundTemperatureT0_BMP(float t0);

// Getters 
float getGroundPressureP0_BMP();
float getGroundTemperatureT0_BMP();

float getTemperaturaBMPAtual();
float getPressaoBMPAtual();



#endif // FUNCOES_BMP_H