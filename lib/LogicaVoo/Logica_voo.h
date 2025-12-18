#ifndef LOGICA_VOO_H
#define LOGICA_VOO_H

#include <Arduino.h> 

// --- Declarações de Funções da Lógica de Voo ---

// Funções de Detecção de Eventos
bool detectLaunch(float verticalAcceleration, float filteredAltitude);
bool detectBurnout(float verticalAcceleration, unsigned long timeSinceLaunch);
bool detectAirbrakesActuation(float filteredAltitude, float filteredVerticalVelocity);
bool detectApogee(float filteredVerticalVelocity, float filteredAltitude);
bool detectLanding(float filteredVerticalVelocity, float filteredAltitude, unsigned long timeSinceApogee);
// Função para obter o estado de inclinação (tilt)
float readCurrentTilt();

// Funções de Atuação dos Airbrakes
bool setupServo();
void commandAirbrakes(float desiredPosition);
void retractAirbrakes(); // Função específica para retração total

// Função de Checagem de Saúde 
bool checkFlightSystemHealth(float filteredAltitude, float filteredVerticalVelocity); // Verifica se os parâmetros para a lógica de voo estão ok

#endif // LOGICA_VOO_H