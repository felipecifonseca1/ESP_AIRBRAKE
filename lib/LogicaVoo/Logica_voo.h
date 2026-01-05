#ifndef LOGICA_VOO_H
#define LOGICA_VOO_H

#include <Arduino.h> 

// --- Functions declarations ---

bool detectLaunch(float verticalAcceleration, float filteredAltitude);
bool detectBurnout(float verticalAcceleration, unsigned long timeSinceLaunch);
bool detectAirbrakesActuation(float filteredAltitude, float filteredVerticalVelocity);
bool detectApogee(float filteredVerticalVelocity, float filteredAltitude);
bool detectApogeeByRegression(float filteredAltitude, unsigned long currentTime_ms);
bool detectLanding(float filteredVerticalVelocity, float filteredAltitude, unsigned long timeSinceApogee);
bool checkFlightSystemHealth(float filteredAltitude, float filteredVerticalVelocity); // Verifica se os parâmetros para a lógica de voo estão ok

float readCurrentTilt();
bool setupServo();
void commandAirbrakes(float desiredPosition);
void retractAirbrakes(); 


#endif // LOGICA_VOO_H