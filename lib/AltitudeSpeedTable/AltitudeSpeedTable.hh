#ifndef ALTITUDESPEEDTABLE_HH
#define ALTITUDESPEEDTABLE_HH

#include "cmath"

#define TABLE_SIZE 3254 //Passo de 1m 

extern const float speedTable[];

float lookUpSpeed(float altitude);

#endif