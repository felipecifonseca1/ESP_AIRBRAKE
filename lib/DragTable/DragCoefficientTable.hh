#ifndef DRAGCOEFFICIENTTABLE_HH
#define DRAGCOEFFICIENTTABLE_HH

#include <cmath>

// -----------------------------------------------------------------------------
// Fixed‐size dimensions
// -----------------------------------------------------------------------------
#define MACH_COUNT  14 // 0.05..0.70 in steps of 0.05
#define ACTUATION_COUNT 6  // 0..100 in steps of 20

float getNearestActuation(float inputVelocity, float inputDragCoeff);

#endif