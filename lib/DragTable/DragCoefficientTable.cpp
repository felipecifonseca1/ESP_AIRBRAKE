#include "DragCoefficientTable.hh"
#include <Arduino.h>

// Possible servo actuations (0 to 1)
constexpr float actuations[ACTUATION_COUNT] = {
     0, 0.20, 0.40, 0.60, 0.80, 1
};

// -----------------------------------------------------------------------------
// dragCoeffTable[i][j] = drag coefficient
//   velocity = velocities[i]
//   actuation = actuations[j]
// -----------------------------------------------------------------------------
constexpr float dragCoeffTable[MACH_COUNT][ACTUATION_COUNT] = {
    // act:     0.0    0.2    0.4    0.6    0.8    1.0
    /* 0.05 */ { 0.421, 0.551, 0.681, 0.812, 0.942, 1.072 },
    /* 0.10 */ { 0.384, 0.514, 0.644, 0.774, 0.904, 1.034 },
    /* 0.15 */ { 0.383, 0.513, 0.643, 0.774, 0.904, 1.034 },
    /* 0.20 */ { 0.383, 0.513, 0.643, 0.773, 0.903, 1.033 },
    /* 0.25 */ { 0.382, 0.512, 0.642, 0.772, 0.902, 1.033 },
    /* 0.30 */ { 0.383, 0.513, 0.643, 0.773, 0.903, 1.033 },
    /* 0.35 */ { 0.384, 0.514, 0.644, 0.774, 0.904, 1.034 },
    /* 0.40 */ { 0.385, 0.515, 0.645, 0.775, 0.905, 1.035 },
    /* 0.45 */ { 0.385, 0.515, 0.646, 0.776, 0.906, 1.036 },
    /* 0.50 */ { 0.386, 0.516, 0.646, 0.777, 0.907, 1.037 },
    /* 0.55 */ { 0.387, 0.517, 0.647, 0.777, 0.907, 1.038 },
    /* 0.60 */ { 0.388, 0.518, 0.648, 0.778, 0.908, 1.038 },
    /* 0.65 */ { 0.391, 0.521, 0.651, 0.781, 0.912, 1.042 },
    /* 0.70 */ { 0.394, 0.525, 0.655, 0.785, 0.915, 1.045 }};

/**
 * @brief Finds the nearest actuation percentage for given velocity and drag coefficient.
 * @details Uses a lookup table to find the closest matching actuation.
 * @param inputVelocity Current velocity input.
 * @param inputDragCoeff Current drag coefficient input.
 * @return Nearest actuation percentage (0 to 1).
 **/
float getNearestActuation(float inputVelocity, float inputDragCoeff)
{
    // 1) Find nearest velocity row
    float nearest_velocity = 0.05;
    if (inputVelocity < 0) {
        return actuations[0];
    }
    if (inputVelocity > 0.8) {
        nearest_velocity = 0.8;
    }
    else {
        nearest_velocity = float(round((100*inputVelocity)/5)*5)/100;
    }
    int bestVelIdx = round(nearest_velocity/0.05) - 1;
    bestVelIdx = std::max(std::min(bestVelIdx, 13), 0);


    // 2) Within that velocity row, find nearest drag coefficient
    if (inputDragCoeff < 0.4) {
        return actuations[0];
    }
    int bestActIdx = 0;
    float bestCdDiff = std::abs(inputDragCoeff - dragCoeffTable[bestVelIdx][0]);
    float diff = bestCdDiff;
    for (int j = 1; j < ACTUATION_COUNT; ++j) {
        diff = std::abs(inputDragCoeff - dragCoeffTable[bestVelIdx][j]);
        if (diff < bestCdDiff) {
            bestCdDiff = diff;
            bestActIdx = j;
        }
    }
    // 3) Return the matching actuation percentage
    return actuations[bestActIdx];
}