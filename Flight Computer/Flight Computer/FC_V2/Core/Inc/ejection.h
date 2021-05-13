#include "main.h"

// Public variables
float alt_previous[NUM_MEAS_REG];
float time_previous[NUM_MEAS_REG];
float timalt_previous[NUM_MEAS_REG];
float timsqr_previous[NUM_MEAS_REG];

// Public function define
float runAltitudeMeasurements(uint32_t currTick, uint16_t currAlt);
//float getAverageVelocity();
float filterAltitude(float altitude, float deltaT);
float LSLinRegression();
