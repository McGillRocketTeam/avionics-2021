#include "main.h"

// Public variables
float alt_previous[NUM_MEAS_AVGING];

// Public function define
float runAltitudeMeasurements(uint32_t currTick, uint16_t currAlt);
float getAverageVelocity();
float filterAltitude(float altitude, float time);
