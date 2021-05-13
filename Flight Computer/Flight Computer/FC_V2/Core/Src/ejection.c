/*
 * ejection.c
 *
 *  Created on: Mar 18, 2021
 *      Author: a cat
 */

#include "ejection.h"

uint32_t prevTick = 0;
float T;						// sampling period, time of each loop
float alpha;

// Private functions
void storeAltitude(float new_altitude, float deltaT, float cTime);

// cutoff frequency of lowpass filter
float cutoff = 0.00001;

// Public function implementation
float runAltitudeMeasurements(uint32_t currTick, uint16_t currAlt){
  T = (float)(currTick - prevTick);

  prevTick = currTick;
//  alt_meas = currAlt - alt_ground; // Measures AGL altitude in feet
  float alt_filtered = filterAltitude((currAlt - alt_ground), T);
  storeAltitude(alt_filtered, T, currTick);
  return alt_filtered;
}

// Private function implementation

// Low-pass filter - rocket at high speeds pressure fluctuates and affects altitude reading, usually at a high frequency, so low pass filter filters those high freuqency changes out
// and keeps just the overall, low frequency changes (caused by altitude change)
float filterAltitude(float altitude, float deltaT){
  float alpha = deltaT / (cutoff + deltaT);
  return (altitude * alpha + (1 - alpha) * alt_previous[NUM_MEAS_REG - 1]);
}

void storeAltitude(float new_altitude, float deltaT, float cTime){

  alt_previous[currElem] = new_altitude;
  time_previous[currElem] = cTime;
  timalt_previous[currElem] = cTime * new_altitude;
  timsqr_previous[currElem] = cTime * cTime;

  if (currElem == (NUM_MEAS_REG - 1)){
	  currElem = 0;
  }
  else{
	  currElem += 1;
  }
}

//float getAverageVelocity(){
//  float average_vel = 0;
//  for (int i = 0; i < NUM_MEAS_REG; i++)
//  {
//    average_vel += vel_previous[i];
//  }
//  return average_vel/NUM_MEAS_REG;
//}
//
//float getVelocityGradient() {
//	float gradient = 0;
//	for (int i = 0; i < NUM_MEAS_REG - 1; i++) {
////		gradient += (vel_previous[i+1] - vel_previous[i]);
//		gradient += vel_previous[i];
//	}
//	gradient = gradient / NUM_MEAS_REG;
//
//	return gradient;
//}

float LSLinRegression(){
	float xsum = 0, ysum = 0, xy = 0, xsqr = 0;

	for (uint8_t i = 0; i < NUM_MEAS_REG; i++){
		xsum += time_previous[i];
	    ysum += alt_previous[i];
	    xy += timalt_previous[i];
	    xsqr += timsqr_previous[i];
	}

	return (float)(NUM_MEAS_REG*xy - (xsum*ysum))/(NUM_MEAS_REG*xsqr - (xsum*xsum));
}
