#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <SPI.h>

// determine whether EEPROM functions at the end of the sketch
// are compiled. EEPROM is currently not used. uncomment as necessary.
//#define EEPROMfunctions
#define DEBUG_MODE
//#define BYPASS_ALL

#ifdef EEPROMfunctions
#include <EEPROM.h>
#endif

// Pin definitions
#define main1   4
#define main2   5
#define drogue1 2
#define drogue2 3
#define buzzer  6
#define led     13

// TODO: move these to an ejection.h file?
// configurations
#define BUZZER_FREQ             3750    // Hz (buzzer sound frequency)
#define NUM_MEAS_REG            50      // number of historic measurements for linear regression
#define ALT_MEAS_AVGING         100     // number of samples to average
#define NUM_DESCENDING_SAMPLES  30      // number of descending slope values for apogee detection to pass
#define LAUNCH_THRESHOLD        500     // change in altitude to detect launch
#define LANDING_THRESHOLD       20      // change in altitude to detect landing
#define LANDING_SAMPLES         100     // samples to detect landing

#define LOCAL_PRESSURE          101200  // Pa
#define MAIN_DEPLOYMENT         1500    // ft
#define THRESHOLD_ALTITUDE      10000   // ft

#define TIME_LOCK_APOGEE        10000   // ms (10 s)
#define TIME_LOCK_SLEEP         600000  // ms (10 min)

#ifdef DEBUG_MODE                       // multimeter needs more time to get accurate current reading
#define DROGUE_DELAY            1500    // ms
#define MAIN_DELAY              1500    // ms
#else
#define DROGUE_DELAY            500     // ms
#define MAIN_DELAY              500     // ms
#endif

Adafruit_BME280 bme; // I2C

const float cutoff = 0.00001; // cutoff freq of low-pass filter (units unknown lol)

// Programming variables
float altitude = 0;
float alt_ground = 0;
float alt_filtered = 0;
uint32_t prevTick = 0;
uint32_t count = 0; // for landing detection
uint8_t currElem = 0;
uint32_t time_launch = 0;

// arrays to store previous data
float alt_previous[NUM_MEAS_REG];
float time_previous[NUM_MEAS_REG];
float timalt_previous[NUM_MEAS_REG];
float timsqr_previous[NUM_MEAS_REG];

// 'boolean' indicators of events
uint8_t apogee_reached = 0;
uint8_t in_flight = 0;
uint8_t main_deployed = 0;
uint8_t stage = 0;

void setup() {
  // initialize pin modes
  Serial.begin(9600);
  pinMode(main1, OUTPUT);
  pinMode(main2, OUTPUT);
  pinMode(drogue1, OUTPUT);
  pinMode(drogue2, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);

  // set pin states
  digitalWrite(main1, LOW);
  digitalWrite(main2, LOW);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);
  digitalWrite(led, HIGH);
  noTone(buzzer);

  // Reset ejection arrays
  memset(alt_previous, 0, NUM_MEAS_REG*sizeof(*alt_previous));
  memset(time_previous, 0, NUM_MEAS_REG*sizeof(*time_previous));
  memset(timalt_previous, 0, NUM_MEAS_REG*sizeof(*timalt_previous));
  memset(timsqr_previous, 0, NUM_MEAS_REG*sizeof(*timsqr_previous));
  
  // initialize BME280 sensor
  if (!bme.begin()) {  
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    tone(buzzer, BUZZER_FREQ);
    delay(5000);
    noTone(buzzer);
    while (1);
  }
  else {
    for (int i = 0 ; i < 3; i++) {
      tone(buzzer, BUZZER_FREQ);
      delay(100);
      noTone(buzzer);
      delay(100);
    }
  }

  // get ground level pressure/altitude
  for (int i = 0; i < ALT_MEAS_AVGING; i++) {
    alt_ground += getAltitude(); //takes sea-level pressure and reads alt 500 times
  }
  alt_ground = alt_ground/ALT_MEAS_AVGING; //average of alt readings

#ifdef DEBUG_MODE
  Serial.print("----------- GROUND ALTITUDE: ");
  Serial.print(alt_ground);
  Serial.println(" -----------");
#endif

#ifdef BYPASS_ALL
  delay(1000);
  
  Serial.println("-------- LAUNCHED --------\n");
  for (int i = 0 ; i < 1; i++) { // beep once
      tone(buzzer, BUZZER_FREQ);
      delay(250);
      noTone(buzzer);
  }

  delay(2000);
  
  Serial.print("----------- AT APOGEE - DEPLOYING DROGUE AT ");
  Serial.print(altitude);
  Serial.println(" ft -----------\n");

  for (int i = 0 ; i < 2; i++) { // beep twice
      tone(buzzer, BUZZER_FREQ);
      delay(250);
      noTone(buzzer);
      delay(250);
  }
  // drogue deployment
  digitalWrite(drogue1, HIGH);
  digitalWrite(drogue2, HIGH);
  delay(DROGUE_DELAY);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);

  delay(2000);
  
  Serial.print("----------- DEPLOYING MAIN AT ");
  Serial.print(altitude);
  Serial.println(" ft -----------\n");

  for (int i = 0 ; i < 3; i++) { // beep thrice
      tone(buzzer, BUZZER_FREQ);
      delay(150);
      noTone(buzzer);
      delay(150);
  }

  // main deployment
  digitalWrite(main1, HIGH);
  digitalWrite(main2, HIGH);
  delay(MAIN_DELAY);
  digitalWrite(main1, LOW);
  digitalWrite(main2, LOW);

  delay(2000);

  Serial.println("-------- LANDED --------\n");
  for (int i = 0 ; i < 4; i++) { // beep thrice, long
      tone(buzzer, BUZZER_FREQ);
      delay(1500);
      noTone(buzzer);
      delay(1500);
  }

  while (1);
#endif  // BYPASS_ALL
  
  // wait for launch
  while(alt_filtered < LAUNCH_THRESHOLD){
    altitude = getAltitude();
    alt_filtered = runAltitudeMeasurements(millis(), altitude);
#ifdef DEBUG_MODE
    Serial.print("altitude read = ");
    Serial.print(altitude);
    Serial.print(" -- ");
    Serial.print("filtered altitude = ");
    Serial.print(alt_filtered);
    Serial.println();
#endif
    delay(50);
  }
  time_launch = millis(); // set current time as launched time
  
  // moved to apogee detection stage
  stage = 1;
  in_flight = 1;

#ifdef DEBUG_MODE
  Serial.println("-------- LAUNCHED --------\n");
  for (int i = 0 ; i < 1; i++) { // beep once
      tone(buzzer, BUZZER_FREQ);
      delay(250);
      noTone(buzzer);
//      delay(100);
  }
#endif
  
  // apogee detection
  uint8_t numNVals = 0;
  float fittedSlope = 0;
  while (1) {
    altitude = getAltitude();
    alt_filtered = runAltitudeMeasurements(millis(), altitude);
    fittedSlope = LSLinRegression();

    if (millis() - time_launch > TIME_LOCK_APOGEE && fittedSlope < 0) {
      numNVals += 1;
      if (numNVals > NUM_DESCENDING_SAMPLES) {
        break;
      }
    }
    else {
      numNVals = 0;
    }
    
    delay(15);
  }
  stage = 2;
  
#ifdef DEBUG_MODE
  Serial.print("----------- AT APOGEE - DEPLOYING DROGUE AT ");
  Serial.print(altitude);
  Serial.println(" ft -----------\n");

  for (int i = 0 ; i < 2; i++) { // beep twice
      tone(buzzer, BUZZER_FREQ);
      delay(250);
      noTone(buzzer);
      delay(250);
  }
#endif
  
  // drogue deployment at apogee
  digitalWrite(drogue1, HIGH);
  digitalWrite(drogue2, HIGH);
  delay(DROGUE_DELAY);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);
  apogee_reached = 1;
  
  // wait for main deployment altitude
  while (alt_filtered > MAIN_DEPLOYMENT){
    altitude = getAltitude();
    alt_filtered = runAltitudeMeasurements(millis(), altitude);
    delay(50);
  }
  stage = 3;
  
  // main deployment
  digitalWrite(main1, HIGH);
  digitalWrite(main2, HIGH);
  delay(MAIN_DELAY);
  digitalWrite(main1, LOW);
  digitalWrite(main2, LOW);
  main_deployed = 1;
  
#ifdef DEBUG_MODE
  Serial.print("----------- DEPLOYING MAIN AT ");
  Serial.print(altitude);
  Serial.println(" ft -----------\n");

  for (int i = 0 ; i < 4; i++) { // beep thrice
      tone(buzzer, BUZZER_FREQ);
      delay(150);
      noTone(buzzer);
      delay(150);
  }
#endif

  // landing detection
  while (count < LANDING_SAMPLES) {
    altitude = getAltitude();
    alt_filtered = runAltitudeMeasurements(millis(), altitude);
    if (alt_filtered - alt_previous[NUM_MEAS_REG - 1] < LANDING_THRESHOLD && millis() - time_launch > TIME_LOCK_SLEEP)
      count++;
    else
      count = 0;
    
    delay(15);
  }
  stage = 4;
  in_flight = 0;
  
#ifdef DEBUG_MODE
  Serial.println("-------- LANDED --------\n");
  for (int i = 0 ; i < 4; i++) { // beep thrice, long
      tone(buzzer, BUZZER_FREQ);
      delay(1500);
      noTone(buzzer);
      delay(1500);
  }
#endif
}

void loop() {
#ifdef DEBUG_MODE
  // blink built-in LED
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
#endif
  // otherwise do nothing
}

// runs the following:
//    - get current filtered altitude
//    - store filtered altitude and update arrays
//    - return current filtered altitude
float runAltitudeMeasurements(uint32_t currTick, float currAlt) {
  float T = ((currTick - prevTick) / 1000);
  prevTick = currTick;
//  alt_meas = getAltitude() - alt_ground; //Measures AGL altitude in feet
  float alt_filtered = filterAltitude((currAlt - alt_ground), T);
  storeAltitude(alt_filtered, T, currTick);
  return alt_filtered;
}

// Low-pass filter - rocket at high speeds pressure fluctuates and affects 
// altitude reading, usually at a high frequency, so low pass filter filters 
// those high frequency changes out and keeps just the overall, low frequency 
// changes (caused by altitude change)
float filterAltitude(float alt, float deltaT){
  // the following formulas comes from https://en.wikipedia.org/wiki/Low-pass_filter
  // section: Discrete-time realization, simple IIR filter
  float alpha = (float) (deltaT / (cutoff + deltaT));
  return (alt * alpha + (1 - alpha) * alt_previous[NUM_MEAS_REG - 1]);
}

// returns current altitude in feet, unfiltered
float getAltitude() { 
  return bme.readAltitude(LOCAL_PRESSURE/100) * 3.28084;
}

// stores values to arrays
void storeAltitude(float new_altitude, float deltaT, float cTime) {
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

// performs linear regression for apogee detection
float LSLinRegression() {
  float xsum = 0, ysum = 0, xy = 0, xsqr = 0;

  for (uint8_t i = 0; i < NUM_MEAS_REG; i++) {
    xsum += time_previous[i];
    ysum += alt_previous[i];
    xy += timalt_previous[i];
    xsqr += timsqr_previous[i];
  }

  return (float) (NUM_MEAS_REG*xy - (xsum*ysum))/(NUM_MEAS_REG*xsqr - (xsum*xsum));
}


#ifdef EEPROMfunctions
//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value) {
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address) {
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
#endif
