#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>

Adafruit_BME280 bme; // I2C?

// Pin definitions
#define main1 4
#define main2 5
#define drogue1 2
#define drogue2 3
#define buzzer 6
#define led 13
// Input pin definitions
//#define in11 
//#define in12 
//#define in21 
//#define in22

// Control variables
float local_pressure = 101200; //hPa (sea level)
float threshold_altitude = 10000; //feet
float main_deployment = 1500; //feet
long drogue_delay = 500; // milliseconds. time that drogue is HIGH
long main_delay = 500; // milliseconds. time that main is HIGH
float landing_threshold = 1;//change in altitude to detect landing
float drogue_deployment_velocity = 10;

// Configuration variables
float freq = 3750; // buzzer sound frequency.
float a = 1; // Cut-off frequency of low-pass filter in Hz.
const int num_meas = 100; // Number of historic measurements for averaging.


// Programming variables
float alt_meas;
float ground_alt = 0;
float T; // sampling period, time of each loop
float alt_previous[num_meas];
float vel_previous[num_meas];
//float alt_filtered;
float t_previous_loop, t_previous_buzz;
float average_gradient;
boolean apogee_reached = false;
boolean launched = false;
boolean main_deployed = false;
// Relay output checks
boolean R11; // drogue1
boolean R12; // drogue2
boolean R21; // main1
boolean R22; // main2
long time11;
long time12;
long time21;
long time22;

void setup() {
  
  Serial.begin(9600);
  pinMode(main1, OUTPUT);
  pinMode(main2, OUTPUT);
  pinMode(drogue1, OUTPUT);
  pinMode(drogue2, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);

  digitalWrite(main1, LOW);
  digitalWrite(main2, LOW);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);
  digitalWrite(buzzer, LOW);
  digitalWrite(led, HIGH);
  
  
  if (!bme.begin()) {  
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    tone(buzzer, freq);
    delay(5000);
    noTone(buzzer);
    while (1);
  }
  else{
    
    for (int i = 0 ; i < 3; i++){
    tone(buzzer, freq);
    delay(100);
    noTone(buzzer);
    delay(100);
    }
  }
  for (int i = 0; i < 500; i++){
    ground_alt += getAltitude(); //takes sea-level pressure and reads alt 500 times
  }
  ground_alt = ground_alt/500; //average of alt readings
  a = 2*3.14159*a; 

  float alt_filtered =0;
  while(alt_filtered < 150){//waiting to launch
    alt_filtered=runMeasurements();
    printValues(alt_filtered);
    delay(50);
  }
  //Has now launched, wait until apogee
  while (getAverageVelocity() > -drogue_deployment_velocity || alt_filtered < threshold_altitude) //while moving up and hasn't reached threshold altitude yet
  {
    alt_filtered=runMeasurements();
    printValues(alt_filtered);
    delay(5);
  }
  //Store altitude ejected at = alt_filtered
//  EEPROMWritelong(int address, long value)
  
  //At Apogee so deploy drogue
  digitalWrite(drogue1, HIGH);
  digitalWrite(drogue2, HIGH);
  delay(drogue_delay);
  digitalWrite(drogue1, LOW);
  digitalWrite(drogue2, LOW);
  //Wait until get below main deployment altitude
  while (alt_filtered > main_deployment){
    alt_filtered=runMeasurements();
    printValues(alt_filtered);
    delay(5);
  }
  //Deploy Main
  digitalWrite(main1, HIGH);
  digitalWrite(main2, HIGH);
  delay(main_delay);
  digitalWrite(main1, LOW);
  digitalWrite(main2, LOW);

  //wait until at rest
//  while (alt_previous[0] - alt_previous[num_meas - 1] < landing_threshold)
//  {
//    runMeasurements();
//    printValues();
//    delay(5);
//  }
  //put bme280 to sleep
}

void loop() {

}

void printValues(float alt_filtered){
  Serial.print(alt_meas);
  Serial.print(",");
  Serial.print(average_gradient);
  Serial.print(",");
  Serial.println(alt_filtered);
}

float runMeasurements(){
  T = (millis() - t_previous_loop) / 1000;
  t_previous_loop = millis();
  alt_meas = getAltitude() - ground_alt; //Measures AGL altitude in feet
  float alt_filtered = filterAltitude(alt_meas, T);
  storeAltitude(alt_filtered, T);
  return alt_filtered;
}



// Low-pass filter - rocket at high speeds pressure fluctuates and affects altitude reading, usaully at a high frequency, so low pass filter filters those high freuqency changes out
//and keeps just the overall, low frequency changes (caused by altitude change)
float filterAltitude(float altitude, float time){
  return (1 - time * a) * alt_previous[num_meas - 1] + a * time * altitude;
}

float getAltitude(){//returns altitude in feet
  return bme.readAltitude(local_pressure / 100) * 3.28084;
}
void storeAltitude(float new_altitude, float time){
  for (int i = 0; i < num_meas - 1; i++)
  {
    vel_previous[i] = vel_previous[i + 1]; 
    alt_previous[i] = alt_previous[i + 1];
  }
  alt_previous[num_meas - 1] = new_altitude;
  vel_previous[num_meas - 1] = (alt_previous[num_meas - 2]-alt_previous[num_meas - 1])/time;
}

float getAverageVelocity(){
  float average_vel = 0;
  for (int i = 0; i < num_meas; i++)
  {
    average_vel += vel_previous[i];
  }
  return average_vel/num_meas;
}

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value)
      {
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

long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }
