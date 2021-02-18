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
long main_delay = 3500; // milliseconds. time that main is HIGH

// Configuration variables
float freq = 3750; // buzzer sound frequency.
float a = 1; // Cut-off frequency of low-pass filter in Hz.
const int num_meas = 10; // Number of historic measurements for averaging.


// Programming variables
float alt_meas;
float ground_alt = 0;
float T; // sampling period, time of each loop
float alt_previous[num_meas];
float alt_filtered;
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
//  pinMode(in11, INPUT);
//  pinMode(in12, INPUT);
//  pinMode(in21, INPUT);
//  pinMode(in22, INPUT);

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
    ground_alt += bme.readAltitude(local_pressure/100); //takes sea-level pressure and reads alt 500 times
  }
  ground_alt = ground_alt/500; //average of alt readings
  a = 2*3.14159*a; 
}

void loop() {

    
    if (main_deployed == false){
      T = (millis() - t_previous_loop)/1000; //millis() = time since program start running T running time of curr loop (s)
      t_previous_loop = millis(); //total time 
      
      alt_meas = (bme.readAltitude(local_pressure/100) - ground_alt)*3.28084; //Measures AGL altitude in feet
  
      // Low-pass filter - rocket at high speeds pressure fluctuates and affects altitude reading, usaully at a high frequency, so low pass filter filters those high freuqency changes out 
      //and keeps just the overall, low frequency changes (caused by altitude change)
      alt_filtered = (1 - T * a) * alt_previous[num_meas-1] + a * T * alt_meas;
      
      // Slide window of 10 measurement history.
      for (int i = 0; i < num_meas-1; i++){
        alt_previous[i] = alt_previous[i+1];
      }
      alt_previous[num_meas-1] = alt_filtered;
      
      // Launch Detection
      if (alt_filtered > 150 && launched == false){
        launched = true;

      }
  
      //Average gradient of 10 past measurements.
      average_gradient = 0;
      for (int i = 0; i < num_meas-1; i++){
        average_gradient += (alt_previous[i+1]- alt_previous[i]);
      }
      if (T>0){
      average_gradient /= (num_meas);
      }
      
      // Apogee detection
      if (alt_filtered > threshold_altitude && launched && apogee_reached == false){
        
        if (average_gradient < -2){ //what is the purpose of this -2?
          apogee_reached = true;
          digitalWrite(drogue1, HIGH);
          digitalWrite(drogue2, HIGH);
          delay(drogue_delay);
//          R11 = digitalRead(in11);
//          R12 = digitalRead(in12);
//            if(R11){
//              time11 = millis();
//            }if(R12){
//              time12 = millis();
//            }
          
          digitalWrite(drogue1, LOW);
          digitalWrite(drogue2, LOW);
        }
      }
      Serial.print(t_previous_loop);
      tone(buzzer, freq);
      delay(100);
      noTone(buzzer);
      delay(1000);
      // Main Deployment detection
      //if (apogee_reached && alt_filtered < main_deployment && main_deployed == false){
      if(t_previous_loop > 10000 && main_deployed == false){
          main_deployed = true;
          digitalWrite(main1, HIGH);
          digitalWrite(main2, HIGH);
          tone(buzzer, freq);
          delay(main_delay);
//          R21 = digitalRead(in21);
//          R22 = digitalRead(in22);
//            if(R21){
//              time21 = millis();
//            }if(R22){
//              time22 = millis();
//            }
          noTone(buzzer);
          digitalWrite(main1, LOW);
          digitalWrite(main2, LOW);
      }
  
  
     //  Simultaneous buzzer (does not interrupt loop whilst buzzing)
//      if ((millis() - t_previous_buzz)> 1000){
//        tone(buzzer, freq);
//      }
//      if((millis() - t_previous_buzz) > 1100){
//        noTone(buzzer);
//        t_previous_buzz = millis();
//      }
//  
//      } 
//    else{ //Longer buzz to indicate program completion
//      
//      tone(buzzer, freq);
//      delay(1000);
//      noTone(buzzer);
//      delay(1000);
//      EEPROM.write(0, R11);
//      EEPROMWritelong(1, time11);
//      EEPROM.write(5, R12);
//      EEPROMWritelong(6, time12);
//      EEPROM.write(10, R21);
//      EEPROMWritelong(11, time21);
//      EEPROM.write(15, R22);
//      EEPROMWritelong(16, time22);

    }


    //Just for debuggging.
//    Serial.print(alt_meas);
//    Serial.print(",");
//    Serial.print(average_gradient);
//    Serial.print(",");
//    Serial.println(alt_filtered);
//    
    delay(5);
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
