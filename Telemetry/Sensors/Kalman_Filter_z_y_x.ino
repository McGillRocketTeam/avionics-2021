#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
using namespace BLA;

#define SEALEVELPRESSURE_HPA (1014) // for Montreal! Changes depending on our location 

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BME280 bme; 
SFE_UBLOX_GNSS myGPS;

void setup() {
  Serial.begin(9600);
  unsigned status = bme.begin();  
    if (!status) {
        Serial.println("BME280 not working");
        while (1) delay(10);
    }

    if(!bno.begin())
  {
    Serial.print("BNO055 not working");
    while(1);
  }
  bno.setExtCrystalUse(true);

  Wire.begin();
  if (myGPS.begin() == false) // Connect to the u-blox module using Wire port
  {
    Serial.println(F("GPS not working"));
    while (1);
  }
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void loop() {
  // Initial Conditions
  float reference_altitude = 0;
  for (int i=1; i<=1000; i++){
     reference_altitude = reference_altitude + bme.readAltitude(SEALEVELPRESSURE_HPA);
  }
  reference_altitude = reference_altitude/1000; // Initial altitude taken from the BME280
                                                // Considered the reference or base value
                                                // 1000 samples and do the average

  float reference_u_t_z = 0;
  for (int i=1; i<=1000; i++){
    imu::Vector<3> accl = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    reference_u_t_z = reference_u_t_z + accl.z();
  }
  reference_u_t_z = reference_u_t_z/1000;

  float reference_u_t_y = 0;
  for (int i=1; i<=1000; i++){
    imu::Vector<3> accl = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    reference_u_t_y = reference_u_t_y + accl.y();
  }
  reference_u_t_y = reference_u_t_y/1000;

  float reference_u_t_x = 0;
  for (int i=1; i<=1000; i++){
    imu::Vector<3> accl = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    reference_u_t_x = reference_u_t_x + accl.x();
  }
  reference_u_t_x = reference_u_t_x/1000;

  float reference_latitude = 0;
  for (int i=1; i<=1000; i++) {
    reference_latitude = reference_latitude + ((myGPS.getLatitude()/10000000)*111000);
  }
  reference_latitude = reference_latitude/1000;

  float reference_longitude = 0;
  for (int i=1; i<=1000; i++) {
    reference_longitude = reference_longitude + ((myGPS.getLongitude()/10000000)*cos(myGPS.getLatitude()/10000000)*111320);
  }
  reference_longitude = reference_longitude/1000;
  
  BLA::Matrix<6,1> X_t_old = {0,
                              0,
                              0,
                              0,
                              0,
                              0,}; // "Initial" position and velocity in z 
  BLA::Matrix<6,6> E_t_old = {0.147880991, 0, 0, 0, 0, 0,
                              0, 7.03, 0, 0, 0, 0,
                              0, 0, 12.59, 0, 0, 0,
                              0, 0, 0, 1.21e-8, 0, 0,
                              0, 0, 0, 0, 1.21e-8, 0, 
                              0, 0, 0, 0, 0, 1.21e-8}; // Initial error (co)variance 
  BLA::Matrix<6,6> I = {1, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0,
                        0, 0, 0, 1, 0, 0,
                        0, 0, 0, 0, 1, 0,
                        0, 0, 0, 0, 0, 1}; // Identity matrix 
  float DeltaT = 0.007; // Sampling Period! Can only be found once code is complete

  // Matrices Declared
  BLA::Matrix<6,6> A;
  BLA::Matrix<6,6> B;
  BLA::Matrix<6,1> u_t;
  BLA::Matrix<6,1> X_t_pred;
  BLA::Matrix<6,6> Q;
  BLA::Matrix<6,6> E_t_pred;
  BLA::Matrix<6,1> Y_t;
  BLA::Matrix<6,6> R;
  BLA::Matrix<6,6> K;
  BLA::Matrix<6,1> X_t;
  BLA::Matrix<6,6> E_t;
  
  // Body Part 
  while (true) {
    //Serial.println(millis());
    imu::Vector<3> accl = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  A = {1, 0, 0, DeltaT, 0, 0,
       0, 1, 0, 0, DeltaT, 0,
       0, 0, 1, 0, 0, DeltaT,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1}; // State transition matrix using the sampling period 
  B = {0.5*DeltaT*DeltaT, 0, 0, 0, 0, 0,
       0, 0.5*DeltaT*DeltaT, 0, 0, 0, 0,
       0, 0, 0.5*DeltaT*DeltaT, 0, 0, 0,
       0, 0, 0, DeltaT, 0, 0,
       0, 0, 0, 0, DeltaT, 0,
       0, 0, 0, 0, 0, DeltaT}; // Control matrix using the sampling period 
  u_t(0,0) = accl.z()-reference_u_t_z; // Acceleration of the rocket in the z direction, given by the accelerometer 
  u_t(1,0) = accl.y()-reference_u_t_y;
  u_t(2,0) = accl.x()-reference_u_t_x;
  u_t(3,0) = u_t(0,0);
  u_t(4,0) = u_t(1,0);
  u_t(5,0) = u_t(2,0);
  X_t_pred = A*X_t_old+B*u_t; // Predicted State (Accelerometer State)

  Q =  {3.04e-2, 0, 0, 0, 0, 0,
       0, 3.53e-2, 0, 0, 0, 0,
       0, 0, 666666, 0, 0, 0,
       0, 0, 0, 1.21e-8, 0, 0,
       0, 0, 0, 0, 1.21e-8, 0,
       0, 0, 0, 0, 0, 1.21e-8}; // Process noise (co)variance 
  E_t_pred = A*E_t_old*~A+Q; // Predicted error (co)variance 

  Y_t = {bme.readAltitude(SEALEVELPRESSURE_HPA)-reference_altitude,
         ((myGPS.getLatitude()/10000000)*111000)-reference_latitude,
         ((myGPS.getLongitude()/10000000)*cos(myGPS.getLatitude()/10000000)*111320)-reference_longitude,
         X_t_pred(3,0),
         X_t_pred(4,0),
         X_t_pred(5,0)}; // Measured state (BME280 State)

  R = {0.147880991, 0, 0, 0, 0, 0,
       0, 7.03, 0, 0, 0, 0,
       0, 0, 12.59, 0, 0, 0,
       0, 0, 0, 1.21e-8, 0, 0,
       0, 0, 0, 0, 1.21e-8, 0, 
       0, 0, 0, 0, 0, 1.21e-8}; // Measured noise (co)variance
  K = E_t_pred*((E_t_pred+R).Inverse()); // Kalman Gain

  X_t = X_t_pred+K*(Y_t-X_t_pred); // Current estimate of the rocket's position and velocity in z 

  E_t = (I-K)*E_t_pred; // Current error (co)variance

  //Serial << "Measured position: " << Y_t(0,0) << '\n';
  //Serial << "Predicted position: " << X_t_pred(0,0) << '\n';
  //Serial << "Predicted/Measured velocity: " << Y_t(1,0) << '\n';
  //Serial << "Predicted State: " << X_t_pred << '\n';
  //Serial << "Measured State: " << Y_t << '\n';
  //Serial << X_t(0,0) << '\n';

  X_t_old = X_t; // Previous state estimate is the current state estimate
  E_t_old = E_t; // Previous error (co)variance is the current error (co)variance
  }                 
}
