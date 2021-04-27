#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h"

SFE_UBLOX_GPS myGPS;

float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int seaLevelhPa = 102540;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));
  
  // Check BNO
  while (!myGPS.begin()) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("GPS initialization failed!"));
  }
  
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop() {
  long latitude = myGPS.getLatitude();
  latitude /= pow(10,7);

  long longitude = myGPS.getLongitude();
  longitude /= pow(10,7);

  long gpsAltitude = myGPS.getAltitude();
  gpsAltitude /= pow(10,3);
  
  String str = "";
  str += "Latitude:";
  str += latitude;
  str += ",";
  str += "Longitude:";
  str += longitude;
  str += ",";
  str += "Altitude(GPS,m):";
  str += gpsAltitude;
  str += "\n";
  
  Serial.println(str);
  Serial.println();

  Serial.println(F("All sensor test"));
  
  delay(500);
  delay(500);
}
