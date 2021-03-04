#include <Wire.h>
#include <SD.h>
#include "SparkFun_Ublox_Arduino_Library.h"

SFE_UBLOX_GPS myGPS;
File myFile;

float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int seaLevelhPa = 102540;
int NEO_fail = 4;
int SD_fail = 5;
int LOOP =22;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));

  digitalWrite(NEO_fail, LOW);
  digitalWrite(SD_fail, LOW);
  digitalWrite(LOOP, LOW);
  
  // NEO INITIALIZATION ===============================================
  while (!myGPS.begin()) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("GPS initialization failed!"));
    digitalWrite(NEO_fail, HIGH);
  }
  digitalWrite(NEO_fail, LOW);
  
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  // NEO END INIT =====================================================
  
  // SD INITIALIZATION ================================================
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    Serial.println("SD card rip");
    digitalWrite(SD_fail, HIGH);
  }
  digitalWrite(SD_fail, LOW);

  // Remove file if exists
  if (SD.exists("example.txt")) {
     SD.remove("example.txt");
  }
  myFile =SD.open("example.txt", FILE_WRITE);
  myFile.print("we are writing");
  myFile.close();
  // SD END INIT ======================================================
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

  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.print(str + "\n");
  myFile.close();
  
  delay(500);
  digitalWrite(LOOP, LOW);
  delay(500);
  digitalWrite(LOOP, HIGH);
}
