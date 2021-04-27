#include <Wire.h>
#include <SD.h>
#include "RTClib.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include "SparkFun_Ublox_Arduino_Library.h"


Adafruit_BNO055 bno;
Adafruit_BMP280 bmp;
SFE_UBLOX_GPS myGPS;
RTC_PCF8523 rtc;

File myFile;

float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int seaLevelhPa = 102540;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));
  
  // RTC INITIALIZATION ==============================================
  while (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  // RTC END INIT =====================================================

  // BNO INITIALIZATION ===============================================
   while (!bno.begin())
  {
    Serial.print("BNO055 initialization failed!");
  }
  
  bno.setExtCrystalUse(true);
  // BNO END INIT =====================================================

  // BMP INITIALIZATION ===============================================
  while (!bmp.begin()) {
    Serial.println(F("BMP280 initialization failed!"));
  }
  
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  // BMP END INIT =====================================================

  // NEO INITIALIZATION ===============================================
  while (!myGPS.begin()) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("GPS initialization failed!"));
  }
  
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  // NEO END INIT =====================================================

  // SD INITIALIZATION ================================================
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    Serial.println("SD card rip");
  }

  // Remove file if exists
  if (SD.exists("example.txt")) {
     SD.remove("example.txt");
  }
  myFile =SD.open("example.txt", FILE_WRITE);
  myFile.print("we are writing");
  myFile.close();
  // SD END INIT ======================================================

}

String getDateTime(){
  String dateTime = "";
  DateTime now = rtc.now();
  dateTime += String(now.hour(), DEC);
  dateTime += ":";
  dateTime += String(now.minute(), DEC);
  dateTime += ":";
  dateTime += String(now.second(), DEC);
  return dateTime;
}

void loop() {

  //======= BNO =========================================================================
  imu::Vector<3> accel_euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accel_x = accel_euler.x();
  accel_y = accel_euler.y();
  accel_z = accel_euler.z();

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = euler.y();
  roll = euler.z();
  yaw = euler.x();

  //======= BMP =========================================================================
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  
  real_altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.190295));

  //======= NEO =========================================================================
  long latitude = myGPS.getLatitude();
  latitude /= pow(10,7);

  long longitude = myGPS.getLongitude();
  longitude /= pow(10,7);

  long gpsAltitude = myGPS.getAltitude();
  gpsAltitude /= pow(10,3);

  //======= str =========================================================================
  String str = "Temp: " + String(temp) + ",";
  str += "Pressure:" + String(pressure) +  ",";
  str += "Altitude(BMP,m):" + String(real_altitude) + ",";
  str += "Pitch:" + String(pitch) + ",";
  str += "Roll:" + String(roll) + ",";
  str += "Yaw:" + String(yaw) + ",";
  str += "Latitude:" + String(latitude) + ",";
  str += "Longitude:" + String(longitude) + ",";
  str += "Altitude(GPS,m):" + String(gpsAltitude) + ",";
  str += "Time:" + getDateTime() + "\n";

  Serial.println(str);
  
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.print(str+ "\n");
  myFile.close();
  
  delay(500);
  delay(500);
}
