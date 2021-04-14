#include <Wire.h>
#include <SD.h>
#include "RTClib.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include "SparkFun_Ublox_Arduino_Library.h"

#define xtendSerial Serial2
Adafruit_BNO055 bno;
Adafruit_BMP280 bmp;
SFE_UBLOX_GPS myGPS;
RTC_PCF8523 rtc;
File myFile;


float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int seaLevelhPa = 102540;
String str = "hi\n";
/* x: default - nothing happens
 * b: begin - initializing the sensors/rtc/gps
 * s: slow transmission - 1s per transmission
 * f: fast transmission - 500ms per transmission
 */
String incomingByte = "x";
boolean isInitialized = false;
int duration = 1000;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println(F("All sensor test"));
  xtendSerial.begin(9600);

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
  myFile.print("We are writing");
  myFile.close();
}

void loop() {

  readReceived();

  //constant polling to reduce power usage
  while(incomingByte == "x"){
    readReceived();
  }

  //We are out of x 
  if (incomingByte == "b") {
    initialize();                           //initilializes sensor + set incomingByte to "s" + isinit
  }

  //Setting polling frequency
  if (incomingByte == "s") {
    duration = 1000;
  } else if (incomingByte == "f") {
    duration = 500;
  }

  if (isInitialized) {
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
  
    long longitude = myGPS.getLongitude();
  
    long gpsAltitude = myGPS.getAltitude();
    gpsAltitude /= pow(10,3);
  
    //======= str =========================================================================
    str = "Temp: " + String(temp) + ", ";
    str += "Pressure:" + String(pressure) +  ", ";
    str += "Altitude(BMP,m):" + String(real_altitude) + ",";
    str += "Pitch:" + String(pitch) + ",";
    str += "Roll:" + String(roll) + ",";
    str += "Yaw:" + String(yaw) + ",";
    str += "Latitude:" + String(latitude) + ",";
    str += "Longitude:" + String(longitude) + ",";
    str += "Altitude(GPS,m):" + String(gpsAltitude) + ",";
    str += "Time:" + getDateTime() + "\n";
    Serial.println(str + "\n");
    
    //====== sending ======================================================================
    fileWrite(str);
    xtendSerial.print(str);
  }

  delay(duration);
}

void readReceived() {
  
  //Reading received byte
  if (xtendSerial.available()) {
    incomingByte = xtendSerial.read();      // will not be -1
    Serial.println("We received data! - \n" + incomingByte);
  }
}

void initialize() {
  // RTC INITIALIZATION ==============================================
  while (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    fileWrite("Couldn't find RTC");
    xtendSerial.print("noRTC");
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    fileWrite("RTC is not running");
    xtendSerial.print("RTC no run");
  }

  // BNO INITIALIZATION ===============================================
   while (!bno.begin())
  {
    Serial.print("BNO055 initialization failed!");
    fileWrite("BNO initialization failed!");
    xtendSerial.print("noBNO");
  }
  
  bno.setExtCrystalUse(true);

  // BMP INITIALIZATION ===============================================
  while (!bmp.begin()) {
    Serial.println(F("BMP280 initialization failed!"));
    fileWrite("BMP280 initialization failed!");
    xtendSerial.print("noBMP");
  }
  
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // NEO INITIALIZATION ===============================================
  while (!myGPS.begin()) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("GPS initialization failed!"));
    fileWrite("GPS initialization failed!");
    xtendSerial.print("noGPS");
  }
  
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  // MANUALLY SETTING INCOMINGBYTE ====================================
  incomingByte = "s";
  isInitialized = true;
  xtendSerial.print("init");
  fileWrite("Initialized");  
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

void fileWrite(String s) {
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.print(s + "\n");
  myFile.close();
}
