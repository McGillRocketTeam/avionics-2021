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

/* x: default - nothing happens
 * b: begin - initializing the sensors/rtc/gps
 * s: slow transmission - 1s per transmission
 * f: fast transmission - 500ms per transmission
 */
String incomingByte = "x";
boolean isInitialized = false;
int duration = 1000;
float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int SEA_LEVEL_hPA = 102540;
uint8_t runNum = 0;
char fileName[11];

char packet[91];

void setup() {

  Serial.begin(115200);
  xtendSerial.begin(9600);
  Wire.begin();

  // SD INITIALIZATION ================================================
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    sendAndWriteError("SD card rip");
  }
  //looks for free file from 0 to 255, if it reached 255 it will loop back to 0
  sprintf(fileName, "run_%03d.txt", runNum);
  while (SD.exists(fileName)) {
    sprintf(fileName, "run_%03d.txt", ++runNum);
    
    //a full loop has been done
    if (runNum == 0) {
      sendAndWriteError("Check SD card, delete old txt files");
      break;
    }
  }

}

void loop() {

  //======= RX ==========================================================================
  readReceived();

  //initializes 
  if (incomingByte == "b") {
    initialize();                           //initilializes sensor + set incomingByte to "s" + isinit
  }

  //Setting polling frequency
  if (incomingByte == "s") {
    duration = 1000;
  } else if (incomingByte == "f") {
    duration = 500;
  }
  
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
  real_altitude = 44330 * (1.0 - pow(pressure / SEA_LEVEL_hPA, 0.190295));

  //======= NEO =========================================================================
  long latitude = myGPS.getLatitude();
  long longitude = myGPS.getLongitude();

  //======= TX =========================================================================
  sprintf(packet, "%.2f;%.2
  String str = String(pitch) + ";" + String(roll) + ";" + String(yaw) + ";";
  str += String(accel_x) + ";" + String(accel_y) + ";" + String(accel_z) + ";";
  str += String(pressure) +  ";" + String(real_altitude) + ";";
  str += String(latitude) + ";" + String(longitude) + ";";
  str += getDateTime();
  //position estimate - kalman filter

  Serial.println(str);
  
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.print(str+ "\n");
  myFile.close();
  
  delay(500);
}

void initialize() {
  // RTC INITIALIZATION ==============================================
  while (! rtc.begin()) {
    sendAndWriteError("Couldn't find RTC");
  }

  // BNO INITIALIZATION ===============================================
  while (!bno.begin()) {
    sendAndWriteError("BNO055 initialization failed!");
  }  
  bno.setExtCrystalUse(true);

  // BMP INITIALIZATION ===============================================
  while (!bmp.begin()) {
    sendAndWriteError("BMP280 initialization failed!");
  }  
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // NEO INITIALIZATION ===============================================
  while (!myGPS.begin()){ //Connect to the Ublox module using Wire port
    sendAndWriteError("GPS initialization failed!");
  }  
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  // MANUALLY SETTING INCOMINGBYTE ====================================
  incomingByte = "s";
  isInitialized = true;
  sendAndWriteError("Initialized");
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

void readReceived() {
  
  //Reading received byte
  if (xtendSerial.available()) {
    incomingByte = xtendSerial.read();      // will not be -1
    Serial.println("We received data! - " + incomingByte);
  }
}

void sendAndWrite(char* string) {
  
    // Checks if the SD card is well connected to not corrupt the data
  if (SD.exists(fileName)) {
    myFile = SD.open(fileName, FILE_WRITE);
    myFile.print(string);
    myFile.print("\n");
    myFile.close();
  } else {
    Serial.print("SD card is not well connected");
  }

  // Sends data through XTEND
  xtendSerial.print(string);

  // Writes to serial monitor
  Serial.println(string);  
}

void sendAndWriteError(String string) {
  
    // Checks if the SD card is well connected to not corrupt the data
  if (SD.exists(fileName)) {
    myFile = SD.open(fileName, FILE_WRITE);
    myFile.print(string);
    myFile.print("\n");
    myFile.close();
  } else {
    Serial.print("SD card is not well connected");
  }

  // Sends data through XTEND
  xtendSerial.print(string);

  // Writes to serial monitor
  Serial.println(string);  
}
