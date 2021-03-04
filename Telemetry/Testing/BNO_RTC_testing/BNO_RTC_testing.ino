#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_BNO055.h>

RTC_PCF8523 rtc;
Adafruit_BNO055 bno;

float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int seaLevelhPa = 102540;
int RTC_fail = 4;
int BNO_fail = 12;
int LOOP = 22;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));

  digitalWrite(RTC_fail, LOW);
  digitalWrite(BNO_fail, LOW);
  digitalWrite(LOOP, LOW);
  
  // RTC INITIALIZATION ==============================================
  while (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    digitalWrite(RTC_fail, HIGH);
  }
  digitalWrite(RTC_fail, LOW);

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
    digitalWrite(BNO_fail, HIGH);
  }
  digitalWrite(BNO_fail, LOW);
  
  bno.setExtCrystalUse(true);
  // BNO END INIT =====================================================

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
  imu::Vector<3> accel_euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accel_x = accel_euler.x();
  accel_y = accel_euler.y();
  accel_z = accel_euler.z();

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = euler.y();
  roll = euler.z();
  yaw = euler.x();
  
  String str = "";
  str += "Time:";
  str += getDateTime();
  str += ", ";
  str += "Pitch:";
  str += pitch;
  str += ", ";
  str += "Roll:";
  str += roll;
  str += ", ";
  str += "Yaw: ";
  str += yaw;
  str += "\n";

  Serial.println(str);
  Serial.println();

  Serial.println(F("All sensor test"));
  
  delay(500);
  digitalWrite(LOOP, LOW);
  delay(500);
  digitalWrite(LOOP, HIGH);
}
