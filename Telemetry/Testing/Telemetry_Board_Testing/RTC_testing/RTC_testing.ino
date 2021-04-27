#include <Wire.h>
#include "RTClib.h"

RTC_PCF8523 rtc;

float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int seaLevelhPa = 102540;
int FAILURE = 4;
int LOOP =22;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));

  digitalWrite(FAILURE, LOW);
  digitalWrite(LOOP, LOW);
  
  // Check RTC
  while (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    digitalWrite(FAILURE, HIGH);
  }
  digitalWrite(FAILURE, LOW);

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

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
  // put your main code here, to run repeatedly:
  String str = "";
  str += "Time:";
  str += getDateTime();
  str += "\n";

  Serial.println(str);
  Serial.println();

  Serial.println(F("All sensor test"));
  
  delay(500);
  digitalWrite(LOOP, LOW);
  delay(500);
  digitalWrite(LOOP, HIGH);
}
