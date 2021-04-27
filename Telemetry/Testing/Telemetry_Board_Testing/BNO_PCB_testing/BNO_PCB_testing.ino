#include <Wire.h>
#include <SD.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno;
File myFile;

float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int seaLevelhPa = 102540;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));
  
  // Check BNO
  while (!bno.begin())
  {
    Serial.print("BNO055 initialization failed!\n");
  }
  
  bno.setExtCrystalUse(true);
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
  str += "Pitch:";
  str += pitch;
  str += ",";
  str += "Roll:";
  str += roll;
  str += ",";
  str += "Yaw:";
  str += yaw;
  str += "\n";
  
  Serial.println(str);
  Serial.println();

  Serial.println(F("All sensor test"));
  
  delay(500);
  delay(500);
}
