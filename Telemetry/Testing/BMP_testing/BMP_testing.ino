#include <Wire.h>
#include <SD.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
File myFile;

float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int seaLevelhPa = 102540;
int FAILURE = 4;
int LOOP =22;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));

  digitalWrite(FAILURE, LOW);
  digitalWrite(LOOP, LOW);
  
  // Check BMP
  while (!bmp.begin()) {
    Serial.println(F("BMP280 initialization failed!"));
    digitalWrite(FAILURE, HIGH);
  }
  digitalWrite(FAILURE, LOW);
  
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();


  real_altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.190295));

  String str = "";
   str += "Temp:";
  str += temp;
  str += ",";
  str += "Pressure:";
  str += pressure;
  str += ",";
  str += "Altitude(BMP,m):";
  str += real_altitude;
  
  Serial.println(str);
  Serial.println();

  Serial.println(F("All sensor test"));
  
  delay(500);
  digitalWrite(LOOP, LOW);
  delay(500);
  digitalWrite(LOOP, HIGH);
}
