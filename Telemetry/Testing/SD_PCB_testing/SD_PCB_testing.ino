#include <Wire.h>
#include <SD.h>
#include <Adafruit_BNO055.h>

File myFile;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));

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
}

void loop() {


  Serial.println();
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.print("WE ARE ALIVE!\n");
  myFile.close();

  Serial.println("All sensor test\n");
  
  delay(500);
  delay(500);
}
