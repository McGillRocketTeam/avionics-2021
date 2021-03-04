#include <Wire.h>
#include <SD.h>
#include <Adafruit_BNO055.h>

File myFile;
int FAILURE = 4;
int LOOP = 22;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("All sensor test"));

  digitalWrite(FAILURE, LOW);
  digitalWrite(LOOP, LOW);

  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    Serial.println("SD card rip");
    digitalWrite(FAILURE, HIGH);
  }
  digitalWrite(FAILURE, LOW);

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
  myFile.print("WE ARE LIVE!\n");
  myFile.close();

  Serial.println("All sensor test\n");
  
  delay(500);
  digitalWrite(22, LOW);
  delay(500);
  digitalWrite(22, HIGH);
}
