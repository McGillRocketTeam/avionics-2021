#include <Wire.h>
#include <SD.h>

#define xtendSerial Serial2

File myFile;
String str = "hi\n";

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
  myFile.print("we are writing");
  myFile.close();
  // SD END INIT ======================================================

  // XTEND INITIALIZATION =============================================
  
  // XTEND END INIT ====================================================
}

void loop() {


  Serial.println();
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.print("str");
  myFile.close();
  xtendSerial.println(str);

  Serial.println("All sensor test\n");
  
  delay(1000);
}
