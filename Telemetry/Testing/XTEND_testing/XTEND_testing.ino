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
}

void loop() {


  Serial.println(str);
  xtendSerial.println(str);

  Serial.println("All sensor test\n");
  
  delay(1000);
}
