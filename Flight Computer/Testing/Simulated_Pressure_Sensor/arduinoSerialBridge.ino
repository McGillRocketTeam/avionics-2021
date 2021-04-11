#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  mySerial.begin(38400);
  mySerial.println("H");
}


char rxBuf[100];
int counter = 0;
void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available() > 0)
  {
    Serial.write(mySerial.read());
  }

  if (Serial.available())
  {
    mySerial.write(Serial.read());
  }
  
}


//    while (Serial.available())
//      rxBuf[counter++] = Serial.read();
//    counter = 0;
////    Serial.println((char *) rxBuf);
