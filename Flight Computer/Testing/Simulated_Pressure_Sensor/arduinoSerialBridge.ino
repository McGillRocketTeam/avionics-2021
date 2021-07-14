#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX


void setup() {
  Serial.begin(38400); 		// serial between Arduino and PC
  mySerial.begin(38400);	// serial between STM32 and Arduino
  mySerial.println("H");	// not necessary but it's here anyway
}


void loop() {
  
  if (mySerial.available() > 0)		// if STM32 sends data, pass it to PC
  {
    Serial.write(mySerial.read());
  }

  if (Serial.available())			// if PC hsends data, pass it to STM32
  {
    mySerial.write(Serial.read());
  }
  
}
