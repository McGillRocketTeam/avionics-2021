#include <IridiumSBD.h>
#include <SoftwareSerial.h>

int Setup = 1;
int ByteReceived;
#define STATUS 13
#define ROCKBLOCK_RX_PIN  3 // Recieve data pin from Rockblock (seial data from RockBLOCK)
#define ROCKBLOCK_TX_PIN  4 // Transmit data pin to Rockblock (serial data to RockBLOCK)
#define ROCKBLOCK_SLEEP_PIN 4 // on/off pin for power savings
#define ROCKBLOCK_BAUD 19200 // serial modem communication baud rate
//#define CONSOLE_BAUD 115200 // serial terminal communications baud rate
#define CONSOLE_BAUD 19200 // serial terminal communications baud rate
#define DIAGNOSTICS true // Set "true" to see serial diagnostics

SoftwareSerial ssIridium(ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN);  // Type Arduino Stream
IridiumSBD isbd(ssIridium, ROCKBLOCK_SLEEP_PIN);               // this is my RockBLOCK

void setup()
{
 // Start the serial port at 115200 baud rate so that I can 
 // see output on the a terminal from modem initiation   
 Serial.begin(CONSOLE_BAUD);

 delay(1000);  
 Serial.println("Begin REM setup.");    
  
 // LED on 13 for reporting status on modem communication
 pinMode(STATUS, OUTPUT); 

 // Sleep pin on modem
 pinMode(ROCKBLOCK_SLEEP_PIN, OUTPUT);
 digitalWrite(ROCKBLOCK_SLEEP_PIN, HIGH);

 int n=25;
 // Setup the RockBLOCK
 isbd.adjustSendReceiveTimeout(n); // This determines how long RockBLOCK will try to send message before timeout.
                                   // n = 25 seconds per attempt before system times out as per global variable setting.
                                   // If transmission times out, will delay according to setPowerProfile() so that 
                                   // modem power (capacitor) has an opportunity to recharge.
  
 isbd.setPowerProfile(1); // Use high power (0) when running off high-capacity lipo.
                          // Currently set to (1) for testing / development via laptop power.


 Serial.println("REM setup complete.");       


}

void loop() {
 
 if(Setup == 1) {  
  // Unit was just powered on
  Serial.println("REM was just powered on.");
  Serial.println("Modem will test connection.");
  Setup = 0;  // finish setup
 }
  /*  
  delay(500);
  Serial.print("Sending...\t\t");
  Serial.println("AT\r");
  Serial.write("AT\r");
  delay(500);
  */
  
  if (Serial.available() > 0)
  {
    String h;
    //ByteReceived = Serial.read();
    h = Serial.readString();
    Serial.print("\nReceiving...\t\t\n");
    //Serial.print(ByteReceived);   
    Serial.print(h);
    Serial.print("        ");      
    Serial.print(ByteReceived, HEX);
    Serial.print("       ");     
    Serial.print(char(ByteReceived));
    
    if(ByteReceived == '1') // Single Quote! This is a character.
    {
      digitalWrite(STATUS,HIGH);
      Serial.print(" LED ON ");
    }
    
    if(ByteReceived == '0')
    {
      digitalWrite(STATUS,LOW);
      Serial.print(" LED OFF");
    }
    
    Serial.println();    // End the line

  // END Serial Available
  }
  //Serial.println("");
  delay(500);
}
