#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <sx1262.h>
#include "RTClib.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include "SparkFun_Ublox_Arduino_Library.h"

#define MICROSD_CS 9  // MicroSD card read CS
#define SX1262_CS 10  // SX1262 CS
#define RESET 16      //SX1262 Reset line
#define BUSY 14       //SX1262 BUSY line, must be low before transmission  
#define DIO1 15       // DIO1
#define ANT_SW 17     // Antenna switch
#define irq_set_mask                                0b1000000001  // Set mask to detect TX/RX timeout and TxDone

SX1262 device; // Create instance of SX1262 class
Adafruit_BNO055 bno;
Adafruit_BMP280 bmp;
SFE_UBLOX_GPS myGPS;
RTC_PCF8523 rtc;
File myFile;

command_status_t command_status;

uint32_t frequency = 448000000; // 448 MHz
uint16_t preamble_length = 12;
uint8_t tx_address = 0x00;
uint8_t rx_address = 0x00;

uint16_t irq;
uint16_t irq_status;
uint8_t device_status;
char packet[94]; 
uint8_t string_length;

#define irq_set_mask                                0b1000000001  // Set mask to detect TX/RX timeout and TxDone

char RTC_err[] = "Couldn't find RTC";
char BNO_err[] = "BNO055 initialization failed!";
char BMP_err[] = "BMP280 initialization failed!";
char GPS_err[] = "GPS initialization failed!";
char SD_err[] = "microSD reader initialization failed!";
char SD_full_err[] = "microSD card full!";

boolean isInitialized = false;
int duration = 500;
float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
long longitude, latitude;
byte hour, minutes, sec;
DateTime now;
int SEA_LEVEL_hPA = 102540;
uint8_t runNum = 0;
char fileName[11];
char errorMess[35];

void setup() {
  
  device.begin(SX1262_CS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Wire.begin();
  Serial.begin(9600);
  
  delay(10);

  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy Failed");
  }
  
  command_status = device.setPacketType(0x01); // Set packet type to LoRa
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPacketType Failed");
  }

  command_status = device.setRxTxFallbackMode(0x20); // Set fallback mode to STDBY_RC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRxTxFallbackMode Failed");
  }

  command_status = device.setDIO2AsRfSwitchCtrl(0x01); // Set DIO2 to control RF swtich
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDIO2AsRfSwitchCtrl Failed");
  }
  command_status = device.setDIO3AsTCXOCtrl(TCXO_CTRL_3_0V); // Set DIO3 to control TCXO with default delay
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDIO3AsTCXOCtrl Failed");
  }

  command_status = device.calibrateFunction(0xFF); // Calibrate all 
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("calibrateFunction Failed");
  } 
  delay(50);
  
  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy Failed");
  }

  command_status = device.setRegulatorMode(0x01); // Set regulator mode to both LDO and DC-DC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRegulatorMode Failed");
  }

  command_status = device.calibrateImage(0x6F, 0x75); // Calibrate Image from 440 MHz to 470 MHz
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("calibrateImage Failed");
  }
  
  command_status = device.setRfFrequency(frequency); // Set RF frequency
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRfFrequency Failed");
  }

  command_status = device.setPaConfig(DUTY_CYCLE_17dBm, HP_MAX_14dBm, PA_DEVICESEL_1262); // Set Power Amplifier Config
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPaConfig Failed");
  }

  command_status = device.setTxParams(PLUS_14_dBm, SX1262_PA_RAMP_200U); // Set Tx parameters
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setTxParams Failed");
  }
  
  command_status = device.setBufferBaseAddress(tx_address, rx_address);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setBufferBaseAddress Failed");
  }
  
  command_status = device.setLoRaModulationParams(LORA_SF9, LORA_BW_062, LORA_CR_4_5, 0x00); // No low data optimization
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
  
  command_status = device.setLoRaPacketParams(preamble_length, 0, 0xFF, 1, 0); // Set variable length, payload size of 5 bytes,  CRC off, and invert_iq is standard
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
  }

  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDioIrqParams Failed");
  }

  Serial.println("Radio Setup Complete");

  // SD INITIALIZATION ================================================
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    sendAndWrite(SD_err);
  }
  //looks for free file from 0 to 255, if it reached 255 it will loop back to 0
  sprintf(fileName, "run_%03u.txt", runNum);
  while (SD.exists(fileName)) {
    sprintf(fileName, "run_%03d.txt", ++runNum);
    
    //a full loop has been done
    if (runNum == 0) {
      sendAndWrite(SD_full_err);
      break;
    }
  }
  Serial.print(fileName);

  myFile =SD.open(fileName, FILE_WRITE);
  myFile.print("");
  myFile.close();

}

void loop() {

  //dummy if statement to replace the frequency change by receiving a bit from gs
  if (isInitialized) {
    initialize();
  }
  
  if (isInitialized) {
    //======= BNO =========================================================================
    imu::Vector<3> accel_euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    accel_x = accel_euler.x();
    accel_y = accel_euler.y();
    accel_z = accel_euler.z();
  
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    pitch = euler.y();
    roll = euler.z();
    yaw = euler.x();
  
    //======= BMP =========================================================================
    temp = bmp.readTemperature();
    pressure = bmp.readPressure();  
    real_altitude = 44330 * (1.0 - pow(pressure / SEA_LEVEL_hPA, 0.190295));
  
    //======= NEO =========================================================================
    latitude = myGPS.getLatitude();
    longitude = myGPS.getLongitude();
  
    //======= RTC =========================================================================
    now = rtc.now();
    hour = now.hour();
    minutes = now.minute();
    sec = now.second();
  
    //======= TX =========================================================================
//    sprintf(packet, "s%07.2f;%07.2f;%07.2f;%06.2f;%06.2f;%06.2f;%09.2f;%08.2f;%09ld;%09ld;%02hu:%02hu:%02hue",
//              pitch, roll, yaw, accel_x, accel_y, accel_z, pressure, real_altitude, latitude, longitude, hour, minutes, sec);
    sprintf(packet, "bonjour");
    sendAndWrite(packet);
  }
  
  //note: there's an extra delay inside transmission to not corrupt the sx1262
  delay(duration);
}

void sendAndWrite(char* data_string) {

  //==== SD CARD ===================================================================
  // Checks if the SD card is well connected to not corrupt the data
  if (SD.exists(fileName)) {
    myFile = SD.open(fileName, FILE_WRITE);
    myFile.print(data_string);
    myFile.print("\n");
    myFile.close();
  } else {
    Serial.print("SD card is not well connected");
  }

  //==== SERIAL MONITOR =============================================================
  Serial.println(data_string);
  
  //==== TRANSMITTING ===============================================================
  string_length = sizeof(data_string);
  device.clearIrqStatus(SX1262_IRQ_TX_DONE | SX1262_IRQ_TIMEOUT);

  command_status = device.writeBuffer(0,(uint8_t*)data_string,string_length);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("writeBuffer Failed");
  }
  
  command_status = device.setLoRaPacketParams(preamble_length, 0, string_length, 1, 0); // Set variable length, payload size of 5 bytes,  CRC off, and invert_iq is standard
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
    device.getIrqStatus(&irq_status);
    Serial.print("PacketParam IRQ Status: ");
    Serial.println(irq_status, BIN);
  }
  
  command_status = device.setTx(0x061A80); // 400,000 * 15.625 us = 6.25s
  delay(1000);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setTx Failed");
    Serial.print("Device setTX command status:");
    Serial.println(command_status);

    command_status = device.getStatus(&device_status);
    Serial.print("Device Status");
    Serial.println(device_status, BIN);
  }
  
  while ( (!(irq & SX1262_IRQ_TX_DONE)) && (!(irq & SX1262_IRQ_TIMEOUT)) ) // Wait until tx done or timeout flag is indicated
  {
    device.getIrqStatus(&irq);
  }
  
  if ((irq & SX1262_IRQ_TIMEOUT)) {
    Serial.println("TIMEOUT!");
  }

  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy Failed");
  }

  command_status = device.setPacketType(0x01); // Set packet type to LoRa
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPacketType Failed");
  }

  command_status = device.setRfFrequency(frequency); // Set RF frequency
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRfFrequency Failed");
  }

  command_status = device.setPaConfig(DUTY_CYCLE_17dBm, HP_MAX_14dBm, PA_DEVICESEL_1262); // Set Power Amplifier Config
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPaConfig Failed");
  }

  command_status = device.setTxParams(PLUS_14_dBm, SX1262_PA_RAMP_200U); // Set Tx parameters
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setTxParams Failed");
  }
  
  command_status = device.setBufferBaseAddress(tx_address, rx_address);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setBufferBaseAddress Failed");
  }
  
  command_status = device.setLoRaModulationParams(LORA_SF9, LORA_BW_062, LORA_CR_4_5, 0x00); // No low data optimization
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
  
  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDioIrqParams Failed");
  }
}

void initialize() {
  // RTC INITIALIZATION ==============================================
  while (! rtc.begin()) {
    sendAndWrite(RTC_err);
  }

  // BNO INITIALIZATION ===============================================
  while (!bno.begin()) {
    sendAndWrite(BNO_err);
  }  
  bno.setExtCrystalUse(true);

  // BMP INITIALIZATION ===============================================
  while (!bmp.begin()) {
    sendAndWrite(BMP_err);
  }  
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // NEO INITIALIZATION ===============================================
  while (!myGPS.begin()){ //Connect to the Ublox module using Wire port
    sendAndWrite(GPS_err);
  }  
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  // MANUALLY SETTING INCOMINGBYTE ====================================
  isInitialized = true;
}
