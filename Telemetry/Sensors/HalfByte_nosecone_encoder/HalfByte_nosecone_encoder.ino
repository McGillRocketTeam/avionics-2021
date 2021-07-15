/*
 * TODO:
 * 1. the 's' was changed back from the 'h'
 * 2. check if the reception was same gibberish
 * 3. send default 000;000 ... instead
 * 4. compare the results from the reception 
 */

#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <sx1262.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include "RTClib.h"

#define MICROSD_CS 9  // MicroSD card read CS
#define SX1262_CS 10  // SX1262 CS
#define RESET 16      //SX1262 Reset line
#define BUSY 14       //SX1262 BUSY line, must be low before transmission  
#define DIO1 15       // DIO1
#define ANT_SW 17     // Antenna switch
#define irq_set_mask                                0b1000000001  // Set mask to detect TX/RX timeout and TxDone

// Declare SX1262 object
SX1262 device;
void sx1262_setup();

// Used to observe command status for SX1262
command_status_t command_status;

// Initialize the IridiumSBD, BMP280, BNO055, RTC, ublox NEO-M9N and SD card file objects
//IridiumSBD modem(IridiumSerial);
Adafruit_BMP280 bmp; 
Adafruit_BNO055 bno;
RTC_PCF8523 rtc;
SFE_UBLOX_GPS myGPS;
File myFile;
DateTime now;

uint32_t freqeuncy = 448000000; // 448 MHz
uint16_t preamble_length = 12;
uint8_t tx_address = 0x00;
uint8_t rx_address = 0x00;

char init_buffer[101];
uint16_t irq;
uint8_t device_status;
uint8_t *send_data;

char RTC_err[] = "Couldn't find RTC";
char BNO_err[] = "BNO055 initialization failed!";
char BMP_err[] = "BMP280 initialization failed!";
char GPS_err[] = "GPS initialization failed!";
char SD_err[] = "microSD reader initialization failed!";
char SD_full_err[] = "microSD card full!";

boolean isInitialized = false;
int duration = 500;
float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
long latitude, longitude;
byte hour, minutes, sec;
imu::Vector<3> accel_euler;
imu::Vector<3> euler;
int seaLevelhPa = 102540;
uint8_t runNum = 0;
char fileName[11];
char errorMess[35];

//halfbyte converter variables
uint8_t index_buffer = 0; //index of the unencoded string
uint8_t len_buffer; //length of the unencoded message
bool isOdd_buffer; //true if the unencoded message is odd lengthed
uint8_t index_encoded; //index of the encoded string
uint8_t halflength; //length of the encoded string
char* encodedData; //char pointer to the encoded string

void setup() {
  device.begin(SX1262_CS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Nose Cone Telemetry test");
    
  initialize();

   // SD INITIALIZATION ================================================
  pinMode(10, OUTPUT);
  while(!SD.begin(MICROSD_CS)){
    Serial.println(SD_err);
  }
  //looks for free file from 0 to 255, if it reached 255 it will loop back to 0
  sprintf(fileName, "run_%03u.txt", runNum);
  while (SD.exists(fileName)) {
    sprintf(fileName, "run_%03d.txt", ++runNum);
    
    //a full loop has been done
    if (runNum == 0) {
      Serial.println(SD_full_err);
      break;
    }
  }
  Serial.print(fileName);

  myFile =SD.open(fileName, FILE_WRITE);
  myFile.print("");
  myFile.close();
  
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
  
  command_status = device.setRfFrequency(freqeuncy); // Set RF frequency
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
  
  command_status = device.setLoRaPacketParams(preamble_length, 0, 0xFF, 1, 0); // Set variable length, payload size of 255 bytes,  CRC on, and invert_iq is standard
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
  }

  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDioIrqParams Failed");
  }

  Serial.println("Setup Complete");
}





void loop() {
  //======= BNO =========================================================================
  accel_euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accel_x = accel_euler.x();
  accel_y = accel_euler.y();
  accel_z = accel_euler.z();

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = euler.y();
  roll = euler.z();
  yaw = euler.x();

  //======= BMP =========================================================================
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();  
  real_altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.190295));

  //======= NEO =========================================================================
  latitude = myGPS.getLatitude();
  longitude = myGPS.getLongitude();

  //======= RTC =========================================================================
  now = rtc.now();
  hour = now.hour();
  minutes = now.minute();
  sec = now.second();

  // Get the IMEI
//  char IMEI[16];
//  err = modem.getIMEI(IMEI, sizeof(IMEI));
//  if (err != ISBD_SUCCESS)
//  {
//     Serial.print(F("getIMEI failed: error "));
//     Serial.println(err);
//     return;
//  }
  snprintf(init_buffer, sizeof(init_buffer), "s%07.2f;%07.2f;%07.2f;%06.2f;%06.2f;%06.2f;%06.2f;%09.2f;%08.2f;%09ld;%09ld;%02hu:%02hu:%02hue",
              pitch, roll, yaw, accel_x, accel_y, accel_z, temperature, pressure, real_altitude, latitude, longitude, hour, minutes, sec);

  writeSerialandSD(init_buffer);
  halfbyteEncoder(init_buffer);  
  
  delay(200);

}

void sx1262_setup() {
  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy Failed");
  }
  
  command_status = device.setPacketType(0x01); // Set packet type to LoRa
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPacketType Failed");
  }
  
  command_status = device.setRfFrequency(freqeuncy); // Set RF frequency
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

void writeSerialandSD(char* string) {
  // Checks if the SD card is well connected to not corrupt the data
  if (SD.exists(fileName)) {
    myFile = SD.open(fileName, FILE_WRITE);
    myFile.println(string);
    myFile.close();
  } else {
    Serial.print("SD card is not well connected");
  }

  Serial.println(string);
  Serial.println();
}

void sendRadio(char* string, uint8_t data_length) {
  
  send_data = (uint8_t*)malloc((int)data_length);
  memcpy(send_data, string, data_length);

  device.clearIrqStatus(SX1262_IRQ_TX_DONE | SX1262_IRQ_TIMEOUT); // Clear TX Done and Timeout Flags
  
  // Write new array to buffer
  command_status = device.writeBuffer(0,send_data,data_length); // No offset
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("writeBuffer Failed");
  }

  command_status = device.setLoRaPacketParams(preamble_length, 0, data_length, 1, 0);

  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
    Serial.print("setLoRaPacketParams command status: ");
    Serial.println(command_status);
  }
  
  command_status = device.setTx(0x061A80); // 400,000 (0x061A80) * 15.625 us = 6.25s
  delay(1400);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setTx Failed");
    Serial.print("Device setTX command status:");
    Serial.println(command_status);

    command_status = device.getStatus(&device_status);
    Serial.print("Device Status");
    Serial.println(device_status, BIN);
  } else {
    while ( (!(irq & SX1262_IRQ_TX_DONE)) && (!(irq & SX1262_IRQ_TIMEOUT)) ) // Wait until tx done or timeout flag is indicated
    {
      device.getIrqStatus(&irq);
    }
  }
  if ((irq & SX1262_IRQ_TIMEOUT)) {
    Serial.println("TIMEOUT!");
  }

  sx1262_setup();
  
  free(send_data);
}

void initialize() {
  // RTC INITIALIZATION ==============================================
  while (! rtc.begin()) {
    Serial.println(RTC_err);
  }

  // BNO INITIALIZATION ===============================================
  while (!bno.begin()) {
    Serial.println(BNO_err);
  }  
  bno.setExtCrystalUse(true);

  // BMP INITIALIZATION ===============================================
  while (!bmp.begin()) {
    Serial.println(BMP_err);
  }  
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // NEO INITIALIZATION ===============================================
  while (!myGPS.begin()){ //Connect to the Ublox module using Wire port
    Serial.println(GPS_err);
  }  
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  // MANUALLY SETTING INCOMINGBYTE ====================================
  isInitialized = true;
  
}

void halfbyteEncoder(char* buffer) {
    
  //Calculating length of new string
  len_buffer = strlen(buffer);
  isOdd_buffer = false;
  if(len_buffer%2 == 1) { //add 1 to make it even
    len_buffer++;
    isOdd_buffer = true; //extra character is added in the encoded string
  }
  halflength = len_buffer/2; //half byte makes it half length

  //Malloc the necessary space
  encodedData = (char*)malloc(halflength); //half the number of characters

  //Combine two chars at a time into one char 
  index_encoded = 0;
  for(index_buffer = 0; index_buffer < len_buffer; index_buffer += 2) { //increased 2 at a time

    //Get the corresponding char
    buffer[index_buffer] = compress(buffer[index_buffer]);
    buffer[index_buffer + 1] = compress(buffer[index_buffer + 1]);

    //Combining the two
    buffer[index_buffer] = buffer[index_buffer] << 4; //Shift the first char by 4 to only put it at the beginning
    encodedData[index_encoded] = buffer[index_buffer] | buffer[index_buffer + 1]; //bitwise or
    
    index_encoded++; //incrementing to next character
  }

  //extra character in case of oddness - add two ending code ('e')
  if(isOdd_buffer) {
    buffer[index_buffer] = compress(buffer[index_buffer]); //get the corresponding code of the last char
    buffer[index_buffer] = buffer[index_buffer] << 4; //shift to the left 
    encodedData[index_encoded]= buffer[index_buffer] | 00001101; //bitwise or
  }

  //Send encoded data through radio
  sendRadio(encodedData, index_encoded);
  
  free(encodedData);
}

char compress(char character) {
  if (character == 's') {
    return(0x0C); //00001100
    
  } else if (character == 'e') {
    return(0x0D); //00001101
    
  } else if (character == ';') {
    return(0x0A); //00001010
    
  } else if (character == ':') {
    return(0x0B); //00001011
    
  } else if (character == '.') {
    return(0x0F); //00001111
    
  } else if (character == '-') {
    return(0x0E); //00001110
    
  } else {
    return(character - '0');
  }
}
