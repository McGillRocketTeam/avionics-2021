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

char init_buffer[150] = {0};
uint16_t irq;
uint8_t device_status;
uint8_t data_length = 0;
uint8_t *send_data;

float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
int latitude, longitude;

imu::Vector<3> accel_euler;
imu::Vector<3> euler;
int seaLevelhPa = 102540;
int i = 0;
int err;

void initialize() {
  // Check BMP
  if (!bmp.begin()) {
    Serial.println(F("BMP280 initialization failed!"));
    while (1);
  }
  
  // Check BNO
  if (!bno.begin())
  {
    Serial.print("BNO055 initialization failed!");
    while (1);
  }

  // Check GPS
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("GPS initialization failed!"));
    while (1);
  }

  // Check SD Card
  pinMode(MICROSD_CS, OUTPUT);
  digitalWrite(MICROSD_CS, HIGH); // Unselect SD Card
  if (!SD.begin(9)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  // Check RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  // Remove file if exists
  if (SD.exists("example.txt")) {
     SD.remove("example.txt");
  }
  // Create file
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.close();

  char header[] = "Temperature;Pressure;Altitude(BMP,m);Pitch;Roll;Yaw;AccelX;AccelY;AccelZ;Latitude;Longitude;Time";
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.println(header);
  myFile.close();

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  bno.setExtCrystalUse(true);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  
}

void setup() {
  device.begin(SX1262_CS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Nose Cone Telemetry test");
    
  
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

void loop() {
  accel_euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accel_x = accel_euler.x();
  accel_y = accel_euler.y();
  accel_z = accel_euler.z();

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = euler.y();
  roll = euler.z();
  yaw = euler.x();

  temp = bmp.readTemperature();
  pressure = bmp.readPressure();

  real_altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.190295));

  latitude = myGPS.getLatitude();

  longitude = myGPS.getLongitude();

  // Get the IMEI
//  char IMEI[16];
//  err = modem.getIMEI(IMEI, sizeof(IMEI));
//  if (err != ISBD_SUCCESS)
//  {
//     Serial.print(F("getIMEI failed: error "));
//     Serial.println(err);
//     return;
//  }
  
  now = rtc.now();

  data_length = snprintf(init_buffer, sizeof(init_buffer), "%f;%f;%f;%f;%f;%f;%f;%d;%d;%d:%d:%d", real_altitude,pitch,roll,yaw,accel_x,accel_y,accel_z,
                                                                                            latitude,longitude,now.hour(),now.minute(),now.second());
  data_length++;
  data_length++;
  Serial.println("Data Length");
  Serial.println(data_length);
  
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.println(init_buffer);
  myFile.close();

  send_data = (uint8_t*)malloc(data_length);
  memcpy(send_data, init_buffer, data_length);

  Serial.println((char*)send_data);
  Serial.println();

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
  delay(200);

}
