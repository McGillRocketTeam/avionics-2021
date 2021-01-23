/* 
 * Connections to Teensy 4.0
 * Pins: 16     17     18     19
 *       NR    BUSY    DIO   A_SW
 * ----------------------------------------------------------
 * ----------------------------------------------------------
 * ----------------------------------------------------------------
 * ----------- SX1262 Breakout ------------------------------------
 * ----------------------------------------------------------------
 * ----------------------------------------------------------
 * ----------------------------------------------------------
 *       3V3   GND    MISO   MOSI   SCK    SS   
 * Pins: 3V     G      12     11     13    10
 * 
*/

#include <SPI.h>
#include <sx1262.h>

#define RESET 16      //SX126X Reset line
#define BUSY 17       //SX126X BUSY line, must be low before transmission  
#define DIO1 18      // DIO1
#define ANT_SW 19     // Antenna switch

#define NSS 10        //SX126X SPI device select, active low


SX1262 device;

command_status_t command_status;



uint32_t freqeuncy = 448000000; // 448 MHz
uint16_t preamble_length = 12; // From 10 to  65535 symbols
uint8_t tx_address = 0x00;
uint8_t rx_address = 0x00;

uint8_t received_data[5] = {0};

#define irq_set_mask                                0b1000000011 // Set Mask for TXDone, RXDone and Rx/Tx Timeout

void setup() {
  // Initialize BUSY line and SS line

  device.begin(NSS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(9600);
  
  delay(100);

  // Begin RX setup
/*
 * ----- Procedure For RX Setup -------
 * Set device in Standby mode
 * SetPacketType to LoRa
 * SetDIO3AsTXCOCtrl
 * SetDIO2asRFSwitch
 * Calibrate Function
 * IF POSSIBLE --- Calibrate image 
 * Set device in Standby mode
 * SetRegulatorMode to LDO + DC-DC
 * SetRF frequnecy
 * Set bufferaddress pointers with SetBufferBaseAddress(...)
 * Define Modulation Parameters
 * Set Packet Params
 * COnfigure IO pins and IRQ with SetDioIrqParams(...)
 *  if symbol  time  is  equal  or  above  16.38  ms  set low data rate optimization
 * 
 * 
 * 
 */
  
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
  
  command_status = device.setBufferBaseAddress(tx_address, rx_address);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setBufferBaseAddress Failed");
  }
  
  command_status = device.setLoRaModulationParams(LORA_SF7, LORA_BW_062, LORA_CR_4_5, 0x00); // No low data optimization
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
  
  command_status = device.setLoRaPacketParams(preamble_length, 0, 0xFF, 0, 0); // Set variable length, payload size of 5 bytes,  CRC on, and invert_iq is standard
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
  }

  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDioIrqParams Failed");
  }

  command_status = device.stopTimerOnPreamble(0x00);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("stopTimerOnPreamble Failed");
  }

  command_status = device.setLoRaSymbNumTimeout(0x00);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaSymbNumTimeout Failed");
  }
  
  command_status = device.setRx(0xFFFFFF);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRx Failed");
  }

  Serial.println("Setup Complete");
}

void loop() {

// ---------- SIMPLE RECEIVE --------------
  command_status = device.receiveContinuous(received_data, 5, 0);
  if (command_status != COMMAND_SUCCESS) {
    Serial.print("receiveContinuous Failed, Command Status:");
    Serial.println(command_status);
    uint16_t irq__status;
    device.getIrqStatus(&irq__status);
    Serial.print("IRQ Status:");
    Serial.println(irq__status, BIN);
  } else {
    Serial.println("\n");
    Serial.println("-----  Read Data ------");
    for(int i = 0; i< 5; i++) {
      Serial.print(received_data[i]);
    }
    Serial.println("\n");
  }
  
  delay(1000);
  
}
