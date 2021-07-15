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

SX1262 device; // Create instance of SX1262 class

command_status_t command_status;

uint32_t freqeuncy = 448000000; // 448 MHz
uint16_t preamble_length = 12;
uint8_t tx_address = 0x00;
uint8_t rx_address = 0x00;

uint16_t value = 1;
uint16_t irq;
uint16_t irq_status;
uint8_t device_status;
char data_string[] = "24.42;32.3;34.32";
uint8_t string_length = sizeof(data_string);

#define irq_set_mask                                0b1000000001  // Set mask to detect TX/RX timeout and TxDone


void setup() {

  device.begin(NSS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(9600);
  
  delay(10);

  // Begin TX setup
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
 * SetPaConfig
 * SetTxparams
 * **Change TxClampConfig
 * Set bufferaddress pointers with SetBufferBaseAddress(...)
 * Define Modulation Parameters
 * Set Packet Params
 * COnfigure IO pins and IRQ with SetDioIrqParams(...)
 * Send the payload to the data buffer with the command WriteBuffer(...)
 * start transmission with the command SetTx()
 * Wait for the IRQ TxDone or Timeout: once the packet has been sent the chip goes automatically to STDBY_RC mode
 * Clear the IRQ TxDone flag
 * 
 *  if symbol  time  is  equal  or  above  16.38  ms  set low data rate optimization
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

  Serial.println("Setup Complete");
}

void loop() {
 delay(500);
// ------------- Simple TX ----------------------------- 

  Serial.println(data_string);

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
