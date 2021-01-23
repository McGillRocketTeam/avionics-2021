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

lora_sf_t sf = LORA_SF8;
lora_bw_t bw = LORA_BW_041;
lora_cr_t cr = LORA_CR_4_6;

tcxo_ctrl_voltages_t txco_voltage = TCXO_CTRL_3_0V;
uint32_t freqeuncy = 448000000; // 448 MHz

uint16_t DIO_clear_mask = 0; // all bits 0
uint16_t preamble_length = 8;

uint8_t tx_address = 0x00;
uint8_t rx_address = 0x00;


//uint8_t* RxData = new uint8_t[5];
uint8_t data[5] = {1,2,3,4,5};
uint8_t read_data[5] = {0};
int value = 1;

#define SX126X_IRQ_ALL                              0b1111111111  //  9     0     all interrupts
#define SX126X_IRQ_NONE                             0b0000000000  //  9     0     no interrupts

#define SX126X_IRQ_TIMEOUT                          0b1000000000  //  9     9     Rx or Tx timeout
#define SX126X_IRQ_CAD_DETECTED                     0b0100000000  //  8     8     channel activity detected
#define SX126X_IRQ_CAD_DONE                         0b0010000000  //  7     7     channel activity detection finished
#define SX126X_IRQ_CRC_ERR                          0b0001000000  //  6     6     wrong CRC received
#define SX126X_IRQ_HEADER_ERR                       0b0000100000  //  5     5     LoRa header CRC error
#define SX126X_IRQ_HEADER_VALID                     0b0000010000  //  4     4     valid LoRa header received
#define SX126X_IRQ_SYNC_WORD_VALID                  0b0000001000  //  3     3     valid sync word detected
#define SX126X_IRQ_PREAMBLE_DETECTED                0b0000000100  //  2     2     preamble detected
#define SX126X_IRQ_RX_DONE                          0b0000000010  //  1     1     packet received
#define SX126X_IRQ_TX_DONE                          0b0000000001  //  0     0     packet transmission completed

#define irq_set_mask                                0b1000000011
uint8_t number = 250;

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

  command_status = device.setDIO2AsRfSwitchCtrl(0x01); // Set DIO2 to control RF swtich
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDIO2AsRfSwitchCtrl Failed");
  }
  command_status = device.setDIO3AsTCXOCtrl(txco_voltage); // Set DIO3 to control TCXO with default delay
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
  
  command_status = device.setRfFrequency(freqeuncy); // Set RF frequency
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRfFrequency Failed");
  }
  
  command_status = device.setBufferBaseAddress(tx_address, rx_address);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setBufferBaseAddress Failed");
  }
  
  command_status = device.setLoRaModulationParams(sf, bw, cr, 0); // No low data optimization
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
  
  command_status = device.setLoRaPacketParams(preamble_length, 1, 5, 1, 0); // Set fixed length, payload size of 5 bytes,  CRC on, and invert_iq is standard
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
  }

  command_status = device.setDioIrqParams(SX126X_IRQ_ALL,irq_set_mask,SX126X_IRQ_NONE,SX126X_IRQ_NONE);
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
  for(int i = 0; i< 5; i++) {
    data[i] = i+value;
    Serial.print(data[i]);
  }
  Serial.println("");

  // Write new array to buffer
  device.writeBuffer(0,data,5);

  delay(500);

  // Read new array in buffer
  device.readBuffer(0, 5, read_data);
  
  Serial.println("Read Data");
  for(int i = 0; i< 5; i++) {
    Serial.print(read_data[i]);
  }
  Serial.println("");

  value++;
  if (value >250){
    while (true) {}
  }
  delay(5000);
}
