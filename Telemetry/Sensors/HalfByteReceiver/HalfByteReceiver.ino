
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
 * Pins: 3V    GND     12     11     13    10
 * 
*/

#include <SPI.h>
#include <sx1262.h>
#include <stdio.h>

#define RESET 16      //SX126X Reset line
#define BUSY 17       //SX126X BUSY line, must be low before transmission  
#define DIO1 18      // DIO1
#define ANT_SW 19     // Antenna switch

#define NSS 10        //SX126X SPI device select, active low


SX1262 device; // Create instance of device class

command_status_t command_status;

void sx1262_setup();

uint32_t freqeuncy = 448000000; // 448 MHz
uint16_t preamble_length = 12; // From 10 to  65535 symbols
uint8_t tx_address = 0x00;
uint8_t rx_address = 0x00;

uint8_t received_data[150] = {0};
uint8_t last_packet[150] = {0};

uint8_t rssi = 0; // set initial value of non possible rssi value
float converted_rssi;
uint8_t device_status;
uint8_t rx_status[2] = {0};
uint8_t rx_stats[6] = {0};
uint16_t number_of_packets = 0;
uint16_t number_of_crc_errors = 0;
uint16_t number_of_header_errors = 0;
uint16_t irq_status;
uint8_t loop_counter = 0;
unsigned long break_time;
#define irq_set_mask                                0b1000000011 // Set Mask for TXDone, RXDone and Rx/Tx Timeout

//Halfbyte decoded variables
uint8_t decoded_size; //size of decoded message
uint8_t index_decoded;
uint8_t second_char; //second character encoded in the same char
uint8_t index_encoded;
char* received; //char pointer to the decoded message


void setup() {

  device.begin(NSS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(9600);
  
  delay(10);

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
  
  command_status = device.setLoRaModulationParams(LORA_SF9, LORA_BW_062, LORA_CR_4_5, 0x00); // No low data optimization
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
  
  command_status = device.setLoRaPacketParams(preamble_length, 0, 0xFF, 1, 0); // Set variable length, payload size of 5 bytes,  CRC on, and invert_iq is standard
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
  
  command_status = device.setBufferBaseAddress(tx_address, rx_address);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setBufferBaseAddress Failed");
  }
  
  command_status = device.setLoRaModulationParams(LORA_SF9, LORA_BW_062, LORA_CR_4_5, 0x00); // No low data optimization
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
  
  command_status = device.setLoRaPacketParams(preamble_length, 0, 0xFF, 1, 0); // Set variable length, payload size of 5 bytes,  CRC on, and invert_iq is standard
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
  }

  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDioIrqParams Failed");
  }
}

void loop() {

// ---------- SIMPLE RECEIVE --------------

  device.clearIrqStatus(SX1262_IRQ_RX_DONE);
  command_status = device.setRx(0x061A80); // 400,000 * 15.625 us = 6.25s timeout
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRx Failed");
  }

  device.getIrqStatus(&irq_status);
  break_time = millis();
  while( (!(irq_status & SX1262_IRQ_TIMEOUT)) && (!(irq_status & SX1262_IRQ_RX_DONE)) ){
    device.getIrqStatus(&irq_status);
    if ((millis() - break_time) > 20000) {
      Serial.println("Timeout ERROR!!!");
      break;
    }
  }

  if( irq_status & SX1262_IRQ_TIMEOUT ) {
      Serial.println("RX TIMEOUT!");
      device.clearIrqStatus(SX1262_IRQ_TIMEOUT);
  } else {
    if ((irq_status & SX1262_IRQ_HEADER_ERR) || (irq_status & SX1262_IRQ_CRC_ERR)) {
      device.clearIrqStatus(SX1262_IRQ_HEADER_ERR | SX1262_IRQ_CRC_ERR);
      Serial.println("CRC Error");
    } else if (irq_status & SX1262_IRQ_RX_DONE) {
      
        command_status = device.readBuffer(0, 150, received_data, true);
        
        if (command_status != COMMAND_SUCCESS) {
          Serial.print("receiveContinuous Failed, Command Status: ");
          Serial.println(command_status);
          
          device.getIrqStatus(&irq_status);
          Serial.print("IRQ Status: ");
          Serial.println(irq_status, BIN);
      
          command_status = device.getStatus(&device_status);
          Serial.print("Device Status: ");
          Serial.println(device_status, BIN);
          
        } else {
          device.getIrqStatus(&irq_status);
          Serial.print("IRQ Status: ");
          Serial.println(irq_status, BIN);
          Serial.println("\n");
          device.getRXBufferStatus(rx_status); // Update size (rx_status[0]) and offset (rx_status[1]) for new packet
          Serial.println("-----  Read Data ------");
          Serial.println("Size of packet");
          Serial.println(rx_status[0]);
      
          memcpy(last_packet, received_data, rx_status[0]);
          
          Serial.print((char*)received_data);  
      
          Serial.println("\n");
          Serial.print("RSSI: ");
          device.getRssiInst(&rssi);
          converted_rssi = -rssi/2; // Signal Power in dBm
          Serial.println(converted_rssi);
          Serial.println("\n");
        }
     }
  }
  
  if (loop_counter > 10) {
      device.getStats(rx_stats);
      number_of_packets = (rx_stats[0] << 8) + rx_stats[1];
      Serial.print("Number of Packets: ");
      Serial.println(number_of_packets);
      number_of_crc_errors = (rx_stats[2] << 8) + rx_stats[3];
      Serial.print("Number of CRC Errors: ");
      Serial.println(number_of_crc_errors);
      number_of_header_errors = (rx_stats[4] << 8) + rx_stats[5];
      Serial.print("Number of Header Errors: ");
      Serial.println(number_of_header_errors);
      loop_counter = 0;
    }
  Serial.println();
  Serial.println("Last Packet Received");
  halfbyteDecode(last_packet, rx_status[0]);
  Serial.println(); 
  
  loop_counter++;

//  sx1262_setup(); 
//  delay(200);
}

void halfbyteDecode(uint8_t* buffer, uint8_t size) {

  //Write to serial the encoded message
  Serial.print("Encoded: ");
  Serial.println((char*)buffer);

  //Decoded message should be twice the size of the received packet
  decoded_size = size*2;
  received = (char*)malloc(decoded_size+1); //malloc the correct size + 1 for \0

  //Decode one char at a time and store in two chars
  index_decoded = 0;
  for(index_encoded = 0; index_encoded < size; index_encoded++) {

    second_char = buffer[index_encoded]; //copy char
    buffer[index_encoded] = buffer[index_encoded] >> 4; //shift right by 4 to isolate 4 MSB
    second_char = (second_char & 0b00001111); //isolate 4 LSB
    
    received[index_decoded] = uncompress(buffer[index_encoded]); //decode the specific first character
    received[index_decoded + 1] = uncompress(second_char); //decode the specific second character
    
    index_decoded += 2; //increment to 2nd later index
  }
  
  received[index_decoded]='\0'; //finish string with \0 to stop printing when serial printing
  
  Serial.print("Decoded: ");
  Serial.println(received);
}

char uncompress(uint8_t compressed){
  if (compressed == 0b00001100) {
    return 's';
    
  } else if (compressed == 0b00001101) {
    return 'e';
    
  } else if (compressed == 0b00001010) {
    return ';';
    
  } else if (compressed == 0b00001011) {
    return ':';
    
  } else if (compressed == 0b00001111) {
    return '.';
    
  } else if (compressed == 0b00001110) {
    return '-';
    
  } else {
    return (char)(compressed + '0');
  }  
}
