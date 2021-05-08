#include <SoftwareSerial.h>
#include <sx1262.h>
#include <SPI.h>

#define SX1262_CS 10  // SX1262 CS
#define RESET 16      //SX1262 Reset line
#define BUSY 14       //SX1262 BUSY line, must be low before transmission
#define DIO1 15       // DIO1
#define ANT_SW 17     // Antenna switch
#define irq_set_mask 0b1000000010  // Set mask to detect TX/RX timeout and TxDone
#define data_length 1000

#define DEBUG

static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 38400;
uint32_t freqeuncy = 448000000; // 448 MHz
uint16_t preamble_length = 12;
uint8_t tx_address = 0x00;
uint8_t rx_address = 0x00;

command_status_t command_status;

char init_buffer[100] = "hello";
uint16_t irq;
uint8_t device_status;

char receive_data[data_length];
SX1262 device;
void sx1262_setup();
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void sx1262_setup() {
  delay(100);
  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy Failed");
  }
#endif
  
  command_status = device.setPacketType(0x01); // Set packet type to LoRa
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPacketType Failed");
  }
#endif
  
  command_status = device.setRfFrequency(freqeuncy); // Set RF frequency
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRfFrequency Failed");
  }
#endif

  command_status = device.setPaConfig(DUTY_CYCLE_17dBm, HP_MAX_14dBm, PA_DEVICESEL_1262); // Set Power Amplifier Config
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPaConfig Failed");
  }
#endif

  command_status = device.setTxParams(PLUS_14_dBm, SX1262_PA_RAMP_200U); // Set Tx parameters
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setTxParams Failed");
  }
#endif
  
  command_status = device.setBufferBaseAddress(tx_address, rx_address);
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setBufferBaseAddress Failed");
  }
#endif
  
  command_status = device.setLoRaModulationParams(LORA_SF9, LORA_BW_062, LORA_CR_4_5, 0x00); // No low data optimization
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
#endif

  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDioIrqParams Failed");
  }
#endif
}

void setup()
{
  device.begin(SX1262_CS,BUSY,RESET,DIO1,ANT_SW); // Store pin ports for SX1262 class
  SPI.begin();
  Serial.begin(38400);
  ss.begin(GPSBaud);
  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC

#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy Failed");
  }
#endif
  
  command_status = device.setPacketType(0x01); // Set packet type to LoRa
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPacketType Failed");
  }
#endif

  command_status = device.setRxTxFallbackMode(0x20); // Set fallback mode to STDBY_RC
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRxTxFallbackMode Failed");
  }
#endif

  command_status = device.setDIO2AsRfSwitchCtrl(0x01); // Set DIO2 to control RF swtich
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDIO2AsRfSwitchCtrl Failed");
  }
#endif
  command_status = device.setDIO3AsTCXOCtrl(TCXO_CTRL_3_0V); // Set DIO3 to control TCXO with default delay
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDIO3AsTCXOCtrl Failed");
  }
#endif

  command_status = device.calibrateFunction(0xFF); // Calibrate all 
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("calibrateFunction Failed");
  }
#endif
  delay(50);
  
  command_status = device.setStandBy(0); // Set device in standby mode, STDBY_RC
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setStandBy Failed");
  }
#endif

  command_status = device.setRegulatorMode(0x01); // Set regulator mode to both LDO and DC-DC
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRegulatorMode Failed");
  }
#endif

  command_status = device.calibrateImage(0x6F, 0x75); // Calibrate Image from 440 MHz to 470 MHz
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("calibrateImage Failed");
  }
#endif
  
  command_status = device.setRfFrequency(freqeuncy); // Set RF frequency
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setRfFrequency Failed");
  }
#endif

  command_status = device.setPaConfig(DUTY_CYCLE_17dBm, HP_MAX_14dBm, PA_DEVICESEL_1262); // Set Power Amplifier Config
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setPaConfig Failed");
  }
#endif

  command_status = device.setTxParams(PLUS_14_dBm, SX1262_PA_RAMP_200U); // Set Tx parameters
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setTxParams Failed");
  }
#endif
  
  command_status = device.setBufferBaseAddress(tx_address, rx_address);
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setBufferBaseAddress Failed");
  }
#endif
  
  command_status = device.setLoRaModulationParams(LORA_SF9, LORA_BW_062, LORA_CR_4_5, 0x00); // No low data optimization
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaModulationParams Failed");
  }
#endif
  
  command_status = device.setLoRaPacketParams(preamble_length, 0, 0xFF, 1, 0); // Set variable length, payload size of 255 bytes,  CRC on, and invert_iq is standard
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setLoRaPacketParams Failed");
  }
#endif

  command_status = device.setDioIrqParams(SX1262_IRQ_ALL,irq_set_mask,SX1262_IRQ_NONE,SX1262_IRQ_NONE);
#ifndef DEBUG
  if (command_status != COMMAND_SUCCESS) {
    Serial.println("setDioIrqParams Failed");
  }
#endif
}

void loop()
{

  // This sketch displays information every time a new sentence is correctly encoded.
//  while (ss.available() > 0){
//    Serial.print((char)ss.read());
//  }

  device.clearIrqStatus(SX1262_IRQ_RX_DONE | SX1262_IRQ_TIMEOUT); // Clear TX Done and Timeout Flags

  command_status = device.setLoRaPacketParams(preamble_length, 0, data_length, 1, 0);

  if (command_status != COMMAND_SUCCESS) {
#ifndef DEBUG
    Serial.println("setLoRaPacketParams Failed");
    Serial.print("setLoRaPacketParams command status: ");
    Serial.println(command_status);
#endif
  }
  
  command_status = device.setRx(0x061A80); // 400,000 (0x061A80) * 15.625 us = 6.25s

  device.getIrqStatus(&irq);
#ifndef DEBUG
  Serial.println(irq);
#endif

  if (command_status != COMMAND_SUCCESS) {
#ifndef DEBUG
    Serial.println("setRx Failed");
    Serial.print("Device setRX command status:");
    Serial.println(command_status);
#endif

    command_status = device.getStatus(&device_status);
#ifndef DEBUG
    Serial.print("Device Status");
    Serial.println(device_status, BIN);
#endif
  } else {
    while ( (!(irq & SX1262_IRQ_RX_DONE)) && (!(irq & SX1262_IRQ_TIMEOUT)) ) // Wait until rx done or timeout flag is indicated
    {
      device.getIrqStatus(&irq);
    }
  }

  if ((irq & SX1262_IRQ_TIMEOUT)) {
#ifndef DEBUG
    Serial.println("TIMEOUT!");
#endif
  }
  uint8_t buf[2];
  device.getRXBufferStatus(buf);
  
#ifndef DEBUG
  Serial.print("Size of payload: ");
  Serial.println(buf[0]);
  Serial.print("Offset of payload: ");
  Serial.println(buf[1]);
#endif
  device.readBuffer(buf[1], buf[0], (uint8_t*)receive_data, true);
  Serial.println(receive_data);
  sx1262_setup();

  delay(2000);
}
