/*
    sx1262.cpp - MRT Semtech SX1262 Library
    Created by Jake Peters

    Things to Finish:
        
*/

#include <sx1262.h>
#include <SPI.h>
#include "Arduino.h"

/* 
    ============================= Setup ================================
*/
SX1262::SX1262(void) {

}

void SX1262::begin(uint8_t NSS, uint8_t BUSY, uint8_t RESET, uint8_t DIO1, uint8_t ANT_SW) {
    _NSS = NSS;
    _BUSY = BUSY;
    _RESET = RESET;
    _DIO1 = DIO1;
    _ANT_SW = ANT_SW;

    pinMode(_BUSY, INPUT); 
    pinMode(_NSS, OUTPUT);
    pinMode(_DIO1, INPUT);
    pinMode(_RESET,OUTPUT);
  
    digitalWrite(_NSS, HIGH); // Unselect device

    delay(10);
    digitalWrite(_RESET,0);
    delay(50);
    digitalWrite(_RESET,1);
    delay(50);
}


/*
    ================================ Transmit/Receive Functions ==================
*/
command_status_t SX1262::receiveContinuous(uint8_t *received_data, uint8_t length, uint8_t offset) {

    uint16_t irq_status;
    getIrqStatus(&irq_status);
    
    if( irq_status & SX1262_IRQ_RX_DONE ) {
        clearIrqStatus(SX1262_IRQ_RX_DONE);
        if ((irq_status & SX1262_IRQ_HEADER_ERR) || (irq_status & SX1262_IRQ_CRC_ERR)) {
            clearIrqStatus(SX1262_IRQ_HEADER_ERR | SX1262_IRQ_CRC_ERR);
            return COMMAND_ERROR_FLAG;
        }
        return readBuffer(offset, length, received_data, true);
    }
    return COMMAND_ERROR_FLAG;
}

/* 
    ============================= Passing Instructions ================================
*/

command_status_t SX1262::writeCommand(uint8_t opcode, uint8_t *cmd_buffer, uint16_t cmd_len) {
    command_status_t status = COMMAND_SUCCESS;

    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

    // while(digitalRead(_BUSY));
    if (isBusy()) {
        return COMMAND_BUSY_TIMEOUT;
    }

    digitalWrite(_NSS, LOW);
    SPI.transfer(opcode);
    uint8_t cmd_status = 0; 
    for (uint8_t i = 0; i < cmd_len; i++) {
        cmd_status = SPI.transfer(cmd_buffer[i]); 
    }
    if( ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_TIMEOUT) ||
        ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_ERROR) ||
        ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_FAILED) ||
        (cmd_status == 0x00) || (cmd_status == 0xFF)) {
            Serial.println("Command Error!");
            for (uint8_t i = 0; i < cmd_len; i++) {
                Serial.print(cmd_buffer[i]);
                Serial.print(" ");
            }
            Serial.println();
            Serial.println(cmd_status, BIN);
            Serial.println(cmd_status);
            status = COMMAND_SPI_FAILED;
    } 
    
    digitalWrite(_NSS, HIGH);
    SPI.endTransaction();
    delay(10);
    return status;
    
}

command_status_t SX1262::readCommand(uint8_t opcode, uint8_t *cmd_buffer, uint16_t cmd_len, uint8_t *read_buffer, uint8_t read_offset, bool single_val, bool get_status) {
    command_status_t status = COMMAND_SUCCESS;
    
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

    // while(digitalRead(_BUSY));
    if (isBusy()) {
        return COMMAND_BUSY_TIMEOUT;
    }

    digitalWrite(_NSS, LOW);
    SPI.transfer(opcode);
    uint8_t cmd_status = 0;
    for (uint8_t i = 0; i < cmd_len; i++) {
        if (i==0) {
            if (get_status) {
                read_buffer[i] = SPI.transfer(cmd_buffer[i]);
                cmd_status = read_buffer[i];
                continue;
            }
            cmd_status = SPI.transfer(cmd_buffer[i]); // Get Status (first byte)
        } else if (single_val){
            *read_buffer = SPI.transfer(cmd_buffer[i]);
        } else {
            if (i < read_offset) {
                SPI.transfer(cmd_buffer[i]);
            } else {
                read_buffer[i-read_offset] = SPI.transfer(cmd_buffer[i]);
            }
        }
    }
    
    // Check status
    if( ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_TIMEOUT) ||
        ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_ERROR) ||
        ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_FAILED) ||
        (cmd_status == 0x00) || (cmd_status == 0xFF)) {
        status = COMMAND_SPI_FAILED;
    } 

    digitalWrite(_NSS, HIGH);
    SPI.endTransaction();
    delay(10);
    return status;
}
/* 
    ============================= Operational Mode Functions ================================
*/
command_status_t SX1262::setSleep(uint8_t sleep_config) {
    return writeCommand(SX1262_SET_SLEEP, &sleep_config, 1);
}

command_status_t SX1262::setStandBy(uint8_t mode) {
    return writeCommand(SX1262_STANDBY, &mode, 1);
}

command_status_t SX1262::setFs() {
    return writeCommand(SX1262_SET_FS, 0, 0);
}

command_status_t SX1262::setTx(uint32_t timeout) {
    uint8_t cmd_buf[3];
    cmd_buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );   // bit 23 to 15
    cmd_buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );    // bit 15 to 8
    cmd_buf[2] = ( uint8_t )( timeout & 0xFF );             // bit 7 to 0
    return writeCommand(SX1262_SET_TX, cmd_buf, 3);
}

command_status_t SX1262::setRx(uint32_t timeout) {
    uint8_t cmd_buf[3];
    cmd_buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );   // bit 23 to 15
    cmd_buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );    // bit 15 to 8
    cmd_buf[2] = ( uint8_t )( timeout & 0xFF );             // bit 7 to 0
    return writeCommand(SX1262_SET_RX, cmd_buf, 3);
}

command_status_t SX1262::stopTimerOnPreamble(uint8_t enable) {
    return writeCommand(SX1262_STOP_TIMER_ON_PREAMBLE, &enable, 1);
}

command_status_t SX1262::setRxDutyCycle(uint32_t rx_period, uint32_t sleep_period) {
    uint8_t cmd_buf[6];
    cmd_buf[0] = ( uint8_t )( ( rx_period >> 16 ) & 0xFF );   // bit 23 to 15
    cmd_buf[1] = ( uint8_t )( ( rx_period >> 8 ) & 0xFF );    // bit 15 to 8
    cmd_buf[2] = ( uint8_t )( rx_period & 0xFF );             // bit 7 to 0
    cmd_buf[3] = ( uint8_t )( ( sleep_period >> 16 ) & 0xFF );   // bit 23 to 15
    cmd_buf[4] = ( uint8_t )( ( sleep_period >> 8 ) & 0xFF );    // bit 15 to 8
    cmd_buf[5] = ( uint8_t )( sleep_period & 0xFF );             // bit 7 to 0
    return writeCommand(SX1262_SET_RX_DUTY_CYCLE, cmd_buf, 6);
}

command_status_t SX1262::setCAD() {
    return writeCommand(SX1262_SET_CAD, 0, 0);
}

command_status_t SX1262::setRegulatorMode(uint8_t mode) {
    return writeCommand(SX1262_SET_REGULATOR_MODE, &mode, 1);
}

command_status_t SX1262::calibrateFunction(uint8_t calib_param) {
    return writeCommand(SX1262_CALIBRATE, &calib_param, 1);
}

command_status_t SX1262::calibrateImage(uint8_t freq1, uint8_t freq2) {
    uint8_t cmd_buf[2];
    cmd_buf[0] = freq1;
    cmd_buf[1] = freq2;
    return writeCommand(SX1262_CALIBRATE_IMAGE, cmd_buf, 2);
}

command_status_t SX1262::setPaConfig(uint8_t pa_duty_cycle, uint8_t hp_max, uint8_t device) {
    uint8_t cmd_buf[4];
    cmd_buf[0] = pa_duty_cycle;
    cmd_buf[1] = hp_max;
    cmd_buf[2] = device;
    cmd_buf[3] = 0x01;
    return writeCommand(SX1262_SET_PA_CONFIG, cmd_buf, 4);
}

command_status_t SX1262::setRxTxFallbackMode(uint8_t mode) {
    return writeCommand(SX1262_SET_RX_TX_FALLBACK_MODE, &mode, 1);
}

/* 
    ==================================== Registers and Buffer Access =======================================
*/
command_status_t SX1262::writeBuffer(uint8_t offset, uint8_t *tx_buf, uint8_t size) {
    if (size > 254) {
        return COMMAND_BUFFER_ERROR;
    }
    uint16_t buf_size = size+1;
    uint8_t cmd_buf[buf_size];
    cmd_buf[0] = offset; 
    for (uint8_t i = 0; i < size; i++) {
        cmd_buf[i+1] = tx_buf[i];
    }
    return writeCommand(SX1262_WRITE_BUFFER, cmd_buf, buf_size);
}

command_status_t SX1262::readBuffer(uint8_t offset, uint8_t size, uint8_t *received_buf, bool checkStatus) {
    uint8_t rx_status[2] = {0};
    if (checkStatus) {
        getRXBufferStatus(rx_status); // Update size and offset for new packet
        size = rx_status[0];
        offset = rx_status[1];
    }
    if (size > 254) {
        return COMMAND_BUFFER_ERROR;
    }

    uint16_t buf_size = size+2;
    uint8_t cmd_buf[buf_size] = {SX1262_CMD_NOP};
    cmd_buf[0] = offset;
    return readCommand(SX1262_READ_BUFFER, cmd_buf, buf_size, received_buf, 2);

}

/* 
    ==================================== DIO and IRQ Control Functions =======================================
*/
command_status_t SX1262::setDioIrqParams(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask) {
    uint8_t cmd_buf[8];
    cmd_buf[0] = ( uint8_t )( ( irq_mask >> 8 ) & 0xFF );       // bit 15 to 8 IRQ Mask
    cmd_buf[1] = ( uint8_t )( irq_mask & 0xFF );                // bit 7 to 0
    cmd_buf[2] = ( uint8_t )( ( dio1_mask >> 8 ) & 0xFF );      // bit 15 to 8 DIO1 Mask
    cmd_buf[3] = ( uint8_t )( dio1_mask & 0xFF );               // bit 7 to 0
    cmd_buf[4] = ( uint8_t )( ( dio2_mask >> 8 ) & 0xFF );      // bit 15 to 8 DIO2 Mask
    cmd_buf[5] = ( uint8_t )( dio2_mask & 0xFF );               // bit 7 to 0
    cmd_buf[6] = ( uint8_t )( ( dio3_mask >> 8 ) & 0xFF );      // bit 15 to 8 DIO3 Mask
    cmd_buf[7] = ( uint8_t )( dio3_mask & 0xFF );               // bit 7 to 0
    return writeCommand(SX1262_SET_DIO_IRQ_PARAMS, cmd_buf, 8);
}

command_status_t SX1262::getIrqStatus(uint16_t *irq_status) {
    uint8_t read_buf[2] = {SX1262_CMD_NOP};
    uint8_t cmd_buf[3] = {SX1262_CMD_NOP};
    command_status_t command_status = readCommand(SX1262_GET_IRQ_STATUS, cmd_buf, 3, read_buf, 1);
    *irq_status = read_buf[1] | (read_buf[0] << 8);  
    return command_status;
}

command_status_t SX1262::clearIrqStatus(uint16_t clear_param) {
    uint8_t cmd_buf[2];
    cmd_buf[0] = ( uint8_t )( ( clear_param >> 8 ) & 0xFF );       // bit 15 to 8 
    cmd_buf[1] = ( uint8_t )( clear_param & 0xFF );                // bit 7 to 0
    return writeCommand(SX1262_CLEAR_IRQ_STATUS, cmd_buf, 2);
}

command_status_t SX1262::setDIO2AsRfSwitchCtrl(uint8_t enable) {
    return writeCommand(SX1262_SET_DIO2_AS_RF_SWITCH_CTRL, &enable, 1);
}

command_status_t SX1262::setDIO3AsTCXOCtrl(uint8_t tcxo_voltage, uint32_t delay) {
    uint8_t cmd_buf[4];
    cmd_buf[0] = tcxo_voltage;
    cmd_buf[1] = ( uint8_t )( ( delay >> 16 ) & 0xFF );   // bit 23 to 15
    cmd_buf[2] = ( uint8_t )( ( delay >> 8 ) & 0xFF );    // bit 15 to 8
    cmd_buf[3] = ( uint8_t )( delay & 0xFF );             // bit 7 to 0

    return writeCommand(SX1262_SET_DIO3_AS_TCXO_CTRL, cmd_buf, 4);
}
/* 
    ====================== RF Modulation and Packet-Related Functions ===========================
*/
command_status_t SX1262::setRfFrequency(uint32_t rf_frequency) {
    // --------------- MAYBE CHANGE?? -----------------------
    // uint32_t  raw_freqeuncy = (uint32_t)(rf_frequency*(uint32_t(1) << 25))/32000000; 
    uint32_t  raw_freqeuncy = (uint32_t)((double)rf_frequency / (double)FREQ_STEP); // (rf_freqeuncy (Hz) * 2^25)/ FreqXTAL (32 MHz)
    // ------------------------------------------------
    uint8_t cmd_buf[4] = { (uint8_t)((raw_freqeuncy >> 24) & 0xFF), (uint8_t)((raw_freqeuncy >> 16) & 0xFF), (uint8_t)((raw_freqeuncy >> 8) & 0xFF), (uint8_t)(raw_freqeuncy & 0xFF)};
    return writeCommand(SX1262_SET_RF_FREQUENCY, cmd_buf, 4);
}

command_status_t SX1262::setPacketType(uint8_t type) {
    return writeCommand(SX1262_SET_PACKET_TYPE, &type, 1);
}

command_status_t SX1262::getPacketType(uint8_t *type) {
    uint8_t cmd_buf[2] = {SX1262_CMD_NOP};
    return readCommand(SX1262_GET_PACKET_TYPE, cmd_buf, 2, type, 1, true);
}

command_status_t SX1262::setTxParams(uint8_t power, uint8_t ramp_time) {
    uint8_t cmd_buf[2] = {power, ramp_time};
    return writeCommand(SX1262_SET_TX_PARAMS, cmd_buf, 2);
}

command_status_t SX1262::setLoRaModulationParams(uint8_t s_factor, uint8_t bandwidth, uint8_t coding_rate, uint8_t ldro) {
    uint8_t cmd_buf[4] = {s_factor, bandwidth, coding_rate, ldro};
    return writeCommand(SX1262_SET_MODULATION_PARAMS, cmd_buf, 4);
}

command_status_t SX1262::setLoRaPacketParams(uint16_t preamble_length, uint8_t header, uint8_t payload_length, uint8_t crc, uint8_t invert_iq) {
    uint8_t cmd_buf[6] = {0};
    cmd_buf[0] = ( uint8_t )( ( preamble_length >> 8 ) & 0xFF );    // bit 15 to 8
    cmd_buf[1] = ( uint8_t )( preamble_length & 0xFF );             // bit 7 to 0
    cmd_buf[2] = header;
    cmd_buf[3] = payload_length;
    cmd_buf[4] = crc;
    cmd_buf[5] = invert_iq;
    
    return writeCommand(SX1262_SET_PACKET_PARAMS, cmd_buf, 6);
}

command_status_t SX1262::setBufferBaseAddress(uint8_t tx_address, uint8_t rx_address) {
    uint8_t cmd_buf[2] = {tx_address, rx_address};
    return writeCommand(SX1262_SET_BUFFER_BASE_ADDRESS, cmd_buf, 2);
}

command_status_t SX1262::setLoRaSymbNumTimeout(uint8_t value) {
    return writeCommand(SX1262_SET_LORA_SYMB_NUM_TIMEOUT, &value, 1);
}
/* 
    ============================= Communication Status Information ================================
*/
command_status_t SX1262::getStatus(uint8_t *status) {
    uint8_t cmd_buf[1] = {SX1262_CMD_NOP};
    return readCommand(SX1262_GET_STATUS, cmd_buf, 1, status,1,true,true);
}

command_status_t SX1262::getRXBufferStatus(uint8_t *rx_status) {
    uint8_t cmd_buf[2] = {SX1262_CMD_NOP};
    return readCommand(SX1262_GET_RX_BUFFER_STATUS, cmd_buf, 2, rx_status);
}

command_status_t SX1262::getPacketStatus(uint8_t *pkt_status) {
    uint8_t cmd_buf[4] = {SX1262_CMD_NOP};
    return readCommand(SX1262_GET_PACKET_STATUS, cmd_buf, 4, pkt_status);
}

command_status_t SX1262::getRssiInst(uint8_t *rssi) {
    uint8_t cmd_buf[2] = {SX1262_CMD_NOP};
    return readCommand(SX1262_GET_RSSI_INST, cmd_buf, 2, rssi,1,true);
}

command_status_t SX1262::getStats(uint8_t *stats) {
    uint8_t cmd_buf[7] = {SX1262_CMD_NOP};
    return readCommand(SX1262_GET_STATS, cmd_buf, 7, stats);
}

command_status_t SX1262::resetStats() {
    uint8_t cmd_buf[6] = {SX1262_CMD_NOP};
    return writeCommand(SX1262_RESET_STATS, cmd_buf, 6);
}

/* 
    ======================================== Miscellaneous ==============================================
*/

command_status_t SX1262::getDeviceErrors(uint16_t *device_errors) { // TODO Check THIS
    uint8_t read_buf[2] = {0};
    uint8_t cmd_buf[3] = {SX1262_CMD_NOP};
    command_status_t command_status = readCommand(SX1262_GET_DEVICE_ERRORS, cmd_buf, 3, read_buf);
    *device_errors = read_buf[1] | (read_buf[0] << 8);  
    return command_status;
}

command_status_t SX1262::clearDeviceErrors() {
    uint8_t cmd_buf[2] = {SX1262_CMD_NOP};
    return writeCommand(SX1262_CLEAR_DEVICE_ERRORS, cmd_buf, 2);

}

bool SX1262::isBusy() {
    uint16_t busy_timeout = 0;
    while(digitalRead(_BUSY)) {
        delay(5);
        busy_timeout++;

        if (busy_timeout > 400) {
            Serial.println(F("ERROR - BUSY TIMEOUT!"));
            return true;
        }
    }
    return false;
}