/*
    sx1262.cpp - MRT Semtech SX1262 Library
    Created by Jake Peters

    Things to Finish:
        

    Something about copyright....

*/

#include <sx1262.h>
#include <SPI.h>

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
}

/* 
    ============================= Passing Instructions ================================
*/

command_status_t SX1262::writeCommand(uint8_t opcode, uint8_t *cmd_buffer, uint16_t cmd_len) {
    command_status_t status = COMMAND_SUCCESS;
    if (!isBusy()) {
        digitalWrite(_NSS, LOW);
        SPI.transfer(opcode);
        uint8_t cmd_status; 
        for (uint8_t i = 0; i < cmd_len; i++) {
            cmd_status = SPI.transfer(cmd_buffer[i]);
        }
        if( ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_TIMEOUT) ||
            ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_ERROR) ||
            ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_FAILED) ||
            (cmd_status == 0x00) || (cmd_status == 0xFF)) {
            status = COMMAND_SPI_FAILED;
        } 
        digitalWrite(_NSS, HIGH);
        return status;
    }
    status = COMMAND_BUSY_TIMEOUT;
    return status;
}

command_status_t SX1262::readCommand(uint8_t opcode, uint8_t *cmd_buffer, uint16_t cmd_len, uint8_t *read_buffer, uint8_t read_offset) {
    command_status_t status = COMMAND_SUCCESS;
    if (!isBusy()) {
        digitalWrite(_NSS, LOW);
        SPI.transfer(opcode);
        uint8_t cmd_status
        for (uint8_t i = 0; i < cmd_len; i++) {
            if (i >= read_offset) {
                read_buffer[i] = SPI.transfer(cmd_buffer[i]);
            } else if (i=0) {
                cmd_status = SPI.transfer(cmd_buffer[i]); // Get Status (first byte)
            } else {
                SPI.transfer(cmd_buffer[i]);
            }
        }

        // Get Status (first byte)
        

        // Check status
        if( ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_TIMEOUT) ||
            ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_ERROR) ||
            ((cmd_status & 0b00001110) == SX1262_STATUS_CMD_FAILED) ||
            (cmd_status == 0x00) || (cmd_status == 0xFF)) {
            status = COMMAND_SPI_FAILED;
        } 
        digitalWrite(_NSS, HIGH);
        return status;
    } 
    status = COMMAND_BUSY_TIMEOUT;
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
command_status_t SX1262::writeBuffer(uint8_t *tx_buf, uint8_t size) {
    if (size > 254) {
        command_status_t command_status = COMMAND_BUFFER_ERROR;
        return command_status;
    }
    uint16_t buf_size = size+1;
    uint8_t cmd_buf[buf_size];
    cmd_buf[0] = SX1262_CMD_NOP; // No offset
    for (uint8_t i = 0; i < size; i++) {
        cmd_buf[i+1] = tx_buf[i];
    }
    return writeCommand(SX1262_WRITE_BUFFER, cmd_buf, buf_size);
}

command_status_t SX1262::readBuffer(uint8_t size, uint8_t *recieved_buf) {
    if (size > 254) {
        command_status_t command_status = COMMAND_BUFFER_ERROR;
        return command_status;
    }
    uint16_t buf_size = size+2;
    uint8_t cmd_buf[buf_size] = {SX1262_CMD_NOP};
    return readCommand(SX1262_READ_BUFFER, cmd_buf, buf_size, recieved_buf, 2);

}

/* 
    ==================================== DIO and IRQ Control Functions =======================================
*/
command_status_t SX1262::setDIO2AsRfSwitchCtrl(uint8_t enable) {
    return writeCommand(SX1262_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &enable, 1);
}

command_status_t SX1262::setDIO3AsTCXOCtrl(tcxo_ctrl_voltages_t tcxo_voltage, uint32_t delay) {
    // Add getError()
    /*
if(getDeviceErrors() & SX126X_XOSC_START_ERR) {
    clearDeviceErrors();
  }
    */

    uint8_t cmd_buf[4];

    cmd_buf[0] = tcxo_voltage;
    cmd_buf[1] = ( uint8_t )( ( delay >> 16 ) & 0xFF );   // bit 23 to 15
    cmd_buf[2] = ( uint8_t )( ( delay >> 8 ) & 0xFF );    // bit 15 to 8
    cmd_buf[3] = ( uint8_t )( delay & 0xFF );             // bit 7 to 0

    // CHeck Error Status
    return writeCommand(SX1262_CMD_SET_DIO3_AS_TCXO_CTRL, cmd_buf, 4);
}
/* 
    ====================== RF Modulation and Packet-Related Functions ===========================
*/
command_status_t SX1262::setRfFrequency(uint32_t rf_frequency) {
    uint32_t  raw_freqeuncy = (rf_frequency*(uint32_t(1) << 25))/32000000; // (rf_freqeuncy (Hz) * 2^25)/ FreqXTAL (32 MHz)
    uint8_t cmd_buf[4] = { (uint8_t)((raw_freqeuncy >> 24) & 0xFF), (uint8_t)((raw_freqeuncy >> 16) & 0xFF), (uint8_t)((raw_freqeuncy >> 8) & 0xFF), (uint8_t)(raw_freqeuncy & 0xFF)};
    return writeCommand(SX1262_SET_RF_FREQUENCY, cmd_buf, 4);
}

command_status_t SX1262::setPacketType(uint8_t type) {
    return writeCommand(SX1262_SET_PACKET_TYPE, &type, 1);
}

uint8_t SX1262::getPacketType() {
    uint8_t type = 255;
    uint8_t cmd_buf[2] = {SX1262_CMD_NOP};
    command_status_t command_status = readCommand(SX1262_GET_PACKET_TYPE, cmd_buf, 2, &type);
    return type;
}

command_status_t SX1262::setTxParams(output_tx_power_t power, ramp_time_t ramp_time) {
    uint8_t cmd_buf[2] = {power, ramp_time};
    return writeCommand(SX1262_SET_TX_PARAMS, cmd_buf, 2);
}

command_status_t SX1262::setLoRaModulationParams(lora_sf_t s_factor, lora_bw_t bandwidth, lora_cr_t coding_rate, uint8_t ldro) {
    uint8_t cmd_buf[4] = {s_factor, bandwidth, coding_rate, ldro};
    return writeCommand(SX1262_SET_MODULATION_PARAMS, cmd_buf, 4);
}

command_status_t SX1262::setLoRaPacketParams(uint16_t preamble_length, uint8_t header, uint8_t payload_length, uint8_t crc, uint8_t invert_iq) {
    uint8_t cmd_buf[6];
    cmd_buf[0] = ( uint8_t )( ( preamble_length >> 8 ) & 0xFF );    // bit 15 to 8
    cmd_buf[1] = ( uint8_t )( preamble_length & 0xFF );             // bit 7 to 0
    cmd_buf[2] = header;
    cmd_buf[3] = payload_length;
    cmd_buf[4] = crc;
    cmd_buf[5] = invert_iq;
    return writeCommand(SX1262_SET_PACKET_PARAMS, cmd_buf, 6);
}
/* 
    ============================= Communication Status Information ================================
*/
uint8_t SX1262::getStatus() {
    uint8_t cmd_buf[1] = {SX1262_CMD_NOP};
    uint8_t status = 0;
    command_status_t command_status = readCommand(SX1262_GET_STATUS, cmd_buf, 1, &status);
    #ifdef DEBUG
        Serial.print(command_status);
    #endif
    return status;
}


/* 
    ======================================== Miscellaneous ==============================================
*/

uint32_t SX1262::getDeviceErrors() {
    uint8_t read_buf[2] = {0};
    uint8_t cmd_buf[3] = {SX1262_CMD_NOP};
    readCommand(SX1262_GET_DEVICE_ERRORS, cmd_buf, 3, read_buf);

    uint32_t status = (read_buf[0] << 8) | (read_buf[1]);

    return status;
}

uint8_t SX1262::clearDeviceErrors() {
    uint8_t cmd_buf[2] = {SX1262_CMD_NOP};
    uint8_t read_buf[1] = {0};
    command_status_t status = readCommand(SX1262_CLEAR_DEVICE_ERRORS, cmd_buf, 2, read_buf);
    #ifdef DEBUG
        Serial.print(status);
    #endif
    return read_buf[0];
}

bool SX1262::isBusy() {
    uint8_t busy_timeout = 0;
    while(digitalRead(_BUSY)) {
        delay(1);
        busy_timeout++;

        if (busy_timeout > 10) {
            Serial.println(F("ERROR - BUSY TIMEOUT!"));
            return true;
        }
    }
    return false;
}