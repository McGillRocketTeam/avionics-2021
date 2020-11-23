/*
    sx1262.h - MRT Semtech SX1262 Library
    Created by Jake Peters

    Things to Finish:
        Finish enums for different values/definitions

    Something about copyright....

*/

#ifndef SX1262_h
#define SX1262_h

#include "Arduino.h"

/*
================= Public Defines =====================
*/
#define SX1262_CMD_NOP                              0x00

// ================================================== OPCODES =====================================

// Set Operational Modes
#define SX1262_SET_SLEEP                            0x84
#define SX1262_STANDBY                              0x80
#define SX1262_SET_FS                               0xC1
#define SX1262_SET_TX                               0x83
#define SX1262_SET_RX                               0x82
#define SX1262_STOP_TIMER_ON_PREAMBLE               0x9F
#define SX1262_SET_RX_DUTY_CYCLE                    0x94
#define SX1262_SET_CAD                              0xC5
#define SX1262_SET_TX_CONTINUOUS_WAVE               0xD1
#define SX1262_SET_TX_INFINITE_PREAMBLE             0xD2
#define SX1262_SET_REGULATOR_MODE                   0x96
#define SX1262_CALIBRATE                            0x89
#define SX1262_CALIBRATE_IMAGE                      0x98
#define SX1262_SET_PA_CONFIG                        0x95
#define SX1262_SET_RX_TX_FALLBACK_MODE              0x93

// register and buffer access commands
#define SX1262_WRITE_REGISTER                       0x0D
#define SX1262_READ_REGISTER                        0x1D
#define SX1262_WRITE_BUFFER                         0x0E
#define SX1262_READ_BUFFER                          0x1E

//SX1262_CMD_GET_DEVICE_ERRORS
#define SX1262_PA_RAMP_ERR                           0b100000000  //  8     8     device errors: PA ramping failed
#define SX1262_PLL_LOCK_ERR                          0b001000000  //  6     6                    PLL failed to lock
#define SX1262_XOSC_START_ERR                        0b000100000  //  5     5                    crystal oscillator failed to start
#define SX1262_IMG_CALIB_ERR                         0b000010000  //  4     4                    image calibration failed
#define SX1262_ADC_CALIB_ERR                         0b000001000  //  3     3                    ADC calibration failed
#define SX1262_PLL_CALIB_ERR                         0b000000100  //  2     2                    PLL calibration failed
#define SX1262_RC13M_CALIB_ERR                       0b000000010  //  1     1                    RC13M calibration failed
#define SX1262_RC64K_CALIB_ERR                       0b000000001  //  0     0                    RC64K calibration failed

/* 
    ====================== RF Modulation and Packet-Related Functions ===========================
*/

// RF modulation and packet commands
#define SX1262_SET_RF_FREQUENCY                   0x86
#define SX1262_SET_PACKET_TYPE                    0x8A
#define SX1262_GET_PACKET_TYPE                    0x11
#define SX1262_SET_TX_PARAMS                      0x8E
#define SX1262_SET_MODULATION_PARAMS              0x8B
#define SX1262_SET_PACKET_PARAMS                  0x8C
#define SX1262_SET_CAD_PARAMS                     0x88
#define SX1262_SET_BUFFER_BASE_ADDRESS            0x8F
#define SX1262_SET_LORA_SYMB_NUM_TIMEOUT          0x0A

// SetTxParams RampTime
#define SX1262_PA_RAMP_10U                            0x00        //  7     0     ramp time: 10 us
#define SX1262_PA_RAMP_20U                            0x01        //  7     0                20 us
#define SX1262_PA_RAMP_40U                            0x02        //  7     0                40 us
#define SX1262_PA_RAMP_80U                            0x03        //  7     0                80 us
#define SX1262_PA_RAMP_200U                           0x04        //  7     0                200 us
#define SX1262_PA_RAMP_800U                           0x05        //  7     0                800 us
#define SX1262_PA_RAMP_1700U                          0x06        //  7     0                1700 us
#define SX1262_PA_RAMP_3400U                          0x07        //  7     0                3400 us


//SX1262_CMD_GET_STATUS
#define SX1262_STATUS_MODE_STDBY_RC                 0b00100000  //  6     4     current chip mode: STDBY_RC
#define SX1262_STATUS_MODE_STDBY_XOSC               0b00110000  //  6     4                        STDBY_XOSC
#define SX1262_STATUS_MODE_FS                       0b01000000  //  6     4                        FS
#define SX1262_STATUS_MODE_RX                       0b01010000  //  6     4                        RX
#define SX1262_STATUS_MODE_TX                       0b01100000  //  6     4                        TX
#define SX1262_STATUS_DATA_AVAILABLE                0b00000100  //  3     1     command status: packet received and data can be retrieved
#define SX1262_STATUS_CMD_TIMEOUT                   0b00000110  //  3     1                     SPI command timed out
#define SX1262_STATUS_CMD_ERROR                     0b00001000  //  3     1                     invalid SPI command
#define SX1262_STATUS_CMD_FAILED                    0b00001010  //  3     1                     SPI command failed to execute
#define SX1262_STATUS_TX_DONE                       0b00001100  //  3     1                     packet transmission done
#define SX1262_STATUS_SPI_FAILED                    0b11111111  //  7     0     SPI transaction failed

// status commands
#define SX1262_GET_STATUS                         0xC0
#define SX1262_GET_RSSI_INST                      0x15
#define SX1262_GET_RX_BUFFER_STATUS               0x13
#define SX1262_GET_PACKET_STATUS                  0x14
#define SX1262_GET_DEVICE_ERRORS                  0x17
#define SX1262_CLEAR_DEVICE_ERRORS                0x07
#define SX1262_GET_STATS                          0x10
#define SX1262_RESET_STATS                        0x00

// DIO and IRQ control
#define SX1262_CMD_SET_DIO_IRQ_PARAMS                 0x08
#define SX1262_CMD_GET_IRQ_STATUS                     0x12
#define SX1262_CMD_CLEAR_IRQ_STATUS                   0x02
#define SX1262_CMD_SET_DIO2_AS_RF_SWITCH_CTRL         0x9D
#define SX1262_CMD_SET_DIO3_AS_TCXO_CTRL              0x97

//SX1262_CMD_SET_PACKET_TYPE
#define SX1262_PACKET_TYPE_GFSK                       0x00        //  7     0     packet type: GFSK
#define SX1262_PACKET_TYPE_LORA                       0x01        //  7     0                  LoRa

/*
================= Public Types ====================
*/
// Read/Write Comand status
typedef enum {
    COMMAND_SUCCESS = 0,
    COMMAND_BUSY_TIMEOUT = 1,
    COMMAND_SPI_FAILED = 2,
    COMMAND_ERROR_FLAG = 3,
    COMMAND_BUFFER_ERROR = 4,
} command_status_t;

// SX1262 TCXO control voltages
typedef enum
{
    TCXO_CTRL_1_6V = 0x00,
    TCXO_CTRL_1_7V = 0x01,
    TCXO_CTRL_1_8V = 0x02,
    TCXO_CTRL_2_2V = 0x03,
    TCXO_CTRL_2_4V = 0x04,
    TCXO_CTRL_2_7V = 0x05,
    TCXO_CTRL_3_0V = 0x06,
    TCXO_CTRL_3_3V = 0x07,
} tcxo_ctrl_voltages_t;

// SX1262 SetTxParam Ramp time
typedef enum
{
    SET_RAMP_10U = 0x00,        // 10 us
    SET_RAMP_20U = 0x01,        // 20 us
    SET_RAMP_40U = 0x02,        // 40 us
    SET_RAMP_80U = 0x03,        // 80 us
    SET_RAMP_200U = 0x04,       // 200 us
    SET_RAMP_800U = 0x05,       // 800 us
    SET_RAMP_1700U = 0x06,      // 1700 us
    SET_RAMP_3400U = 0x07,      // 3400 us
} ramp_time_t;

// SX1262 LoRa spreading factor 
typedef enum
{
    LORA_SF5  = 0x05,
    LORA_SF6  = 0x06,
    LORA_SF7  = 0x07,
    LORA_SF8  = 0x08,
    LORA_SF9  = 0x09,
    LORA_SF10 = 0x0A,
    LORA_SF11 = 0x0B,
    LORA_SF12 = 0x0C,
} lora_sf_t;

// SX1262 LoRa bandwidths
typedef enum 
{
    LORA_BW_500 = 0x06,
    LORA_BW_250 = 0x05,
    LORA_BW_125 = 0x04,
    LORA_BW_062 = 0x03,
    LORA_BW_041 = 0x0A,
    LORA_BW_031 = 0x02,
    LORA_BW_020 = 0x09,
    LORA_BW_015 = 0x01,
    LORA_BW_010 = 0x08,
    LORA_BW_007 = 0x00,
} lora_bw_t;

// SX1262 LoRa coding rate
typedef enum 
{
    LORA_CR_4_5 = 0x01,
    LORA_CR_4_6 = 0x02,
    LORA_CR_4_7 = 0x03,
    LORA_CR_4_8 = 0x04,
} lora_cr_t;

typedef enum
{
    MINUS_17_dBm = 0xEF,
    MINUS_9_dBm  = 0xF7,
    PLUS_14_dBm  = 0x0E,
    PLUS_22_dBm  = 0x16,
} output_tx_power_t;

/*
Class for sx1262
*/
class SX1262 {
    public:
        /* 
            ============================= Setup ================================
        */
        SX1262(void);
        /*
            @brief Setup SX1262 Class
        */
        void begin(uint8_t _NSS, uint8_t _BUSY, uint8_t _RESET, uint8_t _DIO1, uint8_t _ANT_SW);

        /* 
            ============================= Passing Instructions ================================
        */
        /*
            @brief Send command with no response
        */
        command_status_t writeCommand(uint8_t opcode, uint8_t *cmd_buffer, uint16_t cmd_len);
        /*
            @brief Send command and store response
        */
        command_status_t readCommand(uint8_t opcode, uint8_t *cmd_buffer, uint16_t cmd_len, uint8_t *read_buffer,  uint8_t read_offset=1);

        /* 
            ============================= Operational Mode Functions ================================
        */

        /*
            @brief Set device to SLEEP mode. Can only be sent while in STDBY mode.

            @param sleep_config Sleep mode configuration:
                [7:3]   -   Reserved
                [2]     -   0: cold start/1: warm start
                [1]     -   0: RFU
                [0]     -   0: RTC timeout disabled/1: wake-up on RTC timeout

        */
        command_status_t setSleep(uint8_t sleep_config);

        /*
            @brief Set device mode to Standby_RC.

            @param mode Standby mode configuration, 0 for STDBY_RC and 1 for STDBY_XOSC.
        */
        command_status_t setStandBy(uint8_t mode);

        /*
            @brief Set device in freqeuncy synthesis mode.
        */
        command_status_t setFs();

        /*
            @brief Set device in transmit mode.

            @param timeout 23-bit timeout before returning back to STDBY_RC if the TX_DONE IRQ is not generated.
                Timeout duration = timeout[23:0] * 15.625us
        */
        command_status_t setTx(uint32_t timeout);
        
        /*
            @brief Set device in receiver mode

            @param timeout 23-bit timeout before returning back to STDBY_RC if Sync word or Header not detected. 0xFFFFFF is Rx Continuous mode.
                Timeout duration = timeout[23:0] * 15.625us
                
        */
        command_status_t setRx(uint32_t timeout);

        /*
            @brief Set device to listen for new packets switching between RX and SLEEP modes.

            @param rx_period Period to listen for packets for in RX mode.
                Rx duration = rx_period[23:0] * 15.625us

            @param sleep_period Period to sleep for if no packet is received.
                Sleep duration = sleep_period[23:0] * 15.625us
        */
        command_status_t setRxDutyCycle(uint32_t rx_period, uint32_t sleep_period);

        /*
            @brief Set device in Channel Activity Detection mode for LoRa packets.
        */
        command_status_t setCAD();

        /*
            @brief Set device regulator mode.

            @param mode 0 for LDO only and 1 for LDO+DC-DC.
        */
        command_status_t setRegulatorMode(uint8_t mode);
        
        /*
            @brief Calibrate several blocks of the device.

            @param calib_param 
                        Bit 0:  RC64K
                        Bit 1:  RC13M
                        Bit 2:  PLL
                        Bit 3:  ADC
                        Bit 4:  ADC bulk N
                        Bit 5:  ADC bulk P
                        Bit 6:  Image Calibration
                        Bit 7:  0 (RFU)
        */
        command_status_t calibrateFunction(uint8_t calib_param);

        /*
            @brief Calibrate image rejection for operating freqeuncy band.

            @param freq1 Lower frequency.

            @param freq2 Upper frequency.
        */
        command_status_t calibrateImage(uint8_t freq1, uint8_t freq2);

        /*
            @brief PA config to use and to differentiate between SX1262 and SX1261.

            @param pa_duty_cycle Controls duty cycle of PA.

            @param hp_max Size of PA in the SX1262.

            @param device 0 for the SX1262 and 1 for the SX1261.
        */
        command_status_t setPaConfig(uint8_t pa_duty_cycle, uint8_t hp_max, uint8_t device);


        /*
            @brief Defines which mode the chip goes to after successful transmission or packet reception.

            @param mode Desired mode to return to. 
                0x40:   FS
                0x30:   STDBY_XOSC
                0x20:   STDBY_RC 
        */
        command_status_t setRxTxFallbackMode(uint8_t mode);

        /* 
            ==================================== Registers and Buffer Access =======================================
        */

        /*
            @brief Store data payload to be transmitted. Payload greater than 255 bytes will wrap around back to 0.

            @param Data to be transmitted.
        */
        command_status_t writeBuffer(uint8_t *tx_buf, uint8_t size);

        /*
            @brief Read bytes of received payload.

            @param Size of buffer in bytes to read. 
        */
        command_status_t readBuffer(uint8_t size);

        /* 
            ====================================== DIO and IRQ Control Functions ==========================================
        */
        /*
            @brief Set DIO3 to control voltage for TCXO
        */
        command_status_t setDIO2AsRfSwitchCtrl(uint8_t enable);
        /*
            @brief Set DIO3 to control voltage for TCXO

            @param delay Time needed for the TCXO to appear and stabilize (Timeout duration = delay(23:0)*15.625us)  
        */
        command_status_t setDIO3AsTCXOCtrl(tcxo_ctrl_voltages_t tcxo_voltage, uint32_t delay=350);


        /* 
            ================================== RF Modulation and Packet-Related Functions ====================================
        */
        /*
            @brief Set freqeuncy of the device

            @param rf_frequency Desired freqeuncy in Hz, ex. 450000000 (450 MHz)
        */
        command_status_t setRfFrequency(uint32_t rf_freqeuncy);

        /*
            @brief Set the current operating packet type

            @param type 0x00: Set to GFSK packet type or 0x01: Set to LoRa packet type
        */
        command_status_t setPacketType(uint8_t type);

        /*
            @brief Get the current operating packet type
        */
        uint8_t getPacketType();

        /*
            @brief Set TX output power and TX ramping time

            @param power Output power defined in dBm. -17 dBm to +14 dBm for low power PA and -9 dBm to +22 dBm for high power PA.

            @param ramp_time TX power ramp time.
        */
        command_status_t setTxParams(output_tx_power_t power, ramp_time_t ramp_time);

        /*
            @brief Configure the modulation parameters for LoRa modulation of the radio.

            @param s_factor Spreading factor.

            @param bandwidth Bandwidth.
            
            @param coding_rate Coding Rate.

            @param ldro Low Data Rate Optimization, 0 = off, 1 = on
        */
        command_status_t setLoRaModulationParams(lora_sf_t s_factor, lora_bw_t bandwidth, lora_cr_t coding_rate, uint8_t ldro);

        /*
            @brief Configure the packet parameters for LoRa modulation of the radio.

            @param preamble_length Set preamble length.

            @param header 0 = variable length packet, 1 = fixed length packet.

            @param payload_length Size of payload in bytes.

            @param crc 0 = CRC off, 1 = CRC on.

            @param invert_iq 0 = standard, 1 = inverted.
        */
        command_status_t setLoRaPacketParams(uint16_t preamble_length, uint8_t header, uint8_t payload_length, uint8_t crc, uint8_t invert_iq);
        /* 
            ============================= Communication Status Information ================================
        */
        /*
            @brief Get status of device
        */
        uint8_t getStatus();

        /* 
            ========================================== Miscellaneous ==========================================
        */

        /*
            @brief Get device error flags
        */
        uint32_t getDeviceErrors();

        /*
            @brief Clear device error flags
        */
        uint8_t clearDeviceErrors();


        bool isBusy();


    private:
        uint8_t _NSS, _BUSY, _RESET, _DIO1, _ANT_SW;

};










#endif // SX1262_h