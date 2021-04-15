#include "sx126x_hal.h"
#include "main.h"
/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to the opcode be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to the data be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_write( const void* hspi, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length ){
	HAL_StatusTypeDef status;
	while(HAL_GPIO_ReadPin(BUSY_GPIO_Port,BUSY_Pin) == GPIO_PIN_SET);
			HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
			status = HAL_SPI_Transmit(hspi, command, command_length, 100);
			status = HAL_SPI_Transmit(hspi, data, data_length, 100);
			HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
		return status;
}

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to the opcode be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer that receives the data
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read( const void* hspi, const uint8_t* command, const uint16_t command_lenght,
                                     uint8_t* data, const uint16_t data_length ){
	HAL_StatusTypeDef status;
	while(HAL_GPIO_ReadPin(BUSY_GPIO_Port,BUSY_Pin) == GPIO_PIN_SET);
			HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
			status = HAL_SPI_Transmit(hspi, command, command_lenght, 100);
			status = HAL_SPI_Receive(hspi, data, data_length, 100);
			HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
		return status;
}

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_reset( const void* hspi ){

	if(HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_WritePin(NRESET_GPIO_Port, NRESET_Pin, GPIO_PIN_RESET);
		HAL_Delay(5);
		HAL_GPIO_WritePin(NRESET_GPIO_Port, NRESET_Pin, GPIO_PIN_SET);
	}
}

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wakeup( const void* hspi ){
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
}

