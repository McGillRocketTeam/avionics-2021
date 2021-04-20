/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sx126x_hal.h"
#include "sx126x.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void transmitBuffer(char buffer[]){
	uint8_t length = 0;
	while(buffer[length] != 0){
		length++;
	}
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, length, 100);
}

void transmitStatus(HAL_StatusTypeDef status){
	if(status == HAL_OK){
		HAL_UART_Transmit(&huart3, (uint8_t*)"Status: HAL_OK\n", 15, 100);
	} else if(status == HAL_ERROR){
		HAL_UART_Transmit(&huart3, (uint8_t*)"Status: HAL_ERROR\n", 18, 100);
	} else if(status == HAL_BUSY){
		HAL_UART_Transmit(&huart3, (uint8_t*)"Status: HAL_BUSY\n", 17, 100);
	} else if(status == HAL_TIMEOUT){
		HAL_UART_Transmit(&huart3, (uint8_t*)"Status: HAL_TIMEOUT\n", 20, 100);
	} else {
		HAL_UART_Transmit(&huart3, (uint8_t*)"Status: Unknown status Received\n", 32, 100);
	}
}

void transmitIRQ(sx126x_irq_mask_t irq){
	if(irq == SX126X_IRQ_TX_DONE){
		transmitBuffer("Tx Done\n");
	} else {
		transmitBuffer("Big Sad\n");
	}
}

HAL_StatusTypeDef writeCommand(uint8_t opcode, uint8_t params[], uint16_t numOfParams){
	HAL_StatusTypeDef status;
	while(HAL_GPIO_ReadPin(BUSY_1_GPIO_Port, BUSY_1_Pin) == GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_1_GPIO_Port, NSS_1_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi1, &opcode, 1, 100);
	status = HAL_SPI_Transmit(&hspi1, (uint8_t*)params, numOfParams, 100);
	HAL_GPIO_WritePin(NSS_1_GPIO_Port, NSS_1_Pin, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef readCommand(uint8_t opcode, uint8_t params[], uint8_t response[], uint16_t numOfParams){
	HAL_StatusTypeDef status;
	while(HAL_GPIO_ReadPin(BUSY_1_GPIO_Port, BUSY_1_Pin) == GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_1_GPIO_Port, NSS_1_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi1, &opcode, 1, 100);
	status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)params, (uint8_t*)response, numOfParams, 100);
	HAL_GPIO_WritePin(NSS_1_GPIO_Port, NSS_1_Pin, GPIO_PIN_SET);
	return status;
}

void Tx_setup(){

	HAL_GPIO_WritePin(NRESET_1_GPIO_Port, NRESET_1_Pin, GPIO_PIN_SET); //Make sure reset is off
	HAL_GPIO_WritePin(NRESET_2_GPIO_Port, NRESET_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_1_GPIO_Port, NSS_1_Pin, GPIO_PIN_SET); //Make sure chip select is off
	HAL_GPIO_WritePin(NSS_2_GPIO_Port, NSS_2_Pin, GPIO_PIN_SET);

	//set to standby
	sx126x_set_standby(&hspi1, 0);

	//set packet type to LORA
	sx126x_set_pkt_type(&hspi1, 1);

	//set frequency
	sx126x_set_rf_freq(&hspi1, 448000000);

	//set pa config
	struct sx126x_pa_cfg_params_s *params = malloc(sizeof(sx126x_pa_cfg_params_t));
	params->pa_duty_cycle=2;
	params->hp_max=3;
	params->device_sel=0;
	params->pa_lut=1;
	sx126x_set_pa_cfg(&hspi1, params);
	free(params);

	//set tx params
	int8_t power = 14;
	sx126x_set_tx_params(&hspi1, power, SX126X_RAMP_200_US);

	//set buffer base address
	sx126x_set_buffer_base_address(&hspi1, 0, 0);

	//set modulation parameters
	struct sx126x_mod_params_lora_s *mod_params = malloc(sizeof(sx126x_mod_params_lora_t));
	mod_params->sf=9;
	mod_params->bw=3;
	mod_params->cr=1;
	mod_params->ldro=0;
	sx126x_set_lora_mod_params(&hspi1, mod_params);
	free(mod_params);

	//set dio and irq parameters
	sx126x_set_dio_irq_params(&hspi1,1023,0b1000000001,0,0);

	//sx126x_set_standby(&hspi1, 0);

	//sx126x_set_pkt_type(&hspi1, 1);

	sx126x_set_rx_tx_fallback_mode(&hspi1, 0x20);

	sx126x_set_dio2_as_rf_sw_ctrl(&hspi1, 1);

	sx126x_set_dio3_as_tcxo_ctrl(&hspi1, 0x06, 100);

	sx126x_cal(&hspi1, SX126X_CAL_ALL);

	HAL_Delay(50);

	//sx126x_set_standby(&hspi1, 0);

	sx126x_set_reg_mode(&hspi1, 0x01);

	//set image calibration
	uint8_t opcode = 0x98;
	uint8_t params1[4] = {0x6F,0x75};
	writeCommand(opcode, params1, 2);

	//sx126x_set_rf_freq(&hspi1, 448000000);

	/*
	//set pa config
	struct sx126x_pa_cfg_params_s *params2 = malloc(sizeof(sx126x_pa_cfg_params_t));
	params2->pa_duty_cycle=2;
	params2->hp_max=3;
	params2->device_sel=0;
	params2->pa_lut=1;
	sx126x_set_pa_cfg(&hspi1, params2);
	free(params2);
	*/

	//sx126x_set_tx_params(&hspi1, 14, SX126X_RAMP_200_US);

	//sx126x_set_buffer_base_address(&hspi1, 0, 0);

	/*
	//modulation parameters
	struct sx126x_mod_params_lora_s *mod_params0 = malloc(sizeof(sx126x_mod_params_lora_t));
	mod_params0->sf=9;
	mod_params0->bw=3;
	mod_params0->cr=1;
	mod_params0->ldro=0;
	sx126x_set_lora_mod_params(&hspi1, mod_params0);
	free(mod_params0);
	*/

	//packet params
	struct sx126x_pkt_params_lora_s *lora_params = malloc(sizeof(sx126x_pkt_params_lora_t));
	lora_params->preamble_len_in_symb=12;
	lora_params->header_type=0;
	lora_params->pld_len_in_bytes=0xFF;
	lora_params->crc_is_on=1;
	lora_params->invert_iq_is_on=0;
	sx126x_set_lora_pkt_params(&hspi1, lora_params);
	free(lora_params);

	//sx126x_set_dio_irq_params(&hspi1,1023,0b1000000001,0,0);

}

void TxProtocol(uint8_t data[], uint8_t data_length){

	HAL_StatusTypeDef command_status;
	sx126x_irq_mask_t irq = SX126X_IRQ_ALL;
	command_status = sx126x_clear_irq_status(&hspi1, irq);

	command_status = sx126x_write_buffer(&hspi1, 0, data, data_length);

	//packet params
	struct sx126x_pkt_params_lora_s *lora_params = malloc(sizeof(sx126x_pkt_params_lora_t));
	lora_params->preamble_len_in_symb=12;
	lora_params->header_type=0;
	lora_params->pld_len_in_bytes=10;
	lora_params->crc_is_on=1;
	lora_params->invert_iq_is_on=0;
	command_status = sx126x_set_lora_pkt_params(&hspi1, lora_params);
	free(lora_params);
	command_status = sx126x_set_tx(&hspi1, 1000);//0x061A80);
	HAL_Delay(1400);

	if (command_status != HAL_OK) {
		transmitBuffer("setTx Failed\n");
		transmitBuffer("Set TX command status: ");
		transmitStatus(command_status);

		sx126x_chip_status_t device_status;
		command_status = sx126x_get_status(&hspi1, &device_status);

		transmitBuffer("Get Status command status: ");
		transmitStatus(command_status);
	}

	//get irq status
	uint8_t opcode3 = 0x12;
	uint8_t params3[3] = {0,0,0};
	uint8_t data3[3];
	readCommand(opcode3, params3, data3, 3);

	irq = data3[2] | data3[1] << 8;
	while ( (!(irq & SX126X_IRQ_TX_DONE)) && (!(irq & SX126X_IRQ_TIMEOUT)) ) {
		readCommand(opcode3, params3, data3, 3);
		irq = data3[2] << 8 | data3[1] << 8;
	}
	if ((irq & SX126X_IRQ_TIMEOUT)) {
		transmitBuffer("TIMEOUT!\n");
	} else {
		transmitBuffer("TX DONE!\n");
	}

	//setup();

	HAL_Delay(200);


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  Tx_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t buffer[10] = {1,2,3,4,5,6,7,8,9,10};
	  TxProtocol(buffer, 10);

	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NSS_1_Pin|NSS_2_Pin|NRESET_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRESET_1_GPIO_Port, NRESET_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NSS_1_Pin NSS_2_Pin NRESET_2_Pin */
  GPIO_InitStruct.Pin = NSS_1_Pin|NSS_2_Pin|NRESET_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUSY_1_Pin BUSY_2_Pin DIO1_2_Pin */
  GPIO_InitStruct.Pin = BUSY_1_Pin|BUSY_2_Pin|DIO1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NRESET_1_Pin */
  GPIO_InitStruct.Pin = NRESET_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRESET_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO1_1_Pin DIO2_1_Pin DIO3_1_Pin */
  GPIO_InitStruct.Pin = DIO1_1_Pin|DIO2_1_Pin|DIO3_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
