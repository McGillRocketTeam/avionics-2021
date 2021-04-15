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

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
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
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi1, &opcode, 1, 100);
	status = HAL_SPI_Transmit(&hspi1, (uint8_t*)params, numOfParams, 100);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef readCommand(uint8_t opcode, uint8_t params[], uint8_t response[], uint16_t numOfParams){
	HAL_StatusTypeDef status;
	while(HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin) == GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi1, &opcode, 1, 100);
	status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)params, (uint8_t*)response, numOfParams, 100);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
	return status;
}

void TxProtocol(){

	//set to standby
	sx126x_set_standby(&hspi1, 0);

	//set packet type
	sx126x_set_pkt_type(&hspi1, 1);

	//set frequency
	sx126x_set_rf_freq(&hspi1, 432000000);

	//set pa config
	struct sx126x_pa_cfg_params_s *params = malloc(sizeof(sx126x_pa_cfg_params_t));
	params->pa_duty_cycle=3;
	params->hp_max=5;
	params->device_sel=0;
	params->pa_lut=1;
	sx126x_set_pa_cfg(&hspi1, params);
	free(params);

	//set tx params
	int8_t power = 22;
	sx126x_set_tx_params(&hspi1, power, SX126X_RAMP_80_US);

	//set buffer base address
	sx126x_set_buffer_base_address(&hspi1, 0, 0);

	//write buffer
	uint8_t buffer[2] = {1,2};
	sx126x_write_buffer(&hspi1, 0, buffer, 2);

	//modulation parameters
	struct sx126x_mod_params_lora_s *mod_params = malloc(sizeof(sx126x_mod_params_lora_t));
	mod_params->sf=8;
	mod_params->bw=0;
	mod_params->cr=1;
	mod_params->ldro=0;
	sx126x_set_lora_mod_params(&hspi1, mod_params);
	free(mod_params);

	//packet params
	struct sx126x_pkt_params_lora_s *lora_params = malloc(sizeof(sx126x_pkt_params_lora_t));
	lora_params->preamble_len_in_symb=8;
	lora_params->header_type=SX126X_LORA_PKT_EXPLICIT;
	lora_params->pld_len_in_bytes=2;
	lora_params->crc_is_on=0;
	lora_params->invert_iq_is_on=0;
	sx126x_set_lora_pkt_params(&hspi1, lora_params);
	free(lora_params);

	//set dio and irq params
	sx126x_set_dio_irq_params(&hspi1,1,1,0,0);

	//set tx mode
	sx126x_set_tx(&hspi1, 100);

	//wait for tx done
	while(HAL_GPIO_ReadPin(DIO1_GPIO_Port, DIO1_Pin) == GPIO_PIN_SET);

	//clear irq status
	sx126x_irq_mask_t irq = SX126X_IRQ_TX_DONE  ;
	sx126x_get_and_clear_irq_status(&hspi1, &irq);


	//1. Set to Standby Mode
	//uint8_t opcode = 128; //0x80
	//uint8_t params1[1] = {0};
	//uint16_t numParams = 1;
	//writeCommand(opcode, params1, numParams);

	//2. Set Packet Type
	//opcode = 138; //0x8A
	//uint8_t params2[1] = {1};
	//numParams = 1;
	//writeCommand(opcode, params2, numParams);

	//Get Packet Type test
	//opcode = 17;
	//uint8_t params[2] = {0,0};
	//uint8_t buffer[2];
	//numParams = 2;
	//HAL_StatusTypeDef status = readCommand(opcode, params, buffer, numParams);

	//3. Set Rf Frequency
	//opcode = 134; //0x86
	//uint8_t params3[4] = {0,0,0,255};
	//numParams = 4;
	//writeCommand(opcode, params3, numParams);

	//4. Set PA Config
	//opcode = 149; //0x95
	//uint8_t params4[4] = {3,5,0,1}; //check PA operating modes with optimal settings
	//numParams = 4;
	//writeCommand(opcode, params4, numParams);

	//6-3. Set Tx Parameters
	//opcode = 142; //0x8E
	//uint8_t params6_3[2] = {22,3};
	//numParams = 2;
	//writeCommand(opcode, params6_3, numParams);

	//5. Set Buffer Base Addresses
	//opcode = 143; //0x8F
	//uint8_t params5[2] = {0,0};
	//numParams = 2;
	//writeCommand(opcode, params5, numParams);

	//7. Write Data to Buffer
	//opcode = 14; //0x0E
	//uint8_t params7[2] = {0,4};
	//numParams = 2;
	//writeCommand(opcode, params7, numParams);

	//6-1. Set Modulation Parameters
	//opcode = 139; //0x8B
	//uint8_t params6_1[8] = {8,0,1,0,0,0,0,0};
	//numParams = 8;
	//writeCommand(opcode, params6_1, numParams);

	//6-2. Set Packet Parameters
	//opcode = 140; //0x8C
	//uint8_t params6_2[8] = {4,0,0,1,1,0,0,0};
	//numParams = 8;
	//writeCommand(opcode, params6_2, numParams);

	//8. Set DIO and IRQ
	//opcode = 131; //0x83
	//uint8_t params8[8] = {1,0,1,0,0,0,0,0};
	//numParams = 8;
	//writeCommand(opcode, params8, numParams);

	//9. Set to Tx Mode
	//opcode = 131; //0x83
	//uint8_t params9[3] = {255,255,255};
	//numParams = 3;
	//writeCommand(opcode, params9, numParams);

	//Get status
	//opcode = 192; //0xC0
	//uint8_t params_get_0[1] = {0};
	//uint8_t buffer0[1];
	//readCommand(opcode, params_get_0, buffer0, 1);

	//Get Device Errors
	//opcode = 23; //0x17
	//uint8_t params_get_1[3] = {0,0,0};
	//uint8_t buffer1[3];
	//readCommand(opcode, params_get_1, buffer1, 3);

	//Clear Device Errors
	//opcode = 7; //0x07
	//uint8_t params_get_2[2] = {0,0};
	//uint8_t buffer2[2];
	//readCommand(opcode, params_get_2, buffer2, 2);

	//Get IRQ Status
	//opcode = 18; //0x12
	//uint8_t params_get_irq[3] = {0,0,0};
	//uint8_t buffer_irq[3];
	//readCommand(opcode, params_get_irq, buffer_irq, 3);

	//Clear IRQ status
	//opcode = 2; //0x02
	//uint8_t params_get_3[2] = {1,0};
	//writeCommand(opcode, params_get_3, 2);

	transmitBuffer("IRQ Status: ");
	transmitIRQ(irq);
	transmitBuffer("\n\n");

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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(NRESET_GPIO_Port, NRESET_Pin, GPIO_PIN_SET); //Make sure reset is off
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	TxProtocol();
	HAL_Delay(500);
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
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRESET_GPIO_Port, NRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRESET_Pin */
  GPIO_InitStruct.Pin = NRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO1_Pin DIO2_Pin DIO3_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|DIO2_Pin|DIO3_Pin;
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
