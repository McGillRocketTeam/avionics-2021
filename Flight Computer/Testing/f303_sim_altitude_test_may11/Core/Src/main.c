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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ejection.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_MODE
#define CONFIG_TIME

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Transmission buffer
//static uint8_t rx_buffer[TX_BUF_DIM];
//static uint8_t tx_buffer[TX_BUF_DIM];
static uint8_t msg_buffer[TX_BUF_DIM];

// Ejection Variables
float alt_meas;
float alt_ground = 0;
float t_previous_loop, t_previous_buzz;
float average_gradient;

uint8_t apogee_reached = 0;
uint8_t inFlight = 0;
uint8_t main_deployed = 0;

uint32_t count = 0;

uint8_t readingLps = 0;

long time11;
long time12;
long time21;
long time22;

// Continuity and current measurements
float vsense_input;
float vsense_drogue;
float vsense_main;
uint8_t drogue_cont_1;
uint8_t drogue_cont_2;
uint8_t main_cont_1;
uint8_t main_cont_2;

// Telemetry Variables
uint8_t telemetry_counter = 0;

// Variables to store converted sensor data
float acceleration[] = {0, 0, 0};
float angular_rate[]= {0, 0, 0};
float pressure = 0;
float temperature = 0;
float latitude;
float longitude;
float time;

uint32_t data_raw_pressure;

// RTC date and time structures
RTC_DateTypeDef sdatestructureget;
RTC_TimeTypeDef stimestructureget;

// Timer variable
volatile uint8_t currTask = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

// Convert pressure to altitude function
uint32_t getAltitude();
void get_pressure(float *pressure);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  // Reset GPIOs
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  // Reset error status --- indicators -> 0:SD, 1:LSM, 2:LPS, 3:Radio, 4:GPS, 5:Any
  memset(FC_Errors, 0, 6*sizeof(*FC_Errors));
  FC_Errors[5] = 1;

  // Get initial sensor values (will return 0)
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

  // Stay inside loop until button pressed
  while (HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin)){
	  if (FC_Errors[5])
		  HAL_GPIO_TogglePin(LED_Status_GPIO_Port, LED_Status_Pin);
	  HAL_Delay(250);
  }
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);

  sprintf((char *)msg_buffer, "---------- TELL SCRIPT TO START ----------\n");
  HAL_UART_Transmit(&huart2, msg_buffer, strlen((char const *)msg_buffer), 1000);

  sprintf((char *)msg_buffer, "S\n"); // transmit s to start
  HAL_UART_Transmit(&huart2, msg_buffer, strlen((char const *)msg_buffer), 1000);


  // Reset LED status (now indicating ejection events)
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  // ---------- START OF EJECTION CODE ----------
    uint32_t altitude = 0;
    uint32_t alt_filtered = 0;

    // Get ground-level pressure and set as bias
    for (uint16_t i = 0; i < ALT_MEAS_AVGING; i++){
  	  altitude = getAltitude();
  	  alt_ground += altitude;
    }
    alt_ground = alt_ground/ALT_MEAS_AVGING; 			// Average of altitude readings

  #ifdef DEBUG_MODE
    sprintf((char *)msg_buffer, "---------- AVERAGE OF ALT READINGS: %hu ft. NOW WAITING FOR LAUNCH ----------\n", (uint16_t)alt_ground);
    HAL_UART_Transmit(&huart2, msg_buffer, strlen((char const *)msg_buffer), 1000);
  #endif

    // Waiting for launch
    // TODO: Replace 500 by the actual value
    while(alt_filtered < 500){							// Waiting to launch
  	altitude = getAltitude();
  	alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
  #ifdef DEBUG_MODE
  	sprintf((char *)msg_buffer, "Filtered Alt =  %hu\n\n", (uint16_t)alt_filtered);
  	HAL_UART_Transmit(&huart2, msg_buffer, strlen((char const *)msg_buffer), 1000);
  #endif
  	HAL_Delay(15);
    }

    // Launched -> Wait for apogee
    // Set flight indicator
    inFlight = 1;
  #ifdef DEBUG_MODE
    sprintf((char *)msg_buffer, "---------- LAUNCHED ----------\n");
    HAL_UART_Transmit(&huart2, msg_buffer, strlen((char const *)msg_buffer), 1000);

    // Indicate status thru LED
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  #endif

    while (getAverageVelocity() > -DROGUE_DEPLOYMENT_VEL || alt_filtered < THRESHOLD_ALTITUDE){ // while moving up and hasn't reached threshold altitude yet
		altitude = getAltitude();
		alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
		HAL_Delay(15);
    }

  // At apogee -> Deploy drogue
  #ifdef DEBUG_MODE
    sprintf((char *)msg_buffer, "---------- AT APOGEE - DEPLOYING DROGUE AT %hu ft ----------\n", (uint16_t)altitude);
    HAL_UART_Transmit(&huart2, msg_buffer, strlen((char const *)msg_buffer), 1000);

    // Indicate status thru LED
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  #endif

    // Deploy drogue
    HAL_Delay(DROGUE_DELAY);


    // Wait for main deployment altitude
    while (alt_filtered > MAIN_DEPLOYMENT){
      altitude = getAltitude();
      alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
      HAL_Delay(15);
    }

    // At main deployment altitude -> Deploy main
  #ifdef DEBUG_MODE
    sprintf((char *)msg_buffer, "---------- DEPLOYING MAIN AT %hu ft ----------\n", (uint16_t)altitude);
    HAL_UART_Transmit(&huart2, msg_buffer, strlen((char const *)msg_buffer), 1000);

    // Indicate status thru LED
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  #endif

    HAL_Delay(MAIN_DELAY);

    // Landing detection
    while (count < LANDING_SAMPLES) {
      if (filterAltitude(altitude, time) - alt_previous[NUM_MEAS_AVGING] < LANDING_THRESHOLD)
      	count++;
      else
      	count = 0;
      HAL_Delay(15);
    }

    // Set flight status to 0
    inFlight = 0;

    // Landing
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  // ---------- END OF EJECTION CODE ----------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_Delay(1000);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
#ifdef CONFIG_TIME
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Thursday May 6th 2021 */
  sdatestructure.Year = 0x21;
  sdatestructure.Month = RTC_MONTH_MAY;
  sdatestructure.Date = 0x06;
  sdatestructure.WeekDay = RTC_WEEKDAY_THURSDAY;

  if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }

  /*##-2- Configure the Time #################################################*/
  /* Set Time: 11:00:00 PM */
  stimestructure.Hours = 0x11;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_PM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BCD) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }
#endif
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Status_Pin */
  GPIO_InitStruct.Pin = LED_Status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Status_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint32_t getAltitude(){
	get_pressure(&pressure);
	uint32_t altitude = 145442.1609 * (1.0 - pow(pressure/LOCAL_PRESSURE, 0.190266436));
	return altitude;
}

void get_pressure(float *pressure) {
	// first need to transmit a '0\r\n'
	// so that the script knows to send a value
	// need the \n since script uses readline() and searches for \n termination
	uint8_t startMessage[] = "0\n";

//	#ifdef DEBUG_MODE
//	  sprintf((char *)msg_buffer, "---------- ENTERED get_pressure ----------\nstartMessage = \n");
//	  HAL_UART_Transmit(&huart2, msg_buffer, strlen((char const *)msg_buffer), 5);
//	#endif

	uint32_t timeout = 1000;
	HAL_UART_Transmit(&huart2, startMessage, sizeof(startMessage), timeout);

	// now receive input from script
	uint16_t max_loop_count = 10;
	uint16_t loop_count = 0;

	uint8_t rxBuf[10]; // buffer of 10 chars
	uint8_t rxCurrent; // current receive char
	uint8_t rxIndex = 0;

	int done = 0;
	while (loop_count < max_loop_count && !done) {
		HAL_UART_Receive(&huart2, (uint8_t*) &rxCurrent, 1, timeout);
		if (rxCurrent != '\n' && rxIndex < sizeof(rxBuf)) {
			rxBuf[rxIndex++] = rxCurrent;
		} else {
			// convert to uint32_t as data_raw_pressure
			data_raw_pressure = (uint32_t) (atoi((char *) rxBuf));
			*pressure = (float) data_raw_pressure;
			memset(rxBuf, 0, sizeof(rxBuf));
			done = 1;
		}
		__HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
		loop_count++;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
