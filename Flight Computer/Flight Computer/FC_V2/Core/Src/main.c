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
#include <string.h>
#include "lsm6dsr_reg.h"
#include "lps22hh_reg.h"
#include "ejection.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---------- COMMENT THESE OUT AS NEEDED ---------- //
#define		DEBUG_MODE
//#define		TRANSMIT_RADIO
// ------------------------------------------------- //

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Transmission buffer
static uint8_t tx_buffer[TX_BUF_DIM];
#ifdef DEBUG_MODE
char msg[1000];
#endif

// Ejection Variables
float alt_meas;
float alt_ground = 0;
float t_previous_loop, t_previous_buzz;
float average_gradient;

uint8_t apogee_reached = 0;
uint8_t launched = 0;
uint8_t main_deployed = 0;

long time11;
long time12;
long time21;
long time22;

// Telemetry Variables
uint8_t telemetry_counter = 0;
//float real_altitude;
//uint8_t relay_check_drogue1;
//uint8_t relay_check_drogue2;
//uint8_t relay_check_main1;
//uint8_t relay_check_main2;
stmdev_ctx_t dev_ctx_lsm;
stmdev_ctx_t dev_ctx_lps;
uint16_t transmit_delay_time = 1000;	// After landing, transmit every second (change this)

// Variables to store converted sensor data
float acceleration[] = {0, 0, 0};
float angular_rate[]= {0, 0, 0};
float pressure = 0;
float temperature = 0;

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
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// LSM6DSR functions
extern stmdev_ctx_t lsm6dsr_init(void);
extern void get_acceleration(stmdev_ctx_t dev_ctx, float *acceleration_mg);
extern void get_angvelocity(stmdev_ctx_t dev_ctx, float *angular_rate_mdps);

// LPS22HH functions
extern stmdev_ctx_t lps22hh_init(void);
extern void get_pressure(stmdev_ctx_t dev_ctx,  float *pressure);
extern void get_temperature(stmdev_ctx_t dev_ctx,  float *temperature);

// Convert pressure to altitude function
uint32_t getAltitude();

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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Reset GPIOs
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Drogue_1_GPIO_Port, Relay_Drogue_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Drogue_2_GPIO_Port, Relay_Drogue_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Main_1_GPIO_Port, Relay_Main_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Main_2_GPIO_Port, Relay_Main_2_Pin, GPIO_PIN_RESET);

  // Initialize sensors
  // TODO: Add check mechanism (eg. if dev_ctx_lps return an error, sound the buzzer and light LED. Else, do smth)
//  dev_ctx_lsm = lsm6dsr_init();
  dev_ctx_lps = lps22hh_init();

  // Start timer
  HAL_TIM_Base_Start_IT(&htim2);

  // Buzzer
  // TODO: Add buzzer here to signal start of ejection

  // ---------- START OF EJECTION CODE ----------
  uint32_t altitude = 0;
  uint32_t alt_filtered = 0;
//  uint32_t currTick;

  // ---------- Get ground-level pressure and set as bias ----------
  for (uint16_t i = 0; i < ALT_MEAS_AVGING; i++){
      altitude = getAltitude();
      alt_ground += altitude;
  }
  alt_ground = alt_ground/ALT_MEAS_AVGING; 			// Average of altitude readings

  // ---------- Waiting for launch ----------
  // TODO: Replace 150 by the actual value
  while(alt_filtered < 150){							// Waiting to launch
	  altitude = getAltitude(dev_ctx_lps, &pressure);
      alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
#ifdef DEBUG_MODE
      sprintf(msg, "Filtered Alt =  %hu\n", alt_filtered);
      HAL_UART_Transmit(&huart2, msg, strlen((char const *)msg), 1000);
      HAL_Delay(1000);
#else
      HAL_Delay(50);
#endif

  }

  // ---------- Launched -> Wait for apogee ----------
  while (getAverageVelocity() > -DROGUE_DEPLOYMENT_VEL || alt_filtered < THRESHOLD_ALTITUDE) // while moving up and hasn't reached threshold altitude yet
  {
	  altitude = getAltitude(dev_ctx_lps, &pressure);
      alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
      HAL_Delay(5);
  }

  // ---------- At apogee -> Deploy drogue ----------
  // TODO: Modify code to actually deploy both relays
  HAL_GPIO_WritePin(Relay_Drogue_1_GPIO_Port, Relay_Drogue_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Relay_Drogue_2_GPIO_Port, Relay_Drogue_2_Pin, GPIO_PIN_SET);
  HAL_Delay(DROGUE_DELAY);
  HAL_GPIO_WritePin(Relay_Drogue_1_GPIO_Port, Relay_Drogue_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Drogue_2_GPIO_Port, Relay_Drogue_2_Pin, GPIO_PIN_RESET);

  // ---------- Wait for main deployment altitude ----------
  while (alt_filtered > MAIN_DEPLOYMENT){
	  altitude = getAltitude(dev_ctx_lps, &pressure);
      alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
      HAL_Delay(5);
  }

  // ---------- At main deployment altitude -> Deploy main ----------
  // TODO: Modify code to actually deploy both relays
  HAL_GPIO_WritePin(Relay_Main_1_GPIO_Port, Relay_Main_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Relay_Main_2_GPIO_Port, Relay_Main_2_Pin, GPIO_PIN_SET);
  HAL_Delay(MAIN_DELAY);
  HAL_GPIO_WritePin(Relay_Main_1_GPIO_Port, Relay_Main_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Main_2_GPIO_Port, Relay_Main_2_Pin, GPIO_PIN_RESET);

  // ---------- END OF EJECTION CODE ----------
  // Stop the timer from interrupting and enter while loop
  HAL_TIM_Base_Stop_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	get_pressure(dev_ctx_lps, &pressure);
	get_temperature(dev_ctx_lps,  &temperature);
//	get_acceleration(dev_ctx_lsm, acceleration);
//	get_angvelocity(dev_ctx_lsm, angular_rate);
//	get_gps();
	sprintf((char *)tx_buffer, "TIME -- Hour:%hu\t\t Minute:%hu\t Second:%hu\nDATA -- Temperature:%hu\tPressure:%hu\n", (uint16_t)stimestructureget.Hours, (uint16_t)stimestructureget.Minutes, (uint16_t)stimestructureget.Seconds, (uint16_t)temperature, (uint16_t)pressure);
#ifndef DEBUG_MODE
	HAL_UART_Transmit(&huart1, tx_buffer, strlen((char const *)tx_buffer), 1000);
#else
	HAL_UART_Transmit(&huart2, tx_buffer, strlen((char const *)tx_buffer), 1000);
#endif

	HAL_Delay(transmit_delay_time);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

//  RTC_DateTypeDef sdatestructure;
//  RTC_TimeTypeDef stimestructure;
//
//  /*##-1- Configure the Date #################################################*/
//  /* Set Date: Tuesday April 1st 2021 */
//  sdatestructure.Year = 0x21;
//  sdatestructure.Month = RTC_MONTH_APRIL;
//  sdatestructure.Date = 0x01;
//  sdatestructure.WeekDay = RTC_WEEKDAY_THURSDAY;
//
//  if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
//  {
//	  /* Initialization Error */
//	  Error_Handler();
//  }
//
//  /*##-2- Configure the Time #################################################*/
//  /* Set Time: 01:45:00 */
//  stimestructure.Hours = 0x01;
//  stimestructure.Minutes = 0x45;
//  stimestructure.Seconds = 0x00;
//  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
//  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
//  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
//
//  if (HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BCD) != HAL_OK)
//  {
//	  /* Initialization Error */
//	  Error_Handler();
//  }

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, LED_Status_Pin|Relay_Main_1_Pin|Relay_Main_2_Pin|LED3_Pin
                          |LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Relay_Drogue_1_Pin|Relay_Drogue_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Status_Pin Relay_Main_1_Pin Relay_Main_2_Pin LED3_Pin
                           LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED_Status_Pin|Relay_Main_1_Pin|Relay_Main_2_Pin|LED3_Pin
                          |LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Relay_Drogue_1_Pin Relay_Drogue_2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Relay_Drogue_1_Pin|Relay_Drogue_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Function Definitions
uint32_t getAltitude(stmdev_ctx_t dev_ctx){
	get_pressure(dev_ctx_lps, &pressure);
	uint32_t altitude = pressure;
	return altitude;						// TODO: Add conversion from pressure to altitude here
}

// Callbacks
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim2){
		switch (currTask){
			case 0:
				// Date/Time
				HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
				currTask++;
				break;
			case 1:
				// Pressure/Temp
				get_pressure(dev_ctx_lps, &pressure);
				get_temperature(dev_ctx_lps,  &temperature);
				currTask++;
				break;
			case 2:
				// Acceleration/Ang Velocity
//				get_acceleration(dev_ctx_lsm, acceleration);
//				get_angvelocity(dev_ctx_lsm, angular_rate);
				currTask++;
				break;
			case 3:
				// GPS
//				get_gps();
				currTask++;
				break;
			default:
				// Transmit to radio
				sprintf((char *)tx_buffer, "TIME -- Hour:%hu\t\t Minute:%hu\t Second:%hu\nDATA -- Temperature:%hu\tPressure:%hu\n", (uint16_t)stimestructureget.Hours, (uint16_t)stimestructureget.Minutes, (uint16_t)stimestructureget.Seconds, (uint16_t)temperature, (uint16_t)pressure);
#ifndef DEBUG_MODE
				HAL_UART_Transmit(&huart1, tx_buffer, strlen((char const *)tx_buffer ), 1000);
#else
				HAL_UART_Transmit(&huart2, tx_buffer, strlen((char const *)tx_buffer), 1000);
#endif
				currTask = 0;
				break;
		}
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
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);
  __BKPT();
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
