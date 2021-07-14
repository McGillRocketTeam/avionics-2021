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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lsm6dsr_reg.h"
#include "lps22hh_reg.h"
#include "ejection.h"
#include "gps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---------- COMMENT THESE OUT AS NEEDED ---------- //
#define		DEBUG_MODE
//#define		CONFIG_TIME
//#define		TRANSMIT_RADIO
#define 	SIM_PRESSURE_SENSOR
// ------------------------------------------------- //

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Transmission buffer
static uint8_t tx_buffer[TX_BUF_DIM];
#ifdef DEBUG_MODE
static uint8_t msg[1000];
#endif

// Ejection Variables
float alt_meas;
float alt_ground = 0;
float t_previous_loop, t_previous_buzz;
float average_gradient;

uint8_t apogee_reached = 0;
uint8_t launched = 0;
uint8_t main_deployed = 0;

uint8_t readingLps = 0;

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
float latitude;
float longitude;
float time;

// RTC date and time structures
RTC_DateTypeDef sdatestructureget;
RTC_TimeTypeDef stimestructureget;

// Timer variable
volatile uint8_t currTask = 0;

// SD card FatFS variables
FATFS FatFs; 	//Fatfs handle, may need to be static
FIL fil; 		//File handle
FRESULT fres; 	//Result after operations


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

// LSM6DSR functions
extern stmdev_ctx_t lsm6dsr_init(void);
extern void get_acceleration(stmdev_ctx_t dev_ctx, float *acceleration_mg);
extern void get_angvelocity(stmdev_ctx_t dev_ctx, float *angular_rate_mdps);

// LPS22HH functions
#ifdef SIM_PRESSURE_SENSOR
  extern void get_pressure(float *pressure);
  extern void get_temperature(float *temperature);
#else
  extern stmdev_ctx_t lps22hh_init(void);
  extern void get_pressure(stmdev_ctx_t dev_ctx,  float *pressure);
  extern void get_temperature(stmdev_ctx_t dev_ctx,  float *temperature);
#endif

// GPS functions
void GPS_Poll(float*, float*, float*);

// Convert pressure to altitude function
uint32_t getAltitude();

// SD initialization with new file and initial write
FRESULT sd_init();

// save data to sd card
FRESULT sd_save(float pressure, float temperature, float accelx,
		float magx, float latitude, float longitude, float gpsTime);

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
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  // INDICATE TO SCRIPT TO START READING ALTITUDE VALUES FROM CSV
#ifdef SIM_PRESSURE_SENSOR
  sprintf((char *)msg, "---------- TELL SCRIPT TO START ----------\n");
  HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);

  sprintf((char *)msg, "S\n"); // transmit s to start
  HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif

  // Reset GPIOs
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED_Status_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Drogue_1_GPIO_Port, Relay_Drogue_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Drogue_2_GPIO_Port, Relay_Drogue_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Main_1_GPIO_Port, Relay_Main_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Main_2_GPIO_Port, Relay_Main_2_Pin, GPIO_PIN_RESET);

  // Initialize sensors
  // TODO: Add check mechanism (eg. if dev_ctx_lps return an error, sound the buzzer. Else, do smth)
#ifndef SIM_PRESSURE_SENSOR
  dev_ctx_lsm = lsm6dsr_init();
  dev_ctx_lps = lps22hh_init();
#endif


  // Get initial sensor values
  get_acceleration(dev_ctx_lsm, acceleration);
  get_angvelocity(dev_ctx_lsm, angular_rate);
  GPS_Poll(&latitude, &longitude, &time);
#ifndef SIM_PRESSURE_SENSOR
  get_pressure(dev_ctx_lps, &pressure);
  get_temperature(dev_ctx_lps,  &temperature);
#else
  get_pressure(&pressure);
  get_temperature(&temperature);
#endif

#ifdef DEBUG_MODE
      sprintf((char *)msg, "---------- INITIALIZED ALL SENSORS ----------\n");
      HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
      HAL_Delay(1000);
#endif

  // Start timer
  HAL_TIM_Base_Start_IT(&htim2);

  // Start buzzer
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

#ifdef DEBUG_MODE
  HAL_Delay(5000);		// Turning off buzzer in debug mode because it really b annoying
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
#endif

  // initialize SD card and open file to begin writing
  fres = sd_init();
  if (fres != FR_OK)
  {
	  // TODO: do some error handling, @jennie idk how you want to do this
  }

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

#ifdef DEBUG_MODE
  sprintf((char *)msg, "---------- AVERAGE OF ALT READINGS: %hu ft. NOW WAITING FOR LAUNCH ----------\n", (uint16_t)alt_ground);
  HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif

  // ---------- Waiting for launch ----------
  // TODO: Replace 150 by the actual value
  while(alt_filtered < 150){							// Waiting to launch
	  altitude = getAltitude();
      alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
#ifdef DEBUG_MODE
      sprintf((char *)msg, "Filtered Alt =  %hu\n\n", (uint16_t)alt_filtered);
      HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
      HAL_Delay(1000);
#else
      HAL_Delay(50);
#endif

  }

  // ---------- Launched -> Wait for apogee ----------
#ifdef DEBUG_MODE
  sprintf((char *)msg, "---------- LAUNCHED ----------\n");
  HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif

  while (getAverageVelocity() > -DROGUE_DEPLOYMENT_VEL || alt_filtered < THRESHOLD_ALTITUDE) // while moving up and hasn't reached threshold altitude yet
  {
	  altitude = getAltitude();
      alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
      HAL_Delay(5);
  }

  // ---------- At apogee -> Deploy drogue ----------
#ifdef DEBUG_MODE
  sprintf((char *)msg, "---------- AT APOGEE - DEPLOYING DROGUE AT %hu ft ----------\n", (uint16_t)altitude);
  HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif

  HAL_GPIO_WritePin(Relay_Drogue_1_GPIO_Port, Relay_Drogue_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Relay_Drogue_2_GPIO_Port, Relay_Drogue_2_Pin, GPIO_PIN_SET);
  HAL_Delay(DROGUE_DELAY);
  HAL_GPIO_WritePin(Relay_Drogue_1_GPIO_Port, Relay_Drogue_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Drogue_2_GPIO_Port, Relay_Drogue_2_Pin, GPIO_PIN_RESET);

  // ---------- Wait for main deployment altitude ----------
  while (alt_filtered > MAIN_DEPLOYMENT){
	  altitude = getAltitude();
      alt_filtered = runAltitudeMeasurements(HAL_GetTick(), altitude);
      HAL_Delay(5);
  }

  // ---------- At main deployment altitude -> Deploy main ----------
#ifdef DEBUG_MODE
  sprintf((char *)msg, "---------- DEPLOYING MAIN AT %hu ft ----------\n", (uint16_t)altitude);
  HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif

  HAL_GPIO_WritePin(Relay_Main_1_GPIO_Port, Relay_Main_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Relay_Main_2_GPIO_Port, Relay_Main_2_Pin, GPIO_PIN_SET);
  HAL_Delay(MAIN_DELAY);
  HAL_GPIO_WritePin(Relay_Main_1_GPIO_Port, Relay_Main_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Relay_Main_2_GPIO_Port, Relay_Main_2_Pin, GPIO_PIN_RESET);

  // ---------- END OF EJECTION CODE ----------
#ifdef DEBUG_MODE
  sprintf((char *)msg, "---------- EXITING EJECTION ----------\n");
  HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif

  // Stop the timer from interrupting and enter while loop
  HAL_TIM_Base_Stop_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

#ifndef SIM_PRESSURE_SENSOR
	get_pressure(dev_ctx_lps, &pressure);
	get_temperature(dev_ctx_lps,  &temperature);
#else
	get_pressure(&pressure);
	get_temperature(&temperature);
#endif

	get_acceleration(dev_ctx_lsm, acceleration);
	get_angvelocity(dev_ctx_lsm, angular_rate);
	GPS_Poll(&latitude, &longitude, &time);
	fres = sd_save(pressure, temperature, acceleration[0], angular_rate[0], latitude, longitude, time);
	// TODO: can add handling if fres != FR_OK for sd_save
#ifndef DEBUG_MODE
	// Transmit via radio
	// TODO: Replace with RnD radio
	HAL_UART_Transmit(&huart1, tx_buffer, strlen((char const *)tx_buffer), 1000);
#else
	HAL_UART_Transmit(&huart3, tx_buffer, strlen((char const *)tx_buffer), 1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
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

#ifdef CONFIG_TIME
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday April 8th 2021 */
  sdatestructure.Year = 0x21;
  sdatestructure.Month = RTC_MONTH_APRIL;
  sdatestructure.Date = 0x08;
  sdatestructure.WeekDay = RTC_WEEKDAY_THURSDAY;

  if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
	  /* Initialization Error */
	  Error_Handler();
  }

  /*##-2- Configure the Time #################################################*/
  /* Set Time: 01:45:00 */
  stimestructure.Hours = 0x02;
  stimestructure.Minutes = 0x47;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 163;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 82;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Status_Pin|Relay_Main_1_Pin|Relay_Main_2_Pin|LED3_Pin
                          |LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Relay_Drogue_1_Pin|Relay_Drogue_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_SD_GPIO_Port, SPI2_CS_SD_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : SPI2_CS_SD_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_SD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Function Definitions
// TODO: Move this to sensor_functions.c
uint32_t getAltitude(){
	readingLps = 1;
#ifndef SIM_PRESSURE_SENSOR
	get_pressure(dev_ctx_lps, &pressure);
#else
	get_pressure(&pressure);
#endif
	readingLps = 0;
	uint32_t altitude = 145442.1609 * (1.0 - pow(pressure/LOCAL_PRESSURE, 0.190266436));
	return altitude;
}

/*
 * performs initialization for sd card.
 * mounts the sd card, creates new file for data logging, and
 * writes row of headers for the data into the file.
 *
 * returns FR_OK if all operations complete without error.
 * otherwise returns FRESULT (not FR_OK) to indicate error occurred.
 *
 */
FRESULT sd_init() {
	//Open the file system
	FRESULT fres = f_mount(&FatFs, "", 1); // 1 = mount now
	if (fres != FR_OK) {
#ifdef DEBUG_MODE
		sprintf((char *)msg, "SD f_mount failed, fres = %i\n\n", fres);
		HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		return fres;
    }

	// get current date and time to create new file for data logging
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);

    char filename[35];
	sprintf(filename, "datalog_%hu%hu%hu_%hu%hu%hu.txt",
			(uint16_t) sdatestructureget.Year,  (uint16_t) sdatestructureget.Month, (uint16_t) sdatestructureget.Date,
	  	    (uint16_t) stimestructureget.Hours, (uint16_t) stimestructureget.Minutes, (uint16_t) stimestructureget.Seconds);
	fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS);

	if (fres != FR_OK) {
#ifdef DEBUG_MODE
		sprintf((char *)msg, "SD file open failed, fres = %i\n\n", fres);
		HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	    return fres;
	  }

	  // write data headers (time, altitude, temp, etc.)
	char sdHeader[] = "Hour,Minute,Second,Temperature(C),Pressure(hPA),Accelx,Magx,Longitude,Latitude,Time(GPS)\n";
	UINT bytesWrote;
	fres = f_write(&fil, sdHeader, strlen(sdHeader), &bytesWrote);
	if(fres == FR_OK) {
#ifdef DEBUG_MODE
		printf((char *)msg, "SD header write successful, fres = %i\n\n", fres);
		HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif
	} else {
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
#ifdef DEBUG_MODE
		sprintf((char *)msg, "SD header write failed, fres = %i\n\n", fres);
		HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#endif
		return fres;
	}

	return fres; // if this line is reached then fres = FR_OK
}

FRESULT sd_save(float pressure, float temperature, float accelx,
		float magx, float latitude, float longitude, float gpsTime) {
	sprintf((char *) tx_buffer, "%hu,%hu,%hu,%f,%f,%f,%f,%.3f,%.3f,%.3f\n", (uint16_t)stimestructureget.Hours, (uint16_t)stimestructureget.Minutes, (uint16_t)stimestructureget.Seconds, temperature, pressure, accelx, magx, longitude, latitude, time);
	UINT bytesWrote;
	fres = f_write(&fil, (char *) tx_buffer, strlen((char *) tx_buffer), &bytesWrote);
	if (fres != FR_OK) {
#ifdef DEBUG_MODE
		printf((char *)msg, "SD data write failed, fres = %i, bytes written = %d\n\n", fres, bytesWrote);
		HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#else
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
#endif
	}
	return fres;
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
				if (readingLps != 0){		// Yield to ejection if it has control of the LPS sensor
					break;
				}

#ifndef SIM_PRESSURE_SENSOR
				get_pressure(dev_ctx_lps, &pressure);
				get_temperature(dev_ctx_lps,  &temperature);
#else
				get_pressure(&pressure);
				get_temperature(&temperature);
#endif
				currTask++;
				break;
			case 2:
				// Acceleration/Ang Velocity
#ifndef SIM_PRESSURE_SENSOR // could remove this if testing with an lsm sensor while simulating pressure
				get_acceleration(dev_ctx_lsm, acceleration);
				get_angvelocity(dev_ctx_lsm, angular_rate);
				currTask++;
				break;
#else
				sprintf((char *)tx_buffer, "acceleration = nada, angvelocity = chacha\n");
				HAL_UART_Transmit(&huart1, tx_buffer, strlen((char const *)tx_buffer), 1000);
				currTask++;
#endif
			case 3:
				// GPS
#ifndef SIM_PRESSURE_SENSOR // maybe remove this ifndef?
				GPS_Poll(&latitude, &longitude, &time);
#endif
				currTask++;
				break;
			case 4:
				// Save to SD card
				fres = sd_save(pressure, temperature, acceleration[0], angular_rate[0], latitude, longitude, time);
				if (fres != FR_OK) {
#ifdef DEBUG_MODE
					printf((char *)msg, "SD data write failed, fres = %i\n\n", fres);
					HAL_UART_Transmit(&huart3, msg, strlen((char const *)msg), 1000);
#else
					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
#endif
				}
				currTask++;
				break;
			default:
				// Transmit to radio
				sprintf((char *)tx_buffer, "TIME -- Hour:%hu\t\t Minute:%hu\t Second:%hu\nDATA -- Temperature:%hu\tPressure:%hu\tAccelx:%hu\tMagx:%hu\nGPS  -- Longitude:%.3f\tLatitude:%.3f\tTime:%.3f\n\n", (uint16_t)stimestructureget.Hours, (uint16_t)stimestructureget.Minutes, (uint16_t)stimestructureget.Seconds, (uint16_t)temperature, (uint16_t)pressure, (uint16_t)acceleration[0], (uint16_t)angular_rate[0], longitude, latitude, time);
#ifndef DEBUG_MODE
				HAL_UART_Transmit(&huart1, tx_buffer, strlen((char const *)tx_buffer ), 1000);
#else
				HAL_UART_Transmit(&huart3, tx_buffer, strlen((char const *)tx_buffer), 1000);
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
