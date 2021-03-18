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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include "lsm6dsr_reg.h"
#include "lps22hh_reg.h"
#include "ejection.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---------- COMMENT THESE OUT AS NEEDED ---------- //
//#define		DEBUG_MODE
//#define		TRANSMIT_RADIO
// ------------------------------------------------- //


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for queue01 */
osThreadId_t ejectionTaskHandle;
const osThreadAttr_t ejectionTask_attributes = {
		.name = "ejectionTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 512
};
/* Definitions for queue02 */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
		.name = "telemetryTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 2048
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
uint8_t myQueue01Buffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t myQueue01_attributes = {
		.name = "myQueue01",
		.cb_mem = &myQueue01ControlBlock,
		.cb_size = sizeof(myQueue01ControlBlock),
		.mq_mem = &myQueue01Buffer,
		.mq_size = sizeof(myQueue01Buffer)
};
/* USER CODE BEGIN PV */
static uint8_t tx_buffer[TX_BUF_DIM];

// Ejection Variables
float alt_meas;
float alt_ground = 0;
//float a = 1;					// Hz (Cutoff frequency of low-pass filter)
//float alt_filtered;
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
//float real_altitude;
//uint8_t relay_check_drogue1;
//uint8_t relay_check_drogue2;
//uint8_t relay_check_main1;
//uint8_t relay_check_main2;
stmdev_ctx_t dev_ctx_lsm;
stmdev_ctx_t dev_ctx_lps;

// Variables to store converted sensor data
float acceleration[] = {0, 0, 0};
float angular_rate[]= {0, 0, 0};
float pressure = 0;
float temperature = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void startEjection(void *argument);
void startTelemetry(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// LSM6DSR functions
extern stmdev_ctx_t lsm6dsr_init(void);
extern void get_acceleration(stmdev_ctx_t dev_ctx, float *acceleration_mg);
extern void get_angvelocity(stmdev_ctx_t dev_ctx, float *angular_rate_mdps);

// LPS22HH functions
extern stmdev_ctx_t lps22hh_init(void);
extern void get_pressure(stmdev_ctx_t dev_ctx,  float *pressure);
extern void get_temperature(stmdev_ctx_t dev_ctx,  float *temperature);
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

	/* USER CODE BEGIN 2 */
	// Reset GPIOs
	HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Relay_Drogue_GPIO_Port, Relay_Drogue_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Relay_Main_GPIO_Port, Relay_Main_Pin, GPIO_PIN_RESET);

	// Initialize sensors
	// TODO: Add check mechanism (eg. if dev_ctx_lps return an error, sound the buzzer and light LED. Else, do smth)
	// dev_ctx_lsm = lsm6dsr_init();
	dev_ctx_lps = lps22hh_init();

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of myQueue01 */
	myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */


	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of queue01 */
	ejectionTaskHandle = osThreadNew(startEjection, NULL, &ejectionTask_attributes);

	/* creation of queue02 */
	telemetryTaskHandle = osThreadNew(startTelemetry, NULL, &telemetryTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
	HAL_GPIO_WritePin(GPIOC, LED_Status_Pin|Relay_Drogue_Pin|Relay_Main_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_Status_Pin Relay_Drogue_Pin Relay_Main_Pin */
	GPIO_InitStruct.Pin = LED_Status_Pin|Relay_Drogue_Pin|Relay_Main_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : Button_Pin */
	GPIO_InitStruct.Pin = Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Buzzer_Pin */
	GPIO_InitStruct.Pin = Buzzer_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN startEjection */
/**
 * @brief  Function implementing the queue01 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END startEjection */
void startEjection(void *argument)
{
	/* USER CODE BEGIN 5 */
	osDelay(1000);
	uint16_t real_altitude = 0;
	uint32_t currTick;
	float alt_filtered = 0;

#ifndef DEBUG_MODE
	// ---------- Get ground-level pressure and set as bias ----------
	for (uint16_t i = 0; i < ALT_MEAS_AVGING; i++){
		osMessageQueueGet(myQueue01Handle, &real_altitude, NULL, 0);
		alt_ground += real_altitude;
	}
	alt_ground = alt_ground/ALT_MEAS_AVGING; 			// Average of altitude readings

	// ---------- Waiting for launch ----------
	// TODO: Replace 150 by the actual value
	while(alt_filtered < 150){							// Waiting to launch
		currTick = xTaskGetTickCount();
		alt_filtered = runAltitudeMeasurements(currTick, real_altitude);
//      (print for test)
//		char msg[1000];
//	    sprintf(msg, "Filtered Alt =  %hu\n", alt_filtered);
//	    HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	    osDelay(50);
	}

	// ---------- Launched -> Wait for apogee ----------
	while (getAverageVelocity() > -DROGUE_DEPLOYMENT_VEL || alt_filtered < THRESHOLD_ALTITUDE) // while moving up and hasn't reached threshold altitude yet
	{
		currTick = xTaskGetTickCount();
		alt_filtered = runAltitudeMeasurements(currTick, real_altitude);
		osDelay(5);
	}

	// ---------- At apogee -> Deploy drogue ----------
	// TODO: Modify code to actually deploy both relays
	HAL_GPIO_WritePin(Relay_Drogue_GPIO_Port, Relay_Drogue_Pin, GPIO_PIN_SET);
	osDelay(DROGUE_DELAY);
	HAL_GPIO_WritePin(Relay_Drogue_GPIO_Port, Relay_Drogue_Pin, GPIO_PIN_RESET);

	// ---------- Wait for main deployment altitude ----------
	while (alt_filtered > MAIN_DEPLOYMENT){
		currTick = xTaskGetTickCount();
		alt_filtered = runAltitudeMeasurements(currTick, real_altitude);
		osDelay(5);
	}

	// ---------- At main deployment altitude -> Deploy main ----------
	// TODO: Modify code to actually deploy both relays
	HAL_GPIO_WritePin(Relay_Main_GPIO_Port, Relay_Main_Pin, GPIO_PIN_SET);
	osDelay(MAIN_DELAY);
	HAL_GPIO_WritePin(Relay_Main_GPIO_Port, Relay_Main_Pin, GPIO_PIN_RESET);

	// TODO: Test ejection code
	// TODO: terminate thread here

#endif

	/* Infinite loop */
	for(;;)
	{
#ifdef DEBUG_MODE
		// ---------- (Print to serial for debugging) ----------
		char msg[1000];
		osMessageQueueGet(myQueue01Handle, &real_altitude, NULL, 0);
		sprintf(msg, "Get altitude =  %hu\n", real_altitude);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
		osDelay(1000);
#endif
	}

	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ManageQueue02 */
/**
 * @brief Function implementing the queue02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END startEjection */
void startTelemetry(void *argument)
{
	/* USER CODE BEGIN startTelemetry */
	uint16_t real_altitude = 0;

	/* Infinite loop */

	for (;;) {

		osDelay(50);

		// ---------- Get sensor data ----------
//		get_acceleration(dev_ctx_lsm, acceleration);
//		get_angvelocity(dev_ctx_lsm, angular_rate);
		get_pressure(dev_ctx_lps, &pressure);
		get_temperature(dev_ctx_lps,  &temperature);
		real_altitude = 44330 * (1.0 - pow(pressure / LOCAL_PRESSURE, 0.190295)); // TODO: Verify if this is correct


		// ---------- Transmit data via radio ----------
#ifdef TRANSMIT_RADIO
		// TODO: Replace this with actual variables
		//	sprintf((char *)tx_buffer,"Temp:%d,Pressure:d,Altitude(BMP,m):%d,Pitch:%d,Roll:%d,Yaw:%d,Latitude:%d,Longitude:%d,Altitude(GPS,m):%d",
		//					temp, pressure,pitch, roll, yaw, latitude, longitude, gpsAltitude);

		sprintf((char *)tx_buffer, "Temperature:%d\tPressure:%d\n", temperature, pressure);
		HAL_UART_Transmit(&huart1, tx_buffer, TX_BUF_DIM, TX_BUF_DIM);
#endif

		// ---------- Put pressure data on message queue ----------
		osMessageQueuePut(myQueue01Handle, &real_altitude, 0, 100);


		// ---------- (Print to serial for debugging) ----------
#ifdef DEBUG_MODE
		 char msg[1000];
		 sprintf(msg, "Send altitude =  %hu\n", real_altitude);
		 HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
#endif
	}

	/* USER CODE END startTelemetry */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
