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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <math.h>

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
UART_HandleTypeDef huart2;

/* Definitions for ejection */
osThreadId_t ejectionHandle;
const osThreadAttr_t ejection_attributes = {
		.name = "ejection",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 128 * 4
};
/* Definitions for telemetry */
osThreadId_t telemetryHandle;
const osThreadAttr_t telemetry_attributes = {
		.name = "telemetry",
		.priority = (osPriority_t) osPriorityLow,
		.stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
// Control variables
float local_pressure = 101200.0; //hPa (sea level)
float threshold_altitude = 10000; //feet
float main_deployment = 1500; //feet
long drogue_delay = 500; // milliseconds. time that drogue is HIGH
long main_delay = 500; // milliseconds. time that main is HIGH

// Configuration variables
float freq = 3750; // buzzer sound frequency.
float a = 1; // Cut-off frequency of low-pass filter in Hz.
#define num_meas 10 // Number of historic measurements for averaging.

// Programming variables
float alt_meas;
float ground_alt = 0.0;
float T; // sampling period, time of each loop
float alt_previous[num_meas];
float alt_filtered;
float t_previous_loop, t_previous_buzz;
float average_gradient;
int apogee_reached = 0;
int launched = 0;
int main_deployed = 0;

// Relay output checks
int R11; // drogue1
int R12; // drogue2
int R21; // main1
int R22; // main2

long time11;
long time12;
long time21;
long time22;

uint16_t size;
uint8_t Data[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartEjection(void *argument);
void StartTelemetry(void *argument);

/* USER CODE BEGIN PFP */


int XTENDSerial_Begin(int x);
int bmp_Begin(void);
int bno_Begin(void);
int GPS_Begin(void);
int SD_Begin(int x);
int RTC_Begin(void);
int RTC_initialized(void);

int SD_Exist(char *name);
void SD_remove(char *name);
FILE * SD_Open(char *name, char *mode);
void RTC_Adjust(/*Date time*/);

char * getDateTime(void);

float bno_getAccelerometer_getX();
float bno_getAccelerometer_getY();
float bno_getAccelerometer_getZ();

float bno_getEuler_getX();
float bno_getEuler_getY();
float bno_getEuler_getZ();

float bmp_readTemperature();
float bmp_readPressure();

float GPS_getLatitude();
float GPS_getLongitude();
float GPS_getAltitude();

void XTENDSerial_print(char * str);


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
	/* USER CODE BEGIN 2 */

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

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of ejection */
	ejectionHandle = osThreadNew(StartEjection, NULL, &ejection_attributes);

	/* creation of telemetry */
	telemetryHandle = osThreadNew(StartTelemetry, NULL, &telemetry_attributes);

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
	while (1)
	{
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
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
	HAL_GPIO_WritePin(GPIOA, buzzer_Pin|LED_Pin|main1_Pin|main2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, drogue1_Pin|drogue2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : buzzer_Pin LED_Pin main1_Pin main2_Pin */
	GPIO_InitStruct.Pin = buzzer_Pin|LED_Pin|main1_Pin|main2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : drogue1_Pin drogue2_Pin */
	GPIO_InitStruct.Pin = drogue1_Pin|drogue2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : in22_Pin in21_Pin in12_Pin in11_Pin */
	GPIO_InitStruct.Pin = in22_Pin|in21_Pin|in12_Pin|in11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int begin(void){
	return 1;
}

float readAltitude(float local_pressure){
	return local_pressure;
}




///Empty functions end here
#define false 0
#define true 1

void setup() {
	HAL_GPIO_WritePin(drogue1_GPIO_Port, drogue1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(drogue2_GPIO_Port, drogue2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(main1_GPIO_Port, main1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(main2_GPIO_Port, main2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	if (!begin()){
		///printf("\r\nCould not find a valid BME280 sensor, check wiring!");
		size = sprintf((char *)Data, "Could not find a valid BME280 sensor, check wiring\n");
		HAL_UART_Transmit(&huart2, Data, size, 1000);
		HAL_Delay(2000);

		HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

		HAL_Delay(5000);
		HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
		while (1);
	} else {
		int i;
		for (i = 0 ; i < 3; i++){
			HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
		}
	}

	for (int i = 0; i < 500; i++){
		ground_alt += readAltitude(local_pressure/100.0); //takes sea-level pressure and reads alt 500 times
	}
	ground_alt = ground_alt/500.0; //average of alt readings
	a = 2*3.14159*a;
	size = sprintf((char *)Data, "Ejection Initialized\n");
	HAL_UART_Transmit(&huart2, Data, size, 1000);
}

void loop() {
	if (main_deployed == false){
		T = (HAL_GetTick() - t_previous_loop)/1000; //millis() = time since program start running T running time of curr loop (s)
		t_previous_loop = HAL_GetTick(); //total time

		alt_meas = (readAltitude(local_pressure/100) - ground_alt)*3.28084; //Measures AGL altitude in feet

		// Low-pass filter - rocket at high speeds pressure fluctuates and affects altitude reading, usaully at a high frequency, so low pass filter filters those high freuqency changes out
		//and keeps just the overall, low frequency changes (caused by altitude change)
		alt_filtered = (1 - T * a) * alt_previous[num_meas-1] + a * T * alt_meas;

		// Slide window of 10 measurement history.
		for (int i = 0; i < num_meas-1; i++){
			alt_previous[i] = alt_previous[i+1];
		}
		alt_previous[num_meas-1] = alt_filtered;

		// Launch Detection
		if (alt_filtered > 150 && launched == false){
			launched = true;
		}

		//Average gradient of 10 past measurements.
		average_gradient = 0;
		for (int i = 0; i < num_meas-1; i++){
			average_gradient += (alt_previous[i+1]- alt_previous[i]);
		}
		if (T>0){
			average_gradient /= (num_meas);
		}

		// Apogee detection
		if (alt_filtered > threshold_altitude && launched && apogee_reached == false){

			if (average_gradient < -2){ //what is the purpose of this -2?
				apogee_reached = true;
				HAL_GPIO_WritePin(drogue1_GPIO_Port, drogue1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(drogue2_GPIO_Port, drogue2_Pin, GPIO_PIN_SET);
				HAL_Delay(drogue_delay);

				/* This part is commented out because the original arduino code commented this part out as well
				R11 = HAL_GIPO_ReadPin(in11_GPIO_Port, in11_Pin);
				R12 = HAL_GIPO_ReadPin(in12_GPIO_Port, in12_Pin);
				if(R11){
					time11 = HAL_GetTick();
				}if(R12){
					time12 = HAL_GetTick();
				}
				 */


				HAL_GPIO_WritePin(drogue1_GPIO_Port, drogue1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(drogue2_GPIO_Port, drogue2_Pin, GPIO_PIN_RESET);
			}
		}

		// Main Deployment detection
		if (apogee_reached && alt_filtered < main_deployment && main_deployed == false){
			main_deployed = true;
			HAL_GPIO_WritePin(main1_GPIO_Port, main1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(main2_GPIO_Port, main2_Pin, GPIO_PIN_SET);
			HAL_Delay(main_delay);

			/* This part is commented out because the original arduino code commented this part out as well
			R21 = HAL_GIPO_ReadPin(in21_GPIO_Port, in21_Pin);
			R22 = HAL_GIPO_ReadPin(in22_GPIO_Port, in22_Pin);
			if(R21){
				time21 = HAL_GetTick();
			}if(R22){
				time22 = HAL_GetTick();
			}
			 */

			HAL_GPIO_WritePin(main1_GPIO_Port, main1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(main2_GPIO_Port, main2_Pin, GPIO_PIN_RESET);
		}



	}
	else{ //Longer buzz to indicate program completion

		HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);
		HAL_Delay(1000);

	}
	//Just for debuggging.
	// printf("alt_meas = %d,  average_gradient = %d, alt_filtered = %d",alt_meas,average_gradient,alt_filtered);

	size = sprintf((char *)Data,"alt_meas: %.2d , average_gradient: %.2d , alt_filtered: %.2d\n", alt_meas, average_gradient, alt_filtered);
	HAL_UART_Transmit(&huart2, Data, size, 1000);

	HAL_Delay(5);
}

int XTENDSerial_Begin(int x){
	return x;
}
int bmp_Begin(void){
	return true;
}
int bno_Begin(void)
{
	return true;
}
int myGPS_Begin(void)
{
	return true;
}
int SD_Begin(int x){
	return x;
}
int RTC_Begin(void){
	return true;
}
int RTC_initialized(void){
	return true;
}

int SD_Exist(char *name){
	return false;
}
void SD_remove(char *name){
	return;
}
FILE * SD_Open(char *name, char *mode){
	return NULL;
}
void RTC_Adjust(/*Date time*/){
	return;
}

char * getDateTime(void){
	return "date_time";
}

float bno_getAccelerometer_getX(){
	return 10;
}
float bno_getAccelerometer_getY(){
	return 10;
}
float bno_getAccelerometer_getZ(){
	return 10;
}

float bno_getEuler_getX(){
	return 10;
}
float bno_getEuler_getY(){
	return 10;
}
float bno_getEuler_getZ(){
	return 10;
}

float bmp_readTemperature(){
	return 10;
}
float bmp_readPressure(){
	return 10;
}

float GPS_getLatitude(){
	return 10;
}
float GPS_getLongitude(){
	return 10;
}
float GPS_getAltitude(){
	return 10;
}

void XTENDSerial_print(char * str){
	return;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartEjection */
/**
 * @brief  Function implementing the ejection thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartEjection */
void StartEjection(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	setup();

	for(;;)
	{
		loop();
		osDelay(500);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTelemetry */
/**
 * @brief Function implementing the telemetry thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTelemetry */
void StartTelemetry(void *argument)
{
	/* USER CODE BEGIN StartTelemetry */
	/* Infinite loop */

	//TODO Initialise serial (UART with specific baud rate and Wire.begin which idk)

	char msg[500];

	//TODO sensor initialisation
	//struct FILE *myFile;
	float temp, pressure, real_altitude, accel_x, accel_y, accel_z, pitch, roll, yaw;
	int seaLevelhPa = 102540;



	// Check BMP
	if (!bmp_Begin()) {
		sprintf(msg, "BMP280 initialization failed!\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
		while (1);
	}

	// Check BNO
	if (!bno_Begin())
	{
		sprintf(msg, "BNO055 initialization failed!\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
		while (1);
	}

	// Check GPS
	if (myGPS_Begin() == 0) //Connect to the Ublox module using Wire port
	{
		sprintf(msg, "GPS initialization failed!\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
		while (1);
	}

	// Check SD Card
	if (!SD_Begin(10)) {
		sprintf(msg, "SD card initialization failed!\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
		return;
	}

	// Check RTC
	if (! RTC_Begin()) {
		sprintf(msg, "Couldn't find RTC\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);
		while (1);
	}

	if(SD_Exist("example.txt")){
		SD_remove("example.txt");
	}

	//myFile = SD_Open("example.txt", "wt");
	//fclose(myFile);

	if (! RTC_initialized()) {
		sprintf(msg, "RTC is NOT running!");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		// following line sets the RTC to the date & time this sketch was compiled
		// RTC_Adjust(/*TODO date and time format*/);
		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	}

	sprintf(msg, "Telemetry Initialized\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);


	/*TODO
	 * bno, bmp and GPS configuration (will do after we figure out the drivers)
	 */

	for (;;){

		accel_x = bno_getAccelerometer_getX();
		accel_y = bno_getAccelerometer_getY();
		accel_z = bno_getAccelerometer_getZ();

		yaw = bno_getEuler_getX();
		pitch = bno_getEuler_getY();
		roll = bno_getEuler_getZ();

		temp = bmp_readTemperature();
		pressure = bmp_readPressure();

		real_altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.190295));

		long latitude = GPS_getLatitude();
		latitude /= pow(10,7);

		long longitude = GPS_getLongitude();
		longitude /= pow(10,7);

		long gpsAltitude = GPS_getAltitude();
		gpsAltitude /= pow(10,3);

		size = sprintf((char *)Data,"Temp:%d,Pressure:d,Altitude(BMP,m):%d,Pitch:%d,Roll:%d,Yaw:%d,Latitude:%d,Longitude:%d,Altitude(GPS,m):%d",
				temp, pressure,pitch, roll, yaw, latitude, longitude, gpsAltitude);
		HAL_UART_Transmit(&huart2, Data, size, 1000);
		//getDateTime() not working?

		//myFile = SD_Open("example.txt", "wt");
		//fputs(msg, myFile);
		//fclose(myFile);
		XTENDSerial_print(msg);


		osDelay(500);
	}
	//free(myFile);
	/* USER CODE END StartTelemetry */
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
