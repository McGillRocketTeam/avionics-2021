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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
	// Initialise SysTick to tick at 1ms by initialising it with SystemCoreClock (Hz)/1000

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

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		setup();
		loop();

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
///These are empty functions
///Original ones can be obtained from external libraries (I do not have access to them at the moment :( )
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
		printf("\r\nCould not find a valid BME280 sensor, check wiring!");
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
	printf("alt_meas = %d,  average_gradient = %d, alt_filtered = %d",alt_meas,average_gradient,alt_filtered);

	HAL_Delay(5);
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
