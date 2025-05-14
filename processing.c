/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t RX_Buffer[2] = {0};
uint16_t ADC_output = 0;
uint16_t ADC_filter_buffer[3] = {0}; //store moving avg value
char string_1[40] = "\0";
uint8_t buffer_index = 0;
uint8_t initialized = 0;
static uint8_t sample_counter = 0;
int k = 2; 		// used to adjust the sensitivity of outlier detection
//int flag = 0;
//float distance = 0;
//uint16_t count_1 = 0;
//uint16_t count_2 = 0;
//int min_distance = 10;
//uint8_t uart_buffer[9];
//uint8_t recording_enabled = 1;  // 1 = ON, 0 = OFF
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void delay_uS(uint16_t delay);
void HCSR04_Read();

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_SPI_Receive_IT(&hspi1,RX_Buffer,2);
//  HAL_TIM_Base_Start(&htim6);		// Timer to count to 10 microseconds
//  HAL_TIM_Base_Start_IT(&htim7);  //Timer to trigger ultrasonic, always put base start above
//  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);		// Timer to measure distance
//  HAL_UART_Receive_IT(&huart2,(uint8_t*)uart_buffer,9);  // Receive 1 byte at a time
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 230400;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi){
	if (hspi == &hspi1){
		sample_counter++; // increase number of sample received
		if (sample_counter % 2 == 0)	// only process every second sample
		{
			//convert 2 bytes back into 12 bit ADC value
			uint16_t new_value = RX_Buffer[0] |	(RX_Buffer[1]<<8);
			// getting the current average of the collected  sample(s)
			int sum = 0;
			for (int i = 0; i < initialized; i++)
			{
				sum += ADC_filter_buffer[i];
			}
			uint16_t cur_avg = 0;
			if (initialized > 0)
			{
				cur_avg = sum / initialized;
			}
			else
			{
				cur_avg = new_value;
			}
			// calculate the variance
			int variance_sum = 0;
			for (int j = 0; j < initialized; j++)
			{
				int difference = ADC_filter_buffer[j] - cur_avg;
				variance_sum += (difference*difference);
			}

			int variance = 0;
			if (initialized > 0)
			{
				variance = variance_sum/initialized;
			}
			else
			{
				variance = 0;
			}

			// defining threshold
			int threshold = k*k*variance;

			// check whether new_value is outlier
			int diff = (int) new_value - (int) cur_avg;
			if (diff*diff > threshold)
			{
				new_value = cur_avg;
			}

			//store the ADC value into a buffer to avg out
			ADC_filter_buffer[buffer_index] = new_value;
			//update filter buffer index, % to limit index to 2, will go to 0 after 2
			buffer_index = (buffer_index+1)%3;

			// this basically ensures that the average filter works smoothly with the first and second inputs
			if(initialized < 3){
				initialized ++;
			}

			sum = 0;
			for(int i = 0; i<initialized;i++){
				sum += ADC_filter_buffer[i];
			}

			uint16_t filtered_value = sum/initialized;
			uint8_t reduced_8bit = filtered_value >> 4;
			HAL_UART_Transmit_IT(&huart2, &reduced_8bit,1);

			HAL_SPI_Receive_IT(&hspi1,RX_Buffer,2); //receive through SPI in interrupt
		}
	}
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART2)
//    {
//        // Example: toggle recording if 'r' is received
//    	if (strncmp(uart_buffer, "interrupt", 9) == 0) {
//    		recording_enabled = 0;
//    	}
//    	else if (strncmp(uart_buffer, "resume   ", 9) == 0) {
//    		recording_enabled = 1;
//    	}
//
//    	        // Re-enable UART interrupt
//    	HAL_UART_Receive_IT(&huart2, uart_buffer, 9);
//    }
//}
//
//void delay_uS(uint16_t delay)
//{
//	__HAL_TIM_SET_COUNTER(&htim6 , 0);
//	while (__HAL_TIM_GET_COUNTER(&htim6) < delay)
//	{
//
//	}
//}
//
//void HCSR04_Read()
//{
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_SET);
//	delay_uS(10);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
//}
//
//void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
//{
//	if (htim == &htim7)
//	{
//		  HCSR04_Read();
//	}
//}
//
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	if ((htim == &htim1) && (htim->Channel == 1))
//	{
//		if (flag == 0)
//		{
//			count_1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
//			flag = 1;
//		}
//		else
//		{
//			count_2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
//
//			__HAL_TIM_SET_COUNTER(htim,0);
//			distance = (count_2-count_1)/58.0;
//			flag = 0;
//			if(distance > 0){
//				HAL_UART_Transmit(&huart2, (uint8_t*)string_1, strlen(string_1),10);
//				if (distance <= min_distance && recording_enabled==1){
//					HAL_GPIO_WritePin (GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
//				}
//				else{
//					HAL_GPIO_WritePin (GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
//				}
//			}
//
//		}
//	}
//}
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
