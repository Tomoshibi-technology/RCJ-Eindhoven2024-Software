/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint32_t m_counter;
int16_t MTRS[4]={0, 0, 0, 0};
uint8_t send_array[12] = {250, 0, 50, 251, 0, 50, 252, 0, 50, 253, 0, 50};

int16_t trgt_speed = 0;
int16_t trgt_degree = 0;

uint8_t rxData[3]={};
int16_t position;

uint16_t dtime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void speed_set(int gyro_degree, int goal_speed, int goal_degree, int16_t* mtrspeed, float motor_rate);
void set_array(int16_t* mtrspeed, uint8_t* sendarray);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2){
        m_counter++;
    }
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
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Transmit(&huart6, send_array, 12, 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//    uint32_t Ltika_pcounter = m_counter;
    uint32_t set_pcounter = m_counter;
    uint32_t Tx_pcounter = m_counter;
    uint32_t d_pcounter = m_counter;

    uint8_t Odo_ID = 248;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  dtime = m_counter - d_pcounter;
	  d_pcounter = m_counter;

	  if(m_counter - set_pcounter > 10){
		  set_pcounter = m_counter;
		  trgt_degree = trgt_degree + 1;

		  if(trgt_speed < 30){trgt_speed = trgt_speed + 10;}
		  else{trgt_speed = 500;}

		  if(HAL_GPIO_ReadPin(STRTSW_GPIO_Port, STRTSW_Pin) == 0){
			  trgt_degree = 0;
			  trgt_speed = 0;
		  }
	  }else{}

	  speed_set(27, trgt_speed, trgt_degree, MTRS, 0.7);
	  set_array(MTRS, send_array);

	  if(m_counter - Tx_pcounter > 10){
		  Tx_pcounter = m_counter;
		  if(HAL_GPIO_ReadPin(STRTSW_GPIO_Port, STRTSW_Pin) == 1){
			  HAL_UART_Transmit(&huart6, send_array, 12, 10);
		  }else{
				for(int i=0; i<4; i++){
				  send_array[3*i] = 250 + i;
				  send_array[3*i + 1] = 0;
				  send_array[3*i + 2] = 50;
				}
				HAL_UART_Transmit(&huart6, send_array, 12, 10);
		  }
		  HAL_UART_Transmit(&huart6, &Odo_ID, 1, 10);
		  if(HAL_UART_Receive(&huart6, rxData, 3, 10) == HAL_OK){
			  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  }

		  position = rxData[1] + rxData[2]*200 - 20000;
	  }else{}


//	  if(m_counter - Ltika_pcounter > 1000){
//	  	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	  	Ltika_pcounter = m_counter;
//	  }else{}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STRTSW_Pin */
  GPIO_InitStruct.Pin = STRTSW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STRTSW_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void speed_set(int gyro_degree, int goal_speed, int goal_degree, int16_t* mtrspeed, float motor_rate){
	goal_degree = goal_degree % 360;
	if(goal_degree < 0){goal_degree += 360;}

//    int roll_speed;
//	gyro_degree = gyro_degree % 360;
//	if(gyro_degree < 0){gyro_degree += 360;}

    int roll_speed;
    if(gyro_degree > 180){gyro_degree -= 360;}
    else if(gyro_degree <-180){gyro_degree += 360;}
    else{}


    if (gyro_degree > 0){
        roll_speed = -10 + (-gyro_degree * 3);
        if (gyro_degree < 6){
            roll_speed = 0;
        }
        if (roll_speed < -150){
            roll_speed = -150;
        }
    }else if (gyro_degree < 0){
        roll_speed = 10 + (-gyro_degree * 3);
        if (gyro_degree > -6){
            roll_speed = 0;
        }
        if (roll_speed > 150){
            roll_speed = 150;
        }
    }else{
        roll_speed = 0;
    }


	int conv_degree = -goal_degree + 45;
	if(conv_degree < 0){conv_degree = conv_degree + 360;}

	for(int i=0; i<4; i++){
		mtrspeed[i] = goal_speed * sin((conv_degree + 90.0*i) / 180.0 * 3.1415);
		mtrspeed[i] = (mtrspeed[i] * motor_rate) + (roll_speed * (1 - motor_rate));
	}
}

void set_array(int16_t* mtrspeed, uint8_t* sendarray){
	uint16_t conv_mtrspeed[4];
	for(int i=0; i<4; i++){conv_mtrspeed[i] = mtrspeed[i] + 5000;}
	for(int i=0; i<4; i++){
		sendarray[3*i] = 250+i;
		sendarray[3*i+1] = conv_mtrspeed[i] % 100;
		sendarray[3*i+2] = conv_mtrspeed[i] / 100;
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
