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
#include <stdbool.h>
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile uint32_t u10_counter;
//uint32_t callback_pcounter = 0;
//uint16_t callback_dtime;

uint8_t rxBufA[64]={[0 ... 63] = 255};
uint8_t rxDataA[2]={0,50};

uint8_t rxBufB[128]={[0 ... 127] = 255};
uint8_t rxDataB[2]={0,50};

uint16_t goal_speed= 5000;//目?��?
uint16_t now_speed = 5000;//現在速度
uint16_t dif_speed;//目標と現在の差

int16_t duty = 1600;
int16_t d_duty;//dutyの変化

uint16_t dtime;

uint8_t p_wrtptA = 0;
uint8_t p_wrtptB = 0;

uint8_t p_rdptA = 0;
uint8_t p_rdptB = 0;

uint16_t stop_counterA = 0;
uint16_t stop_counterB = 0;

uint16_t error_counterA = 0;
uint16_t error_counterB = 0;

int8_t stop_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint16_t* stop_counter, uint16_t* error_counter,  uint8_t go_back);
uint8_t readID();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3){
        u10_counter++;
    }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart1){
//		callback_dtime = u10_counter - callback_pcounter;
//		callback_pcounter = u10_counter;
//	}
//}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, 0);

  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t dutyCyc = 50;

  uint32_t Ltika_pcounter = u10_counter;
  uint32_t duty_pcounter = u10_counter;
  uint32_t d_pcounter = u10_counter;

  uint8_t ID;

  if(readID() < 4){
	  ID = readID();//自身のID
  }else{
	  while(1){
		  if(u10_counter - Ltika_pcounter > 10000){
		  	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	    	Ltika_pcounter = u10_counter;
		  }else{}
	  }
  }

  HAL_UART_Receive_DMA(&huart1,rxBufA,64);
  HAL_UART_Receive_DMA(&huart2,rxBufB,128);

  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, duty);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  dtime = u10_counter - d_pcounter;
	  d_pcounter = u10_counter;

//if value did not update, alert by LED and motor sound.
//対?��? against DMA did not start correctly
//bus=0つまりwhen MD does not communicate with Mother,STOP(起動直後�??��がたつきと通信線繋いでな?��?とき�??��動�?)

//speed算�??��
	  readBuf(&huart1, rxBufA, 64, rxDataA, 2, 0, &p_wrtptA, &p_rdptA, &stop_counterA, &error_counterA, 10);
	  readBuf(&huart2, rxBufB, 128, rxDataB, 2, ID, &p_wrtptB, &p_rdptB, &stop_counterB, &error_counterB, 40);

	  goal_speed = rxDataB[0] + rxDataB[1]*100;//試運転用にコメント化

	  now_speed = rxDataA[0] + rxDataA[1]*100;

	  dif_speed = abs(goal_speed - now_speed);
	  d_duty = dif_speed / 50;

//P制御
	  if((u10_counter - duty_pcounter) > dutyCyc){
		  if(goal_speed < now_speed){duty -= d_duty;}
		  else if(goal_speed > now_speed){duty += d_duty;}
		  else{}
		  duty_pcounter = u10_counter;
	  }else{}

//出力リミッ?��?
	  if(duty > 3120){duty = 3120;}
	  else if(duty < 80){duty = 80;}
	  else{duty = duty;}

//stop_flag
	  if(stop_counterA > 1000){stop_flag = 0;}
	  else if(stop_counterB > 1000){stop_flag = 0;}//試運転用にコメント化
	  else if(HAL_GPIO_ReadPin(SLSW_GPIO_Port, SLSW_Pin) != 1){stop_flag = 0;}
	  else if(goal_speed > 20200){stop_flag = 0;}//shutdown command detect
	  else if(u10_counter < 100000){stop_flag = 0;}
	  else{stop_flag = 1;}

//出力部
	  if(stop_flag == 1){
		  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, 1);
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 3200-duty);
	  }else{
		  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, 0);
		  duty = 1600;
		  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 1600);
	  }


//Lチカ部
	  if(u10_counter - Ltika_pcounter > 100000){
	  	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    	Ltika_pcounter = u10_counter;
	  }else{}

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 3199;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SLSW_Pin SW3_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SLSW_Pin|SW3_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW5_Pin SW4_Pin */
  GPIO_InitStruct.Pin = SW5_Pin|SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint16_t* stop_counter, uint16_t* error_counter, uint8_t go_back){
	int wrt_pt = uart->hdmarx->Instance->CNDTR;
	wrt_pt= buf_size - wrt_pt;
	int rd_pt;

	if(wrt_pt != *p_rdpt){//wrtに追?��?付かれてな?��?
		if(buf[*p_rdpt] == 255){//p_rdptが書き換えられてな?��?=追?��?越されてな?��?
			if(wrt_pt != *p_wrtpt){//wrt_ptが�??��んだ=受信した
//正常
				*stop_counter = 0;
				rd_pt = *p_rdpt;
			}else{//wrt_ptが�??��んでな?��?=受信してな?��?
//受信してな?��?
				(*stop_counter)++;
				rd_pt = *p_rdpt;
			}
		}else{//p_rdptが書き換えられた=追?��?越された
//追?��?越された
			(*error_counter)++;
			rd_pt = wrt_pt - go_back;
				if(rd_pt < 0){rd_pt += buf_size;}
		}
	}else{//wrtに追?��?付かれた,追?��?付い?��?
		int front_pt = wrt_pt + 1;
			if(front_pt>buf_size-1){front_pt -= buf_size;}

		if(buf[front_pt] == 255){
//追?��?付い?��?
			(*stop_counter)++;
			rd_pt = *p_rdpt;
		}else{
//追?��?付かれた
			(*error_counter)++;
			rd_pt = wrt_pt - go_back;
				if(rd_pt < 0){rd_pt += buf_size;}
		}
	}

	if(*stop_counter > 65500){*stop_counter = 65500;}
	if(*error_counter > 65500){*error_counter = 65500;}


	while(1){
		int dif_pt = wrt_pt - rd_pt;
			if(dif_pt < 0){dif_pt += buf_size;}
		if(dif_pt <= go_back/2){break;}

		rd_pt++;
			if(rd_pt>buf_size-1){rd_pt -= buf_size;}

		if(buf[rd_pt] == 250+id){
			int goal_rdpt = rd_pt + data_size;//data_sizeに0はとれな?��?,25以上も?��??��?

				if(goal_rdpt>buf_size-1){goal_rdpt -= buf_size;}
			int temp_rdpt = rd_pt;

			buf[rd_pt] = 255;

			for(int i=0; i<data_size; i++){
				temp_rdpt += 1;
					if(temp_rdpt>buf_size-1){temp_rdpt -= buf_size;}

				data[i] = buf[temp_rdpt];
				buf[temp_rdpt] = 255;
			}

			rd_pt = temp_rdpt;

			dif_pt = wrt_pt - rd_pt;
				if(dif_pt < 0){dif_pt += buf_size;}
			if(dif_pt >= buf_size/2){}
			else{break;}
		}else{buf[rd_pt] = 255;}
	}

	*p_rdpt = rd_pt;
//	*p_wrtpt = buf_size - (uart->hdmarx->Instance->CNDTR);
	*p_wrtpt = wrt_pt;
}

uint8_t readID(){
	uint8_t id;
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==1){id=0;}
	else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==1){id=1;}
	else if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)==1){id=2;}
	else if(HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin)==1){id=3;}
	else if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin)==1){id=4;}
	else{id=5;}
	return id;
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
