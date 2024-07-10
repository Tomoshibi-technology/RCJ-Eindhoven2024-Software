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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t rxBufA[2] = {};
uint8_t rxBufB[128] = {[0 ... 127] = 255};
uint8_t send_array[3] = {};

uint8_t p_wrtptB = 0;
uint8_t p_rdptB = 0;

uint8_t rx_check = 0;
uint8_t stop_counter = 0;
uint8_t error_counter = 0;

long totalAng = 0;
long ptotalAng = 0;
uint16_t firstAng;
uint16_t Angle;
uint16_t pAngle;

int16_t position = 0;
uint16_t send_position = 20000;

volatile uint32_t counter;
uint64_t u_counter;
uint16_t dtime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint8_t* stop_counter, uint8_t* error_counter);
uint8_t readID();
int readINDEX(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t readCounter()
{
  return (counter*65535) + TIM3->CNT;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3){
        counter++;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t AS5600_ADDR = 0x36 << 1;
  uint8_t ANGLE_ADDR = 0x0E;
  uint8_t ID;

  uint32_t d_pcounter, Ltika_pcounter, buf_pcounter;
  d_pcounter = Ltika_pcounter = readCounter();

  if(readID() < 2){
	  ID = readID();//自身のID
  }else{
	  while(1){
		  if(readCounter() - Ltika_pcounter > 10000){
		  	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	    	Ltika_pcounter = readCounter();
		  }else{}
	  }
  }

  HAL_UART_Receive_DMA(&huart2,rxBufB,128);

  HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDR, &ANGLE_ADDR, 1, 10);
  HAL_I2C_Master_Receive(&hi2c1, AS5600_ADDR, rxBufA, 2, 10);

  pAngle = rxBufA[0]*256 + rxBufA[1];
  ptotalAng = firstAng = pAngle;

  while (1)
  {
	u_counter = readCounter();

	HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDR, &ANGLE_ADDR, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, AS5600_ADDR, rxBufA, 2, 10);

	dtime = readCounter() - d_pcounter;
	d_pcounter = d_pcounter + dtime;

	Angle = rxBufA[0]*256 + rxBufA[1];

	if(Angle-pAngle>0 && abs(Angle-pAngle)<3000){
		totalAng = ptotalAng + (Angle-pAngle);
	}else if(Angle-pAngle<0 && abs(Angle-pAngle)<3000){
		totalAng = ptotalAng + (Angle-pAngle);
	}else if(Angle-pAngle>0 && abs(Angle-pAngle)>3000){
		totalAng = ptotalAng - ((4095-Angle)+pAngle);
	}else if(Angle-pAngle<0 && abs(Angle-pAngle)>3000){
		totalAng = ptotalAng + Angle + (4095-pAngle);
	}else{
		totalAng = ptotalAng;
	}

	ptotalAng = totalAng;
	pAngle = Angle;

	position = (totalAng - firstAng)*1000/25129;


	readBuf(&huart2, rxBufB, 128, &rx_check, ID, &p_wrtptB, &p_rdptB, &stop_counter, &error_counter);

	if(rx_check == 1){buf_pcounter = readCounter(); rx_check = 0;}

	if((u_counter - buf_pcounter) > 800){
		send_position = position + 20000;
		send_array[0] = 254;
		send_array[1] = send_position % 200;
		send_array[2] = send_position / 200;

		if(HAL_UART_Transmit(&huart2, send_array, 3, 1) == HAL_OK){
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
	}

//	if(rx_check == 1){
//		send_position = position + 20000;
//		send_array[0] = 254;
//		send_array[1] = send_position % 200;
//		send_array[2] = send_position / 200;
//
//		HAL_Delay(1);
//
//		if(HAL_UART_Transmit(&huart2, send_array, 3, 10) == HAL_OK){
//			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		}
//	}


//	if(readCounter() - Ltika_pcounter > 1000000){
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		Ltika_pcounter = readCounter();
//	}else{}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0090194B;
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
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin
                           SW5_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin
                          |SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint8_t* stop_counter, uint8_t* error_counter){
	int wrt_pt = uart->hdmarx->Instance->CNDTR;
	wrt_pt= buf_size - wrt_pt;
	int rd_pt;

	if(wrt_pt != *p_rdpt){//wrtに追い付かれてない
		if(buf[*p_rdpt] == 255){//p_rdptが書き換えられてない=追い越されてない
			if(wrt_pt != *p_wrtpt){//wrt_ptが進んだ=受信した
//正常
				rd_pt = *p_rdpt;
			}else{//wrt_ptが進んでない=受信してない
//受信してない
				(*stop_counter)++;
				rd_pt = *p_rdpt;
			}
		}else{//p_rdptが書き換えられた=追い越された
//追い越された
			rd_pt = wrt_pt - 40;
				if(rd_pt < 0){rd_pt += buf_size;}
			(*error_counter)++;
		}
	}else{//wrtに追い付かれた,追い付いた
		int front_pt = wrt_pt + 1;
			if(front_pt>buf_size-1){front_pt -= buf_size;}
		if(front_pt == 255){//追い付いた
			rd_pt = *p_rdpt;
		}else{//追い付かれた
			rd_pt = wrt_pt - 40;
				if(rd_pt < 0){rd_pt += buf_size;}
			(*error_counter)++;
		}
	}

	*data = 0;

	while(1){
		int dif_pt = wrt_pt - rd_pt;
			if(dif_pt < 0){dif_pt += buf_size;}
		if(dif_pt <= 20){break;}

		rd_pt++;
			if(rd_pt>buf_size-1){rd_pt -= buf_size;}

//		if(buf[rd_pt] == 250+id){
//			int goal_rdpt = rd_pt + data_size;//data_sizeに0はとれない,25以上もだめ
//				if(goal_rdpt>buf_size-1){goal_rdpt -= buf_size;}
//			int temp_rdpt = rd_pt;
//			buf[rd_pt] = 255;
//
//			for(int i=0; temp_rdpt==goal_rdpt; i++){
//				temp_rdpt += 1;
//					if(temp_rdpt>buf_size-1){temp_rdpt -= buf_size;}
//
//				data[i] = buf[temp_rdpt];
//				buf[temp_rdpt] = 255;
//			}
//
//			rd_pt = temp_rdpt;
//
//			dif_pt = wrt_pt - rd_pt;
//				if(dif_pt < 0){dif_pt += buf_size;}
//			if(dif_pt >= buf_size/2){}
//			else{break;}
//		}else{buf[rd_pt] = 255;}

		if(buf[rd_pt] == 248+id){
			*data = 1;
			buf[rd_pt] = 255;

			dif_pt = wrt_pt - rd_pt;
				if(dif_pt < 0){dif_pt += buf_size;}
			if(dif_pt >= buf_size/2){}
			else{break;}
			break;
		}else{buf[rd_pt] = 255;}
	}

	*p_rdpt = rd_pt;
//	*p_wrtpt = buf_size - (uart->hdmarx->Instance->CNDTR);
	*p_wrtpt = wrt_pt;
}

int readINDEX(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size){
	int index = uart->hdmarx->Instance->CNDTR;
	index = buf_size - index;
	return index;
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
