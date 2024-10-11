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
#include "ws2812.h"
#include "led.h"
#include "sdma_transmit.h"

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
DMA_HandleTypeDef hdma_tim3_ch2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
WS2812 Neopixel(&htim3, TIM_CHANNEL_2, &hdma_tim3_ch2);
LED led(&Neopixel);


uint8_t p_wrtptA=0;
uint8_t p_rdptA=0;
uint16_t stop_counterA=0;
uint16_t error_counterA=0;

uint8_t rxBuf[64];
uint8_t Data[12];

int8_t myid = 0;

int16_t circle_x = 0;
int16_t circle_z = 0;
uint8_t circle_r = 0;
uint8_t circle_h = 0;
uint8_t circle_s = 255;
uint8_t circle_v = 50;
uint8_t back_h = 0;
uint8_t back_s = 255;
uint8_t back_v = 50;
uint8_t square_h = 0;
uint8_t square_s = 255;
uint8_t square_v = 50;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint16_t* stop_counter, uint16_t* error_counter, uint8_t go_back);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
	Neopixel.do_forwardRewrite();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	Neopixel.do_backRewrite();
}

uint8_t readID(){
	uint8_t ID = 0;
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)==1){
		ID+=1;
	}
	if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)==1){
		ID+=2;
	}
	if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin)==1){
		ID+=4;
	}
	//養生5番はここを消す。ハード故障。
//	if(HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin)==1){
//		ID+=8;
//	}
	if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin)==1){
		ID+=16;
	}
	return ID;
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
  for(uint8_t i=0; i<64; i++){
	  rxBuf[i] = 255;
  }

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
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart2, rxBuf, 64);
  HAL_Delay(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	readBuf(&huart2, rxBuf, 64, Data, 12, 0, &p_wrtptA, &p_rdptA, &stop_counterA, &error_counterA, 30);
	myid = readID();

	circle_x = Data[0]-100;
	circle_z = Data[1]-100;
	circle_r = Data[2];
	circle_h = Data[3];
	circle_s = Data[4];
	circle_v = Data[5];
	back_h = Data[6];
	back_s = Data[7];
	back_v = Data[8];
	square_h = Data[9];
	square_s = Data[10];
	square_v = Data[11];

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Neopixel.clear();


	  //----------------------------------------------
	  uint16_t PANEL_START_X = 0;
	  uint16_t PANEL_START_Z = 0;

	  if	 (myid==0){PANEL_START_X = 0; PANEL_START_Z = 0;}
	  else if(myid==1){PANEL_START_X = 0; PANEL_START_Z = 16;}
	  else if(myid==2){PANEL_START_X = 0; PANEL_START_Z = 32;}

	  else if(myid==3){PANEL_START_X = 16; PANEL_START_Z = 0;}
	  else if(myid==4){PANEL_START_X = 16; PANEL_START_Z = 16;}
	  else if(myid==5){PANEL_START_X = 16; PANEL_START_Z = 32;}

	  else if(myid==6){PANEL_START_X = 32; PANEL_START_Z = 0;}
	  else if(myid==7){PANEL_START_X = 32; PANEL_START_Z = 16;}
	  else if(myid==8){PANEL_START_X = 32; PANEL_START_Z = 32;}

//	  void LED::show(int travel_x, int circle_x, int circle_z, int circle_r, int hue, int hue_of_back){
//	      NEOPIXEL->clear();
	  for(int px=0; px<16; px++){
		  for(int pz=0; pz<16; pz++){
			  int x = px + PANEL_START_X;
			  int z = pz + PANEL_START_Z;

			  //BACK_GROUND
			  int hue=back_h;
			  int sat=back_s;
			  int val=back_v;

			  //square
			  if((x==0 || x==1)||(x==46 || x==47)){
					hue = square_h; sat = square_s; val = square_v;
			  }
			  if((z==0 || z==1)||(z==46 || z==47)){
			  		hue = square_h; sat = square_s; val = square_v;
			  }

			  //CIRCLE
			  int8_t cx = 47-circle_x;
			  int8_t cz = circle_z;
			  uint8_t cr = circle_r;
			  uint8_t myx = x;
			  uint8_t myz = z;
			  float distance = (myx-cx)*(myx-cx)+(myz-cz)*(myz-cz);
			  if(cr*cr>=distance){
				hue = circle_h; sat = circle_s; val = circle_v;
			  }

			  //SET
			  uint16_t pixel_num = 0;
			  if(pz%2 == 0){
				   pixel_num = pz*16 + px;
			  }else{
				  pixel_num = pz*16 + 15 - px;
			  }
			  Neopixel.set_hsv(pixel_num, hue, sat, val);
		  }
	  }
	  Neopixel.show();






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
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 15-1;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW3_Pin SW4_Pin SW5_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW3_Pin|SW4_Pin|SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint16_t* stop_counter, uint16_t* error_counter, uint8_t go_back){
	int wrt_pt = uart->hdmarx->Instance->CNDTR;
	wrt_pt= buf_size - wrt_pt;
	int rd_pt;

	if(wrt_pt != *p_rdpt){//wrtに追??��?��?付かれてな??��?��?
		if(buf[*p_rdpt] == 255){//p_rdptが書き換えられてな??��?��?=追??��?��?越されてな??��?��?
			if(wrt_pt != *p_wrtpt){//wrt_ptが�???��?��んだ=受信した
//正常
				*stop_counter = 0;
				rd_pt = *p_rdpt;
			}else{//wrt_ptが�???��?��んでな??��?��?=受信してな??��?��?
//受信してな??��?��?
				(*stop_counter)++;
				rd_pt = *p_rdpt;
			}
		}else{//p_rdptが書き換えられた=追??��?��?越された
//追??��?��?越された
			(*error_counter)++;
			rd_pt = wrt_pt - go_back;
				if(rd_pt < 0){rd_pt += buf_size;}
		}
	}else{//wrtに追??��?��?付かれた,追??��?��?付い??��?��?
		int front_pt = wrt_pt + 1;
			if(front_pt>buf_size-1){front_pt -= buf_size;}

		if(buf[front_pt] == 255){
//追??��?��?付い??��?��?
			(*stop_counter)++;
			rd_pt = *p_rdpt;
		}else{
//追??��?��?付かれた
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
			int goal_rdpt = rd_pt + data_size;//data_sizeに0はとれな??��?��?,25以上も??��?��???��?��?

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
