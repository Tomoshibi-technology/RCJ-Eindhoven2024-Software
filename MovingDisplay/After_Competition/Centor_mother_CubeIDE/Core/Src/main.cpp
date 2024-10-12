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
#include "joystick.h"

#include "SSD1306.h"
#include "fonts.h"
#include <stdio.h>

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t p_wrtptA=0;
uint8_t p_rdptA=0;
uint16_t stop_counterA=0;
uint16_t error_counterA=0;

uint8_t rxBuf[128];
uint8_t Data[4];

uint16_t adc_value[2];
int16_t stick[2];
int16_t theta=0;
uint8_t radius=0;

uint8_t mode=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint16_t* stop_counter, uint16_t* error_counter, uint8_t go_back);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1,rxBuf,128);
  JOYSTICK joystick(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	ssd1306_Init(&hi2c1);
	HAL_Delay(1000);
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen(&hi2c1);

	HAL_Delay(100);
	ssd1306_SetCursor(7,13);
	ssd1306_WriteString("Tomoshibi",Font_11x18,White);
	ssd1306_SetCursor(12,33);
	ssd1306_WriteString("Technology",Font_11x18,White);

	ssd1306_UpdateScreen(&hi2c1);

	HAL_Delay(1000);

  while (1)
  {
	readBuf(&huart1, rxBuf, 128, Data, 4, 0, &p_wrtptA, &p_rdptA, &stop_counterA, &error_counterA, 30);

	joystick.sampling();
	joystick.get_adcValue(adc_value);

	for(int n=0; n<2; n++){
		if(2000<adc_value[n] && adc_value[n]<2096){
			stick[n] = 0;
		}else{
			stick[n] = adc_value[n] - 2048;
		}
	}
	theta = (atan2(stick[1], stick[0])/3.1415*180.0+180.0) * 255/360;
	if(stick[0]==0 && stick[1]==0)theta=0;
	if(theta == 250)theta=251;
	radius = sqrt(stick[0]*stick[0] + stick[1]*stick[1]) * 250/2600;
	if(radius>255 || radius==250 )radius = 255;

	uint8_t vol = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_4);
	mode = vol<<2 | HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_15)<<1 | HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_13);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	char buf[32];
	ssd1306_Fill(Black);

	ssd1306_SetCursor(0,0);
    snprintf(buf, sizeof(buf), "t:%3d r:%3d", theta, radius);
	ssd1306_WriteString(buf,Font_11x18,White);

	ssd1306_SetCursor(0,20);
//    snprintf(buf, sizeof(buf), "vol:%d", vol);
//	ssd1306_WriteString(buf,Font_11x18,White);
	if(vol==0){
		ssd1306_WriteString("vol:Display",Font_11x18,White);
	}else{
		ssd1306_WriteString("vol:Arm",Font_11x18,White);
	}

	ssd1306_SetCursor(0,40);
    snprintf(buf, sizeof(buf), "mode:%d", mode);
	ssd1306_WriteString(buf,Font_11x18,White);

	ssd1306_UpdateScreen(&hi2c1);

	uint8_t send_array[5] = {250,Data[3],theta,radius,mode};//start, h, x, y, mode
	HAL_UART_Transmit(&huart5, send_array, 5, 1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint16_t* stop_counter, uint16_t* error_counter, uint8_t go_back){
	int wrt_pt = uart->hdmarx->Instance->NDTR;
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
