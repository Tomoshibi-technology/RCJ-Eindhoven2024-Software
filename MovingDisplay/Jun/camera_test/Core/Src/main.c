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
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t rxBuf[128]={};
uint8_t rxColor[3]={0,0};

int travel_x;
int travel_y;
uint8_t send_array[11];

int hue = 0;
int hue_back = 127;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, int go_back);
int readINDEX(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_array(int tx, int cx, int cz, uint8_t r, uint8_t h, uint8_t h2, uint8_t v){
	send_array[0]=220;

	if(cx - tx > 48+r){
		cx = cx -((48*2) - (r*2)+1);
	}else if(cx - tx < -(48-r)){
		cx = cx + ((48*2) + (r*2)-1);
	}

//	c_x = cx;

	tx += 5000; cx += 5000; cz += 5000;
	uint8_t h_out = h/2.5;
	uint8_t h_out2 = h2/2.5;
	uint8_t v_out = v/2.5;
	for(int i = 1; i <3 ;i++){
		send_array[i] = tx%100;
		tx = (int)tx/100;
	}
	for(int i = 3; i <5 ;i++){
		send_array[i] = cx%100;
		cx = (int)cx/100;
	}
	for(int i = 5; i <7 ;i++){
		send_array[i] = cz%100;
		cz = (int)cz/100;
	}
	send_array[7] = r;
	if(h_out > 100){h_out = 100;}
	send_array[8] = h_out;
	if(h_out2 > 100){h_out2 = 100;}
	send_array[9] = h_out2;
	if(v_out > 100){v_out = 100;}
	send_array[10] = v_out;
}

uint8_t rgbToHue(int r, int g, int b) {
    // RGB値を0-15から0-1に正規化
    float rf = r / 15.0;
    float gf = g / 15.0;
    float bf = b / 15.0;

    // 最大値、最小値、および範囲を求める
    float max_val = rf;
    if (gf > max_val) max_val = gf;
    if (bf > max_val) max_val = bf;

    float min_val = rf;
    if (gf < min_val) min_val = gf;
    if (bf < min_val) min_val = bf;

    float delta = max_val - min_val;

    float h = 0.0;

    // 色相（H）の計算
    if (delta == 0) {
        h = 0; // 無彩色（グレースケール）
    } else {
        if (max_val == rf) {
            h = 60 * ((gf - bf) / delta);
            if (h < 0) h += 360; // Hが負の場合360を加算
        } else if (max_val == gf) {
            h = 60 * ((bf - rf) / delta + 2);
        } else if (max_val == bf) {
            h = 60 * ((rf - gf) / delta + 4);
        }
    }

    // Hを0-255の範囲にスケール変換
    h = (h / 360) * 255;

    return h;
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart2,rxBuf,128);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(CAMERA_POWER_GPIO_Port, CAMERA_POWER_Pin, GPIO_PIN_SET);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		readBuf(&huart2, rxBuf, 128, rxColor, 3, 10);
		HAL_Delay(1000);

//		travel_x = -1*odom1.get_travel() - xf;
//		travel_y = -1*odom2.get_travel() - yf;

		travel_x = 31;

		if( (rxColor[0]==15 && rxColor[1]==15) && rxColor[2]==15){
			hue = hue_back;
		}else{
			hue = rgbToHue(rxColor[0],rxColor[1],rxColor[2]);
			hue_back = (hue + 90)%250;
		}

		float size = 12.0+((-1.0)*travel_y*0.1);
		set_array((-1)*travel_x, (-1)*126, 24, (int)size, hue, hue_back, 21);
		HAL_UART_Transmit(&huart3,(uint8_t*)&send_array, 11, 100);
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERA_POWER_GPIO_Port, CAMERA_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAMERA_POWER_Pin */
  GPIO_InitStruct.Pin = CAMERA_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAMERA_POWER_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, int go_back){
	int wrt_pt = buf_size - uart->hdmarx->Instance->NDTR;
	int rd_pt = wrt_pt - go_back;
	if(rd_pt < 0){
		rd_pt += buf_size;
	}

	while(1){
		uint8_t readData = buf[rd_pt];
//		buf[rd_pt] = 255; //読み込み済みのしるし
		if(readData == 250){ //rd_ptがスタートバイトの位置になる
			for(int i=1; i<=data_size; i++){ //rd_pt+i が今回よむポインタになるように
				if(rd_pt + i > buf_size - 1){ //回りきっていないか確認
					rd_pt = rd_pt - buf_size;
				}
				data[i-1] = buf[rd_pt + i]; //返すデータに入れる
//				buf[rd_pt+i] = 255; //読み込み済みのしるし。
			}
			break;
		}

		//次のための処理
		rd_pt++;
		if(rd_pt == buf_size){ //buf_sizeは配列の一番最後の次
			rd_pt = rd_pt - buf_size;
		}
		if(rd_pt == wrt_pt){//スタートバイトに出会えず書き込みポインタまできたら終わり
//			for(int i=0; i<data_size; i++){
//				data[i] = 255; //無かったことを表す数字
//			}
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
