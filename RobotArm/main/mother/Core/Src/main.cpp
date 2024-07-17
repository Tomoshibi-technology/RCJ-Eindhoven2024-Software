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
#include "ws2812c.h"
#include "calc.h"
#include "STS.h"
#include "BNO055.hpp"










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
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
WS2812C NeopixelTape(&htim3, TIM_CHANNEL_1, &hdma_tim3_ch1_trig);
CALC calc;
STS servo0(&huart2, 0);
STS servo1(&huart2, 1);
STS servo2(&huart2, 2);
STS servo3(&huart2, 3);

int16_t servoPos0 = 0;
int16_t servoPos1 = 0;
int16_t servoPos2 = 0;
int16_t servoPos3 = 0;
uint8_t rxBuf[128];
uint8_t tweliteRxBuf[128];

int16_t ledPos0 = 0;
int16_t ledPos1 = 0;
int16_t ledPos2 = 0;

uint8_t sendArray[8] = {255, 255, 0, 0, 0, 0, 0, 0};
uint16_t degree = 0;

uint8_t ready = 0;
int16_t gyro;

uint8_t ID = 0;

uint32_t m = 0;

uint8_t mode = 0;
uint16_t count = 0;
uint8_t hue = 0;

static int16_t i = 8;
static int16_t moveRotation = 0;



uint8_t tweliteRead[9] = {0};









/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void get_position(uint8_t servoID);
void sendData(uint16_t angle, uint8_t speed, int16_t rotation);
void twelite();










/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)
  {
    NeopixelTape.execute();
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  NeopixelTape.init();

  if(HAL_GPIO_ReadPin(dipsw1_GPIO_Port, dipsw1_Pin) == 1){
	  ID = 1;
  }else if(HAL_GPIO_ReadPin(dipsw2_GPIO_Port, dipsw2_Pin) == 1){
	  ID = 2;
  }else if(HAL_GPIO_ReadPin(dipsw3_GPIO_Port, dipsw3_Pin) == 1){
	  ID = 3;
  }else {
	  while(1);
  }

  HAL_UART_Receive_DMA(&huart2, rxBuf, sizeof(rxBuf));
  HAL_UART_Receive_DMA(&huart5, tweliteRxBuf, sizeof(tweliteRxBuf));
  HAL_GPIO_WritePin(servosw_GPIO_Port, servosw_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);

  while (!ready)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x28 << 1, 10, 1000) == HAL_OK)
    {
      ready = 1;
    }
    else
    {
      HAL_Delay(100);
    }
  }










  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  unsigned char address = 0x28;
  BNO055 bno055(hi2c1, address);
  EULAR e;










  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    e = bno055.get_eular();
    gyro = (e.z / M_PI) * 180;

    ledPos0 = (-gyro + ((servoPos0 % 4096 + 4096) % 4096) * 360 / 4096 + 360) % 360;
    ledPos2 = (ledPos0 + ((servoPos2 % 4096 + 4096) % 4096) * 360 / 4096) % 360;

    get_position(0);
    get_position(2);

    if (HAL_GPIO_ReadPin(slidesw1_GPIO_Port, slidesw1_Pin) == 1 && HAL_GPIO_ReadPin(slidesw2_GPIO_Port, slidesw2_Pin) == 0)
    {
      servo0.moveCont(1500, 8191, servoPos0);
      servo2.moveCont(1500, 8191, servoPos2);

      if (i != 0)
      {
        i += 8;
        if (i >= 180)
        {
          i -= 360;
        }
      }

      moveRotation = calc.calcRotation(i, gyro);

      if (moveRotation > 0)
      {
        moveRotation += 10;
      }
      if (moveRotation < 0)
      {
        moveRotation -= 10;
      }

      sendData(0, 0, moveRotation);

      for (uint8_t led = 0; led < 16; led++)
      {
        NeopixelTape.set_hsv(led, calc.similarityRise(led, ledPos0, 90, 128, 100), 255, calc.similarityNormal(led, ledPos0, 90));
        NeopixelTape.show();
      }
      for (uint8_t led = 32; led < 48; led++)
      {
        NeopixelTape.set_hsv(led, calc.similarityRise(led, ledPos2, 90, 128, 100), 255, calc.similarityNormal(led, ledPos2, 90));
        NeopixelTape.show();
      }
    }
    else if (HAL_GPIO_ReadPin(slidesw1_GPIO_Port, slidesw1_Pin) == 1 && HAL_GPIO_ReadPin(slidesw2_GPIO_Port, slidesw2_Pin) == 1)
    {
      servo0.moveCont(1500, 0, servoPos0);
      servo2.moveCont(1500, 0, servoPos2);

      if (i != 8)
      {
        i -= 8;
        if (i < -180)
        {
          i += 360;
        }
      }

      moveRotation = calc.calcRotation(i, gyro);

      if (moveRotation > 0)
      {
        moveRotation += 10;
      }
      if (moveRotation < 0)
      {
        moveRotation -= 10;
      }

      sendData(0, 0, moveRotation);

      for (uint8_t led = 0; led < 16; led++)
      {
        NeopixelTape.set_hsv(led, calc.similarityRise(led, ledPos2, 90, 128, 100), 255, calc.similarityNormal(led, 360 - ledPos2, 90));
        NeopixelTape.show();
      }
      for (uint8_t led = 32; led < 48; led++)
      {
        NeopixelTape.set_hsv(led, calc.similarityRise(led, ledPos2, 90, 128, 100), 255, calc.similarityNormal(led, 360 - ledPos2, 90));
        NeopixelTape.show();
      }
    }
    else
    {
//      servo0.moveCont(1000, 2048, servoPos0);
//      servo1.moveStop1(1000, 2048);
//      servo2.moveCont(1000, 2048, servoPos2);
//      servo3.moveStop3(1000, 1800);
      sendData(0, 0, 0);
      for (uint8_t led = 0; led < 48; led++)
      {
        NeopixelTape.set_hsv(led, calc.similarityRise(led, 180, 360, hue, 100), 255, 10);
        NeopixelTape.show();
      }
    }

    twelite();










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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40-1;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 1000000;
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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(servosw_GPIO_Port, servosw_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : measure_Pin */
  GPIO_InitStruct.Pin = measure_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(measure_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : servosw_Pin */
  GPIO_InitStruct.Pin = servosw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(servosw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : slidesw1_Pin slidesw2_Pin */
  GPIO_InitStruct.Pin = slidesw1_Pin|slidesw2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : dipsw3_Pin dipsw2_Pin dipsw1_Pin */
  GPIO_InitStruct.Pin = dipsw3_Pin|dipsw2_Pin|dipsw1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : dipsw4_Pin dipsw5_Pin */
  GPIO_InitStruct.Pin = dipsw4_Pin|dipsw5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void get_position(uint8_t servoID)
{
  static int16_t inst_pos0 = 0;
  static int16_t inst_pos1 = 0;
  static int16_t inst_pos2 = 0;
  static int16_t inst_pos3 = 0;
  static uint8_t readPos = 0;
  uint8_t checksum = 0;
  uint8_t read[15] = {0};
  uint8_t position[6] = {0};

  switch (servoID)
  {
  case 0:
    servo0.send();
    break;
  case 1:
    servo1.send();
    break;
  case 2:
    servo2.send();
    break;
  case 3:
    servo3.send();
    break;
  default:
    break;
  }

  HAL_Delay(1);

  readPos = huart2.hdmarx->Instance->NDTR;
  readPos = sizeof(rxBuf) - readPos;

  if (readPos >= 15)
  {
    for (int i = 0; i < 15; i++)
    {
      read[i] = rxBuf[readPos - 14 + i];
      HAL_Delay(1);
    }
  }

  for (int i = 0; i < 8; i++)
  {
    if (read[i] == 255 && read[i + 1] == 255)
    {
      for (int j = 0; j < 6; j++)
      {
        position[j] = read[i + j + 2];
      }
    }
  }

  for (int i = 0; i < 5; i++)
  {
    checksum += position[i];
  }
  checksum = ~checksum;

  if (checksum == position[5] && position[0] == 0)
  {
    inst_pos0 = position[3] + position[4] * 256;
    servoPos0 = servo0.calculate_position(inst_pos0);
  }
  if (checksum == position[5] && position[0] == 1)
  {
    inst_pos1 = position[3] + position[4] * 256;
    servoPos1 = servo1.calculate_position(inst_pos1);
  }
  if (checksum == position[5] && position[0] == 2)
  {
    inst_pos2 = position[3] + position[4] * 256;
    servoPos2 = servo2.calculate_position(inst_pos2);
  }
  if (checksum == position[5] && position[0] == 3)
  {
    inst_pos3 = position[3] + position[4] * 256;
    servoPos3 = servo3.calculate_position(inst_pos3);
  }
}

void sendData(uint16_t angle, uint8_t speed, int16_t rotation)
{
  uint8_t checksum = 0;

  rotation += 360;

  sendArray[2] = angle / 256;
  sendArray[3] = angle % 256;
  sendArray[4] = speed;
  sendArray[5] = rotation / 256;
  sendArray[6] = rotation % 256;

  for (uint8_t i = 2; i < 7; i++)
  {
    checksum += sendArray[i];
  }
  checksum = ~checksum;
  checksum += 10;
  sendArray[7] = checksum;
  HAL_UART_Transmit(&huart3, sendArray, 8, 100);
  HAL_Delay(1);
}

void twelite()
{
  static uint8_t readPos = 0;
  uint8_t tweliteData[4] = {0};

  HAL_Delay(1);

  readPos = huart5.hdmarx->Instance->NDTR;
  readPos = sizeof(tweliteRxBuf) - readPos;

  if (readPos >= 9)
  {
    for (int i = 0; i < 9; i++)
    {
      tweliteRead[i] = tweliteRxBuf[readPos - 8 + i];
      HAL_Delay(1);
    }
  }

  for (int i = 0; i < 5; i++)
  {
    if (tweliteRead[i] == 250)
    {
      for (int j = 0; j < 4; j++)
      {
        tweliteData[j] = tweliteRead[i + j + 1];
      }
    }
  }

  mode = tweliteData[0] - 5;
  count = (tweliteData[1] - 5) * 240 + tweliteData[2] - 5;
  hue = tweliteData[3];
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
