

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
TIM_HandleTypeDef htim6;
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

uint32_t millis = 0;

uint16_t countLocal = 0;

// static int16_t i = 8;
// static int16_t moveRotation = 0;

uint8_t tweliteData[4] = {0};

<<<<<<< HEAD
uint16_t beat = 0;
uint8_t measureA = 0;
uint8_t measureB = 0;
uint8_t gesture = 250;

uint8_t pfmStatus = 0;
=======








>>>>>>> parent of feb78f6 (本番パフォーマンス)

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
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void get_position(uint8_t servoID);
void sendData(uint16_t angle, uint8_t speed, int16_t rotation);
void twelite();
void setMode();
<<<<<<< HEAD
// void modeError();
// void mode0();
// void mode1();
// void mode2();
// void mode3();
// void mode4();
// void mode5();
// void mode6();
// void mode7();
// void mode8();
// void mode9();
// void mode10();
// void mode11();
// void mode12();
// void mode13();
// void mode14();
// void mode15();
// void mode16();
// void mode17();
// void mode18();
// void mode19();
// void mode20();
// void mode21();
// void mode22();
// void mode23();
// void mode24();
// void mode25();
void gesture0();
void gesture1();
void gesture2();
=======
void mode0();
void mode1();
void mode2();
void mode3();
void mode4();
void mode5();
void mode6();
void mode7();
void mode8();
void mode9();
void mode10();
void mode11();
void mode12();
void mode13();
void mode14();
void mode15();
void modeError();
>>>>>>> parent of feb78f6 (本番パフォーマンス)

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    millis++;
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  NeopixelTape.init();

  //  if (HAL_GPIO_ReadPin(dipsw1_GPIO_Port, dipsw1_Pin) == 1)
  //  {
  //    ID = 1;
  //  }
  //  else if (HAL_GPIO_ReadPin(dipsw2_GPIO_Port, dipsw2_Pin) == 1)
  //  {
  //    ID = 2;
  //  }
  //  else if (HAL_GPIO_ReadPin(dipsw3_GPIO_Port, dipsw3_Pin) == 1)
  //  {
  //    ID = 3;
  //  }
  //  else
  //  {
  //    while (1)
  //      ;
  //  }

  HAL_TIM_Base_Start_IT(&htim6);
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

<<<<<<< HEAD
    twelite();
    //	setMode();

    if (gesture == 0)
    {
      pfmStatus = 1;
    }
    if (gesture == 1)
    {
      pfmStatus = 2;
    }
    if (gesture == 2)
    {
      pfmStatus = 3;
    }

    if (pfmStatus == 1)
    {
      gesture0();
    }
    else if (pfmStatus == 2)
    {
      gesture1();
    }
    else if (pfmStatus == 3)
    {
      gesture2();
    }
    else
    {
      for (uint8_t led = 0; led < 48; led++)
      {
        NeopixelTape.set_hsv(led, 180, 255, 255);
      }
      NeopixelTape.show();
      HAL_Delay(1);
      servo0.moveCont(1000, 2047, servoPos0);
      servo1.moveStop1(1000, 2047);
      servo2.moveCont(500, 2047, servoPos2);
      servo3.moveStop3(500, 1800);
    }
=======
        twelite();
        setMode();
        if (mode == 0)
        {
          mode0();
        }
        else if (mode == 1 || millis < 7500)
        {
          mode1();
        }
        else if (mode == 2 || millis < 22500)
        {
          mode2();
        }
        else if (mode == 3 || millis < 37000)
        {
          mode3();
        }
        else if (mode == 4 || millis < 52000)
        {
          mode4();
        }
        else if (mode == 5 || millis < 60000)
        {
          mode5();
        }
        else if (mode == 6 || millis < 67000)
        {
          mode6();
        }
        else if (mode == 7 || millis < 74500)
        {
          mode7();
        }
        else if (mode == 8 || millis < 100000)
        {
          mode8();
        }
        else if (mode == 9 || millis < 200000)
        {
          mode9();
        }
        else if (mode == 10 || millis < 208500)
        {
          mode10();
        }
        else if (mode == 11 || millis < 222500)
        {
          mode11();
        }
        else if (mode == 12 || millis < 234700)
        {
          mode12();
        }
        else if (mode == 13 || millis < 238000)
        {
          mode13();
        }
        else if (mode == 14 || millis < 244500)
        {
          mode14();
        }
        else if (mode == 15 || millis >= 244500)
        {
          mode15();
        }
        else
        {
          modeError();
        }










>>>>>>> parent of feb78f6 (本番パフォーマンス)
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  htim3.Init.Prescaler = 1 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40 - 1;
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
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
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
  GPIO_InitStruct.Pin = slidesw1_Pin | slidesw2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : dipsw3_Pin dipsw2_Pin dipsw1_Pin */
  GPIO_InitStruct.Pin = dipsw3_Pin | dipsw2_Pin | dipsw1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : dipsw4_Pin dipsw5_Pin */
  GPIO_InitStruct.Pin = dipsw4_Pin | dipsw5_Pin;
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
  uint8_t tweliteRead[9] = {0};

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
    if (tweliteRead[i] == 250 && tweliteRead[i + 1] <= 20 && tweliteRead[i + 1] >= 5)
    {
      for (int j = 0; j < 4; j++)
      {
        tweliteData[j] = tweliteRead[i + j + 1];
      }
    }
  }

<<<<<<< HEAD
  //  mode = tweliteData[0] - 5;
  //  count = (tweliteData[1] - 5) * 240 + tweliteData[2] - 5;
  //  beat = (int)(count / 20.32);
  gesture = tweliteData[1] - 5;
  //  if (mode < 10)
  //  {
  //    measureA = beat / 4;
  //  }
  //  else
  //  {
  //    measureB = (beat - 18) / 4;
  //  }
=======
  mode = tweliteData[0] - 5;
  count = (tweliteData[1] - 5) * 240 + tweliteData[2] - 5;
>>>>>>> parent of feb78f6 (本番パフォーマンス)
  hue = tweliteData[3];
}

void gesture0()
{
  uint16_t nowTime = millis;
  static uint16_t preTime = millis;
  if (nowTime - preTime < 1500)
  {
<<<<<<< HEAD
    servo0.moveCont(1000, 3071, servoPos0);
    servo1.moveStop1(1000, 2047);
    servo2.moveCont(500, 2047, servoPos2);
    servo3.moveStop3(1000, 1800);
    for (uint8_t led = 0; led < 48; led++)
    {
      NeopixelTape.set_hsv(led, 127, 255, 255);
    }
=======
    millis = 0;
    modeStatus++;
  }
  if (mode == 2 && modeStatus == 1)
  {
    millis = 7500;
    modeStatus++;
  }
  if (mode == 3 && modeStatus == 2)
  {
    millis = 22500;
    modeStatus++;
  }
  if (mode == 4 && modeStatus == 3)
  {
    millis = 37000;
    modeStatus++;
  }
  if (mode == 5 && modeStatus == 4)
  {
    millis = 52000;
    modeStatus++;
  }
  if (mode == 6 && modeStatus == 5)
  {
    millis = 60000;
    modeStatus++;
  }
  if (mode == 7 && modeStatus == 6)
  {
    millis = 67000;
    modeStatus++;
  }
  if (mode == 8 && modeStatus == 7)
  {
    millis = 745000;
    modeStatus++;
  }
  if (mode == 9 && modeStatus == 8)
  {
    millis = 1000000;
    modeStatus++;
  }
  if (mode == 10 && modeStatus == 11)
  {
    millis = 2000000;
    modeStatus++;
  }
  if (mode == 11 && modeStatus == 10)
  {
    millis = 2008500;
    modeStatus++;
  }
  if (mode == 12 && modeStatus == 11)
  {
    millis = 2022500;
    modeStatus++;
  }
  if (mode == 13 && modeStatus == 12)
  {
    millis = 2034700;
    modeStatus++;
  }
  if (mode == 14 && modeStatus == 13)
  {
    millis = 2038000;
    modeStatus++;
  }
  if (mode == 15 && modeStatus == 14)
  {
    millis = 2044500;
    modeStatus++;
  }
}

void mode0()
{
  servo0.moveCont(500, 2048, servoPos0);
  servo1.moveStop1(500, 2048);
  servo2.moveCont(500, 2048, servoPos2);
  servo3.moveStop3(500, 1800);
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, hue, 255, 100);
    NeopixelTape.show();
    HAL_Delay(1);
  }
}

void mode1()
{
  servo0.moveCont(0, 2048, servoPos0);
  servo1.moveStop1(0, 2048);
  servo2.moveCont(0, 2048, servoPos2);
  servo3.moveStop3(0, 1800);
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, 0, 0, 0);
>>>>>>> parent of feb78f6 (本番パフォーマンス)
    NeopixelTape.show();
    HAL_Delay(1);
  }
  else if (nowTime - preTime < 3000)
  {
    servo0.moveCont(1000, 1023, servoPos0);
    servo1.moveStop1(1000, 2047);
    servo2.moveCont(500, 2047, servoPos2);
    servo3.moveStop3(1000, 3000);
    for (uint8_t led = 0; led < 48; led++)
    {
      NeopixelTape.set_hsv(led, 127, 255, 255);
    }
    NeopixelTape.show();
    HAL_Delay(1);
  }
  else
  {
    preTime = millis;
  }
}

void gesture1()
{
  uint16_t nowTime1 = millis;
  static uint16_t preTime1 = millis;
  uint16_t nowTime2 = millis;
  static uint16_t preTime2 = millis;
  if (nowTime1 - preTime1 < 4000)
  {
    servo0.moveCont(1000, 8191, servoPos0);
    servo1.moveStop1(1000, 1000);
    servo2.moveCont(500, 4095, servoPos2);
    servo3.moveStop3(100, 1800);
  }
  else if (nowTime1 - preTime1 < 8000)
  {
    servo0.moveCont(1000, 0, servoPos0);
    servo1.moveStop1(1000, 3000);
    servo2.moveCont(500, 0, servoPos2);
    servo3.moveStop3(100, 1800);
  }
  else
  {
    preTime1 = millis;
  }
  if (nowTime2 - preTime2 <= 53000)
  {
    for (uint8_t led = 0; led < 48; led++)
    {
      NeopixelTape.set_hsv(led, 180 - (nowTime2 - preTime2) / 1000, 255, 255);
    }
    NeopixelTape.show();
    HAL_Delay(1);
  }
  else
  {
    for (uint8_t led = 0; led < 48; led++)
    {
      NeopixelTape.set_hsv(led, 127, 255, 255);
    }
    NeopixelTape.show();
    HAL_Delay(1);
  }
}

void gesture2()
{
  uint16_t nowTime = millis;
  static uint16_t preTime = millis;
  if (nowTime - preTime < 1500)
  {
    servo0.moveCont(1000, 2047, servoPos0);
    servo1.moveStop1(1000, 1000);
    servo2.moveCont(500, 2047, servoPos2);
    servo3.moveStop3(500, 1800);
    for (uint8_t led = 0; led < 48; led++)
    {
      NeopixelTape.set_hsv(led, 127, 255, 255);
    }
    NeopixelTape.show();
    HAL_Delay(1);
  }
  else if (nowTime - preTime < 3000)
  {
    servo0.moveCont(1000, 2047, servoPos0);
    servo1.moveStop1(1000, 3000);
    servo2.moveCont(500, 2047, servoPos2);
    servo3.moveStop3(500, 3000);
    for (uint8_t led = 0; led < 48; led++)
    {
      NeopixelTape.set_hsv(led, 127, 255, 255);
    }
    NeopixelTape.show();
    HAL_Delay(1);
  }
<<<<<<< HEAD
  else
  {
    preTime = millis;
  }
}

// void setMode()
//{
//   static uint8_t modeStatus = 0;
//   if (mode == 1 && modeStatus == 0)
//   {
//     millis = 0;
//     modeStatus++;
//   }
//   if (mode == 2 && modeStatus == 1)
//   {
//     millis = 7500;
//     modeStatus++;
//   }
//   if (mode == 3 && modeStatus == 2)
//   {
//     millis = 22500;
//     modeStatus++;
//   }
//   if (mode == 4 && modeStatus == 3)
//   {
//     millis = 37000;
//     modeStatus++;
//   }
//   if (mode == 5 && modeStatus == 4)
//   {
//     millis = 52000;
//     modeStatus++;
//   }
//   if (mode == 6 && modeStatus == 5)
//   {
//     millis = 60000;
//     modeStatus++;
//   }
//   if (mode == 7 && modeStatus == 6)
//   {
//     millis = 67000;
//     modeStatus++;
//   }
//   if (mode == 8 && modeStatus == 7)
//   {
//     millis = 745000;
//     modeStatus++;
//   }
//   if (mode == 9 && modeStatus == 8)
//   {
//     millis = 1000000;
//     modeStatus++;
//   }
//   if (mode == 10 && modeStatus == 11)
//   {
//     millis = 2000000;
//     modeStatus++;
//   }
//   if (mode == 11 && modeStatus == 10)
//   {
//     millis = 2008500;
//     modeStatus++;
//   }
//   if (mode == 12 && modeStatus == 11)
//   {
//     millis = 2022500;
//     modeStatus++;
//   }
//   if (mode == 13 && modeStatus == 12)
//   {
//     millis = 2034700;
//     modeStatus++;
//   }
//   if (mode == 14 && modeStatus == 13)
//   {
//     millis = 2038000;
//     modeStatus++;
//   }
//   if (mode == 15 && modeStatus == 14)
//   {
//     millis = 2044500;
//     modeStatus++;
//   }
// }
//
// void mode0()
//{
//   servo0.moveCont(500, 2048, servoPos0);
//   servo1.moveStop1(500, 2048);
//   servo2.moveCont(500, 2048, servoPos2);
//   servo3.moveStop3(500, 1800);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, hue, 255, 100);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode1()
//{
//   servo0.moveCont(0, 2048, servoPos0);
//   servo1.moveStop1(0, 2048);
//   servo2.moveCont(0, 2048, servoPos2);
//   servo3.moveStop3(0, 1800);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, 0, 0, 0);
//     NeopixelTape.show();
//     HAL_Delay(1);
//   }
// }
//
// void mode2()
//{
//   servo0.moveCont(0, 2048, servoPos0);
//   servo1.moveStop1(0, 2048);
//   servo2.moveCont(0, 2048, servoPos2);
//   servo3.moveStop3(0, 1800);
// }
//
// void mode3()
//{
//   countLocal = millis - 22500;
//   if (ID == 1)
//   {
//     for (uint8_t led = 0; led < 16; led++)
//     {
//       if (countLocal < 10000)
//       {
//         servo0.moveCont(0, 2048, servoPos0);
//         servo1.moveStop1(0, 2048);
//         servo2.moveCont(0, 2048, servoPos2);
//         servo3.moveStop3(0, 1800);
//         NeopixelTape.set_hsv(led, hue, 255, 255);
//         NeopixelTape.set_hsv(led + 16, hue, 255, calc.similarityNormal(led, 180, countLocal / 30));
//         NeopixelTape.set_hsv(led + 32, hue, 255, 0);
//       }
//       if (countLocal > 10000 && countLocal < 12550)
//       {
//         servo0.moveCont(0, 2048, servoPos0);
//         servo1.moveStop1(0, 2048);
//         servo2.moveCont(0, 2048, servoPos2);
//         servo3.moveStop3(0, 1800);
//         NeopixelTape.set_hsv(led, hue, 255, 255);
//         NeopixelTape.set_hsv(led + 16, hue, 255, 255);
//         NeopixelTape.set_hsv(led + 32, hue, 255, (countLocal - 10000) / 10);
//       }
//       if (countLocal > 12550)
//       {
//         servo0.moveCont(500, 2048, servoPos0);
//         servo1.moveStop1(2000, 1024);
//         servo2.moveCont(500, 2048, servoPos2);
//         servo3.moveStop3(2000, 2800);
//         NeopixelTape.set_hsv(led, hue, 255, 0);
//         NeopixelTape.set_hsv(led + 16, hue, 255, 0);
//         NeopixelTape.set_hsv(led + 32, hue, 255, 0);
//       }
//     }
//     NeopixelTape.show();
//     HAL_Delay(1);
//   }
//   else
//   {
//     for (uint8_t led = 0; led < 48; led++)
//     {
//       NeopixelTape.set_hsv(led, 0, 0, 0);
//     }
//     NeopixelTape.show();
//     HAL_Delay(1);
//     servo0.moveCont(500, 2048, servoPos0);
//     servo1.moveStop1(500, 2048);
//     servo2.moveCont(500, 2048, servoPos2);
//     servo3.moveStop3(500, 1800);
//   }
// }
//
// void mode4()
//{
//   countLocal = millis - 37000;
//   if (ID == 1)
//   {
//     for (uint8_t led = 0; led < 48; led++)
//     {
//       NeopixelTape.set_hsv(led, 0, 0, 0);
//     }
//     NeopixelTape.show();
//     HAL_Delay(1);
//     servo0.moveCont(500, 2048, servoPos0);
//     servo1.moveStop1(500, 1000);
//     servo2.moveCont(500, 2048, servoPos2);
//     servo3.moveStop3(500, 3000);
//   }
//   if (ID == 2 && countLocal < 7500)
//   {
//     servo0.moveCont(1000, 6144, servoPos0);
//     servo1.moveStop1(500, 2048);
//     servo2.moveCont(1000, 6144, servoPos2);
//     servo3.moveStop3(500, 1800);
//     for (uint8_t led = 0; led < 48; led++)
//     {
//       NeopixelTape.set_hsv(led, hue, 255, 100);
//       NeopixelTape.show();
//     }
//   }
//   if (ID == 2 && countLocal > 7500)
//   {
//     servo0.moveCont(1000, 6144, servoPos0);
//     servo1.moveStop1(2000, 1000);
//     servo2.moveCont(1000, 6144, servoPos2);
//     servo3.moveStop3(2000, 3000);
//     for (uint8_t led = 0; led < 48; led++)
//     {
//       NeopixelTape.set_hsv(led, hue, 255, 0);
//     }
//     NeopixelTape.show();
//     HAL_Delay(1);
//   }
//   if (ID == 3 && countLocal < 7500)
//   {
//     servo0.moveCont(500, 2048, servoPos0);
//     servo1.moveStop1(500, 2048);
//     servo2.moveCont(500, 2048, servoPos2);
//     servo3.moveStop3(500, 1800);
//   }
//   if (ID == 3 && countLocal > 7500 && countLocal < 11800)
//   {
//     servo0.moveCont(0, 2048, servoPos0);
//     servo1.moveStop1(0, 2048);
//     servo2.moveCont(0, 2048, servoPos2);
//     servo3.moveStop3(0, 1800);
//     moveRotation = calc.calcRotation((countLocal - 7500) / 11, gyro);
//     sendData(0, 0, moveRotation);
//     for (uint8_t led = 0; led < 16; led++)
//     {
//       NeopixelTape.set_hsv(led, calc.similarityRise(led, (ledPos0 + 180) % 360, 90, hue, 100), 255, calc.similarityNormal(led, (ledPos0 + 180) % 360, 90));
//     }
//     NeopixelTape.show();
//     HAL_Delay(1);
//   }
//   if (ID == 3 && countLocal > 11800)
//   {
//     sendData(0, 0, 0);
//     servo0.moveCont(2000, 3072, servoPos0);
//     servo1.moveStop1(2000, 1024);
//     servo2.moveCont(1000, 2048, servoPos2);
//     servo3.moveStop3(2000, 2800);
//     for (uint8_t led = 0; led < 16; led++)
//     {
//       NeopixelTape.set_hsv(led, 0, 0, 0);
//     }
//     NeopixelTape.show();
//     HAL_Delay(1);
//   }
// }
//
// void mode5()
//{
//   servo0.moveCont(2000, 2048, servoPos0);
//   servo1.moveStop1(2000, 2048);
//   servo2.moveCont(2000, 2048, servoPos2);
//   servo3.moveStop3(2000, 1800);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, 0, 0, 0);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode6()
//{
//   countLocal = millis - 60000;
//   servo0.moveCont(0, 2048, servoPos0);
//   servo1.moveStop1(0, 2048);
//   servo2.moveCont(0, 2048, servoPos2);
//   servo3.moveStop3(0, 1800);
//   if ((countLocal / 3) % 360 > 300)
//   {
//     for (uint8_t led = 32; led < 48; led++)
//     {
//       NeopixelTape.set_hsv(led, hue, 255, 255);
//     }
//   }
//   else if ((countLocal / 3) % 360 > 30)
//   {
//     for (uint8_t led = 0; led < 16; led++)
//     {
//       NeopixelTape.set_hsv(led, hue, 255, 255);
//       NeopixelTape.set_hsv(led + 16, hue, 255, calc.similarityNormal(led, 180, (countLocal / 3) % 360));
//       NeopixelTape.set_hsv(led + 32, hue, 255, 0);
//     }
//   }
//   else
//   {
//     for (uint8_t led = 0; led < 16; led++)
//     {
//       NeopixelTape.set_hsv(led, hue, 255, 0);
//       NeopixelTape.set_hsv(led + 16, hue, 255, 0);
//       NeopixelTape.set_hsv(led + 32, hue, 255, 0);
//     }
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode7()
//{
//   static uint16_t randomNum = 0;
//   randomNum += 47;
//   servo0.moveCont(0, 2000 + randomNum % 128, servoPos0);
//   servo1.moveStop1(0, 2000 + randomNum % 128);
//   servo2.moveCont(0, 2000 + randomNum % 128, servoPos2);
//   servo3.moveStop3(0, 2000 + randomNum % 128);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, randomNum, randomNum, randomNum);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode8()
//{
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, 0, 0, 0);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode9()
//{
//   servo0.moveCont(0, 2048, servoPos0);
//   servo1.moveStop1(0, 2048);
//   servo2.moveCont(0, 2048, servoPos2);
//   servo3.moveStop3(0, 1800);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, 0, 0, 0);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode10()
//{
//   servo0.moveCont(0, 2048, servoPos0);
//   servo1.moveStop1(0, 2048);
//   servo2.moveCont(0, 2048, servoPos2);
//   servo3.moveStop3(0, 1800);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, 0, 0, 0);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode11()
//{
//   countLocal = millis - 208500;
//   static uint8_t status = 0;
//   static uint16_t degree = 0;
//
//   servo0.moveCont(1000, countLocal % 2000 * 2, servoPos0);
//   servo1.moveStop1(1000, countLocal % 2000 + 1100);
//   servo2.moveCont(1000, countLocal % 2000 * 2, servoPos2);
//   servo3.moveStop3(1000, countLocal % 2000 / 2 + 1900);
//
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, calc.similarityPeak(led, (countLocal / 3) % 360, 90, countLocal % 256, 50), 255, calc.similarityNormal(led, (countLocal / 3) % 360, 90));
//     NeopixelTape.show();
//     HAL_Delay(1);
//   }
// }
//
// void mode12()
//{
//   countLocal = millis - 222500;
//   static int16_t i = 8;
//   if (countLocal % 6000 < 3000)
//   {
//     servo0.moveCont(1500, 8191, servoPos0);
//     servo2.moveCont(1500, 8191, servoPos2);
//
//     if (i != 0)
//     {
//       i += 8;
//       if (i >= 180)
//       {
//         i -= 360;
//       }
//     }
//
//     moveRotation = calc.calcRotation(i, gyro);
//
//     if (moveRotation > 0)
//     {
//       moveRotation += 10;
//     }
//     if (moveRotation < 0)
//     {
//       moveRotation -= 10;
//     }
//
//     sendData(0, 0, moveRotation);
//
//     for (uint8_t led = 0; led < 16; led++)
//     {
//       NeopixelTape.set_hsv(led, calc.similarityRise(led, ledPos0, 90, 128, 100), 255, calc.similarityNormal(led, ledPos0, 90));
//       NeopixelTape.show();
//     }
//     for (uint8_t led = 32; led < 48; led++)
//     {
//       NeopixelTape.set_hsv(led, calc.similarityRise(led, ledPos2, 90, 128, 100), 255, calc.similarityNormal(led, ledPos2, 90));
//       NeopixelTape.show();
//     }
//   }
//   else
//   {
//     servo0.moveCont(1500, 0, servoPos0);
//     servo2.moveCont(1500, 0, servoPos2);
//
//     if (i != 8)
//     {
//       i -= 8;
//       if (i < -180)
//       {
//         i += 360;
//       }
//     }
//
//     moveRotation = calc.calcRotation(i, gyro);
//
//     if (moveRotation > 0)
//     {
//       moveRotation += 10;
//     }
//     if (moveRotation < 0)
//     {
//       moveRotation -= 10;
//     }
//
//     sendData(0, 0, moveRotation);
//
//     for (uint8_t led = 0; led < 16; led++)
//     {
//       NeopixelTape.set_hsv(led, calc.similarityRise(led, ledPos2, 90, 128, 100), 255, calc.similarityNormal(led, 360 - ledPos2, 90));
//       NeopixelTape.show();
//     }
//     for (uint8_t led = 32; led < 48; led++)
//     {
//       NeopixelTape.set_hsv(led, calc.similarityRise(led, ledPos2, 90, 128, 100), 255, calc.similarityNormal(led, 360 - ledPos2, 90));
//       NeopixelTape.show();
//     }
//   }
// }
//
// void mode13()
//{
//   countLocal = millis - 234700;
//   sendData(0, 0, 60);
//   servo0.moveCont(1000, countLocal * 2, servoPos0);
//   servo2.moveCont(1000, countLocal * 2, servoPos2);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, calc.similarityPeak(led, (countLocal / 3) % 360, 90, countLocal % 256, 50), 255, calc.similarityNormal(led, (countLocal / 3) % 360, 90));
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode14()
//{
//   static uint16_t i = 60;
//   if (i > 1)
//   {
//     i--;
//   }
//   sendData(0, 0, 0);
//   servo0.moveCont(0, 6000, servoPos0);
//   servo1.moveStop1(0, 2048);
//   servo2.moveCont(0, 6000, servoPos2);
//   servo3.moveStop3(0, 1900);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, hue, 255, i);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void mode15()
//{
//   countLocal = millis - 244500;
//   servo0.moveCont(0, 6000, servoPos0);
//   servo1.moveStop1(0, 2048);
//   servo2.moveCont(0, 6000, servoPos2);
//   servo3.moveStop3(0, 1900);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, hue, 255, 0);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
//
// void modeError()
//{
//   servo0.moveCont(500, 2048, servoPos0);
//   servo1.moveStop1(500, 2048);
//   servo2.moveCont(500, 2048, servoPos2);
//   servo3.moveStop3(500, 1800);
//   for (uint8_t led = 0; led < 48; led++)
//   {
//     NeopixelTape.set_hsv(led, 0, 0, 0);
//   }
//   NeopixelTape.show();
//   HAL_Delay(1);
// }
=======
  if (ID == 3 && countLocal < 7500)
  {
    servo0.moveCont(500, 2048, servoPos0);
    servo1.moveStop1(500, 2048);
    servo2.moveCont(500, 2048, servoPos2);
    servo3.moveStop3(500, 1800);
  }
  if (ID == 3 && countLocal > 7500 && countLocal < 11400)
  {
    servo0.moveCont(0, 2048, servoPos0);
    servo1.moveStop1(0, 2048);
    servo2.moveCont(0, 2048, servoPos2);
    servo3.moveStop3(0, 1800);
    moveRotation = calc.calcRotation((countLocal - 7500) / 10, gyro);
    sendData(0, 0, moveRotation);
    for (uint8_t led = 0; led < 16; led++)
    {
      NeopixelTape.set_hsv(led, calc.similarityRise(led, (ledPos0 + 180) % 360, 90, hue, 100), 255, calc.similarityNormal(led, (ledPos0 + 180) % 360, 90));
    }
    NeopixelTape.show();
    HAL_Delay(1);
  }
  if (ID == 3 && countLocal > 11400)
  {
    sendData(0, 0, 0);
    servo0.moveCont(2000, 3072, servoPos0);
    servo1.moveStop1(2000, 1024);
    servo2.moveCont(1000, 2048, servoPos2);
    servo3.moveStop3(2000, 2800);
  }
}

void mode5()
{
  servo0.moveCont(0, 2048, servoPos0);
  servo1.moveStop1(0, 2048);
  servo2.moveCont(0, 2048, servoPos2);
  servo3.moveStop3(0, 1800);
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, 0, 0, 0);
  }
  NeopixelTape.show();
  HAL_Delay(1);
}

void mode6()
{
  servo0.moveCont(0, 2048, servoPos0);
  servo1.moveStop1(0, 2048);
  servo2.moveCont(0, 2048, servoPos2);
  servo3.moveStop3(0, 1800);
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, 0, 0, 0);
  }
  NeopixelTape.show();
  HAL_Delay(1);
}

void mode7()
{
  servo0.moveCont(0, 2048, servoPos0);
  servo1.moveStop1(0, 2048);
  servo2.moveCont(0, 2048, servoPos2);
  servo3.moveStop3(0, 1800);
  static uint16_t randomNum = 0;
  randomNum += 47;
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, randomNum, randomNum, randomNum);
  }
  NeopixelTape.show();
  HAL_Delay(1);
}

void mode8()
{
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, 0, 0, 0);
  }
  NeopixelTape.show();
  HAL_Delay(1);
}

void mode9() {}

void mode10() {}

void mode11()
{
  countLocal = millis - 208500;
  static uint8_t status = 0;
  static uint16_t degree = 0;

  servo0.moveCont(1000, countLocal % 2000 * 2, servoPos0);
  servo1.moveStop1(1000, countLocal % 2000 + 1100);
  servo2.moveCont(1000, countLocal % 2000 * 2, servoPos2);
  servo3.moveStop3(1000, countLocal % 2000 / 2 + 1900);

  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, calc.similarityPeak(led, (countLocal / 3) % 360, 90, countLocal % 256, 50), 255, calc.similarityNormal(led, (countLocal / 3) % 360, 90));
    NeopixelTape.show();
    HAL_Delay(1);
  }
}

void mode12()
{
  countLocal = millis - 222500;
  static int16_t i = 8;
  if (countLocal % 6000 < 3000)
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
  else
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
}

void mode13()
{
  countLocal = millis - 234700;
  sendData(0, 0, 60);
  servo0.moveCont(1000, countLocal * 2, servoPos0);
  servo2.moveCont(1000, countLocal * 2, servoPos2);
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, calc.similarityPeak(led, (countLocal / 3) % 360, 90, countLocal % 256, 50), 255, calc.similarityNormal(led, (countLocal / 3) % 360, 90));
  }
  NeopixelTape.show();
  HAL_Delay(1);
}

void mode14()
{
  countLocal = millis - 238000;
  sendData(0, 0, 0);
  servo0.moveCont(0, 6000, servoPos0);
  servo1.moveStop1(0, 2048);
  servo2.moveCont(0, 6000, servoPos2);
  servo3.moveStop3(0, 1900);
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, hue, 255, (7000 - countLocal) / 30);
  }
  NeopixelTape.show();
  HAL_Delay(1);
}

void mode15()
{
  countLocal = millis - 244500;
  servo0.moveCont(0, 6000, servoPos0);
  servo1.moveStop1(0, 2048);
  servo2.moveCont(0, 6000, servoPos2);
  servo3.moveStop3(0, 1900);
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, hue, 255, 0);
  }
  NeopixelTape.show();
  HAL_Delay(1);
}

void modeError()
{
  servo0.moveCont(500, 2048, servoPos0);
  servo1.moveStop1(500, 2048);
  servo2.moveCont(500, 2048, servoPos2);
  servo3.moveStop3(500, 1800);
  for (uint8_t led = 0; led < 48; led++)
  {
    NeopixelTape.set_hsv(led, 0, 0, 0);
  }
  NeopixelTape.show();
  HAL_Delay(1);
}
>>>>>>> parent of feb78f6 (本番パフォーマンス)










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

#ifdef USE_FULL_ASSERT
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
