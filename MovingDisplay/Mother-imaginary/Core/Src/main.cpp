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
#include "BNO055.h"
#include "performance.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart5_rx;

/* USER CODE BEGIN PV */
//uint8_t ready = 0 ;

uint32_t m_counter = 0;
uint32_t rst_mcounter = 0;
uint32_t now_mcounter = 0;

int16_t MTRS[4]={0, 0, 0, 0};
uint8_t send_array[12] = {250, 210, 210, 251, 210, 210, 252, 210, 210, 253, 210, 210};

uint8_t rxDataX[3]={};
uint8_t rxDataY[3]={};

int16_t position[3] = {0, 0, 0};//{x, y, speed}
float rotate;

int16_t p_position[3] = {0, 0, 0};//{x, y, speed}
int16_t dposition[2] = {0,0};

int16_t cur_movement[3] = {0, 0, 0};//{x, y, speed}
uint8_t display[13] = {};
//{startBit, x, y, r, circle_H, circle_S, circle_V,
//background_H, background_S, background_V, frame_H, frame_S, frame_V}

int16_t cur_position_rec[2];//{x, y}
int16_t cur_position_pol[2];//{r, degree}


uint8_t rxBuf[64]={0};
uint8_t rxData[4]={0,0,0,0};
uint8_t p_wrtpt = 0;
uint8_t p_rdpt = 0;
uint16_t stop_counter = 0;
uint16_t error_counter = 0;

//uint8_t perform_mode;
//uint16_t perform_count;
//uint16_t perform_beat;
//uint8_t perform_hue;

uint16_t perform[4] = {};//{mode, count, beat, hue}


uint16_t dtime;

int zero_thr = 25;
int dclr_thr = 400;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void speed_set(int gyro_degree, int goal_speed, int goal_degree, int16_t* mtrspeed, float motor_rate);
void set_array(int16_t* mtrspeed, uint8_t* sendarray);
void readBuf(UART_HandleTypeDef* uart, uint8_t* buf, int buf_size, uint8_t* data, int data_size, uint8_t id, uint8_t* p_wrtpt, uint8_t* p_rdpt, uint16_t* stop_counter, uint16_t* error_counter,  uint8_t go_back);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Transmit(&huart6, send_array, 12, 10);

  HAL_UART_Receive_DMA(&huart5,rxBuf,64);

  BNO055 bno055(&hi2c1);

  if (!bno055.begin()) {
	  // センサ初期化失敗時の処??��?��?
	  uint32_t Ltika_pcounter = m_counter;
	  while (1){
		if(m_counter - Ltika_pcounter > 100){
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		Ltika_pcounter = m_counter;
		}else{}
	  };
  }

  float heading, roll, pitch;

  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  PERFORMANCE performance(&m_counter);

  uint32_t Ltika_pcounter = m_counter;
  uint32_t d_pcounter = m_counter;
  uint32_t speed_pcounter = m_counter;

  uint8_t OdoX_ID[3] = {248, 210, 210};
  uint8_t OdoY_ID[3] = {249, 210, 210};

//  int16_t cur_position_rec[2];
//  int16_t cur_position_pol[2];

  int16_t speed;
  int16_t p_speed = 0;
  int16_t degree;

//setup p_position
  HAL_UART_Transmit(&huart6, OdoX_ID, 3, 1);
  if(HAL_UART_Receive(&huart6, rxDataX, 3, 1) == HAL_OK){
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }else{}
  p_position[0] = rxDataX[1] + rxDataX[2]*200 - 20000;

  HAL_UART_Transmit(&huart6, OdoY_ID, 3, 1);
  if(HAL_UART_Receive(&huart6, rxDataY, 3, 1) == HAL_OK){
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }else{}
  p_position[1] = rxDataY[1] + rxDataY[2]*200 - 20000;

  speed_pcounter = m_counter;


////START loop

  while (1)
  {
	dtime = m_counter - d_pcounter;
	d_pcounter = m_counter;

	if(HAL_GPIO_ReadPin(TACTSW0_GPIO_Port, TACTSW0_Pin) == 1){rst_mcounter = m_counter;}
	now_mcounter = m_counter - rst_mcounter;

////START get NOW-STATUS

//get rotate
	bno055.getEulerAngles(heading, roll, pitch);
	rotate = (int)heading;

//get x coordinate
	HAL_UART_Transmit(&huart6, OdoX_ID, 3, 1);
	if(HAL_UART_Receive(&huart6, rxDataX, 3, 1) == HAL_OK){
	  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	}else{}
	position[0] = rxDataX[1] + rxDataX[2]*200 - 20000;

//get y coordinate
	HAL_UART_Transmit(&huart6, OdoY_ID, 3, 1);
	if(HAL_UART_Receive(&huart6, rxDataY, 3, 1) == HAL_OK){
	  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	}else{}
	position[1] = rxDataY[1] + rxDataY[2]*200 - 20000;

//get speed
	if(m_counter - speed_pcounter >= 20){
		dposition[0] = position[0] - p_position[0];
		p_position[0] = position[0];

		dposition[1] = position[1] - p_position[1];
		p_position[1] = position[1];

		position[2] = pow(pow(dposition[0], 2) + pow(dposition[1], 2), 0.5)*1000 / (m_counter - speed_pcounter);
		position[2] = (position[2]*0.8) + (p_position[2]*0.2);
		p_position[2] = position[2];

		speed_pcounter = m_counter;
	}else{}

////END get NOW-STATUS


////get PERFORMANCE STATUS

//get perform[] from TweLite
	readBuf(&huart5, rxBuf, 64, rxData, 4, 0, &p_wrtpt, &p_rdpt, &stop_counter, &error_counter, 10);
	perform[0] = rxData[0] - 5;
	perform[1] = (rxData[1] - 5)*240 + (rxData[2] - 5);
	perform[2] = perform[1] / 23;
	perform[3] = rxData[3];

//get target
	performance.get_target_status(perform, now_mcounter, cur_movement, display);

////do MOVEMENT

//get movement distance
	for(int i=0; i<2; i++){cur_position_rec[i] = cur_movement[i] - position[i];}

//convert movement distance to polar coordinates
	cur_position_pol[0] = pow(pow(cur_position_rec[0], 2) + pow(cur_position_rec[1], 2), 0.5);
	cur_position_pol[1] = atan2(cur_position_rec[0], cur_position_rec[1]) / 3.1415 * 180.0;

//calculate movement speed from movement distance

	if((position[2] < cur_movement[2]) && (cur_position_pol[0] > dclr_thr)){speed ++;}
	else if(cur_position_pol[0] <= dclr_thr && cur_position_pol[0] > zero_thr){speed -= 2;}//速度制御
	else if(position[2] >= cur_movement[2]){speed --;}
	else{speed = 0;}

	if(cur_position_pol[0] <= dclr_thr && cur_position_pol[0] > zero_thr && speed < 100){speed = 100;}

	if(speed > cur_movement[2]*3){speed = cur_movement[2]*3;}
	else if(speed < 0){speed = 0;}

	speed = ((speed*8) + (p_speed*2) + 10-1)/ 10;
	p_speed = speed;

	degree = cur_position_pol[1];


//calculate motor speed from movement speed
	speed_set(rotate, speed, degree, MTRS, 0.7);

//calculate sending array from motor speed
	set_array(MTRS, send_array);

//send to motor
	if(HAL_GPIO_ReadPin(STRTSW_GPIO_Port, STRTSW_Pin) == 1){
	  HAL_UART_Transmit(&huart6, send_array, 12, 1);
	}else{
		for(int i=0; i<4; i++){
		  send_array[3*i] = 250 + i;
		  send_array[3*i + 1] = 210;
		  send_array[3*i + 2] = 210;
		}
		HAL_UART_Transmit(&huart6, send_array, 12, 1);
	}

//send to panel

//Lチカ
	if(m_counter - Ltika_pcounter > 1000){
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	Ltika_pcounter = m_counter;
	}else{}



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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STRTSW_Pin TACTSW0_Pin */
  GPIO_InitStruct.Pin = STRTSW_Pin|TACTSW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void speed_set(int gyro_degree, int goal_speed, int goal_degree, int16_t* mtrspeed, float motor_rate){
	goal_degree = goal_degree % 360;
	if(goal_degree < 0){goal_degree += 360;}


    int roll_speed;
    if(gyro_degree > 180){gyro_degree -= 360;}
    else if(gyro_degree <-180){gyro_degree += 360;}
    else{}


    if (gyro_degree > 0){
        roll_speed = gyro_degree * 50;
        if (gyro_degree < 2){
            roll_speed = 0;
        }
        if (roll_speed > 500){
            roll_speed = 500;
        }
    }else if (gyro_degree < 0){
        roll_speed = gyro_degree * 50;
        if (gyro_degree > -2){
            roll_speed = 0;
        }
		if (roll_speed < -500){
			roll_speed = -500;
        }

    }else{
        roll_speed = 0;
    }


	int conv_degree = -goal_degree + 45;
	if(conv_degree < 0){conv_degree = conv_degree + 360;}

	for(int i=0; i<4; i++){
		mtrspeed[i] = goal_speed * sin((conv_degree + 90.0*i) / 180.0 * 3.1415);
		mtrspeed[i] = (mtrspeed[i] * motor_rate) + (roll_speed * (1.0 - motor_rate));
	}
}

void set_array(int16_t* mtrspeed, uint8_t* sendarray){
	uint16_t conv_mtrspeed[4];
	for(int i=0; i<4; i++){conv_mtrspeed[i] = 10000 - (mtrspeed[i] + 5000);}
	for(int i=0; i<4; i++){
		sendarray[3*i] = 250+i;
		sendarray[3*i+1] = conv_mtrspeed[i] % 100;
		sendarray[3*i+2] = conv_mtrspeed[i] / 100;
	}
}

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
