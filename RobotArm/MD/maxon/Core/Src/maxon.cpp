#include "maxon.h"

MAXON::MAXON(TIM_HandleTypeDef *htim, uint32_t timChannel, GPIO_TypeDef *gpio, uint16_t shdnPin)
{
  HTIM = htim;
  channel = timChannel;
  GPIO = gpio;
  shdn = shdnPin;
  HAL_TIM_PWM_Start(HTIM, channel);
  HAL_TIMEx_PWMN_Start(HTIM, channel);
}

void MAXON::move(int16_t speed){
	__HAL_TIM_SET_COMPARE(HTIM, channel, speed + 320);
	if(speed == 0){
		HAL_GPIO_WritePin(GPIO, shdn, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIO, shdn, GPIO_PIN_SET);
	}
}
