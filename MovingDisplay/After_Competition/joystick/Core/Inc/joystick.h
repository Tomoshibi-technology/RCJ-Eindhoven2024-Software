/*
 * joystick.h
 *
 *  Created on: Oct 12, 2024
 *      Author: RyukiTsuji
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
//#include <math.h>

class JOYSTICK{
	private:
		ADC_HandleTypeDef* adchandle;
		uint16_t adc_xy[2] = {2047, 2047};

	public:
		JOYSTICK(ADC_HandleTypeDef* ptr_adchandle);
		void sampling();
		void get_adcValue(uint16_t* adc_array);
};


#endif /* INC_JOYSTICK_H_ */
