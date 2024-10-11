#include "joystick.h"

JOYSTICK::JOYSTICK(ADC_HandleTypeDef* ptr_adchandle){
	adchandle = ptr_adchandle;
}

void JOYSTICK::sampling(){
	for(int i=0; i<2; i++){
	  HAL_ADC_Start(adchandle);
	  HAL_ADC_PollForConversion(adchandle, 1);
	  adc_xy[i] = HAL_ADC_GetValue(adchandle);
	}
}

void JOYSTICK::get_adcValue(uint16_t* adc_array){
	for(int i=0; i<2; i++){
	  adc_array[i] = adc_xy[i];
	}
}



