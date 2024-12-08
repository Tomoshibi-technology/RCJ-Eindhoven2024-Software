




#pragma once
//ifndefにしよう

#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
//#include <stdint.h>
#include <math.h>

class PERFORMANCE{
	private:
	uint16_t* perform_array;
	int16_t* movement_array;
	int16_t* display_array;

	int16_t* circle_relative_position_array;
	int16_t* fish_relative_position_array;
	int16_t* now_position_array;


	int16_t circle_position_array[3];
	int16_t fish_position_array[18];
	int16_t line_position_array[2];


	uint8_t r_standard = 10;

	uint8_t circle_H_standard = 0;
	uint8_t S_standard = 251;
	uint8_t V_safety = 50;
	uint8_t V_standard = 7;
	uint8_t V_hard = 150;


	uint16_t shrink_const = 700;

	public:
	PERFORMANCE(uint16_t* ptr_perform_array, int16_t* ptr_display_array, int16_t* ptr_circle_relative_position_array, int16_t* ptr_fish_relative_position_array, int16_t* ptr_now_position_array);
	void cal_drawing_status_performance(uint32_t count);
	void get_drawing_status_performance();
};
