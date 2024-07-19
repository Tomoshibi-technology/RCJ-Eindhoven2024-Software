




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
	uint8_t* camera_array;
	int16_t* movement_array;
	uint8_t* display_array;

	int16_t first_position_array[2];

	int16_t* circle_position_array;
	int16_t* position_array;

	uint32_t first_time;
	uint32_t p_time;
	uint16_t p_count;
	uint16_t first_count;
	uint16_t p_beat;
	uint16_t first_beat;

	int8_t katamuki = 1;
	uint8_t flame_H_goal;

	uint8_t shdn_flag = 0;
	uint8_t fix_flag = 0;
	uint8_t mode_flag[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	uint8_t r_standard = 10;

	uint8_t S_standard = 251;
	uint8_t V_safety = 50;
	uint8_t V_standard = 5;
	uint8_t V_hard = 150;

	public:
	PERFORMANCE(uint16_t* ptr_perform_array, uint8_t* ptr_camera_array, int16_t* ptr_movement_array, uint8_t* ptr_display_array, int16_t* ptr_circle_position_array, int16_t* ptr_position_array);
	void get_target_status(uint32_t time);
	int get_shutdown();
	int get_fixing();
};
