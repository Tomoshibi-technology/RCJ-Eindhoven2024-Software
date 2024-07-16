




#pragma once
//ifndefにしよう

#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
//#include <stdint.h>
//#include <math.h>

class MOVEMENT{
	private:
	uint32_t* ptr_mtime;
	uint32_t first_mtime;

	uint8_t p_rdpt = 0;

	int16_t movement[3] = {0};
	int32_t movement_chart[100][4]
		= {{0,0,0,0},{3000,0,0,0},{6000,0,0,0},{10000,0,0,0}};

	public:
	MOVEMENT(uint32_t* counter);
	void set_first_mtime();
	void set_movement();
	void get_movement(int16_t* movement_array);




};
