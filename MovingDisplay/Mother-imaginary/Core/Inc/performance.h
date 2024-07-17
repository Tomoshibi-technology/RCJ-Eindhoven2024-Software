




#pragma once
//ifndefにしよう

#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
//#include <stdint.h>
//#include <math.h>

class PERFORMANCE{
	private:
	uint32_t* ptr_mtime;
	uint32_t first_mtime;

	public:
	PERFORMANCE(uint32_t* counter);
	void set_first_mtime();
	void get_target_status(uint16_t* perform_array, uint32_t time, int16_t* movement_array, uint8_t* display_array);
};
