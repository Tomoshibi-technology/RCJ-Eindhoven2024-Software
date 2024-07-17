
#include "performance.h"

PERFORMANCE::PERFORMANCE(uint32_t* ptr_counter){
	ptr_mtime = ptr_counter;
	first_mtime = *ptr_mtime;
}

void PERFORMANCE::set_first_mtime(){
	first_mtime = *ptr_mtime;
}

void PERFORMANCE::get_target_status(uint16_t* perform_array, uint32_t time, int16_t* movement_array, uint8_t* display_array){
	if(perform_array[0] == 0){
		movement_array[0] = 0;
		movement_array[1] = 0;
		movement_array[2] = 0;
	}else if(perform_array[0] == 1){
		movement_array[0] = 0;
		movement_array[1] = 0;
		movement_array[2] = 0;
	}else if(perform_array[0] == 2){
		movement_array[0] = 500;
		movement_array[1] = 0;
		movement_array[2] = 200;
	}

//	if(time >= 0 && time < 1000){
//		movement_array[0] = 0;
//		movement_array[1] = 0;
//		movement_array[2] = 0;
//	}else if(time >= 1000 && time < 6000){
//		movement_array[0] = 1000;
//		movement_array[1] = 0;
//		movement_array[2] = 300;
//	}else if(time >= 6000 && time < 11000){
//		movement_array[0] = 1000;
//		movement_array[1] = 1000;
//		movement_array[2] = 300;
//	}else if(time >= 11000 && time < 16000){
//		movement_array[0] = 0;
//		movement_array[1] = 1000;
//		movement_array[2] = 300;
//	}else if(time >= 16000 && time < 21000){
//		movement_array[0] = 0;
//		movement_array[1] = 0;
//		movement_array[2] = 300;
//	}else if(time >= 21000 && time < 22000){
//		movement_array[0] = 0;
//		movement_array[1] = 0;
//		movement_array[2] = 0;
//	}else if(time >= 22000 && time < 30000){
//		movement_array[0] = 1000;
//		movement_array[1] = 1000;
//		movement_array[2] = 300;
//	}else if(time >= 30000){
//
//	}
}



