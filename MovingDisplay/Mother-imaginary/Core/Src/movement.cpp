
#include "movement.h"

MOVEMENT::MOVEMENT(uint32_t* ptr_counter){
	ptr_mtime = ptr_counter;
	first_mtime = *ptr_mtime;
}

void MOVEMENT::set_first_mtime(){
	first_mtime = *ptr_mtime;
}

void MOVEMENT::set_movement(){
	uint8_t rdpt = p_rdpt + 1;
	uint32_t past_mtime = *ptr_mtime - first_mtime;
//配列の終端でのエラー処理を書きたい
	if(past_mtime < movement_chart[rdpt][0]){
		for(int i; i<3; i++){
			movement[i] = movement_chart[p_rdpt][i+1];
		}
		p_rdpt = p_rdpt;
	}else{
		for(int i; i<3; i++){
			movement[i] = movement_chart[rdpt][i+1];
		}
		p_rdpt = rdpt;
	}
}

void MOVEMENT::get_movement(int16_t* movement_array){
	for(int i=0; i<3; i++){
		movement_array[i] = movement[i];
	}
}
