
#include "performance.h"

PERFORMANCE::PERFORMANCE(uint16_t* ptr_perform_array, uint8_t* ptr_camera_array, int16_t* ptr_movement_array, int16_t* ptr_display_array, int16_t* ptr_circle_position_array, int16_t* ptr_fish_position_array, int16_t* ptr_position_array){
	perform_array = ptr_perform_array;
	camera_array = ptr_camera_array;
	movement_array = ptr_movement_array;
	display_array = ptr_display_array;
	circle_position_array = ptr_circle_position_array;
	fish_position_array = ptr_fish_position_array;
	position_array = ptr_position_array;
}


int PERFORMANCE::get_shutdown(){
	return shdn_flag;
}

int PERFORMANCE::get_fixing(){
	return fix_flag;
}

int PERFORMANCE::get_emission(){
	return emission_flag;
}

void PERFORMANCE::get_target_status_superteam(uint32_t time){

		if(position_array[0] > 1000 && position_array[0] < 4000){

			if(katamuki == -1){
				movement_array[0] = 1000;
				movement_array[1] = 0;
				movement_array[2] = 150;
			}else if(katamuki == 1){
				movement_array[0] = 4000;
				movement_array[1] = 0;
				movement_array[2] = 150;
			}

		}else if(position_array[0] <= 1000){
			katamuki = 1;

			movement_array[0] = 4000;
			movement_array[1] = 0;
			movement_array[2] = 150;
		}else if(position_array[0] >=4000){
			katamuki = -1;

			movement_array[0] = 1000;
			movement_array[1] = 0;
			movement_array[2] = 150;
		}

		if(position_array[0] <= 1300){
			katamuki = 1;

			movement_array[0] = 4000;
			movement_array[1] = 0;
			movement_array[2] = 150;
		}else if(position_array[0] >= 3700){
			katamuki = -1;

			movement_array[0] = 1000;
			movement_array[1] = 0;
			movement_array[2] = 150;
		}





	if(perform_array[1] == 250){//ゴミ散乱 ~20
		shdn_flag = 1;
		fix_flag = 1;

		if(mode_flag[0] == 0){

//			katamuki = -1;

//			movement_array[0] = 2000;
//			movement_array[1] = 0;
//			movement_array[2] = 0;

			display_array[1] = 0;//x
			display_array[2] = 0;//y
			display_array[3] = 0;//r
			display_array[4] = 0;//circle_H
			display_array[5] = 0;//circle_S
			display_array[6] = 0;//circle_V
			display_array[7] = display_array[10] = bad_ocean;//background,frame_H
			display_array[8] = display_array[11] = S_standard;//background,frame_S
			display_array[9] = display_array[12] = V_standard;//background,frame_V
		}

		fish_position_array[0] = 1;//ID
		fish_position_array[1] = 1500;//X
		fish_position_array[2] = 150;//Y

		fish_position_array[3] = 1;//ID
		fish_position_array[4] = 1900;//X
		fish_position_array[5] = 300;//Y

		fish_position_array[6] = 1;//ID
		fish_position_array[7] = 2300;//X
		fish_position_array[8] = 150;//Y

		fish_position_array[9] = 1;//ID
		fish_position_array[10] = 2700;//X
		fish_position_array[11] = 300;//Y

		fish_position_array[12] = 1;//ID
		fish_position_array[13] = 3100;//X
		fish_position_array[14] = 150;//Y

		fish_position_array[15] = 1;//ID
		fish_position_array[16] = 3500;//X
		fish_position_array[17] = 300;//Y


		mode_flag[0] = 1;
	}else if(perform_array[1] == 1){//クリーン hueを1秒ずつ減らしていく 20~80
		shdn_flag = 1;
		fix_flag = 1;
		emission_flag = 0;

		if(mode_flag[1] == 0){
			first_time = p_time = time;

//			katamuki = -1;

//			movement_array[0] = 2000;
//			movement_array[1] = 0;
//			movement_array[2] = 0;

			display_array[7] = display_array[10] = bad_ocean;//background,frame_H
			display_array[8] = display_array[11] = S_standard;//background,frame_S
			display_array[9] = display_array[12] = V_standard;//background,frame_V
		}

		fish_position_array[0] = 2;//ID
		fish_position_array[1] = 1500;//X
		fish_position_array[2] = 150;//Y

		fish_position_array[3] = 2;//ID
		fish_position_array[4] = 1900;//X
		fish_position_array[5] = 300;//Y

		fish_position_array[6] = 2;//ID
		fish_position_array[7] = 2300;//X
		fish_position_array[8] = 150;//Y

		fish_position_array[9] = 2;//ID
		fish_position_array[10] = 2700;//X
		fish_position_array[11] = 300;//Y

		fish_position_array[12] = 2;//ID
		fish_position_array[13] = 3100;//X
		fish_position_array[14] = 150;//Y

		fish_position_array[15] = 2;//ID
		fish_position_array[16] = 3500;//X
		fish_position_array[17] = 300;//Y

		if(time - p_time >= 1000){
			display_array[7]--;
			display_array[10]--;
			p_time = time;
		}

		if(display_array[7] < 127 || display_array[10] < 127){
			display_array[7] = 127;
			display_array[10] = 127;
		}else{}

		mode_flag[1] = 1;

	}else if(time - first_time > 60){//はっぴー 80~95
		shdn_flag = 1;
		fix_flag = 1;

		if(mode_flag[2] == 0){

//			katamuki = -1;

//			movement_array[0] = 2000;
//			movement_array[1] = 0;
//			movement_array[2] = 0;

			display_array[7] = display_array[10] = clean_ocean;//background,frame_H
			display_array[8] = display_array[11] = S_standard;//background,frame_S
			display_array[9] = display_array[12] = V_standard;//background,frame_V


			display_array[4] = clean_ocean;
		}

		fish_position_array[0] = 2;//ID
		fish_position_array[1] = 1500;//X
		fish_position_array[2] = 150;//Y

		fish_position_array[3] = 2;//ID
		fish_position_array[4] = 1900;//X
		fish_position_array[5] = 300;//Y

		fish_position_array[6] = 2;//ID
		fish_position_array[7] = 2300;//X
		fish_position_array[8] = 150;//Y

		fish_position_array[9] = 2;//ID
		fish_position_array[10] = 2700;//X
		fish_position_array[11] = 300;//Y

		fish_position_array[12] = 2;//ID
		fish_position_array[13] = 3100;//X
		fish_position_array[14] = 150;//Y

		fish_position_array[15] = 2;//ID
		fish_position_array[16] = 3500;//X
		fish_position_array[17] = 300;//Y



		mode_flag[2] = 1;
	}else if(perform_array[1] == 0){//ぐっど 95~135
		shdn_flag = 1;
		fix_flag = 1;


		if(mode_flag[3] == 0){
//
//			katamuki = -1;
//
//			movement_array[0] = 2000;
//			movement_array[1] = 0;
//			movement_array[2] = 0;

			display_array[7] = display_array[10] = clean_ocean;//background,frame_H
			display_array[8] = display_array[11] = S_standard;//background,frame_S
			display_array[9] = display_array[12] = V_standard;//background,frame_V
		}

		fish_position_array[0] = 2;//ID
		fish_position_array[1] = 1500;//X
		fish_position_array[2] = 150;//Y

		fish_position_array[3] = 2;//ID
		fish_position_array[4] = 1900;//X
		fish_position_array[5] = 300;//Y

		fish_position_array[6] = 2;//ID
		fish_position_array[7] = 2300;//X
		fish_position_array[8] = 150;//Y

		fish_position_array[9] = 2;//ID
		fish_position_array[10] = 2700;//X
		fish_position_array[11] = 300;//Y

		fish_position_array[12] = 2;//ID
		fish_position_array[13] = 3100;//X
		fish_position_array[14] = 150;//Y

		fish_position_array[15] = 2;//ID
		fish_position_array[16] = 3500;//X
		fish_position_array[17] = 300;//Y

		mode_flag[3] = 1;

	}else if(perform_array[1] == 2){//ばいばい 135~155
		shdn_flag = 1;
		fix_flag = 1;

		if(mode_flag[4] == 0){

//			katamuki = -1;
//
//			movement_array[0] = 2000;
//			movement_array[1] = 0;
//			movement_array[2] = 0;

			display_array[7] = display_array[10] = clean_ocean;//background,frame_H
			display_array[8] = display_array[11] = S_standard;//background,frame_S
			display_array[9] = display_array[12] = V_standard;//background,frame_V
		}

		fish_position_array[0] = 2;//ID
		fish_position_array[1] = 1500;//X
		fish_position_array[2] = 150;//Y

		fish_position_array[3] = 2;//ID
		fish_position_array[4] = 1900;//X
		fish_position_array[5] = 300;//Y

		fish_position_array[6] = 2;//ID
		fish_position_array[7] = 2300;//X
		fish_position_array[8] = 150;//Y

		fish_position_array[9] = 2;//ID
		fish_position_array[10] = 2700;//X
		fish_position_array[11] = 300;//Y

		fish_position_array[12] = 2;//ID
		fish_position_array[13] = 3100;//X
		fish_position_array[14] = 150;//Y

		fish_position_array[15] = 2;//ID
		fish_position_array[16] = 3500;//X
		fish_position_array[17] = 300;//Y

		mode_flag[4] = 1;
	}
}









void PERFORMANCE::get_target_status(uint32_t time){

	if(perform_array[0] == 0){
		shdn_flag = 1;
		emission_flag = 1;

//		movement_array[0] = 2000;
//		movement_array[1] = 0;
//		movement_array[2] = 200;
//
//		display_array[1] = 0;//x
//		display_array[2] = 0;//y
//		display_array[3] = 0;//r
//		display_array[4] = 0;;//circle_H
//		display_array[5] = 0;//circle_S
//		display_array[6] = 0;//circle_V
////		display_array[7] = perform_array[3];//background_H
//		display_array[7] = *camera_Hue;
//		display_array[8] = S_standard;//background_S
//		display_array[9] = V_standard;//background_V
//		display_array[10] = perform_array[3];//frame_H
//		display_array[11] = S_standard;//frame_S
//		display_array[12] = V_standard;//frame_V

		fix_flag = 1;
		circle_position_array[0] = 0;
		circle_position_array[1] = 24 * 10;
		circle_position_array[2] = r_standard * 10;//r

		movement_array[0] = 2000;
		movement_array[1] = 0;
		movement_array[2] = 200;

		display_array[1] = 24;//x
		display_array[2] = 24;//y
		display_array[3] = r_standard;//r
		display_array[4] = perform_array[3];//circle_H
		display_array[5] = S_standard;//circle_S
		display_array[6] = V_standard;//circle_V
		display_array[7] = perform_array[3];//background_H
		display_array[8] = S_standard;//background_S
		display_array[9] = V_standard;//background_V
		display_array[10] = perform_array[3];//frame_H
		display_array[11] = S_standard;//frame_S
		display_array[12] = V_standard;//frame_V


		if(camera_array[1] == 1){display_array[4] = circle_H_standard;}
		else{display_array[4] = perform_array[3];}



		mode_flag[0] = 0;
		mode_flag[1] = 0;
		mode_flag[2] = 0;
		mode_flag[3] = 0;
		mode_flag[4] = 0;
		mode_flag[5] = 0;
		mode_flag[6] = 0;
		mode_flag[7] = 0;
		mode_flag[8] = 0;
		mode_flag[9] = 0;
		mode_flag[10] = 0;
		mode_flag[11] = 0;
		mode_flag[12] = 0;
		mode_flag[13] = 0;
		mode_flag[14] = 0;
		mode_flag[15] = 0;
		mode_flag[16] = 0;




	}else if(perform_array[0] == 1){//はじまり　1~15
		shdn_flag = 1;
		emission_flag = 0;

		if(mode_flag[1] == 0){
			fix_flag = 0;
			katamuki = 1;
			p_beat = first_beat = perform_array[2];
			p_count = first_count = perform_array[1];

			movement_array[0] = 2000;
			movement_array[1] = 0;
			movement_array[2] = 200;

			display_array[1] = 24;//x
			display_array[2] = 24;//y
			display_array[3] = r_standard;//r
			display_array[4] = perform_array[3];//circle_H
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		display_array[7] = perform_array[3];
		display_array[10] = perform_array[3];

		if(camera_array[1] == 1){display_array[4] = circle_H_standard;}
		else{display_array[4] = perform_array[3];}

//		if(perform_array[2] >= 8){
//			display_array[4] = 0;
//		}else{
//			display_array[4] = perform_array[3];
//		}

		if(perform_array[2] >= 8 && (perform_array[1] - p_count) >= 1){
			if(display_array[3] > 4 && display_array[3] < 20 && katamuki == -1){
				display_array[3] -= 4;
			}else if(display_array[3] > 4 && display_array[3] < 20 && katamuki == 1){
				display_array[3] += 4;
			}else if(display_array[3] <= 4){
				katamuki = 1;
				display_array[3] += 4;
			}else if(display_array[3] >= 20){
				katamuki = -1;
				display_array[3] -= 4;
			}
			p_count = perform_array[1];
		}

		mode_flag[1] = 1;




	}else if(perform_array[0] == 2){//アームに渡しにいく 16~47
		shdn_flag = 1;
		emission_flag = 0;

		if(mode_flag[2] == 0){
			fix_flag = 0;

			p_beat = first_beat = perform_array[2];
			p_count = first_count = perform_array[1];

			flame_H_goal = perform_array[3] + 128;//枠表示
				flame_H_goal %= 256;

			first_position_array[0] = position_array[0];
			first_position_array[1] = position_array[1];

			movement_array[0] = 2000;
			movement_array[1] = 0;
			movement_array[2] = 200;

//			display_array[1] = 0;//x
//			display_array[2] = 0;//y
//			display_array[3] = 0;//r
			display_array[4] = circle_H_standard;//circle_H
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		display_array[7] = perform_array[3];

		if(perform_array[2] >= 16 && perform_array[2] < 20 && (perform_array[1] - p_count) >= 2){
			if(display_array[3] < r_standard){//半径調整
				display_array[3]++;
			}else if(display_array[3] > r_standard){
				display_array[3]--;
			}else{display_array[3] = r_standard;}
			p_count = perform_array[1];
		}else if(perform_array[2] >= 20 && perform_array[2] < 24 && (perform_array[1] - p_count) >= 1){
			display_array[10] = perform_array[3] + 128;//枠表示
				display_array[10] %= 256;
//			if(display_array[10] != flame_H_goal){
//				display_array[10]++;
//				display_array[10] %= 256;
//			}else{
//				display_array[10] = flame_H_goal;
//			}
//			p_count = perform_array[1];
		}else if(perform_array[2] >= 24 && perform_array[2] < 28){
			circle_position_array[0] = first_position_array[0];
			circle_position_array[1] = 24 * 10;
			circle_position_array[2] = r_standard * 10;//r
			fix_flag = 1;

			movement_array[0] = 2400;//反対に移動
			movement_array[1] = 0;
			movement_array[2] = 200;

		}else if(perform_array[2] >= 28 && perform_array[2] < 42){

			movement_array[0] = 500;//移動はじめ
			movement_array[1] = 700;
			movement_array[2] = 200;

			if(circle_position_array[0] - position_array[0] < 120){
				circle_position_array[0] = first_position_array[0];
				circle_position_array[1] = 24 * 10;
				circle_position_array[2] = r_standard * 10;//r
				fix_flag = 1;
			}else if(circle_position_array[0] - position_array[0] >= 120){//枠にひっぱられる
				fix_flag = 0;
				display_array[1] = 35;
				display_array[2] = 24;
				display_array[3] = r_standard;
			}
			p_count = perform_array[1];
		}else if(perform_array[2] >= 42 && perform_array[2] < 44 && abs(movement_array[0] - position_array[0]) <= 200){//魂あげるために止まる
			fix_flag = 0;
			if((perform_array[1] - p_count) >= 2){
				if(display_array[1] > 24){
					display_array[1]--;
					display_array[2] = 24;
					display_array[3] = r_standard;
				}else{
					display_array[1] = 24;
					display_array[2] = 24;
					display_array[3] = r_standard;
				}
				p_count = perform_array[1];
			}
		}else if(perform_array[2] >= 44){//魂渡す
			fix_flag = 0;
			if((perform_array[1] - p_count) >= 1){
				display_array[2]-= 3;
				p_count = perform_array[1];
			}
		}

		mode_flag[2] = 1;




////やすみ
	}else if(perform_array[0] == 3){//アーム 48~79
		shdn_flag = 0;
		emission_flag = 0;
		mode_flag[3] = 1;//

//		movement_array[0] = 500;
//		movement_array[1] = 0;
//		movement_array[2] = 200;

		display_array[1] = 0;//x
		display_array[2] = 0;//y
		display_array[3] = 0;//r
		display_array[4] = 0;//circle_H
		display_array[5] = 0;//circle_S
		display_array[6] = 0;//circle_V
		display_array[7] = 0;//background_H
		display_array[8] = 0;//background_S
		display_array[9] = 0;//background_V
		display_array[10] = 0;//frame_H
		display_array[11] = 0;//frame_S
		display_array[12] = 0;//frame_V




	}else if(perform_array[0] == 4){//アーム 80~111
		shdn_flag = 0;
		emission_flag = 0;
		mode_flag[4] = 1;//

//		movement_array[0] = 500;
//		movement_array[1] = 0;
//		movement_array[2] = 200;

		display_array[1] = 0;//x
		display_array[2] = 0;//y
		display_array[3] = 0;//r
		display_array[4] = 0;//circle_H
		display_array[5] = 0;//circle_S
		display_array[6] = 0;//circle_V
		display_array[7] = 0;//background_H
		display_array[8] = 0;//background_S
		display_array[9] = 0;//background_V
		display_array[10] = 0;//frame_H
		display_array[11] = 0;//frame_S
		display_array[12] = 0;//frame_V




	}else if(perform_array[0] == 5){//ポール 112~127
		shdn_flag = 0;
		emission_flag = 0;
		mode_flag[5] = 1;

//		movement_array[0] = 500;
//		movement_array[1] = 0;
//		movement_array[2] = 200;

		display_array[1] = 0;//x
		display_array[2] = 0;//y
		display_array[3] = 0;//r
		display_array[4] = 0;//circle_H
		display_array[5] = 0;//circle_S
		display_array[6] = 0;//circle_V
		display_array[7] = 0;//background_H
		display_array[8] = 0;//background_S
		display_array[9] = 0;//background_V
		display_array[10] = 0;//frame_H
		display_array[11] = 0;//frame_S
		display_array[12] = 0;//frame_V




	}else if(perform_array[0] == 6){//ポール 128~142
		shdn_flag = 1;
		emission_flag = 0;
		fix_flag = 0;

		if(mode_flag[6] == 0){
//			movement_array[0] = 500;
//			movement_array[1] = 0;
//			movement_array[2] = 200;

			display_array[1] = 0;//x
			display_array[2] = 0;//y
			display_array[3] = 0;//r
			display_array[4] = 0;//circle_H
			display_array[5] = 0;//circle_S
			display_array[6] = 0;//circle_V
			display_array[7] = 0;//background_H
			display_array[8] = 0;//background_S
			display_array[9] = 0;//background_V
			display_array[10] = 0;//frame_H
			display_array[11] = 0;//frame_S
			display_array[12] = 0;//frame_V
		}
////やすみおわり



		if(perform_array[2] >= 136){
			movement_array[0] = 2000;
			movement_array[1] = 0;
			movement_array[2] = 200;

			display_array[1] = 24;//x
			display_array[2] = 24;//y
			display_array[3] = r_standard;//r
			display_array[4] = circle_H_standard;//circle_H
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		display_array[7] = perform_array[3];
		display_array[10] = perform_array[3];

		mode_flag[6] = 1;



////バグりはじめる

	}else if(perform_array[0] == 7){//すなあらあし 143~158
		fix_flag = 0;
		shdn_flag = 1;
		emission_flag = 0;

		if(mode_flag[7] == 0){
			p_beat = first_beat = perform_array[2];
			p_count = first_count = perform_array[1];

			katamuki = 1;

			movement_array[0] = 2000;
			movement_array[1] = 0;
			movement_array[2] = 200;

			display_array[1] = 24;//x
			display_array[2] = 24;//y
			display_array[3] = r_standard;//r
			display_array[4] = circle_H_standard;//circle_H
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		if(perform_array[2] >= 144){
			shdn_flag = 0;
			display_array[4] += 30;
				display_array[4] %= 256;
			display_array[7] += 30;
				display_array[4] %= 256;
			display_array[10] += 30;
				display_array[4] %= 256;
			display_array[4] = perform_array[3] + 128;//circle_H
				display_array[4] %= 256;
		}


		if(perform_array[2] >= 144 && (perform_array[1] - p_count) >= 1){
			if(display_array[6] > 0 && display_array[6] < 100 && katamuki == -1){
				display_array[6] -= 4;
			}else if(display_array[3] > 4 && display_array[6] < 100 && katamuki == 1){
				display_array[6] += 4;
			}else if(display_array[6] <= 0){
				katamuki = 1;
				display_array[6] += 4;
			}else if(display_array[6] >= 100){
				katamuki = -1;
				display_array[6] -= 4;
			}
			p_count = perform_array[1];
		}

		mode_flag[7] = 1;




	}else if(perform_array[0] == 8){//しぬ 159~
		shdn_flag = 0;
		emission_flag = 1;
		mode_flag[8] = 1;
		fix_flag = 0;

//		movement_array[0] = 0;
//		movement_array[1] = 0;
		movement_array[2] = 0;

		display_array[1] = 0;//x
		display_array[2] = 0;//y
		display_array[3] = 0;//r
		display_array[4] = 0;//circle_H
		display_array[5] = 0;//circle_S
		display_array[6] = 0;//circle_V
		display_array[7] = 0;//background_H
		display_array[8] = 0;//background_S
		display_array[9] = 0;//background_V
		display_array[10] = 0;//frame_H
		display_array[11] = 0;//frame_S
		display_array[12] = 0;//frame_V




	}else if(perform_array[0] == 9){//たまいれる 15s
		shdn_flag = 0;
		emission_flag = 0;
		mode_flag[9] = 1;
		fix_flag = 0;

		p_beat = first_beat = perform_array[2];
		p_count = first_count = perform_array[1];

//		movement_array[0] = 500;
//		movement_array[1] = 0;
		movement_array[2] = 0;

		display_array[1] = 0;//x
		display_array[2] = 0;//y
		display_array[3] = 0;//r
		display_array[4] = 0;//circle_H
		display_array[5] = 0;//circle_S
		display_array[6] = 0;//circle_V
		display_array[7] = 0;//background_H
		display_array[8] = 0;//background_S
		display_array[9] = 0;//background_V
		display_array[10] = 0;//frame_H
		display_array[11] = 0;//frame_S
		display_array[12] = 0;//frame_V

////バグ終了


	}else if(perform_array[0] == 10){//チャージ 1~17
		shdn_flag = 1;
		emission_flag = 0;
		fix_flag = 0;

//		movement_array[0] = 500;
//		movement_array[1] = 0;
		movement_array[2] = 0;

		if(mode_flag[10] == 0){
			p_beat = first_beat = perform_array[2];
			p_count = first_count = perform_array[1];

			display_array[1] = 24;//x
			display_array[2] = 24;//y
			display_array[3] = 0;//r
			display_array[4] = 20;//circle_H
			display_array[5] = S_standard;//circle_S
			display_array[6] = 0;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = 0;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = 0;//frame_V
		}

		if((perform_array[1] - p_count) >= 12){
			if(display_array[6] < V_standard){
				display_array[6] += 1;
			}
			p_count = perform_array[1];
		}

		if(display_array[3] < 70){
			display_array[3] ++;
		}else{
			display_array[3] = 0;
			display_array[4] += 35;
		}

		mode_flag[10] = 1;

////サビ



	}else if(perform_array[0] == 11 || perform_array[0] == 12 || perform_array[0] == 13){//もりあがり 18~47 48~73
		shdn_flag = 1;
		fix_flag = 1;
		emission_flag = 0;


		if(mode_flag[11] == 0 || mode_flag[12] == 0){
			p_beat = first_beat = perform_array[2];
			p_count = first_count = perform_array[1];

			katamuki = -1;

			movement_array[0] = 2000;
			movement_array[1] = 0;
			movement_array[2] = 0;

			display_array[1] = 24;//x
			display_array[2] = 24;//y
			display_array[3] = r_standard;//r
			display_array[4] = perform_array[3] + 128;//circle_H
				display_array[4] %= 256;
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		display_array[4] = perform_array[3] + 128;//circle_H
			display_array[4] %= 256;
		display_array[7] = perform_array[3];
		display_array[10] = perform_array[3];

		if(position_array[0] > 1000 && position_array[0] < 4000){

			if(katamuki == -1){
				movement_array[0] = 1000;
				movement_array[1] = 0;
				movement_array[2] = 300;
			}else if(katamuki == 1){
				movement_array[0] = 4000;
				movement_array[1] = 0;
				movement_array[2] = 300;
			}

			circle_position_array[0] = 2500;
			circle_position_array[1] = 24 * 10;
			circle_position_array[2] = r_standard * 10;//r
		}else if(position_array[0] <= 1000){
			katamuki = 1;

			movement_array[0] = 4000;
			movement_array[1] = 0;
			movement_array[2] = 300;
		}else if(position_array[0] >=4000){
			katamuki = -1;

			movement_array[0] = 1000;
			movement_array[1] = 0;
			movement_array[2] = 300;
		}

		if(position_array[0] <= 1300){
			katamuki = 1;

			movement_array[0] = 4000;
			movement_array[1] = 0;
			movement_array[2] = 300;
		}else if(position_array[0] >= 3700){
			katamuki = -1;

			movement_array[0] = 1000;
			movement_array[1] = 0;
			movement_array[2] = 300;
		}

		mode_flag[11] = 1;
		mode_flag[12] = 1;






//
////工事中
//
//	}else if(perform_array[0] == 11){//もりあがり 18~47 48~73
//		shdn_flag = 1;
//		fix_flag = 1;
//		emission_flag = 0;
//
//
//		if(mode_flag[11] == 0 || mode_flag[12] == 0){
//			p_beat = first_beat = perform_array[2];
//			p_count = first_count = perform_array[1];
//
//			katamuki = -1;
//
//			movement_array[0] = 2000;
//			movement_array[1] = 0;
//			movement_array[2] = 0;
//
//			display_array[1] = 24;//x
//			display_array[2] = 24;//y
//			display_array[3] = r_standard;//r
//			display_array[4] = perform_array[3] + 128;//circle_H
//				display_array[4] %= 256;
//			display_array[5] = S_standard;//circle_S
//			display_array[6] = V_standard;//circle_V
//			display_array[7] = perform_array[3];//background_H
//			display_array[8] = S_standard;//background_S
//			display_array[9] = V_standard;//background_V
//			display_array[10] = perform_array[3];//frame_H
//			display_array[11] = S_standard;//frame_S
//			display_array[12] = V_standard;//frame_V
//		}
//
//		display_array[4] = perform_array[3] + 128;//circle_H
//			display_array[4] %= 256;
//		display_array[7] = perform_array[3];
//		display_array[10] = perform_array[3];
//
//		if(position_array[0] > 1000 && position_array[0] < 4000){
//
//			if(katamuki == -1){
//				movement_array[0] = 1000;
//				movement_array[1] = 0;
//				movement_array[2] = 300;
//			}else if(katamuki == 1){
//				movement_array[0] = 4000;
//				movement_array[1] = 0;
//				movement_array[2] = 300;
//			}
//
//			circle_position_array[0] = 2500;
//			circle_position_array[1] = 24 * 10;
//			circle_position_array[2] = r_standard * 10;//r
//		}else if(position_array[0] <= 1000){
//			katamuki = 1;
//
//			movement_array[0] = 4000;
//			movement_array[1] = 0;
//			movement_array[2] = 300;
//		}else if(position_array[0] >=4000){
//			katamuki = -1;
//
//			movement_array[0] = 1000;
//			movement_array[1] = 0;
//			movement_array[2] = 300;
//		}
//
//		if(position_array[0] <= 1300){
//			katamuki = 1;
//
//			movement_array[0] = 4000;
//			movement_array[1] = 0;
//			movement_array[2] = 300;
//		}else if(position_array[0] >= 3700){
//			katamuki = -1;
//
//			movement_array[0] = 1000;
//			movement_array[1] = 0;
//			movement_array[2] = 300;
//		}
//
//		mode_flag[11] = 1;
//		mode_flag[12] = 1;
//
//
//	}else if(perform_array[0] == 12){//もりあがり 18~47 48~73
//			shdn_flag = 1;
//			fix_flag = 1;
//			emission_flag = 0;
//
//
//			if(mode_flag[11] == 0 || mode_flag[12] == 0){
//				p_beat = first_beat = perform_array[2];
//				p_count = first_count = perform_array[1];
//
//				katamuki = -1;
//
//				movement_array[0] = 2000;
//				movement_array[1] = 0;
//				movement_array[2] = 0;
//
//				display_array[1] = 24;//x
//				display_array[2] = 24;//y
//				display_array[3] = r_standard;//r
//				display_array[4] = perform_array[3] + 128;//circle_H
//					display_array[4] %= 256;
//				display_array[5] = S_standard;//circle_S
//				display_array[6] = V_standard;//circle_V
//				display_array[7] = perform_array[3];//background_H
//				display_array[8] = S_standard;//background_S
//				display_array[9] = V_standard;//background_V
//				display_array[10] = perform_array[3];//frame_H
//				display_array[11] = S_standard;//frame_S
//				display_array[12] = V_standard;//frame_V
//			}
//
//			display_array[4] = perform_array[3] + 128;//circle_H
//				display_array[4] %= 256;
//			display_array[7] = perform_array[3];
//			display_array[10] = perform_array[3];
//
//			if(position_array[0] > 1000 && position_array[0] < 4000){
//
//				if(katamuki == -1){
//					movement_array[0] = 1000;
//					movement_array[1] = 0;
//					movement_array[2] = 300;
//				}else if(katamuki == 1){
//					movement_array[0] = 4000;
//					movement_array[1] = 0;
//					movement_array[2] = 300;
//				}
//
//				circle_position_array[0] = 2500;
//				circle_position_array[1] = 24 * 10;
//				circle_position_array[2] = r_standard * 10;//r
//			}else if(position_array[0] <= 1000){
//				katamuki = 1;
//
//				movement_array[0] = 4000;
//				movement_array[1] = 0;
//				movement_array[2] = 300;
//			}else if(position_array[0] >=4000){
//				katamuki = -1;
//
//				movement_array[0] = 1000;
//				movement_array[1] = 0;
//				movement_array[2] = 300;
//			}
//
//			if(position_array[0] <= 1300){
//				katamuki = 1;
//
//				movement_array[0] = 4000;
//				movement_array[1] = 0;
//				movement_array[2] = 300;
//			}else if(position_array[0] >= 3700){
//				katamuki = -1;
//
//				movement_array[0] = 1000;
//				movement_array[1] = 0;
//				movement_array[2] = 300;
//			}
//
//			mode_flag[11] = 1;
//			mode_flag[12] = 1;
//
//
////工事中
//











//
//	}else if(perform_array[0] == 13){//盛り上がりの最後 74~80
//		shdn_flag = 1;
//		emission_flag = 0;
//
//
////		movement_array[0] = 500;
////		movement_array[1] = 0;
////		movement_array[2] = 200;
//
//		display_array[1] = 0;//x
//		display_array[2] = 0;//y
//		display_array[3] = 0;//r
//		display_array[4] = perform_array[3] + 128;//circle_H
//			display_array[4] %= 256;
//		display_array[5] = S_standard;//circle_S
//		display_array[6] = V_standard;//circle_V
//		display_array[7] = perform_array[3];//background_H
//		display_array[8] = S_standard;//background_S
//		display_array[9] = V_standard;//background_V
//		display_array[10] = perform_array[3];//frame_H
//		display_array[11] = S_standard;//frame_S
//		display_array[12] = V_standard;//frame_V
//
//		mode_flag[13] = 1;




	}else if(perform_array[0] == 14){//フェードアウト 81~94
		shdn_flag = 1;
		emission_flag = 0;

		if(mode_flag[14] == 0){
			p_beat = first_beat = perform_array[2];
			p_count = fadeout_p_count = first_count = perform_array[1];

			katamuki = 1;

			movement_array[0] = 2000;
			movement_array[1] = 0;
			movement_array[2] = 200;

			display_array[1] = 24;//x
			display_array[2] = 24;//y
			display_array[3] = r_standard;//r
			display_array[4] = perform_array[3] + 128;//circle_H
				display_array[4] %= 256;
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		display_array[7] = perform_array[3];
		display_array[10] = perform_array[3];


		if((perform_array[1] - p_count) >= 1){
			if(display_array[3] > 4 && display_array[3] < 20 && katamuki == -1){
				display_array[3] -= 4;
			}else if(display_array[3] > 4 && display_array[3] < 20 && katamuki == 1){
				display_array[3] += 4;
			}else if(display_array[3] <= 4){
				katamuki = 1;
				display_array[3] += 4;
			}else if(display_array[3] >= 20){
				katamuki = -1;
				display_array[3] -= 4;
			}
			p_count = perform_array[1];
		}

		if((perform_array[1] - fadeout_p_count) >= 12){
			if(display_array[6] > 0){
				display_array[6] -= 1;
			}else{display_array[6] = 0;}
			fadeout_p_count = perform_array[1];
		}


//		if((perform_array[1] - p_count) >= 1){
//			if(display_array[3] > 4 && display_array[3] < 20 && katamuki == -1){
//				display_array[3]--;
//			}else if(display_array[3] > 4 && display_array[3] < 20 && katamuki == 1){
//				display_array[3]++;
//			}else if(display_array[3] <= 4 && (perform_array[2] - p_beat) >= 2){
//				katamuki = 1;
//				display_array[3]++;
//				p_beat = perform_array[2];
//			}else if(display_array[3] >= 20 && (perform_array[2] - p_beat) >= 2){
//				katamuki = -1;
//				display_array[3]--;
//				p_beat = perform_array[2];
//			}
//			if(display_array[6] != 0){
//				display_array[6]--;
//			}
//			if(display_array[9] != 0){
//				display_array[9]--;
//			}
//			if(display_array[12] != 0){
//				display_array[12]--;
//			}
//
//			p_count = perform_array[1];
//		}

		mode_flag[14] = 1;




	}else if(perform_array[0] == 15){//余韻 95~105
		shdn_flag = 0;
		emission_flag = 0;
		mode_flag[15] = 1;

//		movement_array[0] = 500;
//		movement_array[1] = 0;
//		movement_array[2] = 200;

		display_array[1] = 0;//x
		display_array[2] = 0;//y
		display_array[3] = 0;//r
		display_array[4] = 0;//circle_H
		display_array[5] = 0;//circle_S
		display_array[6] = 0;//circle_V
		display_array[7] = 0;//background_H
		display_array[8] = 0;//background_S
		display_array[9] = 0;//background_V
		display_array[10] = 0;//frame_H
		display_array[11] = 0;//frame_S
		display_array[12] = 0;//frame_V




	}else if(perform_array[0] == 16){//おわり 106~
		shdn_flag = 0;
		emission_flag = 0;
		mode_flag[16] = 1;

//		movement_array[0] = 500;
//		movement_array[1] = 0;
//		movement_array[2] = 200;

		display_array[1] = 0;//x
		display_array[2] = 0;//y
		display_array[3] = 0;//r
		display_array[4] = 0;//circle_H
		display_array[5] = 0;//circle_S
		display_array[6] = 0;//circle_V
		display_array[7] = 0;//background_H
		display_array[8] = 0;//background_S
		display_array[9] = 0;//background_V
		display_array[10] = 0;//frame_H
		display_array[11] = 0;//frame_S
		display_array[12] = 0;//frame_V

	}
}



