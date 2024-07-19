
#include "performance.h"

PERFORMANCE::PERFORMANCE(uint16_t* ptr_perform_array, uint8_t* ptr_camera_array, int16_t* ptr_movement_array, uint8_t* ptr_display_array, int16_t* ptr_circle_position_array, int16_t* ptr_position_array){
	perform_array = ptr_perform_array;
	camera_array = ptr_camera_array;
	movement_array = ptr_movement_array;
	display_array = ptr_display_array;
	circle_position_array = ptr_circle_position_array;
	position_array = ptr_position_array;
}


int PERFORMANCE::get_shutdown(){
	return shdn_flag;
}

int PERFORMANCE::get_fixing(){
	return fix_flag;
}

void PERFORMANCE::get_target_status(uint32_t time){

	if(perform_array[0] == 0){
		shdn_flag = 1;

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


		if(camera_array[1] == 1){display_array[4] = camera_array[0];}
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

		if(camera_array[1] == 1){display_array[4] = 20;}
		else{display_array[4] = perform_array[3];}

//		if(perform_array[2] >= 8){
//			display_array[4] = 0;
//		}else{
//			display_array[4] = perform_array[3];
//		}

		if((perform_array[2] >= 8 && perform_array[1] - p_count) >= 1){
			if(display_array[3] > 4 && display_array[3] < 20 && katamuki == -1){
				display_array[3] -= 2;
			}else if(display_array[3] > 4 && display_array[3] < 20 && katamuki == 1){
				display_array[3] += 2;
			}else if(display_array[3] <= 4){
				katamuki = 1;
				display_array[3] += 2;
//				p_beat = perform_array[2];
			}else if(display_array[3] >= 20){
				katamuki = -1;
				display_array[3] -= 2;
//				p_beat = perform_array[2];
			}
			p_count = perform_array[1];
		}

		mode_flag[1] = 1;




	}else if(perform_array[0] == 2){//アームに渡しににいく 16~47
		shdn_flag = 1;

		if(mode_flag[2] == 0){
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
			display_array[4] = 0;//circle_H
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		if(perform_array[2] >= 16 && perform_array[2] < 20 && (perform_array[1] - p_count) >= 2){
			if(display_array[3] < r_standard){//半径調整
				display_array[3]++;
			}else if(display_array[3] > r_standard){
				display_array[3]--;
			}else{}
			p_count = perform_array[1];
		}else if(perform_array[2] >= 20 && perform_array[2] < 24 && (perform_array[1] - p_count) >= 1){
//			display_array[10] = perform_array[3] + 128;//枠表示
//				display_array[10] %= 256;
			if(display_array[10] != flame_H_goal){
				display_array[10]++;
				display_array[10] %= 256;
			}else{
				display_array[10] = flame_H_goal;
			}
			p_count = perform_array[1];
		}else if(perform_array[2] >= 24){
			movement_array[0] = 500;//移動はじめ
			movement_array[1] = 700;
			movement_array[2] = 200;

			circle_position_array[0] = first_position_array[0];
			circle_position_array[1] = 24 * 10;
			circle_position_array[2] = r_standard * 10;//r

			if(circle_position_array[0] - position_array[0] < 120){
				fix_flag = 1;
			}else if(circle_position_array[0] - position_array[0] >= 120){//枠にひっぱられる
				fix_flag = 0;
				display_array[1] = 35;
				display_array[2] = 24;
				display_array[3] = r_standard;
			}
			p_count = perform_array[1];
		}else if(position_array[0] <= 600 && perform_array[2] - first_beat > 8){//魂あげるために止まる
			fix_flag = 0;
			if((perform_array[1] - p_count) >= 2){
				if(display_array[1] >= 24){
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
		}
//			else if(perform_array[2] > 44){
//			fix_flag = 0;
//			display_array[2]--;
//		}

//こっからさらに，魂を渡す演出



		mode_flag[2] = 1;




////やすみ
	}else if(perform_array[0] == 3){//アーム 48~79
		shdn_flag = 0;
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

		mode_flag[6] = 1;



////バグりはじめる

	}else if(perform_array[0] == 7){//すなあらあし 143~158
		shdn_flag = 1;

		if(mode_flag[7] == 0){
			movement_array[0] = 2000;
			movement_array[1] = 0;
			movement_array[2] = 200;

			display_array[1] = 0;//x
			display_array[2] = 0;//y
			display_array[3] = 0;//r
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

		if(perform_array[2] >= 144){
			shdn_flag = 0;
			display_array[4] ++;
				display_array[4] %= 256;
			display_array[7] ++;
				display_array[4] %= 256;
			display_array[10] ++;
				display_array[4] %= 256;
		}

		mode_flag[7] = 1;




	}else if(perform_array[0] == 8){//しぬ 159~
		shdn_flag = 0;
		mode_flag[8] = 1;

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
		mode_flag[9] = 1;

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
		mode_flag[10] = 1;

//		movement_array[0] = 500;
//		movement_array[1] = 0;
		movement_array[2] = 0;

		if(mode_flag[10] == 0){
			p_beat = first_beat = perform_array[2];
			p_count = first_count = perform_array[1];

			display_array[1] = 24;//x
			display_array[2] = 24;//y
			display_array[3] = 0;//r
			display_array[4] = perform_array[3];//circle_H
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3] + 128;//background_H
				display_array[7] %= 256;
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		if(perform_array[2] >= 2){
			if(perform_array[1] - p_count > 7){
				display_array[3]++;
				p_count = perform_array[1];
			}
		}

////サビ



	}else if(perform_array[0] == 11 || perform_array[0] == 12){//もりあがり 18~47 48~73
		shdn_flag = 1;
		fix_flag = 1;


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
				display_array[7] %= 256;
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

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




//		if(position_array[0] > 1000 && position_array[0] < 2000){
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
//			circle_position_array[1] = 24;
//			circle_position_array[2] = r_standard;//r
//		}else if(position_array[0] >= 2000 && position_array[0] < 3000){
//
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
//			circle_position_array[1] = 24;
//			circle_position_array[2] = r_standard;//r
//
//		}else if(position_array[0] >= 3000 && position_array[0] < 4000){
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
//			circle_position_array[1] = 24;
//			circle_position_array[2] = r_standard;//r
//
//		}else if(position_array[0] <= 1000){
//			katamuki = 1;
//
//			movement_array[0] = 4000;
//			movement_array[1] = 0;
//			movement_array[2] = 300;
//		}else if(position_array[0] >= 4000){
//			katamuki = -1;
//
//			movement_array[0] = 1000;
//			movement_array[1] = 0;
//			movement_array[2] = 300;
//		}


		mode_flag[11] = 1;
		mode_flag[12] = 1;

//	}else if(perform_array[0] == 12){// 74
//		shdn_flag = 1;
//
////		movement_array[0] = 500;
////		movement_array[1] = 0;
////		movement_array[2] = 200;
//
//		display_array[1] = 0;//x
//		display_array[2] = 0;//y
//		display_array[3] = 0;//r
//		display_array[4] = 0;//circle_H
//		display_array[5] = 0;//circle_S
//		display_array[6] = 0;//circle_V
//		display_array[7] = perform_array[3];//background_H
//		display_array[8] = S_standard;//background_S
//		display_array[9] = V_standard;//background_V
//		display_array[10] = perform_array[3];//frame_H
//		display_array[11] = S_standard;//frame_S
//		display_array[12] = V_standard;//frame_V
//
//		mode_flag[12] = 1;



	}else if(perform_array[0] == 13){//盛り上がりの最後 74~80
		shdn_flag = 1;

//		movement_array[0] = 500;
//		movement_array[1] = 0;
//		movement_array[2] = 200;

		display_array[1] = 0;//x
		display_array[2] = 0;//y
		display_array[3] = 0;//r
		display_array[4] = 0;//circle_H
		display_array[5] = 0;//circle_S
		display_array[6] = 0;//circle_V
		display_array[7] = perform_array[3];//background_H
		display_array[8] = S_standard;//background_S
		display_array[9] = V_standard;//background_V
		display_array[10] = perform_array[3];//frame_H
		display_array[11] = S_standard;//frame_S
		display_array[12] = V_standard;//frame_V

		mode_flag[13] = 1;




	}else if(perform_array[0] == 14){//フェードアウト 81~94

		if(mode_flag[14] == 0){
			p_beat = first_beat = perform_array[2];
			p_count = first_count = perform_array[1];

			katamuki = 1;

			shdn_flag = 1;

			movement_array[0] = 2000;
			movement_array[1] = 0;
			movement_array[2] = 200;

			display_array[1] = 24;//x
			display_array[2] = 24;//y
			display_array[3] = 0;//r
			display_array[4] = perform_array[3] + 128;//circle_H
				display_array[7] %= 256;
			display_array[5] = S_standard;//circle_S
			display_array[6] = V_standard;//circle_V
			display_array[7] = perform_array[3];//background_H
			display_array[8] = S_standard;//background_S
			display_array[9] = V_standard;//background_V
			display_array[10] = perform_array[3];//frame_H
			display_array[11] = S_standard;//frame_S
			display_array[12] = V_standard;//frame_V
		}

		if((perform_array[1] - p_count) >= 1){
			if(display_array[3] > 4 && display_array[3] < 20 && katamuki == -1){
				display_array[3]--;
			}else if(display_array[3] > 4 && display_array[3] < 20 && katamuki == 1){
				display_array[3]++;
			}else if(display_array[3] <= 4 && (perform_array[2] - p_beat) >= 2){
				katamuki = 1;
				display_array[3]++;
				p_beat = perform_array[2];
			}else if(display_array[3] >= 20 && (perform_array[2] - p_beat) >= 2){
				katamuki = -1;
				display_array[3]--;
				p_beat = perform_array[2];
			}
			if(display_array[6] != 0){
				display_array[6]--;
			}
			if(display_array[9] != 0){
				display_array[9]--;
			}
			if(display_array[12] != 0){
				display_array[12]--;
			}

			p_count = perform_array[1];
		}

		mode_flag[14] = 1;




	}else if(perform_array[0] == 15){//余韻 95~105
		shdn_flag = 0;
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



