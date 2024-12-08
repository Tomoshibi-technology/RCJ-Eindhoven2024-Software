
#include "performance.h"

PERFORMANCE::PERFORMANCE(uint16_t* ptr_perform_array, int16_t* ptr_display_array, int16_t* ptr_circle_relative_position_array, int16_t* ptr_fish_relative_position_array, int16_t* ptr_now_position_array){
	perform_array = ptr_perform_array;//TweLiteからのデータ
	display_array = ptr_display_array;//displayに送るデータ
	circle_relative_position_array = ptr_circle_relative_position_array;//円の相対座標
	fish_relative_position_array = ptr_fish_relative_position_array;//おさかなの相対座標
	now_position_array = ptr_now_position_array;//ロボットの絶対座標 {x, y, speed}

	circle_position_array[0] = now_position_array[0];
	circle_position_array[1] = 24 * 10;
	circle_position_array[2] = r_standard * 10;//r

}

void PERFORMANCE::get_drawing_status_performance(){

}


void PERFORMANCE::cal_drawing_status_perfomance(uint32_t count){

//	perform_array[0] = hue
//	perform_array[1] = θ
//	perform_array[2] = r
//	perform_array[3] = mode

	//送られてきた速度ベクトルを処理して、目標速度ベクトルを決める
	//モードを処理して描画モードを選択する
	//停止、まる固定、おさかな固定,まる運び


//ラインポジション
	line_position_array[0] = now_position_array[0] - 220;
	line_position_array[1] = now_position_array[0] + 220;

//サークルポジション
	//circle_position_array[0]は基本前の値と同じ

	circle_position_array[1] = 24 * 10;

	circle_position_array[2] = r_standard * 10;//r
	circle_position_array[2] = circle_position_array[2] - now_position_array[1] * circle_position_array[2] / shrink_const;//前後
//	if(perform_array[3] == 0){
//		circle_position_array[2] = circle_position_array[2] + circle_position_array[2] * sin() / 8;
//	}else{}

	if(perform_array[3] == 3){
		if(circle_position_array[0] - circle_position_array[2] <= line_position_array[0]){
			circle_position_array[0] = line_position_array[0] + circle_position_array[2];
//		}else if((circle_position_array[0] - circle_position_array[2] > line_position_array[0]) || (circle_position_array[0] + circle_position_array[2] > line_position_array[1])){
//
		}else if(circle_position_array[0] + circle_position_array[2] >= line_position_array[1]){
			circle_position_array[0] = line_position_array[1] - circle_position_array[2];
		}else{}
	}else{}

//まるとかの光
	if(perform_array[3] == 0){
		display_array[4] = perform_array[0] + 128;//circle_H
		display_array[5] = S_standard;//circle_S
		display_array[6] = V_standard;//circle_V
		display_array[7] = perform_array[0];//background_H
		display_array[8] = S_standard;//background_S
		display_array[9] = V_standard;//background_V
		display_array[10] = perform_array[0];//frame_H
		display_array[11] = S_standard;//frame_S
		display_array[12] = V_standard;//frame_V

	}else if(perform_array[3] == 1){
		display_array[4] = perform_array[0] + 128;//circle_H
			display_array[4] %= 256;
		display_array[5] = S_standard;//circle_S
		display_array[6] = V_standard;//circle_V
		display_array[7] = perform_array[0];//background_H
		display_array[8] = S_standard;//background_S
		display_array[9] = V_standard;//background_V
		display_array[10] = perform_array[0];//frame_H
		display_array[11] = S_standard;//frame_S
		display_array[12] = V_standard;//frame_V

	}else if(perform_array[3] == 2){
		display_array[4] = perform_array[0];//circle_H
		display_array[5] = S_standard;//circle_S
		display_array[6] = V_standard;//circle_V
		display_array[7] = perform_array[0];//background_H
		display_array[8] = S_standard;//background_S
		display_array[9] = V_standard;//background_V
		display_array[10] = perform_array[0];//frame_H
		display_array[11] = S_standard;//frame_S
		display_array[12] = V_standard;//frame_V

	}else if(perform_array[3] == 3){
		display_array[4] = perform_array[0] + 128;//circle_H
			display_array[4] %= 256;
		display_array[5] = S_standard;//circle_S
		display_array[6] = V_standard;//circle_V
		display_array[7] = perform_array[0];//background_H
		display_array[8] = S_standard;//background_S
		display_array[9] = V_standard;//background_V
		display_array[10] = perform_array[0] + 128;//枠表示
			display_array[10] %= 256;
		display_array[11] = S_standard;//frame_S
		display_array[12] = V_standard;//frame_V

	}else{
		display_array[4] = perform_array[0] + 128;//circle_H
		display_array[5] = S_standard;//circle_S
		display_array[6] = V_standard;//circle_V
		display_array[7] = perform_array[0];//background_H
		display_array[8] = S_standard;//background_S
		display_array[9] = V_standard;//background_V
		display_array[10] = perform_array[0];//frame_H
		display_array[11] = S_standard;//frame_S
		display_array[12] = V_standard;//frame_V
	}


//おさかなの設定
	if(perform_array[3] == 2){
		fish_position_array[0] = 2;//ID
		fish_position_array[1] = circle_position_array[0];//X
		fish_position_array[2] = 150;//Y

		fish_position_array[3] = 2;//ID
		fish_position_array[4] = circle_position_array[0] + 400;//X
		fish_position_array[5] = 300;//Y

		fish_position_array[6] = 2;//ID
		fish_position_array[6] = circle_position_array[0] - 200;//X
		fish_position_array[8] = 50;//Y

		fish_position_array[9] = 2;//ID
		fish_position_array[10] = circle_position_array[0] + 700;//X
		fish_position_array[11] = 75;//Y

		fish_position_array[12] = 2;//ID
		fish_position_array[13] = circle_position_array[0] - 400;//X
		fish_position_array[14] = 250;//Y

		fish_position_array[15] = 2;//ID
		fish_position_array[16] = circle_position_array[0] - 700;//X
		fish_position_array[17] = 200;//Y

	}else{
		fish_position_array[0] = 0;//ID
		fish_position_array[1] = 1500;//X
		fish_position_array[2] = 150;//Y

		fish_position_array[3] = 0;//ID
		fish_position_array[4] = 1900;//X
		fish_position_array[5] = 300;//Y

		fish_position_array[6] = 0;//ID
		fish_position_array[7] = 2300;//X
		fish_position_array[8] = 150;//Y

		fish_position_array[9] = 0;//ID
		fish_position_array[10] = 2700;//X
		fish_position_array[11] = 300;//Y

		fish_position_array[12] = 0;//ID
		fish_position_array[13] = 3100;//X
		fish_position_array[14] = 150;//Y

		fish_position_array[15] = 0;//ID
		fish_position_array[16] = 3500;//X
		fish_position_array[17] = 300;//Y
	}


}
