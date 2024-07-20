import controlP5.*;
import processing.serial.*;

import ddf.minim.*;
import ddf.minim.analysis.*;
import ddf.minim.effects.*;
import ddf.minim.signals.*;
import ddf.minim.spi.*;
import ddf.minim.ugens.*;

AudioPlayer performanceA;
AudioPlayer performanceB;
AudioPlayer ready;
AudioPlayer pon;
Minim minim;
FFT fft;


String bgm_ready = "Lightrail.mp3";
String bgm_performanceA = "parformanceA.wav";
String bgm_performanceB = "parformanceB.wav";
String bgm_pon = "button.mp3";


ControlP5 cp5; 
int slider1;
boolean toggle1 = false;
Textlabel textlabel;

Serial myPort;


void setup() {
	printArray(Serial.list());

	// size(1920,1080);
	fullScreen();
  frameRate(66);

	int available_serialport = 0; // シリアル検索プログラムで調べたシリアルポートの番号に変更s
	String arduinoPort = Serial.list()[available_serialport];
	myPort = new Serial(this, arduinoPort, 115200); // シリアルポートの設定
	
	cp5 = new ControlP5(this);

	cp5.addSlider("slider1")
		.setPosition(80,30)
		.setSize(1350,50)
		.setRange(1,254)
		.setValue(127)
    .setColorForeground(color(20,10,100))   // バー色
    .setColorActive(color(10,20,230))   // マウス選択色
  ;

	cp5.addToggle("toggle1")
		.setPosition(80,100)
		.setSize(500,50)
		.setValue(false)
	;

  textlabel = cp5.addTextlabel("info")
    .setPosition(20, 850)
    .setSize(200, 30)
    .setFont(createFont("Arial", 20))
    .setColor(color(0, 0, 0))
    .setText("Slider Value: 50")
  ;  // 初期テキスト

	minim = new Minim(this);
  performanceA = minim.loadFile(bgm_performanceA,1024);
  performanceB = minim.loadFile(bgm_performanceB,1024);
	ready = minim.loadFile(bgm_ready,1024);
	pon = minim.loadFile(bgm_pon,1024);

	//fft = new FFT(performance.bufferSize(), performance.sampleRate());
}

boolean ready_musicflg = false;

boolean startflg = false;
int start_millis;

long A_start_time = 0;
long A_stop_time = 75000;
long B_start_time = 90000;
long B_stop_time = 140000;

boolean startflg_A = false;
int beat_millis_A = 469 ; // 1000/2.13333 128bpm
long pre_beat_millis_A = 0;
int beat_count_A = 0;
int small_millis_A = 23 ; // 469/23=20.391カウント 
long pre_small_millis_A = 0;
int small_count_A = 0;

boolean startflg_B = false;
int beat_millis_B = 469 ; // 1000/2.13333 128bpm
long pre_beat_millis_B = 0;
int beat_count_B = 0;
int small_millis_B = 23 ; // 469/23=20.391カウント 
long pre_small_millis_B = 0;
int small_count_B = 0;

void draw() {
	//　時間経過
	if(toggle1 && !(startflg)){
		start_millis = millis();
		startflg = true;
  }
  if(!(startflg)){
    ready.play();
  }
  
  long now_millis = millis() - start_millis;
  if(startflg){
    ready.close();
    if(now_millis>A_start_time && !(startflg_A)){ // Aをスタートさせる
      performanceA.play();
      startflg_A = true;
      pre_beat_millis_A = now_millis + 205;
      pre_small_millis_A = now_millis + 205;
    }
    if(now_millis>A_stop_time && startflg_A){ // Aをとめる
      performanceA.close();
      startflg_A = false;
      beat_count_A = 0;
      small_count_A = 0;
    }
  	if(startflg_A){ // Aのカウント
  		if(pre_beat_millis_A + beat_millis_A < now_millis){ // beatカウント
        //pon.play();
        //pon.rewind();
  			pre_beat_millis_A += beat_millis_A;
  			beat_count_A++;
        small_count_A = beat_count_A * 21;
  		}
      if(pre_small_millis_A + small_millis_A < now_millis){ // 小さい方カウント
        pre_small_millis_A = now_millis; // 徐々にズレが蓄積しても問題ないため
        small_count_A++;
      }
    }
    //-----------ここまでA---------------
    if(now_millis>B_start_time && !(startflg_B)){ // Bをスタートさせる
      performanceB.play();
      startflg_B = true;
      pre_beat_millis_B = now_millis + 205;
      pre_small_millis_B = now_millis + 205;
    }
    if(now_millis>B_stop_time && startflg_B){ // Aをとめる
      performanceB.close();
      startflg_B = false;
      beat_count_B = 0;  
      small_count_B = 0;
    }
    if(startflg_B){ // Aのカウント
      if(pre_beat_millis_B + beat_millis_B < now_millis){ // beatカウント
        //pon.play();
        //pon.rewind();
        pre_beat_millis_B += beat_millis_B;
        beat_count_B++;
        small_count_B = beat_count_B * 21;
      }
      if(pre_small_millis_B + small_millis_B < now_millis){ // 小さい方カウント
        pre_small_millis_B = now_millis; // 徐々にズレが蓄積しても問題ないため
        small_count_B++;
      }
    }
  }  
  
  
	//Mode選ぶ
	int mode = 0;
	int myHue = 0;
  int raw_count = 0;
  
  if(!(startflg)){
     mode = 0;
    myHue = slider1;
    raw_count = 0;
  }else if(A_start_time<=now_millis && now_millis<=A_stop_time){
    if(beat_count_A <= 15){
      mode = 1;
      myHue = (beat_count_A*5 + 20)%255;
    }else if(beat_count_A <= 47){
      mode = 2;
      myHue = (100+beat_count_A*2)%255;
    }else if(beat_count_A <= 78){
      mode = 3;
      myHue = (150+beat_count_A*2)%255;
    }else if(beat_count_A <= 110){
      mode = 4;
      myHue = (50+beat_count_A*2)%255;
    }else if(beat_count_A <= 127){
      mode = 5;
      myHue = (90+beat_count_A*2)%255;
    }else if(beat_count_A <= 142){
      mode = 6;
      myHue = (130+beat_count_A*5)%255;
    }else if(beat_count_A <= 158){ // バグ開始 
      mode = 7;
      myHue = (small_count_A*30 + 20)%255;
    }else{
      mode = 8;
      myHue = 0;
    }
    raw_count = small_count_A;
  }else if(A_stop_time<=now_millis && now_millis<=B_start_time){
    mode = 9;
    myHue = 0;
    raw_count = 0;
  }else if(B_start_time<=now_millis && now_millis<=B_stop_time){
    if(beat_count_B <= 17){
      mode = 10;
      myHue = (150+beat_count_B)%255;
    }else if(beat_count_B <= 47){
      mode = 11;
      myHue = (180+beat_count_B)%255;
    }else if(beat_count_B <= 73){
      mode = 12;
      myHue = (beat_count_B*10 + 20)%255;
    }else  if(beat_count_B <= 80){
      mode = 13;
      myHue = (beat_count_B*10 + 20)%255;
    }else  if(beat_count_B <= 94){
      mode = 14;
      myHue = (beat_count_B*10 + 20)%255;
    }else  if(beat_count_B <= 105){
      mode = 15;
      myHue = 120;
    }else{
      mode = 16;
      myHue = 0;
    }
    raw_count = small_count_B;
  }else{
    mode = 16;
    myHue = 0;
    raw_count = 0;
  }
  
  if(now_millis%30 == 0){
     print(small_count_A);
     print("___");
     print(beat_count_A);
     print("___");
     print(small_count_B);
     print("___");
     print(beat_count_B);
     print("___");
     print(now_millis);
     print("_______");
     print(mode);
     print("___");
     print(myHue);
     print("___");
     println(raw_count);
  }
    
  
	// 色の調整
	if(myHue == 250){
		myHue = 251;
	}

	 //通信
	if(raw_count%2 == 0){ //20FPS
		myPort.write(250);
		myPort.write(mode+5); //1         
		myPort.write(byte(raw_count/240 + 5)); //2
		myPort.write(byte(raw_count%240 + 5)); //3
		myPort.write(myHue); //4
	}



	//if(myPort.available() > 0){
	//	// print(myPort.read());
	//	// print("______");
	//}
	//if(raw_count_A%20 == 0){
	//	 print(mode);
	//	 print("___");
	//	 print(raw_count_A);
	//	 print("___");
	//	 print(myHue);
	//	 println("___");
	//}

	//お絵描き

	colorMode( HSB ); 
  if(mode==9){
  	background(myHue,0,0); // 背景色をスライダーの値に変更
  }else{
    background(myHue,200,250); // 背景色をスライダーの値に変更
  }


  textlabel.setText(
    "mode: "+mode+ 
    "\ncount: "+raw_count+ 
    "\nHue: "+myHue+
    "\nbeatA: "+beat_count_A+ 
    "\nbeatB: "+beat_count_B 
  );

	if(mode != 0){
		// rect(0,0,1920,1080);
		// colorMode(RGB, 255);
		// background(0);
		// fft.forward(in.mix);
		
		// noStroke();
		// colorMode(HSB, 360, 100, 100, 255);
		// for(int i = 0; i < fft.specSize(); i++){
		// 	float hue = map(i, 0, fft.specSize(), 0, 360);
		// 	fill(hue, 100, 100, 10);
		// 	float radious = fft.getBand(i) * 5;
		// 	ellipse(width/2, height/2, radious * 2, radious * 2);
		// }
	}
}
