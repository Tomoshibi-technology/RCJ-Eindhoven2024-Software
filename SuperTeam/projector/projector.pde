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

Serial motherPort;
Serial grovePort;


void setup() {
	printArray(Serial.list());

	// size(1920,1080);
	fullScreen();
  frameRate(60);

	int available_motherPort = 0; // シリアル検索プログラムで調べたシリアルポートの番号に変更s
	String PortA = Serial.list()[available_motherPort];
	motherPort = new Serial(this, PortA, 115200); // シリアルポートの設定

  int available_grovePort = 1; // シリアル検索プログラムで調べたシリアルポートの番号に変更s
  String PortB = Serial.list()[available_grovePort];
  grovePort = new Serial(this, PortB, 115200); // シリアルポートの設定


	
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

}

int gesture = 250;
boolean flg = false;
int countNum = 0;

void draw(){
  if(flg == true){
    countNum++;
    //gesture == 0;
   if(countNum == 180){
     flg = false;
     countNum = 0;
     gesture = 250;
   }  
  }else{ 
    if(grovePort.available()>0){
       gesture = grovePort.read() - 48;
       print(gesture);
       print("______");
       
       flg = true;
    }
  }
  if(gesture < 0){
    gesture = 250;
  }
  
  println(gesture);
    
  
  
    int mode=0;
    int raw_count = 0;
    int myHue = slider1;
    if(myHue == 250){
      myHue = 251;
    }
    
    	 //通信
	//if(raw_count%2 == 0){ //20FPS
		motherPort.write(250);
		motherPort.write(mode+5); //1         
		//motherPort.write(byte(raw_count/240 + 5)); //2
    motherPort.write(gesture + 5);
		motherPort.write(byte(raw_count%240 + 5)); //3
		motherPort.write(myHue); //4
	//}

  
  textlabel.setText(
    "gesture: "+gesture
    //"\ncount: "+raw_count+ 
    //"\nHue: "+myHue+
    //"\nbeatA: "+beat_count_A+ 
    //"\nbeatB: "+beat_count_B 
  );

}
