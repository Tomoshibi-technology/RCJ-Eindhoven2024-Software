import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;

ControlIO control;
ControlDevice device;
String[] names = {"A", "B", "X", "Y", "SL", "SR", "PLUS", "STICK", "HOME", "R", "ZR"};

void setup() {
  size(500, 360);
  colorMode(HSB, 360, 100, 100);
  rectMode(CENTER);
  textAlign(LEFT, CENTER);

  control = ControlIO.getInstance(this);
  // 設定ファイルからデバイスを取得
  device = control.getMatchedDevice("joy-con-r");
  if (device == null) {
    println("No suitable device configured");
    System.exit(-1);
  }
}

void draw() {
  background(200, 60, 30);

  // ボタンの状態を表示
  int i = 0;
  for (String name : names) {
    stroke(#eeeeee);
    if (device.getButton(name).pressed()) fill(30, 60, 90); else noFill();
    rect(30, 30 + i * 30, 15, 15);
    fill(#eeeeee);
    text(name, 50, 30 + i * 30);
    i++;
  }
  
  // スティックの状態を表示
  float radius = 75;
  stroke(#eeeeee);
  noFill();
  translate(width/2, height/2);
  beginShape();
  for (i = 0; i < 8; i++) {
    float angle = (float)i / 8 * TWO_PI;
    vertex(radius * cos(angle), radius * sin(angle));
  }  
  endShape(CLOSE);

  noStroke();
  fill(30, 60, 90);
  int pos = device.getHat("POV").getPos();
  float x = 0, y = 0;
  if (pos > 0) {
    // 「横持ち」基準で上下左右が決められているため
    // 右の Joy-Con の場合は HALF_PI を足す
    // 左の Joy-Con の場合は HALF_PI を引く
    float angle = (float)pos / 8 * TWO_PI + HALF_PI;
    rotate(angle);
    x = radius;
  }
  ellipse(x, y, 20, 20);
}
