#include <iostream>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "RasPiDS3/RasPiDS3.hpp"
#include "RasPiMS/RasPiMS.hpp"
#include <cstdio>

using namespace std;
using namespace RPDS3;
using namespace RPMS;

char* SettingFileName[] = {(char*)"MotorSetting"};

string pathGet();
ofstream Log;

int main(void){   
  DualShock3 controller;
  MotorSerial ms;

  int MAX = 200;    //PWMの最大値

  /*-------GPIOピン割り当て-------*/
  //動作確認LED
  int RunLED = 13;
  //エアシリンダ
  int Air_1 = 20;
  //リミットスイッチ
  int Lsin_1 = 2;
  /*-------割り当てここまで-------*/

  cout << "Main start." << endl;
  //プログラム起動時刻の取得
  time_t progStartTime = time(NULL);
  struct tm *pnow = localtime(&progStartTime);
  //ログファイルを開く
  string path(pathGet().c_str());
  path.erase(path.find_last_of('/'));
  Log.open(path + "/Log.txt");
  Log << pnow->tm_year + 1900<< ":" << pnow->tm_mon + 1 << ":" << pnow->tm_mday << ":" <<  pnow->tm_hour << ":" << pnow->tm_min << ":" << pnow->tm_sec << endl;
  Log << "Path :" + path << endl; 
  //MDD通信セットアップ
  try{
		ms.init();
	}
	catch(const char *str){
    return -1;
	}
  Log << "Success setup MotorSerial." << endl;
  //コントローラー接続確認
  if(!controller.connectedCheck()){
    cout << "Controller unconnected." << endl;
    return 0;
  }
  Log << "Success connect to controller." << endl;

  cout << "Standby MotorDriverDriver Communication." << endl;
  Log << "Standby MotorDriverDriver Communication." << endl;

  pinMode(RunLED, OUTPUT);
  pinMode(Air_1, OUTPUT);
  pinMode(Lsin_1, INPUT);

  pullUpDnControl(Lsin_1, PUD_DOWN);

  digitalWrite(RunLED, 1);

  //STARTボタンとCROSSボタンがが押されるまで実行	
	UPDATELOOP(controller, !(controller.button(START) && controller.button(CROSS))){ 	
    //全モーターの非常停止。SELECTを押すと作動、もう一度押すと解除
    if(controller.press(SELECT)){
      ms.send(255, 255, 0);
      cout << "All Motor Stop." << endl;
      UPDATELOOP(controller, !controller.press(SELECT));
    }
    //ここから足回り(メカナムによる移動)
    //LeftStickのみでロボットを操作できる。R2を押している間のみ有効
    bool dual = false;  //int
    if(controller.button(R1))
      dual = true;

    //低速モード。L2を押している間のみ有効
    double regulation = 1;
    if(controller.button(L1) == true)
      regulation = 0.25;

    //右側前後
    double left_x = 0;
    double left_y = 0;
    double left_theta = 0;
    int left_w = 0;
    double lb, lf;

    left_y = controller.stick(LEFT_Y);
    left_x = controller.stick(LEFT_X);
    left_w = sqrt(pow(left_x, 2) + pow(left_y, 2)) * 2;
    if(left_w > MAX)
      left_w = MAX;
    left_theta = (atan2(-left_y, left_x) + M_PI);
    if(left_theta >= 0 && left_theta <= (M_PI/2)){
      lb = (left_theta * 4 / M_PI) - 1;
      lf = 1;
    }else if(left_theta > (M_PI/2) && left_theta <= M_PI){
      lb = 1;
      lf = -(left_theta * 4 / M_PI) + 3;
    }else if(left_theta > (M_PI) && left_theta <= (M_PI*3/2)){
      lb = -(left_theta * 4 / M_PI) + 5;
      lf = -1;
    }else if(left_theta > (M_PI*3/2) && left_theta <= (M_PI*2)){
      lb = -1;
      lf = (left_theta * 4 / M_PI) - 7;
    }
    ms.send(1, 2, -(lf * left_w * regulation));
    ms.send(1, 3, lb * left_w * regulation);
  
    //左側前後
    double right_x = 0;
    double right_y = 0;
    double right_theta = 0;
    int right_w = 0;
    double rf, rb;

    right_y = controller.stick(RIGHT_Y);
    right_x = controller.stick(RIGHT_X);
    right_w = sqrt(pow(right_x, 2) + pow(right_y, 2)) * 2;
    if(right_w > MAX)
      right_w = MAX;
    right_theta = (atan2(-right_y, right_x) + M_PI);

    if(right_theta >= 0 && right_theta <= (M_PI/2)){
      rf = -(right_theta * 4 / M_PI) + 1;
      rb = -1;
    }else if(right_theta > (M_PI/2) && right_theta <= M_PI){
      rf = -1;
      rb = (right_theta * 4 / M_PI) - 3;
    }else if(right_theta > (M_PI) && right_theta <= (M_PI*3/2)){
      rf = (right_theta * 4 / M_PI) - 5;
      rb = 1;
    }else if(right_theta > (M_PI*3/2) && right_theta <= (M_PI*2)){
      rf = 1;
      rb = -(right_theta * 4 / M_PI) + 7;
    }
    if(dual){
      ms.send(2, 2, lb * left_w * regulation);
      ms.send(2, 3, -(lf * left_w * regulation));
    }else{
      ms.send(2, 2, -(rf * right_w * regulation));
      ms.send(2, 3, rb * right_w * regulation);
      //cout << -(rf * left_w * regulation) << " : " << rb * left_w * regulation << endl;
    }
    //上下機構
    if(controller.press(UP)){
      ms.send(3, 2, 200);
      cout << "UP" << endl;
    }
    if(controller.press(DOWN)){
      ms.send(3, 2, -200);
    }

    if(controller.release(UP) || controller.release(DOWN)){
      ms.send(3, 2, 0);
    }
    //回転機構
    static bool airSpin = false;
    if(controller.press(TRIANGLE))
      airSpin = !airSpin;
    if(airSpin == true){
      digitalWrite(Air_1, 1);
    }
    else{
      digitalWrite(Air_1, 0);
    }
    //ロジャーアーム上下
    static bool RogerArm = false;
    if(controller.press(TRIANGLE))
      RogerArm = !RogerArm;
    if(RogerArm == true){
      ms.send(4, 2, 200);
    }
    else{
      ms.send(4, 2, 0);
    }
    //ロジャーアーム回転
    /*if(controller.press(SQUARE)){
      ms.send(3, 2, 200);
      ms.send(3, 3, 200);
      digitalWrite(Air_1, 1);
    }
    if(controller.release(SQUARE)){
      ms.send(3, 2, 0);
      ms.send(3, 3, 0);
      digitalWrite(Air_1, 0);
    }
    if(controller.press(TRIANGLE)){
      ms.send(4, 2, 200);
      ms.send(4, 3, 200);
    }
    if(controller.release(TRIANGLE)){
      ms.send(4, 2, 0);
      ms.send(4, 3, 0);
     }*/
    //CIRCLEを押すとモーター回転、もう一度押すと停止するサンプル
    static bool motorSpin = false;
    if(controller.press(CIRCLE))
      motorSpin = !motorSpin;
    if(motorSpin == true){    
      ms.send(5, 2, 200);
      ms.send(5, 3, 200);
    }else{
      ms.send(5, 2, 0);
      ms.send(5, 3, 0);
    }
  }
  Log << "Successful completion" << endl; 
  cout << "Main End." << endl;
  digitalWrite(Air_1, 0);
  digitalWrite(RunLED, 0);
  return 0;
}

string pathGet(){
  //現在のパスを取得
  char buf[512] = {};
  readlink("/proc/self/exe", buf, sizeof(buf) - 1);   // 実行ファイルのパスを取得
  string path(buf);
  return path;
}
