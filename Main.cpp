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

string pathGet();
ofstream Log;

int main(void){   
  DualShock3 controller;
  MotorSerial ms;
  int MAX = 200;    //PWMの最大値
  /*-------GPIOピン割り当て-------*/
  //動作確認LED
  int RunLED = 13;
  //電磁弁
  int Air_1 = 16;   //回転・上下機構
  int Air_2 = 17;   //吸引①
  int Air_3 = 18;   //吸引②
  //リミットスイッチ
  int Lsin_1 = 2;
  //確認LED
  //小型モーター（回転・上下機構）
  int SMotor_L = 26;
  int SMotor_R = 20;
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
  pinMode(Air_2, OUTPUT);
  pinMode(Air_3, OUTPUT);
  pinMode(SMotor_L, OUTPUT);
  pinMode(SMotor_R, OUTPUT);

  pinMode(Lsin_1, INPUT);
  //リミットスイッチのプルダウン
  pullUpDnControl(Lsin_1, PUD_DOWN);

  digitalWrite(RunLED, 1);
  digitalWrite(SMotor_L, 0);
  digitalWrite(SMotor_R, 0);

  //STARTボタンが押されるまで実行
  UPDATELOOP(controller, !(controller.button(START))){ 	
    //全モーターの非常停止。SELECTを押すと作動、もう一度押すと解除
    if(controller.press(SELECT)){
      ms.send(255, 255, 0);
      cout << "全モーター非常停止" << endl;
      UPDATELOOP(controller, !controller.press(SELECT));
      cout << "非常停止解除" << endl;
    }
    double regulation = 1;    //足回りの速さ
    //L1を押した時のアクション
    if(controller.button(L1)){
      regulation = 0.2;         //低速モード（出力が通常の0.2倍）
      if(controller.press(TRIANGLE)){
        cout << "自動制御1" << endl;
        delay(500);
        cout << "自動制御1終了" << endl;
      }
      if(controller.press(SQUARE)){
        cout << "自動制御2" << endl;
        delay(1500);
        cout << "自動制御2終了" << endl;
      }
      //回転腕で箱を掴む、離す
      //CIRCLEを押すと0.5秒回転（掴む）、もう一度押すと0.5秒逆回転（離す）
      static bool SMotorSpin = false;
      static bool OpenFlag = false;
      static bool CloseFlag = false;
      if(controller.press(CIRCLE))
        SMotorSpin = !SMotorSpin;
      if(SMotorSpin){
        if(!OpenFlag){
          digitalWrite(SMotor_L, 1);
          delay(500);
          digitalWrite(SMotor_L, 0);
          cout << "つかんだ" << endl;
          OpenFlag = true;
          CloseFlag = false;
        }
      }else{
        if(!CloseFlag && controller.press(CIRCLE)){
          digitalWrite(SMotor_R, 1);
          delay(500);
          digitalWrite(SMotor_R, 0);
          cout << "はなした" << endl;
          CloseFlag = true;
          OpenFlag = false;
        }
      }
      //回転・上下機構の回転 CROSSを押すと回転、もう一度押すと元に戻る
      static bool airSpin = false;
      if(controller.press(CROSS))
        airSpin = !airSpin;
      if(airSpin && controller.press(CROSS)){
        digitalWrite(Air_1, 1);
        cout << "回転腕回転" << endl;
      }
      if(!airSpin && controller.press(CROSS)){
        digitalWrite(Air_1, 0);
        cout << "回転腕が元に戻る" << endl;
      }
    }
    bool dual = false;    //trueにするとLeftStickのみでロボット全体を操作できる
    //R1を押した時のアクション
    if(controller.button(R1)){
      regulation = 0.1;     //超低速モード（出力が通常のの0.1倍）
      //電磁弁1作動
      if(controller.press(SQUARE)){
        digitalWrite(Air_2, 1);
        delay(60);
        digitalWrite(Air_2, 0);
        cout << "電磁弁1作動" << endl;
      }
      //電磁弁2作動
      if(controller.press(CIRCLE)){
        digitalWrite(Air_3, 1);
        delay(60);
        digitalWrite(Air_3, 0);
        cout << "電磁弁2作動" << endl;
      }
    }
    //橋用タイヤ
    if(controller.button(L1) && controller.button(R1)){
      //ms.send(1, 2, -MAX);
      ms.send(1, 2, ((controller.stick(LEFT_Y) + controller.stick(RIGHT_Y)) / 2));
      cout << ((controller.stick(LEFT_Y) + controller.stick(RIGHT_Y)) / 2) << endl;
    }

    //--------------ここから足回り(メカナムによる移動)--------------
    //左側前後
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
    ms.send(1, 3, lf * left_w * regulation);  //左前
    ms.send(2, 3, lb * left_w * regulation);  //左後
    //右側前後
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
      ms.send(4, 3, -(lb * left_w * regulation)); //右後
      ms.send(5, 3, -(lf * left_w * regulation)); //右前
    }else{
      ms.send(4, 3, rf * right_w * regulation); //右前
      ms.send(5, 3, rb * right_w * regulation); //右後
    }
    //回転腕機構の上下
    if(controller.press(RIGHT)){
      ms.send(2, 2, -MAX);
      cout << "回転腕上昇" << endl;
    }
    if(controller.press(DOWN)){
      ms.send(2, 2, MAX);
      cout << "回転腕下降" << endl;
    }
    if(controller.release(RIGHT) || controller.release(DOWN))
      ms.send(2, 2, 0);
    //ロジャーアームの上下
    if(controller.press(UP)){
      ms.send(4, 2, MAX);
      cout << "ロジャーアーム上昇" << endl;
    }
    if(controller.press(LEFT)){
      ms.send(4, 2, -MAX);
      cout << "ロジャーアーム下降" << endl;
    }
    if(controller.release(UP) || controller.release(LEFT))
      ms.send(4, 2, 0);
    //吸引機構の回転
    if(controller.button(L2)){      //左回転
      ms.send(16, 2, (controller.stick(LEFT_T) / 2));
      cout << (controller.stick(LEFT_T) / 2) << endl;
    }
    if(controller.button(R2)){      //右回転
      ms.send(16, 2, -(controller.stick(RIGHT_T) / 2));
      cout << -(controller.stick(RIGHT_T) / 2) << endl;
    }
    if(controller.release(L2) || controller.release(R2))
      ms.send(16, 2, 0);

    //回転・上下機構用モーター SQUAREを押すと0.5秒回転、もう一度押すと0.5秒逆回転
    //SQUAREを押すとつかむ、もう一度押すとはなす
    /*static bool SMotorSpin = false;
    static bool OpenFlag = false;
    static bool CloseFlag = false;
    if(controller.press(CIRCLE))
      SMotorSpin = !SMotorSpin;
    if(SMotorSpin){
      if(!OpenFlag){
        digitalWrite(SMotor_L, 1);
        delay(500);
        digitalWrite(SMotor_L, 0);
        cout << "つかんだ" << endl;
        OpenFlag = true;
        CloseFlag = false;
      }
    }else{
      if(!CloseFlag && controller.press(CIRCLE)){
        digitalWrite(SMotor_R, 1);
        delay(500);
        digitalWrite(SMotor_R, 0); 
        cout << "はなした" << endl;
        CloseFlag = true;
        OpenFlag = false;
      }
    }*/
    //電磁石 TRIANGLEを押すとON、もう一度押すとOFF
    static bool MagnetFlag = false;
    if(controller.press(TRIANGLE))
      MagnetFlag = !MagnetFlag;
    if(MagnetFlag && controller.press(TRIANGLE) && !(controller.button(L1))){
      ms.send(11, 3, 200);
      cout << "電磁石ON" << endl;
    }
    if(!MagnetFlag && controller.press(TRIANGLE) && !(controller.button(L1))){
      ms.send(11, 3, 0);
      cout << "電磁石OFF" << endl;
    }
    //真空ポンプ  CROSS押すとON、もう一度押すとOFF
    static bool PompSpin = false;
    if(controller.press(CROSS))
      PompSpin = !PompSpin;
    if(PompSpin && controller.press(CROSS) && !(controller.button(L1)) ){
      ms.send(11, 2, 200);
      cout << "真空ポンプ作動中" << endl;
    }
    if(!PompSpin && controller.press(CROSS) && !(controller.button(L1))){
      ms.send(11, 2, 0);
      cout << "真空ポンプ作動停止" << endl; 
    }
  }
  Log << "Successful completion" << endl; 
  cout << "プログラム終了" << endl;
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
