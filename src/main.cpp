/*
参考
https://ichiken-engineering.com/inverted_pendulum1/
https://qiita.com/MuAuan/items/8dacc75b2e94fc644798
https://qiita.com/mshr299/items/8015044269ba3d5fffee
https://qiita.com/coppercele/items/e4d71537a386966338d0
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_TB6612.h>
#include <DFRobot_BMI160.h>
#include <MadgwickAHRS.h>

#define BMI160_ADDRESS 0x69
#define PWM_MIN 20 // 最小PWM値
#define PWM_MAX 255 // 最大PWM値

DFRobot_BMI160 bmi160;
Madgwick filter;

float roll = 0;
float prevRoll = 0;

// PID制御用パラメータ
float Kp = 10.0;
float Ki = 1.0;
float Kd = 0.8;

float targetAngle = 0.0; // 目標角度（直立）
float dt, preTime;
float P, I, D, U, preP;
float power = 0.0;
int pwm;

int stoptheta = 30; //倒立許容角度

const int offsetA = -1;
const int offsetB = 1;

const uint8_t BattVoltPin = A0;
const uint8_t PWMA = D1;
const uint8_t AIN1 = D2;
const uint8_t AIN2 = D3;
const uint8_t BIN1 = D4;
const uint8_t BIN2 = D5;
const uint8_t PWMB = D6;
const uint8_t MY_SCL = D8;
const uint8_t MY_SDA = D9;
const uint8_t STBY = D10;

int16_t speed = 0;

Motor motorA(AIN1, AIN2, PWMA, offsetA, STBY); // AIN1, AIN2, PWMA
Motor motorB(BIN1, BIN2, PWMB, offsetB, STBY); // BIN1, BIN2, PWMB

const float vref = 3.49;

void motorControl(){
    if(speed == 0) {
        brake(motorA, motorB);
    }else{
        forward(motorA, motorB, speed);
    }
    Serial.printf("Speed: %d\n", speed);
}

void setup() {
    Serial.begin(115200);
    delay(4000);

    Serial.printf("Setup start !\n");

    //init the hardware bmin160  
    if (bmi160.softReset() != BMI160_OK){
      Serial.println("reset false");
      while(1);
    }
    
    //set and init the bmi160 i2c address
    if (bmi160.I2cInit(BMI160_ADDRESS) != BMI160_OK){
      Serial.println("init false");
      while(1);
    }

    filter.begin(100);

    Serial.println("Setup done!");
}

void loop() {
  // センサーデータ取得
  int i, rslt;
  int16_t rawData[6]={0};
  float data[6]={0};

  rslt = bmi160.getAccelGyroData(rawData);
  if(rslt == 0){
    for(i=0;i<6;i++){
      if (i<3){
        // 最初の三つのデータはジャイロデータ
        data[i] = rawData[i]*3.14/180.0; // deg
        // Serial.print(rawData[i]*3.14/180.0);
      }else{
        // 後ろの三つのデータは加速度データ
        data[i] = rawData[i]/16384.0; // g
        // Serial.print(rawData[i]/16384.0);Serial.print("\t");
      }
    }
    filter.updateIMU(data[0], data[1], data[2], data[3], data[4], data[5]); // ジャイロと加速度から角度を計算
  }else{
    Serial.println("err");
  }
  roll = filter.getRoll(); // ロール角を取得

  dt = (micros() - preTime) * 0.000001;  // 処理時間を求める
  preTime = micros(); // 現在時間を保存

  // PID制御
  P = roll - targetAngle; // 現在の角度と目標角度から偏差を求める
  I += P * dt; // 偏差の積分
  D = (P - preP) / dt; // 偏差の微分

  // アンチワインドアップ
  // I制御に値が溜まって、積分の飽和が発生し応答が悪くなるので、大きくなりすぎたらリセットする
  if (100 < abs(I * Ki)) I = 0;

  power = Kp * P + Ki * I + Kd * D; // 制御量を計算
  pwm = (int)(constrain(abs(power), PWM_MIN, PWM_MAX)); // PWM値に変換

  if (roll > stoptheta)
}
