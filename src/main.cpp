/*
参考
https://ichiken-engineering.com/inverted_pendulum1/
https://qiita.com/MuAuan/items/8dacc75b2e94fc644798
https://qiita.com/mshr299/items/8015044269ba3d5fffee
https://qiita.com/coppercele/items/e4d71537a386966338d0
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

#include <SparkFun_TB6612.h>
#include <MadgwickAHRS.h>
#include <BMI160Gen.h>


#define SERIAL

#define BMI160_ADDRESS 0x69
#define PWM_MIN 90 // 最小PWM値
#define PWM_MAX 255 // 最大PWM値

String mymacAddress = "B8:F8:62:F9:C4:D4";

Madgwick filter;

float roll = 0;
float prevRoll = 0;

// PID制御用パラメータ
float Kp = 10.0;
float Ki = 0.0;
float Kd = 0.0;

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

typedef struct {
  int speed;
} cData;

cData setData;

const float vref = 3.49;

uint64_t lasttime;
int count = 0;

void motorControl(){
    if(speed == 0) {
        brake(motorA, motorB);
    }else{
        forward(motorA, motorB, speed);
    }
    Serial.printf("Speed: %d\n", speed);
}

// 受信コールバック関数
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if (data_len != sizeof(cData)) {
    Serial.println("Received data size mismatch");
    return;
  }
  memcpy(&setData, data, data_len);
  speed = constrain(setData.speed, -255, 255);
  motorControl();
}

void setup() {
    Serial.begin(115200);
    delay(4000);

    Serial.printf("Setup start !\n");

    BMI160.setupI2C(MY_SDA, MY_SCL);
    BMI160.begin(BMI160GenClass::I2C_MODE, BMI160_ADDRESS);
    BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);// ジャイロのレンジ設定
    BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);// 加速度のレンジ設定 

    filter.begin(100);

    WiFi.mode(WIFI_STA);

    if(esp_now_init() != ESP_OK){
      Serial.println("Failed to initialize ESP-NOW");
      return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    lasttime = millis();

    Serial.println("Setup done!");
}

void loop() {
  
  // uint64_t now = millis();
  // if (now - lasttime < 10) return; // 10ms周期
  // lasttime = now;

  // // 6軸センサ読み出し
  // int ax, ay, az, gx, gy, gz;
  // BMI160.readMotionSensor(ax, ay, az, gx, gy, gz);

  // // 加速度値を分解能で割って加速度[G]に変換する
  // float acc_x = ax / 16384.0; // LSB = 2G / 2^15 = 1/16384 G
  // float acc_y = ay / 16384.0;
  // float acc_z = az / 16384.0;

  // // 角速度値を分解能で割って角速度[deg/sec]に変換する
  // float gyro_x = gx / 16.384; // LSB = 2000deg/sec / 2^15 = 1/16.384 deg/sec
  // float gyro_y = gy / 16.384;
  // float gyro_z = gz / 16.384;

  // // Madgwickフィルタの計算
  // filter.updateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);

  // roll = filter.getRoll(); // ロール角を取得

  // #ifdef SERIAL
  //   count++;
  //   if(count >= 20){
  //     Serial.printf("Roll: %f\n", roll);
  //     count = 0;
  //   }
  // #endif

  // dt = (micros() - preTime) * 0.000001;  // 処理時間を求める
  // preTime = micros(); // 現在時間を保存

  // // PID制御
  // P = roll - targetAngle; // 現在の角度と目標角度から偏差を求める
  // I += P * dt; // 偏差の積分
  // D = (P - preP) / dt; // 偏差の微分

  // // アンチワインドアップ
  // // I制御に値が溜まって、積分の飽和が発生し応答が悪くなるので、大きくなりすぎたらリセットする
  // if (100 < abs(I * Ki)) I = 0;

  // power = Kp * P + Ki * I + Kd * D; // 制御量を計算
  // pwm = (int)(constrain(abs(power), PWM_MIN, PWM_MAX)); // PWM値に変換

  // if (roll > stoptheta)
}
