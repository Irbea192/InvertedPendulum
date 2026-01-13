#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_TB6612.h>
#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

// PID制御用パラメータ
float Kp = 10.0;
float Ki = 1.0;
float Kd = 0.8;

float targetAngle = 0.0; // 目標角度（直立）
float integral = 0.0;
float lastError = 0.0;

unsigned long lastTime = 0;

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

const float vref = 3.49;

int16_t speed = 0;

Motor motorA(AIN1, AIN2, PWMA, offsetA, STBY); // AIN1, AIN2, PWMA
Motor motorB(BIN1, BIN2, PWMB, offsetB, STBY); // BIN1, BIN2, PWMB

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
    if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
      Serial.println("init false");
      while(1);
    }

    Serial.println("Setup done!");
}

void loop() {
  delay(10);
}
