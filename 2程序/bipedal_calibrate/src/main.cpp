#include <Arduino.h>
#include "SF_Servo.h"
#include "Wire.h"

SF_Servo servos = SF_Servo(Wire);

int16_t servo1Offset,servo2Offset,servo3Offset,servo4Offset;
int16_t servo5Offset,servo6Offset,servo7Offset,servo8Offset;

void setServosAngle(int16_t val1,int16_t val2,int16_t val3,int16_t val4,int16_t val5,int16_t val6,int16_t val7,int16_t val8){
  servos.setAngle(1, 180+val1);
  servos.setAngle(2, 180+val2);
  servos.setAngle(3, 180+val3);
  servos.setAngle(4, 180+val4);
  servos.setAngle(5, 180+val5);
  servos.setAngle(6, 180+val6);
  servos.setAngle(7, 180+val7);
  servos.setAngle(8, 180+val8);
}


void read(){
  if (Serial.available() > 0) {
    // 读取串口输入的数据，按逗号分隔
    servo1Offset = Serial.readStringUntil(',').toInt();
    servo2Offset = Serial.readStringUntil(',').toInt();
    servo3Offset = Serial.readStringUntil(',').toInt(); 
    servo4Offset = Serial.readStringUntil(',').toInt(); // 最后一个数以换行结尾
    servo5Offset = Serial.readStringUntil(',').toInt();
    servo6Offset = Serial.readStringUntil(',').toInt();
    servo7Offset = Serial.readStringUntil(',').toInt(); 
    servo8Offset = Serial.readStringUntil('\n').toInt();
  }
}

void setup(){
  Serial.begin(115200);
  Wire.begin(1, 2, 400000UL);
  servos.init();
  servos.setAngleRange(0,300);
  servos.setPluseRange(500,2500);

}


void loop() {
  // put your main code here, to run repeatedly:
  read();
  Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",servo1Offset,servo2Offset,servo3Offset,servo4Offset,servo5Offset,servo6Offset,servo7Offset,servo8Offset);
  setServosAngle(servo1Offset,servo2Offset,servo3Offset,servo4Offset,servo5Offset,servo6Offset,servo7Offset,servo8Offset);//四个舵机的舵值
}



