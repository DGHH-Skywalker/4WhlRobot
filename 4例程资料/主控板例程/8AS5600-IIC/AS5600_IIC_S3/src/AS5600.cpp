#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h> 

#define _2PI 6.28318530718f



// AS5600 相关
double Sensor_AS5600::getSensorAngle() {
  uint8_t angle_reg_msb = 0x0C;//数据地址

  byte readArray[2];//读取数据
  uint16_t readValue = 0;//高位和低位相加后的数据

  wire->beginTransmission(0x36);//初始化
  wire->write(angle_reg_msb);//将指针指向数据地址
  wire->endTransmission(false);//关闭


  wire->requestFrom(0x36, (uint8_t)2);//读取两字节
  for (byte i=0; i < 2; i++) {
    readArray[i] = wire->read();
  }
  //将两字节数据转化为角度值
  int _bit_resolution=12;
  int _bits_used_msb=11-7;
  float cpr = pow(2, _bit_resolution);
  int lsb_used = _bit_resolution - _bits_used_msb;

  uint8_t lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
  uint8_t msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );
  
  readValue = ( readArray[1] &  lsb_mask );
  readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
  return (readValue/ (float)cpr) * _2PI; 

}

//AS5600 相关

//=========角度处理相关=============
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;  //使得 Mot_Num 可以统一在该文件调用
   
}
void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire=_wire;
    wire->begin();   //电机Sensor
    delay(500);
    getSensorAngle(); 
    delayMicroseconds(1);//延迟微妙
    vel_angle_prev = getSensorAngle(); 
    vel_angle_prev_ts = micros();
    delay(1);
    getSensorAngle(); 
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); 
    angle_prev_ts = micros();
}
//更新获取的角度值和时间戳
void Sensor_AS5600::Sensor_update() {
    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // 圈数检测
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

//获取角度值（不带圈数），不需要查询硬件获取数据，而是直接获取更新函数中保留的角度值，减少计算量
float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}
//获取角度值，带圈数
float Sensor_AS5600::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}
//获取速度值
float Sensor_AS5600::getVelocity() {
    // 计算采样时间
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // 快速修复奇怪的情况（微溢出）
    if(Ts <= 0) Ts = 1e-3f;
    // 速度计算
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;    
    // 保存变量以待将来使用
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
