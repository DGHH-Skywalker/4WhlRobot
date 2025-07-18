#include <Arduino.h>
#include "MT6701_SSI.h"
#include "pid.h"
#include "lowpass_filter.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
float voltage_power_supply;
float Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0;
#define _3PI_2 4.71238898038f
float zero_electric_angle = 0;
int PP = 1, DIR = 1;
int pwmA = 32;
int pwmB = 33;
int pwmC = 25;
Sensor_MT6701_SSI sensor = Sensor_MT6701_SSI(0);
SPIClass hspi(VSPI);
//初始变量及函数定义
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。1

PIDController VelPID = PIDController(0.5, 0, 0, 100000, 6);
PIDController AnglePID = PIDController(100, 0, 0, 100000, 1000);
LowPassFilter VelFLT = LowPassFilter(0.05);



// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);  //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2 * PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc) {

  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  //写入PWM到PWM 0 1 2 通道
  ledcWrite(0, dc_a * 255);
  ledcWrite(1, dc_b * 255);
  ledcWrite(2, dc_c * 255);
}

void setTorque(float Uq, float angle_el) {
  sensor.Sensor_update();
  Uq = _constrain(Uq, -voltage_power_supply / 2, voltage_power_supply / 2);
  float Ud = 0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha = -Uq * sin(angle_el);
  Ubeta = Uq * cos(angle_el);

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  setPwm(Ua, Ub, Uc);
}

void DFOC_Vbus(float power_supply) {
  voltage_power_supply = power_supply;
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  ledcSetup(0, 30000, 8);  //pwm频道, 频率, 精度
  ledcSetup(1, 30000, 8);  //pwm频道, 频率, 精度
  ledcSetup(2, 30000, 8);  //pwm频道, 频率, 精度
  ledcAttachPin(pwmA, 0);
  ledcAttachPin(pwmB, 1);
  ledcAttachPin(pwmC, 2);
  Serial.println("完成PWM初始化设置");
  // sensor.enableInterrupts(doA, doB, doC);
  hspi.begin(MT6701_CLK, MT6701_DO, 12, -1);  //CLK,MISO,MOSI,CS
  sensor.Sensor_init(&hspi);
}


float _electricalAngle() {
  return _normalizeAngle((float)(DIR * PP) * sensor.getSensorAngle() - zero_electric_angle);
}


void DFOC_alignSensor(int _PP, int _DIR) {
  PP = _PP;
  DIR = _DIR;
  setTorque(1, _3PI_2);
  delay(3000);
  sensor.Sensor_update();
  zero_electric_angle = _electricalAngle();
  setTorque(0, _3PI_2);
  Serial.print("0电角度：");
  Serial.println(zero_electric_angle);
}

float DFOC_M0_Angle() {
  return sensor.getAngle();
}


float DFOC_M0_Velocity() {
  float vel = sensor.getVelocity();
  float velFlt = VelFLT(vel);
  return velFlt;
}

void setVelPID(float P,float I,float D,float ramp){
  VelPID.P = P;
  VelPID.I = I;
  VelPID.D = D;
  VelPID.output_ramp = ramp;
}

void setAnglePID(float P,float I,float D,float ramp){
  AnglePID.P = P;
  AnglePID.I = I;
  AnglePID.D = D;
  AnglePID.output_ramp = ramp;
}

void setVelocity(float target){
  // setTorque(VelPID((target-DFOC_M0_Velocity())*180/PI),_electricalAngle());   //速度闭环
  setTorque(VelPID(target-DFOC_M0_Velocity()),_electricalAngle());   //速度闭环
}

void setAngle(float target){
  // setTorque(VelPID((target-DFOC_M0_Angle())*180/PI),_electricalAngle());
  setTorque(VelPID(target-DFOC_M0_Angle()),_electricalAngle());
}

//==============串口接收==============
float motor_target;
int commaPosition;
String serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {

      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n');  //检测字符串中的逗号
      if (commaPosition != -1)                //如果有逗号存在就向下执行
      {
        motor_target = command.substring(0, commaPosition).toDouble();  //电机角度
        Serial.println(motor_target);
      }
      // reset the command buffer
      received_chars = "";
    }
  }
  return command;
}

float serial_motor_target() {
  return motor_target;
}



float MITController(float angle, float vel, float torque,float kp,float kd){
  float torqueRef = kp * (angle-DFOC_M0_Angle()) + kd * (vel - DFOC_M0_Velocity()) + torque;
  setTorque(torqueRef, _electricalAngle());
  return torqueRef;
}


