#include <Arduino.h>
#include "SF_Servo.h"
#include "SF_IMU.h"
#include <math.h>
#include "bipedal_data.h"
#include "MPU6050_tockn.h"
#include "SF_CAN.h"
#include "config.h"
#include "SF_BLDC.h"
#include "pid.h"

SF_BLDC motors = SF_BLDC(Serial2);
SF_BLDC_DATA BLDCData;
#define myID 0x01

// PIDController pitch_pid_big=PIDController(0.2,0.01,0,10000,50);
PIDController pitch_pid_mini=PIDController(3.5,-0.009,0,10000,50);
// PIDController roll_pid_big=PIDController(0.2,0.01,0,10000,50);
// PIDController roll_pid_big=PIDController(0.2,0.01,0,10000,50);
PIDController roll_pid_mini=PIDController(-0.35,0.008,0,10000,50);

int value = 0; 
float pitch_off=2;
float roll_off=-6;
// 引入CAN控制
SF_CAN CAN;
#define TRANSMIT_RATE_MS 1
unsigned long previousMillis = 0;  // will store last time a message was send

float  Tartget_Roll_angle =0;
float  Tartget_pitch_angle =0;
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

motor_data MotorData;
motorstatus motorStatus;

int flat = 0;//模式切换标志位
//设置轮子方向dir
void setRobotparam(){
  pitch_off = 0;
  roll_off = -6;
  //后面两个电机速度控制方向
  motorStatus.M0Dir = 1;//左后电机控制方向
  motorStatus.M1Dir = -1;//右后电机控制方向

  // flat = 1;//电机转速反馈方向校准模式，将电机设置为固定转速
  flat = 0;//电机取消反馈方向校准模式
  //后面两个电机速度反馈方向
  motorStatus.M0SpdDir = 1;//左后电机反馈方向
  motorStatus.M1SpdDir = -1;//右后电机反馈方向

  //前面两个电机速度控制方向
  motorStatus.M3Dir = -1;//左前电机控制方向
  motorStatus.M4Dir = -1;//右前电机控制方向
}

SF_Servo servos = SF_Servo(Wire); // 实例化舵机
IKparam IKParam;
MPU6050 mpu6050 = MPU6050(Wire);
void read();
void setpid();
float roll, pitch, yaw, init_pitch;
float gyroX, gyroY, gyroZ;
float taget_pitch, target_roll;
float target_pitch_prev, target_roll_prev;
// 引入遥控器控制
#define PPM_PIN 15                                                                             // Pin connected to the PPM signal
#define NUM_CHANNELS 8                                                                         // The number of PPM channels you expect
#define SYNC_GAP 3000                                                                          // Minimum pulse length in microseconds that signals a frame reset
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数
volatile uint16_t ppmValues[NUM_CHANNELS] = {0};
volatile uint8_t currentChannel = 0;
volatile uint32_t lastTime = 0;

hw_timer_t *timer = NULL; // Timer for accurate pulse width measurement
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

float steering;
float forwardBackward,forwardBackward_h;
float remote_H = 0;
float roll_EH = 0;
float t = 0;
const float pi = 3.1415926;
float height = 120;
float right_front = 70, right_rear = 70, left_front = 70, left_rear = 70;
float x1 = 0, x2 = 0, x3 = 0, x4 = 0, Y1 = right_front, y2 = left_front, y3 = right_rear, y4 = left_rear;
int r1 = -1, r2 = -1, r3 = 1, r4 = 1;
float faai = 0.5, Ts = 1;
// float xf = -45, xs = 45, h = 60;
float left_front_h = 60, left_rear_h = 60, right_front_h = 60, right_rear_h = 60;
float xf = 0, xs = 0, h = 60;
float sita1_1 = 0, sita2_1 = 0, sita1_2 = 0, sita2_2 = 0;
float sita1_3 = 0, sita2_3 = 0, sita1_4 = 0, sita2_4 = 0;
const float l1 = 60, l2 = 100;
int move_target = 0;
uint8_t motionMode = 0;
int controlmode = 0;

float motor1target, motor2target, sendmotor1target, sendmotor2target;
int steadyState = 0;


// ppm读取中断程序
void IRAM_ATTR onPPMInterrupt()
{
  uint32_t now = micros();
  uint32_t duration = now - lastTime;
  lastTime = now;

  if (duration >= SYNC_GAP)
  {
    // Sync pulse detected, reset channel index
    currentChannel = 0;
  }
  else if (currentChannel < NUM_CHANNELS)
  {
    ppmValues[currentChannel] = duration;
    currentChannel++;
  }
}

// 限幅函数
float constrainValue(float value, float minValue, float maxValue)
{
  if (value > maxValue)
    return maxValue;
  if (value < minValue)
    return minValue;
  return value;
}

void trot()
{
  float sigma, zep, xep_b, xep_z, left_rear_zep, left_front_zep, right_rear_zep, right_front_zep;
  // left_front_zep = h * (1 - cos(sigma)) / 2;
  // left_rear_zep = h * (1 - cos(sigma)) / 2;
  // right_front_zep = h * (1 - cos(sigma)) / 2;
  // right_rear_zep = h * (1 - cos(sigma)) / 2;

  // Serial.println(zep);
  if (t <= Ts * faai)
  {
    sigma = 2 * pi * t / (faai * Ts);
    left_front_zep = left_front_h * (1 - cos(sigma)) / 2;
    left_rear_zep = left_rear_h * (1 - cos(sigma)) / 2;
    right_front_zep = right_front_h * (1 - cos(sigma)) / 2;
    right_rear_zep = right_rear_h * (1 - cos(sigma)) / 2;
    // left_front_zep = h * (1 - cos(sigma)) / 2;
    // left_rear_zep = h * (1 - cos(sigma)) / 2;
    // // right_front_zep =
    // right_rear_zep = h * (1 - cos(sigma)) / 2;
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * pi)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * pi)) + xf;

    // 输出y
    // 双足步态
    if (controlmode == 1)
    {
      if (motionMode == 0)
      {
        
        Y1 = -left_front_zep + height+20;
        y2 = 0 + height;
        y3 = -right_rear_zep + height+20;
        y4 = 0 + height;
      }
      else if (motionMode == 1)
      {
        // 狗子步态
        Y1 = -left_front_zep + height;
        y2 = 0 + height;
        y3 = 0 + height;
        y4 = -left_rear_zep + height;
      }
    }

    else if (controlmode == 0)
    {
      //双足步态
      //固定值+遥控器前后和轮速反馈+遥控器左右+遥控器陀螺仪保持水平
      Y1 = left_front_h + 50 + forwardBackward_h/2 + steering + steadyState * -taget_pitch + 1 * remote_H + roll_EH + steadyState * target_roll;
      Y1 = constrainValue(Y1, 70, 150);
      y2 = right_front_h + 50 + forwardBackward_h/2 - steering + steadyState * -taget_pitch + 1 * remote_H - roll_EH - steadyState * target_roll;
      y2 = constrainValue(y2, 70, 150);
      y3 = right_rear_h + 50 - forwardBackward_h/2 + steering - steadyState * -taget_pitch + 1 * remote_H + roll_EH +  steadyState * target_roll;
      y3 = constrainValue(y3, 70, 150);
      y4 = left_rear_h + 50 - forwardBackward_h/2 - steering - steadyState * -taget_pitch + 1 * remote_H - roll_EH - steadyState * target_roll;
      y4 = constrainValue(y4, 70, 150);
    }

    // 输出x
    x1 = -xep_z * r1 + move_target;
    x2 = -xep_b * r2 + move_target;
    x3 = -xep_z * r3 + move_target;
    x4 = -xep_b * r4 + move_target;
  }
  else if (t > Ts * faai && t <= Ts)
  {
    sigma = 2 * pi * (t - Ts * faai) / (faai * Ts);
    left_front_zep = left_front_h * (1 - cos(sigma)) / 2;
    left_rear_zep = left_rear_h * (1 - cos(sigma)) / 2;
    right_front_zep = right_front_h * (1 - cos(sigma)) / 2;
    right_rear_zep = right_rear_h * (1 - cos(sigma)) / 2;
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * pi)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * pi)) + xf;

    // 输出y
    if (controlmode == 1)
    {
      if (motionMode == 0)
      {
        
        Y1 = 0 + height;
        y2 = -right_front_zep + height+20;
        y3 = 0 + height;
        y4 = -left_rear_zep + height+20;
      }
      else if (motionMode == 1)
      {
        Y1 = 0 + height;
        y2 = -right_front_zep + height;
        y3 = -right_rear_zep + height;
        y4 = 0 + height;
      }
    }

    else if (controlmode == 0)
    {
      //双足步态
      //固定值+遥控器前后和轮速反馈
      Y1 = left_front_h + 50 + forwardBackward_h/2 + steering + steadyState * -taget_pitch + 1 * remote_H + roll_EH + steadyState * target_roll;
      Y1 = constrainValue(Y1, 70, 150);
      y2 = right_front_h + 50 + forwardBackward_h/2 - steering + steadyState * -taget_pitch + 1 * remote_H - roll_EH - steadyState * target_roll;
      y2 = constrainValue(y2, 70, 150);
      y3 = right_rear_h + 50 - forwardBackward_h/2 + steering - steadyState * -taget_pitch + 1 * remote_H + roll_EH +  steadyState * target_roll;
      y3 = constrainValue(y3, 70, 150);
      y4 = left_rear_h + 50 - forwardBackward_h/2 - steering - steadyState * -taget_pitch + 1 * remote_H - roll_EH - steadyState * target_roll;
      y4 = constrainValue(y4, 70, 150);
    }

    // 输出x
    x1 = -xep_b * r1 + move_target;
    x2 = -xep_z * r2 + move_target;
    x3 = -xep_b * r3 + move_target;
    x4 = -xep_z * r4 + move_target;
  }
}


#define L1 60
#define L2 100
#define L3 100
#define L4 60
#define L5 40
#define L6 0

#define LFSERVO_OFFSET
#define RFSERVO_OFFSET
#define LRSERVO_OFFSET
#define RRSERVO_OFFSET

void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear,
                   uint16_t servoRightFront, uint16_t servoRightRear,
                   uint16_t servoBackLeftFront, uint16_t servoBackLeftRear,
                   uint16_t servoBackRightFront, uint16_t servoBackRightRear)
{
  //舵机偏置调节3,5,-5,-7,3,-5,-8,8
  // 控制前两条腿
  servos.setAngle(3, servoLeftFront + SERVO_OFFSET_3);  // 1左前腿 -前
  servos.setAngle(4, servoLeftRear + SERVO_OFFSET_4);   // 1左前腿 -后
  servos.setAngle(2, servoRightFront + SERVO_OFFSET_2); // 1右前腿 - 前
  servos.setAngle(1, servoRightRear + SERVO_OFFSET_1);  // 1右前腿 - 后
  
  // 控制后两条腿
  servos.setAngle(7, servoBackLeftFront + SERVO_OFFSET_7); // 右后腿-后
  servos.setAngle(8, servoBackLeftRear + SERVO_OFFSET_8);   // 右后腿-前
  servos.setAngle(6, servoBackRightFront + SERVO_OFFSET_6);// 左后腿-后
  servos.setAngle(5, servoBackRightRear + SERVO_OFFSET_5);  // 左后腿-前
  // // 控制后两条腿
  // servos.setAngle(7, servoBackLeftFront + -8);  // 2左后腿 - 前   2右后腿-后
  // servos.setAngle(8, servoBackLeftRear + 8);   // 2左后腿 - 后  2右后腿-前
  // servos.setAngle(6, servoBackRightFront + -5); // 2右后腿 - 前   2左后腿-后
  // servos.setAngle(5, servoBackRightRear + 3);  // 2右后腿 - 后       2左后腿-前
  
}

int16_t alphaLeftToAngle, betaLeftToAngle, alphaRightToAngle, betaRightToAngle;
int16_t alphaBackRightToAngle, betaBackRightToAngle, alphaBackLefToAngle, betaBackLeftToAngle;
int16_t alpha1ToAngle, beta1ToAngle, alpha2ToAngle, beta2ToAngle;
void inverseKinematics()
{
  // x1 = 0;
  // Y1 = right_front + remote_H;

  // // x2 = 0;
  // y2 =left_front + remote_H;

  // // x3 = 0;
  // y3 = right_rear + remote_H;
  // x4 = 0;
  // y4 = 70;

  float alpha1, alpha2, beta1, beta2;
  uint16_t servoLeftFront, servoLeftRear, servoRightFront, servoRightRear;
  uint16_t servoBackLeftFront, servoBackLeftRear, servoBackRightFront, servoBackRightRear;

  // x1 = -x1;
  // x2 = -x2;

  x3 = -x3;
  x4 = -x4;

  float aRight = 2 * x2 * L1;
  float bRight = 2 * y2 * L1;
  float cRight = x2 * x2 + y2 * y2 + L1 * L1 - L2 * L2;
  float dRight = 2 * L4 * (x2 - L5);
  float eRight = 2 * L4 * y2;
  float fRight = ((x2 - L5) * (x2 - L5) + L4 * L4 + y2 * y2 - L3 * L3);

  IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
  beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 4)
    IKParam.alphaRight = alpha1;
  else
    IKParam.alphaRight = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 4)
    IKParam.betaRight = beta1;
  else
    IKParam.betaRight = beta2;

  float aLeft = 2 * x1 * L1;
  float bLeft = 2 * Y1 * L1;
  float cLeft = x1 * x1 + Y1 * Y1 + L1 * L1 - L2 * L2;
  float dLeft = 2 * L4 * (x1 - L5);
  float eLeft = 2 * L4 * Y1;
  float fLeft = ((x1 - L5) * (x1 - L5) + L4 * L4 + Y1 * Y1 - L3 * L3);

  alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
  beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 4)
    IKParam.alphaLeft = alpha1;
  else
    IKParam.alphaLeft = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 4)
    IKParam.betaLeft = beta1;
  else
    IKParam.betaLeft = beta2;

  alphaLeftToAngle = (int)((IKParam.alphaLeft / 6.28) * 360); // 弧度转角度
  betaLeftToAngle = (int)((IKParam.betaLeft / 6.28) * 360);

  alphaRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
  betaRightToAngle = (int)((IKParam.betaRight / 6.28) * 360);

  servoLeftFront = 90 + betaLeftToAngle;
  servoLeftRear = 90 + alphaLeftToAngle;
  servoRightFront = 270 - betaRightToAngle;
  servoRightRear = 270 - alphaRightToAngle;

  // 新增两条后腿的逆解计算逻辑 (以 x3, y3 和 x4, y4 为坐标)
  float aBackRight = 2 * x3 * L1;
  float bBackRight = 2 * y3 * L1;
  float cBackRight = x3 * x3 + y3 * y3 + L1 * L1 - L2 * L2;
  float dBackRight = 2 * L4 * (x3 - L5);
  float eBackRight = 2 * L4 * y3;
  float fBackRight = ((x3 - L5) * (x3 - L5) + L4 * L4 + y3 * y3 - L3 * L3);

  IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = 2 * atan((bBackRight + sqrt((aBackRight * aBackRight) + (bBackRight * bBackRight) - (cBackRight * cBackRight))) / (aBackRight + cBackRight));
  alpha2 = 2 * atan((bBackRight - sqrt((aBackRight * aBackRight) + (bBackRight * bBackRight) - (cBackRight * cBackRight))) / (aBackRight + cBackRight));
  beta1 = 2 * atan((eBackRight + sqrt((dBackRight * dBackRight) + eBackRight * eBackRight - (fBackRight * fBackRight))) / (dBackRight + fBackRight));
  beta2 = 2 * atan((eBackRight - sqrt((dBackRight * dBackRight) + eBackRight * eBackRight - (fBackRight * fBackRight))) / (dBackRight + fBackRight));

  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 4)
    IKParam.alphaRight = alpha1;
  else
    IKParam.alphaRight = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 4)
    IKParam.betaRight = beta1;
  else
    IKParam.betaRight = beta2;

  float aBackLeft = 2 * x4 * L1;
  float bBackLeft = 2 * y4 * L1;
  float cBackLeft = x4 * x4 + y4 * y4 + L1 * L1 - L2 * L2;
  float dBackLeft = 2 * L4 * (x4 - L5);
  float eBackLeft = 2 * L4 * y4;
  float fBackLeft = ((x4 - L5) * (x4 - L5) + L4 * L4 + y4 * y4 - L3 * L3);

  alpha1 = 2 * atan((bBackLeft + sqrt((aBackLeft * aBackLeft) + (bBackLeft * bBackLeft) - (cBackLeft * cBackLeft))) / (aBackLeft + cBackLeft));
  alpha2 = 2 * atan((bBackLeft - sqrt((aBackLeft * aBackLeft) + (bBackLeft * bBackLeft) - (cBackLeft * cBackLeft))) / (aBackLeft + cBackLeft));
  beta1 = 2 * atan((eBackLeft + sqrt((dBackLeft * dBackLeft) + eBackLeft * eBackLeft - (fBackLeft * fBackLeft))) / (dBackLeft + fBackLeft));
  beta2 = 2 * atan((eBackLeft - sqrt((dBackLeft * dBackLeft) + eBackLeft * eBackLeft - (fBackLeft * fBackLeft))) / (dBackLeft + fBackLeft));

  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 4)
    IKParam.alphaLeft = alpha1;
  else
    IKParam.alphaLeft = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 4)
    IKParam.betaLeft = beta1;
  else
    IKParam.betaLeft = beta2;
  // setServoAngle(servoLeftFront,servoLeftRear,servoRightFront,servoRightRear);

  alphaBackLefToAngle = (int)((IKParam.alphaLeft / 6.28) * 360); // 弧度转角度
  betaBackLeftToAngle = (int)((IKParam.betaLeft / 6.28) * 360);

  alphaBackRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
  betaBackRightToAngle = (int)((IKParam.betaRight / 6.28) * 360);

  servoBackLeftFront = 90 + betaBackLeftToAngle;
  servoBackLeftRear = 90 + alphaBackLefToAngle;

  servoBackRightFront = 270 - betaBackRightToAngle;
  servoBackRightRear = 270 - alphaBackRightToAngle;

  // 最后调用函数驱动所有 8 个舵机
  setServoAngle(servoLeftFront, servoLeftRear, servoRightFront, servoRightRear,
                servoBackLeftFront, servoBackLeftRear, servoBackRightFront, servoBackRightRear);
}

// int mapJoystickValuerollzeparam(int inputValue)
// {
//   // Ensure the input value is within the expected range
//   if (inputValue < 1000)
//     inputValue = 1000;
//   if (inputValue > 2000)
//     inputValue = 2000;
//   int output = ((inputValue - 1000) / 20);
//   return output;
// }

float mapJoystickValuerollforwardback(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue > 1430 && inputValue < 1500)
    inputValue = 1500;
  float output = ((inputValue - 1500) / 12);

  return output;
}

float mapJoystickValuesteering(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue > 1420 && inputValue < 1500)
    inputValue = 1500;
  float mappedValue = (inputValue - 1500) / 12;
  return mappedValue;
}

int mapJoystickValuerollzeparam(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue > 1420 && inputValue < 1500)
    inputValue = 1500;
  int output = ((inputValue - 1500) / 20);

  return output;
}

void setup()
{
  
  controlmode = 0;
  motionMode = 0;
  steadyState = 0;
  Serial.begin(115200); // 用于调试
  
  setRobotparam();
  
  Wire.begin(1, 2, 400000UL);
  CAN.init(CAN_TX, CAN_RX);
  CAN.setMode(1);
  CAN.setDeviceID(0x01);
  
  servos.init();
  servos.setAngleRange(0, 300);
  servos.setPluseRange(500, 2500);
  
  motors.init();
  motors.setModes(4, 4);


  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), onPPMInterrupt, RISING);

  trot();//让机器人初始状态是站立的，让陀螺仪自检
  inverseKinematics();
  delay(5000);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true,1000,1000);
  Serial.println("初始化完成");
  delay(5000);
}

float rollBias, pitchBias;
float alpha = 0.02; // 设置滤波系数（值越小，平滑度越高）

float lowPassFilter(float currentValue, float previousValue, float alpha)
{
  // currentValue: 当前读取的遥控器数据
  // previousValue: 上一次滤波后的数据
  return alpha * currentValue + (1 - alpha) * previousValue;
}

// 设置滤波后的数据
uint16_t filteredPPMValues1 = 1500; // 通道1   控制左右转弯
uint16_t filteredPPMValues2 = 1500; // 通道2   控制前进后退
uint16_t filteredPPMValues3 = 0; // 通道3
uint16_t filteredPPMValues4 = 1500; // 通道4
uint16_t filteredPPMValues5 = 2000; // 通道5
uint16_t filteredPPMValues6 = 0; // 通道6
uint16_t filteredPPMValues7 = 2000; // 通道7   控制模式切换 双足形态和狗子形态
uint16_t filteredPPMValues8 = 0; // 通道8

void remote_mode_switch()
{
  if (filteredPPMValues7 < 1200)
  {
    motionMode = 0;
    steadyState = 0;
    t = t + 0.009;
  }
  else
  {
    motionMode = 1;
    steadyState = 1;
    t = t + 0.01;
  }
}
// 车子和狗形态的切换模式
void mode_change()
{
  if (filteredPPMValues5 > 1900)
  {
    // 车轮形态
    controlmode = 0;
    xf = 0;
    xs = 0;
  }
  else
  {
    // 狗子步态
    controlmode = 1;
    if (motionMode == 1)
    {
      // steering 将 steering 设置进去 // 步态转弯 一边xs xf大 一边xs xf小
      xs = -forwardBackward * 0.6;
      xf = forwardBackward * 0.6;
    }
  }
}

void getMotorValue()
{
  BLDCData = motors.getBLDCData();
  motorStatus.M0Speed = motorStatus.M0SpdDir * BLDCData.M0_Vel;
  motorStatus.M1Speed = motorStatus.M1SpdDir * BLDCData.M1_Vel;
}

float mapToRange(float input)
{
  // 定义输入范围
  float inputMin = 930.0;
  float inputMax = 1970.0;

  // 映射到输出范围 [0, 1]
  float outputMin = 0.0;
  float outputMax = 1.0;

  // 计算映射值
  if (input < inputMin)
    input = inputMin; // 防止输入值小于范围
  if (input > inputMax)
    input = inputMax; // 防止输入值大于范围
  return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}


//二进制转浮点型
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
//浮点型转二进制
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits){
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

uint8_t r_buf[8];
void can_control()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    getMotorValue();

    uint8_t motorcommand[8];
    MotorData.motor1taget = _constrain(MotorData.motor1taget,-100,100);
    MotorData.motor2taget = _constrain(MotorData.motor2taget,-100,100);
    uint16_t target1_int = float_to_uint(MotorData.motor1taget, -100.0, 100.0, 16); // 压缩为 16 位
    uint16_t target2_int = float_to_uint(MotorData.motor2taget, -100.0, 100.0, 16); // 压缩为 16 位

    uint32_t t_id;
    t_id = 0x02;
    motorcommand[0] = target1_int >> 8;          // target1 高 8 位
    motorcommand[1] = target1_int & 0xFF;        // target1 低 8 位
    motorcommand[2] = target2_int >> 8;          // target2 高 8 位
    motorcommand[3] = target2_int & 0xFF;        // target2 低 8 位
    motorcommand[4] = 0;         
    motorcommand[5] = 0;       
    motorcommand[6] = 0;          
    motorcommand[7] = 0;

    CAN.sendMsg(&t_id, motorcommand);//发送数据
  }
  // CAN.receiveMsg(r_buf);//接收数据
  // // CAN.listenAllMsg(10);//监视接收到的信息
  // uint16_t target1_rec = (r_buf[0] << 8) | r_buf[1];
  // uint16_t target2_rec = (r_buf[2] << 8) | r_buf[3];
  
  //   // 解压为浮点数
  // float motor3Speed = uint_to_float(target1_rec, -100.0, 100.0, 16);//接收左前轮轮速
  // float motor4Speed = uint_to_float(target2_rec, -100.0, 100.0, 16);//接收右前轮轮速
  // Serial.printf("motor3Speed: %.2f, motor4Speed: %.2f\n", motor3Speed, motor4Speed);

}

float target_vel = 40;
float target_vel_prev=0;
float target_forwardback;

float kp = 1.2; // 比例系数
float ki = 0.02; // 积分系数
float accumulated_error = 0; // 全局误差累计


int count = 0;

float pitch_pid_kp ;
float pitch_pid_ki ;
float roll_pid_kp ;
float roll_pid_ki ;
void read(){
  
  static String received_chars;
  String command = "";
  if (Serial.available() > 0) 
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n') 
    {
      // 用户输入结束
        command = received_chars;
        const char* str = command.c_str();
        sscanf(str,"%f,%f",&pitch_off,&roll_off);
        Serial.print(pitch_off);
        Serial.print(',');
        Serial.println(roll_off);
        // setpid();
        received_chars = "";
    }
  }
}
void setpid(){
  pitch_pid_mini.P = pitch_pid_kp;
  pitch_pid_mini.I = pitch_pid_ki;
  roll_pid_mini.P = roll_pid_kp;
  roll_pid_mini.I = roll_pid_ki;
}

void loop()
{
  read(); // 串口读取
  can_control();

  mpu6050.update();
  remote_mode_switch();
  mode_change();
  pitch = mpu6050.getAngleX()+pitch_off;
  if(pitch > -2 && pitch < 2)pitch = 0;
  roll = -mpu6050.getAngleY()+roll_off;
  if(roll > -2 && roll < 2)roll = 0;
  yaw = mpu6050.getAngleZ();
  gyroY = mpu6050.getGyroY();
  gyroX = mpu6050.getGyroX();
  gyroZ = mpu6050.getGyroZ();
  filteredPPMValues1 = lowPassFilter(ppmValues[0], filteredPPMValues1, alpha);
  filteredPPMValues2 = lowPassFilter(ppmValues[1], filteredPPMValues2, alpha);
  filteredPPMValues3 = lowPassFilter(ppmValues[2], filteredPPMValues3, alpha);
  filteredPPMValues4 = lowPassFilter(ppmValues[3], filteredPPMValues4, alpha);
  filteredPPMValues5 = lowPassFilter(ppmValues[4], filteredPPMValues5, alpha);
  filteredPPMValues6 = lowPassFilter(ppmValues[5], filteredPPMValues6, alpha);
  filteredPPMValues7 = lowPassFilter(ppmValues[6], filteredPPMValues7, alpha);
  filteredPPMValues8 = lowPassFilter(ppmValues[7], filteredPPMValues8, alpha);

  remote_H = mapJoystickValuerollzeparam(filteredPPMValues3);//遥控器腿高
  roll_EH = 1.5*mapJoystickValuerollzeparam(filteredPPMValues4);//遥控器翻滚
  // Serial.printf("%d,%d,%d,%d,%d,%.d,%d,%d\n",
  //               filteredPPMValues1, filteredPPMValues2, filteredPPMValues3, filteredPPMValues4,
  //               filteredPPMValues5, filteredPPMValues6, filteredPPMValues7, filteredPPMValues8);

  forwardBackward = mapJoystickValuerollforwardback(filteredPPMValues2);//遥控器前后
  steering = mapJoystickValuesteering(filteredPPMValues1);//遥控器左右
  target_vel = 1 * (motorStatus.M0Speed + motorStatus.M1Speed)/2;
  target_vel = lowPassFilter(target_vel, target_vel_prev, alpha);
  target_forwardback = 2.2 * forwardBackward;
  target_vel = constrain(target_vel,-50,50);
  target_forwardback = constrain(target_forwardback,-50,50);
  forwardBackward_h = 1.25 * (target_forwardback + target_vel);

  target_vel_prev = target_vel;
  // forwardBackward_h = 0;
  int sign = steering > 0 ? 1 : -1;//机器人左转时左轮慢一点右轮快一点；右转时左轮快一点右轮慢一点
  float mappedValue = mapToRange(filteredPPMValues6);
  if (mappedValue < 0.06)
    mappedValue = 0;
  if (controlmode == 0)
  {
    motor1target = 0.35 * (forwardBackward + 0.32 * motorStatus.M0Speed) - 1 * steering / 3;//左后
    motor2target = 0.35 * (forwardBackward +  0.32 * motorStatus.M1Speed) + 1 * steering / 3;//右后
    // sendmotor1target = -((0.18 * (forwardBackward - motorStatus.M0Speed)) - 1 * steering / 20);
    // sendmotor2target = -(0.18 * (forwardBackward + motorStatus.M1Speed)) + 1 * steering / 20);
    sendmotor1target = 0.35 * (forwardBackward + 0.32 * motorStatus.M0Speed) + 1 * steering / 3;//右前
    sendmotor2target = 0.35 * (forwardBackward +  0.32 * motorStatus.M1Speed) - 1 * steering / 3;//左前
    
    // motor1target = 0.35 * (forwardBackward  + 0.32 * motorStatus.M0Speed) - 1 * steering - sign*steering/6;//左后
    // motor2target = 0.35 * (forwardBackward  + 0.32 * motorStatus.M0Speed) + 1 * steering - sign*steering/6;//右后
    // sendmotor1target = 0.35 * (forwardBackward + 0.32 * motorStatus.M0Speed) + 1 * steering - sign*steering/6;//右前
    // sendmotor2target = 0.35 * (forwardBackward + 0.32 * motorStatus.M0Speed) - 1 * steering - sign*steering/6;//左前
    sendmotor1target = sendmotor1target;
    sendmotor2target = sendmotor2target;
  }
  else
  {
    motor1target = mappedValue * (0.35 * (forwardBackward + 0.02 * motorStatus.M0Speed) - 1 * steering/3);
    motor2target = mappedValue * (0.35 * (forwardBackward + 0.02 * motorStatus.M1Speed) + 1 * steering/3);

    sendmotor1target = mappedValue * (0.35 * (forwardBackward + 0.02 * motorStatus.M1Speed) + 1 * steering/6);
    sendmotor2target = mappedValue * (0.35 * (forwardBackward + 0.02 * motorStatus.M0Speed) - 1 * steering/6);

    sendmotor1target = sendmotor1target;
    sendmotor2target = sendmotor2target;
    // sendmotor1target = 0;
    // sendmotor2target = 0;
  }

  count++;
  if(count>200){
    count = 0;
    Serial.printf("%f,%f,%f\n",target_vel,motorStatus.M0SpdDir * BLDCData.M0_Vel,motorStatus.M1SpdDir * BLDCData.M1_Vel);
    // Serial.printf("%f,%f,%f,%f,%d\n",- 1 * steering/3,- sign*steering/2,sendmotor1target,sendmotor2target,sign);
  }
  //控制模式
  if(flat==0)
    motors.setTargets(motorStatus.M0Dir*motor1target, motorStatus.M1Dir*motor2target);
  //调试模式
  else if(flat==1)
    motors.setTargets(2, 2);
  MotorData.motor1taget = motorStatus.M4Dir*sendmotor1target;//can发送给另一块主控的右前轮目标速度
  MotorData.motor2taget = motorStatus.M3Dir*sendmotor2target;//can发送给另一块主控的左前轮目标速度
  // MotorData.motor1taget = 3;
  // MotorData.motor2taget = 8;
  // 单纯步态步频
  
  if (t >= Ts)
  {
    t = 0;
  }

  pitchBias = taget_pitch + 0.00008 * taget_pitch;
  trot();
  if(steadyState == 1)//自稳
  // {
    Tartget_Roll_angle = (0 - roll ) * roll_pid_mini.P;
    target_roll = target_roll - roll_pid_mini.I * (Tartget_Roll_angle - gyroY); //-gyroX
    target_roll = constrainValue(target_roll, -90, 90);
    
    Tartget_pitch_angle = (0 - pitch ) * pitch_pid_mini.P;
    taget_pitch = taget_pitch - pitch_pid_mini.I * (Tartget_pitch_angle - gyroX); //-gyroZ
    taget_pitch = constrainValue(taget_pitch, -90, 90);

  // }else{
  //   taget_pitch = 0;
  //   target_roll = 0;
  // }
  
  inverseKinematics();

}
