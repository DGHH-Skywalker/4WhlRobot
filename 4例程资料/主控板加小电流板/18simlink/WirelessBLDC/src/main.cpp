#include <Arduino.h>
#include "SF_BLDC.h"
#include "SF_Wireless.h"

// UDP 连接
unsigned int local = 2317;           // 通讯端口
const char *password = "Toan123456"; // wifi密码
const char *ssid = "TOAN_2.4G";      // wifi名称
uint8_t cnt;
IPAddress ipServidor(192, 168, 10, 28); // 电脑IP(要链接的IP)
IPAddress ipClient(192, 168, 10, 180);  // 扩展主控的IP  自己设置

SF_BLDC motors = SF_BLDC(Serial2);
SF_Wireless wireless = SF_Wireless(ssid, password); // 参数1:IP名 参数2:wifi密码

SF_BLDC_Wireless_DATA REV_Matlab_Values; // 定义UDP数据接收对象
SF_BLDC_DATA driver;

// 无线数据接收任务进程
void wiressRecv(void *pvParameters)
{
    while (1)
    {
        REV_Matlab_Values = wireless.recFromWireless();
        vTaskDelay(1);
    }
}

// 无线数据发送函数
void wiressSend()
{
    cnt++;
    if (cnt > 2)
    {
        cnt = 0;
        wireless.sendToWireless(driver);
        // Serial.print(driver.M0_angle,4);
        // Serial.print("\n");
        // Serial.println(REV_Matlab_Values.M0_Target);
    }
}

void setup()
{
    Serial.begin(115200);
    motors.init();                                  // 电机控制部分初始化
    wireless.WiFiInit(ipClient, ipServidor, local); // UDP 无线网络初始化
    xTaskCreatePinnedToCore(wiressRecv,            // 具体实现的函数
                            "RECV",                 // 任务名称
                            8192,                   // 堆栈大小
                            NULL,                   // 输入参数
                            1,                      // 任务优先级
                            NULL,                   //
                            0                       // 核心  0\1
    );

    motors.setModes(4, 4); // 控制模式设置
}

void wirelessRun()
{
    driver = motors.getBLDCData();                                               // 获取电机状态数据
    wiressSend();                                                                //无线发送电机状态数据
    motors.setModes(REV_Matlab_Values.M0_Mode, REV_Matlab_Values.M1_Mode);       // 设置电机模式(双力矩)
    motors.setTargets(REV_Matlab_Values.M0_Target, REV_Matlab_Values.M1_Target); // 设置电机目标值

    //角度
    motors.setM0AnglePID(REV_Matlab_Values.M0_POS_P, REV_Matlab_Values.M0_POS_I, REV_Matlab_Values.M0_POS_D, 100000);
    motors.setM1AnglePID(REV_Matlab_Values.M1_POS_P, REV_Matlab_Values.M1_POS_I, REV_Matlab_Values.M1_POS_D, 100000);
    //速度
    motors.setM0VelPID(REV_Matlab_Values.M0_VEL_P, REV_Matlab_Values.M0_VEL_I, REV_Matlab_Values.M0_VEL_D, 100000);
    motors.setM1VelPID(REV_Matlab_Values.M1_VEL_P, REV_Matlab_Values.M1_VEL_I, REV_Matlab_Values.M1_VEL_D, 100000);
    //电流
    motors.setM0CurrentPID(REV_Matlab_Values.M0_CURR_P, REV_Matlab_Values.M0_CURR_I, REV_Matlab_Values.M0_CURR_D, 1.5);
    motors.setM1CurrentPID(REV_Matlab_Values.M1_CURR_P, REV_Matlab_Values.M1_CURR_I, REV_Matlab_Values.M1_CURR_D, 1.5);
}

void loop()
{
   wirelessRun();
}
