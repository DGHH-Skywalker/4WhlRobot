#include <Wire.h>
#include <arduino.h>
#define MT6701_I2C_ADDRESS 0x06  // MT6701 的设备地址

TwoWire iic0 = TwoWire(0);//如果是AS5600编码器，采用IIC通信，双电机控制需要实例化两条IIC总线

byte readAngle(byte regAddress) {
  byte Byte = 0;

  iic0.beginTransmission(MT6701_I2C_ADDRESS);  // 开始传输到 MT6701
  iic0.write(regAddress);  // 选择角度寄存器，数据地址
  iic0.endTransmission(false);  // 不发送停止信号

  iic0.requestFrom(MT6701_I2C_ADDRESS, 1);  // 请求 1 字节数据
  if (iic0.available() == 1) {
    Byte = iic0.read();  // 读取数据
    return Byte;
  }
  return -1;  
}



void setup() {
  iic0.begin(2,1,400000UL);  // 初始化 I2C 总线
  Serial.begin(115200);  // 初始化串口通信
  Serial.print("iic初始化成功");
}

void loop() {
  byte highByte = readAngle(0x03);  // 读取角度值
  byte lowByte = readAngle(0x04);
  float angle = (highByte << 6) | (lowByte >> 2);
  angle = (angle / 8192.0) * 360.0 / 2;
  angle = angle * (3.141592653589793 / 180.0);
  Serial.print("Angle: ");
  Serial.println(angle);
  delay(1000);  // 每秒读取一次
}

