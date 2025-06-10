#include<arduino.h>
#include "AS5600.h"

//AS5600
Sensor_AS5600 S0 = Sensor_AS5600(0);
TwoWire S0_I2C = TwoWire(0);

void setup() {
  Serial.begin(115200);
  S0_I2C.begin(1,2, 400000UL);
  S0.Sensor_init(&S0_I2C);   //初始化编码器0
  Serial.println("编码器加载完毕");
}

void loop() 
{
  S0.Sensor_update(); 
  Serial.println("SSS");
  Serial.println(S0.getAngle());
}