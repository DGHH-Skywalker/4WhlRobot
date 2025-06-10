#include<arduino.h>

#define ADC_PIN 25  // 假设你使用的是GPIO25,主控板背面的EA引脚

void setup() {
  Serial.begin(115200);  // 初始化串口通信
  pinMode(ADC_PIN, INPUT);  // 设置ADC引脚为输入模式
}

void loop() {
  int adcValue = analogRead(ADC_PIN);  // 读取ADC值
  Serial.println(adcValue);  // 将ADC值打印到串口监视器
  delay(1000);  // 延迟1秒
}