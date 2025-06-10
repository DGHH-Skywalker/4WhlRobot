#include <Arduino.h>

const int ledPin = 25;  // GPIO 25  对应主控板背面EN引脚

void setup() {
  pinMode(ledPin, OUTPUT);  // 设置 GPIO 25 为输出模式
}

void loop() {
  digitalWrite(ledPin, HIGH);  // 点亮 LED
  delay(1000);                 // 延时 1 秒
  digitalWrite(ledPin, LOW);   // 熄灭 LED
  delay(1000);                 // 延时 1 秒
}