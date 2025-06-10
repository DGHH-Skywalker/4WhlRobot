#include <Arduino.h>

const int ledPin = 1;  // GPIO 1  对应主控板背面1号排母

void setup() {
  pinMode(ledPin, OUTPUT);  // 设置 GPIO 1 为输出模式
}

void loop() {
  digitalWrite(ledPin, HIGH);  // 点亮 LED
  delay(1000);                 // 延时 1 秒
  digitalWrite(ledPin, LOW);   // 熄灭 LED
  delay(1000);                 // 延时 1 秒
}