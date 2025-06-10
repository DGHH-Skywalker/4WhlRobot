#include <Arduino.h>

const int ledPin = 1;  // GPIO 1

void setup() {
  ledcSetup(0, 5000, 8);  // 设置 PWM 通道 0，频率 5000 Hz，8 位分辨率
  ledcAttachPin(ledPin, 0);  // 将 GPIO 1 连接到 PWM 通道 0
}

void loop() {
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    ledcWrite(0, dutyCycle);  // 设置 PWM 占空比
    delay(15);                // 延时 15 毫秒
  }
  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    ledcWrite(0, dutyCycle);  // 设置 PWM 占空比
    delay(15);                // 延时 15 毫秒
  }
}