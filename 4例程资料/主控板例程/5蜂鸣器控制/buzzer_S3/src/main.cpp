#include<arduino.h>

const int buzzerPin = 9;  // 蜂鸣器连接的GPIO引脚

void setup() {
  pinMode(buzzerPin, OUTPUT);  // 设置蜂鸣器引脚为输出模式
}

void loop() {
  // 播放简单的音乐
  tone(buzzerPin, 262);  // C4
  delay(500);
  noTone(buzzerPin);
  delay(100);

  tone(buzzerPin, 294);  // D4
  delay(500);
  noTone(buzzerPin);
  delay(100);

  tone(buzzerPin, 330);  // E4
  delay(500);
  noTone(buzzerPin);
  delay(100);

  tone(buzzerPin, 349);  // F4
  delay(500);
  noTone(buzzerPin);
  delay(100);

  tone(buzzerPin, 392);  // G4
  delay(500);
  noTone(buzzerPin);
  delay(100);

  tone(buzzerPin, 440);  // A4
  delay(500);
  noTone(buzzerPin);
  delay(100);

  tone(buzzerPin, 494);  // B4
  delay(500);
  noTone(buzzerPin);
  delay(100);

  tone(buzzerPin, 523);  // C5
  delay(500);
  noTone(buzzerPin);
  delay(100);
}

