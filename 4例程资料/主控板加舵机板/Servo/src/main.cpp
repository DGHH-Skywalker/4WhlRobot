#include <Arduino.h>
#include "SF_Servo.h"
#include "Wire.h"

SF_Servo servos = SF_Servo(Wire);

unsigned long currTime,prevTime;
void setup(){
  Serial.begin(115200);
  Wire.begin(1, 2, 400000UL);
  servos.init();
  prevTime = millis();
}

void loop() {
  servos.setPWM(0, 0, 100);
  servos.setPWM(1, 0, 100);
  servos.setPWM(2, 0, 100);
  servos.setPWM(3, 0, 100);
  servos.setPWM(4, 0, 100);
  servos.setPWM(5, 0, 100);
  servos.setPWM(6, 0, 100);
  servos.setPWM(7, 0, 100);
  servos.setPWM(8, 0, 100);
  servos.setPWM(9, 0, 100);
  servos.setPWM(10, 0, 100);
  servos.setPWM(11, 0, 100);
  delay(1000);
  servos.setPWM(0, 0, 200);
  servos.setPWM(1, 0, 200);
  servos.setPWM(2, 0, 200);
  servos.setPWM(3, 0, 200);
  servos.setPWM(4, 0, 200);
  servos.setPWM(5, 0, 200);
  servos.setPWM(6, 0, 200);
  servos.setPWM(7, 0, 200);
  servos.setPWM(8, 0, 200);
  servos.setPWM(9, 0, 100);
  servos.setPWM(9, 0, 200);
  servos.setPWM(10, 0, 100);
  servos.setPWM(11, 0, 200);
}