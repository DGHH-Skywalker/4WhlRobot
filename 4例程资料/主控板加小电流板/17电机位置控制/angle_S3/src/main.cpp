#include "SF_BLDC.h"
#include <Arduino.h>

SF_BLDC motors = SF_BLDC(Serial2);

void setup() {
  motors.init();
  motors.setModes(2,2);
}

void loop(){
  motors.setTargets(20,20);
}