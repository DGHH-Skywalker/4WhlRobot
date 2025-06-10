#include "SF_BLDC.h"
#include <Arduino.h>

SF_BLDC motors = SF_BLDC(Serial2);

void setup() {
  motors.init();
  motors.setModes(1,1);
}

void loop(){
  motors.setTargets(2,2);
}