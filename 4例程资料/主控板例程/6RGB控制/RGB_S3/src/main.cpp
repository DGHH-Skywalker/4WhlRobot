#include "SF_RGB.h"

SF_RGB inBoard_RGB = SF_RGB();

void setup(){
  inBoard_RGB.begin(); 

  inBoard_RGB.setGlobalBrightness(100);//全局亮度100%

  inBoard_RGB.setColor(0,255,0,0);
  inBoard_RGB.show();
  delay(1000);

  inBoard_RGB.setColorBrightness(0,0,255,0,1);
  inBoard_RGB.show();
  delay(1000);  

  inBoard_RGB.setColor(0,0,0,255);
  inBoard_RGB.show();
  delay(1000); 

}

void loop(){
  inBoard_RGB.setColor(255,0,0,100);
  inBoard_RGB.show();
}

