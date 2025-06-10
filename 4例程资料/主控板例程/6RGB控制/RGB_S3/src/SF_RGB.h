#ifndef _SF_RGB_h
#define _SF_RGB_h

#include "Arduino.h"
#include "esp32-hal.h"



#define RGB_PIN  4
#define NR_OF_LEDS   256  
#define NR_OF_ALL_BITS 24*NR_OF_LEDS

enum LED_TYPE
{					  //R  G  B
	TYPE_RGB = 0x06,  //00 01 10
	TYPE_RBG = 0x09,  //00 10 01
	TYPE_GRB = 0x12,  //01 00 10
	TYPE_GBR = 0x21,  //10 00 01
	TYPE_BRG = 0x18,  //01 10 00
	TYPE_BGR = 0x24	  //10 01 00
};

class SF_RGB
{

protected:
	unsigned int ledCounts;
	char pin;
	float globalBr;
	
	char rOffset;
	char gOffset;
	char bOffset;
	
	float realTick;
	rmt_reserve_memsize_t rmt_mem;
	rmt_data_t led_data[NR_OF_ALL_BITS];
	rmt_obj_t* rmt_send = NULL;

public:
	SF_RGB();
	bool begin(char Pin=RGB_PIN, unsigned int Counts=1);
	void setLedType(LED_TYPE t);
	void setGlobalBrightness(unsigned int brightness);
	void setPixel(int index, char r, char g, char b);
	void setColor(int index, unsigned long rgb);
	void setColor(int index, char red, char green, char blue);
	void setColorBrightness(int index, char red, char green, char blue, unsigned int brightness);
	void show();
	uint32_t Wheel(byte pos);

};

#endif

