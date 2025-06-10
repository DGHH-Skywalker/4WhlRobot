#include "SF_RGB.h"

//初始化设置
SF_RGB::SF_RGB()
{
	
} 
//RMT通道设置
bool SF_RGB::begin(char Pin, unsigned int Counts)
{	
	pin = Pin;
	ledCounts = Counts;
	//memory default 64， Optional:64,128,192,256,320,384,448,512
	rmt_mem = RMT_MEM_64; 
	globalBr = 0.5;
	setLedType(TYPE_GRB);

	if ((rmt_send = rmtInit(pin, true, rmt_mem)) == NULL){
		return false;
	}

	for(int i=0;i<ledCounts;i++)
	{
		for (int bit = 0; bit < 24; bit++) {
			led_data[i*24+bit].level0 = 1;
			led_data[i*24+bit].duration0 = 4;
			led_data[i*24+bit].level1 = 0;
			led_data[i*24+bit].duration1 = 8;
		}
	}
	realTick = rmtSetTick(rmt_send, 100);
	return true;
}
//设置RGB类型
void SF_RGB::setLedType(LED_TYPE t)
{
	rOffset = (t >> 4) & 0x03;
	gOffset = (t >> 2) & 0x03;
	bOffset = t & 0x03;
}


void SF_RGB::setColor(int index, unsigned long rgb)
{
	setColor(index, rgb >> 16, rgb >> 8, rgb);
}
//设定红绿蓝三通道的颜色值
void SF_RGB::setColor(int index, char red, char green, char blue)
{
	char p[3];
	p[rOffset] = constrain(red, 0, 255) * globalBr;
	p[gOffset] = constrain(green, 0, 255) * globalBr;
	p[bOffset] = constrain(blue, 0, 255) * globalBr;
	setPixel(index, p[0], p[1], p[2]);
}

void SF_RGB::setColorBrightness(int index, char red, char green, char blue, unsigned int brightness)
{
	char p[3];
	float br;
	br = constrain(brightness, 0, 100); 
	br = br/100;
	p[rOffset] = constrain(red, 0, 255) * br;
	p[gOffset] = constrain(green, 0, 255) * br;
	p[bOffset] = constrain(blue, 0, 255) * br;
	setPixel(index, p[0], p[1], p[2]);
}

//设置RGB亮度
void SF_RGB::setGlobalBrightness(unsigned int brightness)
{
	globalBr = constrain(brightness, 0, 100) / 100;
}


//遍历设置像素颜色
void SF_RGB::setPixel(int index, char r, char g, char b)
{
	unsigned long color = r << 16 | g << 8 | b ;
	for (int bit = 0; bit < 24; bit++) {
		if (color & (1 << (23-bit))) {
			led_data[index*24+bit].level0 = 1;
			led_data[index*24+bit].duration0 = 8;
			led_data[index*24+bit].level1 = 0;
			led_data[index*24+bit].duration1 = 4;
		} else {
			led_data[index*24+bit].level0 = 1;
			led_data[index*24+bit].duration0 = 4;
			led_data[index*24+bit].level1 = 0;
			led_data[index*24+bit].duration1 = 8;
		}
	}
}
//设置好的led数据写入
void SF_RGB::show()
{
	rmtWrite(rmt_send, led_data, ledCounts*24);
}

uint32_t SF_RGB::Wheel(byte pos)
{
	unsigned long WheelPos = pos % 0xff;
	if (WheelPos < 85) {
		return ((255 - WheelPos * 3) << 16) | ((WheelPos * 3) << 8);
	}
	if (WheelPos < 170) {
		WheelPos -= 85;
		return (((255 - WheelPos * 3) << 8) | (WheelPos * 3));
	}
	WheelPos -= 170;
	return ((WheelPos * 3) << 16 | (255 - WheelPos * 3));
}

