#include "MT6701_SSI.h"
#include <Arduino.h> 

#define CRC_CHECK

Sensor_MT6701_SSI::Sensor_MT6701_SSI(int Mot_Num){
    _Mot_Num=Mot_Num;  //使得 Mot_Num 可以统一在该文件调用
    CS_Pin = (_Mot_Num == 0) ? M0_CS : M1_CS;
    
}

void Sensor_MT6701_SSI::Sensor_init(SPIClass* _spi){
    mt_spi = _spi;
    pinMode(CS_Pin, OUTPUT);
    digitalWrite(CS_Pin,HIGH);
    mt_spi->begin();

    getSensorAngle(); // call once
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle(); // call again
    vel_angle_prev_ts = micros();
    delay(1);       
    getSensorAngle(); // call once
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); // call again
    angle_prev_ts = micros();
}

void Sensor_MT6701_SSI::Sensor_update() {
    float val = getSensorAngle();
    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // if overflow happened track it as full rotation
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

float Sensor_MT6701_SSI::getMechanicalAngle() {
    return angle_prev;
}

float Sensor_MT6701_SSI::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}

float Sensor_MT6701_SSI::getVelocity() {
    // calculate sample time
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0) Ts = 1e-3f;
    // velocity calculation
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;    
    // save variables for future pass
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}


double Sensor_MT6701_SSI::getSensorAngle(){
    mt_spi->beginTransaction(MT6701SSISettings);
    digitalWrite(CS_Pin, LOW);
    uint32_t value = mt_spi->transfer32(0x83ffffff);
    digitalWrite(CS_Pin, HIGH);
    mt_spi->endTransaction();
    value <<= MT6701_DATA_POS; // 需要去掉最高位数据 rawdata 32位长度
    value >>= 8; // 这时候才是mt6701发出的24位数据 
    #ifdef CRC_CHECK
        uint8_t s[3];
        s[0]=value>>18;
        s[1]=value>>12;
        s[2]=value>>6;
        if(CRC6(s, 3)==(value&0x3f))
            return (value>>10) / MT6701_CPR* _2PI;
        else
            return -1;
    #else
        return (value>>10) / MT6701_CPR* _2PI;
    #endif
    

}

uint8_t Sensor_MT6701_SSI::CRC6(uint8_t *data, uint8_t length)  
{  
	uint8_t i;  
	uint8_t crc = 0;    // Initial value  
	
	while(length--)  
	{  
		 crc ^= *data++; // crc ^= *data; data++;
		for (i=6; i>0; --i)  
        { 
            if (crc & 0x20)
                crc = (crc << 1) ^ 0x03;
            else
                crc = (crc << 1);
        }
    }  
	return crc&0x3f;  
} 


