#ifndef __MT6701_SSI_H__
#define __MT6701_SSI_H__

#include "Arduino.h"
#include "SPI.h"
// #include "common/base_classes/Sensor.h"

#define _2PI 6.28318530718f
#define MT6701_CPR 16384.0f
#define MT6701_BITORDER MSBFIRST
#define MT6701_DATA_POS 1

#define MT6701_DO 19
#define MT6701_CLK 18
#define M0_CS 5
// #define M0_CS 22 // V4
#define M1_CS 14
// #define CRC_CHECK
 
static SPISettings MT6701SSISettings(1000000, MT6701_BITORDER, SPI_MODE2); // @suppress("Invalid arguments")


class Sensor_MT6701_SSI{
    public:
        Sensor_MT6701_SSI(int Mot_Num);
        void Sensor_init(SPIClass* _spi);
        void Sensor_update();
        float getAngle();
        float getVelocity();
        float getMechanicalAngle();
        double getSensorAngle();
        uint8_t CRC6(uint8_t *data, uint8_t length);
    private:
        int _Mot_Num;
        int CS_Pin;
        float angle_prev=0; // result of last call to getSensorAngle(), used for full rotations and velocity
        long angle_prev_ts=0; // timestamp of last call to getAngle, used for velocity
        float vel_angle_prev=0; // angle at last call to getVelocity, used for velocity
        long vel_angle_prev_ts=0; // last velocity calculation timestamp
        int32_t full_rotations=0; // full rotation tracking
        int32_t vel_full_rotations=0; // previous full rotation value for velocity calculation
        SPIClass* mt_spi = NULL;
};


#endif
