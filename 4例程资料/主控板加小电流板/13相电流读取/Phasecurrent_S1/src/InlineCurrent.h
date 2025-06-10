#include <Arduino.h>
//电流环类
class CurrSense
{
  public:
    CurrSense(int Mot_Num);//创建对象时必须提供实参
    float readADCVoltageInline(const int pinA);//读取引脚adc值并转化为电压
    void configureADCInline(const int pinA,const int pinB, const int pinC);//初始化引脚配置
    void calibrateOffsets();//在零电流的时候采样1000个点计算偏移量
    void init();//配置引脚加计算偏移量
    void getPhaseCurrents();
    float current_a,current_b,current_c;
    int pinA;
    int pinB;
    int pinC;
    float offset_ia;
    float offset_ib;
    float offset_ic;
    float _shunt_resistor;
    float amp_gain;
    
    float volts_to_amps_ratio;
    
    float gain_a;
    float gain_b;
    float gain_c;
  private:
    int _Mot_Num;
};
