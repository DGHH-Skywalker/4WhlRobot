#include <SPI.h>
#include <arduino.h>

// 定义SSI引脚
#define CLK_PIN 23
#define DATA_PIN 19
#define CS_PIN 22

// 定义SSI时钟频率
#define SSI_CLOCK_FREQ 1000000 // 1 MHz
// 定义MT6701的位顺序和SPI模式
#define MT6701_BITORDER MSBFIRST
#define SPI_MODE2 2

// 初始化SPI
SPIClass spi(HSPI);
// 配置SPI设置
SPISettings MT6701SSISettings(SSI_CLOCK_FREQ, MT6701_BITORDER, SPI_MODE2);


uint32_t readAngle() {
  uint32_t angle = 0;

  // 选择从设备
  digitalWrite(CS_PIN, LOW);

  // 使用配置好的SPI设置
  spi.beginTransaction(MT6701SSISettings);

  // 读取角度数据
  angle = spi.transfer32(0x83ffffff);

  // 结束SPI事务
  spi.endTransaction();

  // 取消选择从设备
  digitalWrite(CS_PIN, HIGH);

  // 返回角度数据
  angle <<= 1; // 需要去掉最高位数据 rawdata 32位长度
  angle >>= 8; // 这时候才是mt6701发出的24位数据 
  return (angle>>10) / 16384.0f* 6.28318530718f;
    
}


void setup() {
  // 初始化串口
  Serial.begin(115200);

  // 初始化SPI
  spi.begin(CLK_PIN, DATA_PIN, 21, -1);

  // 设置SPI模式和时钟频率
  spi.setDataMode(SPI_MODE0);
  spi.setFrequency(SSI_CLOCK_FREQ);

  // 初始化MT6701
  digitalWrite(CS_PIN, HIGH); // 确保从设备未被选中
  pinMode(CS_PIN, OUTPUT);
}

void loop() {
  // 读取MT6701的角度数据
  float angle = readAngle();
  
  // 打印角度数据
  Serial.print("Angle: ");
  Serial.println(angle);

  // 延迟一段时间
  delay(1000);
}
