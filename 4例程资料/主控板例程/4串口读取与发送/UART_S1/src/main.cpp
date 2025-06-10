#include<arduino.h>

void setup() {
  Serial.begin(115200);  // 初始化默认串口（UART0）
  Serial1.begin(115200, SERIAL_8N1, 16, 17);  // 初始化UART1，波特率为115200，8位数据位，无校验位，1位停止位，RX引脚为GPIO16，TX引脚为GPIO17
}

void loop() {
  if (Serial.available() > 0) {  // 检查默认串口是否有数据可读
    char receivedChar = Serial.read();  // 读取一个字符
    Serial.print("Received on UART0: ");
    Serial.println(receivedChar);  // 打印接收到的字符

    // 将接收到的字符发送到UART1
    Serial1.print("Sending to UART1: ");
    Serial1.println(receivedChar);
  }

  if (Serial1.available() > 0) {  // 检查UART1是否有数据可读
    char receivedChar = Serial1.read();  // 读取一个字符
    Serial.print("Received on UART1: ");
    Serial.println(receivedChar);  // 打印接收到的字符

    // 将接收到的字符发送到默认串口
    Serial.print("Sending to UART0: ");
    Serial.println(receivedChar);
  }
}
