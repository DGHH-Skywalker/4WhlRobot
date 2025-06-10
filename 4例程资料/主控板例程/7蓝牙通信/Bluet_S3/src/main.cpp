#include <arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// 定义服务和特征的 UUID
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// 当设备连接或断开连接时调用
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);

  // 创建 BLE 设备
  BLEDevice::init("ESP32_BLE_Server");

  // 创建 BLE 服务器
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 创建 BLE 服务
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 创建 BLE 特征
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // 创建描述符
  pCharacteristic->addDescriptor(new BLE2902());

  // 启动服务
  pService->start();

  // 启动广播
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // 设置广播参数
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("等待客户端连接...");
}
char* d;
void loop() {
  // 如果设备已连接，发送数据
  if (deviceConnected) {
    pCharacteristic->setValue("Hello from ESP32");
    Serial.printf("%s\n",pCharacteristic->getValue());
    pCharacteristic->notify();
    delay(1000); // 每秒发送一次
    
    
  }


}
