#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "DHT.h"
#include <Wire.h>

// ===== DHT11 =====
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ===== ICM20948 =====
#define ICM_ADDR 0x68
#define SDA_PIN 21
#define SCL_PIN 22

// ===== BLE =====
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-1234-abcdefabcdef"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// ===== BLE Callback =====
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("手機已連線");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("手機已斷線");
  }
};

// ===== I2C Functions =====
void writeByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByte(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, 1);
  return Wire.read();
}

int16_t readWord(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, 2);
  return Wire.read() << 8 | Wire.read();
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  // DHT
  dht.begin();

  // ICM
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("初始化 ICM-20948...");
  writeByte(0x06, 0x80);
  delay(100);
  writeByte(0x06, 0x01);

  uint8_t whoami = readByte(0x00);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoami, HEX);

  // BLE
  BLEDevice::init("ESP32_AllSensor");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("等待手機連線...");
}

// ===== LOOP =====
void loop() {

  if (deviceConnected) {

    // ===== 讀 DHT =====
    float temp = dht.readTemperature();

    if (isnan(temp)) {
      Serial.println("DHT11 讀取失敗");
      return;
    }

    // ===== 讀 ICM =====
    int16_t ax = readWord(0x2D);
    int16_t ay = readWord(0x2F);
    int16_t az = readWord(0x31);

    int16_t gx = readWord(0x33);
    int16_t gy = readWord(0x35);
    int16_t gz = readWord(0x37);

    float accX = ax / 16384.0;
    float accY = ay / 16384.0;
    float accZ = az / 16384.0;

    float gyrX = gx / 131.0;
    float gyrY = gy / 131.0;
    float gyrZ = gz / 131.0;

    // ===== 組合資料 =====
    char buffer[100];
    sprintf(buffer,
      "T:%.1fC A:%.2f,%.2f,%.2f G:%.2f,%.2f,%.2f",
      temp, accX, accY, accZ, gyrX, gyrY, gyrZ
    );

    // ===== BLE 傳送 =====
    pCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    pCharacteristic->notify();

    // ===== Serial 顯示 =====
    Serial.println(buffer);
  }

  delay(2000);
}
