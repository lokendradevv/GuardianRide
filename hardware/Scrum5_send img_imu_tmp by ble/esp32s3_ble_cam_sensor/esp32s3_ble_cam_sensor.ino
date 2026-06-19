// ============================================================
// ESP32-S3 BLE CAM + DHT11 + ICM-20948 合併版
// BLE Service:
//   Char 1 (NOTIFY) : 圖片分包資料
//   Char 2 (WRITE)  : 控制指令 START / STOP
//   Char 3 (NOTIFY) : 感測器資料 (溫度 + 濕度 + 加速度 + 陀螺儀)
// ============================================================

#include "esp_camera.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "DHT.h"
#include <Wire.h>

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

#define DHTPIN   3
#define DHTTYPE  DHT11
DHT dht(DHTPIN, DHTTYPE);

#define ICM_ADDR  0x68
#define SDA_PIN   38
#define SCL_PIN   39

#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_DATA_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_CTRL_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_SENSOR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26aa"

BLEServer*          pServer       = nullptr;
BLECharacteristic*  pDataChar     = nullptr;
BLECharacteristic*  pCtrlChar     = nullptr;
BLECharacteristic*  pSensorChar   = nullptr;

bool deviceConnected = false;
bool streaming       = false;

#define MTU_SIZE        512
#define FRAME_DELAY     200
#define SENSOR_INTERVAL 1

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("✅ 裝置已連線！");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    streaming = false;
    Serial.println("❌ 裝置已斷線，重新廣播...");
    pServer->startAdvertising();
  }
};

class MyCtrlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String val = pChar->getValue().c_str();
    if (val == "START") {
      streaming = true;
      Serial.println("▶️  開始串流");
    } else if (val == "STOP") {
      streaming = false;
      Serial.println("⏹  停止串流");
    }
  }
};

void icmWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t icmReadByte(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, 1);
  return Wire.read();
}

int16_t icmReadWord(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, 2);
  return (Wire.read() << 8) | Wire.read();
}

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ 相機初始化失敗: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);

  Serial.println("✅ 相機初始化成功 (QVGA 320x240)");
  return true;
}

void initICM() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Serial.println("初始化 ICM-20948...");
  icmWriteByte(0x06, 0x80);
  delay(100);
  icmWriteByte(0x06, 0x01);
  uint8_t whoami = icmReadByte(0x00);
  Serial.printf("ICM WHO_AM_I = 0x%02X (預期 0xEA)\n", whoami);
}

void initBLE() {
  BLEDevice::init("ESP32-S3-CAM");
  BLEDevice::setMTU(517);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pDataChar = pService->createCharacteristic(
    CHAR_DATA_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pDataChar->addDescriptor(new BLE2902());

  pCtrlChar = pService->createCharacteristic(
    CHAR_CTRL_UUID, BLECharacteristic::PROPERTY_WRITE);
  pCtrlChar->setCallbacks(new MyCtrlCallbacks());

  pSensorChar = pService->createCharacteristic(
    CHAR_SENSOR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pSensorChar->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("✅ BLE 廣播中，裝置名稱: ESP32-S3-CAM");
}

void sendFrame(uint8_t* data, size_t len, uint16_t frame_id) {
  const size_t DATA_PER_CHUNK = MTU_SIZE - 6;
  uint16_t total = (len + DATA_PER_CHUNK - 1) / DATA_PER_CHUNK;

  for (uint16_t i = 0; i < total; i++) {
    if (!deviceConnected) return;

    size_t offset    = i * DATA_PER_CHUNK;
    size_t chunk_len = min(DATA_PER_CHUNK, len - offset);

    uint8_t packet[MTU_SIZE];
    packet[0] = (frame_id >> 8) & 0xFF;
    packet[1] = frame_id & 0xFF;
    packet[2] = (i >> 8) & 0xFF;
    packet[3] = i & 0xFF;
    packet[4] = (total >> 8) & 0xFF;
    packet[5] = total & 0xFF;
    memcpy(packet + 6, data + offset, chunk_len);

    pDataChar->setValue(packet, chunk_len + 6);
    pDataChar->notify();
    delay(5);
  }
}

// ✅ 加上濕度
void sendSensorData() {
  float temp = dht.readTemperature();
  float humi = dht.readHumidity();      // ← 新增讀濕度

  if (isnan(temp) || isnan(humi)) {
    Serial.println("⚠️ DHT11 讀取失敗，跳過本次感測器傳送");
    return;
  }

  int16_t ax = icmReadWord(0x2D);
  int16_t ay = icmReadWord(0x2F);
  int16_t az = icmReadWord(0x31);
  int16_t gx = icmReadWord(0x33);
  int16_t gy = icmReadWord(0x35);
  int16_t gz = icmReadWord(0x37);

  float accX = ax / 16384.0f;
  float accY = ay / 16384.0f;
  float accZ = az / 16384.0f;
  float gyrX = gx / 131.0f;
  float gyrY = gy / 131.0f;
  float gyrZ = gz / 131.0f;

  char buffer[120];
  // ✅ 格式加上 H:濕度
  snprintf(buffer, sizeof(buffer),
    "T:%.1fC H:%.1f%% A:%.2f,%.2f,%.2f G:%.2f,%.2f,%.2f",
    temp, humi, accX, accY, accZ, gyrX, gyrY, gyrZ
  );

  pSensorChar->setValue((uint8_t*)buffer, strlen(buffer));
  pSensorChar->notify();
  Serial.println(buffer);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-S3 BLE CAM + Sensor ===");

  dht.begin();
  initICM();

  if (!initCamera()) {
    Serial.println("🔴 相機初始化失敗，停止執行");
    while (true) delay(1000);
  }

  initBLE();
  Serial.println("⏳ 等待裝置連線...");
}

uint16_t frame_id    = 0;
uint16_t sensor_tick = 0;

void loop() {
  if (!deviceConnected || !streaming) {
    delay(100);
    return;
  }

  unsigned long t0 = millis();

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ 拍照失敗");
    delay(100);
    return;
  }

  Serial.printf("📸 Frame %d | %d bytes | %d chunks\n",
    frame_id, fb->len,
    (fb->len + (MTU_SIZE - 6) - 1) / (MTU_SIZE - 6));

  sendFrame(fb->buf, fb->len, frame_id++);
  esp_camera_fb_return(fb);

  sensor_tick++;
  if (sensor_tick >= SENSOR_INTERVAL) {
    sensor_tick = 0;
    sendSensorData();
  }

  long elapsed = millis() - t0;
  if (elapsed < FRAME_DELAY) {
    delay(FRAME_DELAY - elapsed);
  }
}
