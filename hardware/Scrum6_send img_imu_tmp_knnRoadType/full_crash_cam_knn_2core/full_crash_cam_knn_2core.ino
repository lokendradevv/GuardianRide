// ============================================================
// ESP32-S3 BLE CAM + DHT11 + ICM-20948 + 摔車警報 + KNN路面偵測
// FreeRTOS 架構：
//   Core 0: BLE + 相機傳輸（原本 loop）
//   Core 1: IMU Task（100Hz）+ KNN Task（每500ms推論）
//   Mutex 保護 knn_buf 防止 race condition
// ============================================================

#include "esp_camera.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "DHT.h"
#include <Wire.h>
#include <math.h>
#include "knn_model.h"

// ─── 相機腳位 ────────────────────────────────────────────
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

// ─── 感測器 ──────────────────────────────────────────────
#define DHTPIN   3
#define DHTTYPE  DHT11
DHT dht(DHTPIN, DHTTYPE);
#define ICM_ADDR  0x68
#define SDA_PIN   38
#define SCL_PIN   39

// ─── 蜂鳴器 & 按鍵 ───────────────────────────────────────
#define BUZZER_PIN     2
#define BUTTON_PIN     14

// ─── 救護車音效 ───────────────────────────────────────────
#define SIREN_HIGH_HZ  960
#define SIREN_LOW_HZ   770
#define SIREN_PHASE_MS 500

// ─── 摔車偵測 ─────────────────────────────────────────────
#define IMPACT_THRESHOLD    4.5f
#define FREE_FALL_THRESHOLD 0.6f
#define IMPACT_WINDOW_MS    500

// ─── 按鍵 ────────────────────────────────────────────────
#define DOUBLE_CLICK_MS  400
#define DEBOUNCE_MS       30

// ─── BLE ─────────────────────────────────────────────────
#define SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_DATA_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_CTRL_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_SENSOR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define MTU_SIZE        512
#define FRAME_DELAY     200
#define SENSOR_INTERVAL 1

// ─── FreeRTOS ─────────────────────────────────────────────
#define IMU_TASK_HZ     100   // IMU 採樣頻率
#define KNN_TASK_MS     500   // KNN 推論間隔（ms）
SemaphoreHandle_t buf_mutex  = NULL;  // knn_buf 的門鎖
SemaphoreHandle_t i2c_mutex  = NULL;  // I2C 的門鎖（DHT/ICM 共用）

BLEServer*         pServer     = nullptr;
BLECharacteristic* pDataChar   = nullptr;
BLECharacteristic* pCtrlChar   = nullptr;
BLECharacteristic* pSensorChar = nullptr;
bool deviceConnected = false;
bool streaming       = false;
bool icmOK           = false;

// ─── DHT11 快取 ───────────────────────────────────────────
float lastTemp = 0, lastHumi = 0;
bool  dhtReady = false;
unsigned long lastDHTTime = 0;

// ─── 摔車狀態機 ──────────────────────────────────────────
enum CrashState { NORMAL, IMPACT_DETECTED, ALARM_ACTIVE };
CrashState    crashState = NORMAL;
unsigned long impactTime = 0;
unsigned long alarmStart = 0;
bool          inHighPhase = true;
unsigned long phaseStart  = 0;

// ─── 靜音模式 ────────────────────────────────────────────
bool quietMode = false;

// ─── 按鍵狀態 ────────────────────────────────────────────
int           btnLastState    = HIGH;
unsigned long btnLastDebounce = 0;
int           btnClickCount   = 0;
unsigned long btnFirstClick   = 0;

// ─── KNN buffer（受 buf_mutex 保護）─────────────────────
float knn_buf_z[KNN_WINDOW];
float knn_buf_x[KNN_WINDOW];
float knn_buf_y[KNN_WINDOW];
int   knn_buf_idx  = 0;
bool  knn_buf_full = false;

// ─── 最新 IMU 數值（供 loop 讀取）───────────────────────
volatile float imu_accX = 0, imu_accY = 0, imu_accZ = 0;
volatile float imu_gyrX = 0, imu_gyrY = 0, imu_gyrZ = 0;
volatile float imu_mag  = 1.0f;

// ─── KNN 結果（全域）────────────────────────────────────
char  knn_result_str[32]     = "warming_up";
int   knn_votes[KNN_CLASSES] = {0};
float knn_dists[KNN_N];

// ─── KNN push（不加鎖，由呼叫者負責）───────────────────
// 座標轉換：新Z=原ax（重力軸），新X=原az，新Y=原ay
void knn_push(float az, float ax, float ay) {
  knn_buf_z[knn_buf_idx] = ax * 9.81f;  // 新Z = 原ax
  knn_buf_x[knn_buf_idx] = az * 9.81f;  // 新X = 原az
  knn_buf_y[knn_buf_idx] = ay * 9.81f;  // 新Y = 原ay
  knn_buf_idx++;
  if (knn_buf_idx >= KNN_WINDOW) { knn_buf_idx = 0; knn_buf_full = true; }
}

void knn_extract_features(float* z, float* x, float* y, float* feat) {
  float* sigs[3] = {z, x, y};
  for (int s = 0; s < 3; s++) {
    float mn=0, mx=sigs[s][0], mi=sigs[s][0], sd=0;
    for (int i=0;i<KNN_WINDOW;i++) mn += sigs[s][i];
    mn /= KNN_WINDOW;
    for (int i=0;i<KNN_WINDOW;i++) {
      if (sigs[s][i]>mx) mx=sigs[s][i];
      if (sigs[s][i]<mi) mi=sigs[s][i];
      sd += (sigs[s][i]-mn)*(sigs[s][i]-mn);
    }
    sd = sqrtf(sd/KNN_WINDOW);
    float tmp[KNN_WINDOW];
    memcpy(tmp, sigs[s], KNN_WINDOW*sizeof(float));
    for (int i=0;i<KNN_WINDOW-1;i++)
      for (int j=i+1;j<KNN_WINDOW;j++)
        if (tmp[j]<tmp[i]) { float t=tmp[i]; tmp[i]=tmp[j]; tmp[j]=t; }
    feat[s*4+0] = mn;
    feat[s*4+1] = sd;
    feat[s*4+2] = mx - mi;
    feat[s*4+3] = tmp[KNN_WINDOW*3/4] - tmp[KNN_WINDOW/4];
  }
}

void knn_run_unsafe() {
  if (!knn_buf_full) {
    strncpy(knn_result_str, "warming_up", sizeof(knn_result_str));
    memset(knn_votes, 0, sizeof(knn_votes));
    return;
  }
  float feat[KNN_DIM];
  knn_extract_features(knn_buf_z, knn_buf_x, knn_buf_y, feat);
  float feat_s[KNN_DIM];
  for (int i=0;i<KNN_DIM;i++)
    feat_s[i] = (feat[i] - KNN_SCALER_MEAN[i]) / KNN_SCALER_STD[i];
  for (int i=0;i<KNN_N;i++) {
    float d=0;
    for (int j=0;j<KNN_DIM;j++) {
      float diff = feat_s[j] - KNN_TRAIN_X[i][j];
      d += diff*diff;
    }
    knn_dists[i] = d;
  }
  memset(knn_votes, 0, sizeof(knn_votes));
  for (int k=0;k<KNN_K;k++) {
    int best=-1; float best_d=1e30f;
    for (int i=0;i<KNN_N;i++)
      if (knn_dists[i]<best_d) { best_d=knn_dists[i]; best=i; }
    if (best>=0) { knn_votes[KNN_TRAIN_Y[best]]++; knn_dists[best]=1e30f; }
  }
  int winner=0;
  for (int i=1;i<KNN_CLASSES;i++)
    if (knn_votes[i]>knn_votes[winner]) winner=i;
  strncpy(knn_result_str, KNN_CLASS_NAMES[winner], sizeof(knn_result_str));
}

// ─── IMU Task（Core 1，100Hz）────────────────────────────
void imuTask(void* param) {
  const TickType_t period = pdMS_TO_TICKS(1000 / IMU_TASK_HZ);
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    if (icmOK) {
      // 讀 IMU（加 i2c_mutex 防止跟 DHT 衝突）
      float ax, ay, az, gx, gy, gz;
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        ax = icmReadWord(0x2D) / 16384.0f;
        ay = icmReadWord(0x2F) / 16384.0f;
        az = icmReadWord(0x31) / 16384.0f;
        gx = icmReadWord(0x33) / 131.0f;
        gy = icmReadWord(0x35) / 131.0f;
        gz = icmReadWord(0x37) / 131.0f;
        xSemaphoreGive(i2c_mutex);
      } else {
        vTaskDelayUntil(&lastWake, period);
        continue;
      }

      float mag = sqrtf(ax*ax + ay*ay + az*az);

      // 更新全域 IMU 數值（volatile，loop 直接讀）
      imu_accX = ax; imu_accY = ay; imu_accZ = az;
      imu_gyrX = gx; imu_gyrY = gy; imu_gyrZ = gz;
      imu_mag  = mag;

      // 摔車偵測（不需要 mutex，只讀 crashState）
      updateCrashDetection(mag);

      // push 進 KNN buffer（加 buf_mutex）
      if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        knn_push(az, ax, ay);
        xSemaphoreGive(buf_mutex);
      }
    }
    vTaskDelayUntil(&lastWake, period);
  }
}

// ─── KNN Task（Core 1，每 500ms 推論）───────────────────
void knnTask(void* param) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(KNN_TASK_MS));

    // 複製 buffer 後再推論，縮短鎖的時間
    float local_z[KNN_WINDOW], local_x[KNN_WINDOW], local_y[KNN_WINDOW];
    bool  local_full = false;

    if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(local_z, knn_buf_z, sizeof(knn_buf_z));
      memcpy(local_x, knn_buf_x, sizeof(knn_buf_x));
      memcpy(local_y, knn_buf_y, sizeof(knn_buf_y));
      local_full = knn_buf_full;
      xSemaphoreGive(buf_mutex);
    } else continue;

    if (!local_full) {
      strncpy(knn_result_str, "warming_up", sizeof(knn_result_str));
      continue;
    }

    // 推論（不需要鎖，用本地複製的資料）
    float feat[KNN_DIM];
    knn_extract_features(local_z, local_x, local_y, feat);
    float feat_s[KNN_DIM];
    for (int i=0;i<KNN_DIM;i++)
      feat_s[i] = (feat[i] - KNN_SCALER_MEAN[i]) / KNN_SCALER_STD[i];
    for (int i=0;i<KNN_N;i++) {
      float d=0;
      for (int j=0;j<KNN_DIM;j++) {
        float diff = feat_s[j] - KNN_TRAIN_X[i][j];
        d += diff*diff;
      }
      knn_dists[i] = d;
    }
    memset(knn_votes, 0, sizeof(knn_votes));
    for (int k=0;k<KNN_K;k++) {
      int best=-1; float best_d=1e30f;
      for (int i=0;i<KNN_N;i++)
        if (knn_dists[i]<best_d) { best_d=knn_dists[i]; best=i; }
      if (best>=0) { knn_votes[KNN_TRAIN_Y[best]]++; knn_dists[best]=1e30f; }
    }
    int winner=0;
    for (int i=1;i<KNN_CLASSES;i++)
      if (knn_votes[i]>knn_votes[winner]) winner=i;
    strncpy(knn_result_str, KNN_CLASS_NAMES[winner], sizeof(knn_result_str));
  }
}

// ─── 蜂鳴器 ──────────────────────────────────────────────
void buzzerTone(uint32_t freq) { ledcWriteTone(BUZZER_PIN, freq); }
void buzzerOff() { ledcWriteTone(BUZZER_PIN, 0); }
void beep(uint32_t freq, uint32_t ms) { buzzerTone(freq); delay(ms); buzzerOff(); }

// ─── BLE Callbacks ───────────────────────────────────────
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { deviceConnected=true; Serial.println("✅ 連線！"); }
  void onDisconnect(BLEServer* s) override {
    deviceConnected=false; streaming=false;
    Serial.println("❌ 斷線，重新廣播..."); s->startAdvertising();
  }
};
class MyCtrlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String val=pChar->getValue().c_str();
    if      (val=="START")       { streaming=true;  Serial.println("▶️  開始串流"); }
    else if (val=="STOP")        { streaming=false; Serial.println("⏹  停止串流"); }
    else if (val=="RESET_ALARM") { crashState=NORMAL; buzzerOff(); Serial.println("🔕 遠端重置"); }
  }
};

// ─── ICM-20948 ───────────────────────────────────────────
void icmWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.write(data); Wire.endTransmission();
}
uint8_t icmReadByte(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR); Wire.write(reg);
  Wire.endTransmission(false); Wire.requestFrom(ICM_ADDR,1);
  return Wire.available()?Wire.read():0xFF;
}
int16_t icmReadWord(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR); Wire.write(reg);
  Wire.endTransmission(false); Wire.requestFrom(ICM_ADDR,2);
  return (Wire.available()>=2)?(Wire.read()<<8)|Wire.read():0;
}
bool initICM() {
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000); delay(50);
  Serial.println("--- I2C 掃描 ---");
  bool found=false;
  for (uint8_t addr=1;addr<127;addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission()==0) { Serial.printf("  找到：0x%02X\n",addr); found=true; }
  }
  if (!found) { Serial.println("  ❌ 沒有 I2C！"); return false; }
  uint8_t whoami=icmReadByte(0x00);
  Serial.printf("ICM WHO_AM_I=0x%02X",whoami);
  if (whoami!=0xEA) { Serial.println(" ❌"); return false; }
  Serial.println(" ✅");
  icmWriteByte(0x06,0x80); delay(100);
  icmWriteByte(0x06,0x01); delay(50);
  icmWriteByte(0x14, 0x06);  // ACCEL_CONFIG，0x06 = ±16g
  Serial.println("✅ ICM-20948 OK");
  return true;
}

// ─── 相機初始化 ───────────────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel=LEDC_CHANNEL_0; config.ledc_timer=LEDC_TIMER_0;
  config.pin_d0=Y2_GPIO_NUM; config.pin_d1=Y3_GPIO_NUM;
  config.pin_d2=Y4_GPIO_NUM; config.pin_d3=Y5_GPIO_NUM;
  config.pin_d4=Y6_GPIO_NUM; config.pin_d5=Y7_GPIO_NUM;
  config.pin_d6=Y8_GPIO_NUM; config.pin_d7=Y9_GPIO_NUM;
  config.pin_xclk=XCLK_GPIO_NUM; config.pin_pclk=PCLK_GPIO_NUM;
  config.pin_vsync=VSYNC_GPIO_NUM; config.pin_href=HREF_GPIO_NUM;
  config.pin_sccb_sda=SIOD_GPIO_NUM; config.pin_sccb_scl=SIOC_GPIO_NUM;
  config.pin_pwdn=PWDN_GPIO_NUM; config.pin_reset=RESET_GPIO_NUM;
  config.xclk_freq_hz=20000000; config.pixel_format=PIXFORMAT_JPEG;
  config.grab_mode=CAMERA_GRAB_WHEN_EMPTY; config.fb_location=CAMERA_FB_IN_PSRAM;
  config.frame_size=FRAMESIZE_QVGA; config.jpeg_quality=15; config.fb_count=2;
  esp_err_t err=esp_camera_init(&config);
  if (err!=ESP_OK) { Serial.printf("❌ 相機失敗:0x%x\n",err); return false; }
  sensor_t* s=esp_camera_sensor_get();
  s->set_vflip(s,1); s->set_hmirror(s,1);
  s->set_whitebal(s,1); s->set_awb_gain(s,1); s->set_exposure_ctrl(s,1);
  Serial.println("✅ 相機 OK (QVGA)");
  return true;
}

// ─── BLE 初始化 ───────────────────────────────────────────
void initBLE() {
  BLEDevice::init("ESP32-S3-CAM"); BLEDevice::setMTU(517);
  pServer=BLEDevice::createServer(); pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService=pServer->createService(SERVICE_UUID);
  pDataChar=pService->createCharacteristic(CHAR_DATA_UUID,BLECharacteristic::PROPERTY_NOTIFY);
  pDataChar->addDescriptor(new BLE2902());
  pCtrlChar=pService->createCharacteristic(CHAR_CTRL_UUID,BLECharacteristic::PROPERTY_WRITE);
  pCtrlChar->setCallbacks(new MyCtrlCallbacks());
  pSensorChar=pService->createCharacteristic(CHAR_SENSOR_UUID,BLECharacteristic::PROPERTY_NOTIFY);
  pSensorChar->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising* pAdv=BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID); pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("✅ BLE 廣播中");
}

// ─── 圖片分包傳送 ─────────────────────────────────────────
void sendFrame(uint8_t* data, size_t len, uint16_t frame_id) {
  const size_t DPC=MTU_SIZE-6;
  uint16_t total=(len+DPC-1)/DPC;
  for (uint16_t i=0;i<total;i++) {
    if (!deviceConnected) return;
    size_t offset=i*DPC, chunk_len=min(DPC,len-offset);
    uint8_t packet[MTU_SIZE];
    packet[0]=(frame_id>>8)&0xFF; packet[1]=frame_id&0xFF;
    packet[2]=(i>>8)&0xFF;        packet[3]=i&0xFF;
    packet[4]=(total>>8)&0xFF;    packet[5]=total&0xFF;
    memcpy(packet+6,data+offset,chunk_len);
    pDataChar->setValue(packet,chunk_len+6); pDataChar->notify(); delay(5);
  }
}

// ─── 感測器資料傳送 ───────────────────────────────────────
void sendSensorData() {
  // 座標轉換後的值（直覺顯示）
  float accZ = imu_accX;   // 新Z（垂直地面）= 原ax
  float accX = imu_accZ;   // 新X = 原az
  float accY = imu_accY;   // 新Y = 原ay
  float gyrX = imu_gyrX;
  float gyrY = imu_gyrY;
  float gyrZ = imu_gyrZ;
  float mag  = imu_mag;

  const char* stateStr=(crashState==ALARM_ACTIVE)  ?"CRASH":
                       (crashState==IMPACT_DETECTED)?"IMPACT":"OK";
  char proba_str[64]="";
  for (int i=0;i<KNN_CLASSES;i++) {
    int pct = knn_buf_full ? (knn_votes[i]*100/KNN_K) : 0;
    char tmp[8]; snprintf(tmp,sizeof(tmp),"%d",pct);
    strcat(proba_str,tmp);
    if (i<KNN_CLASSES-1) strcat(proba_str,",");
  }
  char buffer[256];
  if (dhtReady && icmOK)
    snprintf(buffer,sizeof(buffer),
      "T:%.1fC H:%.1f%% A:%.2f,%.2f,%.2f G:%.2f,%.2f,%.2f M:%.2fg S:%s Q:%d R:%s P:%s",
      lastTemp,lastHumi,accX,accY,accZ,gyrX,gyrY,gyrZ,mag,
      stateStr,quietMode?1:0,knn_result_str,proba_str);
  else if (!dhtReady && icmOK)
    snprintf(buffer,sizeof(buffer),
      "T:ERR H:ERR A:%.2f,%.2f,%.2f G:%.2f,%.2f,%.2f M:%.2fg S:%s Q:%d R:%s P:%s",
      accX,accY,accZ,gyrX,gyrY,gyrZ,mag,
      stateStr,quietMode?1:0,knn_result_str,proba_str);
  else
    snprintf(buffer,sizeof(buffer),"T:ERR H:ERR A:ERR G:ERR S:%s Q:%d R:%s P:%s",
             stateStr,quietMode?1:0,knn_result_str,proba_str);
  Serial.println(buffer);
  if (deviceConnected) {
    pSensorChar->setValue((uint8_t*)buffer,strlen(buffer));
    pSensorChar->notify();
  }
}

// ─── 按鍵 ────────────────────────────────────────────────
void onSingleClick() {
  if (crashState==ALARM_ACTIVE) { Serial.println("💡 雙擊解除警報"); return; }
  quietMode=!quietMode;
  if (quietMode) { buzzerOff(); beep(900,150); delay(70); beep(500,150); Serial.println("🔇 靜音ON"); }
  else           { beep(1800,200); Serial.println("🔊 靜音OFF"); }
  if (deviceConnected) {
    const char* msg=quietMode?"QUIET_ON":"QUIET_OFF";
    pSensorChar->setValue((uint8_t*)msg,strlen(msg)); pSensorChar->notify();
  }
}
void onDoubleClick() {
  crashState=NORMAL; quietMode=false; buzzerOff();
  beep(1500,80); delay(40); beep(1500,80); delay(40); beep(1500,80);
  Serial.println("✅ 我沒事！");
  if (deviceConnected) {
    const char* msg="SAFE";
    pSensorChar->setValue((uint8_t*)msg,strlen(msg)); pSensorChar->notify();
  }
}
void updateButton() {
  int reading=digitalRead(BUTTON_PIN); unsigned long now=millis();
  if (reading==btnLastState) { btnLastDebounce=now; return; }
  if ((now-btnLastDebounce)<DEBOUNCE_MS) return;
  btnLastDebounce=now; btnLastState=reading;
  if (reading==LOW) {
    btnClickCount++;
    if (btnClickCount==1) btnFirstClick=now;
    else if (btnClickCount>=2) { onDoubleClick(); btnClickCount=0; btnFirstClick=0; return; }
  }
  if (btnClickCount==1&&(now-btnFirstClick)>DOUBLE_CLICK_MS) {
    onSingleClick(); btnClickCount=0; btnFirstClick=0;
  }
}
void checkButtonTimeout() {
  if (btnClickCount==1&&(millis()-btnFirstClick)>DOUBLE_CLICK_MS) {
    onSingleClick(); btnClickCount=0; btnFirstClick=0;
  }
}

// ─── 救護車音效（非阻塞）─────────────────────────────────
void updateBuzzer() {
  if (crashState!=ALARM_ACTIVE||quietMode) { buzzerOff(); return; }
  unsigned long now=millis();
  if (now-phaseStart>=SIREN_PHASE_MS) {
    inHighPhase=!inHighPhase; phaseStart=now;
    buzzerTone(inHighPhase?SIREN_HIGH_HZ:SIREN_LOW_HZ);
  }
}

// ─── 摔車偵測（由 IMU Task 呼叫）────────────────────────
void updateCrashDetection(float mag) {
  unsigned long now=millis();
  if (crashState==ALARM_ACTIVE) return;
  switch(crashState) {
    case NORMAL:
      if (mag>IMPACT_THRESHOLD) { crashState=IMPACT_DETECTED; impactTime=now;
        Serial.printf("⚡ 撞擊！%.2fg\n",mag); }
      break;
    case IMPACT_DETECTED:
      if (now-impactTime>IMPACT_WINDOW_MS) { crashState=NORMAL; Serial.println("↩️  取消"); }
      else if (mag<FREE_FALL_THRESHOLD) {
        crashState=ALARM_ACTIVE; alarmStart=now; phaseStart=now; inHighPhase=true;
        if (!quietMode) buzzerTone(SIREN_HIGH_HZ);
        Serial.printf("🚨 摔車！%.2fg%s\n",mag,quietMode?"（靜音）":"");
        if (deviceConnected) {
          const char* msg="CRASH_DETECTED";
          pSensorChar->setValue((uint8_t*)msg,strlen(msg)); pSensorChar->notify();
        }
      }
      break;
    default: break;
  }
}

// ─── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-S3 BLE CAM + 摔車 + KNN (FreeRTOS) ===");

  // 建立 Mutex
  buf_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();

  ledcAttach(BUZZER_PIN,2000,8);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  beep(800,120); delay(60); beep(1200,120); delay(60); beep(1800,120);

  dht.begin();
  icmOK=initICM();
  if (!initCamera()) { Serial.println("🔴 相機失敗"); while(true) delay(1000); }
  initBLE();

  // 啟動 IMU Task（Core 1，優先度 2）
  xTaskCreatePinnedToCore(imuTask, "IMU", 4096, NULL, 2, NULL, 1);
  // 啟動 KNN Task（Core 1，優先度 1）
  xTaskCreatePinnedToCore(knnTask, "KNN", 8192, NULL, 1, NULL, 1);

  Serial.println("─────────────────────────────────");
  Serial.println("按鍵：1下=靜音 / 2下=我沒事");
  Serial.printf("IMU採樣: %dHz | KNN推論: 每%dms\n", IMU_TASK_HZ, KNN_TASK_MS);
  Serial.printf("KNN窗口:%d樣本 = %.1f秒 @%dHz\n",
                KNN_WINDOW, (float)KNN_WINDOW/IMU_TASK_HZ, IMU_TASK_HZ);
  Serial.printf("KNN訓練點:%d  類別:%d種\n", KNN_N, KNN_CLASSES);
  for (int i=0;i<KNN_CLASSES;i++) Serial.printf("  P[%d]=%s\n",i,KNN_CLASS_NAMES[i]);
  Serial.println("─────────────────────────────────");
  Serial.println("⏳ 等待連線...");
}

// ─── Loop（Core 0：BLE + 相機）──────────────────────────
uint16_t frame_id=0, sensor_tick=0;

void loop() {
  unsigned long now=millis();
  updateButton(); checkButtonTimeout();
  updateBuzzer();

  // DHT11（加 i2c_mutex）
  if (now-lastDHTTime>=2000) {
    lastDHTTime=now;
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50))==pdTRUE) {
      float t=dht.readTemperature(), h=dht.readHumidity();
      xSemaphoreGive(i2c_mutex);
      if (!isnan(t)&&!isnan(h)) { lastTemp=t; lastHumi=h; dhtReady=true; }
      else Serial.println("⚠️ DHT11 失敗");
    }
  }

  if (deviceConnected&&streaming) {
    unsigned long t0=millis();
    camera_fb_t* fb=esp_camera_fb_get();
    if (fb) {
      Serial.printf("📸 Frame %d|%d bytes\n",frame_id,fb->len);
      sendFrame(fb->buf,fb->len,frame_id++);
      esp_camera_fb_return(fb);
    } else Serial.println("❌ 拍照失敗");
    sensor_tick++;
    if (sensor_tick>=SENSOR_INTERVAL) { sensor_tick=0; sendSensorData(); }
    long elapsed=millis()-t0;
    if (elapsed<FRAME_DELAY) delay(FRAME_DELAY-elapsed);
  } else {
    sendSensorData();
    delay(200);
  }
}
