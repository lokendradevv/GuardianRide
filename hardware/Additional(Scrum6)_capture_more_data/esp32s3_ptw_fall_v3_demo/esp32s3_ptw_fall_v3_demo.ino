// ============================================================
// ESP32-S3 BLE CAM + DHT11 + ICM-20948 + PTW摔車警報
// 摔車演算法：Boubezoul et al. (2013)
//   "A simple fall detection algorithm for powered two wheelers"
//   Control Engineering Practice, 21(3), pp.286-297
//
// 新增：KNN 觸發回放功能
//   偵測到不規則路面（pothole/asphalt_bumps/worn_out_road）時
//   傳送最近 10 幀（circular buffer）供事後驗證 false alarm
//   BLE 通知流程：
//     1. 傳送 "ROAD_EVENT:<type>" 感測器通知
//     2. 依序傳送 10 幀（frame_id 從 0xF000 開始，與一般串流區隔）
//
// FreeRTOS 架構：
//   Core 0: BLE + 相機傳輸（loop）
//   Core 1: IMU Task（100Hz）+ KNN Task（每500ms推論）
//   Mutex 保護 knn_buf 與 I2C 防止 race condition
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
#include "freertos/event_groups.h"

// ─── Event Group bits ────────────────────────────────────────
#define EVT_CRASH_PENDING  (1 << 0)   // 車禍事件待處理
#define EVT_ROAD_PENDING   (1 << 1)   // 路面事件待處理
EventGroupHandle_t evt_group = NULL;

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

// ─── PTW 摔車偵測（Boubezoul et al. 2013）────────────────
#define PTW_LATERAL_THRESHOLD   1.5f
#define PTW_ROLLRATE_THRESHOLD  100.0f
#define PTW_FREEFALL_THRESHOLD  0.5f
#define PTW_CONFIRM_WINDOW_MS   600

// ─── 按鍵 ────────────────────────────────────────────────
#define DOUBLE_CLICK_MS  400
#define DEBOUNCE_MS       30
#define LONG_PRESS_MS    1000  // 長按門檻（ms）

// ─── BLE ─────────────────────────────────────────────────
#define SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_DATA_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_CTRL_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_SENSOR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define MTU_SIZE        512
#define FRAME_DELAY     200
#define SENSOR_INTERVAL 1

// ─── KNN 路面事件回放 ─────────────────────────────────────
// 觸發類別：pothole / asphalt_bumps / worn_out_road
// 偵測到時傳送最近 ROAD_BUF_SIZE 幀
#define ROAD_BUF_SIZE       10      // 存最近 10 幀 ≈ 2 秒
#define ROAD_EVENT_COOLDOWN 5000    // 觸發後冷卻時間（ms），避免連續觸發
#define ROAD_FRAME_ID_BASE  0xF000  // 路面事件幀 frame_id 起始值
#define ROAD_PLAYBACK_DELAY 500     // 事件幀之間的播放間隔（ms）

// ─── 車禍事件回放 ──────────────────────────────────────────
// 觸發條件：PTW 摔車三條件同時成立
// 傳送最近 10 幀 + 之後 10 幀（共 20 幀）+ 100 筆 IMU 波形
#define CRASH_BUF_SIZE        5       // 撞擊前存 5 幀
#define CRASH_FRAME_ID_BASE   0xE000  // 車禍幀 frame_id 起始值（與路面區隔）
#define CRASH_AFTER_FRAMES    5       // 撞擊後再拍 5 幀
#define CRASH_IMU_WINDOW      100     // IMU 波形長度（筆）= 前後各 50 筆 @100Hz
#define IMU_CRASH_HEADER_0    0xFF
#define IMU_CRASH_HEADER_1    0xDB    // 0xDB = 車禍 IMU（0xDA = 路面 IMU）

// ─── FreeRTOS ─────────────────────────────────────────────
#define IMU_TASK_HZ     100
#define KNN_TASK_MS     500
SemaphoreHandle_t buf_mutex  = NULL;
SemaphoreHandle_t i2c_mutex  = NULL;
SemaphoreHandle_t road_mutex = NULL;  // 保護 road frame buffer

// ─── BLE 物件 ─────────────────────────────────────────────
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
enum CrashState { NORMAL, ALARM_ACTIVE };
CrashState    crashState = NORMAL;
unsigned long alarmStart = 0;
bool          inHighPhase = true;
unsigned long phaseStart  = 0;

// ─── PTW 三條件狀態 ──────────────────────────────────────
bool          ptw_cond1      = false;
bool          ptw_cond2      = false;
unsigned long ptw_cond1_time = 0;
unsigned long ptw_cond2_time = 0;

// ─── 靜音模式 ────────────────────────────────────────────
bool quietMode = false;

// ─── 按鍵狀態 ────────────────────────────────────────────
int           btnLastState    = HIGH;
unsigned long btnLastDebounce = 0;
int           btnClickCount   = 0;
unsigned long btnFirstClick   = 0;
bool          btnLongFired    = false;   // 長按已觸發，避免重複
unsigned long btnPressStart   = 0;       // 按下的時間

// ─── KNN buffer ──────────────────────────────────────────
float knn_buf_z[KNN_WINDOW];
float knn_buf_x[KNN_WINDOW];
float knn_buf_y[KNN_WINDOW];
int   knn_buf_idx  = 0;
bool  knn_buf_full = false;

// ─── IMU 數值 ────────────────────────────────────────────
volatile float imu_accX = 0, imu_accY = 0, imu_accZ = 0;
volatile float imu_gyrX = 0, imu_gyrY = 0, imu_gyrZ = 0;
volatile float imu_mag  = 1.0f;

// ─── KNN 結果 ────────────────────────────────────────────
char  knn_result_str[32]     = "warming_up";
int   knn_votes[KNN_CLASSES] = {0};
float knn_dists[KNN_N];

// ─── Road Event Frame Buffer（Circular）─────────────────
// 每幀最大約 30KB，10 幀最多 300KB，PSRAM 8MB 完全夠
struct RoadFrame {
  uint8_t* data;    // PSRAM 分配的 JPEG 資料
  size_t   len;     // 資料長度
  bool     valid;   // 是否有效
};
RoadFrame road_buf[ROAD_BUF_SIZE];
int  road_buf_idx   = 0;   // 下一個寫入位置
bool road_buf_full  = false;

// ─── Road Event 觸發旗標（KNN Task → loop，用 EventGroup）──
// volatile bool road_event_pending 改用 EVT_ROAD_PENDING bit
char           road_event_type[32] = "";
unsigned long  road_last_trigger   = 0;

// ─── IMU KNN Window 快照（觸發時複製，供傳送）───────────────
// header: 0xFF 0xDA + KNN_WINDOW * 3 floats (z, x, y)
// 共 2 + 30*3*4 = 362 bytes，一個 BLE 封包可容納
#define IMU_PKT_HEADER_0  0xFF
#define IMU_PKT_HEADER_1  0xDA
float imu_snap_z[KNN_WINDOW];  // acc_z（= knn_buf_z）
float imu_snap_x[KNN_WINDOW];  // acc_x
float imu_snap_y[KNN_WINDOW];  // acc_y
bool  imu_snap_valid = false;

// ─── 車禍 IMU Ring Buffer（100筆，由 imuTask 持續滾動寫入）─
// 用 ring buffer 存最近 100 筆，觸發時直接快照
#define CRASH_IMU_BUF   CRASH_IMU_WINDOW   // = 100
float crash_imu_ax[CRASH_IMU_BUF];   // 原始 ax（g）
float crash_imu_ay[CRASH_IMU_BUF];   // 原始 ay（g）
float crash_imu_az[CRASH_IMU_BUF];   // 原始 az（g）
float crash_imu_gx[CRASH_IMU_BUF];   // gyrX（dps）
float crash_imu_gy[CRASH_IMU_BUF];   // gyrY
float crash_imu_gz[CRASH_IMU_BUF];   // gyrZ
float crash_imu_mg[CRASH_IMU_BUF];   // magnitude（g）
int   crash_imu_idx = 0;              // 下一個寫入位置（ring）

// 車禍事件快照（觸發時複製）
float crash_snap_ax[CRASH_IMU_BUF];
float crash_snap_ay[CRASH_IMU_BUF];
float crash_snap_az[CRASH_IMU_BUF];
float crash_snap_gx[CRASH_IMU_BUF];
float crash_snap_gy[CRASH_IMU_BUF];
float crash_snap_gz[CRASH_IMU_BUF];
float crash_snap_mg[CRASH_IMU_BUF];
int   crash_snap_start = 0;   // 快照起始 index（最舊那筆）
bool  crash_snap_valid = false;

// 車禍事件旗標（imuTask → loop，用 EventGroup）
// volatile bool crash_event_pending 改用 EVT_CRASH_PENDING bit
int            crash_after_count     = 0;

// ─── 一般串流 frame_id ───────────────────────────────────
uint16_t frame_id    = 0;
uint16_t sensor_tick = 0;

// ============================================================
// Road Frame Buffer 初始化（PSRAM 分配）
// ============================================================
void initRoadBuffer() {
  for (int i = 0; i < ROAD_BUF_SIZE; i++) {
    road_buf[i].data  = (uint8_t*)ps_malloc(35 * 1024);  // 每幀預留 35KB
    road_buf[i].len   = 0;
    road_buf[i].valid = false;
    if (!road_buf[i].data) {
      Serial.printf("❌ PSRAM 分配失敗 road_buf[%d]\n", i);
    }
  }
  Serial.printf("✅ Road buffer 初始化（%d 幀 × 35KB = %dKB PSRAM）\n",
                ROAD_BUF_SIZE, ROAD_BUF_SIZE * 35);
}

// ─── 把最新一幀存進 circular buffer ─────────────────────
void pushRoadFrame(uint8_t* data, size_t len) {
  if (!road_buf[road_buf_idx].data) return;
  if (xSemaphoreTake(road_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

  size_t copy_len = (len > 35*1024) ? 35*1024 : len;
  memcpy(road_buf[road_buf_idx].data, data, copy_len);
  road_buf[road_buf_idx].len   = copy_len;
  road_buf[road_buf_idx].valid = true;

  road_buf_idx++;
  if (road_buf_idx >= ROAD_BUF_SIZE) {
    road_buf_idx = 0;
    road_buf_full = true;
  }
  xSemaphoreGive(road_mutex);
}

// ─── 觸發後依序傳送 10 幀（最舊 → 最新）────────────────
void sendRoadEventFrames(const char* event_type) {
  if (!deviceConnected) return;

  // 先通知 Python 端：即將傳送事件幀
  char notify[64];
  snprintf(notify, sizeof(notify), "ROAD_EVENT:%s", event_type);
  pSensorChar->setValue((uint8_t*)notify, strlen(notify));
  pSensorChar->notify();
  delay(50);

  Serial.printf("🛣️  傳送 Road Event [%s]，10 幀回放中...\n", event_type);

  // 計算從哪個 index 開始（最舊的那幀）
  int total_valid = 0;
  int start_idx;

  if (xSemaphoreTake(road_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  if (road_buf_full) { start_idx = road_buf_idx; total_valid = ROAD_BUF_SIZE; }
  else               { start_idx = 0;            total_valid = road_buf_idx;  }
  size_t lens[ROAD_BUF_SIZE];
  bool   valids[ROAD_BUF_SIZE];
  for (int i = 0; i < ROAD_BUF_SIZE; i++) {
    lens[i]   = road_buf[i].len;
    valids[i] = road_buf[i].valid;
  }
  xSemaphoreGive(road_mutex);

  // ★ 步驟1：先傳所有影像幀
  uint16_t event_frame_id = ROAD_FRAME_ID_BASE;
  for (int i = 0; i < total_valid; i++) {
    int idx = (start_idx + i) % ROAD_BUF_SIZE;
    if (!valids[idx] || lens[idx] == 0) continue;
    if (!deviceConnected) break;
    Serial.printf("  📤 事件幀 %d/%d (%d bytes)\n", i+1, total_valid, lens[idx]);
    sendFrame(road_buf[idx].data, lens[idx], event_frame_id++);
    delay(ROAD_PLAYBACK_DELAY);
  }

  // ★ 步驟2：影像傳完訊號
  const char* frames_done = "ROAD_FRAMES_DONE";
  pSensorChar->setValue((uint8_t*)frames_done, strlen(frames_done));
  pSensorChar->notify();
  delay(200);  // 等 Python 處理完最後一幀
  Serial.println("  📢 影像傳完，準備傳 IMU 快照");

  // ★ 步驟3：傳 IMU 快照
  if (imu_snap_valid) {
    const int SNAP_BYTES = 2 + KNN_WINDOW * 3 * sizeof(float);
    uint8_t imu_pkt[SNAP_BYTES];
    imu_pkt[0] = IMU_PKT_HEADER_0;
    imu_pkt[1] = IMU_PKT_HEADER_1;
    uint8_t* ptr = imu_pkt + 2;
    memcpy(ptr,                              imu_snap_z, KNN_WINDOW * sizeof(float));
    memcpy(ptr + KNN_WINDOW*sizeof(float),   imu_snap_x, KNN_WINDOW * sizeof(float));
    memcpy(ptr + KNN_WINDOW*sizeof(float)*2, imu_snap_y, KNN_WINDOW * sizeof(float));
    pDataChar->setValue(imu_pkt, SNAP_BYTES);
    pDataChar->notify();
    delay(100);
    Serial.printf("  📊 IMU 快照傳送（%d bytes）\n", SNAP_BYTES);
  }

  // ★ 步驟4：結束通知
  snprintf(notify, sizeof(notify), "ROAD_EVENT_END:%s", event_type);
  pSensorChar->setValue((uint8_t*)notify, strlen(notify));
  pSensorChar->notify();
  Serial.printf("✅ Road Event 傳送完畢\n");
}

// ============================================================
// 車禍事件傳送（圖像前後20幀 + IMU 100筆波形）
// ============================================================
void sendCrashIMUData() {
  if (!crash_snap_valid || !deviceConnected) return;

  // 封包格式：header(2) + CRASH_IMU_BUF*7*4 bytes
  // 7個通道：ax,ay,az,gx,gy,gz,mag 各100筆
  // 共 2 + 100*7*4 = 2802 bytes → 需要分包
  const int CHANNELS  = 7;
  const int DATA_BYTES = CRASH_IMU_BUF * CHANNELS * sizeof(float);  // 2800
  const int PKT_BYTES  = 2 + DATA_BYTES;                             // 2802

  uint8_t* pkt = (uint8_t*)malloc(PKT_BYTES);
  if (!pkt) { Serial.println("❌ malloc crash IMU 失敗"); return; }

  pkt[0] = IMU_CRASH_HEADER_0;  // 0xFF
  pkt[1] = IMU_CRASH_HEADER_1;  // 0xDB

  // 依 ring buffer 順序（最舊 → 最新）排列後打包
  float* dst_ax = (float*)(pkt + 2 + CRASH_IMU_BUF*0*sizeof(float));
  float* dst_ay = (float*)(pkt + 2 + CRASH_IMU_BUF*1*sizeof(float));
  float* dst_az = (float*)(pkt + 2 + CRASH_IMU_BUF*2*sizeof(float));
  float* dst_gx = (float*)(pkt + 2 + CRASH_IMU_BUF*3*sizeof(float));
  float* dst_gy = (float*)(pkt + 2 + CRASH_IMU_BUF*4*sizeof(float));
  float* dst_gz = (float*)(pkt + 2 + CRASH_IMU_BUF*5*sizeof(float));
  float* dst_mg = (float*)(pkt + 2 + CRASH_IMU_BUF*6*sizeof(float));

  for (int i = 0; i < CRASH_IMU_BUF; i++) {
    int idx = (crash_snap_start + i) % CRASH_IMU_BUF;
    dst_ax[i] = crash_snap_ax[idx];
    dst_ay[i] = crash_snap_ay[idx];
    dst_az[i] = crash_snap_az[idx];
    dst_gx[i] = crash_snap_gx[idx];
    dst_gy[i] = crash_snap_gy[idx];
    dst_gz[i] = crash_snap_gz[idx];
    dst_mg[i] = crash_snap_mg[idx];
  }

  // 用 sendFrame 分包傳送（frame_id 用 0xEFFF 表示 IMU 資料包）
  sendFrame(pkt, PKT_BYTES, 0xEFFF);
  free(pkt);
  Serial.printf("  📊 車禍 IMU 波形傳送（%d bytes）\n", PKT_BYTES);
}

void sendCrashEventFrames() {
  if (!deviceConnected) return;

  // 1. 通知 Python 端：車禍事件開始
  const char* notify_start = "CRASH_EVENT_START";
  pSensorChar->setValue((uint8_t*)notify_start, strlen(notify_start));
  pSensorChar->notify();
  delay(50);

  Serial.println("🚨 傳送車禍事件幀...");

  // 2. 傳送撞擊前 10 幀（road_buf circular buffer）
  // ★ IMU 波形改到前幀之後傳，確保 Python 已進入接收狀態
  int total_valid = 0;
  int start_idx;
  if (xSemaphoreTake(road_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  if (road_buf_full) { start_idx = road_buf_idx; total_valid = ROAD_BUF_SIZE; }
  else               { start_idx = 0;            total_valid = road_buf_idx;  }
  size_t lens[ROAD_BUF_SIZE];
  bool   valids[ROAD_BUF_SIZE];
  for (int i=0;i<ROAD_BUF_SIZE;i++) {
    lens[i]   = road_buf[i].len;
    valids[i] = road_buf[i].valid;
  }
  xSemaphoreGive(road_mutex);

  // 只取最近 CRASH_BUF_SIZE（5）幀
  int send_count = (total_valid > CRASH_BUF_SIZE) ? CRASH_BUF_SIZE : total_valid;
  int send_start = (total_valid > CRASH_BUF_SIZE) ?
                   (start_idx + total_valid - CRASH_BUF_SIZE) % ROAD_BUF_SIZE :
                   start_idx;

  uint16_t crash_fid = CRASH_FRAME_ID_BASE;
  for (int i = 0; i < send_count; i++) {
    int idx = (send_start + i) % ROAD_BUF_SIZE;
    if (!valids[idx] || lens[idx]==0 || !deviceConnected) continue;
    Serial.printf("  📤 前幀 %d/%d (%d bytes)\n", i+1, send_count, lens[idx]);
    sendFrame(road_buf[idx].data, lens[idx], crash_fid++);
    delay(500);
  }

  // 3. 通知 Python：開始傳送撞擊後幀
  const char* notify_after = "CRASH_EVENT_AFTER";
  pSensorChar->setValue((uint8_t*)notify_after, strlen(notify_after));
  pSensorChar->notify();

  // 4. 補拍並傳送撞擊後幀
  for (int i = 0; i < CRASH_AFTER_FRAMES; i++) {
    if (!deviceConnected) break;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      Serial.printf("  📤 後幀 %d/%d (%d bytes)\n", i+1, CRASH_AFTER_FRAMES, fb->len);
      sendFrame(fb->buf, fb->len, crash_fid++);
      esp_camera_fb_return(fb);
    }
    delay(500);
  }

  // ★ 步驟5：前後幀全部傳完 → 通知 Python
  const char* frames_done = "CRASH_FRAMES_DONE";
  pSensorChar->setValue((uint8_t*)frames_done, strlen(frames_done));
  pSensorChar->notify();
  delay(200);  // 等 Python 處理完最後一幀
  Serial.println("  📢 影像傳完，準備傳 IMU 波形");

  // ★ 步驟6：傳 IMU 100筆波形
  sendCrashIMUData();
  delay(100);

  // ★ 步驟7：結束通知
  const char* notify_end = "CRASH_EVENT_END";
  pSensorChar->setValue((uint8_t*)notify_end, strlen(notify_end));
  pSensorChar->notify();
  Serial.println("✅ 車禍事件傳送完畢");
}

// ============================================================
// KNN 相關函式
// ============================================================
void knn_push(float az, float ax, float ay) {
  knn_buf_z[knn_buf_idx] = ax * 9.81f;
  knn_buf_x[knn_buf_idx] = az * 9.81f;
  knn_buf_y[knn_buf_idx] = ay * 9.81f;
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

// ─── KNN 是否為不規則路面 ────────────────────────────────
bool isRoughRoad(const char* cls) {
  return (strcmp(cls, "pothole")       == 0 ||
          strcmp(cls, "asphalt_bumps") == 0 ||
          strcmp(cls, "worn_out_road") == 0);
}

// ============================================================
// PTW 摔車偵測（Boubezoul et al. 2013）
// ============================================================
void updateCrashDetection(float mag) {
  if (crashState == ALARM_ACTIVE) return;

  unsigned long now    = millis();
  float lateral  = fabsf(imu_accY);
  float rollRate = fabsf(imu_gyrX);

  if (lateral > PTW_LATERAL_THRESHOLD) {
    ptw_cond1 = true; ptw_cond1_time = now;
    Serial.printf("⚡ [C1] 側向:%.2fg\n", lateral);
  }
  if (rollRate > PTW_ROLLRATE_THRESHOLD) {
    ptw_cond2 = true; ptw_cond2_time = now;
    Serial.printf("🔄 [C2] Roll:%.2f°/s\n", rollRate);
  }
  if (mag < PTW_FREEFALL_THRESHOLD) {
    bool c1v = ptw_cond1 && (now - ptw_cond1_time < PTW_CONFIRM_WINDOW_MS);
    bool c2v = ptw_cond2 && (now - ptw_cond2_time < PTW_CONFIRM_WINDOW_MS);
    if (c1v && c2v) {
      crashState = ALARM_ACTIVE;
      alarmStart = now; phaseStart = now; inHighPhase = true;
      ptw_cond1 = false; ptw_cond2 = false;
      if (!quietMode) buzzerTone(SIREN_HIGH_HZ);
      Serial.printf("🚨 PTW 摔車確認！\n");

      // ★ 複製 IMU ring buffer 快照（撞擊前 100 筆）
      memcpy(crash_snap_ax, crash_imu_ax, sizeof(crash_imu_ax));
      memcpy(crash_snap_ay, crash_imu_ay, sizeof(crash_imu_ay));
      memcpy(crash_snap_az, crash_imu_az, sizeof(crash_imu_az));
      memcpy(crash_snap_gx, crash_imu_gx, sizeof(crash_imu_gx));
      memcpy(crash_snap_gy, crash_imu_gy, sizeof(crash_imu_gy));
      memcpy(crash_snap_gz, crash_imu_gz, sizeof(crash_imu_gz));
      memcpy(crash_snap_mg, crash_imu_mg, sizeof(crash_imu_mg));
      crash_snap_start = crash_imu_idx;
      crash_snap_valid = true;
      crash_after_count = 0;

      // ★ 用 EventGroup 通知 Core 0（跨核心可見）
      xEventGroupSetBitsFromISR(evt_group, EVT_CRASH_PENDING, NULL);

      if (deviceConnected) {
        const char* msg = "CRASH_DETECTED";
        pSensorChar->setValue((uint8_t*)msg, strlen(msg));
        pSensorChar->notify();
      }
    }
  }
  if (ptw_cond1 && (now - ptw_cond1_time > PTW_CONFIRM_WINDOW_MS)) {
    ptw_cond1 = false; Serial.println("↩️  C1 超時");
  }
  if (ptw_cond2 && (now - ptw_cond2_time > PTW_CONFIRM_WINDOW_MS)) {
    ptw_cond2 = false; Serial.println("↩️  C2 超時");
  }
}

// ============================================================
// IMU Task（Core 1，100Hz）
// ============================================================
void imuTask(void* param) {
  const TickType_t period = pdMS_TO_TICKS(1000 / IMU_TASK_HZ);
  TickType_t lastWake = xTaskGetTickCount();
  while (true) {
    if (icmOK) {
      float ax, ay, az, gx, gy, gz;
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        ax = icmReadWord(0x2D) / 4096.0f;
        ay = icmReadWord(0x2F) / 4096.0f;
        az = icmReadWord(0x31) / 4096.0f;
        gx = icmReadWord(0x33) / 131.0f;
        gy = icmReadWord(0x35) / 131.0f;
        gz = icmReadWord(0x37) / 131.0f;
        xSemaphoreGive(i2c_mutex);
      } else { vTaskDelayUntil(&lastWake, period); continue; }

      float mag = sqrtf(ax*ax + ay*ay + az*az);
      imu_accX = ax; imu_accY = ay; imu_accZ = az;
      imu_gyrX = gx; imu_gyrY = gy; imu_gyrZ = gz;
      imu_mag  = mag;

      updateCrashDetection(mag);

      // ★ 持續寫入車禍 IMU ring buffer（不需要 mutex，單一寫入者）
      crash_imu_ax[crash_imu_idx] = ax;
      crash_imu_ay[crash_imu_idx] = ay;
      crash_imu_az[crash_imu_idx] = az;
      crash_imu_gx[crash_imu_idx] = gx;
      crash_imu_gy[crash_imu_idx] = gy;
      crash_imu_gz[crash_imu_idx] = gz;
      crash_imu_mg[crash_imu_idx] = mag;
      crash_imu_idx = (crash_imu_idx + 1) % CRASH_IMU_BUF;

      if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        knn_push(az, ax, ay);
        xSemaphoreGive(buf_mutex);
      }
    }
    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================
// KNN Task（Core 1，每 500ms）
// 推論後若為不規則路面 → 設旗標通知 loop 傳送事件幀
// ============================================================
void knnTask(void* param) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(KNN_TASK_MS));

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

    // ── 不規則路面偵測 → 觸發事件幀回傳 ──────────────────
    unsigned long now = millis();
    bool road_already_pending = (xEventGroupGetBits(evt_group) & EVT_ROAD_PENDING) != 0;
    if (isRoughRoad(knn_result_str) &&
        !road_already_pending &&
        deviceConnected &&
        (now - road_last_trigger > ROAD_EVENT_COOLDOWN)) {
      strncpy(road_event_type, knn_result_str, sizeof(road_event_type));
      road_last_trigger = now;

      // ★ 複製 KNN window 快照
      if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(imu_snap_z, knn_buf_z, sizeof(knn_buf_z));
        memcpy(imu_snap_x, knn_buf_x, sizeof(knn_buf_x));
        memcpy(imu_snap_y, knn_buf_y, sizeof(knn_buf_y));
        imu_snap_valid = knn_buf_full;
        xSemaphoreGive(buf_mutex);
      }

      // ★ 用 EventGroup 通知 Core 0（跨核心可見）
      xEventGroupSetBits(evt_group, EVT_ROAD_PENDING);
      Serial.printf("🛣️  KNN 觸發 [%s]，標記事件幀傳送\n", knn_result_str);
    }
  }
}

// ============================================================
// 蜂鳴器
// ============================================================
void buzzerTone(uint32_t freq) { ledcWriteTone(BUZZER_PIN, freq); }
void buzzerOff()               { ledcWriteTone(BUZZER_PIN, 0);   }
void beep(uint32_t freq, uint32_t ms) { buzzerTone(freq); delay(ms); buzzerOff(); }

void updateBuzzer() {
  if (crashState != ALARM_ACTIVE || quietMode) { buzzerOff(); return; }
  unsigned long now = millis();
  if (now - phaseStart >= SIREN_PHASE_MS) {
    inHighPhase = !inHighPhase; phaseStart = now;
    buzzerTone(inHighPhase ? SIREN_HIGH_HZ : SIREN_LOW_HZ);
  }
}

// ============================================================
// BLE Callbacks
// ============================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true; Serial.println("✅ BLE 連線！");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false; streaming = false;
    Serial.println("❌ BLE 斷線"); s->startAdvertising();
  }
};
class MyCtrlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String val = pChar->getValue().c_str();
    if      (val == "START")       { streaming = true;  Serial.println("▶️  串流開始"); }
    else if (val == "STOP")        { streaming = false; Serial.println("⏹  串流停止"); }
    else if (val == "RESET_ALARM") {
      crashState = NORMAL; ptw_cond1 = false; ptw_cond2 = false; buzzerOff();
      Serial.println("🔕 遠端重置");
    }
  }
};

// ============================================================
// ICM-20948 驅動
// ============================================================
void icmWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.write(data); Wire.endTransmission();
}
uint8_t icmReadByte(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR); Wire.write(reg);
  Wire.endTransmission(false); Wire.requestFrom(ICM_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}
int16_t icmReadWord(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR); Wire.write(reg);
  Wire.endTransmission(false); Wire.requestFrom(ICM_ADDR, 2);
  return (Wire.available() >= 2) ? (Wire.read() << 8) | Wire.read() : 0;
}

bool initICM() {
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000); delay(50);
  Serial.println("--- I2C 掃描 ---");
  bool found = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { Serial.printf("  找到：0x%02X\n", addr); found = true; }
  }
  if (!found) { Serial.println("  ❌ 無 I2C"); return false; }
  uint8_t whoami = icmReadByte(0x00);
  Serial.printf("ICM WHO_AM_I=0x%02X", whoami);
  if (whoami != 0xEA) { Serial.println(" ❌"); return false; }
  Serial.println(" ✅");
  icmWriteByte(0x06, 0x80); delay(100);  // reset
  icmWriteByte(0x06, 0x01); delay(50);   // wake up
  // ★ 切到 Bank 2 → 設定 ±8g
  icmWriteByte(0x7F, 0x20);   // Bank 2
  icmWriteByte(0x14, 0x04);   // ACCEL_CONFIG: ±8g
  delay(10);
  icmWriteByte(0x7F, 0x00);   // 切回 Bank 0
  delay(10);
  Serial.println("✅ ICM-20948 OK（±8g）");
  return true;
}

// ============================================================
// 相機初始化
// ============================================================
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
  config.grab_mode=CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location=CAMERA_FB_IN_PSRAM;
  config.frame_size=FRAMESIZE_QVGA;
  config.jpeg_quality=15; config.fb_count=2;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("❌ 相機失敗:0x%x\n", err); return false; }
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s,1); s->set_hmirror(s,1);
  s->set_whitebal(s,1); s->set_awb_gain(s,1); s->set_exposure_ctrl(s,1);
  Serial.println("✅ 相機 OK (QVGA)");
  return true;
}

// ============================================================
// BLE 初始化
// ============================================================
void initBLE() {
  BLEDevice::init("ESP32-S3-CAM"); BLEDevice::setMTU(517);
  pServer = BLEDevice::createServer(); pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pDataChar = pService->createCharacteristic(CHAR_DATA_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pDataChar->addDescriptor(new BLE2902());
  pCtrlChar = pService->createCharacteristic(CHAR_CTRL_UUID, BLECharacteristic::PROPERTY_WRITE);
  pCtrlChar->setCallbacks(new MyCtrlCallbacks());
  pSensorChar = pService->createCharacteristic(CHAR_SENSOR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pSensorChar->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID); pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("✅ BLE 廣播中");
}

// ============================================================
// 圖片分包傳送
// ============================================================
void sendFrame(uint8_t* data, size_t len, uint16_t fid) {
  const size_t DPC = MTU_SIZE - 6;
  uint16_t total = (len + DPC - 1) / DPC;
  for (uint16_t i = 0; i < total; i++) {
    if (!deviceConnected) return;
    size_t offset = i * DPC;
    size_t chunk_len = min(DPC, len - offset);
    uint8_t packet[MTU_SIZE];
    packet[0]=(fid>>8)&0xFF; packet[1]=fid&0xFF;
    packet[2]=(i>>8)&0xFF;   packet[3]=i&0xFF;
    packet[4]=(total>>8)&0xFF; packet[5]=total&0xFF;
    memcpy(packet+6, data+offset, chunk_len);
    pDataChar->setValue(packet, chunk_len+6); pDataChar->notify(); delay(5);
  }
}

// ============================================================
// 感測器資料傳送
// ============================================================
void sendSensorData() {
  float accZ=imu_accX, accX=imu_accZ, accY=imu_accY;
  float gyrX=imu_gyrX, gyrY=imu_gyrY, gyrZ=imu_gyrZ, mag=imu_mag;
  const char* stateStr = (crashState==ALARM_ACTIVE)?"CRASH":"OK";
  const char* c1str = ptw_cond1?"1":"0";
  const char* c2str = ptw_cond2?"1":"0";
  char proba_str[64]="";
  for (int i=0;i<KNN_CLASSES;i++) {
    int pct = knn_buf_full?(knn_votes[i]*100/KNN_K):0;
    char tmp[8]; snprintf(tmp,sizeof(tmp),"%d",pct);
    strcat(proba_str,tmp);
    if (i<KNN_CLASSES-1) strcat(proba_str,",");
  }
  char buffer[256];
  if (dhtReady&&icmOK)
    snprintf(buffer,sizeof(buffer),
      "T:%.1fC H:%.1f%% A:%.2f,%.2f,%.2f G:%.2f,%.2f,%.2f M:%.2fg S:%s C1:%s C2:%s Q:%d R:%s P:%s",
      lastTemp,lastHumi,accX,accY,accZ,gyrX,gyrY,gyrZ,mag,
      stateStr,c1str,c2str,quietMode?1:0,knn_result_str,proba_str);
  else if (!dhtReady&&icmOK)
    snprintf(buffer,sizeof(buffer),
      "T:ERR H:ERR A:%.2f,%.2f,%.2f G:%.2f,%.2f,%.2f M:%.2fg S:%s C1:%s C2:%s Q:%d R:%s P:%s",
      accX,accY,accZ,gyrX,gyrY,gyrZ,mag,
      stateStr,c1str,c2str,quietMode?1:0,knn_result_str,proba_str);
  else
    snprintf(buffer,sizeof(buffer),
      "T:ERR H:ERR A:ERR G:ERR S:%s C1:%s C2:%s Q:%d R:%s P:%s",
      stateStr,c1str,c2str,quietMode?1:0,knn_result_str,proba_str);
  Serial.println(buffer);
  if (deviceConnected) {
    pSensorChar->setValue((uint8_t*)buffer,strlen(buffer)); pSensorChar->notify();
  }
}

// ============================================================
// 按鍵處理
// ============================================================
void onLongPress() {
  // 長按（>1秒）→ 強制觸發 road event + 複製 IMU 快照（測試用）
  bool already = (xEventGroupGetBits(evt_group) & EVT_ROAD_PENDING) != 0;
  if (!already && deviceConnected) {
    strncpy(road_event_type, "pothole", sizeof(road_event_type));
    road_last_trigger = millis();

    // ★ 複製 KNN window 快照（與 knnTask 觸發時相同）
    if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(imu_snap_z, knn_buf_z, sizeof(knn_buf_z));
      memcpy(imu_snap_x, knn_buf_x, sizeof(knn_buf_x));
      memcpy(imu_snap_y, knn_buf_y, sizeof(knn_buf_y));
      imu_snap_valid = knn_buf_full;
      xSemaphoreGive(buf_mutex);
    }

    xEventGroupSetBits(evt_group, EVT_ROAD_PENDING);
    beep(1200, 80); delay(40); beep(1200, 80);
    Serial.println("🧪 [TEST] 長按強制觸發 road event（pothole）+ IMU 快照");
  } else {
    beep(400, 200);
    Serial.println("⚠️  road event 冷卻中或未連線");
  }
}

void onSingleClick() {
  if (crashState==ALARM_ACTIVE) { Serial.println("💡 雙擊解除"); return; }
  quietMode=!quietMode;
  if (quietMode) { buzzerOff(); beep(900,150); delay(70); beep(500,150); Serial.println("🔇 靜音ON"); }
  else           { beep(1800,200); Serial.println("🔊 靜音OFF"); }
  if (deviceConnected) {
    const char* msg=quietMode?"QUIET_ON":"QUIET_OFF";
    pSensorChar->setValue((uint8_t*)msg,strlen(msg)); pSensorChar->notify();
  }
}
void onDoubleClick() {
  crashState=NORMAL; ptw_cond1=false; ptw_cond2=false; quietMode=false; buzzerOff();
  beep(1500,80); delay(40); beep(1500,80); delay(40); beep(1500,80);
  Serial.println("✅ 我沒事！");
  if (deviceConnected) {
    const char* msg="SAFE";
    pSensorChar->setValue((uint8_t*)msg,strlen(msg)); pSensorChar->notify();
  }
}
void updateButton() {
  int reading=digitalRead(BUTTON_PIN); unsigned long now=millis();

  // ── 長按偵測（按住不放）─────────────────────────────────
  if (reading==LOW && btnLastState==LOW) {
    if (!btnLongFired && btnPressStart>0 && (now-btnPressStart)>=LONG_PRESS_MS) {
      btnLongFired = true;
      btnClickCount = 0; btnFirstClick = 0;  // 取消單/雙擊
      onLongPress();
    }
    btnLastDebounce = now;
    return;
  }

  if (reading==btnLastState) { btnLastDebounce=now; return; }
  if ((now-btnLastDebounce)<DEBOUNCE_MS) return;
  btnLastDebounce=now; btnLastState=reading;

  if (reading==LOW) {
    // 按下
    btnPressStart = now;
    btnLongFired  = false;
    btnClickCount++;
    if (btnClickCount==1) btnFirstClick=now;
    else if (btnClickCount>=2) { onDoubleClick(); btnClickCount=0; btnFirstClick=0; return; }
  } else {
    // 放開：長按已觸發就不算 click
    if (btnLongFired) { btnLongFired=false; btnPressStart=0; return; }
    btnPressStart = 0;
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

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-S3 BLE CAM + PTW + Road Event v2 ===");

  buf_mutex  = xSemaphoreCreateMutex();
  i2c_mutex  = xSemaphoreCreateMutex();
  road_mutex = xSemaphoreCreateMutex();
  evt_group  = xEventGroupCreate();

  ledcAttach(BUZZER_PIN,2000,8);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  beep(800,120); delay(60); beep(1200,120); delay(60); beep(1800,120);

  dht.begin();
  icmOK = initICM();
  if (!initCamera()) { while(true) delay(1000); }
  initRoadBuffer();   // ★ Road frame buffer 初始化
  initBLE();

  xTaskCreatePinnedToCore(imuTask, "IMU", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(knnTask, "KNN", 8192, NULL, 1, NULL, 1);

  Serial.println("─────────────────────────────────────────────");
  Serial.printf("Road buffer: %d 幀 × 35KB = %dKB PSRAM\n",
                ROAD_BUF_SIZE, ROAD_BUF_SIZE*35);
  Serial.printf("觸發冷卻: %dms | 播放間隔: %dms\n",
                ROAD_EVENT_COOLDOWN, ROAD_PLAYBACK_DELAY);
  Serial.println("⏳ 等待連線...");
}

// ============================================================
// Loop（Core 0）
// ============================================================
void loop() {
  unsigned long now = millis();
  updateButton(); checkButtonTimeout(); updateBuzzer();

  // DHT11
  if (now-lastDHTTime>=2000) {
    lastDHTTime=now;
    if (xSemaphoreTake(i2c_mutex,pdMS_TO_TICKS(50))==pdTRUE) {
      float t=dht.readTemperature(), h=dht.readHumidity();
      xSemaphoreGive(i2c_mutex);
      if (!isnan(t)&&!isnan(h)) { lastTemp=t; lastHumi=h; dhtReady=true; }
      else Serial.println("⚠️  DHT11 失敗");
    }
  }

  // ── 一般串流模式：持續拍照存入 circular buffer ──────────
  if (deviceConnected && streaming) {
    unsigned long t0 = millis();
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      // ★ 每幀都存入 road circular buffer
      pushRoadFrame(fb->buf, fb->len);

      // ★ 讀取 EventGroup bits（跨核心安全）
      EventBits_t bits = xEventGroupGetBits(evt_group);

      // ★ 車禍事件 → 最高優先
      if (bits & EVT_CRASH_PENDING) {
        xEventGroupClearBits(evt_group, EVT_CRASH_PENDING);
        esp_camera_fb_return(fb);
        sendCrashEventFrames();
        sendSensorData();
        return;
      }

      // ★ KNN 路面事件
      if (bits & EVT_ROAD_PENDING) {
        xEventGroupClearBits(evt_group, EVT_ROAD_PENDING);
        esp_camera_fb_return(fb);
        sendRoadEventFrames(road_event_type);
        sendSensorData();
        return;
      }

      // 一般串流傳送
      Serial.printf("📸 Frame %d | %d bytes\n", frame_id, fb->len);
      sendFrame(fb->buf, fb->len, frame_id++);
      esp_camera_fb_return(fb);
    } else {
      Serial.println("❌ 拍照失敗");
    }
    sensor_tick++;
    if (sensor_tick>=SENSOR_INTERVAL) { sensor_tick=0; sendSensorData(); }
    long elapsed=millis()-t0;
    if (elapsed<FRAME_DELAY) delay(FRAME_DELAY-elapsed);

  } else {
    // ── 非串流模式：仍然持續拍照存入 buffer ───────────────
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      pushRoadFrame(fb->buf, fb->len);  // ★ 非串流也要存
      esp_camera_fb_return(fb);
    }

    // 非串流時若觸發，仍傳送事件幀
    if (deviceConnected) {
      EventBits_t bits = xEventGroupGetBits(evt_group);
      if (bits & EVT_CRASH_PENDING) {
        xEventGroupClearBits(evt_group, EVT_CRASH_PENDING);
        sendCrashEventFrames();
      }
      if (bits & EVT_ROAD_PENDING) {
        xEventGroupClearBits(evt_group, EVT_ROAD_PENDING);
        sendRoadEventFrames(road_event_type);
      }
    }

    sendSensorData();
    delay(200);
  }
}
