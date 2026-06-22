#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "esp_http_server.h"

// ==============================
// 設定你的 WiFi
// ==============================
const char* ssid     = "李秉則的iPhone";
const char* password = "19990223";

// ==============================
// 針腳定義（依你的板子調整）
// ==============================
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

// ==============================
// MJPEG Stream 設定
// ==============================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

// ==============================
// 串流 Handler
// ==============================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("❌ 相機擷取失敗");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.println("❌ JPEG 轉換失敗");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }

    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64,
                             _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                  strlen(_STREAM_BOUNDARY));
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (res != ESP_OK) break;
  }
  return res;
}

// ==============================
// 首頁 Handler（顯示串流畫面）
// ==============================
static esp_err_t index_handler(httpd_req_t *req) {
  const char* html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <title>ESP32-S3 CAM</title>
  <style>
    body { background:#111; color:#eee; text-align:center; font-family:sans-serif; }
    img  { max-width:100%; border:2px solid #444; border-radius:8px; margin-top:20px; }
    h2   { color:#4fc3f7; }
  </style>
</head>
<body>
  <h2>📷 ESP32-S3-CAM 即時預覽</h2>
  <img src='/stream' />
</body>
</html>
  )";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

// ==============================
// 啟動 Web Server
// ==============================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("✅ Web Server 啟動成功");
  }
}

// ==============================
// 相機初始化
// ==============================
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

  // ✅ 修正：fb_location 移到 if/else 裡面
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_UXGA;  // 1600x1200
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;  // ← PSRAM存在才用
    Serial.println("✅ PSRAM 找到，使用高解析度");
  } else {
    config.frame_size   = FRAMESIZE_SVGA;  // 800x600
    config.jpeg_quality = 12;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;   // ← 沒有PSRAM用RAM
    Serial.println("⚠️  無 PSRAM，使用較低解析度");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ 相機初始化失敗，錯誤碼: 0x%x\n", err);
    return false;
  }

  // 調整感測器參數
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 10);
  s->set_brightness(s, 1);
  s->set_saturation(s, 0);
  s->set_gainceiling(s, (gainceiling_t)2);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  Serial.println("✅ 相機初始化成功！");
  return true;
}

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-S3-CAM 測試 ===");

  if (!initCamera()) {
    Serial.println("🔴 相機初始化失敗，停止執行");
    while (true) delay(1000);
  }

  Serial.println("📸 測試拍照中...");
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    Serial.printf("✅ 拍照成功！大小: %d bytes, 解析度: %dx%d\n",
                  fb->len, fb->width, fb->height);
    esp_camera_fb_return(fb);
  } else {
    Serial.println("❌ 拍照失敗");
  }

  Serial.printf("📡 連接 WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("✅ WiFi 連接成功！");
  Serial.printf("📌 IP 位址: http://%s\n", WiFi.localIP().toString().c_str());
  Serial.println("👉 用瀏覽器開啟上方網址查看即時影像");

  startCameraServer();
}

// ==============================
// Loop
// ==============================
void loop() {
  delay(10000);
  Serial.printf("⏱ 運行中... IP: %s\n", WiFi.localIP().toString().c_str());
}
