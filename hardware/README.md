# GuardianRide — Hardware & Firmware 🏍️🔧

智慧安全帽硬體端的韌體開發紀錄，包含感測器資料擷取、影像辨識、BLE 傳輸與跌倒/路況偵測等功能，依照開發迭代（Scrum）分階段呈現。

## 硬體元件

- **ESP32-S3-CAM**：主控板，內建相機模組
- **ICM20948**：九軸 IMU（加速度計 / 陀螺儀 / 磁力計）
- **DHT11**：溫濕度感測器
- **BLE (Bluetooth Low Energy)**：用於與手機 App 傳輸感測資料、影像與偵測結果
- **Wi-Fi**：用於影像串流與雲端辨識

## 開發歷程（Scrum）

| 階段 | 主題 | 說明 |
|------|------|------|
| **Scrum1** | IMU & 溫度資料擷取（BLE） | 透過 BLE 讀取 ICM20948（IMU）與 DHT11（溫度）感測數據，建立感測器與韌體的基礎串接 |
| **Scrum2 / Scrum3** | Wi-Fi 影像傳輸 | 使用 ESP32-S3-CAM 透過 Wi-Fi 取得即時影像畫面 |
| **Scrum4** | 影像辨識（PC 端 YOLO） | 將 ESP32 傳來的影像於電腦端以 YOLO 模型進行車輛辨識（`car_detector.py`） |
| **Scrum5** | 影像 + 感測資料整合傳輸（BLE） | 整合相機影像與 IMU / 溫度資料，統一透過 BLE 傳送 |
| **Scrum6** | 路況辨識（KNN） | 加入 KNN 模型，於裝置端進行路況類型分類，並透過 BLE 即時回傳辨識結果 |
| **Additional / Scrum7** | 跌倒偵測 + 資料蒐集 | 擴充跌倒偵測（Fall Detection）KNN 模型，並提供 BLE viewer 工具蒐集更多訓練資料 |

## 資料夾結構

```
hardware/
├── Scrum1_get_IMU_TMP_data_by_ble/
│   └── BLE_DHT11_ICM20948/
│       └── BLE_DHT11_ICM20948.ino
│
├── Scrum2_get_img_by_wifi/
│
├── Scrum3_get_img_by_wifi/
│   └── ESP32_S3_cam_test/
│       └── ESP32_S3_cam_test.ino
│
├── Scrum4_img_identify_car_yolo_in_pc/
│   └── ESP32_S3_cam_test/
│       ├── ESP32_S3_cam_test.ino
│       └── car_detector.py
│
├── Scrum5_send_img_imu_tmp_by_ble/
│   └── esp32s3_ble_cam_sensor/
│       └── esp32s3_ble_cam_sensor.ino
│
├── Scrum6_send_img_imu_tmp_knnRoadType/
│   ├── ble_viewer_with_c1c2c3.py
│   └── full_crash_cam_knn_2core/
│       ├── full_crash_cam_knn_2core.ino
│       ├── knn_model.h
│       └── knn_model1.h
│
└── Additional_Scrum7_capture_more_data/
    ├── ble_viewer_with_c1c2c3_capture.py
    └── esp32s3_ptw_fall_v3_demo/
        ├── esp32s3_ptw_fall_v3_demo.ino
        ├── knn_model.h
        └── knn_modelneck1.h
```

## 開發環境

- **Arduino IDE**（ESP32 board package）
- **Python 3**（用於 PC 端 BLE viewer / YOLO 影像辨識腳本）
- 主要函式庫：BLE、ICM20948、DHT11、ESP32 Camera

## 使用方式

1. 使用 Arduino IDE 開啟對應 Scrum 資料夾內的 `.ino` 檔案
2. 選擇開發板：`ESP32S3 Dev Module`
3. 燒錄至 ESP32-S3-CAM
4. 若該階段含 Python 腳本（如 `ble_viewer_*.py` / `car_detector.py`），於 PC 端安裝相依套件後執行，用於接收 BLE 資料或進行影像辨識

## 備註

此資料夾記錄了硬體韌體從基礎感測器讀取，逐步擴充到影像辨識、路況分類與跌倒偵測的完整開發過程，對應 [NTPU AI Hackathon 2025](../README.md) 專案的硬體端實作。