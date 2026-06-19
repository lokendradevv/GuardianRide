#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ble_viewer_road_event.py
ESP32-S3 BLE CAM + Sensor Viewer
新增：
  - Road Event 回放（收到 ROAD_EVENT 時播放 10 幀，停在最後一幀）
  - 事件幀自動儲存至 POTHOLE_ACCIDENT_DATA/<timestamp>/
  - C1 / C2 / C3 摔車條件顯示
"""

import asyncio, struct, threading, time, queue, os
import cv2, numpy as np
from collections import deque
from datetime import datetime
from bleak import BleakClient, BleakScanner

# ── BLE 設定 ──────────────────────────────────────
SERVICE_UUID     = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_DATA_UUID   = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHAR_CTRL_UUID   = "beb5483e-36e1-4688-b7f5-ea07361b26a9"
CHAR_SENSOR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26aa"
DEVICE_NAME      = "ESP32-S3-CAM"
SENSOR_TIMEOUT   = 3.0
RECONNECT_INTERVAL = 3.0

# ── 儲存路徑 ──────────────────────────────────────
SAVE_ROOT = r"C:\Users\PT Zenbook\Desktop\研究所準備\修課\IoT\code\POTHHOLE_ACCIDENT_DATA"
ROAD_FRAME_ID_BASE = 0xF000   # 對應 .ino 的 ROAD_FRAME_ID_BASE

# ── KNN 類別 ──────────────────────────────────────
CLASS_ORDER = ["asphalt_bumps", "pothole", "regular_road", "worn_out_road"]
CLASS_COLORS = {
    "regular_road":  (80,  200, 80),
    "worn_out_road": (80,  160, 80),
    "asphalt_bumps": (30,  180, 230),
    "pothole":       (50,  50,  230),
}
ROUGH_ROAD = {"pothole", "asphalt_bumps", "worn_out_road"}
IMU_PKT_HEADER   = bytes([0xFF, 0xDA])  # 路面 IMU 快照 header
IMU_CRASH_HEADER = bytes([0xFF, 0xDB])  # 車禍 IMU 波形 header
KNN_WINDOW       = 30                   # 路面 IMU 快照筆數
CRASH_IMU_WINDOW = 100                  # 車禍 IMU 波形筆數
CRASH_FRAME_ID_BASE = 0xE000            # 車禍幀 frame_id 起始值

# ── IMU 接收旗標（影像傳完後才接受）──────────────────────────
road_imu_ready  = False   # 收到 ROAD_FRAMES_DONE 後才接受路面 IMU
crash_imu_ready = False   # 收到 CRASH_FRAMES_DONE 後才接受車禍 IMU

# ── 共用狀態 ──────────────────────────────────────
connection_status = "DISCONNECTED"
last_sensor_time  = 0.0
has_camera        = False
frame_count       = 0

sensor_data = {
    "temp":"--","humi":"--",
    "accX":"--","accY":"--","accZ":"--",
    "gyrX":"--","gyrY":"--","gyrZ":"--",
    "mag":"--","state":"OK","quiet":"0",
}

knn_result     = "warming_up"
knn_proba      = {c: 0.0 for c in CLASS_ORDER}
knn_confidence = 0.0

# ── Crash Event 狀態 ──────────────────────────────
crash_event_active     = False  # 車禍事件進行中
crash_collecting_after = False  # 是否在收撞擊後幀
crash_event_before     = []     # 撞擊前幀（cv2 image）
crash_event_after      = []     # 撞擊後幀
crash_event_raw_b      = []     # 撞擊前原始 JPEG
crash_event_raw_a      = []     # 撞擊後原始 JPEG
crash_imu_data         = None   # IMU 波形 dict
crash_playback_frames  = []     # 播放清單（before+after）
crash_playback_idx     = 0
crash_playback_done    = False
crash_save_dir         = ""

# ── Road Event 狀態 ───────────────────────────────
road_event_active   = False   # 正在播放事件幀
road_event_type_str = ""      # 觸發的路面類型
road_event_frames   = []      # 收到的事件幀（cv2 image）
road_event_raw      = []      # 收到的原始 JPEG bytes（用於存檔）
road_playback_idx   = 0       # 目前播放到第幾幀
road_playback_done  = False   # 是否播放完畢（停在最後一幀）
road_event_save_dir = ""      # 本次事件的儲存資料夾

frame_queue: queue.Queue = queue.Queue(maxsize=2)
stop_event = threading.Event()

# ── 折線圖 buffer ─────────────────────────────────
PLOT_LEN = 200
buf_accX = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
buf_accY = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
buf_accZ = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
buf_gyrX = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
buf_gyrY = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
buf_gyrZ = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
buf_mag  = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
buf_pitc = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
buf_roll = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)

FONT = cv2.FONT_HERSHEY_SIMPLEX

# ── 建立儲存資料夾 ────────────────────────────────
def make_event_dir(event_type: str) -> str:
    ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = os.path.join(SAVE_ROOT, f"{ts}_{event_type}")
    os.makedirs(folder, exist_ok=True)
    return folder

def save_event_frames(frames_raw: list, folder: str, event_type: str):
    """把事件幀 JPEG bytes 存成 jpg 檔，並儲存觸發當時所有感測器數值"""
    for i, raw in enumerate(frames_raw):
        path = os.path.join(folder, f"frame_{i:02d}.jpg")
        with open(path, "wb") as f:
            f.write(raw)

    # 計算觸發當時的衍生數值
    try:    ay_val   = abs(float(sensor_data.get("accY","0")))
    except: ay_val   = 0.0
    try:    gyrX_val = abs(float(sensor_data.get("gyrX","0")))
    except: gyrX_val = 0.0
    try:    mag_val  = float(sensor_data.get("mag","1.0"))
    except: mag_val  = 1.0

    c1_hit = ay_val   > 1.5
    c2_hit = gyrX_val > 100.0
    c3_hit = mag_val  < 0.5

    proba_lines = "\n".join(
        f"  {cls:<20}: {knn_proba.get(cls,0.0):.0%}"
        for cls in CLASS_ORDER
    )

    meta_path = os.path.join(folder, "info.txt")
    with open(meta_path, "w", encoding="utf-8") as f:
        f.write("=" * 40 + "\n")
        f.write("  ROAD EVENT TRIGGER LOG\n")
        f.write("=" * 40 + "\n\n")
        f.write("[Event]\n")
        f.write(f"  event_type  : {event_type}\n")
        f.write(f"  timestamp   : {datetime.now().isoformat()}\n")
        f.write(f"  frames      : {len(frames_raw)}\n\n")
        f.write("[KNN Result]\n")
        f.write(f"  winner      : {knn_result}\n")
        f.write(f"  confidence  : {knn_confidence:.0%}\n")
        f.write(f"  votes:\n{proba_lines}\n\n")
        f.write("[Accelerometer (g)]\n")
        f.write(f"  X           : {sensor_data.get('accX','--')}\n")
        f.write(f"  Y           : {sensor_data.get('accY','--')}\n")
        f.write(f"  Z           : {sensor_data.get('accZ','--')}\n")
        f.write(f"  magnitude   : {sensor_data.get('mag','--')} g\n\n")
        f.write("[Gyroscope (dps)]\n")
        f.write(f"  X (roll)    : {sensor_data.get('gyrX','--')}\n")
        f.write(f"  Y           : {sensor_data.get('gyrY','--')}\n")
        f.write(f"  Z           : {sensor_data.get('gyrZ','--')}\n\n")
        f.write("[Fall Conditions (Boubezoul 2013)]\n")
        f.write(f"  C1 |ay|     : {ay_val:.3f} g   (>1.5g)   {'HIT' if c1_hit else 'ok'}\n")
        f.write(f"  C2 |gyrX|   : {gyrX_val:.2f} dps (>100)    {'HIT' if c2_hit else 'ok'}\n")
        f.write(f"  C3 mag      : {mag_val:.3f} g   (<0.5g)   {'HIT' if c3_hit else 'ok'}\n\n")
        f.write("[Environment]\n")
        f.write(f"  temperature : {sensor_data.get('temp','--')} C\n")
        f.write(f"  humidity    : {sensor_data.get('humi','--')} %\n\n")
        f.write("[System]\n")
        f.write(f"  crash_state : {sensor_data.get('state','--')}\n")
        f.write(f"  quiet_mode  : {'ON' if sensor_data.get('quiet')=='1' else 'OFF'}\n")

    print(f"💾 事件幀已儲存至：{folder}（{len(frames_raw)} 幀）")


# ── 感測器解析 ────────────────────────────────────
def parse_sensor(raw: str):
    global knn_result, knn_proba, knn_confidence
    global road_event_active, road_event_type_str
    global road_event_frames, road_event_raw
    global road_playback_idx, road_playback_done, road_event_save_dir
    global road_imu_ready
    global crash_event_active, crash_collecting_after
    global crash_event_before, crash_event_after
    global crash_event_raw_b, crash_event_raw_a
    global crash_imu_data, crash_playback_frames
    global crash_playback_idx, crash_playback_done, crash_save_dir
    global crash_imu_ready

    # ── Crash Event 通知 ──────────────────────────
    if raw == "CRASH_EVENT_START":
        crash_event_active     = True
        crash_collecting_after = False
        crash_event_before     = []
        crash_event_after      = []
        crash_event_raw_b      = []
        crash_event_raw_a      = []
        crash_imu_data         = None
        crash_playback_done    = False
        crash_playback_frames  = []
        crash_imu_ready        = False   # ★ 重置，等 CRASH_FRAMES_DONE
        crash_save_dir = make_event_dir("CRASH")
        print("🚨 Crash Event 開始，準備接收事件幀...")
        return

    if raw == "CRASH_EVENT_AFTER":
        crash_collecting_after = True
        print("  🔄 開始接收撞擊後幀...")
        return

    if raw == "CRASH_FRAMES_DONE":
        crash_imu_ready = True
        print("  📢 車禍影像傳完，等待 IMU 波形...")
        return

    if raw == "CRASH_EVENT_END":
        crash_playback_frames = crash_event_before + crash_event_after
        crash_playback_idx    = 0
        crash_playback_done   = False
        print(f"✅ Crash Event 結束：前{len(crash_event_before)}幀 + 後{len(crash_event_after)}幀")
        _save_crash_event()
        return

    # ── Road Event 通知 ───────────────────────────
    if raw == "ROAD_FRAMES_DONE":
        road_imu_ready = True
        print("  📢 路面影像傳完，等待 IMU 快照...")
        return

    if raw.startswith("ROAD_EVENT:") and not raw.startswith("ROAD_EVENT_END"):
        event_type = raw.split(":", 1)[1]
        road_event_active   = True
        road_event_type_str = event_type
        road_event_frames   = []
        road_event_raw      = []
        road_playback_idx   = 0
        road_playback_done  = False
        road_imu_ready      = False   # ★ 重置，等 ROAD_FRAMES_DONE
        road_event_save_dir = make_event_dir(event_type)
        print(f"🛣️  Road Event 開始：{event_type}，準備接收事件幀...")
        return

    if raw.startswith("ROAD_EVENT_END:"):
        event_type = raw.split(":", 1)[1]
        print(f"✅ Road Event 結束：{event_type}，共 {len(road_event_frames)} 幀")
        # 儲存到電腦
        if road_event_raw:
            save_event_frames(road_event_raw, road_event_save_dir, event_type)
        road_playback_done = True
        return

    # ── 一般事件字串 ──────────────────────────────
    if raw in ("CRASH_DETECTED","SAFE","QUIET_ON","QUIET_OFF","RESET"):
        if raw == "CRASH_DETECTED": sensor_data["state"] = "CRASH"
        elif raw == "SAFE":         sensor_data["state"] = "OK"
        elif raw == "QUIET_ON":     sensor_data["quiet"] = "1"
        elif raw == "QUIET_OFF":    sensor_data["quiet"] = "0"
        return

    try:
        for p in raw.split():
            if   p.startswith("T:"): sensor_data["temp"]  = p[2:].rstrip("C")
            elif p.startswith("H:"): sensor_data["humi"]  = p[2:].rstrip("%")
            elif p.startswith("M:"): sensor_data["mag"]   = p[2:].rstrip("g")
            elif p.startswith("S:"): sensor_data["state"] = p[2:]
            elif p.startswith("Q:"): sensor_data["quiet"] = p[2:]
            elif p.startswith("R:"): knn_result = p[2:]
            elif p.startswith("P:"):
                parts = p[2:].split(",")
                if len(parts) == len(CLASS_ORDER):
                    knn_proba = {CLASS_ORDER[i]: int(parts[i])/100.0
                                 for i in range(len(CLASS_ORDER))}
                    knn_confidence = max(knn_proba.values()) if knn_proba else 0.0
            elif p.startswith("A:"):
                v = p[2:].split(",")
                if len(v)==3:
                    sensor_data["accX"],sensor_data["accY"],sensor_data["accZ"] = v
                    ax,ay,az = float(v[0]),float(v[1]),float(v[2])
                    buf_accX.append(ax); buf_accY.append(ay); buf_accZ.append(az)
                    buf_mag.append((ax**2+ay**2+az**2)**0.5)
                    buf_pitc.append(np.degrees(np.arctan2(-ax, np.sqrt(ay**2+az**2))))
                    buf_roll.append(np.degrees(np.arctan2(ay, az)))
            elif p.startswith("G:"):
                v = p[2:].split(",")
                if len(v)==3:
                    sensor_data["gyrX"],sensor_data["gyrY"],sensor_data["gyrZ"] = v
                    buf_gyrX.append(float(v[0]))
                    buf_gyrY.append(float(v[1]))
                    buf_gyrZ.append(float(v[2]))
    except Exception as e:
        print(f"⚠️  解析錯誤: {e} | raw='{raw}'")

# ── BLE 回呼：影像 ────────────────────────────────
_frame_buf={}; _cur_frame=-1; _total_chunks=0

def image_cb(sender, data: bytearray):
    global _frame_buf,_cur_frame,_total_chunks,frame_count
    global road_event_frames, road_event_raw, road_playback_idx

    if len(data) < 2: return

    # ── 路面 IMU 快照（0xFF 0xDA）────────────────────────────
    if data[0] == 0xFF and data[1] == 0xDA:
        print(f"📊 收到路面 IMU 快照（{len(data)} bytes）")
        _parse_imu_snapshot(bytes(data[2:]))
        return

    if len(data) < 6: return
    fid   = struct.unpack(">H",data[0:2])[0]
    cid   = struct.unpack(">H",data[2:4])[0]
    total = struct.unpack(">H",data[4:6])[0]
    chunk = data[6:]

    if fid != _cur_frame: _frame_buf={}; _cur_frame=fid; _total_chunks=total
    _frame_buf[cid] = chunk

    if len(_frame_buf) == _total_chunks:
        jpeg = b"".join(_frame_buf[i] for i in range(_total_chunks))
        _frame_buf = {}

        # ── 車禍 IMU 波形包（frame_id=0xEFFF，影像傳完後才接受）──
        if fid == 0xEFFF:
            if crash_imu_ready:
                if jpeg[:2] == IMU_CRASH_HEADER:
                    print(f"📊 收到車禍 IMU 波形（{len(jpeg)} bytes）")
                    _parse_crash_imu(jpeg[2:])
                else:
                    print(f"⚠️  0xEFFF header 不符：{jpeg[:2].hex()}")
            else:
                print(f"⚠️  車禍 IMU 波形提早到，丟棄（影像尚未傳完）")
            return

        # 一般影像幀才做 decode
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None: return

        # ── 車禍事件幀（frame_id 0xE000~0xEFFE）─────────────
        if CRASH_FRAME_ID_BASE <= fid < ROAD_FRAME_ID_BASE:
            img_disp = process_cam_frame(img, is_event=True, label="CRASH")
            if crash_collecting_after:
                crash_event_after.append(img_disp)
                crash_event_raw_a.append(jpeg)
            else:
                crash_event_before.append(img_disp)
                crash_event_raw_b.append(jpeg)
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except: pass
            frame_queue.put_nowait(img_disp)
            print(f"  📥 車禍幀 {'後' if crash_collecting_after else '前'}{len(crash_event_after if crash_collecting_after else crash_event_before)} 收到")
            _frame_buf = {}
            return

        # ── 路面事件幀（frame_id >= ROAD_FRAME_ID_BASE）──────
        if fid >= ROAD_FRAME_ID_BASE:
            img = process_cam_frame(img, is_event=True)
            road_event_frames.append(img)
            road_event_raw.append(jpeg)
            road_playback_idx = len(road_event_frames) - 1
            # 放進 frame_queue 顯示
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except: pass
            frame_queue.put_nowait(img)
            print(f"  📥 事件幀 {len(road_event_frames)} 收到（{len(jpeg)} bytes）")

        # ── 一般串流幀 ─────────────────────────────────────
        else:
            frame_count += 1
            img = process_cam_frame(img, is_event=False)
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except: pass
            frame_queue.put_nowait(img)

def _parse_crash_imu(payload: bytes):
    """
    解析車禍 IMU 波形（100筆 × 7通道）
    通道順序：ax, ay, az, gx, gy, gz, mag
    """
    global crash_imu_data
    import struct as _struct
    expected = CRASH_IMU_WINDOW * 7 * 4
    if len(payload) < expected:
        print(f"⚠️  車禍 IMU 長度不足：{len(payload)} < {expected}")
        return
    floats = _struct.unpack_from(f"<{CRASH_IMU_WINDOW*7}f", payload)
    N = CRASH_IMU_WINDOW
    crash_imu_data = {
        "ax": floats[0:N],
        "ay": floats[N:N*2],
        "az": floats[N*2:N*3],
        "gx": floats[N*3:N*4],
        "gy": floats[N*4:N*5],
        "gz": floats[N*5:N*6],
        "mg": floats[N*6:N*7],
    }
    print(f"📊 車禍 IMU 波形收到（{CRASH_IMU_WINDOW} 筆）")


def _save_crash_event():
    """儲存車禍事件：圖像 + IMU CSV + info.txt"""
    if not crash_save_dir: return

    # 儲存前幀
    for i, raw in enumerate(crash_event_raw_b):
        with open(os.path.join(crash_save_dir, f"before_{i:02d}.jpg"), "wb") as f:
            f.write(raw)
    # 儲存後幀
    for i, raw in enumerate(crash_event_raw_a):
        with open(os.path.join(crash_save_dir, f"after_{i:02d}.jpg"), "wb") as f:
            f.write(raw)

    # 儲存 IMU CSV
    if crash_imu_data:
        csv_path = os.path.join(crash_save_dir, "crash_imu.csv")
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("ax,ay,az,gx,gy,gz,magnitude\n")
            N = CRASH_IMU_WINDOW
            for i in range(N):
                f.write(f"{crash_imu_data['ax'][i]},"
                        f"{crash_imu_data['ay'][i]},"
                        f"{crash_imu_data['az'][i]},"
                        f"{crash_imu_data['gx'][i]},"
                        f"{crash_imu_data['gy'][i]},"
                        f"{crash_imu_data['gz'][i]},"
                        f"{crash_imu_data['mg'][i]}\n")
        print(f"📊 車禍 IMU CSV 已存：{csv_path}")

    # 儲存 info.txt
    try:    ay_val   = abs(float(sensor_data.get("accY","0")))
    except: ay_val   = 0.0
    try:    gyrX_val = abs(float(sensor_data.get("gyrX","0")))
    except: gyrX_val = 0.0
    try:    mag_val  = float(sensor_data.get("mag","1.0"))
    except: mag_val  = 1.0

    # 計算 IMU 波形的撞擊峰值（如果有資料）
    peak_mg = max(crash_imu_data["mg"]) if crash_imu_data else 0.0
    peak_ay = max(abs(v) for v in crash_imu_data["ay"]) if crash_imu_data else 0.0
    peak_gx = max(abs(v) for v in crash_imu_data["gx"]) if crash_imu_data else 0.0

    with open(os.path.join(crash_save_dir, "info.txt"), "w", encoding="utf-8") as f:
        f.write("=" * 40 + "\n")
        f.write("  CRASH EVENT LOG\n")
        f.write("=" * 40 + "\n\n")
        f.write("[Event]\n")
        f.write(f"  type        : CRASH\n")
        f.write(f"  timestamp   : {datetime.now().isoformat()}\n")
        f.write(f"  before_frames : {len(crash_event_raw_b)}\n")
        f.write(f"  after_frames  : {len(crash_event_raw_a)}\n\n")
        f.write("[IMU Peak Values (from waveform)]\n")
        f.write(f"  peak magnitude : {peak_mg:.3f} g\n")
        f.write(f"  peak |ay|      : {peak_ay:.3f} g  (C1 lateral)\n")
        f.write(f"  peak |gyrX|    : {peak_gx:.2f} dps (C2 roll rate)\n\n")
        f.write("[Fall Conditions at Trigger]\n")
        f.write(f"  C1 |ay|   : {ay_val:.3f} g    (>1.5g)  {'HIT' if ay_val>1.5 else 'ok'}\n")
        f.write(f"  C2 |gyrX| : {gyrX_val:.2f} dps (>100)   {'HIT' if gyrX_val>100 else 'ok'}\n")
        f.write(f"  C3 mag    : {mag_val:.3f} g    (<0.5g)  {'HIT' if mag_val<0.5 else 'ok'}\n\n")
        f.write("[Sensor at Trigger]\n")
        f.write(f"  accX : {sensor_data.get('accX','--')} g\n")
        f.write(f"  accY : {sensor_data.get('accY','--')} g\n")
        f.write(f"  accZ : {sensor_data.get('accZ','--')} g\n")
        f.write(f"  gyrX : {sensor_data.get('gyrX','--')} dps\n")
        f.write(f"  gyrY : {sensor_data.get('gyrY','--')} dps\n")
        f.write(f"  gyrZ : {sensor_data.get('gyrZ','--')} dps\n")
        f.write(f"  temp : {sensor_data.get('temp','--')} C\n")
        f.write(f"  humi : {sensor_data.get('humi','--')} %\n")

    print(f"💾 車禍事件已儲存：{crash_save_dir}")


def _parse_imu_snapshot(payload: bytes):
    """
    解析 ESP32 傳來的 IMU KNN window 快照
    格式：KNN_WINDOW * 3 floats（z, x, y 各 30 筆）little-endian
    存成 CSV：acc_z, acc_x, acc_y
    """
    import struct as _struct
    expected = KNN_WINDOW * 3 * 4  # 30 * 3 * 4 = 360 bytes
    if len(payload) < expected:
        print(f"⚠️  IMU 快照長度不足：{len(payload)} < {expected}")
        return

    floats = _struct.unpack_from(f"<{KNN_WINDOW*3}f", payload)
    z_vals = floats[0          : KNN_WINDOW]
    x_vals = floats[KNN_WINDOW : KNN_WINDOW*2]
    y_vals = floats[KNN_WINDOW*2 : KNN_WINDOW*3]

    # 存到事件資料夾
    if not road_event_save_dir:
        print("⚠️  IMU 快照收到但尚無事件資料夾")
        return

    csv_path = os.path.join(road_event_save_dir, "imu_window.csv")
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("acc_z,acc_x,acc_y\n")
        for i in range(KNN_WINDOW):
            f.write(f"{z_vals[i]},{x_vals[i]},{y_vals[i]}\n")

    print(f"📊 IMU 快照已存：{csv_path}（{KNN_WINDOW} 筆）")


def process_cam_frame(img, is_event=False, label=""):
    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    img = cv2.flip(img, 1)
    h,w = img.shape[:2]
    if is_event:
        if label == "CRASH":
            # 車禍幀：橘色框
            phase = "AFTER" if crash_collecting_after else "BEFORE"
            cv2.rectangle(img,(0,0),(w-1,h-1),(0,120,255),3)
            cv2.putText(img,f"CRASH {phase}",(8,20),FONT,0.55,(0,120,255),2,cv2.LINE_AA)
        else:
            # 路面事件幀：紅框
            cv2.rectangle(img,(0,0),(w-1,h-1),(50,50,230),3)
            col = CLASS_COLORS.get(road_event_type_str,(200,200,200))
            cv2.putText(img,f"EVENT: {road_event_type_str}",(8,20),FONT,0.5,col,2,cv2.LINE_AA)
            cv2.putText(img,f"Frame {len(road_event_frames)}/10",(8,h-10),FONT,0.4,(255,255,255),1,cv2.LINE_AA)
    else:
        cv2.putText(img,f"LIVE #{frame_count}",(8,h-10),FONT,0.4,(80,220,80),1,cv2.LINE_AA)
    return img

def sensor_cb(sender, data: bytearray):
    global connection_status, last_sensor_time
    raw = data.decode("utf-8", errors="replace").strip()
    print(f"🌡  {raw}")
    parse_sensor(raw)
    last_sensor_time = time.time(); connection_status = "CONNECTED"

# ── 折線圖 ────────────────────────────────────────
PW_PLOT = 700; PH_PLOT = 480; BAR_H = 28

def draw_line(panel, buf, y_range, color, label, h_per_sig, row_y):
    data = np.array(buf, dtype=float)
    W    = panel.shape[1] - 80
    xs   = np.linspace(40, 40+W, len(data)).astype(int)
    if y_range > 0:
        ys = (row_y + h_per_sig//2 - (data/y_range)*(h_per_sig//2-4)).astype(int)
    else:
        ys = np.full(len(data), row_y+h_per_sig//2, dtype=int)
    ys = np.clip(ys, row_y+2, row_y+h_per_sig-2)
    for i in range(1, len(xs)):
        cv2.line(panel,(xs[i-1],ys[i-1]),(xs[i],ys[i]),color,1,cv2.LINE_AA)
    cv2.putText(panel,f"{label}:{data[-1]:+.2f}",(2,row_y+14),FONT,0.32,color,1,cv2.LINE_AA)
    cv2.line(panel,(40,row_y+h_per_sig//2),(40+W,row_y+h_per_sig//2),(40,40,40),1)

def draw_plot_panel() -> np.ndarray:
    panel = np.full((PH_PLOT,PW_PLOT,3),(15,15,15),dtype=np.uint8)
    sc = (50,220,80) if connection_status=="CONNECTED" else (60,60,220)
    panel[:BAR_H]=(25,25,25)
    cv2.circle(panel,(12,BAR_H//2),5,sc,-1,cv2.LINE_AA)
    cv2.putText(panel,connection_status,(24,BAR_H//2+5),FONT,0.44,sc,1,cv2.LINE_AA)
    knn_col = CLASS_COLORS.get(knn_result,(180,180,180))
    knn_txt = f"Road: {knn_result}  ({knn_confidence:.0%})"
    cv2.putText(panel,knn_txt,(PW_PLOT//2-120,BAR_H//2+5),FONT,0.5,knn_col,2,cv2.LINE_AA)
    cv2.line(panel,(0,BAR_H-1),(PW_PLOT,BAR_H-1),(55,55,55),1)
    y0=BAR_H+4; avail=PH_PLOT-y0
    rows=[
        ("ACCEL (g)",  [(buf_accX,(0,180,255),"X"),(buf_accY,(0,255,120),"Y"),(buf_accZ,(255,120,0),"Z")],3.0),
        ("GYRO (dps)", [(buf_gyrX,(180,100,255),"X"),(buf_gyrY,(100,255,180),"Y"),(buf_gyrZ,(255,180,100),"Z")],100.0),
        ("MAG (g)",    [(buf_mag,(255,220,50),"M")],3.0),
        ("TILT (deg)", [(buf_pitc,(100,220,255),"P"),(buf_roll,(255,100,220),"R")],45.0),
    ]
    h_sec = avail//len(rows)
    for ri,(title,signals,y_range) in enumerate(rows):
        sy=y0+ri*h_sec
        cv2.rectangle(panel,(0,sy),(PW_PLOT,sy+h_sec-2),(20,20,20),-1)
        cv2.putText(panel,title,(PW_PLOT-120,sy+12),FONT,0.35,(100,100,100),1,cv2.LINE_AA)
        cv2.line(panel,(0,sy),(PW_PLOT,sy),(40,40,40),1)
        h_sig=(h_sec-4)//len(signals)
        for si,(buf,color,label) in enumerate(signals):
            draw_line(panel,buf,y_range,color,label,h_sig,sy+2+si*h_sig)
    return panel

# ── 感測器面板 ────────────────────────────────────
PW_S, PH_S = 340, 820

def state_color(s):
    return {"CRASH":(50,50,230),"IMPACT":(30,180,230)}.get(s,(80,200,80))

def draw_sensor_panel() -> np.ndarray:
    panel = np.full((PH_S,PW_S,3),(18,18,18),dtype=np.uint8)
    sc=(50,220,80) if connection_status=="CONNECTED" else (60,60,220)
    panel[:BAR_H]=(25,25,25)
    cv2.circle(panel,(12,BAR_H//2),5,sc,-1,cv2.LINE_AA)
    cv2.putText(panel,connection_status,(24,BAR_H//2+5),FONT,0.44,sc,1,cv2.LINE_AA)
    # Road event 狀態列
    if road_event_active or road_playback_done:
        col = CLASS_COLORS.get(road_event_type_str,(200,200,200))
        status = "PLAYING..." if not road_playback_done else f"SAVED: {road_event_type_str}"
        cv2.putText(panel, f"ROAD EVENT: {status}", (100,BAR_H//2+5),
                    FONT, 0.38, col, 1, cv2.LINE_AA)
    cv2.line(panel,(0,BAR_H-1),(PW_S,BAR_H-1),(55,55,55),1)

    cw=(220,220,220); cy2=(50,210,255); cc=(0,230,200)
    def lbl(t,x,y,c=cy2,s=0.38): cv2.putText(panel,t,(x,y),FONT,s,c,1,cv2.LINE_AA)
    def val(t,x,y,c=cw,s=0.52):  cv2.putText(panel,t,(x,y),FONT,s,c,2,cv2.LINE_AA)
    def div(y): cv2.line(panel,(10,y),(PW_S-10,y),(50,50,50),1)

    VX=180; y=BAR_H+20
    cv2.putText(panel,"ESP32-S3 SENSOR",(PW_S//2-90,y),FONT,0.55,cc,2,cv2.LINE_AA)
    y+=6; div(y)
    y+=22; lbl("TEMP",14,y);     val(f"{sensor_data['temp']} C",VX,y)
    y+=28; lbl("HUMIDITY",14,y); val(f"{sensor_data['humi']} %",VX,y)
    y+=10; div(y)
    y+=18; lbl("ACCEL (g)",14,y)
    for k,ax in (("accX","X"),("accY","Y"),("accZ","Z")):
        y+=22; lbl(f"  {ax}:",14,y,cw,0.40); val(sensor_data[k],VX,y,cw,0.46)
    y+=10; div(y)
    y+=18; lbl("GYRO (dps)",14,y)
    for k,ax in (("gyrX","X"),("gyrY","Y"),("gyrZ","Z")):
        y+=22; lbl(f"  {ax}:",14,y,cw,0.40); val(sensor_data[k],VX,y,cw,0.46)
    y+=10; div(y)
    y+=18; cv2.putText(panel,"CRASH DETECTION",(14,y),FONT,0.46,cc,2,cv2.LINE_AA)
    y+=26; lbl("MAGNITUDE",14,y); val(f"{sensor_data['mag']} g",VX,y)
    y+=28; lbl("STATUS",14,y)
    val(sensor_data["state"],VX,y,state_color(sensor_data["state"]))
    y+=28; lbl("QUIET MODE",14,y)
    qc=(80,80,220) if sensor_data["quiet"]=="1" else (80,200,80)
    val("ON" if sensor_data["quiet"]=="1" else "OFF",VX,y,qc)
    y+=20; div(y)

    # KNN
    y+=18; cv2.putText(panel,"KNN ROAD TYPE (ESP32)",(14,y),FONT,0.40,cc,2,cv2.LINE_AA)
    classes_sorted = sorted(knn_proba.keys(), key=lambda k: knn_proba.get(k,0), reverse=True)
    BAR_W = PW_S-30
    for cls in classes_sorted:
        y+=22
        pct=knn_proba.get(cls,0.0); col=CLASS_COLORS.get(cls,(150,150,150))
        is_win=(cls==knn_result)
        lbl(cls,14,y,col if is_win else (120,120,120),0.35)
        cv2.putText(panel,f"{pct:.0%}",(PW_S-42,y),FONT,0.38,
                    col if is_win else (120,120,120),1,cv2.LINE_AA)
        bar_y=y+4
        cv2.rectangle(panel,(14,bar_y),(14+BAR_W,bar_y+8),(40,40,40),-1)
        fill=int(BAR_W*pct)
        if fill>0: cv2.rectangle(panel,(14,bar_y),(14+fill,bar_y+8),col,-1)
        if is_win: cv2.rectangle(panel,(14,bar_y),(14+BAR_W,bar_y+8),col,1)
    y+=20; div(y)

    # C1 / C2 / C3
    y+=18; cv2.putText(panel,"FALL CONDITIONS",(14,y),FONT,0.46,cc,2,cv2.LINE_AA)
    try:    ay_val   = abs(float(sensor_data.get("accY","0")))
    except: ay_val   = 0.0
    try:    gyrX_val = abs(float(sensor_data.get("gyrX","0")))
    except: gyrX_val = 0.0
    try:    mag_val  = float(sensor_data.get("mag","1.0"))
    except: mag_val  = 1.0

    c1_hit=ay_val>1.5; c2_hit=gyrX_val>100.0; c3_hit=mag_val<0.5
    c3_bar_val=max(0.0, 2.0-mag_val); c3_bar_max=2.0
    conditions=[
        ("C1  |ay|",   ay_val,   1.5,   c1_hit,"g",  ay_val,    3.0,         False,(30,180,230)),
        ("C2  |gyrX|", gyrX_val, 100.0, c2_hit,"dps",gyrX_val,  200.0,       False,(180,100,255)),
        ("C3  mag",    mag_val,  0.5,   c3_hit,"g",  c3_bar_val, c3_bar_max,  True, (80,200,80)),
    ]
    BAR_W2=PW_S-30; THR_OK=(80,200,80); THR_HIT=(50,50,230)
    for label,cur,thr,hit,unit,bar_val,maxv,inverted,bar_col in conditions:
        y+=24
        hit_col=THR_HIT if hit else THR_OK
        cv2.putText(panel,label,(14,y),FONT,0.38,hit_col,1,cv2.LINE_AA)
        cv2.putText(panel,f"{cur:.2f} {unit}",(PW_S-90,y),FONT,0.42,
                    THR_HIT if hit else cw, 1 if not hit else 2,cv2.LINE_AA)
        bar_y=y+4
        cv2.rectangle(panel,(14,bar_y),(14+BAR_W2,bar_y+10),(35,35,35),-1)
        fill_ratio=min(bar_val/maxv,1.0) if maxv>0 else 0.0
        fill_w=int(BAR_W2*fill_ratio)
        if fill_w>0:
            cv2.rectangle(panel,(14,bar_y),(14+fill_w,bar_y+10),THR_HIT if hit else bar_col,-1)
        thr_ratio=(maxv-thr)/maxv if inverted else thr/maxv
        thr_x=min(14+int(BAR_W2*thr_ratio),14+BAR_W2-1)
        cv2.line(panel,(thr_x,bar_y-2),(thr_x,bar_y+12),(255,255,255),1)
        thr_lbl=f"<{thr}{unit}" if inverted else f">{thr}{unit}"
        cv2.putText(panel,thr_lbl,(thr_x-2,bar_y-4),FONT,0.28,(180,180,180),1,cv2.LINE_AA)
        y+=14

    y+=10
    all_hit=c1_hit and c2_hit and c3_hit
    if all_hit:
        status_txt="!! ALL CONDITIONS MET !!"; status_col=(50,50,230)
    elif c1_hit or c2_hit or c3_hit:
        active=[x for x,h in[("C1",c1_hit),("C2",c2_hit),("C3",c3_hit)] if h]
        status_txt=f"Active: {' '.join(active)}"; status_col=(30,180,230)
    else:
        status_txt="All clear"; status_col=(80,200,80)
    cv2.putText(panel,status_txt,(14,y),FONT,0.38,status_col,2 if all_hit else 1,cv2.LINE_AA)
    return panel

# ── 相機視窗（含事件回放）────────────────────────
PLAYBACK_INTERVAL = 0.2   # 200ms / 幀
_last_playback_time = 0.0

def get_display_frame(latest_live):
    """
    回放優先順序：車禍 > 路面事件 > 即時串流
    播完停在最後一幀，下次觸發才換下一組
    """
    global road_playback_idx, crash_playback_idx
    global _last_playback_time, crash_playback_done

    now = time.time()

    # 車禍回放
    if crash_playback_frames:
        if not crash_playback_done and crash_playback_idx < len(crash_playback_frames)-1:
            if now - _last_playback_time >= PLAYBACK_INTERVAL:
                crash_playback_idx += 1
                _last_playback_time = now
        if crash_playback_idx >= len(crash_playback_frames)-1:
            crash_playback_done = True
        return crash_playback_frames[crash_playback_idx]

    # 路面事件回放
    if road_event_frames:
        if not road_playback_done and road_playback_idx < len(road_event_frames)-1:
            if now - _last_playback_time >= PLAYBACK_INTERVAL:
                road_playback_idx += 1
                _last_playback_time = now
        return road_event_frames[road_playback_idx]

    return latest_live

# ── BLE 執行緒 ────────────────────────────────────
async def ble_loop():
    global has_camera,connection_status,last_sensor_time,_frame_buf,_cur_frame,_total_chunks
    while not stop_event.is_set():
        print("🔍 掃描中...")
        try: device=await BleakScanner.find_device_by_name(DEVICE_NAME,timeout=10)
        except Exception as e: print(f"⚠️  {e}"); device=None
        if device is None:
            connection_status="DISCONNECTED"; await asyncio.sleep(RECONNECT_INTERVAL); continue
        _frame_buf={}; _cur_frame=-1; _total_chunks=0
        def on_disconnect(c):
            global connection_status
            connection_status="DISCONNECTED"; print("⚠️  BLE斷線")
        try:
            async with BleakClient(device,disconnected_callback=on_disconnect) as client:
                connection_status="CONNECTED"; last_sensor_time=time.time()
                await client.start_notify(CHAR_SENSOR_UUID,sensor_cb)
                try:
                    await client.start_notify(CHAR_DATA_UUID,image_cb)
                    await client.write_gatt_char(CHAR_CTRL_UUID,b"START")
                    has_camera=True; print("📷 相機模式")
                except: has_camera=False; print("📊 純感測器模式")
                while not stop_event.is_set():
                    await asyncio.sleep(0.05)
                    if time.time()-last_sensor_time>SENSOR_TIMEOUT and connection_status=="CONNECTED":
                        connection_status="DISCONNECTED"
                    if not client.is_connected: break
                if has_camera:
                    try: await client.write_gatt_char(CHAR_CTRL_UUID,b"STOP"); await client.stop_notify(CHAR_DATA_UUID)
                    except: pass
                try: await client.stop_notify(CHAR_SENSOR_UUID)
                except: pass
        except Exception as e: print(f"⚠️  {e}"); connection_status="DISCONNECTED"
        if stop_event.is_set(): break
        await asyncio.sleep(RECONNECT_INTERVAL)

def ble_thread_entry():
    loop=asyncio.new_event_loop(); asyncio.set_event_loop(loop)
    try: loop.run_until_complete(ble_loop())
    finally: loop.close()

# ── 主程式 ────────────────────────────────────────
def main():
    os.makedirs(SAVE_ROOT, exist_ok=True)
    t=threading.Thread(target=ble_thread_entry,daemon=True); t.start()

    WIN_SENSOR="ESP32-S3 SENSOR"
    WIN_CAM   ="ESP32-S3 CAM"
    WIN_PLOT  ="ESP32-S3 PLOT + KNN"
    cv2.namedWindow(WIN_SENSOR,cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(WIN_CAM,   cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(WIN_PLOT,  cv2.WINDOW_AUTOSIZE)

    placeholder=np.full((240,320,3),(30,30,30),dtype=np.uint8)
    cv2.putText(placeholder,"Waiting...",(60,120),FONT,0.5,(100,100,100),1,cv2.LINE_AA)

    latest_live = placeholder
    print(f"💾 儲存路徑：{SAVE_ROOT}")
    print("按 Q 結束\n")

    while True:
        cv2.imshow(WIN_SENSOR, draw_sensor_panel())
        cv2.imshow(WIN_PLOT,   draw_plot_panel())

        try:
            frame = frame_queue.get_nowait()
            if road_event_frames and frame is road_event_frames[-1]:
                pass  # 事件幀已在 get_display_frame 管理
            else:
                latest_live = frame
        except queue.Empty:
            pass

        display = get_display_frame(latest_live)
        cv2.imshow(WIN_CAM, display)

        if cv2.waitKey(50)&0xFF==ord('q'): break

    stop_event.set(); t.join(timeout=5); cv2.destroyAllWindows()
    print(f"✅ 結束（共接收 {frame_count} 幀）")

if __name__=="__main__":
    main()
