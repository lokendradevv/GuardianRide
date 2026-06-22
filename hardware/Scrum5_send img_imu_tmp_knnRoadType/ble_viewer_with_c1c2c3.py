#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ble_viewer_with_c1c2c3.py
ESP32-S3 BLE CAM + Sensor Viewer
KNN 結果完全從 ESP32 的 R: 和 P: 欄位讀取，不在電腦端推論
新增：C1 / C2 / C3 摔車條件即時顯示（Boubezoul 2013）
"""

import asyncio, struct, threading, time, queue, os
import cv2, numpy as np
from collections import deque
from bleak import BleakClient, BleakScanner

# ── BLE 設定 ──────────────────────────────────────
SERVICE_UUID     = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_DATA_UUID   = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHAR_CTRL_UUID   = "beb5483e-36e1-4688-b7f5-ea07361b26a9"
CHAR_SENSOR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26aa"
DEVICE_NAME      = "ESP32-S3-CAM"
SENSOR_TIMEOUT   = 3.0
RECONNECT_INTERVAL = 3.0

# KNN 類別順序
CLASS_ORDER = ["asphalt_bumps", "pothole", "regular_road", "worn_out_road"]
CLASS_COLORS = {
    "regular_road":  (80,  200, 80),
    "worn_out_road": (80,  160, 80),
    "asphalt_bumps": (30,  180, 230),
    "pothole":       (50,  50,  230),
}

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

# KNN 結果（從 ESP32 讀取）
knn_result     = "warming_up"
knn_proba      = {c: 0.0 for c in CLASS_ORDER}
knn_confidence = 0.0

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

# ── 感測器解析 ────────────────────────────────────
def parse_sensor(raw: str):
    global knn_result, knn_proba, knn_confidence

    if raw in ("CRASH_DETECTED","SAFE","QUIET_ON","QUIET_OFF","RESET"):
        if raw == "CRASH_DETECTED": sensor_data["state"] = "CRASH"
        elif raw == "SAFE":         sensor_data["state"] = "OK"; sensor_data["quiet"] = "1"
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
                if len(v) == 3:
                    sensor_data["accX"],sensor_data["accY"],sensor_data["accZ"] = v
                    ax,ay,az = float(v[0]),float(v[1]),float(v[2])
                    buf_accX.append(ax); buf_accY.append(ay); buf_accZ.append(az)
                    buf_mag.append((ax**2+ay**2+az**2)**0.5)
                    buf_pitc.append(np.degrees(np.arctan2(-ax, np.sqrt(ay**2+az**2))))
                    buf_roll.append(np.degrees(np.arctan2(ay, az)))
            elif p.startswith("G:"):
                v = p[2:].split(",")
                if len(v) == 3:
                    sensor_data["gyrX"],sensor_data["gyrY"],sensor_data["gyrZ"] = v
                    buf_gyrX.append(float(v[0]))
                    buf_gyrY.append(float(v[1]))
                    buf_gyrZ.append(float(v[2]))
    except Exception as e:
        print(f"⚠️  解析錯誤: {e} | raw='{raw}'")

# ── 折線圖 ────────────────────────────────────────
PW_PLOT = 700
PH_PLOT = 480
BAR_H   = 28

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
    y0    = BAR_H+4
    avail = PH_PLOT-y0
    rows  = [
        ("ACCEL (g)",  [(buf_accX,(0,180,255),"X"),(buf_accY,(0,255,120),"Y"),(buf_accZ,(255,120,0),"Z")],3.0),
        ("GYRO (dps)", [(buf_gyrX,(180,100,255),"X"),(buf_gyrY,(100,255,180),"Y"),(buf_gyrZ,(255,180,100),"Z")],100.0),
        ("MAG (g)",    [(buf_mag,(255,220,50),"M")],3.0),
        ("TILT (deg)", [(buf_pitc,(100,220,255),"P"),(buf_roll,(255,100,220),"R")],45.0),
    ]
    h_sec = avail // len(rows)
    for ri,(title,signals,y_range) in enumerate(rows):
        sy = y0+ri*h_sec
        cv2.rectangle(panel,(0,sy),(PW_PLOT,sy+h_sec-2),(20,20,20),-1)
        cv2.putText(panel,title,(PW_PLOT-120,sy+12),FONT,0.35,(100,100,100),1,cv2.LINE_AA)
        cv2.line(panel,(0,sy),(PW_PLOT,sy),(40,40,40),1)
        h_sig = (h_sec-4)//len(signals)
        for si,(buf,color,label) in enumerate(signals):
            draw_line(panel,buf,y_range,color,label,h_sig,sy+2+si*h_sig)
    return panel

# ── 感測器數值視窗 ────────────────────────────────
PW_S, PH_S = 340, 820   # 高度加大以容納 C1 C2 C3 區塊

def state_color(s):
    return {"CRASH":(50,50,230),"IMPACT":(30,180,230)}.get(s,(80,200,80))

def draw_sensor_panel() -> np.ndarray:
    panel = np.full((PH_S,PW_S,3),(18,18,18),dtype=np.uint8)
    sc = (50,220,80) if connection_status=="CONNECTED" else (60,60,220)
    panel[:BAR_H]=(25,25,25)
    cv2.circle(panel,(12,BAR_H//2),5,sc,-1,cv2.LINE_AA)
    cv2.putText(panel,connection_status,(24,BAR_H//2+5),FONT,0.44,sc,1,cv2.LINE_AA)
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

    # ── KNN（完全來自 ESP32）────────────────────
    y+=18; cv2.putText(panel,"KNN ROAD TYPE (ESP32)",(14,y),FONT,0.40,cc,2,cv2.LINE_AA)
    classes_sorted = sorted(knn_proba.keys(), key=lambda k: knn_proba.get(k,0), reverse=True)
    BAR_W = PW_S - 30
    for cls in classes_sorted:
        y += 22
        pct    = knn_proba.get(cls, 0.0)
        col    = CLASS_COLORS.get(cls,(150,150,150))
        is_win = (cls == knn_result)
        lbl(cls, 14, y, col if is_win else (120,120,120), 0.35)
        cv2.putText(panel,f"{pct:.0%}",(PW_S-42,y),FONT,0.38,
                    col if is_win else (120,120,120),1,cv2.LINE_AA)
        bar_y = y+4
        cv2.rectangle(panel,(14,bar_y),(14+BAR_W,bar_y+8),(40,40,40),-1)
        fill = int(BAR_W*pct)
        if fill > 0:
            cv2.rectangle(panel,(14,bar_y),(14+fill,bar_y+8),col,-1)
        if is_win:
            cv2.rectangle(panel,(14,bar_y),(14+BAR_W,bar_y+8),col,1)

    y+=20; div(y)

    # ── C1 / C2 / C3 摔車條件（Boubezoul 2013）──
    y+=18
    cv2.putText(panel,"FALL CONDITIONS",(14,y),FONT,0.46,cc,2,cv2.LINE_AA)

    # 安全讀取數值
    try:    ay_val   = abs(float(sensor_data.get("accY","0")))
    except: ay_val   = 0.0
    try:    gyrX_val = abs(float(sensor_data.get("gyrX","0")))
    except: gyrX_val = 0.0
    try:    mag_val  = float(sensor_data.get("mag","1.0"))
    except: mag_val  = 1.0

    c1_hit = ay_val   > 1.5
    c2_hit = gyrX_val > 100.0
    c3_hit = mag_val  < 0.5

    # C3 進度條用「反向值」：mag 越低 → 條越長 → 越危險
    # 顯示 max(0, 2.0 - mag_val) 讓 0g 時條滿格，2g 時條空
    c3_bar_val = max(0.0, 2.0 - mag_val)   # 反向：低 mag = 長條
    c3_bar_max = 2.0                         # 對應 mag=0g 時滿格

    # 三條件各佔一列
    # (標籤, 顯示值, 門檻, 是否超標, 單位, bar用值, bar最大值, 是否反向bar, 顏色)
    conditions = [
        ("C1  |ay|",   ay_val,   1.5,   c1_hit, "g",   ay_val,   3.0,         False, (30,  180, 230)),
        ("C2  |gyrX|", gyrX_val, 100.0, c2_hit, "dps", gyrX_val, 200.0,       False, (180, 100, 255)),
        ("C3  mag",    mag_val,  0.5,   c3_hit, "g",   c3_bar_val, c3_bar_max, True,  (80,  200, 80)),
    ]

    BAR_W2 = PW_S - 30
    THR_COLOR_OK  = (80, 200, 80)   # 未超標：綠
    THR_COLOR_HIT = (50,  50, 230)  # 超標：紅

    for label, cur, thr, hit, unit, bar_val, maxv, inverted, bar_col in conditions:
        y += 24

        # 標籤
        hit_col = THR_COLOR_HIT if hit else THR_COLOR_OK
        cv2.putText(panel, label, (14, y), FONT, 0.38, hit_col, 1, cv2.LINE_AA)

        # 數值靠右
        val_str = f"{cur:.2f} {unit}"
        cv2.putText(panel, val_str, (PW_S-90, y), FONT, 0.42,
                    THR_COLOR_HIT if hit else cw, 1 if not hit else 2, cv2.LINE_AA)

        # 進度條背景
        bar_y = y + 4
        cv2.rectangle(panel,(14,bar_y),(14+BAR_W2,bar_y+10),(35,35,35),-1)

        fill_ratio = min(bar_val / maxv, 1.0) if maxv > 0 else 0.0
        fill_w = int(BAR_W2 * fill_ratio)
        if fill_w > 0:
            cv2.rectangle(panel,(14,bar_y),(14+fill_w,bar_y+10),
                          THR_COLOR_HIT if hit else bar_col,-1)

        # 門檻線位置
        # C1/C2 正向：門檻在 thr/maxv 處
        # C3 反向：門檻在 (2.0-0.5)/2.0 = 75% 處（mag=0.5 時 bar_val=1.5）
        if inverted:
            thr_ratio = (maxv - thr) / maxv   # mag=0.5 → bar_val=1.5 → 1.5/2.0=75%
        else:
            thr_ratio = thr / maxv
        thr_x = 14 + int(BAR_W2 * thr_ratio)
        thr_x = min(thr_x, 14+BAR_W2-1)
        cv2.line(panel,(thr_x,bar_y-2),(thr_x,bar_y+12),(255,255,255),1)

        # 門檻標籤
        thr_lbl = f"<{thr}{unit}" if inverted else f">{thr}{unit}"
        cv2.putText(panel, thr_lbl, (thr_x-2, bar_y-4),
                    FONT, 0.28, (180,180,180), 1, cv2.LINE_AA)

        y += 14

    # 整體狀態提示
    y += 10
    all_hit = c1_hit and c2_hit and c3_hit
    status_txt = "!! ALL CONDITIONS MET !!" if all_hit else \
                 f"Conditions: {'C1 ' if c1_hit else ''}{'C2 ' if c2_hit else ''}{'C3 ' if c3_hit else ''}active" \
                 if (c1_hit or c2_hit or c3_hit) else "All clear"
    status_col = (50,50,230) if all_hit else ((30,180,230) if (c1_hit or c2_hit or c3_hit) else (80,200,80))
    cv2.putText(panel, status_txt, (14, y), FONT, 0.38, status_col, 2 if all_hit else 1, cv2.LINE_AA)

    return panel

# ── 相機視窗 ──────────────────────────────────────
def make_cam_frame(img):
    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    img = cv2.flip(img, 1)
    h,w = img.shape[:2]
    cv2.putText(img,f"Frame #{frame_count}",(8,h-10),FONT,0.4,(80,220,80),1,cv2.LINE_AA)
    return img

# ── BLE 回呼 ──────────────────────────────────────
_frame_buf={}; _cur_frame=-1; _total_chunks=0

def image_cb(sender, data: bytearray):
    global _frame_buf,_cur_frame,_total_chunks,frame_count
    if len(data)<6: return
    fid=struct.unpack(">H",data[0:2])[0]; cid=struct.unpack(">H",data[2:4])[0]
    total=struct.unpack(">H",data[4:6])[0]; chunk=data[6:]
    if fid!=_cur_frame: _frame_buf={}; _cur_frame=fid; _total_chunks=total
    _frame_buf[cid]=chunk
    if len(_frame_buf)==_total_chunks:
        jpeg=b"".join(_frame_buf[i] for i in range(_total_chunks))
        arr=np.frombuffer(jpeg,dtype=np.uint8)
        img=cv2.imdecode(arr,cv2.IMREAD_COLOR)
        if img is not None:
            frame_count+=1
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except: pass
            frame_queue.put_nowait(make_cam_frame(img))
        _frame_buf={}

def sensor_cb(sender, data: bytearray):
    global connection_status,last_sensor_time
    raw=data.decode("utf-8",errors="replace").strip()
    print(f"🌡  {raw}")
    parse_sensor(raw)
    last_sensor_time=time.time(); connection_status="CONNECTED"

# ── BLE 執行緒 ────────────────────────────────────
async def ble_loop():
    global has_camera,connection_status,last_sensor_time,_frame_buf,_cur_frame,_total_chunks
    while not stop_event.is_set():
        print("🔍 掃描 BLE 裝置中...")
        try: device=await BleakScanner.find_device_by_name(DEVICE_NAME,timeout=10)
        except Exception as e: print(f"⚠️  {e}"); device=None
        if device is None:
            connection_status="DISCONNECTED"
            await asyncio.sleep(RECONNECT_INTERVAL); continue
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
    t=threading.Thread(target=ble_thread_entry,daemon=True); t.start()

    WIN_SENSOR = "ESP32-S3 SENSOR"
    WIN_CAM    = "ESP32-S3 CAM"
    WIN_PLOT   = "ESP32-S3 PLOT + KNN"

    cv2.namedWindow(WIN_SENSOR,cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(WIN_CAM,   cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(WIN_PLOT,  cv2.WINDOW_AUTOSIZE)

    placeholder=np.full((240,320,3),(30,30,30),dtype=np.uint8)
    cv2.putText(placeholder,"Waiting for camera...",(40,120),FONT,0.5,(100,100,100),1,cv2.LINE_AA)

    print(f"KNN class order: {CLASS_ORDER}")
    print("Press Q to quit\n")

    while True:
        cv2.imshow(WIN_SENSOR, draw_sensor_panel())
        cv2.imshow(WIN_PLOT,   draw_plot_panel())
        try:
            frame=frame_queue.get_nowait(); cv2.imshow(WIN_CAM,frame)
        except queue.Empty:
            if not has_camera: cv2.imshow(WIN_CAM,placeholder)
        if cv2.waitKey(50)&0xFF==ord('q'): break

    stop_event.set(); t.join(timeout=5); cv2.destroyAllWindows()
    print(f"Done" + (f", received {frame_count} frames" if has_camera else ""))

if __name__=="__main__":
    main()
