import cv2
import numpy as np
from ultralytics import YOLO

# ── 設定 ──────────────────────────────────────────
STREAM_URL = "http://172.20.10.2/stream"
CONF       = 0.25
VEHICLE    = {2, 3, 5, 7}   # car, motorcycle, bus, truck
ROTATE     = None            # None / cv2.ROTATE_90_CLOCKWISE / cv2.ROTATE_180 / cv2.ROTATE_90_COUNTERCLOCKWISE
# ─────────────────────────────────────────────────

print("載入 YOLOv8n 模型...")
model = YOLO("yolov8n.pt")
print("模型載入完成！")

cap = cv2.VideoCapture(STREAM_URL)
if not cap.isOpened():
    print("❌ 無法連線，確認 ESP32 IP 與網路")
    exit()

print(f"\n串流連線中：{STREAM_URL}")
print("按 Q 離開\n")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️  掉幀，重試中...")
        continue

    if ROTATE is not None:
        frame = cv2.rotate(frame, ROTATE)

    h, w = frame.shape[:2]
    mid_x = w // 2

    # 畫中線
    cv2.line(frame, (mid_x, 0), (mid_x, h), (0, 255, 255), 2)
    cv2.putText(frame, "CENTER", (mid_x + 4, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # YOLO 推論
    results = model(frame, conf=CONF, verbose=False)[0]

    left_count = right_count = 0

    for box in results.boxes:
        cls_id = int(box.cls[0])
        if cls_id not in VEHICLE:
            continue

        score = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx = (x1 + x2) // 2

        is_left = cx < mid_x
        label   = model.names[cls_id]
        side    = "LEFT" if is_left else "RIGHT"
        color   = (255, 100, 0) if is_left else (0, 140, 255)

        if is_left:
            left_count += 1
        else:
            right_count += 1

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        text = f"{side} {label} {score:.0%}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (x1, y1 - 22), (x1 + tw + 6, y1), color, -1)
        cv2.putText(frame, text, (x1 + 3, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # 左右計數
    cv2.putText(frame, f"<= LEFT  {left_count}", (10, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 100, 0), 2)
    cv2.putText(frame, f"RIGHT => {right_count}", (w - 230, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 140, 255), 2)

    if left_count or right_count:
        print(f"左方來車: {left_count}  |  右方來車: {right_count}")

    cv2.imshow("Vehicle Direction Detector", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("結束")