# -*- coding: utf-8 -*-
"""
显示 ROI 分拣区域 - 调整好框的位置后按 s 保存，坐标写入 config.json
"""

import cv2
import json
import os

CONFIG_FILE = 'config.json'
ROI_KEY = 'DETECTION_ROI'

# 默认区域（可调）
roi = {'x1': 100, 'y1': 80, 'x2': 540, 'y2': 400}

# 尝试从 config.json 加载已有 ROI
if os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
        saved = cfg.get(ROI_KEY)
        if saved:
            roi.update(saved)
            print(f"已加载保存的 ROI: {roi}")
    except Exception:
        pass

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("无法打开摄像头")
    exit(1)

print("=" * 50)
print("ROI 区域调整工具")
print("=" * 50)
print("  W/A/S/D    上/左/下/右 移动框(10px)")
print("  Arrows     微移框(1px)")
print("  +/-        放大/缩小框")
print("  k          保存 ROI 到 config.json")
print("  q          退出")
print(f"\n当前 ROI: {roi}")

selected_corner = None  # 可以用 1-4 选中某个角单独调

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头读取失败")
        break

    x1, y1, x2, y2 = roi['x1'], roi['y1'], roi['x2'], roi['y2']

    # 绘制半透明填充区域
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 255), -1)
    cv2.addWeighted(overlay, 0.12, frame, 0.88, 0, frame)

    # 边框
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # 四个角带坐标标签
    corners = [
        (x1, y1, f"({x1},{y1})"),
        (x2, y1, f"({x2},{y1})"),
        (x2, y2, f"({x2},{y2})"),
        (x1, y2, f"({x1},{y2})"),
    ]
    for cx, cy, label in corners:
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(frame, label, (cx + 6, cy - 6),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    # 区域尺寸
    w = x2 - x1
    h = y2 - y1
    cv2.putText(frame, f"{w}x{h}", (x1 + 6, y2 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)

    # 操作提示
    cv2.putText(frame, "WASD=移动  +/-=大小  k=保存  q=退出",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    cv2.imshow("ROI 分拣区域", frame)
    key = cv2.waitKey(30) & 0xFF

    step_big = 10
    step_small = 1
    resize_step = 10

    if key == ord('q'):
        break
    elif key == ord('k'):
        try:
            cfg_full = {}
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    cfg_full = json.load(f)
            cfg_full[ROI_KEY] = roi
            with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(cfg_full, f, indent=4, ensure_ascii=False)
            print(f"ROI 已保存到 {CONFIG_FILE}: {roi}")
        except Exception as e:
            print(f"保存失败: {e}")
    elif key == ord('w'):
        roi['y1'] = max(0, y1 - step_big)
        roi['y2'] = max(0, y2 - step_big)
    elif key == ord('s'):
        h_frame, w_frame = frame.shape[:2]
        roi['y1'] = min(h_frame, y1 + step_big)
        roi['y2'] = min(h_frame, y2 + step_big)
    elif key == ord('a'):
        roi['x1'] = max(0, x1 - step_big)
        roi['x2'] = max(0, x2 - step_big)
    elif key == ord('d'):
        w_frame = frame.shape[1]
        roi['x1'] = min(w_frame, x1 + step_big)
        roi['x2'] = min(w_frame, x2 + step_big)
    elif key == 82:  # 上箭头
        roi['y1'] = max(0, y1 - step_small)
        roi['y2'] = max(0, y2 - step_small)
    elif key == 84:  # 下箭头
        h_frame, w_frame = frame.shape[:2]
        roi['y1'] = min(h_frame, y1 + step_small)
        roi['y2'] = min(h_frame, y2 + step_small)
    elif key == 81:  # 左箭头
        roi['x1'] = max(0, x1 - step_small)
        roi['x2'] = max(0, x2 - step_small)
    elif key == 83:  # 右箭头
        w_frame = frame.shape[1]
        roi['x1'] = min(w_frame, x1 + step_small)
        roi['x2'] = min(w_frame, x2 + step_small)
    elif key == ord('=') or key == ord('+'):
        roi['x1'] = max(0, x1 - resize_step)
        roi['y1'] = max(0, y1 - resize_step)
        h_frame, w_frame = frame.shape[:2]
        roi['x2'] = min(w_frame, x2 + resize_step)
        roi['y2'] = min(h_frame, y2 + resize_step)
    elif key == ord('-') or key == ord('_'):
        if w > resize_step * 2 and h > resize_step * 2:
            roi['x1'] = x1 + resize_step
            roi['y1'] = y1 + resize_step
            roi['x2'] = x2 - resize_step
            roi['y2'] = y2 - resize_step

cap.release()
cv2.destroyAllWindows()
