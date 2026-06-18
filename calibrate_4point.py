# -*- coding: utf-8 -*-
"""
4 点标定工具 v1.0
在 ROI 四个角分别手动调舵机，记录(像素X, 像素Y, 底座角, 大臂角, 小臂角)
自动计算双线性插值参数并写入 config.json

操作:
  A/D       底座旋转 (-5°/+5°)
  W/S       大臂俯仰 (-5°/+5°)
  Q/E       小臂弯曲 (-5°/+5°)
  Shift+键  微调模式 (1°)
  Enter     记录当前角 -> 进入下一点
  R         重试当前角
   Esc       退出
"""

import cv2
import numpy as np
import sys
import os
import time
import json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config
from serial_comm import SerialComm


CONFIG_FILE = 'config.json'
CALIB_KEY = 'ARM_CALIBRATION'  # 在 config.json 中的顶层键

# 默认标定点顺序：左上→右上→右下→左下
def _load_roi():
    """从 config.json 加载 ROI"""
    roi = None
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
            roi = cfg.get('DETECTION_ROI')
        except Exception:
            pass
    return roi


def _save_calibration(data):
    """保存标定数据到 config.json"""
    cfg = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
    cfg[CALIB_KEY] = data
    with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
        json.dump(cfg, f, indent=4, ensure_ascii=False)
    print(f"\n- 标定数据已保存到 {CONFIG_FILE}")


def _get_corner_label(i):
    labels = ['①', '②', '③', '④']
    return labels[i] if i < len(labels) else f'[{i+1}]'


def _clamp_angle(val, lo, hi):
    return max(lo, min(hi, val))


def main():
    # ---- 加载 ROI ----
    roi = _load_roi()
    if roi is None:
        print("未找到 ROI，请先运行 show_roi.py 设定分拣区域")
        input("按 Enter 退出...")
        return

    x1, y1, x2, y2 = roi['x1'], roi['y1'], roi['x2'], roi['y2']
    corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]

    print("=" * 50)
    print("  4 点标定工具 v1.0")
    print("=" * 50)
    print(f"ROI: ({x1},{y1}) - ({x2},{y2})")
    print(f"标定点顺序: 左上→右上→右下→左下\n")

    # ---- 打开摄像头 ----
    cap = cv2.VideoCapture(config.CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    if not cap.isOpened():
        print("无法打开摄像头")
        input("按 Enter 退出...")
        return
    print("摄像头已打开")

    # ---- 连接串口 ----
    comm = SerialComm()
    port = input(f"请输入串口号 (默认 {config.SERIAL_CONFIG['port']}): ").strip()
    if not port:
        port = config.SERIAL_CONFIG['port']
    if not comm.connect(port):
        print("串口连接失败，请检查 Arduino")
        cap.release()
        input("按 Enter 退出...")
        return
    print("串口已连接\n")

    # 发送 HOME 给一个安全起点
    print("机械臂归位中...")
    comm.send_home_with_ack()
    time.sleep(2)

    # ---- 标定循环 ----
    recorded = []
    current_angle = [90, 90, 120]  # [底座, 大臂, 小臂] 与当前 init 一致
    step_size = 5
    corner_idx = 0
    running = True

    cv2.namedWindow('4点标定', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('4点标定', 800, 600)

    while running and corner_idx < 4:
        ret, frame = cap.read()
        if not ret:
            continue

        px, py = corners[corner_idx]

        overlay = frame.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), -1)
        cv2.addWeighted(overlay, 0.08, frame, 0.92, 0, frame)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

        for i, (cx, cy) in enumerate(corners):
            color = (0, 255, 255) if i == corner_idx else (100, 200, 100)
            cv2.circle(frame, (cx, cy), 6, color, -1)
            cv2.putText(frame, _get_corner_label(i), (cx + 8, cy - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        cv2.line(frame, (px - 15, py), (px + 15, py), (0, 255, 255), 2)
        cv2.line(frame, (px, py - 15), (px, py + 15), (0, 255, 255), 2)

        base_a, shoulder_a, elbow_a = current_angle
        lines = [
            f"标定点 {corner_idx+1}/4 {_get_corner_label(corner_idx)} 步长:{step_size}°",
            f"  [A/D] 底座: {base_a}°   [W/S] 大臂: {shoulder_a}°   [Q/E] 小臂: {elbow_a}°",
            f"  [+]步长增  [-]步长减  Enter=记录  R=重试  Esc=退出",
        ]
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (10, 30 + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        for i, (rcx, rcy, rb, rs, re) in enumerate(recorded):
            cv2.putText(frame, f"#{i+1} OK", (rcx - 35, rcy + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)

        cv2.imshow('4点标定', frame)
        key = cv2.waitKey(50) & 0xFF

        if key == 27:  # Esc 退出（Q 已用于小臂调整）
            running = False
            break

        # ---- 步长 ----
        if key == ord('=') or key == ord('+'):
            step_size = min(15, step_size + 1)
            continue
        if key == ord('-') or key == ord('_'):
            step_size = max(1, step_size - 1)
            continue

        # ---- 舵机单轴调整（只发变化的轴） ----
        new_base, new_shoulder, new_elbow = current_angle

        if key == ord('a'):
            new_base = _clamp_angle(base_a - step_size, 5, 175)
        elif key == ord('d'):
            new_base = _clamp_angle(base_a + step_size, 5, 175)
        elif key == ord('w'):
            new_shoulder = _clamp_angle(shoulder_a - step_size, 90, 180)
        elif key == ord('s'):
            new_shoulder = _clamp_angle(shoulder_a + step_size, 90, 180)
        elif key == ord('q'):
            new_elbow = _clamp_angle(elbow_a - step_size, 90, 160)
        elif key == ord('e'):
            new_elbow = _clamp_angle(elbow_a + step_size, 90, 160)
        elif key == 13 or key == 10:  # Enter
            recorded.append((px, py, base_a, shoulder_a, elbow_a))
            print(f"  {_get_corner_label(corner_idx)} 记录: "
                  f"底座={base_a}° 大臂={shoulder_a}° 小臂={elbow_a}°")
            corner_idx += 1
            if corner_idx < 4:
                # 移到下一个角的大致位置（保持当前角度）
                pass
            continue
        elif key == ord('r') or key == ord('R'):
            if recorded:
                recorded.pop()
                corner_idx -= 1
                print(f"  退回 {_get_corner_label(corner_idx)}")
            continue
        else:
            continue  # 无有效按键

        # ---- 有角度变化时才发命令 ----
        delta_base = new_base - base_a
        delta_shoulder = new_shoulder - shoulder_a
        delta_elbow = new_elbow - elbow_a

        if delta_base != 0 or delta_shoulder != 0 or delta_elbow != 0:
            # 只移动变化的轴，另一个轴保持原值
            x_cmd = new_base if delta_base != 0 else base_a
            y_cmd = new_shoulder if delta_shoulder != 0 else shoulder_a
            z_cmd = new_elbow if delta_elbow != 0 else elbow_a

            print(f"  移动 → 底座{x_cmd}° 大臂{y_cmd}° 小臂{z_cmd}°", end=' ')
            if comm.send_move_with_ack(x_cmd, y_cmd, z_cmd):
                print("ACK")
                current_angle = [x_cmd, y_cmd, z_cmd]
            else:
                print("超时(继续)")

    # ---- 标定完成 ----
    cv2.destroyAllWindows()
    cap.release()

    if len(recorded) < 4:
        print(f"\n警告: 只记录了 {len(recorded)} 个点，标定不完整")
        comm.disconnect()
        input("按 Enter 退出...")
        return

    # ---- 计算标定数据并保存 ----
    calib_data = {
        "roi": {"x1": x1, "y1": y1, "x2": x2, "y2": y2},
        "corners": []
    }
    for (cx, cy, ba, sa, ea) in recorded:
        calib_data["corners"].append({
            "px": cx, "py": cy,
            "base": ba, "shoulder": sa, "elbow": ea
        })

    _save_calibration(calib_data)

    # ---- 显示结果 ----
    print("\n" + "=" * 50)
    print("  标定完成")
    print("=" * 50)
    for i, d in enumerate(calib_data["corners"]):
        print(f"  {_get_corner_label(i)} 像素({d['px']:3d},{d['py']:3d}) "
              f"→ 底座{d['base']:3d}° 大臂{d['shoulder']:3d}° 小臂{d['elbow']:3d}°")

    # 验证：检查 4 个角的角度变化是否合理
    bases = [c['base'] for c in calib_data["corners"]]
    print(f"\n  底座范围: {min(bases)}° ~ {max(bases)}°")
    if max(bases) - min(bases) < 5:
        print("  警告: 底座角度几乎没有变化，检查标定是否正确")

    print("\n分拣盒角度请在 config.json 的 SORT_AREAS 中直接设置")
    comm.disconnect()
    print("\n标定完成，可以运行主程序了")
    input("按 Enter 退出...")


if __name__ == '__main__':
    main()
