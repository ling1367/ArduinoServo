# -*- coding: utf-8 -*-
"""
摄像头标定点显示工具
显示4个标定点的像素坐标位置（可手动调整）
"""

import cv2
import numpy as np
import config

class CameraMarkerDisplay:
    def __init__(self):
        self.cap = None
        self.frame_width = config.FRAME_WIDTH
        self.frame_height = config.FRAME_HEIGHT

        self.markers = [
            {"name": "左上", "pixel_x": 160, "pixel_y": 120, "color": (0, 255, 0)},
            {"name": "右上", "pixel_x": 480, "pixel_y": 120, "color": (255, 0, 0)},
            {"name": "右下", "pixel_x": 480, "pixel_y": 360, "color": (0, 255, 255)},
            {"name": "左下", "pixel_x": 160, "pixel_y": 360, "color": (255, 0, 255)},
        ]

    def init_camera(self):
        self.cap = cv2.VideoCapture(config.CAMERA_ID)
        if not self.cap.isOpened():
            print("Error: Cannot open camera")
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        print(f"Camera initialized: {self.frame_width}x{self.frame_height}")
        return True

    def draw_marker(self, display, marker):
        px, py = marker["pixel_x"], marker["pixel_y"]
        color = marker["color"]
        name = marker["name"]

        cv2.circle(display, (px, py), 20, color, 3)
        cv2.circle(display, (px, py), 5, (255, 255, 255), -1)

        text = f"{name}"
        cv2.putText(display, text, (px - 25, py - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        text_pos = f"({px}, {py})"
        cv2.putText(display, text_pos, (px - 30, py + 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def run(self):
        print("=" * 60)
        print("  摄像头标定点显示工具")
        print("=" * 60)
        print("\n按 q 退出")
        print("按 1-4 切换选中标定点")
        print("按方向键调整标定点位置")
        print("\n标定点像素坐标:")

        selected = 0

        cv2.namedWindow('Camera Markers', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera Markers', 800, 600)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Failed to read frame")
                break

            display = frame.copy()
            h, w = frame.shape[:2]

            cv2.putText(display, "摄像头标定点显示", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(display, "按 q 退出 | 1-4 选择标定点 | 方向键移动",
                       (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            for i, marker in enumerate(self.markers):
                self.draw_marker(display, marker)
                if i == selected:
                    px, py = marker["pixel_x"], marker["pixel_y"]
                    cv2.rectangle(display, (px - 30, py - 40), (px + 60, py + 50), (255, 255, 0), 2)

            cv2.putText(display, f"当前选中: {self.markers[selected]['name']} {self.markers[selected]['pixel_x']},{self.markers[selected]['pixel_y']}",
                       (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow('Camera Markers', display)

            key = cv2.waitKey(30) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('1'):
                selected = 0
                print(f"选中: {self.markers[0]['name']}")
            elif key == ord('2'):
                selected = 1
                print(f"选中: {self.markers[1]['name']}")
            elif key == ord('3'):
                selected = 2
                print(f"选中: {self.markers[2]['name']}")
            elif key == ord('4'):
                selected = 3
                print(f"选中: {self.markers[3]['name']}")
            elif key == 81 or key == 2:
                self.markers[selected]["pixel_x"] = max(0, self.markers[selected]["pixel_x"] - 5)
                print(f"{self.markers[selected]['name']}: ({self.markers[selected]['pixel_x']}, {self.markers[selected]['pixel_y']})")
            elif key == 83 or key == 6:
                self.markers[selected]["pixel_x"] = min(w, self.markers[selected]["pixel_x"] + 5)
                print(f"{self.markers[selected]['name']}: ({self.markers[selected]['pixel_x']}, {self.markers[selected]['pixel_y']})")
            elif key == 82 or key == 0:
                self.markers[selected]["pixel_y"] = max(0, self.markers[selected]["pixel_y"] - 5)
                print(f"{self.markers[selected]['name']}: ({self.markers[selected]['pixel_x']}, {self.markers[selected]['pixel_y']})")
            elif key == 84 or key == 1:
                self.markers[selected]["pixel_y"] = min(h, self.markers[selected]["pixel_y"] + 5)
                print(f"{self.markers[selected]['name']}: ({self.markers[selected]['pixel_x']}, {self.markers[selected]['pixel_y']})")

        cv2.destroyAllWindows()
        self.cap.release()

        print("\n最终标定点坐标:")
        for marker in self.markers:
            print(f"  {marker['name']}: pixel=({marker['pixel_x']}, {marker['pixel_y']})")

if __name__ == '__main__':
    displayer = CameraMarkerDisplay()
    if displayer.init_camera():
        displayer.run()