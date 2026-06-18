# -*- coding: utf-8 -*-
"""
摄像头位置检查工具 - 自动检测摄像头是否移动
"""

import cv2
import numpy as np
import config


class CameraChecker:
    """摄像头检查器类"""
    
    def __init__(self):
        """初始化检查器"""
        self.reference_point = None  # 参考点坐标
        self.threshold = 10  # 允许偏移阈值（像素）
        
    def set_reference_point(self, x, y):
        """
        设置参考点坐标
        
        参数:
            x: 参考点X坐标
            y: 参考点Y坐标
        """
        self.reference_point = (x, y)
        print(f"参考点已设置: ({x}, {y})")
        
    def find_reference_marker(self, frame):
        """
        在画面中查找参考标记（黑色圆点）
        
        参数:
            frame: 输入图像
            
        返回:
            (x, y): 参考点坐标，如果未找到返回None
        """
        try:
            # 转换为灰度图像
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 二值化（找黑色圆点）
            _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
            
            # 查找轮廓
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 寻找符合条件的轮廓（圆形、大小适中）
            for contour in contours:
                area = cv2.contourArea(contour)
                # 过滤太小或太大的轮廓
                if area < 500 or area > 5000:
                    continue
                
                # 计算圆形度
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                
                # 判断是否为圆形
                if circularity > 0.7:
                    # 计算中心点
                    M = cv2.moments(contour)
                    if M['m00'] == 0:
                        continue
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    return (cx, cy)
            
            return None
        except Exception as e:
            print(f"查找参考标记错误: {e}")
            return None
    
    def check_position(self, frame):
        """
        检查摄像头位置是否变化
        
        参数:
            frame: 输入图像
            
        返回:
            (bool, delta_x, delta_y): (是否正常, X偏移, Y偏移)
        """
        # 如果没有设置参考点，先查找并设置
        if self.reference_point is None:
            marker = self.find_reference_marker(frame)
            if marker:
                self.set_reference_point(marker[0], marker[1])
                return (True, 0, 0)
            else:
                print("警告：未找到参考标记，请在工作台左上角放置黑色圆点标记")
                return (None, 0, 0)  # 返回None表示未找到标记
        
        # 查找当前参考点位置
        current_marker = self.find_reference_marker(frame)
        if current_marker is None:
            print("警告：未检测到参考标记")
            return (None, 0, 0)  # 返回None表示未找到标记
        
        # 计算偏移
        delta_x = current_marker[0] - self.reference_point[0]
        delta_y = current_marker[1] - self.reference_point[1]
        
        # 判断是否超出阈值
        if abs(delta_x) > self.threshold or abs(delta_y) > self.threshold:
            print(f"警告：摄像头位置发生变化！")
            print(f"      参考点偏移: X={delta_x}, Y={delta_y}")
            print(f"      建议重新标定坐标")
            return (False, delta_x, delta_y)
        else:
            print(f"摄像头位置检查通过，偏移量: ({delta_x}, {delta_y})")
            return (True, delta_x, delta_y)
    
    def draw_reference_info(self, frame):
        """
        在画面上绘制参考点信息
        
        参数:
            frame: 输入图像
            
        返回:
            frame: 绘制后的图像
        """
        # 查找参考标记
        marker = self.find_reference_marker(frame)
        
        if marker:
            cx, cy = marker
            # 绘制参考点
            cv2.circle(frame, (cx, cy), 10, (0, 0, 255), 2)
            cv2.putText(frame, f"Ref: ({cx}, {cy})", (cx - 40, cy - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # 如果已设置参考点，显示偏移
            if self.reference_point:
                delta_x = cx - self.reference_point[0]
                delta_y = cy - self.reference_point[1]
                cv2.putText(frame, f"Delta: ({delta_x}, {delta_y})", (cx - 40, cy + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        return frame


def test_camera_check():
    """测试摄像头检查功能"""
    # 初始化摄像头
    cap = cv2.VideoCapture(config.CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    # 初始化检查器
    checker = CameraChecker()
    
    print("摄像头检查测试")
    print("按 'q' 退出")
    print("按 's' 设置参考点")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 检查位置
        status, dx, dy = checker.check_position(frame)
        
        # 绘制信息
        frame = checker.draw_reference_info(frame)
        
        # 显示状态
        status_text = "正常" if status else "偏移!"
        color = (0, 255, 0) if status else (0, 0, 255)
        cv2.putText(frame, f"状态: {status_text}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # 显示画面
        cv2.imshow("摄像头检查", frame)
        
        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            marker = checker.find_reference_marker(frame)
            if marker:
                checker.set_reference_point(marker[0], marker[1])
            else:
                print("未找到参考标记")
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    test_camera_check()