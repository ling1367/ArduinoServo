# -*- coding: utf-8 -*-
"""
视觉识别模块 - 负责图像处理和零件识别
"""

import cv2
import numpy as np
import threading
import config
from coordinate_mapper import get_coordinate_mapper
from config_manager import get_config_manager
from exception_handler import (
    get_exception_handler,
    VisionException,
    CameraException,
    ThreadSafeExceptionContext
)


class VisionDetector:
    """视觉检测器类"""
    
    def __init__(self):
        """初始化检测器"""
        self.cap = None
        self.frame = None
        self._frame_lock = threading.Lock()
        self.results = []
        self.coord_mapper = get_coordinate_mapper()
        self.config_manager = get_config_manager()
        self.exception_handler = get_exception_handler()
        self.exception_context = ThreadSafeExceptionContext(self.exception_handler)
        self.perspective_matrix = None
        self.roi = None
        self._load_roi()
        
    def _load_roi(self):
        """从 ConfigManager 加载 ROI 检测区域"""
        self.roi = self.config_manager.get('DETECTION_ROI')
        if self.roi:
            print(f"ROI 检测区域已加载: {self.roi}")
        else:
            print("未配置 ROI（使用全画面检测）")

    def init_camera(self):
        """初始化摄像头"""
        context = {
            'camera_id': config.CAMERA_ID,
            'frame_width': config.FRAME_WIDTH,
            'frame_height': config.FRAME_HEIGHT,
            'method': 'init_camera'
        }
        self.exception_context.update_context(context)
        
        try:
            self.cap = cv2.VideoCapture(config.CAMERA_ID)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
            
            if not self.cap.isOpened():
                raise CameraException(
                    "无法打开摄像头",
                    error_code='CAM001',
                    context=context,
                    severity='critical'
                )
            
            self.exception_handler.logger.info("摄像头初始化成功")
            
            # 加载透视校正参数
            self.load_perspective_calibration()
            return True
            
        except CameraException:
            raise
        except Exception as e:
            self.exception_context.handle_exception(e)
            raise CameraException(
                f"摄像头初始化失败: {e}",
                error_code='CAM002',
                context=context,
                severity='critical'
            ) from e
    
    def release_camera(self):
        """释放摄像头"""
        if self.cap is not None:
            self.cap.release()
    
    def capture_frame(self):
        """捕获一帧图像并应用预处理"""
        context = {
            'cap_initialized': self.cap is not None,
            'method': 'capture_frame'
        }
        
        try:
            if self.cap is None:
                raise CameraException(
                    "摄像头未初始化",
                    error_code='CAM003',
                    context=context
                )
            
            ret, frame_raw = self.cap.read()
            
            if not ret:
                raise CameraException(
                    "无法读取摄像头帧",
                    error_code='CAM004',
                    context=context
                )
            
            if self.perspective_matrix is not None:
                try:
                    if (isinstance(self.perspective_matrix, np.ndarray) and
                        self.perspective_matrix.shape == (3, 3)):
                        frame_raw = cv2.warpPerspective(frame_raw, self.perspective_matrix,
                                                        (config.FRAME_WIDTH, config.FRAME_HEIGHT))
                except Exception as e:
                    self.perspective_matrix = None
                    self.exception_handler.handle_exception(
                        VisionException(
                            "透视校正失败，已禁用",
                            error_code='VIS001',
                            context=context,
                            severity='warning'
                        )
                    )
            
            frame_raw = self.adjust_brightness(frame_raw)
            frame_raw = cv2.GaussianBlur(frame_raw, (3, 3), 0)
            
            with self._frame_lock:
                self.frame = frame_raw
            
            return True
            
        except CameraException:
            raise
        except Exception as e:
            self.exception_context.handle_exception(e)
            raise CameraException(
                f"捕获图像失败: {e}",
                error_code='CAM005',
                context=context
            ) from e
    
    def load_perspective_calibration(self):
        """从ConfigManager加载透视校正参数"""
        perspective_data = self.config_manager.get('PERSPECTIVE_CALIBRATION')
        if perspective_data and 'matrix' in perspective_data:
            self.perspective_matrix = np.array(perspective_data['matrix'], dtype=np.float32)
            print("透视校正参数加载成功")

    def adjust_brightness(self, frame):
        """自动调整图像亮度"""
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            brightness = np.mean(hsv[:, :, 2])

            if brightness < 60:
                hsv[:, :, 2] = np.minimum(hsv[:, :, 2] * 1.2, 255).astype(np.uint8)
            elif brightness > 200:
                hsv[:, :, 2] = np.maximum(hsv[:, :, 2] * 0.85, 0).astype(np.uint8)

            return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        except Exception as e:
            print(f"亮度调整错误: {e}")
            return frame
    
    def detect_color(self, frame, color_name):
        """检测指定颜色的区域"""
        context = {
            'color_name': color_name,
            'frame_shape': frame.shape if frame is not None else None,
            'method': 'detect_color'
        }
        
        try:
            if frame is None:
                raise VisionException(
                    "输入图像为空",
                    error_code='VIS002',
                    context=context
                )
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            color_ranges = self.config_manager.get('COLOR_RANGES', config.COLOR_RANGES)
            ranges = color_ranges.get(color_name, [])
            
            if not ranges:
                raise VisionException(
                    f"未找到颜色配置: {color_name}",
                    error_code='VIS003',
                    context=context
                )

            mask = None
            for r in ranges:
                lower = np.array(r['lower'])
                upper = np.array(r['upper'])
                current_mask = cv2.inRange(hsv, lower, upper)
                if mask is None:
                    mask = current_mask
                else:
                    mask = cv2.bitwise_or(mask, current_mask)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.dilate(mask, kernel, iterations=1)

            return mask
            
        except VisionException:
            raise
        except cv2.error as e:
            raise VisionException(
                f"OpenCV处理错误: {e}",
                error_code='VIS004',
                context=context
            ) from e
        except Exception as e:
            self.exception_context.handle_exception(e)
            raise VisionException(
                f"颜色检测失败: {e}",
                error_code='VIS005',
                context=context
            ) from e
    
    def detect_shape(self, contour):
        """识别轮廓的形状（带置信度过滤）"""
        context = {
            'contour_type': type(contour).__name__,
            'method': 'detect_shape'
        }
        
        try:
            area = cv2.contourArea(contour)
            if area < config.SHAPE_PARAMS['min_area'] or area > config.SHAPE_PARAMS['max_area']:
                return None, 0

            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, config.SHAPE_PARAMS['approx_epsilon'] * peri, True)
            x, y, w, h = cv2.boundingRect(contour)
            circularity = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
            solidity = float(area) / (w * h) if w * h > 0 else 0
            min_confidence = config.SHAPE_PARAMS.get('min_confidence', 0.6)

            if circularity > config.SHAPE_PARAMS['circle_threshold'] and solidity > 0.7:
                confidence = circularity * solidity
                if confidence >= min_confidence:
                    return 'circle', confidence
                return None, confidence
            elif len(approx) == 4:
                aspect_ratio = float(w) / h if h > 0 else 0
                confidence = solidity
                if confidence >= min_confidence:
                    if 0.8 < aspect_ratio < 1.2:
                        return 'square', confidence
                    return 'rectangle', confidence
                return None, confidence

            return 'unknown', 0
            
        except cv2.error as e:
            raise VisionException(
                f"OpenCV轮廓处理错误: {e}",
                error_code='VIS006',
                context=context
            ) from e
        except Exception as e:
            self.exception_context.handle_exception(e)
            raise VisionException(
                f"形状检测失败: {e}",
                error_code='VIS007',
                context=context
            ) from e
    
    def detect_parts(self, frame=None):
        """检测图像中的所有零件"""
        context = {
            'frame_provided': frame is not None,
            'internal_frame_available': self.frame is not None,
            'method': 'detect_parts'
        }
        
        try:
            if frame is None:
                frame = self.frame
            if frame is None:
                raise VisionException(
                    "没有可用的图像帧",
                    error_code='VIS008',
                    context=context
                )

            self.results = []
            color_ranges = self.config_manager.get('COLOR_RANGES', config.COLOR_RANGES)
            
            for color_name in color_ranges.keys():
                try:
                    mask = self.detect_color(frame, color_name)
                except VisionException:
                    # 单个颜色检测失败不影响其他颜色
                    continue
                    
                if mask is None:
                    continue

                try:
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                except cv2.error as e:
                    self.exception_handler.handle_exception(
                        VisionException(
                            f"轮廓检测失败: {e}",
                            error_code='VIS009',
                            context={**context, 'color': color_name}
                        )
                    )
                    continue
                
                for contour in contours:
                    try:
                        shape, score = self.detect_shape(contour)
                    except VisionException:
                        continue
                        
                    if shape is None or shape == 'unknown':
                        continue

                    M = cv2.moments(contour)
                    if M['m00'] == 0:
                        continue

                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    area = cv2.contourArea(contour)

                    # ROI 过滤：框外零件不识别
                    if self.roi:
                        if not (self.roi['x1'] <= cx <= self.roi['x2'] and
                                self.roi['y1'] <= cy <= self.roi['y2']):
                            continue
                    else:
                        h, w = frame.shape[:2]
                        margin = 25
                        if cx < margin or cx > w - margin or cy < margin or cy > h - margin:
                            continue

                    overlap = False
                    for existing in self.results:
                        ex_cx, ex_cy = existing['x'], existing['y']
                        distance = np.sqrt((cx - ex_cx)**2 + (cy - ex_cy)**2)
                        threshold = max(20, min(40, np.sqrt(area) / 4, np.sqrt(existing['area']) / 4))
                        if distance < threshold:
                            overlap = True
                            break

                    if not overlap:
                        self.results.append({
                            'color': color_name,
                            'shape': shape,
                            'x': cx,
                            'y': cy,
                            'area': area,
                            'contour': contour,
                            'confidence': score
                        })

            # 按优先级排序
            sort_priority = self.config_manager.get('SORT_PRIORITY', config.SORT_PRIORITY)
            color_priority = sort_priority.get('color_priority', [])
            shape_priority = sort_priority.get('shape_priority', [])
            use_area_tiebreaker = sort_priority.get('use_area_as_tiebreaker', True)

            # 计算每个零件的优先级分数
            def get_priority(part):
                # 颜色优先级（索引越小优先级越高）
                color_score = color_priority.index(part['color']) if part['color'] in color_priority else 999
                # 形状优先级（索引越小优先级越高）
                shape_score = shape_priority.index(part['shape']) if part['shape'] in shape_priority else 999
                return (color_score, shape_score, -part['area'] if use_area_tiebreaker else 0)

            # 过滤掉未定义颜色的零件（机械臂不处理）
            self.results = [p for p in self.results if p['color'] in color_priority]

            # 排序：先按颜色，再按形状，最后（可选）按面积
            self.results.sort(key=get_priority)
            return self.results
            
        except VisionException:
            raise
        except Exception as e:
            self.exception_context.handle_exception(e)
            raise VisionException(
                f"零件检测失败: {e}",
                error_code='VIS010',
                context=context
            ) from e

    def draw_results(self, frame=None, results=None):
        """在图像上绘制检测结果"""
        if frame is None:
            frame = self.frame.copy()
        if results is None:
            results = self.results

        color_map = {
            'yellow': (0, 255, 255),
            'green': (0, 255, 0)
        }

        for result in results:
            color_bgr = color_map.get(result['color'], (255, 255, 255))
            cv2.drawContours(frame, [result['contour']], -1, color_bgr, 2)
            cx, cy = result['x'], result['y']
            cv2.circle(frame, (cx, cy), 5, color_bgr, -1)
            label = f"{result['color']} {result['shape']}"
            cv2.putText(frame, label, (cx - 30, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)
            b, s, e = self.coord_mapper.pixel_to_calibrated_pose(cx, cy)
            coord_text = f"({b},{s},{e})"
            cv2.putText(frame, coord_text, (cx - 30, cy + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)

        # 绘制 ROI 检测区域
        if self.roi:
            x1, y1, x2, y2 = self.roi['x1'], self.roi['y1'], self.roi['x2'], self.roi['y2']
            overlay = frame.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 255), -1)
            cv2.addWeighted(overlay, 0.08, frame, 0.92, 0, frame)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
            for cx, cy in [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]:
                cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
            label = f"ROI {x2-x1}x{y2-y1}"
            cv2.putText(frame, label, (x1 + 4, y1 - 6),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 1)

        return frame
    
    def get_sort_target(self, color, shape):
        """根据颜色和形状获取分拣目标底座角度(X)

        侧边夹取模式下，Y=抓取时的大臂角度保持不变，只需返回底座目标角度。

        返回:
            int: 目标底座角度
        """
        key = f"{color}_{shape}"
        target = self.config_manager.get(f'SORT_AREAS.{key}', config.SORT_AREAS.get(key))
        if target is None:
            print(f"警告: 未配置分拣目标 {key}，使用默认位置")
            return 90
        return target['x']

    def pixel_to_arm_coord(self, px, py):
        """
        将像素坐标转换为机械臂坐标（供状态机调用）

        参数:
            px: 像素X坐标
            py: 像素Y坐标

        返回:
            (arm_x, arm_y): 机械臂坐标（旧线性映射，兼容保留）
        """
        return self.coord_mapper.pixel_to_arm(px, py)

    def pixel_to_calibrated_pose(self, px, py):
        """
        像素坐标 → (底座, 大臂, 小臂) 双线性插值

        使用 4 点标定数据，在 ROI 内任意位置插值出三个舵机角度。
        未标定时回退到旧 pixel_to_arm_coord + 默认小臂角 110°。

        返回:
            (base_angle, shoulder_angle, elbow_angle)
        """
        return self.coord_mapper.pixel_to_calibrated_pose(px, py)


def test_vision():
    """测试视觉识别模块"""
    detector = VisionDetector()
    
    if not detector.init_camera():
        return
    
    print("按 'q' 键退出")
    print("按 's' 键保存截图")
    
    while True:
        # 捕获图像
        if not detector.capture_frame():
            print("无法获取图像")
            break
        
        # 检测零件
        results = detector.detect_parts()
        
        # 绘制结果
        frame = detector.draw_results()
        
        # 显示结果
        cv2.imshow(config.DISPLAY['window_name'], frame)
        
        # 打印检测结果
        if results:
            print(f"\n检测到 {len(results)} 个零件:")
            for i, r in enumerate(results):
                arm_x, arm_y = detector.coord_mapper.pixel_to_arm(r['x'], r['y'])
                print(f"  {i+1}. {r['color']} {r['shape']}: 像素({r['x']}, {r['y']}) -> 机械臂({arm_x}, {arm_y})")
        
        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('screenshot.png', frame)
            print("截图已保存")
    
    detector.release_camera()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    test_vision()
