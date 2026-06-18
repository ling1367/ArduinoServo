# -*- coding: utf-8 -*-
"""
坐标映射模块 - 统一管理所有坐标转换逻辑
"""

import copy
import numpy as np
from typing import Tuple
import config
from config_manager import get_config_manager


class CoordinateMapper:
    def __init__(self):
        self.mapping_params = {}
        self.calibration_points = []
        self.config_manager = get_config_manager()
        self._load_config()

    def _validate_params(self, params):
        """验证映射参数的有效性"""
        if 'scale_x' in params:
            if params['scale_x'] <= 0 or params['scale_x'] > 10:
                print(f"警告: scale_x={params['scale_x']} 超出有效范围(0, 10]，使用默认值1.0")
                params['scale_x'] = 1.0

        if 'scale_y' in params:
            if params['scale_y'] <= 0 or params['scale_y'] > 10:
                print(f"警告: scale_y={params['scale_y']} 超出有效范围(0, 10]，使用默认值1.0")
                params['scale_y'] = 1.0

        if 'offset_x' in params:
            params['offset_x'] = max(-1000, min(1000, params['offset_x']))

        if 'offset_y' in params:
            params['offset_y'] = max(-1000, min(1000, params['offset_y']))

        return True

    def _load_config(self):
        """加载坐标配置（统一从ConfigManager读取）"""
        self.mapping_params = {
            'offset_x': self.config_manager.get('COORD_MAP.offset_x', config.COORD_MAP['offset_x']),
            'offset_y': self.config_manager.get('COORD_MAP.offset_y', config.COORD_MAP['offset_y']),
            'scale_x': self.config_manager.get('COORD_MAP.scale_x', config.COORD_MAP['scale_x']),
            'scale_y': self.config_manager.get('COORD_MAP.scale_y', config.COORD_MAP['scale_y'])
        }

        self._validate_params(self.mapping_params)

    def save_config(self):
        """保存坐标配置（统一通过ConfigManager写入config.json）"""
        try:
            self.config_manager.set('COORD_MAP', self.mapping_params)
            self.config_manager.save()
            print("坐标配置已保存到 config.json")
            return True
        except Exception as e:
            print(f"坐标配置保存失败: {e}")
            return False

    def pixel_to_arm(self, px: int, py: int) -> Tuple[int, int]:
        """像素坐标转机械臂坐标 (arm = px * scale + offset)"""
        arm_x = px * self.mapping_params['scale_x'] + self.mapping_params['offset_x']
        arm_y = py * self.mapping_params['scale_y'] + self.mapping_params['offset_y']
        return int(arm_x), int(arm_y)

    def arm_to_pixel(self, arm_x: int, arm_y: int) -> Tuple[int, int]:
        """机械臂坐标转像素坐标 (px = (arm - offset) / scale)"""
        px = int((arm_x - self.mapping_params['offset_x']) / self.mapping_params['scale_x'])
        py = int((arm_y - self.mapping_params['offset_y']) / self.mapping_params['scale_y'])
        return px, py

    def validate_pixel_coord(self, px: int, py: int) -> bool:
        """验证像素坐标是否在有效范围内"""
        return 0 <= px <= config.FRAME_WIDTH and 0 <= py <= config.FRAME_HEIGHT

    def validate_arm_coord(self, arm_x: int, arm_y: int) -> bool:
        """验证机械臂坐标（角度）是否在有效范围内"""
        return 5 <= arm_x <= 175 and 70 <= arm_y <= 170

    def add_calibration_point(self, px: int, py: int, arm_x: int, arm_y: int):
        """添加标定点"""
        self.calibration_points.append((px, py, arm_x, arm_y))
        print(f"已添加标定点: 像素({px}, {py}) -> 机械臂({arm_x}, {arm_y})")

    def calculate_mapping_from_calibration(self) -> bool:
        """从标定点计算映射参数"""
        if len(self.calibration_points) < 2:
            print("至少需要2个标定点")
            return False

        px_list = [p[0] for p in self.calibration_points]
        py_list = [p[1] for p in self.calibration_points]
        ax_list = [p[2] for p in self.calibration_points]
        ay_list = [p[3] for p in self.calibration_points]

        try:
            if len(self.calibration_points) == 2:
                p1, p2 = self.calibration_points
                scale_x = (p2[2] - p1[2]) / (p2[0] - p1[0]) if (p2[0] - p1[0]) != 0 else 1
                scale_y = (p2[3] - p1[3]) / (p2[1] - p1[1]) if (p2[1] - p1[1]) != 0 else 1
                offset_x = p1[2] - scale_x * p1[0]
                offset_y = p1[3] - scale_y * p1[1]
            else:
                coeffs_x = np.polyfit(px_list, ax_list, 1)
                coeffs_y = np.polyfit(py_list, ay_list, 1)
                scale_x = coeffs_x[0]
                scale_y = coeffs_y[0]
                offset_x = coeffs_x[1]
                offset_y = coeffs_y[1]

            self.mapping_params = {
                'offset_x': float(offset_x),
                'offset_y': float(offset_y),
                'scale_x': float(scale_x),
                'scale_y': float(scale_y)
            }

            print("映射参数计算完成:")
            print(f"  offset_x = {offset_x:.4f}")
            print(f"  offset_y = {offset_y:.4f}")
            print(f"  scale_x = {scale_x:.4f}")
            print(f"  scale_y = {scale_y:.4f}")

            return True
        except Exception as e:
            print(f"映射参数计算失败: {e}")
            return False

    def reset_calibration(self):
        """重置标定点"""
        self.calibration_points = []
        print("标定点已重置")

    def get_mapping_params(self) -> dict:
        """获取映射参数副本"""
        return copy.deepcopy(self.mapping_params)

    def set_mapping_params(self, params: dict):
        """设置映射参数（带验证）"""
        self._validate_params(params)
        self.mapping_params.update(params)
        print("映射参数已更新")

    # ==================== 4 点标定双线性插值 ====================

    def load_arm_calibration(self) -> dict:
        """从 ConfigManager 加载 4 点标定数据"""
        calib = self.config_manager.get('ARM_CALIBRATION')
        return calib if calib else None

    def pixel_to_calibrated_pose(self, px: int, py: int) -> Tuple[int, int, int]:
        """像素坐标 → (底座角, 大臂角, 小臂角) 双线性插值

        使用 calibrate_4point.py 标定的 4 个角点数据。
        如果未标定，回退到旧的 pixel_to_arm + 默认小臂角。

        返回:
            (base_angle, shoulder_angle, elbow_angle)
        """
        calib = self.load_arm_calibration()
        if calib is None:
            # 未标定：回退到旧逻辑
            arm_x, arm_y = self.pixel_to_arm(px, py)
            return (arm_x, arm_y, 110)

        corners = calib['corners']
        if len(corners) < 4:
            arm_x, arm_y = self.pixel_to_arm(px, py)
            return (arm_x, arm_y, 110)

        roi = calib['roi']
        x1, y1, x2, y2 = roi['x1'], roi['y1'], roi['x2'], roi['y2']

        # 4 个角按顺序: 左上→右上→右下→左下
        c_tl, c_tr, c_br, c_bl = corners

        # 归一化坐标 (0~1)
        tx = (px - x1) / (x2 - x1) if (x2 - x1) != 0 else 0
        ty = (py - y1) / (y2 - y1) if (y2 - y1) != 0 else 0
        tx = max(0, min(1, tx))
        ty = max(0, min(1, ty))

        def lerp(a, b, t):
            return a + (b - a) * t

        # 上边插值
        b_top = lerp(c_tl['base'], c_tr['base'], tx)
        s_top = lerp(c_tl['shoulder'], c_tr['shoulder'], tx)
        e_top = lerp(c_tl['elbow'], c_tr['elbow'], tx)

        # 下边插值
        b_bot = lerp(c_bl['base'], c_br['base'], tx)
        s_bot = lerp(c_bl['shoulder'], c_br['shoulder'], tx)
        e_bot = lerp(c_bl['elbow'], c_br['elbow'], tx)

        # 上下之间插值
        base_angle = int(round(lerp(b_top, b_bot, ty)))
        shoulder_angle = int(round(lerp(s_top, s_bot, ty)))
        elbow_angle = int(round(lerp(e_top, e_bot, ty)))

        return (base_angle, shoulder_angle, elbow_angle)

    def validate_calibrated_pose(self, base: int, shoulder: int, elbow: int) -> bool:
        """验证标定后的舵机角度是否在安全范围内"""
        if not (5 <= base <= 175):
            return False
        if not (90 <= shoulder <= 170):
            return False
        if not (90 <= elbow <= 160):
            return False
        return True


_coordinate_mapper = None

def get_coordinate_mapper() -> CoordinateMapper:
    """获取全局坐标映射器实例"""
    global _coordinate_mapper
    if _coordinate_mapper is None:
        _coordinate_mapper = CoordinateMapper()
    return _coordinate_mapper
