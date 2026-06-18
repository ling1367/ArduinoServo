#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂控制模块 - 负责运动路径规划和安全控制
"""

from config_manager import get_config_manager


class ArmController:
    """机械臂控制器类"""

    Z_MIN = 90
    Z_MAX = 160

    def __init__(self):
        """初始化机械臂控制器"""
        self.current_x = 90    # 底座当前角度
        self.current_y = 90    # 大臂当前角度
        self.current_z = 120   # 小臂当前角度
        self.work_area = {
            'x_min': 5, 'x_max': 175,   # 底座舵机角度限位
            'y_min': 90, 'y_max': 180,   # 大臂舵机角度限位
        }
        self._is_moving = False
        self.config_manager = get_config_manager()

    def get_current_position(self):
        """获取当前位置"""
        return self.current_x, self.current_y

    def update_position(self, x=None, y=None, z=None):
        """更新记录的舵机角度（调用方确认到位后调用）"""
        if x is not None:
            self.current_x = x
        if y is not None:
            self.current_y = y
        if z is not None:
            self.current_z = z

    def validate_movement(self, target_x, target_y, target_z=None):
        """验证移动是否在安全范围内

        参数:
            target_x: 目标底座角度
            target_y: 目标大臂角度
            target_z: 目标小臂角度（可选）

        返回:
            bool: 是否安全
        """
        if target_x < self.work_area['x_min'] or target_x > self.work_area['x_max']:
            print(f"安全检查失败：底座 {target_x}° 超出范围")
            return False
        if target_y < self.work_area['y_min'] or target_y > self.work_area['y_max']:
            print(f"安全检查失败：大臂 {target_y}° 超出范围")
            return False
        if target_z is not None and (target_z < self.Z_MIN or target_z > self.Z_MAX):
            print(f"安全检查失败：小臂 {target_z}° 超出范围")
            return False
        return True
