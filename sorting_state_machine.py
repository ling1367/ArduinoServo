# -*- coding: utf-8 -*-
"""
分拣状态机模块 - 负责管理分拣流程的状态转换
"""

from enum import Enum
import time
import config
from config_manager import get_config_manager
from exception_handler import (
    get_exception_handler,
    StateMachineException,
    ThreadSafeExceptionContext
)


class SortState(Enum):
    """分拣状态枚举"""
    IDLE = "空闲"
    MOVING_TO_PART = "移动到零件"
    GRABBING = "抓取中"
    MOVING_TO_TARGET = "移动到目标"
    RELEASING = "释放中"
    RETURNING_HOME = "回原点"
    COMPLETED = "完成"
    ERROR = "错误"
    EMERGENCY_STOPPED = "紧急停止"


class SortingStateMachine:
    """分拣状态机"""

    def __init__(self, system):
        """初始化状态机

        参数:
            system: SortingSystem实例
        """
        self.system = system
        self.current_state = SortState.IDLE
        self.current_part = None
        self.target_position = None
        self.error_message = None
        self.exception_handler = get_exception_handler()
        self.exception_context = ThreadSafeExceptionContext(self.exception_handler)

    def start_sorting(self, part):
        """开始分拣

        参数:
            part: 检测到的零件信息

        返回:
            bool: 分拣是否成功
        """
        self.current_part = part
        self.current_state = SortState.MOVING_TO_PART
        return self._process_current_state()

    def _check_emergency_stop(self):
        """检查紧急停止状态"""
        with self.system._lock:
            if self.system.emergency_stop:
                self.current_state = SortState.EMERGENCY_STOPPED
                return True
        return False

    def _send_cmd_with_error(self, cmd_func, error_msg):
        """发送指令并处理错误

        参数:
            cmd_func: 指令函数（无参数）
            error_msg: 错误信息

        返回:
            bool: 是否成功
        """
        if not cmd_func():
            self.current_state = SortState.ERROR
            self.error_message = error_msg
            return False
        return True

    def _get_config(self, key):
        """获取状态机配置参数（优先从 config_manager 读取）"""
        config_manager = get_config_manager()
        return config_manager.get(f'STATE_MACHINE_CONFIG.{key}', config.STATE_MACHINE_CONFIG.get(key, 0))

    def _process_current_state(self):
        """处理当前状态

        返回:
            bool: 处理是否成功
        """
        context = {
            'current_state': self.current_state.value,
            'method': '_process_current_state'
        }
        self.exception_context.update_context(context)
        
        try:
            if self._check_emergency_stop():
                return False

            state_handlers = {
                SortState.IDLE: lambda: True,
                SortState.MOVING_TO_PART: self._handle_moving_to_part,
                SortState.GRABBING: self._handle_grabbing,
                SortState.MOVING_TO_TARGET: self._handle_moving_to_target,
                SortState.RELEASING: self._handle_releasing,
                SortState.RETURNING_HOME: self._handle_returning_home,
                SortState.COMPLETED: self._handle_completed,
                SortState.ERROR: self._handle_error,
            }

            handler = state_handlers.get(self.current_state)
            if handler:
                return handler()

            raise StateMachineException(
                f"未知状态: {self.current_state}",
                error_code='SM001',
                context=context
            )
        except StateMachineException:
            raise
        except Exception as e:
            self.exception_context.handle_exception(e)
            self.current_state = SortState.ERROR
            self.error_message = f"状态处理异常: {e}"
            raise StateMachineException(
                f"状态机异常: {e}",
                error_code='SM002',
                context=context
            ) from e

    def _handle_moving_to_part(self):
        """处理移动到零件状态"""
        base_angle, shoulder_angle, elbow_angle = self.system.detector.pixel_to_calibrated_pose(
            self.current_part['x'],
            self.current_part['y']
        )

        if not self.system.arm_controller.validate_movement(base_angle, shoulder_angle, elbow_angle):
            self.current_state = SortState.ERROR
            self.error_message = "目标位置超出安全范围"
            return False

        confirm_delay = self._get_config('confirm_delay', 3.0)
        print(f"  [等待] 检测到零件，等待 {confirm_delay:.1f} 秒后开始移动...")
        time.sleep(confirm_delay)

        move_delay = self._get_config('move_delay', 1.0)

        print(f"  [步骤1] 移动到零件上方... 底座:{base_angle}° 大臂:{shoulder_angle}° 小臂:{elbow_angle}°")
        if not self._send_cmd_with_error(
            lambda: self.system.comm.send_move_with_ack(base_angle, shoulder_angle, z=elbow_angle),
            "移动指令失败"
        ):
            return False

        time.sleep(move_delay)
        self.current_state = SortState.GRABBING
        return self._process_current_state()

    def _handle_grabbing(self):
        """处理抓取状态"""
        base_angle, shoulder_angle, elbow_angle = self.system.detector.pixel_to_calibrated_pose(
            self.current_part['x'],
            self.current_part['y']
        )

        grab_elbow = self._get_config('grab_elbow', 110)
        post_grab_elbow = self._get_config('approach_elbow', 120)
        grab_delay = self._get_config('grab_delay', 0.8)
        action_delay = self._get_config('action_delay', 0.5)

        print(f"  [步骤2] 下降到抓取高度... 小臂从{elbow_angle}°调整到{grab_elbow}°")
        if not self._send_cmd_with_error(
            lambda: self.system.comm.send_move_with_ack(base_angle, shoulder_angle, z=grab_elbow),
            "下降指令失败"
        ):
            return False

        time.sleep(grab_delay)

        if self._check_emergency_stop():
            return False

        part_shape = self.current_part.get('shape', 'circle')
        if part_shape == 'square':
            print(f"  [步骤3] 抓取零件（方形）...")
            grab_func = lambda: self.system.comm.send_grab_square_with_ack()
        else:
            print(f"  [步骤3] 抓取零件（圆形）...")
            grab_func = lambda: self.system.comm.send_grab_circle_with_ack()
        
        if not self._send_cmd_with_error(
            grab_func,
            "抓取指令失败"
        ):
            return False

        time.sleep(action_delay)

        if self._check_emergency_stop():
            return False

        print(f"  [步骤4] 上升... 小臂调整到{post_grab_elbow}°")
        if not self._send_cmd_with_error(
            lambda: self.system.comm.send_move_with_ack(base_angle, shoulder_angle, z=post_grab_elbow),
            "上升指令失败"
        ):
            return False

        time.sleep(action_delay)

        if self._check_emergency_stop():
            return False

        target_x = self.system.detector.get_sort_target(
            self.current_part['color'],
            self.current_part['shape']
        )
        self.target_position = (target_x, shoulder_angle, post_grab_elbow)

        self.current_state = SortState.MOVING_TO_TARGET
        return self._process_current_state()

    def _handle_moving_to_target(self):
        """处理移动到目标状态"""
        target_x, target_y, target_z = self.target_position

        if not self.system.arm_controller.validate_movement(target_x, target_y, target_z):
            self.current_state = SortState.ERROR
            self.error_message = "目标位置超出安全范围"
            return False

        move_delay = self._get_config('move_delay', 1.0)

        print(f"  [步骤5] 移动到目标位置... 底座旋转到{target_x}°")
        if not self._send_cmd_with_error(
            lambda: self.system.comm.send_move_with_ack(target_x, target_y, z=target_z),
            "移动指令失败"
        ):
            return False

        time.sleep(move_delay)
        self.current_state = SortState.RELEASING
        return self._process_current_state()

    def _handle_releasing(self):
        """处理释放状态"""
        target_x, target_y, target_z = self.target_position

        release_elbow = self._get_config('release_elbow', 100)
        lift_elbow = self._get_config('approach_elbow', 120)
        grab_delay = self._get_config('grab_delay', 0.8)
        action_delay = self._get_config('action_delay', 0.5)

        print(f"  [步骤6] 下降到放置高度... 小臂调整到{release_elbow}°")
        if not self._send_cmd_with_error(
            lambda: self.system.comm.send_move_with_ack(target_x, target_y, z=release_elbow),
            "下降指令失败"
        ):
            return False

        time.sleep(grab_delay)

        if self._check_emergency_stop():
            return False

        print("  [步骤7] 释放零件...")
        if not self._send_cmd_with_error(
            lambda: self.system.comm.send_release_with_ack(),
            "释放指令失败"
        ):
            return False

        time.sleep(action_delay)

        if self._check_emergency_stop():
            return False

        print(f"  [步骤8] 上升... 小臂调整到{lift_elbow}°")
        if not self._send_cmd_with_error(
            lambda: self.system.comm.send_move_with_ack(target_x, target_y, z=lift_elbow),
            "上升指令失败"
        ):
            return False

        time.sleep(action_delay)

        if self._check_emergency_stop():
            return False

        self.current_state = SortState.RETURNING_HOME
        return self._process_current_state()

    def _handle_returning_home(self):
        """处理回原点状态"""
        move_delay = self._get_config('move_delay')

        print("  [步骤9] 回原点...")
        if not self._send_cmd_with_error(
            lambda: self.system.comm.send_home_with_ack(),
            "回原点指令失败"
        ):
            return False

        time.sleep(move_delay)
        self.current_state = SortState.COMPLETED
        return self._process_current_state()

    def _handle_completed(self):
        """处理完成状态"""
        self.current_state = SortState.IDLE
        return True

    def _handle_error(self):
        """处理错误状态"""
        print(f"错误：{self.error_message}")
        self.current_state = SortState.IDLE
        return False

    def get_state_info(self):
        """获取当前状态信息

        返回:
            dict: 状态信息
        """
        return {
            'state': self.current_state,
            'part': self.current_part,
            'target': self.target_position,
            'error': self.error_message
        }
