# -*- coding: utf-8 -*-
"""
配置文件 - 所有参数集中管理
"""

# 摄像头配置
CAMERA_ID = 0  # 摄像头编号，0是默认摄像头
FRAME_WIDTH = 640  # 画面宽度
FRAME_HEIGHT = 480  # 画面高度

# ROI 检测区域（由 show_roi.py 设定，为 None 时使用全画面）
DETECTION_ROI = None  # {'x1': 190, 'y1': 210, 'x2': 430, 'y2': 330}

# 颜色识别阈值（HSV颜色空间）
COLOR_RANGES = {
    'yellow': [
        {'lower': [20, 100, 100], 'upper': [35, 255, 255]}      # 黄色
    ],
    'green': [
        {'lower': [35, 100, 100], 'upper': [77, 255, 255]}      # 绿色
    ]
}

# 形状识别参数
SHAPE_PARAMS = {
    'min_area': 500,           # 最小面积，过滤噪点
    'max_area': 50000,         # 最大面积
    'circle_threshold': 0.75,  # 圆形判定阈值（越接近1越圆）
    'approx_epsilon': 0.04,    # 多边形逼近精度
    'min_confidence': 0.6      # 最小置信度阈值
}

# 坐标映射参数（像素坐标 -> 机械臂坐标）
# 需要根据实际标定调整这些参数
COORD_MAP = {
    'offset_x': 320,    # X方向偏移
    'offset_y': 240,    # Y方向偏移
    'scale_x': 1.0,     # X方向缩放比例
    'scale_y': 1.0      # Y方向缩放比例
}

# 分拣区域坐标（机械臂坐标系，需要实际标定）
SORT_AREAS = {
    'yellow_circle': {'x': 170},     # 黄色圆形 → 底座170°
    'yellow_square': {'x': 125},    # 黄色方形 → 底座125°
    'green_circle': {'x': 10},      # 绿色圆形 → 底座10°
    'green_square': {'x': 45},      # 绿色方形 → 底座45°
}

# 串口配置
SERIAL_CONFIG = {
    'port': 'COM3',          # 串口号，根据实际情况修改
    'baudrate': 115200,      # 波特率
    'timeout': 1             # 超时时间（秒）
}

# 通信协议定义
PROTOCOL = {
    'start_byte': 0xAA,         # 起始字节
    'end_byte': 0x55,           # 结束字节
    'cmd_move': 0x01,           # 移动指令
    'cmd_grab': 0x02,           # 抓取指令
    'cmd_release': 0x03,        # 释放指令
    'cmd_home': 0x04,           # 回原点指令
    'cmd_emergency_stop': 0x05,  # 紧急停止指令
    'cmd_init': 0x06              # 初始化舵机指令
}

# 状态机配置（侧边夹取参数）
STATE_MACHINE_CONFIG = {
    'move_delay': 1.0,      # 移动完成后的等待时间（秒）
    'grab_delay': 0.8,      # 抓取/释放操作的等待时间（秒）
    'action_delay': 0.5,    # 动作之间的等待时间（秒）
    'confirm_delay': 3.0,   # 检测到零件后等待时间（秒）
    # 初始角度
    'shoulder_init': 90,    # 大臂初始角度
    'elbow_init': 120,      # 小臂初始角度
    'base_init': 90,        # 底座初始角度
    # 预定位：大臂偏移到零件旁边
    'prep_offset_y': 15,    # 大臂偏移量(度)
    'prep_elbow': 100,      # 预定位时小臂角度
    # 接近与抓取（大臂先到→小臂后到→小臂保持不变）
    'approach_elbow': 120,  # 侧向接近时小臂角度
    'grab_elbow': 110,      # 抓取时小臂角度（此后保持不变直到释放）
}

# 显示配置
DISPLAY = {
    'show_fps': True,                    # 显示帧率
    'show_detection': True,              # 显示检测结果
    'window_name': '零件分拣视觉系统'     # 窗口名称
}

# 分拣优先级配置
SORT_PRIORITY = {
    'color_priority': ['yellow', 'green'],           # 颜色优先级（从高到低）
    'shape_priority': ['circle', 'square'],          # 形状优先级（从高到低）
    'use_area_as_tiebreaker': True                   # 优先级相同时是否按面积排序
}
