# -*- coding: utf-8 -*-
"""
统一配置管理器 - 集中管理所有配置
消除配置分散、不安全修改、加载逻辑混乱等问题
"""

import json
import os
from typing import Any, Dict, Optional
import config as default_config


class ConfigManager:
    """统一配置管理器"""
    
    def __init__(self, config_file: str = 'config.json'):
        self.config_file = config_file
        self.config: Dict[str, Any] = {}
        self._load_config()
    
    def _load_config(self):
        """加载配置文件"""
        self.config = self._load_default_config()
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    self._merge_config(self.config, user_config)
                    print(f"配置文件 {self.config_file} 加载成功")
            except Exception as e:
                print(f"配置文件加载失败: {e}，使用默认配置")
        
        self._validate_config()
    
    def _load_default_config(self) -> Dict[str, Any]:
        """从config模块加载默认配置"""
        return {
            'CAMERA_ID': default_config.CAMERA_ID,
            'FRAME_WIDTH': default_config.FRAME_WIDTH,
            'FRAME_HEIGHT': default_config.FRAME_HEIGHT,
            'DETECTION_ROI': default_config.DETECTION_ROI,
            'COLOR_RANGES': default_config.COLOR_RANGES,
            'SHAPE_PARAMS': default_config.SHAPE_PARAMS,
            'COORD_MAP': default_config.COORD_MAP.copy(),
            'SORT_AREAS': default_config.SORT_AREAS.copy(),
            'SERIAL_CONFIG': default_config.SERIAL_CONFIG.copy(),
            'PROTOCOL': default_config.PROTOCOL.copy(),
            'STATE_MACHINE_CONFIG': default_config.STATE_MACHINE_CONFIG.copy(),
            'DISPLAY': default_config.DISPLAY.copy(),
            'SORT_PRIORITY': default_config.SORT_PRIORITY.copy(),
            'PERSPECTIVE_CALIBRATION': {'matrix': None}
        }
    
    def _merge_config(self, base: Dict, update: Dict):
        """递归合并配置"""
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._merge_config(base[key], value)
            else:
                base[key] = value
    
    def _validate_config(self):
        """验证配置有效性"""
        coord_map = self.config.get('COORD_MAP', {})
        scale_x = coord_map.get('scale_x', 1)
        if scale_x <= 0 or scale_x > 10:
            print(f"警告: scale_x={scale_x} 超出有效范围(0, 10]，使用默认值1.0")
            coord_map['scale_x'] = 1.0
        scale_y = coord_map.get('scale_y', 1)
        if scale_y <= 0 or scale_y > 10:
            print(f"警告: scale_y={scale_y} 超出有效范围(0, 10]，使用默认值1.0")
            coord_map['scale_y'] = 1.0
        
        if 'offset_x' in coord_map:
            coord_map['offset_x'] = max(-1000, min(1000, coord_map['offset_x']))
        if 'offset_y' in coord_map:
            coord_map['offset_y'] = max(-1000, min(1000, coord_map['offset_y']))
        
        serial_config = self.config.get('SERIAL_CONFIG', {})
        if not serial_config.get('port'):
            print("警告: 串口号未配置，使用默认值COM3")
            serial_config['port'] = 'COM3'
    
    def get(self, key: str, default: Any = None) -> Any:
        """获取配置值，支持点号分隔的路径"""
        keys = key.split('.')
        value = self.config
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        return value
    
    def set(self, path: str, value):
        """设置配置值

        参数:
            path: 配置路径，支持点号导航，如 'SERIAL_CONFIG.port', 'COORD_MAP.scale_x'
            value: 配置值
        """
        keys = path.split('.')
        config = self.config
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]
        config[keys[-1]] = value
    
    def save(self) -> bool:
        """保存配置到文件"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=4, ensure_ascii=False)
            print(f"配置已保存到 {self.config_file}")
            return True
        except Exception as e:
            print(f"配置保存失败: {e}")
            return False
    
    def reload(self):
        """重新加载配置"""
        self._load_config()


_config_manager: Optional[ConfigManager] = None

def get_config_manager() -> ConfigManager:
    """获取全局配置管理器实例"""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigManager()
    return _config_manager
