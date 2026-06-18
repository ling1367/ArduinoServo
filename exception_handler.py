# -*- coding: utf-8 -*-
"""
统一异常处理框架 - 提供标准化的异常类型和处理机制
"""

import time
import json
import logging
import threading
from datetime import datetime
from typing import Dict, Any, Optional, Callable, Type, Tuple, List
from collections import defaultdict


class SystemException(Exception):
    """系统基础异常类"""
    
    def __init__(self, message: str, error_code: Optional[str] = None, 
                 context: Optional[Dict[str, Any]] = None, severity: str = 'error'):
        self.message = message
        self.error_code = error_code
        self.context = context or {}
        self.severity = severity
        self.timestamp = datetime.now()
        super().__init__(self.message)
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return {
            'timestamp': self.timestamp.isoformat(),
            'exception_type': type(self).__name__,
            'message': self.message,
            'error_code': self.error_code,
            'context': self.context,
            'severity': self.severity
        }


class CameraException(SystemException):
    """摄像头相关异常"""
    pass


class SerialCommException(SystemException):
    """串口通信异常"""
    pass


class VisionException(SystemException):
    """视觉处理异常"""
    pass


class HardwareException(SystemException):
    """硬件故障异常"""
    pass


class StateMachineException(SystemException):
    """状态机异常"""
    pass


class ConfigException(SystemException):
    """配置异常"""
    pass


class ExceptionHandler:
    """统一异常处理器"""
    
    def __init__(self, logger: Optional[logging.Logger] = None):
        self.logger = logger or self._create_default_logger()
        self.error_stats = defaultdict(int)
        self._lock = threading.Lock()
    
    def _create_default_logger(self) -> logging.Logger:
        """创建默认日志记录器"""
        logger = logging.getLogger('SystemExceptionHandler')
        if not logger.handlers:
            logger.setLevel(logging.INFO)
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger
    
    def handle_exception(self, exception: Exception, 
                        context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """处理异常的统一入口
        
        参数:
            exception: 异常对象
            context: 额外的上下文信息
            
        返回:
            异常信息字典
        """
        with self._lock:
            if isinstance(exception, SystemException):
                error_info = exception.to_dict()
                if context:
                    error_info['context'].update(context)
            else:
                error_info = {
                    'timestamp': datetime.now().isoformat(),
                    'exception_type': type(exception).__name__,
                    'message': str(exception),
                    'error_code': None,
                    'context': context or {},
                    'severity': 'error'
                }
            
            # 记录异常统计
            error_key = f"{error_info['exception_type']}_{error_info['message'][:50]}"
            self.error_stats[error_key] += 1
            
            # 结构化日志记录
            log_message = json.dumps(error_info, ensure_ascii=False)
            
            severity = error_info['severity']
            if severity == 'critical':
                self.logger.critical(log_message)
            elif severity == 'warning':
                self.logger.warning(log_message)
            else:
                self.logger.error(log_message)
            
            return error_info
    
    def get_error_stats(self) -> Dict[str, int]:
        """获取错误统计信息"""
        with self._lock:
            return dict(self.error_stats)
    
    def reset_error_stats(self):
        """重置错误统计"""
        with self._lock:
            self.error_stats.clear()
    
    def retry_with_backoff(self, func: Callable, 
                          max_attempts: int = 3, 
                          base_delay: float = 1.0,
                          exceptions: Tuple[Type[Exception], ...] = (Exception,),
                          context: Optional[Dict[str, Any]] = None,
                          on_retry: Optional[Callable[[int, Exception], None]] = None) -> Any:
        """带退避策略的重试机制
        
        参数:
            func: 要执行的函数
            max_attempts: 最大尝试次数
            base_delay: 基础延迟时间（秒）
            exceptions: 要捕获的异常类型
            context: 上下文信息
            on_retry: 重试回调函数
            
        返回:
            函数执行结果
        """
        attempt = 0
        last_exception = None
        
        while attempt < max_attempts:
            try:
                return func()
            except exceptions as e:
                attempt += 1
                last_exception = e
                
                if attempt >= max_attempts:
                    self.handle_exception(e, {
                        **(context or {}),
                        'attempt': attempt,
                        'max_attempts': max_attempts,
                        'final_attempt': True
                    })
                    raise
                
                # 调用重试回调
                if on_retry:
                    on_retry(attempt, e)
                
                # 计算延迟时间（指数退避）
                delay = base_delay * (2 ** (attempt - 1))
                self.logger.warning(
                    f"重试 {attempt}/{max_attempts}, 等待 {delay:.1f}s - {type(e).__name__}: {e}"
                )
                time.sleep(delay)
        
        raise last_exception


class RecoveryManager:
    """自动恢复管理器"""
    
    def __init__(self, exception_handler: ExceptionHandler):
        self.exception_handler = exception_handler
        self.recovery_strategies: Dict[str, Callable] = {}
        self.component_states: Dict[str, str] = {}
        self._lock = threading.Lock()
    
    def register_recovery_strategy(self, component_name: str, 
                                   recovery_func: Callable[[Exception], bool]):
        """注册组件恢复策略
        
        参数:
            component_name: 组件名称
            recovery_func: 恢复函数，接收异常参数，返回是否恢复成功
        """
        with self._lock:
            self.recovery_strategies[component_name] = recovery_func
            self.component_states[component_name] = 'healthy'
    
    def get_component_state(self, component_name: str) -> Optional[str]:
        """获取组件状态"""
        with self._lock:
            return self.component_states.get(component_name)
    
    def set_component_state(self, component_name: str, state: str):
        """设置组件状态"""
        with self._lock:
            self.component_states[component_name] = state
    
    def attempt_recovery(self, component_name: str, 
                        exception: Exception) -> bool:
        """尝试恢复组件
        
        参数:
            component_name: 组件名称
            exception: 触发恢复的异常
            
        返回:
            是否恢复成功
        """
        with self._lock:
            if component_name not in self.recovery_strategies:
                self.exception_handler.logger.warning(
                    f"组件 {component_name} 没有注册恢复策略")
                self.component_states[component_name] = 'failed'
                return False
            
            recovery_func = self.recovery_strategies[component_name]
        
        try:
            self.component_states[component_name] = 'recovering'
            result = recovery_func(exception)
            
            with self._lock:
                if result:
                    self.component_states[component_name] = 'healthy'
                    self.exception_handler.logger.info(
                        f"组件 {component_name} 恢复成功")
                else:
                    self.component_states[component_name] = 'failed'
                    self.exception_handler.logger.error(
                        f"组件 {component_name} 恢复失败")
            
            return result
        except Exception as e:
            self.exception_handler.handle_exception(e, {
                'component': component_name,
                'recovery_action': 'attempt_recovery'
            })
            with self._lock:
                self.component_states[component_name] = 'failed'
            return False
    
    def get_all_states(self) -> Dict[str, str]:
        """获取所有组件状态"""
        with self._lock:
            return dict(self.component_states)


class ThreadSafeExceptionContext:
    """线程安全的异常上下文管理器"""
    
    def __init__(self, exception_handler: ExceptionHandler, 
                 lock: Optional[threading.Lock] = None):
        self.exception_handler = exception_handler
        self.lock = lock or threading.Lock()
        self.context: Dict[str, Any] = {}
    
    def add_context(self, key: str, value: Any):
        """添加上下文信息"""
        with self.lock:
            self.context[key] = value
    
    def update_context(self, context_dict: Dict[str, Any]):
        """批量更新上下文"""
        with self.lock:
            self.context.update(context_dict)
    
    def clear_context(self):
        """清除上下文"""
        with self.lock:
            self.context.clear()
    
    def get_context(self) -> Dict[str, Any]:
        """获取当前上下文的副本"""
        with self.lock:
            return self.context.copy()
    
    def handle_exception(self, exception: Exception) -> Dict[str, Any]:
        """线程安全的异常处理"""
        context = self.get_context()
        return self.exception_handler.handle_exception(exception, context)
    
    def safe_execute(self, func: Callable, 
                    on_exception: Optional[Callable[[Exception], Any]] = None) -> Any:
        """线程安全的函数执行
        
        参数:
            func: 要执行的函数
            on_exception: 异常处理回调
            
        返回:
            函数执行结果
        """
        try:
            return func()
        except Exception as e:
            self.handle_exception(e)
            if on_exception:
                return on_exception(e)
            raise


# 全局单例
_exception_handler: Optional[ExceptionHandler] = None
_recovery_manager: Optional[RecoveryManager] = None
_singleton_lock = threading.Lock()


def get_exception_handler() -> ExceptionHandler:
    """获取全局异常处理器单例"""
    global _exception_handler
    if _exception_handler is None:
        with _singleton_lock:
            if _exception_handler is None:
                _exception_handler = ExceptionHandler()
    return _exception_handler


def get_recovery_manager() -> RecoveryManager:
    """获取全局恢复管理器单例"""
    global _recovery_manager
    if _recovery_manager is None:
        with _singleton_lock:
            if _recovery_manager is None:
                _recovery_manager = RecoveryManager(get_exception_handler())
    return _recovery_manager


def setup_recovery_system(system: Any) -> RecoveryManager:
    """设置系统的恢复机制
    
    参数:
        system: 分拣系统实例
        
    返回:
        恢复管理器实例
    """
    recovery_manager = get_recovery_manager()
    exception_handler = get_exception_handler()
    
    # 摄像头恢复策略
    def recover_camera(exception: Exception) -> bool:
        try:
            exception_handler.logger.info("尝试恢复摄像头...")
            if hasattr(system, 'detector'):
                system.detector.release_camera()
                time.sleep(1)
                success = system.detector.init_camera()
                if success:
                    exception_handler.logger.info("摄像头恢复成功")
                return success
            return False
        except Exception as e:
            exception_handler.handle_exception(e, {'component': 'camera', 'action': 'recovery'})
            return False
    
    # 串口恢复策略
    def recover_serial(exception: Exception) -> bool:
        try:
            exception_handler.logger.info("尝试恢复串口连接...")
            if hasattr(system, 'comm'):
                system.comm.disconnect()
                time.sleep(1)
                success = system.comm.connect(system.comm.current_port)
                if success:
                    exception_handler.logger.info("串口恢复成功")
                return success
            return False
        except Exception as e:
            exception_handler.handle_exception(e, {'component': 'serial', 'action': 'recovery'})
            return False
    
    recovery_manager.register_recovery_strategy('camera', recover_camera)
    recovery_manager.register_recovery_strategy('serial', recover_serial)
    
    return recovery_manager
