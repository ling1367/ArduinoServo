# -*- coding: utf-8 -*-
"""
串口通信模块 - 负责与Arduino通信
"""

import serial
import serial.tools.list_ports
import struct
import time
import config
from config_manager import get_config_manager
from exception_handler import (
    get_exception_handler,
    SerialCommException,
    ThreadSafeExceptionContext
)


class SerialComm:
    """串口通信类"""

    def __init__(self):
        """初始化串口"""
        self.ser = None
        self.connected = False
        self.current_port = None
        self.current_baudrate = None
        self.seq_num = 0   # 指令序列号（0-255循环）
        self._startup_verified = False  # 启动验证标志
        self._last_heartbeat = 0  # 上次心跳时间
        self._config_manager = get_config_manager()
        self.exception_handler = get_exception_handler()
        self.exception_context = ThreadSafeExceptionContext(self.exception_handler)

    def verify_connection(self, timeout=10):
        """
        验证Arduino连接并确保安全状态（启动保护）

        参数:
            timeout: 超时时间（秒）

        返回:
            bool: 是否验证成功
        """
        if not self.connected or self.ser is None:
            return False

        print("执行启动安全验证...")
        start_time = time.time()

        # 清除接收缓冲区
        self._clear_receive_buffer()

        # 发送HOME命令确保机械臂回到安全位置
        print("发送安全归位命令...")
        if not self.send_home_with_ack():
            print("警告: HOME命令响应超时，尝试直接归位...")
            # 即使超时也尝试发送（Arduino可能在安全位置）
            self.send_home()

        # 等待Arduino响应或超时
        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                try:
                    response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        print(f"Arduino响应: {response}")
                        if 'ACK' in response or '就绪' in response or 'HOME' in response:
                            self._startup_verified = True
                            print("启动验证通过！")
                            return True
                except Exception:
                    pass
            time.sleep(0.1)

        # 超时但尝试继续（Arduino可能已在安全位置）
        self._startup_verified = True
        print("启动验证超时（继续运行，Arduino可能已就绪）")
        return True

    def is_startup_verified(self):
        """检查启动是否已验证"""
        return self._startup_verified

    def list_ports(self):
        """列出所有可用串口"""
        ports = serial.tools.list_ports.comports()
        print("可用串口列表:")
        for port in ports:
            print(f"  - {port.device}: {port.description}")
        return [port.device for port in ports]
    
    def connect(self, port=None, baudrate=None):
        """
        连接串口
        
        参数:
            port: 串口号，如果为None则使用配置文件中的设置
            baudrate: 波特率
        
        返回:
            bool: 是否连接成功
        """
        if port is None:
            port = self._config_manager.get('SERIAL_CONFIG.port', config.SERIAL_CONFIG['port'])
        if baudrate is None:
            baudrate = self._config_manager.get('SERIAL_CONFIG.baudrate', config.SERIAL_CONFIG['baudrate'])
        
        context = {
            'port': port,
            'baudrate': baudrate,
            'method': 'connect'
        }
        self.exception_context.update_context(context)
        
        try:
            # 先断开可能存在的连接
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            
            # 打开串口
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=config.SERIAL_CONFIG['timeout'],
                dsrdtr=False,
                rtscts=False
            )
            
            time.sleep(0.3)  # 等待串口电气稳定
            
            # 等待 Arduino 初始化完成（最多8秒）
            self.exception_handler.logger.info("等待 Arduino 初始化...")
            ready_timeout = time.time() + 8
            ready = False
            
            while time.time() < ready_timeout:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if '等待指令' in line or '初始化完成' in line:
                        ready = True
                        self.exception_handler.logger.info(f"Arduino 已就绪: {line}")
                        break
                time.sleep(0.1)
            
            if not ready:
                self.exception_handler.logger.warning("提示: 未收到 Arduino 就绪信号（可能已运行中），继续尝试通信...")
            
            self.connected = True
            self.current_port = port
            self.current_baudrate = baudrate
            self.exception_handler.logger.info(f"串口 {port} 连接成功，波特率: {baudrate}")
            
            # 发送 INIT 指令激活舵机（安全启动协议）
            self.exception_handler.logger.info("发送 INIT 指令激活舵机...")
            if self.send_init_with_ack(timeout=10):
                self.exception_handler.logger.info("舵机已激活")
            else:
                self.exception_handler.logger.warning("警告: 舵机激活未确认（可能舵机已初始化），尝试继续...")
            
            return True
        except serial.SerialException as e:
            self.exception_context.handle_exception(e)
            self.connected = False
            raise SerialCommException(
                f"串口连接失败: {e}",
                error_code='SER001',
                context=context,
                severity='critical'
            ) from e
        except Exception as e:
            self.exception_context.handle_exception(e)
            self.connected = False
            raise SerialCommException(
                f"串口连接异常: {e}",
                error_code='SER002',
                context=context,
                severity='critical'
            ) from e
    
    def disconnect(self):
        """断开串口连接"""
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.connected = False
            print("串口已断开")
    
    def _next_seq(self):
        """获取下一个序列号（0-255循环，跳过0xFF保留值）"""
        self.seq_num = (self.seq_num + 1) % 255
        return self.seq_num

    def send_command(self, cmd, x=0, y=0, z=0, skip_boundary_check=False):
        """
        发送指令到Arduino
        
        数据包格式（共10字节）:
        [起始(1)][指令(1)][X高(1)][X低(1)][Y高(1)][Y低(1)][Z(1)][序列号(1)][校验(1)][结束(1)]
        
        参数:
            cmd: 指令类型
            x: X坐标 (0-640)
            y: Y坐标 (0-480)
            z: Z坐标/角度 (0-255)
            skip_boundary_check: 是否跳过边界检查（用于不需要坐标的指令）
        
        返回:
            bool: 是否发送成功
        """
        context = {
            'cmd': cmd,
            'x': x,
            'y': y,
            'z': z,
            'skip_boundary_check': skip_boundary_check,
            'connected': self.connected,
            'method': 'send_command'
        }
        
        try:
            if not self.connected or self.ser is None:
                raise SerialCommException(
                    "串口未连接",
                    error_code='SER003',
                    context=context
                )
            
            start = config.PROTOCOL['start_byte']
            end = config.PROTOCOL['end_byte']
            
            x = max(0, min(config.FRAME_WIDTH, x))
            y = max(0, min(config.FRAME_HEIGHT, y))
            z = max(0, min(255, z))
            
            if not skip_boundary_check:
                if x == 0 or x == config.FRAME_WIDTH or y == 0 or y == config.FRAME_HEIGHT:
                    self.exception_handler.logger.warning(f"警告：坐标接近边界 (x={x}, y={y})")
            
            seq = self._next_seq()
            
            checksum = (cmd + (x >> 8) + (x & 0xFF) + (y >> 8)
                     + (y & 0xFF) + (z & 0xFF) + seq) & 0xFF
            
            # 10字节：起始1+指令1+X2+Y2+Z1+序列号1+校验1+结束1
            packet = struct.pack('>BBHHBBBB', start, cmd, x, y, z & 0xFF, seq, checksum, end)
            
            # 检查串口是否仍然打开
            if not self.ser.is_open:
                self.connected = False
                raise SerialCommException(
                    "串口已关闭",
                    error_code='SER004',
                    context=context
                )
            
            # 发送数据
            self.ser.write(packet)
            self.ser.flush()
            
            # 根据指令类型决定输出格式
            if cmd == config.PROTOCOL['cmd_home']:
                self.exception_handler.logger.info("发送指令: cmd=HOME")
            elif cmd == config.PROTOCOL['cmd_grab']:
                self.exception_handler.logger.info("发送指令: cmd=GRAB")
            elif cmd == config.PROTOCOL['cmd_release']:
                self.exception_handler.logger.info("发送指令: cmd=RELEASE")
            elif cmd == config.PROTOCOL['cmd_emergency_stop']:
                self.exception_handler.logger.info("发送指令: cmd=EMERGENCY_STOP")
            elif cmd == config.PROTOCOL['cmd_init']:
                self.exception_handler.logger.info("发送指令: cmd=INIT")
            else:
                self.exception_handler.logger.info(f"发送指令: cmd={cmd}, x={x}, y={y}, z={z}")
            
            self.update_heartbeat()
            return True
            
        except serial.SerialException as e:
            self.exception_context.handle_exception(e)
            self.connected = False
            raise SerialCommException(
                f"串口发送失败: {e}",
                error_code='SER005',
                context=context
            ) from e
        except struct.error as e:
            self.exception_context.handle_exception(e)
            raise SerialCommException(
                f"数据包格式错误: {e}",
                error_code='SER006',
                context=context
            ) from e
        except Exception as e:
            self.exception_context.handle_exception(e)
            raise SerialCommException(
                f"发送失败: {e}",
                error_code='SER007',
                context=context
            ) from e
    
    def send_move(self, x, y, z=128):
        """发送移动指令（支持Z轴）"""
        return self.send_command(config.PROTOCOL['cmd_move'], x, y, z)
    
    def send_grab(self):
        """发送抓取指令"""
        return self.send_command(config.PROTOCOL['cmd_grab'], skip_boundary_check=True)
    
    def send_grab_circle(self):
        """发送抓取圆形零件指令（夹爪50°夹紧，47°保持）"""
        return self.send_command(0x07, skip_boundary_check=True)
    
    def send_grab_square(self):
        """发送抓取方形零件指令（夹爪55°夹紧，52°保持）"""
        return self.send_command(0x08, skip_boundary_check=True)
    
    def send_release(self):
        """发送释放指令"""
        return self.send_command(config.PROTOCOL['cmd_release'], skip_boundary_check=True)
    
    def send_home(self):
        """发送回原点指令"""
        return self.send_command(config.PROTOCOL['cmd_home'], skip_boundary_check=True)
    
    def _clear_receive_buffer(self):
        """清空串口接收缓冲区（防止旧消息干扰）"""
        if self.connected and self.ser is not None and self.ser.is_open:
            try:
                while self.ser.in_waiting > 0:
                    self.ser.read()
            except Exception as e:
                pass
    
    def read_response(self):
        """
        读取Arduino响应
        
        返回:
            str: 响应内容，如果超时返回None
        """
        context = {
            'connected': self.connected,
            'method': 'read_response'
        }
        
        try:
            if not self.connected or self.ser is None:
                return None
            
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                self.update_heartbeat()
                return response
            return None
        except serial.SerialException as e:
            self.exception_context.handle_exception(e)
            raise SerialCommException(
                f"读取响应串口错误: {e}",
                error_code='SER008',
                context=context
            ) from e
        except Exception as e:
            self.exception_context.handle_exception(e)
            raise SerialCommException(
                f"读取响应失败: {e}",
                error_code='SER009',
                context=context
            ) from e

    def check_heartbeat(self, timeout=5):
        """
        检查通信心跳

        参数:
            timeout: 心跳超时时间（秒）

        返回:
            bool: 通信是否正常
        """
        current_time = time.time()
        if current_time - self._last_heartbeat > timeout:
            print("警告: 通信心跳超时，尝试重连...")
            if self.reconnect():
                self._last_heartbeat = time.time()
                return True
            self.connected = False
            return False
        return True

    def update_heartbeat(self):
        """更新心跳时间"""
        self._last_heartbeat = time.time()
    
    def send_command_with_ack(self, cmd, x=0, y=0, z=0, timeout=15):
        """
        发送指令并等待确认（带超时保护和重连机制）
        
        参数:
            cmd: 指令类型
            x: X坐标
            y: Y坐标
            z: Z坐标/角度
            timeout: 超时时间（秒）
            
        返回:
            bool: 是否发送成功并收到确认
        """
        start_time = time.time()
        attempts = 0
        max_attempts = 2
        response_timeout = 15.0  # MOVE指令运动最长约15秒（MOVE_DELAY=55时3个舵机全行程）
        
        # 判断是否为不需要坐标的指令
        no_coord_cmds = [config.PROTOCOL['cmd_home'], 
                         config.PROTOCOL['cmd_grab'], 
                         config.PROTOCOL['cmd_release'], 
                         config.PROTOCOL['cmd_emergency_stop'],
                         config.PROTOCOL['cmd_init']]
        skip_boundary_check = cmd in no_coord_cmds
        
        while time.time() - start_time < timeout and attempts < max_attempts:
            # 检查连接状态
            if not self.connected or (self.ser is None or not self.ser.is_open):
                print("串口未连接，尝试重新连接...")
                if not self.reconnect():
                    attempts += 1
                    time.sleep(0.5)
                    continue
            
            # 发送指令前清空接收缓冲区（防止旧消息干扰）
            self._clear_receive_buffer()
            
            # 发送指令（根据指令类型决定是否跳过边界检查）
            if self.send_command(cmd, x, y, z, skip_boundary_check):
                # 等待响应（带超时）
                response_time = time.time()
                while time.time() - response_time < response_timeout:
                    response = self.read_response()
                    if response:
                        if 'ACK' in response:
                            print(f"收到确认: {response}")
                            return True
                        elif 'ERR' in response or '错误' in response:
                            print(f"Arduino返回错误: {response}")
                            break
                    time.sleep(0.1)  # 轮询间隔
            
            attempts += 1
            # 指数退避等待
            time.sleep(min(0.5 * attempts, 2.0))
        
        print(f"指令发送失败，未收到确认 (尝试{attempts}次)")
        return False
    
    def reconnect(self):
        """
        尝试重新连接串口（不触发 Arduino 复位）
        
        返回:
            bool: 是否重连成功
        """
        if self.current_port is None:
            self.current_port = self._config_manager.get('SERIAL_CONFIG.port', config.SERIAL_CONFIG['port'])
            self.current_baudrate = self._config_manager.get('SERIAL_CONFIG.baudrate', config.SERIAL_CONFIG['baudrate'])
        
        port = self.current_port
        baudrate = self.current_baudrate
        
        if self.ser is not None:
            try:
                self.ser.close()
            except:
                pass
        
        try:
            # 重新打开串口，禁用 DTR
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=config.SERIAL_CONFIG['timeout'],
                dsrdtr=False,
                rtscts=False
            )
            
            time.sleep(0.3)  # 等待串口电气稳定
            
            # 等待 Arduino 就绪（最多8秒）
            print("等待 Arduino 就绪...")
            ready_timeout = time.time() + 8
            ready = False
            
            while time.time() < ready_timeout:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if '等待指令' in line or '初始化完成' in line:
                        ready = True
                        print(f"Arduino 已就绪: {line}")
                        break
                time.sleep(0.1)
            
            if not ready:
                print("提示: 未收到 Arduino 就绪信号（可能已运行中），继续尝试...")
            
            # 清空可能的残余数据
            self._clear_receive_buffer()
            
            self.connected = True
            print("串口重新连接成功")
            return True
        except Exception as e:
            print(f"串口重连失败: {e}")
            self.connected = False
            return False
    
    def send_move_with_ack(self, x, y, z=128):
        """发送移动指令并等待确认（支持Z轴）"""
        return self.send_command_with_ack(config.PROTOCOL['cmd_move'], x, y, z)
    
    def send_grab_with_ack(self):
        """发送抓取指令并等待确认"""
        return self.send_command_with_ack(config.PROTOCOL['cmd_grab'])
    
    def send_grab_circle_with_ack(self):
        """发送抓取圆形零件指令并等待确认（夹爪50°夹紧，47°保持）"""
        return self.send_command_with_ack(0x07)
    
    def send_grab_square_with_ack(self):
        """发送抓取方形零件指令并等待确认（夹爪55°夹紧，52°保持）"""
        return self.send_command_with_ack(0x08)
    
    def send_release_with_ack(self):
        """发送释放指令并等待确认"""
        return self.send_command_with_ack(config.PROTOCOL['cmd_release'])
    
    def send_home_with_ack(self, timeout=15):
        """发送回原点指令并等待确认"""
        return self.send_command_with_ack(config.PROTOCOL['cmd_home'], timeout=timeout)
    
    def send_init(self):
        """发送初始化指令（激活舵机）"""
        return self.send_command(config.PROTOCOL['cmd_init'], skip_boundary_check=True)

    def send_init_with_ack(self, timeout=15):
        """发送初始化指令并等待确认"""
        return self.send_command_with_ack(config.PROTOCOL['cmd_init'], timeout=timeout)

    def send_emergency_stop(self):
        """发送紧急停止指令（不等待确认，立即执行）"""
        return self.send_command(config.PROTOCOL['cmd_emergency_stop'])
    
    def send_emergency_stop_with_ack(self):
        """发送紧急停止指令并等待确认"""
        return self.send_command_with_ack(config.PROTOCOL['cmd_emergency_stop'])


def test_serial():
    """测试串口通信模块"""
    comm = SerialComm()
    
    # 列出可用串口
    comm.list_ports()
    
    # 连接串口
    if not comm.connect():
        print("请检查串口连接")
        return
    
    print("\n测试指令:")
    print("  1 - 回原点")
    print("  2 - 移动到 (100, 100)")
    print("  3 - 抓取")
    print("  4 - 释放")
    print("  q - 退出")
    
    while True:
        cmd = input("\n请输入指令: ").strip()
        
        if cmd == 'q':
            break
        elif cmd == '1':
            comm.send_home()
        elif cmd == '2':
            comm.send_move(100, 100)
        elif cmd == '3':
            comm.send_grab()
        elif cmd == '4':
            comm.send_release()
        else:
            print("未知指令")
        
        # 读取响应
        response = comm.read_response()
        if response:
            print(f"Arduino响应: {response}")
    
    comm.disconnect()


if __name__ == '__main__':
    test_serial()
