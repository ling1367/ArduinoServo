# -*- coding: utf-8 -*-
"""
主程序 - 零件分拣机器人视觉系统
整合视觉识别和机械臂控制
"""

import cv2
import time
import threading
import config
from vision import VisionDetector
from serial_comm import SerialComm
from arm_control import ArmController
from sorting_state_machine import SortingStateMachine, SortState
from config_manager import get_config_manager
from exception_handler import (
    get_exception_handler,
    get_recovery_manager,
    setup_recovery_system,
    CameraException,
    SerialCommException,
    VisionException,
    StateMachineException,
    ThreadSafeExceptionContext
)


class SortingSystem:
    """分拣系统主类"""

    def __init__(self):
        """初始化系统"""
        self.config_manager = get_config_manager()
        self.detector = VisionDetector()
        self.comm = SerialComm()
        self.arm_controller = ArmController()
        self.state_machine = SortingStateMachine(self)
        self.running = False
        self.auto_mode = False
        self.sort_count = 0
        self.success_count = 0
        self._sorting = False
        self.emergency_stop = False
        self._lock = threading.Lock()
        
        # 异常处理相关
        self.exception_handler = get_exception_handler()
        self.exception_context = ThreadSafeExceptionContext(self.exception_handler)
        self.recovery_manager = None
        
    def init_system(self):
        """初始化整个系统"""
        context = {'method': 'init_system'}
        self.exception_context.update_context(context)
        
        try:
            self.exception_handler.logger.info("=" * 50)
            self.exception_handler.logger.info("零件分拣机器人视觉系统")
            self.exception_handler.logger.info("=" * 50)
            
            # 设置恢复系统
            self.recovery_manager = setup_recovery_system(self)
            
            # 初始化摄像头
            self.exception_handler.logger.info("\n[1/2] 初始化摄像头...")
            if not self.detector.init_camera():
                raise CameraException(
                    "摄像头初始化失败！",
                    error_code='INIT001',
                    context=context
                )
            
            # 初始化串口
            self.exception_handler.logger.info("\n[2/2] 初始化串口...")
            self.comm.list_ports()
            
            # 让用户选择串口
            port = input("请输入串口号（直接回车使用默认COM3）: ").strip()
            if port == "":
                port = self.config_manager.get('SERIAL_CONFIG.port', config.SERIAL_CONFIG['port'])
            
            try:
                if not self.comm.connect(port):
                    raise SerialCommException(
                        "串口连接失败！",
                        error_code='INIT002',
                        context={'port': port}
                    )
            except SerialCommException as e:
                self.exception_handler.logger.error("串口连接失败！")
                self.exception_handler.logger.error("请检查：")
                self.exception_handler.logger.error("  1. Arduino是否已连接电脑")
                self.exception_handler.logger.error("  2. 舵机电源是否已连接")
                self.exception_handler.logger.error("  3. 是否关闭了Arduino串口监视器")
                self.exception_handler.logger.error("  4. 串口号是否正确")
                self.detector.release_camera()
                return False  # 串口连接失败则终止程序

            # 启动安全验证：确保机械臂处于安全位置
            if not self.comm.verify_connection(timeout=15):
                self.exception_handler.logger.error("错误: 启动安全验证失败！")
                self.exception_handler.logger.error("请检查：")
                self.exception_handler.logger.error("  1. Arduino是否已正确连接")
                self.exception_handler.logger.error("  2. 舵机电源是否正常")
                self.exception_handler.logger.error("  3. 机械臂是否处于安全位置")
                self.comm.disconnect()
                return False

            self.exception_handler.logger.info("\n系统初始化完成！")
            
            # 发送HOME指令让机械臂归位
            self.exception_handler.logger.info("\n机械臂归位中...")
            try:
                if self.comm.send_home_with_ack(timeout=15):
                    self.exception_handler.logger.info("机械臂已归位")
                else:
                    self.exception_handler.logger.warning("警告: 归位超时，但程序继续运行")
            except SerialCommException as e:
                self.exception_handler.logger.warning("归位指令发送失败，但程序继续运行")
            
            return True
        except Exception as e:
            self.exception_context.handle_exception(e)
            raise
    
    def sort_part(self, part):
        """执行单个零件的分拣"""
        context = {
            'part': {'color': part['color'], 'shape': part['shape'], 'x': part['x'], 'y': part['y']},
            'method': 'sort_part'
        }
        self.exception_context.update_context(context)
        
        if self.emergency_stop:
            self.exception_handler.logger.warning("紧急停止中，无法执行分拣")
            return False

        with self._lock:
            if self._sorting:
                self.exception_handler.logger.warning("分拣正在进行中，请求被忽略")
                return False
            self._sorting = True

        self.exception_handler.logger.info(f"\n开始分拣: {part['color']} {part['shape']}")
        with self._lock:
            self.sort_count += 1

        try:
            success = self.state_machine.start_sorting(part)
            if not success:
                state_info = self.state_machine.get_state_info()
                if state_info['state'] == SortState.EMERGENCY_STOPPED:
                    self.exception_handler.logger.warning("  紧急停止：分拣任务取消")
                elif state_info['state'] == SortState.ERROR:
                    self.exception_handler.logger.error(f"  错误：{state_info['error']}")
                    try:
                        self.comm.send_home()
                    except Exception:
                        pass
                return False
            with self._lock:
                self.success_count += 1
            self.exception_handler.logger.info(f"分拣完成！总计: {self.sort_count}次，成功: {self.success_count}次")
            return True
        except Exception as e:
            self.exception_context.handle_exception(e)
            self.exception_handler.logger.error(f"  错误：分拣过程中发生异常 - {e}")
            try:
                self.comm.send_home()
            except Exception:
                pass
            return False
        finally:
            with self._lock:
                self._sorting = False
    
    def auto_sort(self):
        """自动分拣模式"""
        context = {'method': 'auto_sort'}
        self.exception_context.update_context(context)
        
        self.exception_handler.logger.info("\n进入自动分拣模式...")

        capture_fail_count = 0
        
        try:
            while True:
                with self._lock:
                    if not self.auto_mode or not self.running or self.emergency_stop:
                        break
                    connected = self.comm.connected

                if not connected:
                    self.exception_handler.logger.warning("自动分拣暂停：串口未连接")
                    
                    # 尝试恢复串口
                    if self.recovery_manager:
                        self.recovery_manager.attempt_recovery('serial', SerialCommException("串口断开", context=context))
                    
                    time.sleep(1)
                    continue

                try:
                    if not self.detector.capture_frame():
                        capture_fail_count += 1
                        if capture_fail_count > 5:
                            self.exception_handler.logger.warning("摄像头连续失败，尝试恢复...")
                            if self.recovery_manager:
                                self.recovery_manager.attempt_recovery('camera', CameraException("摄像头读取失败", context=context))
                            capture_fail_count = 0
                        time.sleep(0.1)
                        continue
                    
                    capture_fail_count = 0
                    results = self.detector.detect_parts()
                    
                    if results:
                        self.sort_part(results[0])
                        time.sleep(2)
                    else:
                        time.sleep(0.1)
                except CameraException as e:
                    self.exception_handler.handle_exception(e, context)
                    if self.recovery_manager:
                        self.recovery_manager.attempt_recovery('camera', e)
                    time.sleep(1)
                except SerialCommException as e:
                    self.exception_handler.handle_exception(e, context)
                    if self.recovery_manager:
                        self.recovery_manager.attempt_recovery('serial', e)
                    time.sleep(1)
                except VisionException as e:
                    self.exception_handler.handle_exception(e, context)
                    time.sleep(0.5)
                except Exception as e:
                    self.exception_context.handle_exception(e)
                    time.sleep(0.5)

            if self.emergency_stop:
                self.exception_handler.logger.warning("自动分拣模式：紧急停止")
        except Exception as e:
            self.exception_context.handle_exception(e)
            self.auto_mode = False
            try:
                self.comm.send_home()
            except Exception:
                pass
    
    def reset_emergency_stop(self):
        """重置紧急停止状态"""
        with self._lock:
            self.emergency_stop = False
        print("紧急停止已重置，可以继续操作")
    
    def run(self):
        """运行主循环"""
        self.running = True

        print("\n" + "=" * 50)
        print("操作说明:")
        print("  q - 退出程序")
        print("  s - 手动分拣当前检测到的零件")
        print("  a - 切换自动分拣模式")
        print("  h - 机械臂回原点")
        print("  c - 拍照保存")
        print("  e - 紧急停止")
        print("  r - 重置紧急停止")
        print("=" * 50)

        auto_thread = None
        fps_start_time = time.time()
        fps_frame_count = 0
        fps_value = 0

        capture_fail_count = 0

        while self.running:
            if not self.detector.capture_frame():
                capture_fail_count += 1
                if capture_fail_count > 50:
                    print("摄像头连续读取失败，程序退出")
                    break
                time.sleep(0.05)
                continue
            capture_fail_count = 0

            results = self.detector.detect_parts()
            frame = self.detector.draw_results()

            if self.emergency_stop:
                cv2.putText(frame, "EMERGENCY STOP", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, "Press 'r' to reset", (10, 55),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                status_text = f"Mode: {'AUTO' if self.auto_mode else 'MANUAL'} | Parts: {len(results)}"
                cv2.putText(frame, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if self._sorting:
                cv2.putText(frame, "分拣中... 按 e 紧急停止", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            count_text = f"Sorted: {self.sort_count} | Success: {self.success_count}"
            cv2.putText(frame, count_text, (10, 55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if not self.comm.connected:
                cv2.putText(frame, "SERIAL DISCONNECTED", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 计算并显示FPS
            fps_frame_count += 1
            elapsed = time.time() - fps_start_time
            if elapsed >= 1.0:
                fps_value = fps_frame_count / elapsed
                fps_start_time = time.time()
                fps_frame_count = 0
            cv2.putText(frame, f"FPS: {fps_value:.1f}", (frame.shape[1] - 120, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            window_name = self.config_manager.get('DISPLAY.window_name', config.DISPLAY['window_name'])
            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.running = False
                self.auto_mode = False

            elif key == ord('s'):
                if not self.comm.connected:
                    print("串口未连接，无法执行分拣")
                elif self._sorting:
                    print("正在分拣中，请等待完成")
                elif results:
                    threading.Thread(target=lambda: self.sort_part(results[0]), daemon=True).start()
                else:
                    print("未检测到零件")

            elif key == ord('a'):
                with self._lock:
                    self.auto_mode = not self.auto_mode
                    auto_on = self.auto_mode
                if auto_on:
                    print("自动分拣模式: 开启")
                    if auto_thread is None or not auto_thread.is_alive():
                        auto_thread = threading.Thread(target=self.auto_sort)
                        auto_thread.daemon = True
                        auto_thread.start()
                    else:
                        print("自动分拣线程已在运行")
                else:
                    print("自动分拣模式: 关闭")

            elif key == ord('h'):
                if self.comm.send_home_with_ack():
                    print("机械臂回原点")
                else:
                    print("机械臂回原点失败")
                    print("提示：请检查舵机电源是否连接，串口是否正常，Arduino是否已上传代码")

            elif key == ord('c'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"capture_{timestamp}.png"
                cv2.imwrite(filename, frame)
                print(f"截图已保存: {filename}")

            elif key == ord('e'):
                with self._lock:
                    self.emergency_stop = True
                    self.auto_mode = False
                print("紧急停止：所有操作已停止")
                try:
                    self.comm.send_emergency_stop()
                except Exception:
                    pass

            elif key == ord('r'):
                self.reset_emergency_stop()

        if auto_thread is not None:
            self.auto_mode = False
            auto_thread.join(timeout=1)

        self.detector.release_camera()
        self.comm.disconnect()
        cv2.destroyAllWindows()

        print("\n程序已退出")
        print(f"统计: 总分拣 {self.sort_count} 次，成功 {self.success_count} 次")


def main():
    """主函数"""
    system = SortingSystem()
    if system.init_system():
        system.run()


if __name__ == '__main__':
    main()
