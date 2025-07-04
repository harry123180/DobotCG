#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main.py - 機械臂主控制器 (修正指令觸發穩定性版)
修正Flow3/4指令佇列競爭問題，使用專用佇列確保指令穩定執行
"""

import json
import os
import time
import threading
import traceback
import queue
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum, IntEnum
import logging

# 導入新流程架構模組
from Dobot_Flow1 import Flow1VisionPickExecutor
from Dobot_Flow2 import Flow2UnloadExecutor  
from Dobot_Flow3 import FlowFlipStationExecutor
from Dobot_Flow4 import Flow4VibrationFeedExecutor

# 導入高階API模組
from CCD1HighLevel import CCD1HighLevelAPI
from GripperHighLevel import GripperHighLevelAPI, GripperType
from AngleHighLevel import AngleHighLevel

from pymodbus.client.tcp import ModbusTcpClient
from dobot_api import DobotApiDashboard, DobotApiMove

# 配置常數
DOBOT_BASE_ADDR = 400
CONFIG_FILE = "dobot_config.json"

# ==================== 指令系統 ====================

class CommandType(Enum):
    """指令類型"""
    MOTION = "motion"
    DIO_FLIP = "dio_flip"
    DIO_VIBRATION = "dio_vibration"
    EXTERNAL = "external"
    EMERGENCY = "emergency"

class CommandPriority(IntEnum):
    """指令優先權 (數值越小優先權越高)"""
    EMERGENCY = 0
    MOTION = 1
    DIO_FLIP = 2
    DIO_VIBRATION = 2
    EXTERNAL = 3

@dataclass
class Command:
    """統一指令格式"""
    command_type: CommandType
    command_data: Dict[str, Any]
    priority: CommandPriority
    timestamp: float = field(default_factory=time.time)
    command_id: int = field(default=0)
    callback: Optional[callable] = None

    def __lt__(self, other):
        """優先權比較 (for PriorityQueue)"""
        if self.priority != other.priority:
            return self.priority < other.priority
        return self.timestamp < other.timestamp

# ==================== 專用指令佇列系統 ====================

class DedicatedCommandQueue:
    """專用指令佇列 - 每個執行緒使用專用佇列避免競爭"""
    
    def __init__(self, name: str, max_size: int = 50):
        self.name = name
        self.queue = queue.Queue(max_size)
        self.command_id_counter = 1
        self._lock = threading.Lock()
        self.put_count = 0
        self.get_count = 0
        
    def put_command(self, command: Command) -> bool:
        """加入指令到專用佇列"""
        try:
            with self._lock:
                command.command_id = self.command_id_counter
                self.command_id_counter += 1
                
            self.queue.put_nowait(command)
            self.put_count += 1
            
            print(f"[{self.name}Queue] 指令已加入 - ID:{command.command_id}, 類型:{command.command_type.value}, 佇列大小:{self.queue.qsize()}")
            return True
            
        except queue.Full:
            print(f"[{self.name}Queue] 佇列已滿，丟棄指令: {command.command_type}")
            return False
        except Exception as e:
            print(f"[{self.name}Queue] 加入指令失敗: {e}")
            return False
            
    def get_command(self, timeout: Optional[float] = None) -> Optional[Command]:
        """取得指令"""
        try:
            command = self.queue.get(timeout=timeout)
            self.get_count += 1
            
            if command:
                print(f"[{self.name}Queue] 指令已取出 - ID:{command.command_id}, 類型:{command.command_type.value}, 剩餘:{self.queue.qsize()}")
            
            return command
            
        except queue.Empty:
            return None
        except Exception as e:
            print(f"[{self.name}Queue] 取得指令失敗: {e}")
            return None
            
    def size(self) -> int:
        """取得佇列大小"""
        return self.queue.qsize()
        
    def get_stats(self) -> Dict[str, int]:
        """取得統計信息"""
        return {
            'current_size': self.size(),
            'total_put': self.put_count,
            'total_get': self.get_count,
            'pending': self.put_count - self.get_count
        }

# ==================== 機械臂狀態機 ====================

class DobotStateMachine:
    """機械臂狀態機管理"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        self.modbus_client = modbus_client
        self.base_addr = DOBOT_BASE_ADDR
        
    def set_ready(self, ready: bool = True):
        """設置Ready狀態 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.read_holding_registers(address=self.base_addr, count=1)
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取狀態寄存器失敗: {result}")
                return
                
            current = result.registers[0]
            if ready:
                current |= 0x01
            else:
                current &= ~0x01
            self.modbus_client.write_register(address=self.base_addr, value=current)
        except Exception as e:
            print(f"設置Ready狀態失敗: {e}")
            
    def set_running(self, running: bool = True):
        """設置Running狀態 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.read_holding_registers(address=self.base_addr, count=1)
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取狀態寄存器失敗: {result}")
                return
                
            current = result.registers[0]
            if running:
                current |= 0x02
                current &= ~0x01
            else:
                current &= ~0x02
            self.modbus_client.write_register(address=self.base_addr, value=current)
        except Exception as e:
            print(f"設置Running狀態失敗: {e}")
            
    def set_alarm(self, alarm: bool = True):
        """設置Alarm狀態 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.read_holding_registers(address=self.base_addr, count=1)
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取狀態寄存器失敗: {result}")
                return
                
            current = result.registers[0]
            if alarm:
                current |= 0x04
                current &= ~0x03
            else:
                current &= ~0x04
            self.modbus_client.write_register(address=self.base_addr, value=current)
        except Exception as e:
            print(f"設置Alarm狀態失敗: {e}")
            
    def set_current_flow(self, flow_id: int):
        """設置當前流程ID - PyModbus 3.9.2修正版"""
        try:
            self.modbus_client.write_register(address=self.base_addr + 2, value=flow_id)
        except Exception as e:
            print(f"設置流程ID失敗: {e}")
            
    def set_flow1_complete(self, complete: bool = True):
        """設置Flow1完成狀態 - PyModbus 3.9.2修正版"""
        try:
            value = 1 if complete else 0
            self.modbus_client.write_register(address=self.base_addr + 20, value=value)
        except Exception as e:
            print(f"設置Flow1完成狀態失敗: {e}")

# ==================== 真實機械臂控制器 ====================


class RealRobotController:
    """真實機械臂控制器 - 修正運動完成檢查版本"""
    
    def __init__(self, ip: str, dashboard_port: int = 29999, move_port: int = 30003):
        self.ip = ip
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.is_connected = False
        self.dashboard_api = None
        self.move_api = None
        self.global_speed = 50
        
    def _parse_api_response(self, response: str) -> bool:
        """解析API響應"""
        if not response:
            return False
        try:
            parts = response.strip().split(',')
            if len(parts) >= 1:
                error_code = int(parts[0])
                return error_code == 0
            return False
        except (ValueError, IndexError):
            return False
    
    def _extract_mode_from_response(self, response: str) -> Optional[int]:
        """從RobotMode響應中提取模式值"""
        try:
            if not response:
                return None
            
            # 處理格式如: "0,{4},RobotMode();" 或 "0,4,RobotMode();"
            parts = response.strip().split(',')
            if len(parts) >= 2:
                mode_part = parts[1].strip()
                
                # 移除花括號
                if mode_part.startswith('{') and mode_part.endswith('}'):
                    mode_part = mode_part[1:-1]
                
                return int(mode_part)
            return None
        except (ValueError, IndexError):
            return None
        
    def initialize(self) -> bool:
        """初始化機械臂連接"""
        try:
            self.dashboard_api = DobotApiDashboard(self.ip, self.dashboard_port)
            self.move_api = DobotApiMove(self.ip, self.move_port)
            
            clear_result = self.dashboard_api.ClearError()
            if self._parse_api_response(clear_result):
                print("✓ 清除錯誤成功")
            else:
                print(f"清除錯誤失敗: {clear_result}")
                
            enable_result = self.dashboard_api.EnableRobot()
            if self._parse_api_response(enable_result):
                print("✓ 機械臂啟用成功")
            else:
                print(f"機械臂啟用失敗: {enable_result}")
                return False
            
            # 等待機械臂就緒
            time.sleep(2.0)
            
            if self.set_global_speed(self.global_speed):
                print(f"✓ 初始速度設定成功: {self.global_speed}%")
            else:
                print(f"⚠️ 初始速度設定失敗")
            
            self.is_connected = True
            print(f"✓ 機械臂初始化成功: {self.ip}")
            return True
            
        except Exception as e:
            print(f"機械臂初始化失敗: {e}")
            return False
    
    def set_global_speed(self, speed_percent: int) -> bool:
        """設定全局速度"""
        try:
            if not 1 <= speed_percent <= 100:
                print(f"速度超出範圍: {speed_percent}")
                return False
                
            result = self.dashboard_api.SpeedFactor(speed_percent)
            success = self._parse_api_response(result)
            if success:
                self.global_speed = speed_percent
                print(f"✓ 全局速度設定成功: {speed_percent}%")
            else:
                print(f"全局速度設定失敗: {result}")
            return success
        except Exception as e:
            print(f"設定全局速度異常: {e}")
            return False
    
    def move_j(self, x: float, y: float, z: float, r: float) -> bool:
        """關節運動 - 修正版"""
        try:
            print(f"開始MovJ: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
            
            # 發送運動指令
            result = self.move_api.MovJ(x, y, z, r)
            success = self._parse_api_response(result)
            
            if not success:
                print(f"✗ MovJ指令發送失敗: {result}")
                return False
            
            print(f"MovJ指令發送成功，等待運動完成...")
            
            # 等待運動完成
            if self._wait_for_motion_complete():
                print(f"✓ MovJ完成: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                return True
            else:
                print(f"✗ MovJ超時或失敗: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                return False
                
        except Exception as e:
            print(f"MovJ執行異常: {e}")
            return False
    
    def move_l(self, x: float, y: float, z: float, r: float) -> bool:
        """直線運動 - 修正版"""
        try:
            print(f"開始MovL: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
            
            # 發送運動指令
            result = self.move_api.MovL(x, y, z, r)
            success = self._parse_api_response(result)
            
            if not success:
                print(f"✗ MovL指令發送失敗: {result}")
                return False
            
            print(f"MovL指令發送成功，等待運動完成...")
            
            # 等待運動完成
            if self._wait_for_motion_complete():
                print(f"✓ MovL完成: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                return True
            else:
                print(f"✗ MovL超時或失敗: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                return False
                
        except Exception as e:
            print(f"MovL執行異常: {e}")
            return False
    
    def _wait_for_motion_complete(self, timeout: float = 30.0) -> bool:
        """等待運動完成 - 修正回原本邏輯"""
        try:
            start_time = time.time()
            last_mode = None
            stable_count = 0
            required_stable_checks = 3  # 需要連續3次檢查都是模式5
            
            print("等待運動完成...")
            
            while time.time() - start_time < timeout:
                # 獲取機械臂模式
                result = self.dashboard_api.RobotMode()
                
                if not self._parse_api_response(result):
                    print(f"⚠️ 無法獲取機械臂模式: {result}")
                    time.sleep(0.1)
                    continue
                
                current_mode = self._extract_mode_from_response(result)
                
                if current_mode is None:
                    print(f"⚠️ 無法解析機械臂模式: {result}")
                    time.sleep(0.1)
                    continue
                
                # 根據dobot_api專案知識：mode 5 表示運動完成(IDLE狀態)
                if current_mode == 5:
                    if last_mode == current_mode:
                        stable_count += 1
                        if stable_count >= required_stable_checks:
                            print(f"✓ 機械臂運動完成 (模式: {current_mode})")
                            return True
                    else:
                        stable_count = 1
                        print(f"機械臂進入完成狀態 (模式: {current_mode})")
                else:
                    stable_count = 0
                    if current_mode != last_mode:  # 只在狀態改變時輸出
                        if current_mode == 7:
                            print(f"機械臂運動準備中 (模式: {current_mode})")
                        else:
                            print(f"機械臂狀態: {current_mode}")
                
                last_mode = current_mode
                time.sleep(0.1)  # 檢查間隔
            
            print(f"✗ 等待運動完成超時 ({timeout}秒), 最後模式: {last_mode}")
            print("可能原因：運動指令發送失敗或機械臂未進入IDLE狀態")
            return False
            
        except Exception as e:
            print(f"等待運動完成檢查異常: {e}")
            return False
    
    def sync(self) -> bool:
        """同步等待所有運動完成"""
        try:
            if self.move_api:
                result = self.move_api.Sync()
                return self._parse_api_response(result)
            return False
        except Exception as e:
            print(f"同步等待失敗: {e}")
            return False
    
    def set_do(self, pin: int, value: int) -> bool:
        """設定數位輸出"""
        try:
            result = self.dashboard_api.DOExecute(pin, value)
            success = self._parse_api_response(result)
            if success:
                print(f"✓ DO{pin}設定為{value}")
            else:
                print(f"✗ DO{pin}設定失敗: {result}")
            return success
        except Exception as e:
            print(f"設定DO失敗: {e}")
            return False
    
    def get_di(self, pin: int) -> Optional[int]:
        """讀取數位輸入"""
        try:
            result = self.dashboard_api.DI(pin)
            if self._parse_api_response(result):
                parts = result.strip().split(',')
                if len(parts) >= 2:
                    di_part = parts[1].strip()
                    # 移除花括號
                    if di_part.startswith('{') and di_part.endswith('}'):
                        di_part = di_part[1:-1]
                    return int(di_part)
            return None
        except Exception as e:
            print(f"讀取DI失敗: {e}")
            return None
    
    def emergency_stop(self) -> bool:
        """緊急停止"""
        try:
            result = self.dashboard_api.EmergencyStop()
            success = self._parse_api_response(result)
            if success:
                print("✓ 緊急停止執行成功")
            else:
                print(f"緊急停止執行失敗: {result}")
            return success
        except Exception as e:
            print(f"緊急停止失敗: {e}")
            return False
    
    def get_current_pose(self) -> Optional[Dict[str, float]]:
        """獲取當前位置"""
        try:
            result = self.dashboard_api.GetPose()
            if self._parse_api_response(result):
                # 解析位置數據
                parts = result.strip().split(',')
                if len(parts) >= 5:
                    return {
                        'x': float(parts[1]),
                        'y': float(parts[2]), 
                        'z': float(parts[3]),
                        'r': float(parts[4])
                    }
            return None
        except Exception as e:
            print(f"獲取位置失敗: {e}")
            return None
    
    def joint_move_j(self, j1: float, j2: float, j3: float, j4: float) -> bool:
        """關節角度運動 - 使用JointMovJ"""
        try:
            print(f"開始JointMovJ: (j1:{j1:.1f}, j2:{j2:.1f}, j3:{j3:.1f}, j4:{j4:.1f})")
            
            # 發送關節運動指令
            result = self.move_api.JointMovJ(j1, j2, j3, j4)
            success = self._parse_api_response(result)
            
            if not success:
                print(f"✗ JointMovJ指令發送失敗: {result}")
                return False
            
            print(f"JointMovJ指令發送成功，等待運動完成...")
            
            # 等待運動完成
            if self._wait_for_motion_complete():
                print(f"✓ JointMovJ完成: (j1:{j1:.1f}, j2:{j2:.1f}, j3:{j3:.1f}, j4:{j4:.1f})")
                return True
            else:
                print(f"✗ JointMovJ超時或失敗: (j1:{j1:.1f}, j2:{j2:.1f}, j3:{j3:.1f}, j4:{j4:.1f})")
                return False
                
        except Exception as e:
            print(f"JointMovJ執行異常: {e}")
            return False
    
    def disconnect(self) -> bool:
        """斷開機械臂連接"""
        try:
            if self.dashboard_api:
                disable_result = self.dashboard_api.DisableRobot()
                if self._parse_api_response(disable_result):
                    print("✓ 機械臂已停用")
                else:
                    print(f"⚠️ 機械臂停用失敗: {disable_result}")
                self.dashboard_api.close()
            if self.move_api:
                self.move_api.close()
            self.is_connected = False
            return True
        except Exception as e:
            print(f"機械臂斷開連接失敗: {e}")
            return False

# ==================== 執行緒基類 ====================

class BaseFlowThread(threading.Thread):
    """執行緒基類"""
    
    def __init__(self, name: str, command_queue: DedicatedCommandQueue):
        super().__init__(daemon=True, name=name)
        self.command_queue = command_queue
        self.running = False
        self.status = "停止"
        self.last_error = ""
        self.operation_count = 0
        
    def start_thread(self):
        """啟動執行緒"""
        self.running = True
        self.start()
        
    def stop_thread(self):
        """停止執行緒"""
        self.running = False
        
    def get_status(self) -> Dict[str, Any]:
        """取得執行緒狀態"""
        return {
            'name': self.name,
            'running': self.running,
            'status': self.status,
            'last_error': self.last_error,
            'operation_count': self.operation_count,
            'queue_stats': self.command_queue.get_stats()
        }

# ==================== 運動控制執行緒 ====================

class MotionFlowThread(BaseFlowThread):
    """運動控制執行緒 - 只處理Flow1和Flow2"""
    
    def __init__(self, robot: RealRobotController, command_queue: DedicatedCommandQueue, 
                 state_machine: DobotStateMachine, external_modules: Dict):
        super().__init__("MotionFlow", command_queue)
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        self.flow_executors = {}
        self.current_flow = None
        
    def initialize_flows(self):
        """初始化Flow執行器"""
        try:
            # Flow1: VP視覺抓取
            flow1 = Flow1VisionPickExecutor()
            flow1.initialize(self.robot, self.state_machine, self.external_modules)
            self.flow_executors[1] = flow1
            
            # Flow2: CV出料流程
            flow2 = Flow2UnloadExecutor()
            flow2.initialize(self.robot, self.state_machine, self.external_modules)
            self.flow_executors[2] = flow2
            
            print("✓ Motion Flow執行器初始化完成 (Flow1, Flow2)")
            
        except Exception as e:
            print(f"Motion Flow執行器初始化失敗: {e}")
            self.last_error = str(e)
    
    def run(self):
        """運動控制執行緒主循環"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.MOTION:
                    print(f"[Motion] 收到運動指令，ID: {command.command_id}")
                    self._handle_motion_command(command)
                    
            except Exception as e:
                self.last_error = f"運動控制執行緒錯誤: {e}"
                print(f"[Motion] {self.last_error}")
                traceback.print_exc()
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
    def _handle_motion_command(self, command: Command):
        """處理運動指令"""
        try:
            cmd_data = command.command_data
            cmd_type = cmd_data.get('type', '')
            
            if cmd_type == 'flow_vp_vision_pick':
                self._execute_flow1()
            elif cmd_type == 'flow_unload':
                self._execute_flow2()
            else:
                print(f"[Motion] 未知運動指令類型: {cmd_type}")
                
            self.operation_count += 1
            
        except Exception as e:
            self.last_error = f"處理運動指令失敗: {e}"
            print(f"[Motion] {self.last_error}")
    
    def _execute_flow1(self):
        """執行Flow1"""
        try:
            print("[Motion] 開始執行Flow1 - VP視覺抓取")
            self.state_machine.set_running(True)
            self.state_machine.set_current_flow(1)
            
            flow1 = self.flow_executors.get(1)
            if flow1:
                result = flow1.execute()
                
                if result.success:
                    print("[Motion] ✓ Flow1執行成功")
                    self.state_machine.set_flow1_complete(True)
                    self.state_machine.set_running(False)
                    self.state_machine.set_current_flow(0)
                    self.state_machine.set_ready(True)
                else:
                    print(f"[Motion] ✗ Flow1執行失敗: {result.error_message}")
                    self.state_machine.set_alarm(True)
                    self.state_machine.set_running(False)
                    self.state_machine.set_current_flow(0)
            else:
                print("[Motion] ✗ Flow1執行器未初始化")
                self.state_machine.set_alarm(True)
                
        except Exception as e:
            print(f"[Motion] Flow1執行異常: {e}")
            self.state_machine.set_alarm(True)
            self.state_machine.set_running(False)
            self.state_machine.set_current_flow(0)
    
    def _execute_flow2(self):
        """執行Flow2"""
        try:
            print("[Motion] 開始執行Flow2 - CV出料流程")
            self.state_machine.set_running(True)
            self.state_machine.set_current_flow(2)
            
            flow2 = self.flow_executors.get(2)
            if flow2:
                result = flow2.execute()
                
                if result.success:
                    print("[Motion] ✓ Flow2執行成功")
                    self.state_machine.set_running(False)
                    self.state_machine.set_current_flow(0)
                    self.state_machine.set_ready(True)
                else:
                    print(f"[Motion] ✗ Flow2執行失敗: {result.error_message}")
                    self.state_machine.set_alarm(True)
                    self.state_machine.set_running(False)
                    self.state_machine.set_current_flow(0)
            else:
                print("[Motion] ✗ Flow2執行器未初始化")
                self.state_machine.set_alarm(True)
                
        except Exception as e:
            print(f"[Motion] Flow2執行異常: {e}")
            self.state_machine.set_alarm(True)
            self.state_machine.set_running(False)
            self.state_machine.set_current_flow(0)

# ==================== Flow3翻轉站專用執行緒 ====================

class Flow3FlipStationThread(BaseFlowThread):
    """Flow3翻轉站控制專用執行緒 - 修正指令接收穩定性"""
    
    def __init__(self, robot: RealRobotController, command_queue: DedicatedCommandQueue):
        super().__init__("Flow3FlipStation", command_queue)
        self.robot = robot
        self.flow3_executor = None
        
    def initialize_flows(self):
        """初始化Flow3執行器"""
        try:
            flow3 = FlowFlipStationExecutor()
            flow3.initialize(self.robot, None, {})
            self.flow3_executor = flow3
            print("✓ Flow3翻轉站執行器初始化完成")
        except Exception as e:
            print(f"Flow3執行器初始化失敗: {e}")
            self.last_error = str(e)
    
    def run(self):
        """Flow3執行緒主循環 - 修正指令處理穩定性"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動，專用佇列接收DIO_FLIP指令")
        
        while self.running:
            try:
                # 修正：使用更長的timeout確保能接收到指令
                command = self.command_queue.get_command(timeout=0.2)
                
                if command:
                    print(f"[Flow3] 收到指令 - ID:{command.command_id}, 類型:{command.command_type.value}")
                    
                    # 檢查指令類型
                    if command.command_type == CommandType.DIO_FLIP:
                        cmd_type = command.command_data.get('type', '')
                        if cmd_type == 'flow_flip_station':
                            print(f"[Flow3] 開始處理翻轉站指令，ID: {command.command_id}")
                            self._execute_flip_station()
                        else:
                            print(f"[Flow3] 未知指令子類型: {cmd_type}")
                    else:
                        print(f"[Flow3] 收到非DIO_FLIP指令，忽略: {command.command_type}")
                        
            except Exception as e:
                self.last_error = f"Flow3執行緒錯誤: {e}"
                print(f"[Flow3] {self.last_error}")
                traceback.print_exc()
                time.sleep(0.1)  # 錯誤後短暫休息
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
    def _execute_flip_station(self):
        """執行翻轉站控制"""
        try:
            print("[Flow3] === 開始執行翻轉站控制 ===")
            start_time = time.time()
            
            if not self.flow3_executor:
                print("[Flow3] ✗ Flow3執行器未初始化")
                return
            
            # 執行Flow3
            result = self.flow3_executor.execute()
            
            execution_time = time.time() - start_time
            
            if result.success:
                print(f"[Flow3] ✓ 翻轉站控制執行成功，耗時: {execution_time:.2f}秒")
                print(f"[Flow3] 完成步驟: {result.steps_completed}/{result.total_steps}")
            else:
                print(f"[Flow3] ✗ 翻轉站控制執行失敗: {result.error_message}")
                print(f"[Flow3] 完成步驟: {result.steps_completed}/{result.total_steps}")
                
            self.operation_count += 1
            print("[Flow3] === 翻轉站控制執行完成 ===")
                
        except Exception as e:
            print(f"[Flow3] 翻轉站控制執行異常: {e}")
            traceback.print_exc()

# ==================== Flow4震動投料專用執行緒 ====================

class Flow4VibrationFeedThread(BaseFlowThread):
    """Flow4震動投料控制專用執行緒 - 修正指令接收穩定性"""
    
    def __init__(self, robot: RealRobotController, command_queue: DedicatedCommandQueue):
        super().__init__("Flow4VibrationFeed", command_queue)
        self.robot = robot
        self.flow4_executor = None
        
    def initialize_flows(self):
        """初始化Flow4執行器"""
        try:
            flow4 = Flow4VibrationFeedExecutor()
            flow4.initialize(self.robot, None, {})
            self.flow4_executor = flow4
            print("✓ Flow4震動投料執行器初始化完成")
        except Exception as e:
            print(f"Flow4執行器初始化失敗: {e}")
            self.last_error = str(e)
    
    def run(self):
        """Flow4執行緒主循環 - 修正指令處理穩定性"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動，專用佇列接收DIO_VIBRATION指令")
        
        while self.running:
            try:
                # 修正：使用更長的timeout確保能接收到指令
                command = self.command_queue.get_command(timeout=0.2)
                
                if command:
                    print(f"[Flow4] 收到指令 - ID:{command.command_id}, 類型:{command.command_type.value}")
                    
                    # 檢查指令類型
                    if command.command_type == CommandType.DIO_VIBRATION:
                        cmd_type = command.command_data.get('type', '')
                        if cmd_type == 'flow_vibration_feed':
                            print(f"[Flow4] 開始處理震動投料指令，ID: {command.command_id}")
                            self._execute_vibration_feed()
                        else:
                            print(f"[Flow4] 未知指令子類型: {cmd_type}")
                    else:
                        print(f"[Flow4] 收到非DIO_VIBRATION指令，忽略: {command.command_type}")
                        
            except Exception as e:
                self.last_error = f"Flow4執行緒錯誤: {e}"
                print(f"[Flow4] {self.last_error}")
                traceback.print_exc()
                time.sleep(0.1)  # 錯誤後短暫休息
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
    def _execute_vibration_feed(self):
        """執行震動投料控制"""
        try:
            print("[Flow4] === 開始執行震動投料控制 ===")
            start_time = time.time()
            
            if not self.flow4_executor:
                print("[Flow4] ✗ Flow4執行器未初始化")
                return
            
            # 執行Flow4
            result = self.flow4_executor.execute()
            
            execution_time = time.time() - start_time
            
            if result.success:
                print(f"[Flow4] ✓ 震動投料控制執行成功，耗時: {execution_time:.2f}秒")
                print(f"[Flow4] 完成步驟: {result.steps_completed}/{result.total_steps}")
            else:
                print(f"[Flow4] ✗ 震動投料控制執行失敗: {result.error_message}")
                print(f"[Flow4] 完成步驟: {result.steps_completed}/{result.total_steps}")
                
            self.operation_count += 1
            print("[Flow4] === 震動投料控制執行完成 ===")
                
        except Exception as e:
            print(f"[Flow4] 震動投料控制執行異常: {e}")
            traceback.print_exc()

# ==================== 外部模組執行緒 ====================

class ExternalModuleThread(BaseFlowThread):
    """外部模組交握執行緒"""
    
    def __init__(self, command_queue: DedicatedCommandQueue, external_modules: Dict):
        super().__init__("ExternalModule", command_queue)
        self.external_modules = external_modules
        
    def run(self):
        """外部模組執行緒主循環"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.EXTERNAL:
                    self._handle_external_command(command)
                    
            except Exception as e:
                self.last_error = f"外部模組執行緒錯誤: {e}"
                print(self.last_error)
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
    def _handle_external_command(self, command: Command):
        """處理外部模組指令"""
        try:
            cmd_data = command.command_data
            module_name = cmd_data.get('module', '')
            operation = cmd_data.get('operation', '')
            params = cmd_data.get('params', {})
            
            if module_name in self.external_modules:
                module = self.external_modules[module_name]
                success = self._handle_module_operation(module, module_name, operation, params)
                
                if success:
                    print(f"{module_name}.{operation} 執行成功")
                else:
                    print(f"{module_name}.{operation} 執行失敗")
                    
                self.operation_count += 1
            else:
                print(f"未知外部模組: {module_name}")
                
        except Exception as e:
            self.last_error = f"執行外部模組指令失敗: {e}"
            print(self.last_error)
            
    def _handle_module_operation(self, module, module_name: str, operation: str, params: Dict) -> bool:
        """處理模組操作"""
        try:
            if hasattr(module, operation):
                method = getattr(module, operation)
                if callable(method):
                    if params:
                        return method(**params)
                    else:
                        return method()
            return True
        except Exception as e:
            print(f"模組操作執行失敗: {e}")
            return False

# ==================== 主控制器 ====================

class DobotConcurrentController:
    """Dobot併行控制器 - 專用佇列版本"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        
        # 專用指令佇列 - 每個執行緒一個佇列
        self.motion_queue = DedicatedCommandQueue("Motion")
        self.flow3_queue = DedicatedCommandQueue("Flow3")
        self.flow4_queue = DedicatedCommandQueue("Flow4")
        self.external_queue = DedicatedCommandQueue("External")
        
        # 核心組件
        self.robot = None
        self.modbus_client = None
        self.state_machine = None
        
        # 執行緒
        self.motion_thread = None
        self.flow3_thread = None
        self.flow4_thread = None
        self.external_thread = None
        self.handshake_thread = None
        
        # 狀態
        self.running = False
        self.external_modules = {}
        
        # 上次控制狀態
        self.last_vp_control = 0
        self.last_unload_control = 0
        self.last_flip_control = 0
        self.last_vibration_feed_control = 0
        
    def _load_config(self) -> Dict[str, Any]:
        """載入配置"""
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.config_file)
        
        default_config = {
            "robot": {
                "ip": "192.168.1.6",
                "dashboard_port": 29999,
                "move_port": 30003,
                "default_speed": 50,
                "default_acceleration": 50,
                "enable_collision_detection": True,
                "collision_level": 3
            },
            "modbus": {
                "server_ip": "127.0.0.1",
                "server_port": 502,
                "robot_base_address": 400,
                "timeout": 3.0
            },
            "gripper": {
                "type": "PGE",
                "enabled": True,
                "base_address": 520,
                "status_address": 500,
                "default_force": 50,
                "default_speed": 80
            },
            "vision": {
                "ccd1_base_address": 200,
                "ccd3_base_address": 800,
                "detection_timeout": 10.0,
                "ccd1_enabled": True,
                "ccd3_enabled": False
            },
            "flows": {
                "flow1_enabled": True,
                "flow2_enabled": True,
                "flow3_enabled": True,
                "flow4_enabled": True
            },
            "safety": {
                "enable_emergency_stop": True,
                "max_error_count": 5,
                "auto_recovery": False
            }
        }
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    self._deep_update(default_config, user_config)
            except Exception as e:
                print(f"載入配置檔案失敗，使用預設配置: {e}")
        else:
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"創建預設配置檔案: {config_path}")
            except Exception as e:
                print(f"創建配置檔案失敗: {e}")
                
        return default_config
    
    def _deep_update(self, base_dict: Dict, update_dict: Dict):
        """深度更新字典"""
        for key, value in update_dict.items():
            if key in base_dict and isinstance(base_dict[key], dict) and isinstance(value, dict):
                self._deep_update(base_dict[key], value)
            else:
                base_dict[key] = value
    
    def start(self) -> bool:
        """啟動控制器"""
        print("=== 啟動Dobot併行控制器 (專用佇列版) ===")
        
        if not self._initialize_robot():
            return False
            
        if not self._initialize_modbus():
            return False
            
        self._initialize_state_machine()
        self._initialize_external_modules()
        
        if not self._initialize_threads():
            return False
            
        self.running = True
        self._start_handshake_loop()
        
        print("✓ Dobot併行控制器啟動成功 (專用佇列)")
        return True
    
    def _initialize_robot(self) -> bool:
        """初始化機械臂連接"""
        try:
            robot_config = self.config["robot"]
            
            self.robot = RealRobotController(
                robot_config["ip"],
                robot_config["dashboard_port"],
                robot_config["move_port"]
            )
            
            if not self.robot.initialize():
                return False
                
            print("✓ 機械臂控制器初始化成功")
            return True
            
        except Exception as e:
            print(f"✗ 機械臂初始化失敗: {e}")
            return False
    
    def _initialize_modbus(self) -> bool:
        """初始化Modbus連接"""
        try:
            modbus_config = self.config["modbus"]
            self.modbus_client = ModbusTcpClient(
                host=modbus_config["server_ip"],
                port=modbus_config["server_port"],
                timeout=modbus_config["timeout"]
            )
            
            if self.modbus_client.connect():
                print("✓ Modbus客戶端連接成功")
                return True
            else:
                print("✗ Modbus客戶端連接失敗")
                return False
                
        except Exception as e:
            print(f"✗ Modbus初始化失敗: {e}")
            return False
    
    def _initialize_state_machine(self):
        """初始化狀態機"""
        self.state_machine = DobotStateMachine(self.modbus_client)
        self.state_machine.set_ready(True)
        print("✓ 狀態機初始化完成")
    
    def _initialize_external_modules(self):
        """初始化外部模組"""
        try:
            if self.config["vision"]["ccd1_enabled"]:
                try:
                    ccd1_api = CCD1HighLevelAPI(
                        modbus_host=self.config["modbus"]["server_ip"],
                        modbus_port=self.config["modbus"]["server_port"]
                    )
                    if ccd1_api.connected:
                        self.external_modules['ccd1'] = ccd1_api
                        print("✓ CCD1高階API連接成功")
                    else:
                        print("⚠️ CCD1高階API連接失敗")
                except Exception as e:
                    print(f"⚠️ CCD1高階API初始化失敗: {e}")
            
            if self.config["gripper"]["enabled"]:
                try:
                    gripper_type = GripperType.PGE if self.config["gripper"]["type"] == "PGE" else GripperType.PGC
                    
                    gripper_api = GripperHighLevelAPI(
                        gripper_type=gripper_type,
                        modbus_host=self.config["modbus"]["server_ip"],
                        modbus_port=self.config["modbus"]["server_port"]
                    )
                    if gripper_api.connected:
                        self.external_modules['gripper'] = gripper_api
                        print("✓ 夾爪高階API連接成功")
                    else:
                        print("⚠️ 夾爪高階API連接失敗")
                except Exception as e:
                    print(f"⚠️ 夾爪高階API初始化失敗: {e}")
            
            try:
                angle_api = AngleHighLevel(
                    host=self.config["modbus"]["server_ip"],
                    port=self.config["modbus"]["server_port"]
                )
                if angle_api.connect():
                    self.external_modules['angle'] = angle_api
                    print("✓ 角度校正API連接成功")
                else:
                    print("⚠️ 角度校正API連接失敗")
            except Exception as e:
                print(f"⚠️ 角度校正API初始化失敗: {e}")
                
            print("✓ 外部模組初始化完成")
            
        except Exception as e:
            print(f"外部模組初始化異常: {e}")
    
    def _initialize_threads(self) -> bool:
        """初始化執行緒 - 使用專用佇列"""
        try:
            # 運動控制執行緒
            self.motion_thread = MotionFlowThread(
                self.robot, self.motion_queue, self.state_machine, self.external_modules
            )
            self.motion_thread.initialize_flows()
            
            # Flow3專用執行緒
            self.flow3_thread = Flow3FlipStationThread(self.robot, self.flow3_queue)
            self.flow3_thread.initialize_flows()
            
            # Flow4專用執行緒
            self.flow4_thread = Flow4VibrationFeedThread(self.robot, self.flow4_queue)
            self.flow4_thread.initialize_flows()
            
            # 外部模組執行緒
            self.external_thread = ExternalModuleThread(self.external_queue, self.external_modules)
            
            # 啟動所有執行緒
            self.motion_thread.start_thread()
            self.flow3_thread.start_thread()
            self.flow4_thread.start_thread()
            self.external_thread.start_thread()
            
            print("✓ 執行緒初始化完成 - 專用佇列架構")
            return True
            
        except Exception as e:
            print(f"✗ 執行緒初始化失敗: {e}")
            traceback.print_exc()
            return False
    
    def _start_handshake_loop(self):
        """啟動握手循環"""
        self.handshake_thread = threading.Thread(target=self._handshake_loop, daemon=True)
        self.handshake_thread.start()
        print("✓ 握手循環啟動")
    
    def _handshake_loop(self):
        """Modbus握手循環"""
        while self.running:
            try:
                self._process_control_registers()
                time.sleep(0.1)
                
            except Exception as e:
                print(f"握手循環錯誤: {e}")
                time.sleep(1.0)
    
    def _process_control_registers(self):
        """處理控制寄存器 - 修正指令分派到專用佇列"""
        try:
            result = self.modbus_client.read_holding_registers(address=DOBOT_BASE_ADDR + 40, count=10)
            
            if hasattr(result, 'isError') and result.isError():
                return
                
            registers = result.registers
            
            vp_control = registers[0]
            unload_control = registers[1]
            flip_control = registers[7] if len(registers) > 7 else 0
            vibration_feed_control = registers[8] if len(registers) > 8 else 0
            
            # 處理VP視覺取料控制 (Flow1 - 運動)
            if vp_control == 1 and self.last_vp_control == 0:
                print("收到VP視覺取料指令，分派到運動控制專用佇列")
                command = Command(
                    command_type=CommandType.MOTION,
                    command_data={'type': 'flow_vp_vision_pick'},
                    priority=CommandPriority.MOTION
                )
                if self.motion_queue.put_command(command):
                    self.last_vp_control = 1
                    print("VP視覺取料指令已加入Motion佇列")
                else:
                    print("VP視覺取料指令加入Motion佇列失敗")
                
            elif vp_control == 0 and self.last_vp_control == 1:
                print("VP視覺取料控制指令已清零")
                self.last_vp_control = 0
                
            # 處理出料控制 (Flow2 - 運動)
            if unload_control == 1 and self.last_unload_control == 0:
                print("收到出料指令，分派到運動控制專用佇列")
                command = Command(
                    command_type=CommandType.MOTION,
                    command_data={'type': 'flow_unload'},
                    priority=CommandPriority.MOTION
                )
                if self.motion_queue.put_command(command):
                    self.last_unload_control = 1
                    print("出料指令已加入Motion佇列")
                else:
                    print("出料指令加入Motion佇列失敗")
                
            elif unload_control == 0 and self.last_unload_control == 1:
                print("出料控制指令已清零")
                self.last_unload_control = 0
                
            # 處理翻轉站控制 (Flow3 - 專用佇列)
            if flip_control == 1 and self.last_flip_control == 0:
                print("收到翻轉站指令，分派到Flow3專用佇列")
                command = Command(
                    command_type=CommandType.DIO_FLIP,
                    command_data={'type': 'flow_flip_station'},
                    priority=CommandPriority.DIO_FLIP
                )
                if self.flow3_queue.put_command(command):
                    self.last_flip_control = 1
                    print("翻轉站指令已加入Flow3佇列")
                else:
                    print("翻轉站指令加入Flow3佇列失敗")
                
            elif flip_control == 0 and self.last_flip_control == 1:
                print("翻轉站控制指令已清零")
                self.last_flip_control = 0
            
            # 處理震動投料控制 (Flow4 - 專用佇列)
            if vibration_feed_control == 1 and self.last_vibration_feed_control == 0:
                print("收到震動投料指令，分派到Flow4專用佇列")
                command = Command(
                    command_type=CommandType.DIO_VIBRATION,
                    command_data={'type': 'flow_vibration_feed'},
                    priority=CommandPriority.DIO_VIBRATION
                )
                if self.flow4_queue.put_command(command):
                    self.last_vibration_feed_control = 1
                    print("震動投料指令已加入Flow4佇列")
                else:
                    print("震動投料指令加入Flow4佇列失敗")
                
            elif vibration_feed_control == 0 and self.last_vibration_feed_control == 1:
                print("震動投料控制指令已清零")
                self.last_vibration_feed_control = 0
                
        except Exception as e:
            print(f"處理控制寄存器失敗: {e}")
    
    def stop(self):
        """停止控制器"""
        print("\n=== 停止Dobot併行控制器 ===")
        
        self.running = False
        
        if self.motion_thread:
            self.motion_thread.stop_thread()
        if self.flow3_thread:
            self.flow3_thread.stop_thread()
        if self.flow4_thread:
            self.flow4_thread.stop_thread()
        if self.external_thread:
            self.external_thread.stop_thread()
            
        if self.robot:
            self.robot.disconnect()
        if self.modbus_client:
            self.modbus_client.close()
            
        for name, module in self.external_modules.items():
            try:
                if hasattr(module, 'disconnect'):
                    module.disconnect()
            except Exception as e:
                print(f"斷開{name}失敗: {e}")
        
        print("✓ Dobot併行控制器已停止")
    
    def get_system_status(self) -> Dict[str, Any]:
        """取得系統狀態"""
        return {
            'running': self.running,
            'motion_thread': self.motion_thread.get_status() if self.motion_thread else None,
            'flow3_thread': self.flow3_thread.get_status() if self.flow3_thread else None,
            'flow4_thread': self.flow4_thread.get_status() if self.flow4_thread else None,
            'external_thread': self.external_thread.get_status() if self.external_thread else None
        }

# ==================== 主程序 ====================

def main():
    """主程序"""
    print("="*60)
    print("Dobot M1Pro 併行控制器啟動")
    print("專用佇列架構：四執行緒併行 + 專用指令佇列")
    print("修正指令觸發穩定性，確保Flow3/4可靠執行")
    print("="*60)
    
    controller = DobotConcurrentController()
    
    try:
        if controller.start():
            print("\n系統運行中，按 Ctrl+C 停止...")
            
            while True:
                time.sleep(1)
                
                if int(time.time()) % 10 == 0:
                    status = controller.get_system_status()
                    print(f"\n系統狀態:")
                    print(f"Motion: {status['motion_thread']['status'] if status['motion_thread'] else 'None'}")
                    print(f"Flow3: {status['flow3_thread']['status'] if status['flow3_thread'] else 'None'}")
                    print(f"Flow4: {status['flow4_thread']['status'] if status['flow4_thread'] else 'None'}")
                    
        else:
            print("控制器啟動失敗")
            
    except KeyboardInterrupt:
        print("\n\n收到停止信號...")
    except Exception as e:
        print(f"\n系統錯誤: {e}")
        traceback.print_exc()
    finally:
        controller.stop()
        print("程序結束")

if __name__ == "__main__":
    main()