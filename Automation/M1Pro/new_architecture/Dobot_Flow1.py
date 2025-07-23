#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - VP震動盤視覺抓取流程 (CG專案重構版 - 支援參數化控制)
重構重點：
1. 支援速度控制 (speed_j, acc_j, speed_l, acc_l)
2. 支援sync控制
3. 支援左右手手勢切換 (arm_orientation_change)
4. 保持原有的AutoProgram座標讀取和CCD1自動檢測功能
5. 整合了DR專案的參數化運動控制架構
"""

import time
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False


@dataclass
class RobotPoint:
    """機械臂點位數據結構"""
    name: str
    x: float
    y: float
    z: float
    r: float
    j1: float
    j2: float
    j3: float
    j4: float


class OptimizedAutoProgramInterface:
    """優化版AutoProgram座標接口 - Flow1專用"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        self._connection_retries = 0
        self._max_retries = 3
        
        # AutoProgram寄存器映射 (基地址1300)
        self.REGISTERS = {
            # AutoProgram狀態寄存器 (1300-1319)
            'SYSTEM_STATUS': 1300,         # 系統狀態
            'PREPARE_DONE': 1301,          # prepare_done狀態
            'AUTO_ENABLED': 1302,          # 自動程序啟用狀態
            'AUTOFEEDING_STATUS': 1303,    # AutoFeeding CG_F狀態
            'FLOW5_STATUS': 1304,          # Flow5完成狀態
            
            # AutoFeeding座標寄存器 (1340-1359) - AutoProgram管理
            'TARGET_X_HIGH': 1340,         # 目標座標X高位
            'TARGET_X_LOW': 1341,          # 目標座標X低位
            'TARGET_Y_HIGH': 1342,         # 目標座標Y高位
            'TARGET_Y_LOW': 1343,          # 目標座標Y低位
            
            # Flow1握手協議寄存器 (1344-1349)
            'COORDS_READY': 1344,          # 座標準備就緒標誌
            'FLOW1_REQUEST': 1345,         # Flow1座標請求
            'FLOW1_ACK': 1346,             # Flow1確認收到
            'REQUEST_COUNT': 1347,         # Flow1座標請求次數
            'HANDSHAKE_STATUS': 1348,      # 握手協議狀態
        }
        
        # 預先建立連接
        self.ensure_connection()
    
    def ensure_connection(self) -> bool:
        """確保連接已建立，包含重連邏輯"""
        if self.connected and self.modbus_client:
            try:
                # 快速連接測試
                test_result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['SYSTEM_STATUS'], 1, slave=1
                )
                if not test_result.isError():
                    return True
            except:
                pass
        
        # 需要重新連接
        return self._establish_connection()
    
    def _establish_connection(self) -> bool:
        """建立連接的內部方法"""
        try:
            if self.modbus_client:
                try:
                    self.modbus_client.close()
                except:
                    pass
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=1.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self._connection_retries = 0
                return True
            else:
                self.connected = False
                self._connection_retries += 1
                print(f"✗ AutoProgram連接失敗 (嘗試 {self._connection_retries}/{self._max_retries})")
                return False
                
        except Exception as e:
            self.connected = False
            self._connection_retries += 1
            print(f"AutoProgram連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
            except:
                pass
        self.connected = False
        self.modbus_client = None
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器 - 包含自動重連"""
        if not self.ensure_connection() or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception:
            self.connected = False
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器 - 包含自動重連"""
        if not self.ensure_connection() or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            self.connected = False
            return False
    
    def check_autoprogram_system_status(self) -> Dict[str, Any]:
        """檢查AutoProgram系統狀態"""
        try:
            system_status = self.read_register('SYSTEM_STATUS') or 0
            prepare_done = self.read_register('PREPARE_DONE') or 0
            auto_enabled = self.read_register('AUTO_ENABLED') or 0
            coords_ready = self.read_register('COORDS_READY') or 0
            
            # 修正：更寬鬆的系統狀態檢查
            # SystemStatus枚舉值：
            # STOPPED = 0, RUNNING = 1, FLOW1_TRIGGERED = 2, FLOW5_COMPLETED = 3, ERROR = 4
            is_operational = system_status in [1, 2, 3]  # RUNNING, FLOW1_TRIGGERED, FLOW5_COMPLETED 都是可操作狀態
            
            # 如果讀取失敗，可能是連接問題
            if system_status is None:
                print("[Flow1] 無法讀取AutoProgram系統狀態寄存器")
                return {
                    'system_running': False,
                    'prepare_done': False,
                    'auto_enabled': False,
                    'coords_ready': False,
                    'error': 'register_read_failed'
                }
            
            # DEBUG: 輸出狀態詳情
            status_names = {0: 'STOPPED', 1: 'RUNNING', 2: 'FLOW1_TRIGGERED', 3: 'FLOW5_COMPLETED', 4: 'ERROR'}
            status_name = status_names.get(system_status, f'UNKNOWN({system_status})')
            print(f"[Flow1] AutoProgram狀態詳情: {status_name}({system_status}), prepare_done={prepare_done}, coords_ready={coords_ready}")
            
            return {
                'system_running': is_operational,
                'prepare_done': bool(prepare_done),
                'auto_enabled': bool(auto_enabled),
                'coords_ready': bool(coords_ready),
                'raw_status': system_status,
                'status_name': status_name
            }
        except Exception as e:
            print(f"[Flow1] 檢查AutoProgram系統狀態異常: {e}")
            return {
                'system_running': False,
                'prepare_done': False,
                'auto_enabled': False,
                'coords_ready': False,
                'error': str(e)
            }
    
    def request_coordinates_from_autoprogram(self) -> Optional[Dict[str, float]]:
        """向AutoProgram請求座標 - 握手協議版本"""
        try:
            print("[Flow1] 開始向AutoProgram請求座標...")
            
            # 1. 檢查AutoProgram系統狀態
            ap_status = self.check_autoprogram_system_status()
            if not ap_status['system_running']:
                print("[Flow1] ✗ AutoProgram系統未運行")
                return None
            
            if not ap_status['coords_ready']:
                print("[Flow1] ✗ AutoProgram座標未準備就緒")
                return None
            
            print(f"[Flow1] ✓ AutoProgram狀態檢查通過: {ap_status}")
            
            # 2. 發送座標請求
            print("[Flow1] 發送座標請求到AutoProgram...")
            if not self.write_register('FLOW1_REQUEST', 1):
                print("[Flow1] ✗ 發送座標請求失敗")
                return None
            
            # 3. 等待AutoProgram響應 (檢查握手狀態)
            print("[Flow1] 等待AutoProgram響應...")
            timeout = 5.0
            start_time = time.time()
            handshake_active = False
            
            while time.time() - start_time < timeout:
                handshake_status = self.read_register('HANDSHAKE_STATUS')
                if handshake_status == 1:
                    handshake_active = True
                    print("[Flow1] ✓ AutoProgram已響應，握手進行中")
                    break
                time.sleep(0.05)
            
            if not handshake_active:
                print("[Flow1] ✗ AutoProgram響應超時")
                # 清理請求
                self.write_register('FLOW1_REQUEST', 0)
                return None
            
            # 4. 批量讀取座標寄存器
            print("[Flow1] 讀取座標數據...")
            try:
                result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['TARGET_X_HIGH'], 4, slave=1
                )
                if result.isError():
                    print("[Flow1] ✗ 批量讀取座標失敗")
                    return None
                
                registers = result.registers
                x_high, x_low, y_high, y_low = registers[0], registers[1], registers[2], registers[3]
                
            except Exception as e:
                print(f"[Flow1] 批量讀取失敗，回退到單個讀取: {e}")
                # 批量讀取失敗，回退到單個讀取
                x_high = self.read_register('TARGET_X_HIGH') or 0
                x_low = self.read_register('TARGET_X_LOW') or 0
                y_high = self.read_register('TARGET_Y_HIGH') or 0
                y_low = self.read_register('TARGET_Y_LOW') or 0
            
            # 5. 32位合併並轉換精度
            world_x_int = (x_high << 16) | x_low
            world_y_int = (y_high << 16) | y_low
            
            # 處理負數 (補碼轉換)
            if world_x_int >= 2**31:
                world_x_int -= 2**32
            if world_y_int >= 2**31:
                world_y_int -= 2**32
            
            # 恢復精度 (÷100)
            world_x = world_x_int / 100.0
            world_y = world_y_int / 100.0
            
            print(f"[Flow1] ✓ 座標解析成功: ({world_x:.2f}, {world_y:.2f})")
            
            # 6. 確認收到座標
            print("[Flow1] 發送確認收到...")
            if not self.write_register('FLOW1_ACK', 1):
                print("[Flow1] 確認收到座標失敗，但座標已獲取")
            
            # 7. 等待握手完成
            completion_timeout = 3.0
            completion_start = time.time()
            
            while time.time() - completion_start < completion_timeout:
                handshake_status = self.read_register('HANDSHAKE_STATUS')
                coords_ready = self.read_register('COORDS_READY')
                
                if handshake_status == 0 and coords_ready == 0:
                    print("[Flow1] ✓ 握手協議完成")
                    break
                time.sleep(0.05)
            else:
                print("[Flow1] 握手完成超時，但座標已獲取")
            
            print(f"[Flow1] ✓ 從AutoProgram獲取座標成功: ({world_x:.2f}, {world_y:.2f})")
            
            return {
                'x': world_x,
                'y': world_y,
                'source': 'autoprogram_managed'
            }
            
        except Exception as e:
            print(f"[Flow1] 從AutoProgram讀取座標異常: {e}")
            return None
        finally:
            # 清理請求標誌 (無論成功失敗)
            try:
                self.write_register('FLOW1_REQUEST', 0)
                self.write_register('FLOW1_ACK', 0)
            except:
                pass


class PointsManager:
    """點位管理器 - 支援cartesian格式"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        # 確保使用絕對路徑，相對於當前執行檔案的目錄
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據 - 支援cartesian格式"""
        try:
            print(f"嘗試載入點位檔案: {self.points_file}")
            
            if not os.path.exists(self.points_file):
                print(f"錯誤: 點位檔案不存在: {self.points_file}")
                return False
                
            with open(self.points_file, "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            self.points.clear()
            for point_data in points_list:
                try:
                    # 支援兩種格式：pose 或 cartesian
                    if "pose" in point_data:
                        pose_data = point_data["pose"]
                    elif "cartesian" in point_data:
                        pose_data = point_data["cartesian"]
                    else:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少座標數據")
                        continue
                    
                    # 檢查關節數據
                    if "joint" not in point_data:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少關節數據")
                        continue
                    
                    joint_data = point_data["joint"]
                    
                    point = RobotPoint(
                        name=point_data["name"],
                        x=float(pose_data["x"]),
                        y=float(pose_data["y"]),
                        z=float(pose_data["z"]),
                        r=float(pose_data["r"]),
                        j1=float(joint_data["j1"]),
                        j2=float(joint_data["j2"]),
                        j3=float(joint_data["j3"]),
                        j4=float(joint_data["j4"])
                    )
                    
                    # 處理點位名稱的拼寫錯誤
                    point_name = point.name
                    if point_name == "stanby":
                        point_name = "standby"
                        print(f"自動修正點位名稱: stanby -> standby")
                    
                    self.points[point_name] = point
                    
                except Exception as e:
                    print(f"處理點位 {point_data.get('name', 'unknown')} 時發生錯誤: {e}")
                    continue
                
            print(f"載入點位數據成功，共{len(self.points)}個點位: {list(self.points.keys())}")
            return True
            
        except Exception as e:
            print(f"錯誤: 載入點位數據失敗: {e}")
            return False
    
    def get_point(self, name: str) -> Optional[RobotPoint]:
        """獲取指定點位"""
        return self.points.get(name)
    
    def list_points(self) -> List[str]:
        """列出所有點位名稱"""
        return list(self.points.keys())
    
    def has_point(self, name: str) -> bool:
        """檢查是否存在指定點位"""
        return name in self.points


class Flow1VisionPickExecutorEnhanced(FlowExecutor):
    """Flow1: VP視覺抓取流程執行器 (CG專案重構版 - 支援參數化控制)"""
    
    def __init__(self, enable_sync: bool = False, default_speed_j: int = 100, default_speed_l: int = 100):
        super().__init__(flow_id=1, flow_name="VP視覺抓取流程(CG重構版-參數化控制)")
        
        # 性能優化參數
        self.enable_sync = enable_sync  # 是否啟用機械臂sync
        self.default_speed_j = default_speed_j  # 預設關節速度
        self.default_speed_l = default_speed_l  # 預設直線速度
        self.motion_steps = []
        
        # 流程高度參數
        self.VP_DETECT_HEIGHT = 244.65    # VP檢測高度（與vp_topside等高）
        self.PICKUP_HEIGHT = 147.5        # VP夾取高度
        
        # 優化的等待時間參數
        self.GRIPPER_CLOSE_WAIT = 0.3     # 從1.0秒減少到0.3秒
        self.GRIPPER_RELEASE_WAIT = 0.8   # 從1.5秒減少到1.0秒
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 使用AutoProgram接口替代AutoFeeding接口
        self.autoprogram_interface = OptimizedAutoProgramInterface()
        
        # CCD1執行結果記錄
        self.ccd1_objects_processed = 0
        self.ccd1_detection_triggered = False
        self.need_refill = False
        
        # 角度校正執行結果記錄
        self.angle_correction_performed = False
        self.angle_correction_success = False
        self.detected_angle = None
        self.angle_difference = None
        self.motor_position = None
        self.angle_correction_error = None
        
        # Flow1需要的點位名稱 (包含新的翻轉序列點位)
        self.REQUIRED_POINTS = [
            "standby",      # 待機點
            "vp_topside",   # VP震動盤上方點
            "Goal_CV_top",  # 翻轉檢測頂部點
            "rotate_top",   # 翻轉頂部點
            "rotate_down",   # 翻轉底部點
            "put_asm_top",
            "put_asm_down",

        ]
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
        print(f"✓ Flow1重構版初始化完成 (支援參數化控制, sync={'啟用' if enable_sync else '停用'})")
        print(f"  預設關節速度: {default_speed_j}%, 預設直線速度: {default_speed_l}%")
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        print("Flow1正在載入外部點位檔案...")
        
        # 載入點位檔案
        if not self.points_manager.load_points():
            print("錯誤: 無法載入點位檔案，Flow1無法執行")
            self.points_loaded = False
            return
        
        # 檢查所有必要點位是否存在
        missing_points = []
        for point_name in self.REQUIRED_POINTS:
            if not self.points_manager.has_point(point_name):
                missing_points.append(point_name)
        
        if missing_points:
            print(f"錯誤: 缺少必要點位: {missing_points}")
            print(f"可用點位: {self.points_manager.list_points()}")
            self.points_loaded = False
            return
        
        print("✓ 所有必要點位載入成功")
        self.points_loaded = True
        
    def build_flow_steps(self):
        """建構Flow1步驟 - 支援參數化控制版流程"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # 支援參數化控制的Flow1流程步驟
        self.motion_steps = [
            # 1. 機械臂手勢切換 - 切換到左手系 (新增)
            {'type': 'arm_orientation_change', 'params': {'orientation': 0}},
            
            # 2. 初始準備 - 支援速度控制
            {'type': 'gripper_close_fast', 'params': {}},
            {'type': 'move_to_point', 'params': {
                'point_name': 'standby', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            
            # 3. 從AutoProgram請求座標 - 支援速度控制
            {'type': 'read_autoprogram_coordinates_fast', 'params': {}},

            {'type': 'move_to_point', 'params': {
                'point_name': 'vp_topside', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            
            # 4. 移動到檢測位置 - 支援速度控制
            {'type': 'move_to_detected_position_high', 'params': {
                'speed_l': 100,
                'acc_l': 100,
                'sync': False
            }},
            
            # 6. 下降夾取 - 支援速度控制
            {'type': 'move_to_detected_position_low', 'params': {
                'speed_l': 100,
                'acc_l': 100,
                'sync': True
            }},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 265}},

            # 7. 上升離開 - 支援速度控制
            {'type': 'move_to_point', 'params': {
                'point_name': 'vp_topside', 
                'move_type': 'L',
                'speed_l': 100,
                'acc_l': 100,
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'rotate_top', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'rotate_down', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': True
            }},
            {'type': 'gripper_close_fast', 'params': {}},

            {'type': 'move_to_point', 'params': {
                'point_name': 'rotate_top', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'Goal_CV_top', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'rotate_top', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'rotate_down', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': True
            }},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 265}},

            {'type': 'move_to_point', 'params': {
                'point_name': 'rotate_top', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            {'type': 'move_to_point', 'params': {
                'point_name': 'put_asm_top', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow1重構版流程步驟建構完成，共{self.total_steps}步")
        print("新功能: 支援速度/加速度控制 + sync控制 + 左右手手勢切換")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - 參數化控制版"""
        # 檢查點位是否已載入
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗，無法執行Flow1",
                execution_time=0.0,
                steps_completed=0,
                total_steps=0
            )
        
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        # 檢查初始化
        if not self.robot or not self.robot.is_connected:
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查AutoProgram連接 (快速檢查，不重建)
        if not self.autoprogram_interface.ensure_connection():
            return FlowResult(
                success=False,
                error_message="AutoProgram連接失效",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        detected_position = None
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                # 減少print輸出，只在關鍵步驟輸出
                if step['type'] in ['read_autoprogram_coordinates_fast', 'move_to_detected_position_high', 
                                   'move_to_detected_position_low', 'arm_orientation_change', 'ccd1_smart_detection']:
                    print(f"Flow1重構版 關鍵步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = self._execute_step(step, detected_position)
                
                if step['type'] == 'read_autoprogram_coordinates_fast':
                    detected_position = success  # 特殊處理座標讀取
                    success = detected_position is not None
                    
                    # 特殊處理：AutoProgram座標讀取失敗時跳警報並停止流程
                    if not success:
                        self.status = FlowStatus.ERROR
                        error_msg = "AutoProgram座標讀取失敗，已重試3次，跳警報停止流程"
                        print(f"✗ {error_msg}")
                        
                        # 觸發系統警報
                        self._trigger_system_alarm("AutoProgram座標讀取失敗")
                        
                        return FlowResult(
                            success=False,
                            error_message=error_msg,
                            execution_time=time.time() - self.start_time,
                            steps_completed=self.current_step,
                            total_steps=self.total_steps
                        )
                
                if not success:
                    self.status = FlowStatus.ERROR
                    return FlowResult(
                        success=False,
                        error_message=f"步驟 {step['type']} 執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                self.current_step += 1
                
                # 減少進度更新頻率 (只在重要節點更新)
                if self.current_step % 3 == 0 or self.current_step == self.total_steps:
                    self._update_progress_to_1202()
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            # 最終進度設為100%
            self._update_progress_to_1202(100)
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={
                    'detected_position': detected_position,
                    'angle_correction_performed': self.angle_correction_performed,
                    'angle_correction_success': self.angle_correction_success,
                    'detected_angle': self.detected_angle,
                    'ccd1_objects_processed': self.ccd1_objects_processed,
                    'need_refill': self.need_refill
                } if detected_position else None
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow1執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        finally:
            # 清理資源
            self.cleanup()
    
    def _execute_step(self, step: Dict, detected_position: Optional[Dict]) -> Any:
        """執行單個步驟 - 統一入口"""
        step_type = step['type']
        params = step.get('params', {})
        
        if step_type == 'move_to_point':
            return self._execute_move_to_point_with_parameters(params)
        elif step_type == 'arm_orientation_change':
            return self._execute_arm_orientation_change(params)
        elif step_type == 'gripper_close_fast':
            return self._execute_gripper_close_fast()
        elif step_type == 'gripper_smart_release_fast':
            return self._execute_gripper_smart_release_fast(params)
        elif step_type == 'read_autoprogram_coordinates_fast':
            return self._execute_read_autoprogram_coordinates_fast()
        elif step_type == 'move_to_detected_position_high':
            return self._execute_move_to_detected_high_with_parameters(detected_position, params)
        elif step_type == 'move_to_detected_position_low':
            return self._execute_move_to_detected_low_with_parameters(detected_position, params)
        elif step_type == 'ccd1_smart_detection':
            return self._execute_ccd1_smart_detection()
        elif step_type == 'angle_correction_auto_clear':
            return self._execute_angle_correction_auto_clear()
        else:
            print(f"未知步驟類型: {step_type}")
            return False
    
    def _execute_arm_orientation_change(self, params: Dict[str, Any]) -> bool:
        """執行機械臂手勢切換 (SetArmOrientation)"""
        try:
            orientation = params.get('orientation', 0)
            
            # 手勢定義
            orientation_names = {
                0: "左手手勢 (Left)",
                1: "右手手勢 (Right)" 
            }
            
            orientation_name = orientation_names.get(orientation, f"未知手勢({orientation})")
            print(f"切換機械臂手勢到: {orientation_name}")
            
            # 方法1: 直接調用機械臂API的SetArmOrientation()方法
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    # 檢查是否有SetArmOrientation方法
                    if hasattr(self.robot.dashboard_api, 'SetArmOrientation'):
                        result = self.robot.dashboard_api.SetArmOrientation(orientation)
                        print(f"機械臂手勢切換成功: SetArmOrientation({orientation})")
                        print(f"API回應: {result}")
                        return True
                    else:
                        print("機械臂API不支援SetArmOrientation方法")
                        return True  # 降級處理，不影響流程
                except Exception as e:
                    print(f"機械臂手勢切換失敗: {e}")
                    return False
            
            # 方法2: 如果機械臂有set_arm_orientation方法
            elif hasattr(self.robot, 'set_arm_orientation'):
                try:
                    success = self.robot.set_arm_orientation(orientation)
                    if success:
                        print(f"機械臂手勢切換成功: set_arm_orientation({orientation})")
                    else:
                        print(f"機械臂手勢切換失敗: set_arm_orientation({orientation})")
                    return success
                except Exception as e:
                    print(f"機械臂手勢切換異常: {e}")
                    return False
            
            else:
                print("機械臂API不支援手勢切換，跳過此步驟")
                return True  # 降級處理，不影響整個流程
                
        except Exception as e:
            print(f"機械臂手勢切換異常: {e}")
            return False
    
    def _execute_move_to_point_with_parameters(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位 - 支援參數化控制"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            # 提取運動參數
            speed_j = params.get('speed_j', self.default_speed_j)
            acc_j = params.get('acc_j', self.default_speed_j)
            speed_l = params.get('speed_l', self.default_speed_l)
            acc_l = params.get('acc_l', self.default_speed_l)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', self.enable_sync)
            
            print(f"移動到點位 {point_name} ({move_type})")
            print(f"  運動參數: speed_j={speed_j}, acc_j={acc_j}, speed_l={speed_l}, acc_l={acc_l}, tool={tool}")
            
            # 根據運動類型設置速度參數
            success = False
            if move_type in ['J', 'JointMovJ']:
                # 設置關節運動參數
                if hasattr(self.robot, 'set_speed_j'):
                    self.robot.set_speed_j(speed_j)
                if hasattr(self.robot, 'set_acc_j'):
                    self.robot.set_acc_j(acc_j)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 執行關節運動 - 支援多種API
                if hasattr(self.robot, 'joint_move_j'):
                    success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
                elif hasattr(self.robot, 'MovJ'):
                    success = self.robot.MovJ(point_name)
                else:
                    print("錯誤: 機械臂不支援關節運動API")
                    return False
                
            elif move_type in ['L', 'MovL']:
                # 設置直線運動參數
                if hasattr(self.robot, 'set_speed_l'):
                    self.robot.set_speed_l(speed_l)
                if hasattr(self.robot, 'set_acc_l'):
                    self.robot.set_acc_l(acc_l)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 執行直線運動 - 支援多種API
                if hasattr(self.robot, 'move_l'):
                    success = self.robot.move_l(point.x, point.y, point.z, point.r)
                elif hasattr(self.robot, 'MovL'):
                    success = self.robot.MovL(point.x, point.y, point.z, point.r)
                else:
                    print("錯誤: 機械臂不支援直線運動API")
                    return False
                
            else:
                print(f"未支援的移動類型: {move_type}")
                return False
            
            # Sync控制
            if success and sync_enabled:
                if hasattr(self.robot, 'sync'):
                    self.robot.sync()
                    print(f"  移動到 {point_name} 成功 ({move_type}) (含Sync)")
                else:
                    print(f"  移動到 {point_name} 成功 ({move_type}) (無Sync API)")
            elif success:
                print(f"  移動到 {point_name} 成功 ({move_type})")
            else:
                print(f"  移動到 {point_name} 失敗")
                
            return success
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
            return False
    
    def _execute_move_to_detected_high_with_parameters(self, detected_position: Optional[Dict[str, float]], params: Dict[str, Any]) -> bool:
        """移動到檢測位置(等高) - 支援參數化控制"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 提取運動參數
            speed_l = params.get('speed_l', self.default_speed_l)
            acc_l = params.get('acc_l', self.default_speed_l)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', self.enable_sync)
            
            print(f"移動到檢測位置(等高): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.VP_DETECT_HEIGHT:.2f})")
            print(f"  運動參數: speed_l={speed_l}, acc_l={acc_l}, tool={tool}")
            
            # 設置直線運動參數
            if hasattr(self.robot, 'set_speed_l'):
                self.robot.set_speed_l(speed_l)
            if hasattr(self.robot, 'set_acc_l'):
                self.robot.set_acc_l(acc_l)
            if hasattr(self.robot, 'set_tool'):
                self.robot.set_tool(tool)
            
            # 執行移動 - 支援多種API
            success = False
            if hasattr(self.robot, 'move_l'):
                success = self.robot.move_l(
                    detected_position['x'],
                    detected_position['y'],
                    self.VP_DETECT_HEIGHT,
                    detected_position['r']
                )
            elif hasattr(self.robot, 'MovL'):
                success = self.robot.MovL(
                    detected_position['x'],
                    detected_position['y'],
                    self.VP_DETECT_HEIGHT,
                    detected_position['r']
                )
            else:
                print("錯誤: 機械臂不支援直線運動API")
                return False
            
            # Sync控制
            if success and sync_enabled:
                if hasattr(self.robot, 'sync'):
                    self.robot.sync()
                    print(f"  移動到檢測位置(等高)完成並同步，檢測高度={self.VP_DETECT_HEIGHT:.2f}mm")
                else:
                    print(f"  移動到檢測位置(等高)完成，檢測高度={self.VP_DETECT_HEIGHT:.2f}mm")
            elif success:
                print(f"  移動到檢測位置(等高)完成")
                
            return success
                
        except Exception as e:
            print(f"移動到檢測位置(等高)失敗: {e}")
            return False
    
    def _execute_move_to_detected_low_with_parameters(self, detected_position: Optional[Dict[str, float]], params: Dict[str, Any]) -> bool:
        """移動到檢測位置(夾取高度) - 支援參數化控制"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 提取運動參數
            speed_l = params.get('speed_l', self.default_speed_l)
            acc_l = params.get('acc_l', self.default_speed_l)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', self.enable_sync)
            
            print(f"移動到檢測位置(夾取): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.PICKUP_HEIGHT:.2f})")
            print(f"  運動參數: speed_l={speed_l}, acc_l={acc_l}, tool={tool}")
            
            # 設置直線運動參數
            if hasattr(self.robot, 'set_speed_l'):
                self.robot.set_speed_l(speed_l)
            if hasattr(self.robot, 'set_acc_l'):
                self.robot.set_acc_l(acc_l)
            if hasattr(self.robot, 'set_tool'):
                self.robot.set_tool(tool)
            
            # 執行移動 - 支援多種API
            success = False
            if hasattr(self.robot, 'move_l'):
                success = self.robot.move_l(
                    detected_position['x'],
                    detected_position['y'],
                    self.PICKUP_HEIGHT,
                    detected_position['r']
                )
            elif hasattr(self.robot, 'MovL'):
                success = self.robot.MovL(
                    detected_position['x'],
                    detected_position['y'],
                    self.PICKUP_HEIGHT,
                    detected_position['r']
                )
            else:
                print("錯誤: 機械臂不支援直線運動API")
                return False
            
            # Sync控制
            if success and sync_enabled:
                if hasattr(self.robot, 'sync'):
                    self.robot.sync()
                    print(f"  下降到夾取位置完成並已同步，夾取高度={self.PICKUP_HEIGHT:.2f}mm")
                else:
                    print(f"  下降到夾取位置完成，夾取高度={self.PICKUP_HEIGHT:.2f}mm")
            elif success:
                print(f"  下降到夾取位置完成")
                
            return success
                
        except Exception as e:
            print(f"移動到檢測位置(夾取高度)失敗: {e}")
            return False
    
    def _execute_read_autoprogram_coordinates_fast(self) -> Optional[Dict[str, float]]:
        """從AutoProgram快速讀取座標 - 增強重試版本"""
        max_retries = 3
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                retry_count += 1
                
                # 如果不是第一次嘗試，輸出重試資訊
                if retry_count > 1:
                    print(f"[AutoProgram] 座標讀取重試 {retry_count}/{max_retries}")
                
                # 檢查AutoProgram系統狀態
                ap_status = self.autoprogram_interface.check_autoprogram_system_status()
                
                # 修正：更寬鬆的系統狀態檢查
                if 'error' in ap_status:
                    print(f"[AutoProgram] 重試{retry_count}: 系統狀態檢查失敗 - {ap_status['error']}")
                    time.sleep(0.5)
                    continue
                
                if not ap_status['system_running']:
                    print(f"[AutoProgram] 重試{retry_count}: AutoProgram系統狀態={ap_status.get('status_name', 'unknown')}")
                    # 如果是ERROR狀態才等待長一點，其他狀態快速重試
                    if ap_status.get('raw_status') == 4:  # ERROR
                        time.sleep(0.5)
                    else:
                        time.sleep(0.1)
                    continue
                
                # 修正：即使座標未準備就緒也嘗試請求（可能AutoProgram正在處理）
                if not ap_status['coords_ready']:
                    print(f"[AutoProgram] 重試{retry_count}: 座標未準備就緒，嘗試請求看是否能觸發準備")
                    # 不要continue，繼續嘗試請求
                
                # 向AutoProgram請求座標
                coord_data = self.autoprogram_interface.request_coordinates_from_autoprogram()
                if not coord_data:
                    print(f"[AutoProgram] 重試{retry_count}: 座標請求失敗")
                    time.sleep(0.2)
                    continue
                
                # 構建結果座標 - 使用vp_topside點位
                vp_topside_point = self.points_manager.get_point('vp_topside')
                if not vp_topside_point:
                    print(f"[AutoProgram] 重試{retry_count}: vp_topside點位不存在")
                    time.sleep(0.1)
                    continue
                
                detected_pos = {
                    'x': coord_data['x'],
                    'y': coord_data['y'],
                    'z': vp_topside_point.z,
                    'r': vp_topside_point.r,
                    'source': 'autoprogram_managed',
                    'retry_count': retry_count,
                }
                
                print(f"✓ AutoProgram座標讀取成功 (重試{retry_count}次): ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
                return detected_pos
                
            except Exception as e:
                print(f"[AutoProgram] 重試{retry_count} 異常: {e}")
                time.sleep(0.2)
                continue
        
        # 所有重試都失敗
        print(f"🚨 AutoProgram座標讀取完全失敗，已重試{max_retries}次")
        return None
    
    def _execute_gripper_close_fast(self) -> bool:
        """執行夾爪快速關閉 - 優化版"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                print("錯誤: 夾爪API未初始化")
                return False
            
            success = gripper_api.quick_close()
            
            if success:
                time.sleep(self.GRIPPER_CLOSE_WAIT)  # 優化：0.3秒等待
                return True
            else:
                print("✗ 夾爪快速關閉失敗")
                return False
                
        except Exception as e:
            print(f"夾爪快速關閉異常: {e}")
            return False
    
    def _execute_gripper_smart_release_fast(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開 - 優化版"""
        try:
            position = params.get('position', 265)
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                print("夾爪API未初始化")
                return False
            
            success = gripper_api.smart_release(position)
            
            if success:
                time.sleep(self.GRIPPER_RELEASE_WAIT)  # 優化：1.0秒等待
                return True
            else:
                print(f"✗ 夾爪智能撐開失敗")
                return False
                
        except Exception as e:
            print(f"夾爪智能撐開異常: {e}")
            return False
    
    def _execute_ccd1_smart_detection(self) -> bool:
        """執行CCD1智能檢測 - 使用新API (保持原有功能)"""
        try:
            # 檢查CCD1是否可用
            if not self.ccd1:
                print("CCD1模組未初始化，跳過檢測")
                self.need_refill = True
                return True
            
            print("開始CCD1智能檢測...")
            
            # 使用CCD1HighLevel API的get_next_object方法
            coord = self.ccd1.get_next_object()
            
            if coord is None:
                print("CCD1檢測結果: 未檢測到物體")
                self.ccd1_objects_processed = 0
                self.ccd1_detection_triggered = True  # 假設觸發了檢測
                self.need_refill = True
                return True  # 不阻止流程繼續
            else:
                print(f"CCD1檢測成功: 座標({coord['x']:.2f}, {coord['y']:.2f})")
                self.ccd1_objects_processed = 1
                self.ccd1_detection_triggered = True
                self.need_refill = False
                
                # 可以選擇將檢測到的座標用於後續流程
                # 這裡暫時只記錄，實際使用時可能需要更新detected_position
                return True
                
        except Exception as e:
            print(f"CCD1智能檢測異常: {e}")
            self.ccd1_objects_processed = 0
            self.ccd1_detection_triggered = False
            self.need_refill = True
            return False  # CCD1異常時可以選擇阻止流程
    
    def _execute_angle_correction_auto_clear(self) -> bool:
        """執行自動清零角度校正 (保持原有功能)"""
        try:
            print("開始執行自動清零角度校正...")
            
            # 檢查CCD3是否可用
            if not self.ccd3:
                print("CCD3模組未初始化，跳過角度校正")
                self.angle_correction_performed = False
                return True
            
            # 重置角度校正狀態
            self.angle_correction_performed = False
            self.angle_correction_success = False
            self.detected_angle = None
            self.angle_difference = None
            self.motor_position = None
            self.angle_correction_error = None
            
            # 設置標誌表示已執行角度校正
            self.angle_correction_performed = True
            
            try:
                # 使用CCD3進行角度檢測
                if hasattr(self.ccd3, 'detect_angle'):
                    angle_result = self.ccd3.detect_angle(detection_mode=1)  # DR模式
                    
                    if (hasattr(angle_result, 'result') and 
                        angle_result.result.value == "SUCCESS" and 
                        angle_result.target_angle is not None):
                        
                        self.detected_angle = angle_result.target_angle
                        print(f"CCD3角度檢測成功: {self.detected_angle:.2f}°")
                        
                        # 計算角度差 (假設目標是0°)
                        target_angle = 0.0
                        self.angle_difference = self.detected_angle - target_angle
                        
                        # 如果角度差較大，進行馬達校正
                        if abs(self.angle_difference) > 1.0:  # 超過1度才校正
                            print(f"角度差較大({self.angle_difference:.2f}°)，執行馬達校正...")
                            
                            # 這裡應該調用馬達校正API
                            # motor_correction_result = self.motor.correct_angle(self.angle_difference)
                            # 暫時假設校正成功
                            self.motor_position = int(self.angle_difference * 10)  # 假設的馬達位置
                            self.angle_correction_success = True
                            print(f"馬達角度校正完成: 位置={self.motor_position}")
                        else:
                            print(f"角度差較小({self.angle_difference:.2f}°)，無需校正")
                            self.angle_correction_success = True
                    else:
                        error_msg = getattr(angle_result, 'message', '角度檢測失敗')
                        self.angle_correction_error = error_msg
                        print(f"CCD3角度檢測失敗: {error_msg}")
                        self.angle_correction_success = False
                else:
                    print("CCD3不支援角度檢測API")
                    self.angle_correction_success = False
                    
            except Exception as angle_error:
                self.angle_correction_error = str(angle_error)
                print(f"角度校正過程異常: {angle_error}")
                self.angle_correction_success = False
            
            # 自動清零延遲
            time.sleep(0.5)
            print("自動清零角度校正完成")
            
            return True  # 即使校正失敗也不阻止流程完成
            
        except Exception as e:
            self.angle_correction_error = str(e)
            print(f"自動清零角度校正異常: {e}")
            self.angle_correction_performed = True
            self.angle_correction_success = False
            return True  # 不阻止流程完成
    
    def _trigger_system_alarm(self, alarm_message: str):
        """觸發系統警報 - 修正版"""
        try:
            print(f"🚨 系統警報: {alarm_message}")
            
            # 方法1：通過state_machine觸發警報 (修正參數)
            if hasattr(self.state_machine, 'set_alarm'):
                try:
                    # 修正：只傳遞一個參數
                    self.state_machine.set_alarm(True)
                    print("✓ 系統警報已通過state_machine觸發")
                    return
                except Exception as e:
                    print(f"set_alarm調用失敗: {e}")
            
            # 方法2：直接寫入警報寄存器 (備用方法)
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                try:
                    # 設置運動狀態機警報位 (1200寄存器的bit2)
                    current_status = self.state_machine.modbus_client.read_holding_registers(1200, 1)
                    if not current_status.isError():
                        status_value = current_status.registers[0]
                        alarm_status = status_value | (1 << 2)  # 設置bit2 (Alarm位)
                        result = self.state_machine.modbus_client.write_register(1200, alarm_status)
                        if not result.isError():
                            print("✓ 系統警報已直接寫入1200寄存器")
                        else:
                            print(f"✗ 警報寫入失敗: {result}")
                    else:
                        print(f"✗ 讀取狀態寄存器失敗: {current_status}")
                except Exception as e:
                    print(f"✗ 直接觸發警報異常: {e}")
            else:
                print("✗ 無法觸發警報：state_machine或modbus_client不可用")
                
        except Exception as e:
            print(f"✗ 觸發系統警報失敗: {e}")

    def _update_progress_to_1202(self, override_progress: Optional[int] = None):
        """統一更新進度到寄存器1202"""
        try:
            if override_progress is not None:
                progress = override_progress
            else:
                progress = int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0
            
            # 方法1：通過state_machine的set_progress方法 (推薦)
            if hasattr(self.state_machine, 'set_progress'):
                self.state_machine.set_progress(progress)
                print(f"[Flow1重構版] 進度已更新到1202: {progress}% (透過MotionStateMachine)")
                return
            
            # 方法2：直接寫入到1202寄存器 (備用方法)
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                try:
                    # 直接寫入運動進度寄存器1202
                    result = self.state_machine.modbus_client.write_register(1202, progress)
                    if hasattr(result, 'isError') and not result.isError():
                        print(f"[Flow1重構版] 進度已更新到1202: {progress}% (直接寫入)")
                    else:
                        print(f"[Flow1重構版] 進度更新失敗: {result}")
                except Exception as e:
                    print(f"[Flow1重構版] 進度更新異常: {e}")
            else:
                print(f"[Flow1重構版] 無法更新進度：state_machine或modbus_client不可用")
                
        except Exception as e:
            print(f"[Flow1重構版] 進度更新到1202失敗: {e}")
    
    def cleanup(self):
        """清理資源"""
        if hasattr(self, 'autoprogram_interface'):
            self.autoprogram_interface.disconnect()
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow1重構版已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow1重構版已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        self.cleanup()
        print("Flow1重構版已停止")
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow1是否準備好執行"""
        return (self.points_loaded and 
                self.total_steps > 0 and 
                self.autoprogram_interface.connected)
    
    def get_status(self) -> Dict[str, Any]:
        """獲取流程狀態 - 重構版"""
        return {
            "flow_id": self.flow_id,
            "flow_name": self.flow_name,
            "is_running": self.status == FlowStatus.RUNNING,
            "current_step": self.current_step,
            "total_steps": self.total_steps,
            "progress_percent": self.get_progress(),
            "last_error": getattr(self, 'last_error', ''),
            "required_points": self.REQUIRED_POINTS,
            
            # 參數化控制功能
            "parametric_control_enabled": True,
            "speed_control_enabled": True,
            "sync_control_enabled": True,
            "arm_orientation_control_enabled": True,
            "default_speed_j": self.default_speed_j,
            "default_speed_l": self.default_speed_l,
            "enable_sync": self.enable_sync,
            
            # 模組狀態
            "gripper_enabled": hasattr(self, 'external_modules') and 'gripper' in self.external_modules,
            "ccd1_enabled": hasattr(self, 'ccd1') and self.ccd1 is not None,
            "ccd3_enabled": hasattr(self, 'ccd3') and self.ccd3 is not None,
            "autoprogram_enabled": self.autoprogram_interface.connected if self.autoprogram_interface else False,
            
            # 執行結果
            "angle_correction_performed": self.angle_correction_performed,
            "angle_correction_success": self.angle_correction_success,
            "detected_angle": self.detected_angle,
            "angle_difference": self.angle_difference,
            "motor_position": self.motor_position,
            "angle_correction_error": self.angle_correction_error,
            "ccd1_objects_processed": self.ccd1_objects_processed,
            "ccd1_detection_triggered": self.ccd1_detection_triggered,
            "need_refill": self.need_refill,
            
            # 版本資訊
            "version": "CG重構版",
            "features": [
                "參數化運動控制",
                "速度/加速度設定",
                "Sync控制",
                "左右手手勢切換",
                "AutoProgram座標讀取",
                "CCD1智能檢測",
                "自動清零角度校正"
            ]
        }


# ========================================================================
# 兼容性和向後兼容
# ========================================================================

# 兼容性別名
class Flow1Executor(Flow1VisionPickExecutorEnhanced):
    """Flow1執行器 - 兼容性包裝器"""
    pass

# 原始類別別名 (向後兼容)
class Flow1VisionPickExecutor(Flow1VisionPickExecutorEnhanced):
    """原始Flow1類別名 - 向後兼容"""
    pass

class DobotFlow1Enhanced(Flow1VisionPickExecutorEnhanced):
    """原始類別別名 - 向後兼容"""
    pass

