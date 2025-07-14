#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - Flow1 VP視覺抓取流程 (AutoFeeding管理版)
整合CASE專案的優化改進：
1. 使用AutoFeeding管理的檢測 (CCD1透過AutoFeeding管理)
2. 支援座標獲取失敗重試機制
3. enable_sync參數控制運動精度 vs 速度
4. 優化夾爪等待時間和整體延遲
5. 新增翻轉檢測序列點位：Goal_CV_top, rotate_top, rotate_down
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


class OptimizedAutoFeedingInterface:
    """優化版AutoFeeding座標接口 - 預先建立連接版本"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        self._connection_retries = 0
        self._max_retries = 3
        
        # AutoFeeding寄存器映射 (基地址900)
        self.REGISTERS = {
            'AF_MODULE_STATUS': 900,      # AutoFeeding模組狀態
            'FEEDING_COMPLETE': 940,      # 入料完成標誌
            'TARGET_X_HIGH': 941,         # 料件座標X高位
            'TARGET_X_LOW': 942,          # 料件座標X低位
            'TARGET_Y_HIGH': 943,         # 料件座標Y高位
            'TARGET_Y_LOW': 944,          # 料件座標Y低位
            'AUTOPROGRAM_CONFIRM': 945,   # AutoProgram確認讀取
            'TOTAL_DETECTIONS': 946,      # 檢測結果總數
            'CG_F_COUNT': 947,            # CG_F總數
        }
        
        # 預先建立連接
        self.ensure_connection()
    
    def ensure_connection(self) -> bool:
        """確保連接已建立，包含重連邏輯"""
        if self.connected and self.modbus_client:
            try:
                # 快速連接測試
                test_result = self.modbus_client.read_holding_registers(self.REGISTERS['AF_MODULE_STATUS'], 1, slave=1)
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
                timeout=1.0  # 優化：從3.0秒縮短到1.0秒
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self._connection_retries = 0
                #print("✓ AutoFeeding連接已建立")
                return True
            else:
                self.connected = False
                self._connection_retries += 1
                print(f"✗ AutoFeeding連接失敗 (嘗試 {self._connection_retries}/{self._max_retries})")
                return False
                
        except Exception as e:
            self.connected = False
            self._connection_retries += 1
            print(f"AutoFeeding連接異常: {e}")
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
    
    def check_feeding_complete(self) -> bool:
        """檢查入料是否完成"""
        feeding_complete = self.read_register('FEEDING_COMPLETE')
        return feeding_complete == 1
    
    def read_target_coordinates_fast(self) -> Optional[Dict[str, float]]:
        """快速讀取AutoFeeding座標"""
        try:
            # 快速狀態檢查
            if not self.check_feeding_complete():
                return None
            
            # 批量讀取座標寄存器 (941-947)
            try:
                result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['TARGET_X_HIGH'], 7, slave=1
                )
                if result.isError():
                    return None
                
                registers = result.registers
                x_high, x_low, y_high, y_low = registers[0], registers[1], registers[2], registers[3]
                total_detections, cg_f_count = registers[5], registers[6]
                
            except Exception:
                # 批量讀取失敗，回退到單個讀取
                x_high = self.read_register('TARGET_X_HIGH') or 0
                x_low = self.read_register('TARGET_X_LOW') or 0
                y_high = self.read_register('TARGET_Y_HIGH') or 0
                y_low = self.read_register('TARGET_Y_LOW') or 0
                total_detections = self.read_register('TOTAL_DETECTIONS') or 0
                cg_f_count = self.read_register('CG_F_COUNT') or 0
            
            # 32位合併並轉換精度
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
            
            return {
                'x': world_x,
                'y': world_y,
                'total_detections': total_detections,
                'cg_f_count': cg_f_count
            }
            
        except Exception as e:
            print(f"讀取AutoFeeding目標座標異常: {e}")
            return None
    
    def confirm_coordinate_read_fast(self) -> bool:
        """快速確認讀取座標"""
        return self.write_register('AUTOPROGRAM_CONFIRM', 1)


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


class Flow1VisionPickExecutor(FlowExecutor):
    """Flow1: VP視覺抓取流程執行器 - AutoFeeding管理版"""
    
    def __init__(self, enable_sync: bool = False):
        super().__init__(flow_id=1, flow_name="VP視覺抓取流程(AutoFeeding管理版)")
        
        # 性能優化參數
        self.enable_sync = enable_sync  # 是否啟用機械臂sync
        self.motion_steps = []
        
        # 流程高度參數
        self.VP_DETECT_HEIGHT = 244.65    # VP檢測高度（與vp_topside等高）
        self.PICKUP_HEIGHT = 137.5        # VP夾取高度
        
        # 優化的等待時間參數
        self.GRIPPER_CLOSE_WAIT = 0.3     # 從1.0秒減少到0.3秒
        self.GRIPPER_RELEASE_WAIT = 1.0   # 從1.5秒減少到1.0秒
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 預先建立AutoFeeding連接
        self.autofeeding_interface = OptimizedAutoFeedingInterface()
        
        # Flow1需要的點位名稱 (包含新的翻轉序列點位)
        self.REQUIRED_POINTS = [
            "standby",      # 待機點
            "vp_topside",   # VP震動盤上方點
            "Goal_CV_top",  # 翻轉檢測頂部點
            "rotate_top",   # 翻轉頂部點
            "rotate_down"   # 翻轉底部點
        ]
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
        print(f"✓ Flow1優化版初始化完成 (sync={'啟用' if enable_sync else '停用'})")
        
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
        """建構Flow1步驟 - AutoFeeding管理版流程"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # AutoFeeding管理版的Flow1流程步驟
        self.motion_steps = [
            {'type': 'gripper_close_fast', 'params': {}},
             # 2. VP視覺檢測序列
            {'type': 'read_autofeeding_coordinates_fast', 'params': {}},  # AutoFeeding座標獲取
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'J'}},
            # 1. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
           
            
           
            
            
            
            # 3. 移動到檢測位置 (等高)
            {'type': 'move_to_detected_position_high', 'params': {}},
            
            # 4. 下降夾取
            {'type': 'move_to_detected_position_low', 'params': {}},
            {'type': 'gripper_smart_release_fast', 'params': {'position': 265}},
            
            # 5. 上升離開
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 6. 翻轉檢測序列
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            {'type': 'gripper_close_fast', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow1流程步驟建構完成(AutoFeeding管理版)，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - AutoFeeding管理版 - 統一進度更新到1202"""
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
        
        # 檢查AutoFeeding連接 (快速檢查，不重建)
        if not self.autofeeding_interface.ensure_connection():
            return FlowResult(
                success=False,
                error_message="AutoFeeding連接失效",
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
                if step['type'] in ['read_autofeeding_coordinates_fast', 'move_to_detected_position_high', 'move_to_detected_position_low']:
                    print(f"Flow1 關鍵步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = self._execute_step(step, detected_position)
                
                if step['type'] == 'read_autofeeding_coordinates_fast':
                    detected_position = success  # 特殊處理座標讀取
                    success = detected_position is not None
                    
                    # 特殊處理：AutoFeeding座標讀取失敗時跳警報並停止流程
                    if not success:
                        self.status = FlowStatus.ERROR
                        error_msg = "AutoFeeding座標讀取失敗，已重試20次，跳警報停止流程"
                        print(f"✗ {error_msg}")
                        
                        # 觸發系統警報
                        self._trigger_system_alarm("AutoFeeding座標讀取失敗")
                        
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
                flow_data={'detected_position': detected_position} if detected_position else None
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
            return self._execute_move_to_point_optimized(params)
        elif step_type == 'gripper_close_fast':
            return self._execute_gripper_close_fast()
        elif step_type == 'gripper_smart_release_fast':
            return self._execute_gripper_smart_release_fast(params)
        elif step_type == 'read_autofeeding_coordinates_fast':
            return self._execute_read_autofeeding_coordinates_fast()
        elif step_type == 'move_to_detected_position_high':
            return self._execute_move_to_detected_high_optimized(detected_position)
        elif step_type == 'move_to_detected_position_low':
            return self._execute_move_to_detected_low_optimized(detected_position)
        else:
            print(f"未知步驟類型: {step_type}")
            return False
    
    def _execute_read_autofeeding_coordinates_fast(self) -> Optional[Dict[str, float]]:
        """快速讀取AutoFeeding座標 - 增強重試版本"""
        max_retries = 20
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                retry_count += 1
                
                # 如果不是第一次嘗試，輸出重試資訊
                if retry_count > 1:
                    print(f"[AutoFeeding] 座標讀取重試 {retry_count}/{max_retries}")
                
                # 快速狀態檢查
                af_status = self.autofeeding_interface.read_register('AF_MODULE_STATUS')
                if af_status not in [1, 2]:
                    print(f"[AutoFeeding] 重試{retry_count}: AutoFeeding模組狀態異常 ({af_status})")
                    time.sleep(0.1)  # 重試間隔100ms
                    continue
                
                # 快速等待入料完成
                timeout = 5.0
                start_time = time.time()
                feeding_complete = False
                
                while time.time() - start_time < timeout:
                    if self.autofeeding_interface.check_feeding_complete():
                        feeding_complete = True
                        break
                    time.sleep(0.05)  # 50ms檢查間隔
                
                if not feeding_complete:
                    print(f"[AutoFeeding] 重試{retry_count}: 入料未完成")
                    time.sleep(0.1)
                    continue
                
                # 快速讀取座標
                coord_data = self.autofeeding_interface.read_target_coordinates_fast()
                if not coord_data:
                    print(f"[AutoFeeding] 重試{retry_count}: 座標資料讀取失敗")
                    time.sleep(0.1)
                    continue
                
                # 快速確認讀取
                if not self.autofeeding_interface.confirm_coordinate_read_fast():
                    print(f"[AutoFeeding] 重試{retry_count}: 確認讀取失敗")
                    time.sleep(0.1)
                    continue
                
                # 快速清除標誌等待
                clear_start = time.time()
                flag_cleared = False
                
                while time.time() - clear_start < 0.5:
                    if not self.autofeeding_interface.check_feeding_complete():
                        flag_cleared = True
                        break
                    time.sleep(0.02)  # 20ms檢查間隔
                
                # 構建結果座標 - 使用vp_topside點位
                vp_topside_point = self.points_manager.get_point('vp_topside')
                if not vp_topside_point:
                    print(f"[AutoFeeding] 重試{retry_count}: vp_topside點位不存在")
                    time.sleep(0.1)
                    continue
                
                detected_pos = {
                    'x': coord_data['x'],
                    'y': coord_data['y'],
                    'z': vp_topside_point.z,
                    'r': vp_topside_point.r,
                    'source': 'autofeeding_managed',
                    'retry_count': retry_count,
                    'flag_cleared': flag_cleared,
                    'total_detections': coord_data.get('total_detections', 0),
                    'cg_f_count': coord_data.get('cg_f_count', 0)
                }
                
                print(f"✓ AutoFeeding座標讀取成功 (重試{retry_count}次): ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
                return detected_pos
                
            except Exception as e:
                print(f"[AutoFeeding] 重試{retry_count} 異常: {e}")
                time.sleep(0.1)
                continue
        
        # 所有重試都失敗
        print(f"🚨 AutoFeeding座標讀取完全失敗，已重試{max_retries}次")
        print("⚠️ 可能原因:")
        print("  1. AutoFeeding模組狀態異常")  
        print("  2. 入料未完成或檢測失敗")
        print("  3. 網路通訊問題")
        print("  4. 設備硬體故障")
        return None
    
    def _execute_move_to_point_optimized(self, params: Dict[str, Any]) -> bool:
        """執行移動到外部點位檔案的點位 - 優化版sync控制"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            success = False
            if move_type == 'J':
                # 使用關節角度運動
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
            elif move_type == 'L':
                # 直線運動使用笛卡爾座標
                success = self.robot.move_l(point.x, point.y, point.z, point.r)
            else:
                print(f"未支援的移動類型: {move_type}")
                return False
            
            # 可選的sync控制 - 根據enable_sync參數決定
            if success and self.enable_sync:
                self.robot.sync()
                
            return success
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
            return False
    
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
    
    def _execute_move_to_detected_high_optimized(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(等高) - 優化版"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 切換到左手系
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    result = self.robot.dashboard_api.SetArmOrientation(0)  # 0 = 左手系
                    if "0," in str(result):
                        print("  ✓ 已切換到左手系")
                    else:
                        print(f"  ⚠️ 切換到左手系可能失敗: {result}")
                except Exception as e:
                    print(f"  ⚠️ 切換座標系異常: {e}")
            
            print(f"移動到檢測位置(等高): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.VP_DETECT_HEIGHT:.2f})")
            
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.VP_DETECT_HEIGHT,
                detected_position['r']
            )
            
            if success and self.enable_sync:
                self.robot.sync()
                print(f"MovL已完成並同步: 檢測高度={self.VP_DETECT_HEIGHT:.2f}mm")
                
            return success
                
        except Exception as e:
            print(f"移動到檢測位置(等高)失敗: {e}")
            return False
    
    def _execute_move_to_detected_low_optimized(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(夾取高度) - 優化版"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            print(f"移動到檢測位置(夾取): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.PICKUP_HEIGHT:.2f})")
            
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.PICKUP_HEIGHT,
                detected_position['r']
            )
            
            if success and self.enable_sync:
                self.robot.sync()
                print(f"✓ 下降到夾取位置完成並已同步，夾取高度={self.PICKUP_HEIGHT:.2f}mm")
                
            return success
                
        except Exception as e:
            print(f"移動到檢測位置(夾取高度)失敗: {e}")
            return False
    
    def _trigger_system_alarm(self, alarm_message: str):
        """觸發系統警報 - 當AutoFeeding座標讀取失敗時"""
        try:
            print(f"🚨 系統警報: {alarm_message}")
            
            # 方法1：通過state_machine觸發警報
            if hasattr(self.state_machine, 'set_alarm'):
                self.state_machine.set_alarm(True, alarm_message)
                print("✓ 系統警報已通過state_machine觸發")
                return
            
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
                print(f"[Flow1] 進度已更新到1202: {progress}% (透過MotionStateMachine)")
                return
            
            # 方法2：直接寫入到1202寄存器 (備用方法)
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                try:
                    # 直接寫入運動進度寄存器1202
                    result = self.state_machine.modbus_client.write_register(1202, progress)
                    if hasattr(result, 'isError') and not result.isError():
                        print(f"[Flow1] 進度已更新到1202: {progress}% (直接寫入)")
                    else:
                        print(f"[Flow1] 進度更新失敗: {result}")
                except Exception as e:
                    print(f"[Flow1] 進度更新異常: {e}")
            else:
                print(f"[Flow1] 無法更新進度：state_machine或modbus_client不可用")
                
        except Exception as e:
            print(f"[Flow1] 進度更新到1202失敗: {e}")
    
    def cleanup(self):
        """清理資源"""
        if hasattr(self, 'autofeeding_interface'):
            self.autofeeding_interface.disconnect()
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow1已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow1已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        self.cleanup()
        print("Flow1已停止")
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
                self.autofeeding_interface.connected)

