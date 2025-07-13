#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1_CG_YOLOv11.py - CG Flow1 VP視覺抓取流程 (統一進度更新版)
參考CASE專案Flow1的進度更新機制，統一更新進度到寄存器1202
基於統一Flow架構的運動控制執行器
使用YOLOv11版本CCD1模組進行CG_F物件檢測
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


@dataclass
class CCD1YOLOResult:
    """CCD1 YOLOv11檢測結果數據結構"""
    cg_f_count: int = 0
    cg_b_count: int = 0
    total_detections: int = 0
    cg_f_coords: List[Tuple[float, float]] = None
    cg_f_world_coords: List[Tuple[float, float]] = None
    world_coord_valid: bool = False
    success: bool = False
    error_message: Optional[str] = None

    def __post_init__(self):
        if self.cg_f_coords is None:
            self.cg_f_coords = []
        if self.cg_f_world_coords is None:
            self.cg_f_world_coords = []


class CCD1YOLOInterface:
    """CCD1 YOLOv11接口 - CG專案版本"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # CCD1 YOLOv11寄存器映射 - CG專案基地址200
        self.REGISTERS = {
            # 控制寄存器 (200-201)
            'CONTROL_COMMAND': 200,       # 控制指令
            'STATUS_REGISTER': 201,       # 狀態寄存器
            
            # 完成標誌寄存器 (202-205)
            'CAPTURE_COMPLETE': 202,      # 拍照完成標誌
            'DETECT_COMPLETE': 203,       # 檢測完成標誌
            'OPERATION_SUCCESS': 204,     # 操作成功標誌
            'ERROR_CODE': 205,            # 錯誤代碼
            
            # YOLOv11檢測結果寄存器 (240-259)
            'CG_F_COUNT': 240,            # CG_F檢測數量
            'CG_B_COUNT': 241,            # CG_B檢測數量
            'TOTAL_DETECTIONS': 242,      # 總檢測數量
            'DETECTION_SUCCESS': 243,     # 檢測成功標誌
            
            # CG_F座標寄存器 (244-253) - 最多5個
            'CG_F_1_X': 244,             # CG_F 1號 X座標
            'CG_F_1_Y': 245,             # CG_F 1號 Y座標
            'CG_F_2_X': 246,             # CG_F 2號 X座標
            'CG_F_2_Y': 247,             # CG_F 2號 Y座標
            'CG_F_3_X': 248,             # CG_F 3號 X座標
            'CG_F_3_Y': 249,             # CG_F 3號 Y座標
            'CG_F_4_X': 250,             # CG_F 4號 X座標
            'CG_F_4_Y': 251,             # CG_F 4號 Y座標
            'CG_F_5_X': 252,             # CG_F 5號 X座標
            'CG_F_5_Y': 253,             # CG_F 5號 Y座標
            
            # 世界座標寄存器 (260-279) - YOLOv11版本擴展
            'WORLD_COORD_VALID': 260,     # 世界座標有效標誌
            'CG_F_1_WORLD_X_HIGH': 261,  # CG_F 1號世界X座標高位
            'CG_F_1_WORLD_X_LOW': 262,   # CG_F 1號世界X座標低位
            'CG_F_1_WORLD_Y_HIGH': 263,  # CG_F 1號世界Y座標高位
            'CG_F_1_WORLD_Y_LOW': 264,   # CG_F 1號世界Y座標低位
            'CG_F_2_WORLD_X_HIGH': 265,  # CG_F 2號世界X座標高位
            'CG_F_2_WORLD_X_LOW': 266,   # CG_F 2號世界X座標低位
            'CG_F_2_WORLD_Y_HIGH': 267,  # CG_F 2號世界Y座標高位
            'CG_F_2_WORLD_Y_LOW': 268,   # CG_F 2號世界Y座標低位
            'CG_F_3_WORLD_X_HIGH': 269,  # CG_F 3號世界X座標高位
            'CG_F_3_WORLD_X_LOW': 270,   # CG_F 3號世界X座標低位
            'CG_F_3_WORLD_Y_HIGH': 271,  # CG_F 3號世界Y座標高位
            'CG_F_3_WORLD_Y_LOW': 272,   # CG_F 3號世界Y座標低位
            'CG_F_4_WORLD_X_HIGH': 273,  # CG_F 4號世界X座標高位
            'CG_F_4_WORLD_X_LOW': 274,   # CG_F 4號世界X座標低位
            'CG_F_4_WORLD_Y_HIGH': 275,  # CG_F 4號世界Y座標高位
            'CG_F_4_WORLD_Y_LOW': 276,   # CG_F 4號世界Y座標低位
            'CG_F_5_WORLD_X_HIGH': 277,  # CG_F 5號世界X座標高位
            'CG_F_5_WORLD_X_LOW': 278,   # CG_F 5號世界X座標低位
            'CG_F_5_WORLD_Y_HIGH': 279,  # CG_F 5號世界Y座標高位
        }
        
        self.operation_timeout = 10.0  # 操作超時時間(秒)
    
    def connect(self) -> bool:
        """連接到CCD1 Modbus服務器"""
        if not MODBUS_AVAILABLE:
            print("錯誤: Modbus Client不可用")
            return False
        
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            print(f"連接CCD1 YOLOv11系統: {self.modbus_host}:{self.modbus_port}")
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                print("✓ CCD1 YOLOv11系統連接成功")
                return True
            else:
                print("✗ CCD1 YOLOv11系統連接失敗")
                return False
                
        except Exception as e:
            print(f"CCD1連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
                print("CCD1連接已斷開")
            except:
                pass
        self.connected = False
        self.modbus_client = None
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception:
            return None
    
    def read_cg_yolo_result(self) -> CCD1YOLOResult:
        """讀取CG_F YOLOv11檢測結果 - 被動讀取模式"""
        result = CCD1YOLOResult()
        
        try:
            if not self.connected:
                result.error_message = "CCD1未連接"
                return result
            
            print("讀取CCD1 YOLOv11 CG檢測結果...")
            
            # 直接讀取檢測結果 (假設CCD1已自動完成檢測)
            cg_f_count = self.read_register('CG_F_COUNT') or 0
            cg_b_count = self.read_register('CG_B_COUNT') or 0
            total_detections = self.read_register('TOTAL_DETECTIONS') or 0
            detection_success = self.read_register('DETECTION_SUCCESS') or 0
            world_coord_valid = self.read_register('WORLD_COORD_VALID') or 0
            
            result.cg_f_count = cg_f_count
            result.cg_b_count = cg_b_count
            result.total_detections = total_detections
            result.success = detection_success == 1
            result.world_coord_valid = world_coord_valid == 1
            
            print(f"YOLOv11檢測結果: CG_F={cg_f_count}, CG_B={cg_b_count}, 總計={total_detections}")
            
            # 讀取CG_F像素座標
            if cg_f_count > 0:
                for i in range(min(cg_f_count, 5)):
                    x_reg = f'CG_F_{i+1}_X'
                    y_reg = f'CG_F_{i+1}_Y'
                    
                    x = self.read_register(x_reg)
                    y = self.read_register(y_reg)
                    
                    if x is not None and y is not None:
                        result.cg_f_coords.append((float(x), float(y)))
                        print(f"  CG_F {i+1}: 像素座標({x}, {y})")
            
            # 讀取CG_F世界座標 (如果有效)
            if result.world_coord_valid and cg_f_count > 0:
                for i in range(min(cg_f_count, 5)):
                    x_high_reg = f'CG_F_{i+1}_WORLD_X_HIGH'
                    x_low_reg = f'CG_F_{i+1}_WORLD_X_LOW'
                    y_high_reg = f'CG_F_{i+1}_WORLD_Y_HIGH'
                    y_low_reg = f'CG_F_{i+1}_WORLD_Y_LOW'
                    
                    x_high = self.read_register(x_high_reg) or 0
                    x_low = self.read_register(x_low_reg) or 0
                    y_high = self.read_register(y_high_reg) or 0
                    y_low = self.read_register(y_low_reg) or 0
                    
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
                    
                    result.cg_f_world_coords.append((world_x, world_y))
                    print(f"  CG_F {i+1}: 世界座標({world_x:.2f}, {world_y:.2f}) mm")
            
            # 檢查是否有有效的檢測結果
            if cg_f_count > 0:
                result.success = True
                print(f"✓ 成功讀取到 {cg_f_count} 個CG_F物件")
            else:
                result.success = False
                result.error_message = "未檢測到CG_F物件"
                print("⚠️ 未檢測到CG_F物件")
            
            return result
            
        except Exception as e:
            result.error_message = f"讀取YOLOv11檢測結果異常: {str(e)}"
            print(f"❌ 讀取檢測結果異常: {e}")
            return result


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
    """Flow1: VP視覺抓取流程執行器 - CG專案YOLOv11版本 - 統一進度更新版"""
    
    def __init__(self):
        super().__init__(flow_id=1, flow_name="VP視覺抓取流程(CG-YOLOv11)")
        self.motion_steps = []
        
        # 流程高度參數
        self.VP_DETECT_HEIGHT = 244.65    # VP檢測高度（與vp_topside等高）
        self.PICKUP_HEIGHT = 137.5          # VP夾取高度
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 初始化CCD1 YOLOv11接口
        self.ccd1_interface = CCD1YOLOInterface()
        
        # Flow1需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby",      # 待機點
            "vp_topside",   # VP震動盤上方點
            "flip_pre",     # 翻轉預備點
            "flip_top",     # 翻轉頂部點
            "flip_down"     # 翻轉底部點
        ]
        
        # CCD2 IO控制腳位
        self.CCD2_TRIGGER_PIN = 8  # DO8: 觸發CCD2檢測
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
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
        """建構Flow1步驟 - CG原本流程版本"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # CG原本的Flow1流程步驟 - 保持原有流程不變
        self.motion_steps = [
            # 1. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            
            # 2. VP視覺檢測序列 - YOLOv11版本
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'J'}},
            {'type': 'ccd1_yolo_detection', 'params': {}},  # 使用YOLOv11檢測
            
            # 3. 移動到檢測位置 (等高)
            {'type': 'move_to_detected_position_high', 'params': {}},
            
            # 4. 下降夾取
            {'type': 'move_to_detected_position_low', 'params': {}},
            {'type': 'gripper_smart_release', 'params': {'position': 265}},
            
            # 5. 上升離開
            {'type': 'move_to_point', 'params': {'point_name': 'vp_topside', 'move_type': 'L'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 6. 翻轉檢測序列
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow1流程步驟建構完成(CG原本流程)，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - CG專案YOLOv11版本 - 🔥 參考CASE統一進度更新"""
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
        
        # 連接CCD1 YOLOv11系統
        if not self.ccd1_interface.connect():
            return FlowResult(
                success=False,
                error_message="CCD1 YOLOv11系統連接失敗",
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
                
                print(f"Flow1 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'ccd1_yolo_detection':  # CG YOLOv11檢測
                    detected_position = self._execute_ccd1_yolo_detection()
                    success = detected_position is not None
                elif step['type'] == 'move_to_detected_position_high':
                    success = self._execute_move_to_detected_high(detected_position)
                elif step['type'] == 'move_to_detected_position_low':
                    success = self._execute_move_to_detected_low(detected_position)
                else:
                    print(f"未知步驟類型: {step['type']}")
                    success = False
                
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
                
                # 🔥 參考CASE：統一更新進度到寄存器1202
                self._update_progress_to_1202()
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            # 🔥 參考CASE：最終進度設為100%
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
            # 斷開CCD1連接
            self.ccd1_interface.disconnect()
    
    def _update_progress_to_1202(self, override_progress: Optional[int] = None):
        """🔥 參考CASE：統一更新進度到寄存器1202而不是403"""
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
    
    def _execute_move_to_point(self, params: Dict[str, Any]) -> bool:
        """執行移動到外部點位檔案的點位"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            print(f"移動到點位 {point_name}")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            
            if move_type == 'J':
                # 使用關節角度運動
                return self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
            elif move_type == 'L':
                # 直線運動使用笛卡爾座標
                return self.robot.move_l(point.x, point.y, point.z, point.r)
            else:
                print(f"未支援的移動類型: {move_type}")
                return False
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
            return False
    
    def _execute_gripper_close(self) -> bool:
        """執行夾爪快速關閉 - 包含錯誤處理"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                print("錯誤: 夾爪API未初始化")
                return False
            
            print("夾爪快速關閉")
            success = gripper_api.quick_close()
            
            if success:
                print("✓ 夾爪快速關閉成功")
                time.sleep(1.0)  # 等待1秒確保夾爪完全關閉
                return True
            else:
                print("✗ 夾爪快速關閉失敗")
                return False
                
        except Exception as e:
            print(f"夾爪快速關閉異常: {e}")
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開"""
        try:
            position = params.get('position', 470)
            print(f"夾爪智能撐開到位置: {position}")
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                print("夾爪API未初始化")
                return False
            
            success = gripper_api.smart_release(position)
            
            if success:
                print(f"✓ 夾爪智能撐開指令發送成功")
                time.sleep(1.5)  # 等待夾爪撐開動作完成
                print(f"✓ 夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                print(f"✗ 夾爪智能撐開失敗")
                return False
                
        except Exception as e:
            print(f"夾爪智能撐開異常: {e}")
            return False
    
    def _execute_ccd1_yolo_detection(self) -> Optional[Dict[str, float]]:
        """執行CCD1 YOLOv11檢測 - 取第一個CG_F物件"""
        try:
            print("  使用CCD1 YOLOv11檢測API...")
            
            # 執行YOLOv11檢測
            result = self.ccd1_interface.read_cg_yolo_result()
            
            if not result.success:
                print(f"YOLOv11檢測失敗: {result.error_message}")
                return None
            
            if result.cg_f_count == 0:
                print("YOLOv11未檢測到CG_F物件")
                return None
            
            # 取第一個CG_F物件作為目標
            if result.world_coord_valid and result.cg_f_world_coords:
                # 優先使用世界座標
                world_x, world_y = result.cg_f_world_coords[0]
                
                # 獲取vp_topside點位的Z高度和R值
                vp_topside_point = self.points_manager.get_point('vp_topside')
                if not vp_topside_point:
                    print("錯誤: 無法獲取vp_topside點位")
                    return None
                
                detected_pos = {
                    'x': world_x,
                    'y': world_y,
                    'z': vp_topside_point.z,  # 使用vp_topside的Z高度
                    'r': vp_topside_point.r   # 繼承vp_topside的R值
                }
                
                print(f"YOLOv11檢測成功(世界座標): ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
                print(f"繼承vp_topside - Z:{detected_pos['z']:.2f}, R:{detected_pos['r']:.2f}")
                return detected_pos
                
            elif result.cg_f_coords:
                # 使用像素座標 (需要轉換為機器人座標系)
                pixel_x, pixel_y = result.cg_f_coords[0]
                print(f"警告: 僅獲得像素座標({pixel_x}, {pixel_y})，需要座標轉換")
                # 這裡需要實現像素座標到機器人座標的轉換
                # 暫時返回None，需要根據實際標定參數實現
                return None
            else:
                print("YOLOv11檢測結果無有效座標")
                return None
                
        except Exception as e:
            print(f"YOLOv11檢測異常: {e}")
            return None
    
    def _execute_move_to_detected_high(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(等高)"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 切換到左手系
            print("  切換到左手系（LorR=0）...")
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    result = self.robot.dashboard_api.SetArmOrientation(0)  # 0 = 左手系
                    if "0," in str(result):
                        print("  ✓ 已切換到左手系")
                    else:
                        print(f"  ⚠️ 切換到左手系可能失敗: {result}")
                except Exception as e:
                    print(f"  ⚠️ 切換座標系異常: {e}")
            else:
                print("  ⚠️ 無法訪問座標系切換API，跳過")
            
            print(f"移動到檢測位置(等高): ({detected_position['x']:.2f}, {detected_position['y']:.2f}, {self.VP_DETECT_HEIGHT:.2f})")
            
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.VP_DETECT_HEIGHT,
                detected_position['r']
            )
            
            if success:
                self.robot.sync()
                print(f"MovL已完成並同步: 檢測高度={self.VP_DETECT_HEIGHT:.2f}mm")
                return True
            else:
                print(f"MovL指令執行失敗")
                return False
                
        except Exception as e:
            print(f"移動到檢測位置(等高)失敗: {e}")
            return False
    
    def _execute_move_to_detected_low(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(夾取高度)"""
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
            
            if success:
                self.robot.sync()
                print(f"✓ 下降到夾取位置完成並已同步，夾取高度={self.PICKUP_HEIGHT:.2f}mm")
                return True
            else:
                print(f"✗ 下降到夾取位置失敗")
                return False
                
        except Exception as e:
            print(f"移動到檢測位置(夾取高度)失敗: {e}")
            return False
    
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
        print("Flow1已停止")
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow1是否準備好執行"""
        return self.points_loaded and self.total_steps > 0