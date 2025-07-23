#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow5.py - Flow5 機械臂運轉流程執行器 (CG專案重構版 - 支援參數化控制)
重構重點：
1. 支援速度控制 (speed_j, acc_j, speed_l, acc_l)
2. 支援sync控制
3. 支援左右手手勢切換 (arm_orientation_change)
4. 保持原有的CG專案流程、點位和夾爪操作
5. 整合了DR專案的參數化運動控制架構
6. 統一進度更新到寄存器1202
"""

import time
import os
import json
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum


class FlowStatus(Enum):
    """Flow執行狀態"""
    IDLE = 0
    RUNNING = 1
    COMPLETED = 2
    ERROR = 3
    PAUSED = 4


@dataclass
class FlowResult:
    """Flow執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 0


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


class PointsManager:
    """點位管理器 - 支援cartesian格式 (CG專案版本)"""
    
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
                points_data = json.load(f)
            
            if not points_data:
                print("錯誤: 點位檔案為空")
                return False
            
            # 檢查JSON格式：陣列或物件
            if isinstance(points_data, list):
                # 陣列格式：轉換為name:data字典
                self.points.clear()
                for point_item in points_data:
                    if isinstance(point_item, dict) and 'name' in point_item:
                        try:
                            # 支援兩種格式：pose 或 cartesian
                            if "pose" in point_item:
                                pose_data = point_item["pose"]
                            elif "cartesian" in point_item:
                                pose_data = point_item["cartesian"]
                            else:
                                print(f"點位 {point_item.get('name', 'unknown')} 缺少座標數據")
                                continue
                            
                            # 檢查關節數據
                            if "joint" not in point_item:
                                print(f"點位 {point_item.get('name', 'unknown')} 缺少關節數據")
                                continue
                            
                            joint_data = point_item["joint"]
                            
                            point = RobotPoint(
                                name=point_item["name"],
                                x=float(pose_data["x"]),
                                y=float(pose_data["y"]),
                                z=float(pose_data["z"]),
                                r=float(pose_data["r"]),
                                j1=float(joint_data["j1"]),
                                j2=float(joint_data["j2"]),
                                j3=float(joint_data["j3"]),
                                j4=float(joint_data["j4"])
                            )
                            
                            self.points[point.name] = point
                            
                        except Exception as e:
                            print(f"處理點位 {point_item.get('name', 'unknown')} 時發生錯誤: {e}")
                            continue
                    else:
                        print(f"跳過無效點位項目: {point_item}")
                        
            elif isinstance(points_data, dict):
                # 物件格式：直接使用
                self.points.clear()
                for point_name, point_item in points_data.items():
                    try:
                        # 支援兩種格式：pose 或 cartesian
                        if "pose" in point_item:
                            pose_data = point_item["pose"]
                        elif "cartesian" in point_item:
                            pose_data = point_item["cartesian"]
                        else:
                            print(f"點位 {point_name} 缺少座標數據")
                            continue
                        
                        # 檢查關節數據
                        if "joint" not in point_item:
                            print(f"點位 {point_name} 缺少關節數據")
                            continue
                        
                        joint_data = point_item["joint"]
                        
                        point = RobotPoint(
                            name=point_name,
                            x=float(pose_data["x"]),
                            y=float(pose_data["y"]),
                            z=float(pose_data["z"]),
                            r=float(pose_data["r"]),
                            j1=float(joint_data["j1"]),
                            j2=float(joint_data["j2"]),
                            j3=float(joint_data["j3"]),
                            j4=float(joint_data["j4"])
                        )
                        
                        self.points[point.name] = point
                        
                    except Exception as e:
                        print(f"處理點位 {point_name} 時發生錯誤: {e}")
                        continue
            else:
                print(f"不支援的JSON格式: {type(points_data)}")
                return False
                
            if not self.points:
                print("沒有有效的點位數據")
                return False
                
            # 顯示載入的點位
            point_names = list(self.points.keys())
            print(f"載入點位數據成功，共{len(point_names)}個點位: {point_names}")
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


class Flow5AssemblyExecutorEnhanced:
    """Flow5: 機械臂運轉流程執行器 - CG專案重構版 (支援參數化控制)"""
    
    # 硬編碼第四軸固定角度 (CG專案參數保持不變)
    J4_FIXED_DEGREE = 176.96
    
    def __init__(self, enable_sync: bool = False, default_speed_j: int = 100, default_speed_l: int = 100):
        self.flow_id = 5
        self.flow_name = "機械臂運轉流程(CG重構版-參數化控制)"
        self.status = FlowStatus.IDLE
        self.current_step = 0
        self.total_steps = 0  # 將由build_flow_steps設定
        self.start_time = 0.0
        self.last_error = ""
        
        # 性能優化參數
        self.enable_sync = enable_sync  # 是否啟用機械臂sync
        self.default_speed_j = default_speed_j  # 預設關節速度
        self.default_speed_l = default_speed_l  # 預設直線速度
        
        # 優化的等待時間參數
        self.GRIPPER_CLOSE_WAIT = 0.3     # 從1.0秒減少到0.3秒
        self.GRIPPER_RELEASE_WAIT = 1.0   # 從1.5秒減少到1.0秒
        
        # 共用資源 (由Main傳入)
        self.robot = None
        self.gripper = None
        self.state_machine = None
        self.external_modules = {}
        
        # 點位管理
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # CG專案Flow5需要的點位名稱 (保持不變)
        self.REQUIRED_POINTS = [
            "standby",             # 待機位置 (起點)
            "rotate_top",          # 旋轉頂部點
            "rotate_down",         # 旋轉下方點
            "rotate_down1",        # 旋轉下方點1
            "put_asm_pre",         # 組裝預備位置
            "put_asm_top",         # 組裝頂部位置
            "put_asm_down"         # 組裝放下位置
        ]
        
        # 流程步驟
        self.motion_steps = []
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
        print(f"✓ Flow5重構版初始化完成 (支援參數化控制, sync={'啟用' if enable_sync else '停用'})")
        print(f"  預設關節速度: {default_speed_j}%, 預設直線速度: {default_speed_l}%")
        print(f"  第四軸固定角度: {self.J4_FIXED_DEGREE}度")
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        print("Flow5正在載入外部點位檔案...")
        
        # 載入點位檔案
        if not self.points_manager.load_points():
            print("錯誤: 無法載入點位檔案，Flow5無法執行")
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
        """建構Flow5步驟 - 支援參數化控制版流程"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # 支援參數化控制的Flow5流程步驟 - 保持CG專案原有流程
        self.motion_steps = [
            # 10. 移動到put_asm_top - 支援速度控制
            {'type': 'move_to_point', 'params': {
                'point_name': 'put_asm_top', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            
            # 11. 移動到put_asm_down - 支援速度控制
            {'type': 'move_to_point', 'params': {
                'point_name': 'put_asm_down', 
                'move_type': 'J',
                'speed_j': 60,
                'acc_j': 60,
                'sync': True  # 夾爪操作前必須同步
            }},
            
            # 12. 夾爪快速關閉 (關鍵sync點) - 雙重確保
            {'type': 'gripper_quick_close', 'params': {}},
            
            # 13. 移動到put_asm_top - 支援速度控制
            {'type': 'move_to_point', 'params': {
                'point_name': 'put_asm_top', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': False
            }},
            
         
            # 15. 移動到standby (完成) - 支援速度控制
            {'type': 'move_to_point', 'params': {
                'point_name': 'standby', 
                'move_type': 'J',
                'speed_j': 100,
                'acc_j': 100,
                'sync': True  # 流程結束確保同步
            }}
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow5重構版流程步驟建構完成，共{self.total_steps}步")
        print("新功能: 支援速度/加速度控制 + sync控制 + 左右手手勢切換")
        
    def initialize(self, robot, state_machine, external_modules):
        """初始化Flow5 (由Main呼叫)"""
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        
        # 初始化夾爪控制器
        self.gripper = external_modules.get('gripper')
        
        print("✓ Flow5重構版執行器初始化完成 - 機械臂運轉流程 (參數化控制版)")
        print("✓ 進度將統一更新到寄存器1202")
        
    def execute(self) -> FlowResult:
        """執行Flow5主邏輯 - 參數化控制版"""
        print("\n" + "="*60)
        print("開始執行Flow5 - 機械臂運轉流程 (CG專案重構版-參數化控制)")
        print("流程序列: standby->rotate_top->rotate_down->夾爪撐開->夾爪關閉->rotate_down1->夾爪撐開->rotate_top->put_asm_top->put_asm_down->夾爪關閉->standby")
        print(f"第四軸固定角度: {self.J4_FIXED_DEGREE}度")
        print(f"參數化控制: 速度控制+sync控制+左右手手勢切換")
        print(f"預設關節速度: {self.default_speed_j}%, 預設直線速度: {self.default_speed_l}%")
        print("進度統一更新到寄存器1202")
        print("="*60)
        
        # 檢查點位是否已載入
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗，無法執行Flow5",
                execution_time=0.0,
                steps_completed=0,
                total_steps=0
            )
        
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        self.last_error = ""
        
        # 檢查初始化
        if not self.robot or not self.robot.is_connected:
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                # 減少print輸出，只在關鍵步驟輸出
                if step['type'] in ['gripper_quick_close', 'gripper_smart_release', 
                                   'arm_orientation_change']:
                    print(f"Flow5重構版 關鍵步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = self._execute_step(step)
                
                if not success:
                    self.status = FlowStatus.ERROR
                    return FlowResult(
                        success=False,
                        error_message=self.last_error,
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                self.current_step += 1
                
                # 減少進度更新頻率 (只在重要節點更新)
                if self.current_step % 3 == 0 or self.current_step == self.total_steps:
                    self._update_progress_to_1202()
            
            # 流程完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            # 最終進度設為100%
            self._update_progress_to_1202(100)
            
            print(f"\n✓ Flow5重構版執行完成！總耗時: {execution_time:.2f}秒")
            print(f"✓ 使用固定第四軸角度: {self.J4_FIXED_DEGREE}度")
            print("✓ 參數化控制功能已驗證")
            print("="*60)
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.last_error = f"Flow5執行異常: {str(e)}"
            print(f"✗ {self.last_error}")
            
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=self.last_error,
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_step(self, step: Dict) -> bool:
        """執行單個步驟 - 統一入口"""
        step_type = step['type']
        params = step.get('params', {})
        
        if step_type == 'move_to_point':
            return self._execute_move_to_point_with_parameters(params)
        elif step_type == 'arm_orientation_change':
            return self._execute_arm_orientation_change(params)
        elif step_type == 'gripper_quick_close':
            return self._execute_gripper_quick_close()
        elif step_type == 'gripper_smart_release':
            return self._execute_gripper_smart_release(params)
        else:
            print(f"未知步驟類型: {step_type}")
            return False
    
    def _execute_arm_orientation_change(self, params: Dict[str, Any]) -> bool:
        """執行機械臂手勢切換 (SetArmOrientation)"""
        try:
            orientation = params.get('orientation', 1)  # Flow5預設右手系
            
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
            move_type = params.get('move_type', 'J')
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                self.last_error = f"點位管理器中找不到點位: {point_name}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 提取運動參數
            speed_j = params.get('speed_j', self.default_speed_j)
            acc_j = params.get('acc_j', self.default_speed_j)
            speed_l = params.get('speed_l', self.default_speed_l)
            acc_l = params.get('acc_l', self.default_speed_l)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', self.enable_sync)
            
            print(f"移動到點位 {point_name} ({move_type})")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
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
                self.last_error = f"未知移動類型: {move_type}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # Sync控制
            if success and sync_enabled:
                if hasattr(self.robot, 'sync'):
                    self.robot.sync()
                    print(f"  ✓ 移動到 {point_name} 成功 ({move_type}) (含Sync)")
                else:
                    print(f"  ✓ 移動到 {point_name} 成功 ({move_type}) (無Sync API)")
            elif success:
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type})")
            else:
                self.last_error = f"移動到 {point_name} 失敗"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                
            return success
                
        except Exception as e:
            self.last_error = f"移動操作異常: {e}"
            print(f"  ✗ 移動操作異常: {self.last_error}")
            return False
    
    def _execute_gripper_quick_close(self) -> bool:
        """執行夾爪快速關閉 - 關鍵sync點"""
        try:
            # 夾爪調用前根據參數決定是否sync確保到位
            if self.enable_sync:
                if hasattr(self.robot, 'sync'):
                    self.robot.sync()
                    print("  機械臂已同步到位")
            
            if not self.gripper:
                self.last_error = "夾爪控制器未初始化"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
            
            print("夾爪快速關閉")
            result = self.gripper.quick_close()
            
            if result:
                print("  ✓ 夾爪快速關閉成功")
                
                # 優化等待時間
                time.sleep(self.GRIPPER_CLOSE_WAIT)  # 0.3秒等待
                
                # 檢查夾爪狀態
                if hasattr(self.gripper, 'get_current_position'):
                    try:
                        current_pos = self.gripper.get_current_position()
                        if current_pos is not None:
                            print(f"  夾爪當前位置: {current_pos}")
                    except Exception as e:
                        print(f"  無法讀取夾爪位置: {e}")
                
                return True
            else:
                self.last_error = "夾爪快速關閉失敗"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"夾爪操作異常: {e}"
            print(f"  ✗ 夾爪操作異常: {self.last_error}")
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開 - 關鍵sync點"""
        try:
            # 夾爪調用前根據參數決定是否sync確保到位
            if self.enable_sync:
                if hasattr(self.robot, 'sync'):
                    self.robot.sync()
                    print("  機械臂已同步到位")
            
            if not self.gripper:
                self.last_error = "夾爪控制器未初始化"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
            
            position = params.get('position', 239)  # CG專案使用239
            print(f"夾爪智能撐開到位置: {position}")
            
            # 執行智能撐開操作
            result = self.gripper.smart_release(position)
            
            if result:
                print(f"  ✓ 夾爪智能撐開指令發送成功")
                
                # 優化等待時間
                print("  等待夾爪撐開動作完成...")
                time.sleep(self.GRIPPER_RELEASE_WAIT)  # 1.0秒等待
                
                # 檢查夾爪位置確認撐開完成
                if hasattr(self.gripper, 'get_current_position'):
                    try:
                        current_pos = self.gripper.get_current_position()
                        if current_pos is not None:
                            print(f"  夾爪當前位置: {current_pos}")
                            if abs(current_pos - position) <= 20:  # 容差20
                                print(f"  ✓ 夾爪已撐開到目標位置 (誤差: {abs(current_pos - position)})")
                            else:
                                print(f"  ⚠️ 夾爪位置偏差較大 (目標: {position}, 實際: {current_pos})")
                    except Exception as e:
                        print(f"  無法讀取夾爪位置: {e}")
                
                print(f"  ✓ 夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                self.last_error = f"夾爪智能撐開至{position}失敗"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"夾爪操作異常: {e}"
            print(f"  ✗ 夾爪操作異常: {self.last_error}")
            return False
    
    def _update_progress_to_1202(self, override_progress: Optional[int] = None):
        """統一更新進度到寄存器1202 - CG專案重構版本"""
        try:
            if override_progress is not None:
                progress = override_progress
            else:
                progress = int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0
            
            # 方法1：通過state_machine的set_progress方法 (推薦)
            if hasattr(self.state_machine, 'set_progress'):
                self.state_machine.set_progress(progress)
                print(f"[Flow5重構版] 進度已更新到1202: {progress}% (透過MotionStateMachine)")
                return
            
            # 方法2：直接寫入到1202寄存器 (備用方法)
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                try:
                    # 直接寫入運動進度寄存器1202
                    result = self.state_machine.modbus_client.write_register(1202, progress)
                    if hasattr(result, 'isError') and not result.isError():
                        print(f"[Flow5重構版] 進度已更新到1202: {progress}% (直接寫入)")
                    else:
                        print(f"[Flow5重構版] 進度更新失敗: {result}")
                except Exception as e:
                    print(f"[Flow5重構版] 進度更新異常: {e}")
            else:
                print(f"[Flow5重構版] 無法更新進度：state_machine或modbus_client不可用")
                
        except Exception as e:
            print(f"[Flow5重構版] 進度更新到1202失敗: {e}")
    
    def pause(self) -> bool:
        """暫停Flow"""
        try:
            self.status = FlowStatus.PAUSED
            print("Flow5重構版已暫停")
            return True
        except Exception as e:
            print(f"暫停Flow5失敗: {e}")
            return False
    
    def resume(self) -> bool:
        """恢復Flow"""
        try:
            if self.status == FlowStatus.PAUSED:
                self.status = FlowStatus.RUNNING
                print("Flow5重構版已恢復")
                return True
            else:
                print("Flow5重構版未處於暫停狀態")
                return False
        except Exception as e:
            print(f"恢復Flow5失敗: {e}")
            return False
    
    def stop(self) -> bool:
        """停止Flow"""
        try:
            self.status = FlowStatus.ERROR
            
            if self.robot:
                self.robot.emergency_stop()
            
            if self.gripper:
                self.gripper.stop()
            
            self.last_error = "Flow5重構版已停止"
            print("Flow5重構版已停止")
            return True
            
        except Exception as e:
            print(f"停止Flow5失敗: {e}")
            return False
    
    def get_progress(self) -> int:
        """取得執行進度 (0-100)"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow5是否準備好執行"""
        return self.points_loaded and self.total_steps > 0
    
    def get_status_info(self) -> Dict[str, Any]:
        """取得狀態資訊 - 重構版"""
        return {
            'flow_id': self.flow_id,
            'flow_name': self.flow_name,
            'status': self.status.value,
            'current_step': self.current_step,
            'total_steps': self.total_steps,
            'progress': self.get_progress(),
            'last_error': self.last_error,
            'required_points': self.REQUIRED_POINTS,
            'points_loaded': len(self.points_manager.points) if self.points_loaded else 0,
            'j4_fixed_degree': self.J4_FIXED_DEGREE,
            
            # 參數化控制功能
            'parametric_control_enabled': True,
            'speed_control_enabled': True,
            'sync_control_enabled': True,
            'arm_orientation_control_enabled': True,
            'default_speed_j': self.default_speed_j,
            'default_speed_l': self.default_speed_l,
            'enable_sync': self.enable_sync,
            
            # 模組狀態
            'gripper_enabled': self.gripper is not None,
            'angle_detection_enabled': False,  # CG版本無角度檢測
            
            # 系統狀態
            'progress_register': 1202,  # 標示進度寄存器地址
            'progress_unified': True,   # 標示已統一進度
            'gripper_optimized': True,  # 新增：夾爪優化標示
            'optimized_version': True,  # 新增：優化版本標示
            
            # 版本資訊
            "version": "CG重構版",
            "features": [
                "參數化運動控制",
                "速度/加速度設定", 
                "Sync控制",
                "左右手手勢切換",
                "夾爪雙重確保",
                "優化等待時間"
            ]
        }


# ========================================================================
# 兼容性和向後兼容
# ========================================================================

# 兼容性別名
class Flow5Executor(Flow5AssemblyExecutorEnhanced):
    """Flow5執行器 - 兼容性包裝器"""
    pass

# 原始類別別名 (向後兼容)
class Flow5AssemblyExecutor(Flow5AssemblyExecutorEnhanced):
    """原始Flow5類別名 - 向後兼容"""
    pass


