#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow5.py - Flow5 機械臂運轉流程執行器 (CG版本 - 固定角度 + 進度修正版)
基於Flow3組裝作業流程，使用固定第四軸角度，無AngleHighLevel依賴
參考Flow1/Flow2點位載入方式，禁止使用內建點位
修正：統一將進度更新到寄存器1202而不是503
"""

import time
import os
import json
from typing import Dict, Any, Optional
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
    total_steps: int = 12


class Flow5AssemblyExecutor:
    """Flow5: 機械臂運轉流程執行器 - CG版本使用固定角度 + 進度修正版"""
    
    # 硬編碼第四軸固定角度
    J4_FIXED_DEGREE = 176.96
    
    def __init__(self):
        self.flow_id = 5
        self.flow_name = "機械臂運轉流程"
        self.status = FlowStatus.IDLE
        self.current_step = 0
        self.total_steps = 12  # 更新總步驟數 (移除角度檢測步驟)
        self.start_time = 0.0
        self.last_error = ""
        
        # 共用資源 (由Main傳入)
        self.robot = None
        self.gripper = None
        self.state_machine = None
        self.external_modules = {}
        
        # 點位管理
        self.loaded_points = {}
        self.points_file_path = ""
        
        # 流程步驟
        self.motion_steps = []
        self.build_flow_steps()
        
        # 必要點位列表 (按新流程順序)
        self.REQUIRED_POINTS = [
            "standby",             # 待機位置 (起點)
            "rotate_top",          # 旋轉頂部點
            "rotate_down",         # 旋轉下方點
            "rotate_down1",        # 旋轉下方點1
            "put_asm_pre",         # 組裝預備位置
            "put_asm_top",         # 組裝頂部位置
            "put_asm_down"         # 組裝放下位置
        ]
        
    def initialize(self, robot, state_machine, external_modules):
        """初始化Flow5 (由Main呼叫)"""
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        
        # 初始化夾爪控制器
        self.gripper = external_modules.get('gripper')
        
        # 載入外部點位檔案
        if not self._load_external_points():
            raise RuntimeError("載入外部點位檔案失敗，Flow5無法初始化")
            
        print("✓ Flow5執行器初始化完成 - 機械臂運轉流程 (固定角度 + 進度修正版)")
        print(f"✓ 第四軸固定角度: {self.J4_FIXED_DEGREE}度")
        print("✓ 進度將統一更新到寄存器1202")
        
    def _load_external_points(self) -> bool:
        """載入外部點位檔案 - 修正陣列格式JSON"""
        try:
            print("Flow5正在載入外部點位檔案...")
            
            # 取得當前執行檔案的目錄
            current_dir = os.path.dirname(os.path.abspath(__file__))
            points_dir = os.path.join(current_dir, "saved_points")
            self.points_file_path = os.path.join(points_dir, "robot_points.json")
            
            print(f"嘗試載入點位檔案: {self.points_file_path}")
            
            # 檢查檔案是否存在
            if not os.path.exists(self.points_file_path):
                self.last_error = f"點位檔案不存在: {self.points_file_path}"
                print(f"✗ {self.last_error}")
                return False
                
            # 讀取點位檔案
            with open(self.points_file_path, 'r', encoding='utf-8') as f:
                points_data = json.load(f)
                
            if not points_data:
                self.last_error = "點位檔案為空"
                print(f"✗ {self.last_error}")
                return False
            
            # 檢查JSON格式：陣列或物件
            if isinstance(points_data, list):
                # 陣列格式：轉換為name:data字典
                self.loaded_points = {}
                for point_item in points_data:
                    if isinstance(point_item, dict) and 'name' in point_item:
                        point_name = point_item['name']
                        self.loaded_points[point_name] = point_item
                    else:
                        print(f"跳過無效點位項目: {point_item}")
                        
            elif isinstance(points_data, dict):
                # 物件格式：直接使用
                self.loaded_points = points_data
            else:
                self.last_error = f"不支援的JSON格式: {type(points_data)}"
                print(f"✗ {self.last_error}")
                return False
                
            if not self.loaded_points:
                self.last_error = "沒有有效的點位數據"
                print(f"✗ {self.last_error}")
                return False
                
            # 顯示載入的點位
            point_names = list(self.loaded_points.keys())
            print(f"載入點位數據成功，共{len(point_names)}個點位: {point_names}")
            
            # 檢查必要點位是否存在
            missing_points = []
            for required_point in self.REQUIRED_POINTS:
                if required_point not in self.loaded_points:
                    missing_points.append(required_point)
                    
            if missing_points:
                self.last_error = f"缺少必要點位: {missing_points}"
                print(f"✗ {self.last_error}")
                return False
                
            print("✓ 所有必要點位載入成功")
            return True
            
        except Exception as e:
            self.last_error = f"載入點位檔案異常: {e}"
            print(f"✗ {self.last_error}")
            return False
    
    def build_flow_steps(self):
        """建構Flow5步驟 - 固定角度版本 (12步)"""
        self.motion_steps = [
            # 1. 移動到standby (起點)
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 2. 移動到rotate_top (使用固定角度)
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            
            # 3. 移動到rotate_down (使用固定角度)
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            
            # 4. 夾爪撐開到229
            {'type': 'gripper_smart_release', 'params': {'position': 229}},
            
            # 5. 夾爪快速關閉
            {'type': 'gripper_quick_close', 'params': {}},
            
            # 6. 移動到rotate_down1
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down1', 'move_type': 'J'}},
            
            # 7. 夾爪撐開到229
            {'type': 'gripper_smart_release', 'params': {'position': 229}},
            
            # 8. 移動到rotate_top (使用固定角度)
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            
            # 9. 移動到put_asm_top
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_top', 'move_type': 'J'}},
            
            # 10. 移動到put_asm_down
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_down', 'move_type': 'J'}},
            
            # 11. 夾爪快速關閉
            {'type': 'gripper_quick_close', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            # 12. 移動到standby (完成)
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}}
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow5流程步驟建構完成，共{self.total_steps}步 (固定角度 + 進度修正版)")
    
    def execute(self) -> FlowResult:
        """執行Flow5主邏輯 - 進度修正版"""
        print("\n" + "="*60)
        print("開始執行Flow5 - 機械臂運轉流程 (固定角度 + 進度修正版)")
        print("流程序列: standby->rotate_top->rotate_down->夾爪撐開->夾爪關閉->rotate_down1->夾爪撐開->rotate_top->put_asm_top->put_asm_down->夾爪關閉->standby")
        print(f"第四軸固定角度: {self.J4_FIXED_DEGREE}度")
        print("進度統一更新到寄存器1202")
        print("="*60)
        
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
                
                print(f"Flow5 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'gripper_quick_close':
                    success = self._execute_gripper_quick_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                else:
                    print(f"未知步驟類型: {step['type']}")
                    success = False
                
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
                
                # 🔥 修正：統一更新進度到寄存器1202
                self._update_progress_to_1202()
            
            # 流程完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            # 🔥 修正：最終進度設為100%
            self._update_progress_to_1202(100)
            
            print(f"\n✓ Flow5執行完成！總耗時: {execution_time:.2f}秒")
            print(f"✓ 使用固定第四軸角度: {self.J4_FIXED_DEGREE}度")
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
    
    def _update_progress_to_1202(self, override_progress: Optional[int] = None):
        """🔥 修正方法：統一更新進度到寄存器1202而不是503"""
        try:
            if override_progress is not None:
                progress = override_progress
            else:
                progress = int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0
            
            # 方法1：通過state_machine的set_progress方法 (推薦)
            if hasattr(self.state_machine, 'set_progress'):
                self.state_machine.set_progress(progress)
                print(f"[Flow5] 進度已更新到1202: {progress}% (透過MotionStateMachine)")
                return
            
            # 方法2：直接寫入到1202寄存器 (備用方法)
            if (self.state_machine and 
                hasattr(self.state_machine, 'modbus_client') and 
                self.state_machine.modbus_client is not None):
                try:
                    # 直接寫入運動進度寄存器1202
                    result = self.state_machine.modbus_client.write_register(1202, progress)
                    if hasattr(result, 'isError') and not result.isError():
                        print(f"[Flow5] 進度已更新到1202: {progress}% (直接寫入)")
                    else:
                        print(f"[Flow5] 進度更新失敗: {result}")
                except Exception as e:
                    print(f"[Flow5] 進度更新異常: {e}")
            else:
                print(f"[Flow5] 無法更新進度：state_machine或modbus_client不可用")
                
        except Exception as e:
            print(f"[Flow5] 進度更新到1202失敗: {e}")
    
    def _execute_move_to_point(self, params: Dict[str, Any]) -> bool:
        """執行移動到指定點位 - 使用原始角度"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'J')
            
            # 檢查點位是否存在
            if point_name not in self.loaded_points:
                self.last_error = f"點位不存在: {point_name}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 取得點位數據
            point_item = self.loaded_points[point_name]
            
            # 根據JSON格式提取座標數據
            if 'cartesian' in point_item:
                cartesian_data = point_item['cartesian']
            else:
                self.last_error = f"點位{point_name}缺少cartesian數據"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            # 根據JSON格式提取關節數據
            if 'joint' in point_item:
                joint_data = point_item['joint']
            else:
                self.last_error = f"點位{point_name}缺少joint數據"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            print(f"移動到點位 {point_name} (原始角度)")
            print(f"  關節角度: (j1:{joint_data['j1']:.1f}, j2:{joint_data['j2']:.1f}, j3:{joint_data['j3']:.1f}, j4:{joint_data['j4']:.1f})")
            print(f"  笛卡爾座標: ({cartesian_data['x']:.2f}, {cartesian_data['y']:.2f}, {cartesian_data['z']:.2f}, {cartesian_data['r']:.2f})")
            
            # 執行移動
            if move_type == 'J':
                success = self.robot.joint_move_j(
                    joint_data['j1'], 
                    joint_data['j2'], 
                    joint_data['j3'], 
                    joint_data['j4']
                )
            elif move_type == 'L':
                success = self.robot.move_l(
                    cartesian_data['x'], 
                    cartesian_data['y'], 
                    cartesian_data['z'], 
                    cartesian_data['r']
                )
            else:
                self.last_error = f"未知移動類型: {move_type}"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
            
            if success:
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type})")
                return True
            else:
                self.last_error = f"移動到 {point_name} 失敗"
                print(f"  ✗ 移動操作失敗: {self.last_error}")
                return False
                
        except Exception as e:
            self.last_error = f"移動操作異常: {e}"
            print(f"  ✗ 移動操作異常: {self.last_error}")
            return False
    
    def _execute_gripper_quick_close(self) -> bool:
        """執行夾爪快速關閉"""
        try:
            if not self.gripper:
                self.last_error = "夾爪控制器未初始化"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
            
            print("夾爪快速關閉")
            result = self.gripper.quick_close()
            
            if result:
                print("  ✓ 夾爪快速關閉成功")
                
                # 等待夾爪關閉完成
                time.sleep(1.0)
                
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
        """執行夾爪智慧撐開"""
        try:
            if not self.gripper:
                self.last_error = "夾爪控制器未初始化"
                print(f"  ✗ 夾爪操作失敗: {self.last_error}")
                return False
            
            position = params.get('position', 235)
            print(f"夾爪智能撐開到位置: {position}")
            
            # 執行智能撐開操作
            result = self.gripper.smart_release(position)
            
            if result:
                print(f"  ✓ 夾爪智能撐開指令發送成功")
                
                # 等待夾爪撐開操作完全完成
                print("  等待夾爪撐開動作完成...")
                time.sleep(1.5)
                
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
    
    def pause(self) -> bool:
        """暫停Flow"""
        try:
            self.status = FlowStatus.PAUSED
            print("Flow5已暫停")
            return True
        except Exception as e:
            print(f"暫停Flow5失敗: {e}")
            return False
    
    def resume(self) -> bool:
        """恢復Flow"""
        try:
            if self.status == FlowStatus.PAUSED:
                self.status = FlowStatus.RUNNING
                print("Flow5已恢復")
                return True
            else:
                print("Flow5未處於暫停狀態")
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
            
            self.last_error = "Flow5已停止"
            print("Flow5已停止")
            return True
            
        except Exception as e:
            print(f"停止Flow5失敗: {e}")
            return False
    
    def get_progress(self) -> int:
        """取得執行進度 (0-100)"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def get_status_info(self) -> Dict[str, Any]:
        """取得狀態資訊"""
        return {
            'flow_id': self.flow_id,
            'flow_name': self.flow_name,
            'status': self.status.value,
            'current_step': self.current_step,
            'total_steps': self.total_steps,
            'progress': self.get_progress(),
            'last_error': self.last_error,
            'required_points': self.REQUIRED_POINTS,
            'points_loaded': len(self.loaded_points),
            'points_file_path': self.points_file_path,
            'j4_fixed_degree': self.J4_FIXED_DEGREE,
            'angle_detection_enabled': False,  # CG版本無角度檢測
            'progress_register': 1202,  # 新增：標示進度寄存器地址
            'progress_unified': True    # 新增：標示已統一進度
        }