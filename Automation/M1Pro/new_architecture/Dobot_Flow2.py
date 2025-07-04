#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow2_new.py - Flow2 CV出料流程 (修正版 - 對應新架構Dobot_main.py)
基於統一Flow架構的運動控制執行器
使用外部點位檔案，全部使用JointMovJ運動，支援錯誤處理和同步機制
"""

import time
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


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
                        # 原始格式
                        pose_data = point_data["pose"]
                    elif "cartesian" in point_data:
                        # 新格式
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
                    
                    self.points[point.name] = point
                    
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


class Flow2UnloadExecutor(FlowExecutor):
    """Flow2: CV出料流程執行器 - 使用外部點位檔案版本"""
    
    def __init__(self):
        super().__init__(flow_id=2, flow_name="CV出料流程")
        self.motion_steps = []
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # Flow2需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby",        # 待機點
            "Goal_CV_top",    # CV上方點
            "Goal_CV_down",   # CV下方點
            "rotate_top",     # 旋轉上方點
            "rotate_down"     # 旋轉下方點
        ]
        
        # 流程參數
        self.GRIP_OPEN_POSITION = 370    # 夾爪撐開位置
        self.SYNC_TIMEOUT = 10.0         # 同步超時時間
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        print("Flow2正在載入外部點位檔案...")
        
        # 載入點位檔案
        if not self.points_manager.load_points():
            print("錯誤: 無法載入點位檔案，Flow2無法執行")
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
        """建構Flow2步驟 - 全部使用JointMovJ"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # Flow2流程步驟 - 全部使用JointMovJ
        self.motion_steps = [
            # 1. 到standby點
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 2. 到Goal_CV_top
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            
            # 3. 到Goal_CV_down
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_down', 'move_type': 'J'}},
            
            # 4. 夾爪智慧撐開
            {'type': 'gripper_smart_release', 'params': {'position': 235}},
            
            # 5. 到Goal_CV_top
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            
            # 6. 到rotate_top
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            
            # 7. 到rotate_down
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_down', 'move_type': 'J'}},
            
            # 8. 夾爪快速關閉
            {'type': 'gripper_close', 'params': {}},
            
            # 9. 到rotate_top
            {'type': 'move_to_point', 'params': {'point_name': 'rotate_top', 'move_type': 'J'}},
            
            # 10. 到Goal_CV_top
            {'type': 'move_to_point', 'params': {'point_name': 'Goal_CV_top', 'move_type': 'J'}},
            
            # 11. 到standby
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}}
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow2流程步驟建構完成，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow2主邏輯 - 包含錯誤處理和同步機制"""
        # 檢查點位是否已載入
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗，無法執行Flow2",
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
        
        print("\n" + "="*60)
        print("開始執行Flow2 - CV出料流程")
        print("="*60)
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"Flow2 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
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
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            print("="*60)
            print("Flow2 - CV出料流程執行完成")
            print(f"執行時間: {execution_time:.2f}秒")
            print("="*60)
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow2執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_move_to_point(self, params: Dict[str, Any]) -> bool:
        """執行移動到外部點位檔案的點位 - 使用關節角度運動，包含同步機制"""
        try:
            point_name = params['point_name']
            move_type = params.get('move_type', 'J')
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            print(f"移動到點位 {point_name}")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            
            # Flow2全部使用JointMovJ
            if move_type == 'J':
                # 使用關節角度運動
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
                
                if success:
                    # 同步等待運動完成
                    sync_success = self._wait_for_motion_complete()
                    if sync_success:
                        print(f"✓ 到達點位 {point_name} 並已同步")
                        return True
                    else:
                        print(f"✗ 到達點位 {point_name} 但同步超時")
                        return False
                else:
                    print(f"✗ 移動到點位 {point_name} 失敗")
                    return False
            else:
                print(f"錯誤: Flow2不支援移動類型 {move_type}，僅支援JointMovJ")
                return False
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
            return False
    
    def _wait_for_motion_complete(self) -> bool:
        """等待運動完成 - 同步機制"""
        try:
            if hasattr(self.robot, 'sync'):
                # 使用robot的sync方法
                success = self.robot.sync()
                if success:
                    print("  ✓ 運動同步完成")
                    return True
                else:
                    print("  ✗ 運動同步失敗")
                    return False
            else:
                # 備用方案：檢查運動狀態
                start_time = time.time()
                while time.time() - start_time < self.SYNC_TIMEOUT:
                    if hasattr(self.robot, 'is_motion_complete'):
                        if self.robot.is_motion_complete():
                            print("  ✓ 運動完成檢測通過")
                            return True
                    time.sleep(0.1)
                
                print(f"  ⚠️ 運動同步超時 ({self.SYNC_TIMEOUT}秒)")
                return False
                
        except Exception as e:
            print(f"  運動同步異常: {e}")
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開 - 包含錯誤處理"""
        try:
            position = params.get('position', self.GRIP_OPEN_POSITION)
            print(f"夾爪智能撐開到位置: {position}")
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                print("錯誤: 夾爪API未初始化")
                return False
            
            # 執行智能撐開操作
            success = gripper_api.smart_release(position)
            
            if success:
                print(f"✓ 夾爪智能撐開指令發送成功")
                
                # 等待夾爪撐開操作完全完成
                print("  等待夾爪撐開動作完成...")
                time.sleep(1.5)  # 等待1.5秒確保夾爪完全撐開
                
                # 檢查夾爪位置確認撐開完成
                if hasattr(gripper_api, 'get_current_position'):
                    try:
                        current_pos = gripper_api.get_current_position()
                        if current_pos is not None:
                            print(f"  夾爪當前位置: {current_pos}")
                            if abs(current_pos - position) <= 20:  # 容差20
                                print(f"  ✓ 夾爪已撐開到目標位置 (誤差: {abs(current_pos - position)})")
                            else:
                                print(f"  ⚠️ 夾爪位置偏差較大 (目標: {position}, 實際: {current_pos})")
                    except Exception as e:
                        print(f"  無法讀取夾爪位置: {e}")
                
                print(f"✓ 夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                print(f"✗ 夾爪智能撐開失敗")
                return False
                
        except Exception as e:
            print(f"夾爪智能撐開異常: {e}")
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
                
                # 等待夾爪關閉完成
                time.sleep(1.0)  # 等待1秒確保夾爪完全關閉
                
                # 檢查夾爪狀態
                if hasattr(gripper_api, 'get_current_position'):
                    try:
                        current_pos = gripper_api.get_current_position()
                        if current_pos is not None:
                            print(f"  夾爪當前位置: {current_pos}")
                    except Exception as e:
                        print(f"  無法讀取夾爪位置: {e}")
                
                return True
            else:
                print("✗ 夾爪快速關閉失敗")
                return False
                
        except Exception as e:
            print(f"夾爪快速關閉異常: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow2已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow2已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        print("Flow2已停止")
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow2是否準備好執行"""
        return self.points_loaded and self.total_steps > 0