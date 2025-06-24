#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow2.py - 出料流程 (連續運動優化版)
從standby點開始，經過料件撐開、移動到組裝位置、放下料件，最後回到standby點
"""

import time
from typing import Dict, Any, Optional
from dataclasses import dataclass


@dataclass
class FlowResult:
    """流程執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 16


class DobotFlow2:
    """
    出料流程執行器 (連續運動優化版)
    從standby點開始的完整出料作業流程
    """
    
    def __init__(self, robot, gripper, ccd1, ccd3, state_machine):
        """初始化流程執行器"""
        # 共享資源
        self.robot = robot
        self.gripper = gripper
        self.ccd1 = ccd1
        self.ccd3 = ccd3
        self.state_machine = state_machine
        
        # 流程配置
        self.flow_id = 2
        self.total_steps = 16
        self.current_step = 0
        self.is_running = False
        self.last_error = ""
        
        # 流程參數
        self.SPEED_RATIO = 100
        self.JOINT_TOLERANCE = 5.0  # 關節位置容差(%)
        self.GRIP_OPEN_POSITION = 370  # 撐開位置
        self.GRIP_CLOSE_POSITION = 0   # 關閉位置
        
        # 必要點位列表 (按流程順序)
        self.REQUIRED_POINTS = [
            "standby",               # 起點和終點
            "Rotate_V2",           # 第一個旋轉點
            "Rotate_top",          # 旋轉頂部點
            "Rotate_down",         # 旋轉底部點(撐開料件處)
            "back_stanby_from_asm", # 從組裝區回程的中轉點
            "put_asm_Pre",         # 組裝預備位置
            "put_asm_top",         # 組裝頂部位置
            "put_asm_down"         # 組裝放下位置
        ]
    
    def execute(self) -> FlowResult:
        """執行出料流程"""
        print("\n" + "="*60)
        print("開始執行流程2 - 出料流程 (連續運動優化版)")
        print("="*60)
        
        start_time = time.time()
        self.is_running = True
        self.current_step = 0
        self.last_error = ""
        
        try:
            # 步驟1: 系統檢查和起點位置驗證
            if not self._execute_step(1, "系統檢查和起點位置驗證", self._step_system_check_and_position):
                return self._create_result(False, start_time)
            
            # 🔥 步驟2-4: 第一段連續運動 (到達撐開位置)
            print("  ▶ 開始第一段連續運動 (步驟2-4): 移動到撐開位置...")
            
            # 步驟2: 移動到Rotate_V2並智能關閉夾爪
            if not self._execute_step(2, "移動到Rotate_V2並智能關閉夾爪", self._step_move_to_rotate_v2_and_close):
                return self._create_result(False, start_time)
            
            # 步驟3: 移動到Rotate_top (無sync，連續運動)
            if not self._execute_step(3, "移動到Rotate_top", 
                                    lambda: self._step_move_to_point_no_sync("Rotate_top")):
                return self._create_result(False, start_time)
            
            # 步驟4: 移動到Rotate_down (無sync，連續運動)
            if not self._execute_step(4, "移動到Rotate_down", 
                                    lambda: self._step_move_to_point_no_sync("Rotate_down")):
                return self._create_result(False, start_time)
            
            # 步驟5: 智能張開夾爪撐開料件 (關鍵sync點 - 夾爪調用前)
            if not self._execute_step(5, "智能張開夾爪撐開料件", self._step_smart_grip_open_sync):
                return self._create_result(False, start_time)
            
            # 🔥 步驟6-10: 第二段連續運動 (移動到組裝位置)
            print("  ▶ 開始第二段連續運動 (步驟6-10): 移動到組裝位置...")
            
            continuous_movements_to_asm = [
                (6, "移動到Rotate_top", "Rotate_top"),
                (7, "移動到back_stanby_from_asm", "back_stanby_from_asm"),
                (8, "移動到put_asm_Pre", "put_asm_Pre"),
                (9, "移動到put_asm_top", "put_asm_top"),
                (10, "移動到put_asm_down", "put_asm_down")
            ]
            
            for step_num, step_name, point_name in continuous_movements_to_asm:
                if not self._execute_step(step_num, step_name, 
                                        lambda p=point_name: self._step_move_to_point_no_sync(p)):
                    return self._create_result(False, start_time)
            
            # 步驟11: 快速關閉夾爪放下料件 (關鍵sync點 - 夾爪調用前)
            if not self._execute_step(11, "快速關閉夾爪放下料件", self._step_quick_close_release_sync):
                return self._create_result(False, start_time)
            
            # 🔥 步驟12-16: 第三段連續運動 (回到standby點)
            print("  ▶ 開始第三段連續運動 (步驟12-16): 回到standby點...")
            
            continuous_movements_return = [
                (12, "移動到put_asm_top", "put_asm_top"),
                (13, "移動到put_asm_Pre", "put_asm_Pre"),
                (14, "移動到back_stanby_from_asm", "back_stanby_from_asm"),
                (15, "移動到standby點", "standby")
            ]
            
            for step_num, step_name, point_name in continuous_movements_return:
                if not self._execute_step(step_num, step_name,
                                        lambda p=point_name: self._step_move_to_point_no_sync(p)):
                    return self._create_result(False, start_time)
            
            # 步驟16: 流程結束確認
            if not self._execute_step(16, "流程結束確認", self._step_flow_completion):
                return self._create_result(False, start_time)
            
            # 最終sync確保所有運動完成
            self.robot.sync()
            print("  ✓ 所有運動已完成，機械臂已回到standby點")
            
            # 流程完成
            execution_time = time.time() - start_time
            print(f"\n✓ 流程2執行完成！總耗時: {execution_time:.2f}秒")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.last_error = f"流程執行異常: {str(e)}"
            print(f"✗ {self.last_error}")
            return self._create_result(False, start_time)
        
        finally:
            self.is_running = False
    
    def _execute_step(self, step_num: int, step_name: str, step_func) -> bool:
        """執行單個步驟並更新進度"""
        self.current_step = step_num
        self._update_progress()
        
        print(f"[{step_num}/{self.total_steps}] {step_name}...")
        
        step_start = time.time()
        success = step_func()
        step_time = time.time() - step_start
        
        if success:
            print(f"  ✓ {step_name}完成 (耗時: {step_time*1000:.1f}ms)")
            return True
        else:
            print(f"  ✗ {step_name}失敗")
            return False
    
    def _update_progress(self):
        """更新進度到狀態機"""
        if (self.state_machine and 
            hasattr(self.state_machine, 'modbus_client') and 
            self.state_machine.modbus_client is not None):
            try:
                progress = int((self.current_step / self.total_steps) * 100)
                self.state_machine.modbus_client.write_register(403, progress)
            except Exception:
                pass
    
    def _create_result(self, success: bool, start_time: float) -> FlowResult:
        """創建流程結果"""
        return FlowResult(
            success=success,
            error_message=self.last_error,
            execution_time=time.time() - start_time,
            steps_completed=self.current_step,
            total_steps=self.total_steps
        )
    
    # =================================================================
    # 流程步驟實現
    # =================================================================
    
    def _step_system_check_and_position(self) -> bool:
        """步驟1: 系統檢查和起點位置驗證"""
        # 1. 系統檢查
        if not self.robot.is_ready():
            self.last_error = "機械臂未準備好"
            return False
        
        # 2. 檢查必要點位
        for point_name in self.REQUIRED_POINTS:
            if not self.robot.points_manager.get_point(point_name):
                self.last_error = f"缺少必要點位: {point_name}"
                return False
        
        # 3. 檢查夾爪狀態
        if self.gripper:
            status = self.gripper.get_status()
            if not status['connected']:
                self.last_error = "PGC夾爪未連接"
                return False
            print("  PGC夾爪狀態正常")
        
        # 4. 🔥 關鍵檢查：驗證當前位置是否在standby點附近
        if not self._check_current_position_at_standby():
            return False
        
        print("  系統檢查通過，當前位置符合standby點要求")
        return True
    
    def _check_current_position_at_standby(self) -> bool:
        """檢查當前關節位置是否與standby點差距在容差範圍內"""
        try:
            # 獲取standby點的關節角度
            standby_point = self.robot.points_manager.get_point("standby")
            if not standby_point:
                self.last_error = "找不到stanby點位數據"
                return False
            
            # 獲取當前關節角度
            current_joints = self.robot.get_current_joints()
            
            # 檢查每個關節的偏差
            target_joints = {
                'j1': standby_point.j1,
                'j2': standby_point.j2,
                'j3': standby_point.j3,
                'j4': standby_point.j4
            }
            
            print(f"  檢查關節位置偏差 (容差: {self.JOINT_TOLERANCE}%):")
            
            for joint_name in ['j1', 'j2', 'j3', 'j4']:
                current_angle = current_joints[joint_name]
                target_angle = target_joints[joint_name]
                
                # 計算偏差百分比
                if abs(target_angle) > 1.0:  # 避免除以接近零的數
                    deviation_percent = abs((current_angle - target_angle) / target_angle) * 100
                else:
                    deviation_percent = abs(current_angle - target_angle)  # 絕對偏差
                
                print(f"    {joint_name}: 當前={current_angle:.2f}°, 目標={target_angle:.2f}°, 偏差={deviation_percent:.1f}%")
                
                # 檢查是否超過容差
                if deviation_percent > self.JOINT_TOLERANCE:
                    self.last_error = f"關節{joint_name}位置偏差過大: {deviation_percent:.1f}% > {self.JOINT_TOLERANCE}% (當前位置不在standby點附近)"
                    return False
            
            print("  ✓ 所有關節位置都在容差範圍內")
            return True
            
        except Exception as e:
            self.last_error = f"關節位置檢查失敗: {e}"
            return False
    
    # =================================================================
    # 關鍵sync點 - 夾爪調用前
    # =================================================================
    
    def _step_move_to_rotate_v2_and_close(self) -> bool:
        """步驟2: 移動到Rotate_V2並智能關閉夾爪 (關鍵sync點)"""
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        # 移動到Rotate_V2
        if not self.robot.MovJ("Rotate_V2"):
            self.last_error = "移動到Rotate_V2失敗"
            return False
        
        # 夾爪調用前必須sync確保到位
        self.robot.sync()
        print("  移動到Rotate_V2完成")
        
        # 智能關閉夾爪
        if self.gripper:
            if not self.gripper.smart_release(release_position=self.GRIP_CLOSE_POSITION):
                self.last_error = "智能關閉夾爪失敗"
                return False
            print(f"  智能關閉夾爪到位置{self.GRIP_CLOSE_POSITION}完成")
        
        return True
    
    def _step_smart_grip_open_sync(self) -> bool:
        """步驟5: 智能張開夾爪撐開料件 (關鍵sync點)"""
        # 夾爪調用前先sync等待前面運動完成
        self.robot.sync()
        print("  到達Rotate_down點")
        
        if not self.gripper:
            print("  跳過夾爪撐開 (夾爪未啟用)")
            return True
        
        # 智能張開撐開料件
        if not self.gripper.smart_grip(target_position=self.GRIP_OPEN_POSITION):
            self.last_error = f"智能張開夾爪到{self.GRIP_OPEN_POSITION}位置失敗"
            return False
        
        print(f"  智能張開夾爪到位置{self.GRIP_OPEN_POSITION}完成 (料件已撐開)")
        return True
    
    def _step_quick_close_release_sync(self) -> bool:
        """步驟11: 快速關閉夾爪放下料件 (關鍵sync點)"""
        # 夾爪調用前先sync等待前面運動完成
        self.robot.sync()
        print("  到達put_asm_down點")
        
        if not self.gripper:
            print("  跳過夾爪放下 (夾爪未啟用)")
            return True
        
        # 快速關閉放下料件
        if not self.gripper.quick_close():
            self.last_error = "快速關閉夾爪失敗"
            return False
        
        print(f"  快速關閉夾爪完成 (料件已放下)")
        return True
    
    # =================================================================
    # 無sync版本 - 連續運動優化
    # =================================================================
    
    def _step_move_to_point_no_sync(self, point_name: str) -> bool:
        """通用點位移動 (無sync版本) - 用於連續運動段"""
        if not self.robot.MovJ(point_name):
            self.last_error = f"移動到{point_name}失敗"
            return False
        
        # 移除sync()，純指令發送
        print(f"  移動到{point_name}指令已發送")
        return True
    
    def _step_flow_completion(self) -> bool:
        """步驟16: 流程結束確認"""
        print("  出料流程執行完成")
        print("  機械臂已回到standby點，可以開始下一個循環")
        return True
    
    # =================================================================
    # 狀態查詢和控制方法
    # =================================================================
    
    def get_progress(self) -> int:
        """獲取當前進度百分比"""
        return int((self.current_step / self.total_steps) * 100)
    
    def get_status(self) -> Dict[str, Any]:
        """獲取流程狀態"""
        return {
            "flow_id": self.flow_id,
            "is_running": self.is_running,
            "current_step": self.current_step,
            "total_steps": self.total_steps,
            "progress_percent": self.get_progress(),
            "last_error": self.last_error,
            "required_points": self.REQUIRED_POINTS,
            "gripper_enabled": self.gripper is not None,
            "joint_tolerance_percent": self.JOINT_TOLERANCE,
            "grip_open_position": self.GRIP_OPEN_POSITION,
            "grip_close_position": self.GRIP_CLOSE_POSITION,
            "optimization_enabled": True,  # 標識已優化
            "continuous_movement_segments": [
                "步驟2-4: Rotate_V2→Rotate_top→Rotate_down", 
                "步驟6-10: Rotate_top→back_stanby_from_asm→put_asm_Pre→put_asm_top→put_asm_down",
                "步驟12-15: put_asm_top→put_asm_Pre→back_stanby_from_asm→standby"
            ]
        }
    
    def stop(self) -> bool:
        """停止流程執行"""
        try:
            self.is_running = False
            
            if self.robot:
                self.robot.emergency_stop()
            
            if self.gripper:
                self.gripper.stop()
            
            self.last_error = "流程已停止"
            return True
            
        except Exception as e:
            print(f"停止流程失敗: {e}")
            return False


class Flow2Executor(DobotFlow2):
    """Flow2執行器 - 兼容性包裝器"""
    pass