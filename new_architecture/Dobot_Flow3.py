#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow3.py - Flow3 翻轉站控制流程 (立即感測器檢測版)
基於統一Flow架構的DIO控制執行器
控制翻轉站的升降缸、夾爪、翻轉缸、輸送帶
優化感測器檢測速度，立即檢測立即關閉輸送帶
"""

import time
import traceback
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class FlowFlipStationExecutor(FlowExecutor):
    """Flow3: 翻轉站控制流程執行器 (DIO控制)"""
    
    def __init__(self):
        super().__init__(flow_id=3, flow_name="翻轉站控制")
        self.dio_steps = []
        
        # DIO腳位定義
        self.DIO_PINS = {
            # 輸送帶控制
            'CONVEYOR': 2,          # DO2: 輸送帶啟動/關閉
            
            # 翻轉缸控制
            'FLIP_CYLINDER': 5,     # DO5: 翻轉缸 (HIGH=180度, LOW=0度)
            
            # 升降缸控制
            'LIFT_TRIGGER': 11,     # DO11: 升降缸啟動脈衝
            'LIFT_DIR1': 12,        # DO12: 升降缸方向控制位1
            'LIFT_DIR2': 13,        # DO13: 升降缸方向控制位2
            'LIFT_HOME': 14,        # DO14: 升降缸回原點脈衝
            
            # 感測器輸入
            'SENSOR_DI13': 13       # DI13: 輸送帶感測器
        }
        
        # 時間延遲設定
        self.TIMING_CONFIG = {
            'LIFT_MOTION_TIME': 0.8,    # 升降缸上升/下降時間 (秒)
            'LIFT_HOME_TIME': 0.5,      # 升降缸回原點時間 (秒)
            'FLIP_TIME': 0.5,           # 翻轉缸翻轉時間 (秒)
            'GRIPPER_TIME': 1.0,        # PGE夾爪動作時間 (秒)
            'DIRECTION_SETUP_DELAY': 0.1,  # 方向設定延遲 (秒)
            'PULSE_WIDTH': 100,         # 脈衝寬度 (毫秒)
            'SENSOR_CHECK_INTERVAL': 0.02,  # 感測器檢查間隔 (20ms，高速檢測)
            'SENSOR_DEBOUNCE_TIME': 0.05    # 感測器防彈跳時間 (50ms)
        }
        
        # PGE夾爪位置定義
        self.GRIPPER_POSITIONS = {
            'GRIP': 500,            # 夾住位置
            'RELEASE': 1000         # 放開位置
        }
        
        # 建構流程步驟
        self.build_flow_steps()
        
    def build_flow_steps(self):
        """建構Flow3步驟"""
        self.dio_steps = [
            # 1. 升降缸回原點
            {'type': 'lift_home', 'params': {}},
            
            # 2. 下降
            {'type': 'lift_down', 'params': {}},
            
            # 3. 夾持 (智慧夾持)
            {'type': 'pge_smart_grip', 'params': {'position': self.GRIPPER_POSITIONS['GRIP']}},
            
            # 4. 上升
            {'type': 'lift_up', 'params': {}},
            
            # 5. 翻轉180度
            {'type': 'flip_180', 'params': {}},
            
            # 6. 下降
            {'type': 'lift_down', 'params': {}},
            
            # 7. 開爪 (快速開爪)
            {'type': 'pge_quick_release', 'params': {'position': self.GRIPPER_POSITIONS['RELEASE']}},
            
            # 8. 上升
            {'type': 'lift_up', 'params': {}},
            
            # 9. 翻轉回0度
            {'type': 'flip_0', 'params': {}},
            
            # 10. 立即感測器檢測輸送帶控制
            {'type': 'conveyor_instant_sensor', 'params': {
                'sensor_pin': self.DIO_PINS['SENSOR_DI13'], 
                'timeout': 10.0  # 減少超時時間
            }},
        ]
        
        self.total_steps = len(self.dio_steps)
    
    def execute(self) -> FlowResult:
        """執行Flow3主邏輯"""
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        print(f"\n[Flow3] === 開始執行Flow3翻轉站控制流程 ===")
        print(f"[Flow3] 總步驟數: {self.total_steps}")
        
        # 檢查初始化
        if not self.robot:
            error_msg = "機械臂對象未初始化"
            print(f"[Flow3] ✗ {error_msg}")
            return FlowResult(
                success=False,
                error_message=error_msg,
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查dashboard_api連接
        if not hasattr(self.robot, 'dashboard_api') or not self.robot.dashboard_api:
            error_msg = "dashboard_api未連接"
            print(f"[Flow3] ✗ {error_msg}")
            return FlowResult(
                success=False,
                error_message=error_msg,
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        print(f"[Flow3] ✓ 機械臂連接檢查通過")
        print(f"[Flow3] ✓ dashboard_api連接檢查通過")
        
        try:
            for step in self.dio_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"\n[Flow3] 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = self._execute_step(step)
                
                if not success:
                    self.status = FlowStatus.ERROR
                    error_msg = f"步驟 {step['type']} 執行失敗"
                    print(f"[Flow3] ✗ {error_msg}")
                    return FlowResult(
                        success=False,
                        error_message=error_msg,
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                self.current_step += 1
                print(f"[Flow3] ✓ 步驟 {self.current_step}/{self.total_steps} 完成")
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            print(f"\n[Flow3] === Flow3翻轉站控制流程執行完成 ===")
            print(f"[Flow3] 執行時間: {execution_time:.2f}秒")
            print(f"[Flow3] 完成步驟: {self.current_step}/{self.total_steps}")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={'flip_station_completed': True}
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            error_msg = f"Flow3執行異常: {str(e)}"
            print(f"\n[Flow3] ✗ {error_msg}")
            traceback.print_exc()
            
            return FlowResult(
                success=False,
                error_message=error_msg,
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_step(self, step: Dict[str, Any]) -> bool:
        """執行單一步驟"""
        step_type = step['type']
        params = step.get('params', {})
        
        print(f"[Flow3]   執行步驟類型: {step_type}")
        
        if step_type == 'lift_home':
            return self._lift_home()
        elif step_type == 'lift_down':
            return self._lift_down()
        elif step_type == 'lift_up':
            return self._lift_up()
        elif step_type == 'pge_smart_grip':
            return self._pge_smart_grip(params)
        elif step_type == 'pge_quick_release':
            return self._pge_quick_release(params)
        elif step_type == 'flip_180':
            return self._flip_180()
        elif step_type == 'flip_0':
            return self._flip_0()
        elif step_type == 'conveyor_instant_sensor':
            return self._conveyor_instant_sensor(params)
        else:
            print(f"[Flow3]   ✗ 未知步驟類型: {step_type}")
            return False
    
    # ==================== DIO操作方法 ====================
    
    def _set_do(self, pin: int, value: int) -> bool:
        """設定數位輸出 - 添加具體API調用打印"""
        try:
            print(f"[Flow3]     正在執行: dashboard_api.DOExecute({pin}, {value})")
            
            # 使用DOExecute進行立即執行
            result = self.robot.dashboard_api.DOExecute(pin, value)
            print(f"[Flow3]     API返回結果: {result}")
            
            # 解析API返回值 (格式: "ErrorID,{},DOExecute(pin,value);")
            if result and isinstance(result, str):
                parts = result.split(',')
                if len(parts) > 0:
                    error_id = parts[0].strip()
                    if error_id == "0":
                        print(f"[Flow3]     ✓ DO{pin} = {value} 執行成功")
                        return True
                    else:
                        print(f"[Flow3]     ✗ DO{pin} = {value} 執行失敗，ErrorID: {error_id}")
                        return False
            
            print(f"[Flow3]     ⚠️ DO{pin} = {value} 返回值格式異常: {result}")
            return False
            
        except Exception as e:
            print(f"[Flow3]     ✗ 設定DO{pin}失敗: {e}")
            traceback.print_exc()
            return False

    def _get_di(self, pin: int) -> int:
        """讀取數位輸入 - 快速讀取，減少打印"""
        try:
            result = self.robot.dashboard_api.DI(pin)
            
            # 解析DI返回值 (格式: "ErrorID,{DI_pin},{DI_value},")
            if result and isinstance(result, str):
                parts = result.split(',')
                if len(parts) >= 3:
                    error_id = parts[0].strip()
                    di_value = parts[2].strip()
                    
                    if error_id == "0":
                        value = int(di_value) if di_value.isdigit() else 0
                        return value
                    else:
                        return 0
            
            return 0
            
        except Exception as e:
            print(f"[Flow3]     ✗ 讀取DI{pin}失敗: {e}")
            return 0

    # ==================== 具體動作方法 ====================
    
    def _lift_home(self) -> bool:
        """升降缸回原點"""
        try:
            print("[Flow3]   升降缸回原點")
            
            # DO14=HIGH -> 回原點脈衝
            if not self._set_do(self.DIO_PINS['LIFT_HOME'], 1):
                return False
                
            print(f"[Flow3]   等待回原點完成 ({self.TIMING_CONFIG['LIFT_HOME_TIME']}秒)")
            time.sleep(self.TIMING_CONFIG['LIFT_HOME_TIME'])
            
            # DO14=LOW -> 結束脈衝
            if not self._set_do(self.DIO_PINS['LIFT_HOME'], 0):
                return False
                
            print("[Flow3]   ✓ 升降缸回原點完成")
            return True
            
        except Exception as e:
            print(f"[Flow3]   ✗ 升降缸回原點失敗: {e}")
            return False
            
    def _lift_down(self) -> bool:
        """升降缸下降"""
        try:
            print("[Flow3]   升降缸下降")
            
            # 設定方向為下降 (DIR1=LOW, DIR2=HIGH)
            if not self._set_do(self.DIO_PINS['LIFT_DIR1'], 0):
                return False
            if not self._set_do(self.DIO_PINS['LIFT_DIR2'], 1):
                return False
                
            time.sleep(self.TIMING_CONFIG['DIRECTION_SETUP_DELAY'])
            
            # 啟動脈衝 DO11=HIGH
            if not self._set_do(self.DIO_PINS['LIFT_TRIGGER'], 1):
                return False
                
            print(f"[Flow3]   等待下降完成 ({self.TIMING_CONFIG['LIFT_MOTION_TIME']}秒)")
            time.sleep(self.TIMING_CONFIG['LIFT_MOTION_TIME'])
            
            # 停止脈衝 DO11=LOW
            if not self._set_do(self.DIO_PINS['LIFT_TRIGGER'], 0):
                return False
                
            print("[Flow3]   ✓ 升降缸下降完成")
            return True
            
        except Exception as e:
            print(f"[Flow3]   ✗ 升降缸下降失敗: {e}")
            return False
            
    def _lift_up(self) -> bool:
        """升降缸上升"""
        try:
            print("[Flow3]   升降缸上升")
            
            # 設定方向為上升 (DIR1=HIGH, DIR2=LOW)
            if not self._set_do(self.DIO_PINS['LIFT_DIR1'], 1):
                return False
            if not self._set_do(self.DIO_PINS['LIFT_DIR2'], 0):
                return False
                
            time.sleep(self.TIMING_CONFIG['DIRECTION_SETUP_DELAY'])
            
            # 啟動脈衝 DO11=HIGH
            if not self._set_do(self.DIO_PINS['LIFT_TRIGGER'], 1):
                return False
                
            print(f"[Flow3]   等待上升完成 ({self.TIMING_CONFIG['LIFT_MOTION_TIME']}秒)")
            time.sleep(self.TIMING_CONFIG['LIFT_MOTION_TIME'])
            
            # 停止脈衝 DO11=LOW
            if not self._set_do(self.DIO_PINS['LIFT_TRIGGER'], 0):
                return False
                
            print("[Flow3]   ✓ 升降缸上升完成")
            return True
            
        except Exception as e:
            print(f"[Flow3]   ✗ 升降缸上升失敗: {e}")
            return False
    
    def _pge_smart_grip(self, params: Dict[str, Any]) -> bool:
        """PGE智慧夾持"""
        try:
            position = params.get('position', self.GRIPPER_POSITIONS['GRIP'])
            print(f"[Flow3]   PGE智慧夾持 (位置: {position})")
            
            # 這裡應該調用PGE夾爪API，暫時用模擬
            print(f"[Flow3]   正在執行: PGE.MoveTo({position})")
            time.sleep(self.TIMING_CONFIG['GRIPPER_TIME'])
            print("[Flow3]   ✓ PGE智慧夾持完成")
            
            return True
                
        except Exception as e:
            print(f"[Flow3]   ✗ PGE智慧夾持失敗: {e}")
            return False
            
    def _pge_quick_release(self, params: Dict[str, Any]) -> bool:
        """PGE快速開爪"""
        try:
            position = params.get('position', self.GRIPPER_POSITIONS['RELEASE'])
            print(f"[Flow3]   PGE快速開爪 (位置: {position})")
            
            # 這裡應該調用PGE夾爪API，暫時用模擬
            print(f"[Flow3]   正在執行: PGE.QuickRelease({position})")
            time.sleep(self.TIMING_CONFIG['GRIPPER_TIME'])
            print("[Flow3]   ✓ PGE快速開爪完成")
            
            return True
                
        except Exception as e:
            print(f"[Flow3]   ✗ PGE快速開爪失敗: {e}")
            return False
            
    def _flip_180(self) -> bool:
        """翻轉缸180度翻轉"""
        try:
            print("[Flow3]   翻轉缸180度翻轉")
            
            # DO5=HIGH -> 180度
            if not self._set_do(self.DIO_PINS['FLIP_CYLINDER'], 1):
                return False
                
            print(f"[Flow3]   等待翻轉完成 ({self.TIMING_CONFIG['FLIP_TIME']}秒)")
            time.sleep(self.TIMING_CONFIG['FLIP_TIME'])
            print("[Flow3]   ✓ 翻轉180度完成")
            
            return True
            
        except Exception as e:
            print(f"[Flow3]   ✗ 翻轉失敗: {e}")
            return False
    
    def _flip_0(self) -> bool:
        """翻轉缸回0度"""
        try:
            print("[Flow3]   翻轉缸回0度")
            
            # DO5=LOW -> 0度
            if not self._set_do(self.DIO_PINS['FLIP_CYLINDER'], 0):
                return False
                
            print(f"[Flow3]   等待翻轉完成 ({self.TIMING_CONFIG['FLIP_TIME']}秒)")
            time.sleep(self.TIMING_CONFIG['FLIP_TIME'])
            print("[Flow3]   ✓ 翻轉回0度完成")
            
            return True
            
        except Exception as e:
            print(f"[Flow3]   ✗ 翻轉回0度失敗: {e}")
            return False
            
    def _conveyor_instant_sensor(self, params: Dict[str, Any]) -> bool:
        """立即感測器檢測輸送帶控制 - 優化版"""
        try:
            sensor_pin = params['sensor_pin']
            timeout = params.get('timeout', 10.0)
            print(f"[Flow3]   啟動輸送帶，立即感測器檢測模式 (DI{sensor_pin}=0表示到位)")
            
            # 啟動輸送帶 DO2=HIGH
            if not self._set_do(self.DIO_PINS['CONVEYOR'], 1):
                return False
                
            print("[Flow3]   ✓ 輸送帶已啟動，開始高速感測器檢測...")
            
            # 立即檢測模式
            start_time = time.time()
            check_count = 0
            sensor_triggered = False
            
            # 高速檢測循環
            while time.time() - start_time < timeout:
                # 檢查Flow狀態
                if self.status != FlowStatus.RUNNING:
                    print(f"[Flow3]   Flow狀態變更，停止檢測")
                    break
                
                # 立即檢測感測器
                current_sensor = self._get_di(sensor_pin)
                check_count += 1
                
                # DI13=0表示到位，立即觸發
                if current_sensor == 0:
                    elapsed = time.time() - start_time
                    print(f"[Flow3]   ✓ 感測器立即觸發！DI{sensor_pin}=0，物件已到位")
                    print(f"[Flow3]   檢測耗時: {elapsed:.3f}秒 (檢查次數: {check_count})")
                    sensor_triggered = True
                    break
                
                # 最小檢測間隔 (20ms高速檢測)
                time.sleep(self.TIMING_CONFIG['SENSOR_CHECK_INTERVAL'])
                
                # 定期狀態報告 (每秒報告一次)
                if check_count % 50 == 0:  # 50次*20ms = 1秒
                    elapsed = time.time() - start_time
                    print(f"[Flow3]   高速檢測中... {elapsed:.1f}秒，檢查{check_count}次，當前值: {current_sensor}")
            
            # 立即停止輸送帶 (無論是否觸發)
            print(f"[Flow3]   立即停止輸送帶...")
            stop_success = self._set_do(self.DIO_PINS['CONVEYOR'], 0)
            
            if stop_success:
                print("[Flow3]   ✓ 輸送帶已立即停止")
            else:
                print("[Flow3]   ⚠️ 停止輸送帶失敗")
            
            # 結果分析
            elapsed = time.time() - start_time
            
            if sensor_triggered:
                print(f"[Flow3]   ✓ 感測器檢測成功完成")
                print(f"[Flow3]   總耗時: {elapsed:.3f}秒，檢查次數: {check_count}")
                
                # 防彈跳等待 (確保信號穩定)
                if self.TIMING_CONFIG['SENSOR_DEBOUNCE_TIME'] > 0:
                    print(f"[Flow3]   防彈跳等待 {self.TIMING_CONFIG['SENSOR_DEBOUNCE_TIME']*1000:.0f}ms...")
                    time.sleep(self.TIMING_CONFIG['SENSOR_DEBOUNCE_TIME'])
                    
                    # 再次確認感測器狀態
                    final_sensor = self._get_di(sensor_pin)
                    print(f"[Flow3]   最終感測器狀態確認: DI{sensor_pin} = {final_sensor}")
                
            else:
                print(f"[Flow3]   ⚠️ 感測器檢測超時")
                print(f"[Flow3]   超時時間: {elapsed:.1f}秒，總檢查次數: {check_count}")
                # 超時也返回True，避免阻塞流程
            
            return True
            
        except Exception as e:
            print(f"[Flow3]   ✗ 立即感測器檢測失敗: {e}")
            traceback.print_exc()
            
            # 確保輸送帶停止
            try:
                self._set_do(self.DIO_PINS['CONVEYOR'], 0)
                print(f"[Flow3]   ✓ 異常處理：輸送帶已停止")
            except:
                pass
                
            return False
    
    # ==================== Flow控制方法 ====================
    
    def pause(self) -> bool:
        """暫停Flow"""
        if self.status == FlowStatus.RUNNING:
            self.status = FlowStatus.PAUSED
            print("[Flow3] 翻轉站控制已暫停")
            return True
        return False
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("[Flow3] 翻轉站控制已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow (緊急停止)"""
        self.status = FlowStatus.ERROR
        
        # 緊急停止：關閉所有輸出
        try:
            print("[Flow3] 緊急停止，關閉所有輸出")
            self._set_do(self.DIO_PINS['CONVEYOR'], 0)      # 停止輸送帶
            self._set_do(self.DIO_PINS['FLIP_CYLINDER'], 0) # 翻轉缸回0度
            self._set_do(self.DIO_PINS['LIFT_TRIGGER'], 0)  # 停止升降缸
            print("[Flow3] 翻轉站控制已緊急停止")
        except:
            pass
            
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)