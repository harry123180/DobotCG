#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow4_vibration.py - Flow4 震動投料流程 (DIO控制架構版 - 完善版)
基於統一Flow架構的DIO控制執行器
控制震動投料：DO4 HIGH 2秒，同時DO1進行HIGH-LOW脈衝操作2次
增加完整API調用打印和錯誤處理
"""

import time
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class Flow4VibrationFeedExecutor(FlowExecutor):
    """Flow4: 震動投料流程執行器 (DIO控制)"""
    
    def __init__(self):
        super().__init__(flow_id=4, flow_name="震動投料流程")
        self.dio_steps = []
        
        # DIO腳位定義
        self.DIO_PINS = {
            'VIBRATION_CONTROL': 1,    # DO1: 震動控制 (HIGH-LOW脈衝)
            'FEED_ENABLE': 4,          # DO4: 投料使能 (持續HIGH 2秒)
        }
        
        # 時間延遲設定
        self.TIMING_CONFIG = {
            'FEED_DURATION': 0.3,      # DO4持續時間 (秒)
            'PULSE_HIGH_TIME': 0.3,    # DO1 HIGH持續時間 (秒)
            'PULSE_LOW_TIME': 0.3,     # DO1 LOW持續時間 (秒)
            'PULSE_COUNT': 1           # DO1脈衝次數
        }
        
        # 執行緒控制
        self.pulse_thread = None
        self.pulse_thread_running = False
        
        # 建構流程步驟
        self.build_flow_steps()
        
    def build_flow_steps(self):
        """建構Flow4步驟"""
        self.dio_steps = [
            # 1. 同時啟動投料使能和震動脈衝
            {'type': 'start_vibration_feed', 'params': {}},
            
            # 2. 等待流程完成
            {'type': 'wait_completion', 'params': {'duration': self.TIMING_CONFIG['FEED_DURATION']}},
            
            # 3. 確保所有輸出關閉
            {'type': 'stop_all_outputs', 'params': {}}
        ]
        
        self.total_steps = len(self.dio_steps)
    
    def execute(self) -> FlowResult:
        """執行Flow4主邏輯"""
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        print(f"\n[Flow4] === 開始執行Flow4震動投料流程 ===")
        print(f"[Flow4] 總步驟數: {self.total_steps}")
        
        # 檢查初始化
        if not self.robot or not self.robot.is_connected:
            print(f"[Flow4] ✗ 機械臂未連接或未初始化")
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查dashboard_api連接
        if not hasattr(self.robot, 'dashboard_api') or self.robot.dashboard_api is None:
            print(f"[Flow4] ✗ dashboard_api未初始化")
            return FlowResult(
                success=False,
                error_message="dashboard_api未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        print(f"[Flow4] ✓ 機械臂連接檢查通過")
        print(f"[Flow4] ✓ dashboard_api連接檢查通過")
        
        try:
            for step in self.dio_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"\n[Flow4] 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                print(f"[Flow4]   執行步驟類型: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'start_vibration_feed':
                    success = self._execute_start_vibration_feed()
                elif step['type'] == 'wait_completion':
                    success = self._execute_wait_completion(step['params'])
                elif step['type'] == 'stop_all_outputs':
                    success = self._execute_stop_all_outputs()
                else:
                    print(f"[Flow4] ✗ 未知步驟類型: {step['type']}")
                    success = False
                
                if not success:
                    self.status = FlowStatus.ERROR
                    print(f"[Flow4] ✗ 步驟 {self.current_step + 1}/{self.total_steps} 失敗")
                    return FlowResult(
                        success=False,
                        error_message=f"步驟 {step['type']} 執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                print(f"[Flow4] ✓ 步驟 {self.current_step + 1}/{self.total_steps} 完成")
                self.current_step += 1
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            print(f"\n[Flow4] === Flow4震動投料流程執行完成 ===")
            print(f"[Flow4] 執行時間: {execution_time:.2f}秒")
            print(f"[Flow4] 完成步驟: {self.current_step}/{self.total_steps}")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={'vibration_feed_completed': True}
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            print(f"[Flow4] ✗ Flow4執行異常: {str(e)}")
            return FlowResult(
                success=False,
                error_message=f"Flow4執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_start_vibration_feed(self) -> bool:
        """執行震動投料啟動 - 並行控制DO1和DO4"""
        try:
            print(f"[Flow4]   震動投料流程啟動")
            print(f"[Flow4]   DO4將持續HIGH {self.TIMING_CONFIG['FEED_DURATION']}秒")
            print(f"[Flow4]   DO1將執行{self.TIMING_CONFIG['PULSE_COUNT']}次脈衝操作")
            
            # 啟動DO4 (投料使能) - 增加API調用打印
            print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['FEED_ENABLE']}, 1)")
            try:
                result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['FEED_ENABLE'], 1)
                print(f"[Flow4]     API返回結果: {result}")
                print(f"[Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 1 執行成功")
            except Exception as e:
                print(f"[Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}啟動失敗: {e}")
                return False
            
            print(f"[Flow4]   ✓ DO{self.DIO_PINS['FEED_ENABLE']}已啟動")
            
            # 創建執行緒執行DO1脈衝操作
            self.pulse_thread_running = True
            self.pulse_thread = threading.Thread(target=self._execute_do1_pulses, daemon=True)
            self.pulse_thread.start()
            
            print(f"[Flow4]   ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝執行緒已啟動")
            return True
            
        except Exception as e:
            print(f"[Flow4]   ✗ 震動投料啟動失敗: {e}")
            return False
    
    def _execute_do1_pulses(self):
        """執行DO1脈衝操作 - 在獨立執行緒中運行"""
        try:
            print(f"[Flow4]   DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝操作開始")
            
            for pulse_num in range(self.TIMING_CONFIG['PULSE_COUNT']):
                if not self.pulse_thread_running:
                    break
                    
                print(f"[Flow4]   DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝 {pulse_num + 1}/{self.TIMING_CONFIG['PULSE_COUNT']}")
                
                # DO1 HIGH - 增加API調用打印
                print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 1)")
                try:
                    result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 1)
                    print(f"[Flow4]     API返回結果: {result}")
                    print(f"[Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 1 執行成功")
                    print(f"[Flow4]     DO{self.DIO_PINS['VIBRATION_CONTROL']} HIGH (持續{self.TIMING_CONFIG['PULSE_HIGH_TIME']}秒)")
                except Exception as e:
                    print(f"[Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1} HIGH失敗: {e}")
                    continue
                
                time.sleep(self.TIMING_CONFIG['PULSE_HIGH_TIME'])
                
                # DO1 LOW - 增加API調用打印
                print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 0)")
                try:
                    result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 0)
                    print(f"[Flow4]     API返回結果: {result}")
                    print(f"[Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 0 執行成功")
                    print(f"[Flow4]     DO{self.DIO_PINS['VIBRATION_CONTROL']} LOW (持續{self.TIMING_CONFIG['PULSE_LOW_TIME']}秒)")
                except Exception as e:
                    print(f"[Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1} LOW失敗: {e}")
                    continue
                
                time.sleep(self.TIMING_CONFIG['PULSE_LOW_TIME'])
                
                print(f"[Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1}完成")
            
            print(f"[Flow4]   ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}所有脈衝操作完成")
            
        except Exception as e:
            print(f"[Flow4]   ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝操作失敗: {e}")
        finally:
            self.pulse_thread_running = False
    
    def _execute_wait_completion(self, params: Dict[str, Any]) -> bool:
        """等待流程完成"""
        try:
            duration = params.get('duration', 2.0)
            print(f"[Flow4]   等待流程完成 ({duration}秒)")
            print(f"[Flow4]   此期間DO4保持HIGH，DO1執行脈衝操作")
            
            time.sleep(duration)
            
            # 等待脈衝執行緒結束
            if self.pulse_thread and self.pulse_thread.is_alive():
                print(f"[Flow4]   等待DO1脈衝執行緒完成...")
                self.pulse_thread_running = False
                self.pulse_thread.join(timeout=2.0)
                
                if self.pulse_thread.is_alive():
                    print(f"[Flow4]   ⚠️ DO1脈衝執行緒未正常結束")
                else:
                    print(f"[Flow4]   ✓ DO1脈衝執行緒已結束")
            
            print(f"[Flow4]   ✓ 流程完成等待結束")
            return True
            
        except Exception as e:
            print(f"[Flow4]   ✗ 等待流程完成失敗: {e}")
            return False
    
    def _execute_stop_all_outputs(self) -> bool:
        """確保所有輸出關閉"""
        try:
            print(f"[Flow4]   關閉所有DO輸出")
            success = True
            
            # 關閉DO4 (投料使能) - 增加API調用打印
            print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['FEED_ENABLE']}, 0)")
            try:
                result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['FEED_ENABLE'], 0)
                print(f"[Flow4]     API返回結果: {result}")
                print(f"[Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 0 執行成功")
            except Exception as e:
                print(f"[Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}關閉失敗: {e}")
                success = False
            
            # 關閉DO1 (震動控制) - 增加API調用打印
            print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 0)")
            try:
                result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 0)
                print(f"[Flow4]     API返回結果: {result}")
                print(f"[Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 0 執行成功")
            except Exception as e:
                print(f"[Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}關閉失敗: {e}")
                success = False
            
            # 確保脈衝執行緒停止
            if self.pulse_thread_running:
                self.pulse_thread_running = False
                if self.pulse_thread and self.pulse_thread.is_alive():
                    self.pulse_thread.join(timeout=1.0)
            
            if success:
                print(f"[Flow4]   ✓ 所有DO輸出已關閉")
            else:
                print(f"[Flow4]   ⚠️ 部分DO輸出關閉失敗")
                
            return success
            
        except Exception as e:
            print(f"[Flow4]   ✗ 關閉DO輸出失敗: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("[Flow4] Flow4已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("[Flow4] Flow4已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow4執行"""
        try:
            print(f"[Flow4] 正在停止Flow4執行...")
            self.status = FlowStatus.ERROR
            
            # 停止脈衝執行緒
            if self.pulse_thread_running:
                self.pulse_thread_running = False
                if self.pulse_thread and self.pulse_thread.is_alive():
                    self.pulse_thread.join(timeout=2.0)
            
            # 關閉所有輸出
            self._execute_stop_all_outputs()
            
            print(f"[Flow4] ✓ Flow4已停止")
            return True
            
        except Exception as e:
            print(f"[Flow4] ✗ Flow4停止過程出錯: {e}")
            return False
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)

