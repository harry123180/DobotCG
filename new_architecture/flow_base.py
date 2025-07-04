#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flow_base.py - Flow架構基類模組
定義統一的Flow執行器介面和基本功能
"""

import time
from abc import ABC, abstractmethod
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
    total_steps: int = 0
    flow_data: Dict[str, Any] = None


class FlowExecutor(ABC):
    """Flow執行器基底類別"""
    
    def __init__(self, flow_id: int, flow_name: str):
        self.flow_id = flow_id
        self.flow_name = flow_name
        self.status = FlowStatus.IDLE
        self.current_step = 0
        self.total_steps = 0
        self.start_time = 0.0
        self.last_error = ""
        
        # 共用資源 (由Main傳入)
        self.robot = None
        self.state_machine = None
        self.external_modules = {}
        
    def initialize(self, robot, state_machine, external_modules):
        """初始化Flow (由Main呼叫)"""
        self.robot = robot
        self.state_machine = state_machine
        self.external_modules = external_modules
        
    @abstractmethod
    def execute(self) -> FlowResult:
        """執行Flow主邏輯"""
        pass
        
    @abstractmethod
    def pause(self) -> bool:
        """暫停Flow"""
        pass
        
    @abstractmethod
    def resume(self) -> bool:
        """恢復Flow"""
        pass
        
    @abstractmethod
    def stop(self) -> bool:
        """停止Flow"""
        pass
        
    @abstractmethod
    def get_progress(self) -> int:
        """取得執行進度 (0-100)"""
        pass
        
    def get_status_info(self) -> Dict[str, Any]:
        """取得狀態資訊"""
        return {
            'flow_id': self.flow_id,
            'flow_name': self.flow_name,
            'status': self.status.value,
            'current_step': self.current_step,
            'total_steps': self.total_steps,
            'progress': self.get_progress(),
            'last_error': self.last_error
        }
        
    def execute_with_retry(self, max_retries: int = 2) -> FlowResult:
        """執行Flow with重試機制"""
        for attempt in range(max_retries + 1):
            try:
                result = self.execute()
                if result.success:
                    return result
                    
                print(f"Flow{self.flow_id}執行失敗 (嘗試 {attempt + 1}/{max_retries + 1}): {result.error_message}")
                
                if attempt < max_retries:
                    # 重試前的恢復動作
                    self._recovery_action()
                    time.sleep(1.0)
                    
            except Exception as e:
                print(f"Flow{self.flow_id}執行異常 (嘗試 {attempt + 1}/{max_retries + 1}): {e}")
                
        return FlowResult(
            success=False,
            error_message=f"Flow{self.flow_id}在{max_retries + 1}次嘗試後仍然失敗"
        )
        
    def _recovery_action(self):
        """恢復動作 - 子類別可覆寫"""
        # 基本恢復動作：重置狀態
        self.status = FlowStatus.IDLE
        self.current_step = 0
        
        # 停止機械臂運動
        if self.robot and hasattr(self.robot, 'is_connected') and self.robot.is_connected:
            try:
                if hasattr(self.robot, 'dashboard_api'):
                    # 嘗試暫停和繼續來清除運動狀態
                    self.robot.dashboard_api.pause()
                    time.sleep(0.5)
                    self.robot.dashboard_api.Continue()
            except:
                pass