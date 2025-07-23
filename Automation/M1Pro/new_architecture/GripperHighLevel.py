#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GripperHighLevel_Optimized.py - PGC夾爪高層API優化版
專門配合Gripper_Optimized.py實現最佳性能
重點優化quick_close和smart_release的執行速度
"""

import time
import threading
import os
from typing import Optional, Dict, Any
from enum import IntEnum
import logging
from logging.handlers import RotatingFileHandler

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"Modbus Client模組導入失敗: {e}")
    MODBUS_AVAILABLE = False


# ==================== 夾爪指令枚舉 ====================
class GripperCommand(IntEnum):
    """夾爪指令枚舉"""
    NOP = 0             # 無操作
    INITIALIZE = 1      # 初始化/回零
    STOP = 2           # 停止
    MOVE_ABS = 3       # 絕對位置移動
    SET_FORCE = 5      # 設定力道
    SET_SPEED = 6      # 設定速度
    QUICK_OPEN = 7     # 快速開啟
    QUICK_CLOSE = 8    # 快速關閉


# ==================== 夾爪狀態枚舉 ====================
class GripperStatus(IntEnum):
    """夾爪狀態枚舉"""
    MOVING = 0         # 運動中
    REACHED = 1        # 到達位置
    GRIPPED = 2        # 夾住物體
    DROPPED = 3        # 掉落


# ==================== 夾爪類型枚舉 ====================
class GripperType(IntEnum):
    """夾爪類型枚舉 - 僅支援PGC"""
    PGC = 1           # PGC夾爪


# ==================== 優化版夾爪高層API類 ====================
class GripperHighLevelAPI:
    """
    夾爪高層API優化版 - 專注PGC夾爪高性能控制
    
    主要優化:
    1. 批量寄存器操作減少Modbus通訊次數
    2. 適應性輪詢降低CPU負載
    3. 優化的超時設定提升響應速度
    4. 智能快取減少重複讀取
    5. 配合Gripper_Optimized.py的20ms循環
    """
    
    def __init__(self, gripper_type: GripperType = GripperType.PGC, 
                 modbus_host: str = "127.0.0.1", modbus_port: int = 502,
                 auto_initialize: bool = True):
        """
        初始化優化版夾爪高層API
        
        Args:
            gripper_type: 夾爪類型 (僅支援PGC)
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
            auto_initialize: 是否自動初始化夾爪
        """
        if gripper_type != GripperType.PGC:
            raise ValueError(f"此版本僅支援PGC夾爪，不支援類型: {gripper_type}")
        
        self.gripper_type = gripper_type
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.auto_initialize = auto_initialize
        
        # 高性能線程鎖
        self.modbus_lock = threading.RLock()
        
        # PGC寄存器映射
        self._setup_pgc_registers()
        
        # 指令ID計數器
        self.command_id_counter = 1
        
        # 優化的超時設定
        self.operation_timeout = 5.0    # 一般操作超時
        self.quick_timeout = 2.0        # 快速操作超時
        self.ultra_quick_timeout = 1.0  # 超快速操作超時
        
        # 狀態快取
        self.status_cache = {}
        self.cache_timeout = 0.05  # 50ms快取有效期
        
        # 設置日誌
        self.logger = self._setup_logging()
        
        # 初始化狀態
        self.initialized = False
        
        # 性能統計
        self.performance_stats = {
            'command_count': 0,
            'avg_response_time': 0,
            'max_response_time': 0,
            'success_rate': 0
        }
        
        # 自動連接
        self.connect()
        
    def _setup_logging(self):
        """設置高性能logging配置"""
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler = RotatingFileHandler(
            os.path.join(log_dir, 'GripperHighLevel_Optimized.log'),
            maxBytes=10*1024*1024,
            backupCount=7,
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        
        logger = logging.getLogger("GripperHighLevelOptimized")
        logger.setLevel(logging.INFO)  # 優化：減少debug輸出
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
        
    def _setup_pgc_registers(self):
        """設定PGC夾爪寄存器映射"""
        self.REGISTERS = {
            # 狀態寄存器 (500-519)
            'MODULE_STATUS': 500,      # 模組狀態: 0=離線, 1=在線
            'CONNECT_STATUS': 501,     # 連接狀態: 0=斷開, 1=已連接
            'DEVICE_STATUS': 502,      # 設備狀態(初始化狀態)
            'ERROR_COUNT': 503,        # 錯誤計數
            'GRIP_STATUS': 504,        # 夾持狀態
            'CURRENT_POSITION': 505,   # 當前位置
            
            # 指令寄存器 (520-529)
            'COMMAND': 520,            # 指令代碼
            'PARAM1': 521,             # 參數1
            'PARAM2': 522,             # 參數2
            'COMMAND_ID': 523,         # 指令ID
        }
        
    def connect(self) -> bool:
        """連接到Modbus TCP服務器 - 優化版"""
        if not MODBUS_AVAILABLE:
            self.logger.error("Modbus Client不可用")
            return False
        
        with self.modbus_lock:
            try:
                if self.modbus_client:
                    self.modbus_client.close()
                
                self.logger.info(f"正在連接Modbus TCP服務器: {self.modbus_host}:{self.modbus_port}")
                
                self.modbus_client = ModbusTcpClient(
                    host=self.modbus_host,
                    port=self.modbus_port,
                    timeout=1.0  # 優化：減少連接超時
                )
                
                if self.modbus_client.connect():
                    self.connected = True
                    self.logger.info("PGC夾爪Modbus連接成功")
                    
                    if self.auto_initialize:
                        self.logger.info("開始自動初始化PGC夾爪")
                        if self.initialize(timeout=3.0):  # 優化：減少初始化超時
                            self.initialized = True
                            self.logger.info("PGC夾爪初始化成功")
                        else:
                            self.logger.warning("PGC夾爪初始化失敗，但連接正常")
                    
                    return True
                else:
                    self.logger.error(f"Modbus TCP連接失敗: {self.modbus_host}:{self.modbus_port}")
                    self.connected = False
                    return False
                    
            except Exception as e:
                self.logger.error(f"Modbus TCP連接異常: {e}", exc_info=True)
                self.connected = False
                return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        with self.modbus_lock:
            if self.modbus_client and self.connected:
                try:
                    self.modbus_client.close()
                    self.logger.info("Modbus TCP連接已斷開")
                except Exception as e:
                    self.logger.error(f"斷開連接時發生異常: {e}", exc_info=True)
            
            self.connected = False
            self.modbus_client = None
    
    def _write_register(self, register_name: str, value: int) -> bool:
        """寫入單個寄存器"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return False
        
        with self.modbus_lock:
            try:
                address = self.REGISTERS[register_name]
                result = self.modbus_client.write_register(address=address, value=value)
                
                if not (hasattr(result, 'isError') and result.isError()):
                    return True
                else:
                    self.logger.error(f"寫入寄存器失敗: {register_name}[{address}] = {value}")
                    return False
                    
            except Exception as e:
                self.logger.error(f"寫入寄存器異常: {register_name} = {value}, 錯誤: {e}", exc_info=True)
                return False
    
    def _write_registers_batch(self, start_address: int, values: list) -> bool:
        """批量寫入寄存器 - 核心優化"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return False
        
        with self.modbus_lock:
            try:
                result = self.modbus_client.write_registers(address=start_address, values=values)
                
                if not (hasattr(result, 'isError') and result.isError()):
                    return True
                else:
                    self.logger.error(f"批量寫入寄存器失敗: 地址{start_address}, 數值{values}")
                    return False
                    
            except Exception as e:
                self.logger.error(f"批量寫入寄存器異常: 地址{start_address}, 錯誤: {e}", exc_info=True)
                return False
    
    def _read_register(self, register_name: str, use_cache: bool = True) -> Optional[int]:
        """讀取寄存器 - 帶快取優化"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return None
        
        current_time = time.time()
        
        # 檢查快取
        if use_cache and register_name in self.status_cache:
            cached_data = self.status_cache[register_name]
            if current_time - cached_data['timestamp'] < self.cache_timeout:
                return cached_data['value']
        
        with self.modbus_lock:
            try:
                address = self.REGISTERS[register_name]
                result = self.modbus_client.read_holding_registers(address=address, count=1)
                
                if not (hasattr(result, 'isError') and result.isError()) and len(result.registers) > 0:
                    value = result.registers[0]
                    
                    # 更新快取
                    if use_cache:
                        self.status_cache[register_name] = {
                            'value': value,
                            'timestamp': current_time
                        }
                    
                    return value
                else:
                    self.logger.error(f"讀取寄存器失敗: {register_name}[{address}]")
                    return None
                    
            except Exception as e:
                self.logger.error(f"讀取寄存器異常: {register_name}, 錯誤: {e}", exc_info=True)
                return None
    
    def _log_gripper_status(self, operation: str, success: bool):
        """記錄夾爪狀態"""
        try:
            if not success:  # 只在失敗時記錄詳細狀態
                position = self.get_current_position()
                grip_status = self._read_register('GRIP_STATUS', use_cache=False)
                self.logger.error(f"{operation}失敗 - 位置: {position}, 夾持狀態: {grip_status}")
            else:
                self.logger.info(f"{operation}成功")
                
        except Exception as e:
            self.logger.error(f"記錄夾爪狀態時發生異常: {e}", exc_info=True)
    
    # ==================== 核心優化方法 ====================
    
    def quick_close(self) -> bool:
        """優化的快速關閉 - 狀態確認版"""
        target_position = 0
        self.logger.info(f"PGC快速關閉: {target_position}")
        
        start_time = time.time()
        
        # 使用優化的移動並等待方法
        success = self._move_to_and_wait_optimized(target_position, timeout=self.quick_timeout)
        
        # 性能統計
        response_time = time.time() - start_time
        self._update_performance_stats(response_time, success)
        
        self._log_gripper_status("快速關閉", success)
        return success
        
    def smart_release(self, release_position: int = 470) -> bool:
        """優化的智能釋放 - 狀態確認版"""
        self.logger.info(f"PGC智能釋放到位置: {release_position}")
        
        start_time = time.time()
        
        # 使用優化的移動並等待方法
        success = self._move_to_and_wait_optimized(release_position, timeout=self.quick_timeout)
        
        # 性能統計
        response_time = time.time() - start_time
        self._update_performance_stats(response_time, success)
        
        self._log_gripper_status("智能釋放", success)
        return success
    
    def _move_to_and_wait_optimized(self, position: int, timeout: float = 5.0) -> bool:
        """
        優化的移動到位置並等待完成 - 配合高速Gripper.py
        
        關鍵優化:
        1. 批量寫入減少Modbus通訊次數
        2. 適應性輪詢減少CPU負載
        3. 智能快取減少重複讀取
        """
        if not self._send_command_optimized(GripperCommand.MOVE_ABS, position):
            self.logger.error("移動指令發送失敗")
            return False
        
        # 使用優化的等待完成
        return self._wait_for_completion_optimized(timeout)
        
    def _send_command_optimized(self, command: GripperCommand, param1: int = 0, param2: int = 0) -> bool:
        """優化的指令發送 - 批量寫入核心優化"""
        try:
            # 獲取指令ID
            cmd_id = self.command_id_counter
            self.command_id_counter += 1
            if self.command_id_counter > 65535:
                self.command_id_counter = 1
            
            # 批量寫入所有指令參數 - 關鍵優化：一次Modbus通訊完成4個寄存器寫入
            register_values = [
                int(command),  # COMMAND寄存器
                param1,        # PARAM1寄存器
                param2,        # PARAM2寄存器
                cmd_id         # COMMAND_ID寄存器
            ]
            
            # 一次性寫入4個連續寄存器
            success = self._write_registers_batch(self.REGISTERS['COMMAND'], register_values)
            
            if success:
                self.performance_stats['command_count'] += 1
            
            return success
            
        except Exception as e:
            self.logger.error(f"優化指令發送失敗: {command.name}({param1}, {param2}), 錯誤: {e}", exc_info=True)
            return False
    
    def _wait_for_completion_optimized(self, timeout: float) -> bool:
        """
        優化的等待完成 - 適應性輪詢策略
        
        關鍵優化:
        1. 首次立即檢查可能已完成
        2. 適應性輪詢間隔：10ms -> 20ms -> 50ms -> 100ms
        3. 配合Gripper.py的20ms循環
        """
        start_time = time.time()
        
        # 首次檢查 - 可能已經完成
        status = self._read_register('GRIP_STATUS', use_cache=False)
        if status in [GripperStatus.REACHED, GripperStatus.GRIPPED]:
            elapsed_time = time.time() - start_time
            self.logger.debug(f"動作立即完成，耗時: {elapsed_time:.3f}秒")
            return True
        
        # 適應性輪詢策略 - 配合20ms Gripper循環
        poll_intervals = [0.01, 0.02, 0.05, 0.1]  # 10ms -> 20ms -> 50ms -> 100ms
        poll_index = 0
        
        while time.time() - start_time < timeout:
            # 根據已等待時間動態調整輪詢間隔
            elapsed = time.time() - start_time
            if elapsed > 0.3 and poll_index == 0:  # 300ms後降低頻率
                poll_index = 1
            elif elapsed > 0.8 and poll_index == 1:  # 800ms後進一步降低
                poll_index = 2
            elif elapsed > 2.0 and poll_index == 2:  # 2秒後最低頻率
                poll_index = 3
            
            time.sleep(poll_intervals[poll_index])
            
            # 檢查夾爪狀態 - 不使用快取確保準確性
            status = self._read_register('GRIP_STATUS', use_cache=False)
            if status in [GripperStatus.REACHED, GripperStatus.GRIPPED]:
                elapsed_time = time.time() - start_time
                self.logger.debug(f"動作完成，耗時: {elapsed_time:.3f}秒，狀態: {status}")
                return True
            elif status == GripperStatus.DROPPED:
                elapsed_time = time.time() - start_time
                self.logger.warning(f"動作失敗(掉落)，耗時: {elapsed_time:.3f}秒")
                return True
        
        self.logger.warning(f"PGC動作完成等待超時 (超時時間: {timeout}秒)")
        return False
    
    def _update_performance_stats(self, response_time: float, success: bool):
        """更新性能統計"""
        self.performance_stats['avg_response_time'] = (
            self.performance_stats['avg_response_time'] * 0.9 + response_time * 0.1
        )
        
        if response_time > self.performance_stats['max_response_time']:
            self.performance_stats['max_response_time'] = response_time
        
        # 成功率統計 (簡單移動平均)
        if hasattr(self, '_success_history'):
            self._success_history.append(success)
            if len(self._success_history) > 100:
                self._success_history.pop(0)
            self.performance_stats['success_rate'] = sum(self._success_history) / len(self._success_history)
        else:
            self._success_history = [success]
            self.performance_stats['success_rate'] = 1.0 if success else 0.0
    
    # ==================== 保持向後相容的API ====================
    
    def initialize(self, wait_completion: bool = True, timeout: float = None) -> bool:
        """初始化夾爪 - 優化版"""
        self.logger.info("開始初始化PGC夾爪")
        
        actual_timeout = timeout if timeout is not None else self.operation_timeout
        
        try:
            if not self._send_command_optimized(GripperCommand.INITIALIZE):
                self._log_gripper_status("初始化指令發送", False)
                return False
                
            if wait_completion:
                success = self._wait_for_completion_optimized(actual_timeout)
                self._log_gripper_status("初始化", success)
                if success:
                    self.initialized = True
                return success
            
            return True
                
        except Exception as e:
            self.logger.error(f"初始化失敗: {e}", exc_info=True)
            self._log_gripper_status("初始化", False)
            return False
    
    def move_to_and_wait(self, position: int) -> bool:
        """移動到指定位置並等待完成 - 保持相容"""
        return self._move_to_and_wait_optimized(position, timeout=self.operation_timeout)
    
    def smart_grip(self, target_position: int = 420, max_attempts: int = 3) -> bool:
        """智能夾取 - 優化版"""
        self.logger.info(f"PGC智能夾取到位置: {target_position}, 最大嘗試次數: {max_attempts}")
        
        for attempt in range(max_attempts):
            try:
                initial_pos = self.get_current_position()
                
                if not self._send_command_optimized(GripperCommand.MOVE_ABS, target_position):
                    continue
                
                if not self._wait_for_completion_optimized(self.operation_timeout):
                    continue
                
                final_pos = self.get_current_position()
                if final_pos is None:
                    continue
                
                position_diff = abs(target_position - final_pos)
                
                if position_diff > 20:
                    self.logger.info(f"PGC智能夾取成功 (嘗試{attempt + 1}/{max_attempts})")
                    return True
                    
            except Exception as e:
                self.logger.error(f"第{attempt + 1}次嘗試異常: {e}", exc_info=True)
                
        self.logger.error(f"PGC智能夾取失敗")
        return False
    
    def get_current_position(self) -> Optional[int]:
        """取得當前位置 - 使用快取優化"""
        return self._read_register('CURRENT_POSITION', use_cache=True)
    
    def get_grip_status(self) -> Optional[int]:
        """取得夾持狀態 - 使用快取優化"""
        return self._read_register('GRIP_STATUS', use_cache=True)
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """取得性能統計"""
        return self.performance_stats.copy()


# ==================== 使用範例 ====================
if __name__ == "__main__":
    gripper = GripperHighLevelAPI(gripper_type=GripperType.PGC, auto_initialize=False)
    
    try:
        if gripper.connected:
            print("PGC夾爪連接成功")
            
            # 測試quick_close性能
            print("\n=== 快速關閉性能測試 ===")
            start_time = time.time()
            if gripper.quick_close():
                elapsed = time.time() - start_time
                print(f"快速關閉成功，耗時: {elapsed:.3f}秒")
            
            time.sleep(1)
            
            # 測試smart_release性能
            print("\n=== 智能釋放性能測試 ===")
            start_time = time.time()
            if gripper.smart_release(470):
                elapsed = time.time() - start_time
                print(f"智能釋放成功，耗時: {elapsed:.3f}秒")
            
            # 顯示性能統計
            print("\n=== 性能統計 ===")
            stats = gripper.get_performance_stats()
            print(f"平均響應時間: {stats['avg_response_time']:.3f}秒")
            print(f"最大響應時間: {stats['max_response_time']:.3f}秒")
            print(f"成功率: {stats['success_rate']:.1%}")
            print(f"指令總數: {stats['command_count']}")
            
        else:
            print("PGC夾爪連接失敗")
            
    except KeyboardInterrupt:
        print("\n程序中斷")
    finally:
        gripper.disconnect()
        print("PGC夾爪已斷開連接")