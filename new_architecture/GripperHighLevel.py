#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GripperHighLevel.py - 夾爪高層API模組 (PyModbus 3.9.2修正版)
支援PGC和PGE兩種夾爪類型
完全修正PyModbus 3.9.2 API調用格式
"""

import time
import threading
from typing import Optional, Dict, Any
from enum import IntEnum
import logging

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Modbus Client模組導入失敗: {e}")
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
    """夾爪類型枚舉"""
    PGC = 1           # PGC夾爪 (原有)
    PGE = 2           # PGE夾爪 (新增)


# ==================== 夾爪高層API類 ====================
class GripperHighLevelAPI:
    """
    夾爪高層API - 支援PGC和PGE兩種夾爪類型
    
    主要功能:
    1. 智能夾取 - 自動判斷夾取成功
    2. 快速指令 - 發了就走不等確認
    3. 確認指令 - 等待動作完成
    4. 位置控制 - 精確位置移動
    5. PGE夾爪專用控制
    """
    
    def __init__(self, gripper_type: GripperType = GripperType.PGC, 
                 modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化夾爪高層API
        
        Args:
            gripper_type: 夾爪類型 (PGC或PGE)
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.gripper_type = gripper_type
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 根據夾爪類型設定寄存器映射
        if gripper_type == GripperType.PGC:
            self._setup_pgc_registers()
        elif gripper_type == GripperType.PGE:
            self._setup_pge_registers()
        else:
            raise ValueError(f"不支援的夾爪類型: {gripper_type}")
        
        # 指令ID計數器
        self.command_id_counter = 1
        
        # 操作超時設定
        self.operation_timeout = 10.0  # 動作超時時間(秒)
        self.quick_timeout = 0.5       # 快速指令超時時間(秒)
        
        # 設置日誌
        self.logger = logging.getLogger(f"GripperHighLevel_{gripper_type.name}")
        self.logger.setLevel(logging.INFO)
        
        # 初始化狀態
        self.initialized = False
        
        # 自動連接
        self.connect()
        
    def _setup_pgc_registers(self):
        """設定PGC夾爪寄存器映射 (基地址520)"""
        self.REGISTERS = {
            # 狀態寄存器 (500-519)
            'MODULE_STATUS': 500,      # 模組狀態
            'CONNECT_STATUS': 501,     # 連接狀態
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
        
    def _setup_pge_registers(self):
        """設定PGE夾爪寄存器映射 (基於Modbus地址表)"""
        self.REGISTERS = {
            # PGE控制寄存器 (基於圖片中的地址表)
            'INITIALIZE': 256,         # 0x0100: 初始化實爪
            'FORCE': 257,              # 0x0101: 力值
            'POSITION': 259,           # 0x0103: 運動到指定位置
            'SPEED': 260,              # 0x0104: 以設定速度運行
            
            # PGE狀態寄存器
            'INIT_STATUS': 512,        # 0x0200: 初始化狀態
            'GRIP_STATUS': 513,        # 0x0201: 夾持狀態
            'CURRENT_POSITION': 514,   # 0x0202: 當前位置
        }
        
    def connect(self) -> bool:
        """
        連接到Modbus TCP服務器 - PyModbus 3.9.2修正版
        
        Returns:
            bool: 連接是否成功
        """
        if not MODBUS_AVAILABLE:
            self.logger.error("Modbus Client不可用")
            return False
        
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            self.logger.info(f"正在連接Modbus TCP服務器: {self.modbus_host}:{self.modbus_port}")
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self.logger.info(f"✓ {self.gripper_type.name}夾爪Modbus連接成功")
                
                # 自動初始化
                self.logger.info(f"開始自動初始化{self.gripper_type.name}夾爪...")
                if self.initialize():
                    self.initialized = True
                    self.logger.info(f"✓ {self.gripper_type.name}夾爪初始化成功")
                else:
                    self.logger.warning(f"⚠️ {self.gripper_type.name}夾爪初始化失敗，但連接正常")
                
                return True
            else:
                self.logger.error(f"Modbus TCP連接失敗: {self.modbus_host}:{self.modbus_port}")
                self.connected = False
                return False
                
        except Exception as e:
            self.logger.error(f"Modbus TCP連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
                self.logger.info("Modbus TCP連接已斷開")
            except:
                pass
        
        self.connected = False
        self.modbus_client = None
    
    def _write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器 - PyModbus 3.9.2修正版"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            address = self.REGISTERS[register_name]
            # 修正：使用命名參數
            result = self.modbus_client.write_register(address=address, value=value)
            
            if not (hasattr(result, 'isError') and result.isError()):
                self.logger.debug(f"寫入寄存器 {register_name}[{address}] = {value}")
                return True
            else:
                self.logger.error(f"寫入寄存器失敗: {result}")
                return False
                
        except Exception as e:
            self.logger.error(f"寫入寄存器異常: {e}")
            return False
    
    def _read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器 - PyModbus 3.9.2修正版"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return None
        
        try:
            address = self.REGISTERS[register_name]
            # 修正：使用命名參數
            result = self.modbus_client.read_holding_registers(address=address, count=1)
            
            if not (hasattr(result, 'isError') and result.isError()) and len(result.registers) > 0:
                value = result.registers[0]
                self.logger.debug(f"讀取寄存器 {register_name}[{address}] = {value}")
                return value
            else:
                self.logger.error(f"讀取寄存器失敗: {result}")
                return None
                
        except Exception as e:
            self.logger.error(f"讀取寄存器異常: {e}")
            return None
    
    # ==================== 初始化API ====================
    
    def initialize(self, wait_completion: bool = True) -> bool:
        """
        初始化夾爪
        
        Args:
            wait_completion: 是否等待初始化完成
            
        Returns:
            bool: 初始化是否成功
        """
        self.logger.info(f"開始初始化{self.gripper_type.name}夾爪")
        
        try:
            if self.gripper_type == GripperType.PGC:
                return self._initialize_pgc(wait_completion)
            elif self.gripper_type == GripperType.PGE:
                return self._initialize_pge(wait_completion)
            else:
                return False
                
        except Exception as e:
            self.logger.error(f"初始化失敗: {e}")
            return False
    
    def _initialize_pgc(self, wait_completion: bool) -> bool:
        """初始化PGC夾爪"""
        if not self._send_command(GripperCommand.INITIALIZE):
            return False
            
        if wait_completion:
            return self._wait_for_completion(self.operation_timeout)
        return True
        
    def _initialize_pge(self, wait_completion: bool) -> bool:
        """初始化PGE夾爪"""
        # PGE夾爪初始化：寫入INITIALIZE寄存器
        if not self._write_register('INITIALIZE', 1):
            return False
            
        if wait_completion:
            # 等待初始化完成 - 檢查INIT_STATUS
            start_time = time.time()
            while time.time() - start_time < self.operation_timeout:
                status = self._read_register('INIT_STATUS')
                if status == 1:  # 初始化完成
                    return True
                time.sleep(0.1)
            return False
        return True
    
    # ==================== 通用API (相容PGC) ====================
    
    def smart_grip(self, target_position: int = 420, max_attempts: int = 3) -> bool:
        """
        智能夾取 - 自動適配夾爪類型
        
        Args:
            target_position: 目標位置
            max_attempts: 最大嘗試次數
            
        Returns:
            bool: 夾取是否成功
        """
        if self.gripper_type == GripperType.PGE:
            return self.pge_smart_grip(target_position, max_attempts)
        else:
            # PGC原有邏輯
            return self._pgc_smart_grip(target_position, max_attempts)
    
    def quick_open(self, position: int = None) -> bool:
        """
        快速開啟 - 自動適配夾爪類型
        
        Args:
            position: 開啟位置
            
        Returns:
            bool: 操作是否成功
        """
        if self.gripper_type == GripperType.PGE:
            return self.pge_quick_open(position)
        else:
            # PGC原有邏輯
            return self._send_command(GripperCommand.QUICK_OPEN)
    
    def quick_close(self) -> bool:
        """快速關閉 - 通用方法"""
        if self.gripper_type == GripperType.PGE:
            return self._write_register('POSITION', 500)  # PGE夾爪關閉位置
        else:
            return self._send_command(GripperCommand.QUICK_CLOSE)
    
    def smart_release(self, release_position: int = 50) -> bool:
        """智能釋放 - 通用方法"""
        if self.gripper_type == GripperType.PGE:
            return self.pge_quick_open(release_position)
        else:
            return self.move_to_and_wait(release_position)
    
    # ==================== PGE專用方法 ====================
    
    def pge_smart_grip(self, target_position: int = 500, max_attempts: int = 3) -> bool:
        """PGE智能夾取"""
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return False
        
        self.logger.info(f"PGE智能夾取到位置: {target_position}")
        
        for attempt in range(max_attempts):
            try:
                # 移動到目標位置
                if not self._write_register('POSITION', target_position):
                    continue
                
                # 等待夾取完成並檢查狀態
                if self._wait_for_pge_grip_completion():
                    self.logger.info(f"✓ PGE夾取成功 (嘗試{attempt + 1}/{max_attempts})")
                    return True
                else:
                    self.logger.warning(f"⚠️ PGE夾取失敗 (嘗試{attempt + 1}/{max_attempts})")
                    
            except Exception as e:
                self.logger.error(f"PGE夾取異常: {e}")
                
        self.logger.error(f"✗ PGE夾取失敗，已用完{max_attempts}次嘗試")
        return False
    
    def pge_quick_open(self, position: int = None) -> bool:
        """PGE快速開啟"""
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return False
        
        if position is None:
            position = 1000  # PGE預設開啟位置
        
        self.logger.info(f"PGE快速開啟到位置: {position}")
        return self._write_register('POSITION', position)
    
    def pge_set_force(self, force_percent: int) -> bool:
        """設定PGE夾爪力道"""
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return False
        
        if not 1 <= force_percent <= 100:
            self.logger.error(f"PGE力道超出範圍: {force_percent} (應為1-100)")
            return False
        
        self.logger.info(f"設定PGE夾爪力道: {force_percent}%")
        return self._write_register('FORCE', force_percent)
    
    def pge_set_speed(self, speed_percent: int) -> bool:
        """設定PGE夾爪速度"""
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return False
        
        if not 1 <= speed_percent <= 100:
            self.logger.error(f"PGE速度超出範圍: {speed_percent} (應為1-100)")
            return False
        
        self.logger.info(f"設定PGE夾爪速度: {speed_percent}%")
        return self._write_register('SPEED', speed_percent)
    
    def pge_get_position(self) -> Optional[int]:
        """取得PGE夾爪當前位置"""
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return None
        
        return self._read_register('CURRENT_POSITION')
    
    def pge_get_grip_status(self) -> Optional[int]:
        """取得PGE夾爪夾持狀態"""
        if self.gripper_type != GripperType.PGE:
            self.logger.error("此方法僅適用於PGE夾爪")
            return None
        
        return self._read_register('GRIP_STATUS')
    
    def _wait_for_pge_grip_completion(self, timeout: float = 5.0) -> bool:
        """等待PGE夾取完成並檢查夾持狀態"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            grip_status = self.pge_get_grip_status()
            
            if grip_status == 2:  # 夾住物體
                return True
            elif grip_status == 1:  # 到達位置但未夾住
                return False
            elif grip_status == 3:  # 掉落
                return False
            
            time.sleep(0.1)
        
        self.logger.warning("PGE夾取完成檢查超時")
        return False
    
    # ==================== PGC相容方法 ====================
    
    def _pgc_smart_grip(self, target_position: int, max_attempts: int) -> bool:
        """PGC智能夾取邏輯 (保持原有功能)"""
        self.logger.info(f"PGC智能夾取到位置: {target_position}")
        
        for attempt in range(max_attempts):
            try:
                # 記錄初始位置
                initial_pos = self.get_current_position()
                if initial_pos is None:
                    self.logger.warning("無法讀取初始位置")
                    initial_pos = 0
                
                # 移動到目標位置
                if not self._send_command(GripperCommand.MOVE_ABS, target_position):
                    continue
                
                # 等待運動完成
                if not self._wait_for_completion(self.operation_timeout):
                    continue
                
                # 檢查是否夾到物體
                final_pos = self.get_current_position()
                if final_pos is None:
                    continue
                
                # 如果位置差異大於閾值，表示夾到物體
                position_diff = abs(target_position - final_pos)
                if position_diff > 20:  # 閾值可調整
                    self.logger.info(f"✓ PGC智能夾取成功 (位置差異: {position_diff})")
                    return True
                else:
                    self.logger.warning(f"⚠️ PGC未夾到物體 (位置差異: {position_diff})")
                    
            except Exception as e:
                self.logger.error(f"PGC智能夾取異常: {e}")
                
        self.logger.error(f"✗ PGC智能夾取失敗，已用完{max_attempts}次嘗試")
        return False
    
    def _send_command(self, command: GripperCommand, param1: int = 0, param2: int = 0) -> bool:
        """發送PGC夾爪指令"""
        if self.gripper_type != GripperType.PGC:
            return False
        
        try:
            # 獲取指令ID
            cmd_id = self.command_id_counter
            self.command_id_counter += 1
            
            # 寫入指令參數
            if not self._write_register('PARAM1', param1):
                return False
            if not self._write_register('PARAM2', param2):
                return False
            if not self._write_register('COMMAND_ID', cmd_id):
                return False
            
            # 發送指令
            if not self._write_register('COMMAND', int(command)):
                return False
            
            self.logger.debug(f"發送PGC指令: {command.name}({param1}, {param2}) ID={cmd_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"發送PGC指令失敗: {e}")
            return False
    
    def _wait_for_completion(self, timeout: float) -> bool:
        """等待PGC夾爪動作完成"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 檢查夾爪狀態
            status = self._read_register('GRIP_STATUS')
            if status in [GripperStatus.REACHED, GripperStatus.GRIPPED]:
                return True
            elif status == GripperStatus.DROPPED:
                return False
            
            time.sleep(0.1)
        
        self.logger.warning("PGC動作完成等待超時")
        return False
    
    def get_current_position(self) -> Optional[int]:
        """取得當前位置 - 通用方法"""
        return self._read_register('CURRENT_POSITION')
    
    def move_to_and_wait(self, position: int) -> bool:
        """移動到指定位置並等待完成 - PGC專用"""
        if self.gripper_type != GripperType.PGC:
            return False
        
        if not self._send_command(GripperCommand.MOVE_ABS, position):
            return False
        
        return self._wait_for_completion(self.operation_timeout)
    
    # ==================== 狀態查詢API ====================
    
    def get_status(self) -> Dict[str, Any]:
        """取得夾爪狀態資訊"""
        status_info = {
            'gripper_type': self.gripper_type.name,
            'connected': self.connected,
            'initialized': self.initialized,
            'current_position': self.get_current_position()
        }
        
        if self.gripper_type == GripperType.PGE:
            status_info.update({
                'init_status': self._read_register('INIT_STATUS'),
                'grip_status': self.pge_get_grip_status()
            })
        elif self.gripper_type == GripperType.PGC:
            status_info.update({
                'device_status': self._read_register('DEVICE_STATUS'),
                'grip_status': self._read_register('GRIP_STATUS'),
                'error_count': self._read_register('ERROR_COUNT')
            })
        
        return status_info