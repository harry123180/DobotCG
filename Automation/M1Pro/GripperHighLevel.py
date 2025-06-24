#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GripperHighLevel.py - 夾爪高層API模組
提供簡化的夾爪功能介面，處理複雜的ModbusTCP指令發送和狀態確認
適用於機械臂流程中import使用，大幅簡化夾爪操作編碼複雜度
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


# ==================== 夾爪高層API類 ====================
class GripperHighLevelAPI:
    """
    夾爪高層API - 簡化夾爪功能使用
    
    主要功能:
    1. 智能夾取 - 自動判斷夾取成功
    2. 快速指令 - 發了就走不等確認
    3. 確認指令 - 等待動作完成
    4. 位置控制 - 精確位置移動
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化夾爪高層API
        
        Args:
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # PGC夾爪寄存器映射 (基地址520)
        self.PGC_REGISTERS = {
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
        
        # 指令ID計數器
        self.command_id_counter = 1
        
        # 操作超時設定
        self.operation_timeout = 10.0  # 動作超時時間(秒)
        self.quick_timeout = 0.5       # 快速指令超時時間(秒)
        
        # 設置日誌
        self.logger = logging.getLogger("GripperHighLevel")
        self.logger.setLevel(logging.INFO)
        
        # 自動連接
        self.connect()
    
    def connect(self) -> bool:
        """
        連接到Modbus TCP服務器
        
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
                self.logger.info(f"Modbus TCP連接成功: {self.modbus_host}:{self.modbus_port}")
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
    
    def _read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or not self.modbus_client or register_name not in self.PGC_REGISTERS:
            return None
        
        try:
            address = self.PGC_REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            self.logger.error(f"讀取寄存器失敗: {e}")
            return None
    
    def _write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or not self.modbus_client or register_name not in self.PGC_REGISTERS:
            return False
        
        try:
            address = self.PGC_REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            
            return not result.isError()
                
        except Exception as e:
            self.logger.error(f"寫入寄存器失敗: {e}")
            return False
    
    def _send_command(self, command: GripperCommand, param1: int = 0, param2: int = 0) -> bool:
        """發送夾爪指令"""
        if not self.connected:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            # 生成唯一指令ID
            cmd_id = self.command_id_counter
            self.command_id_counter += 1
            if self.command_id_counter > 65535:
                self.command_id_counter = 1
            
            # 發送指令
            success = True
            success &= self._write_register('COMMAND', command.value)
            success &= self._write_register('PARAM1', param1)
            success &= self._write_register('PARAM2', param2)
            success &= self._write_register('COMMAND_ID', cmd_id)
            
            if success:
                self.logger.info(f"發送夾爪指令: cmd={command.name}, param1={param1}, id={cmd_id}")
            else:
                self.logger.error(f"發送夾爪指令失敗: cmd={command.name}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"發送指令異常: {e}")
            return False
    
    def _wait_for_completion(self, timeout: float = 10.0) -> bool:
        """等待動作完成"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 檢查夾持狀態
            grip_status = self._read_register('GRIP_STATUS')
            if grip_status is not None:
                if grip_status in [GripperStatus.REACHED, GripperStatus.GRIPPED]:
                    return True
                elif grip_status == GripperStatus.DROPPED:
                    self.logger.warning("夾爪掉落狀態")
                    return False
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待動作完成超時: {timeout}秒")
        return False
    
    def _check_initialized(self) -> bool:
        """檢查夾爪是否已初始化"""
        device_status = self._read_register('DEVICE_STATUS')
        return device_status == 1 if device_status is not None else False
    
    def _get_current_position(self) -> Optional[int]:
        """獲取當前位置"""
        return self._read_register('CURRENT_POSITION')
    
    # ==================== 基本操作API ====================
    
    def initialize(self, wait_completion: bool = True) -> bool:
        """
        初始化夾爪
        
        Args:
            wait_completion: 是否等待初始化完成
            
        Returns:
            bool: 操作是否成功
        """
        self.logger.info("夾爪初始化...")
        
        if not self._send_command(GripperCommand.INITIALIZE):
            return False
        
        if wait_completion:
            # 等待初始化完成，使用較長超時時間
            return self._wait_for_completion(timeout=15.0)
        else:
            return True
    
    def stop(self) -> bool:
        """
        停止夾爪動作
        
        Returns:
            bool: 操作是否成功
        """
        self.logger.info("夾爪停止")
        return self._send_command(GripperCommand.STOP)
    
    # ==================== 快速操作API (發了就走) ====================
    
    def quick_open(self) -> bool:
        """
        快速開啟夾爪 (發了就走，不等確認)
        
        Returns:
            bool: 指令發送是否成功
        """
        self.logger.info("夾爪快速開啟 (不等確認)")
        return self._send_command(GripperCommand.QUICK_OPEN)
    
    def quick_close(self) -> bool:
        """
        快速關閉夾爪 (發了就走，不等確認)
        
        Returns:
            bool: 指令發送是否成功
        """
        self.logger.info("夾爪快速關閉 (不等確認)")
        return self._send_command(GripperCommand.QUICK_CLOSE)
    
    def quick_move_to(self, position: int) -> bool:
        """
        快速移動到指定位置 (發了就走，不等確認)
        
        Args:
            position: 目標位置 (0-1000)
            
        Returns:
            bool: 指令發送是否成功
        """
        self.logger.info(f"夾爪快速移動到位置 {position} (不等確認)")
        return self._send_command(GripperCommand.MOVE_ABS, position)
    
    # ==================== 確認操作API (等待完成) ====================
    
    def open_and_wait(self, timeout: float = None) -> bool:
        """
        開啟夾爪並等待完成
        
        Args:
            timeout: 超時時間，None使用預設值
            
        Returns:
            bool: 操作是否成功
        """
        timeout = timeout or self.operation_timeout
        self.logger.info(f"夾爪開啟並等待完成 (超時: {timeout}秒)")
        
        if not self._send_command(GripperCommand.QUICK_OPEN):
            return False
        
        return self._wait_for_completion(timeout)
    
    def close_and_wait(self, timeout: float = None) -> bool:
        """
        關閉夾爪並等待完成
        
        Args:
            timeout: 超時時間，None使用預設值
            
        Returns:
            bool: 操作是否成功
        """
        timeout = timeout or self.operation_timeout
        self.logger.info(f"夾爪關閉並等待完成 (超時: {timeout}秒)")
        
        if not self._send_command(GripperCommand.QUICK_CLOSE):
            return False
        
        return self._wait_for_completion(timeout)
    
    def move_to_and_wait(self, position: int, timeout: float = None) -> bool:
        """
        移動到指定位置並等待完成
        
        Args:
            position: 目標位置 (0-1000)
            timeout: 超時時間，None使用預設值
            
        Returns:
            bool: 操作是否成功
        """
        timeout = timeout or self.operation_timeout
        self.logger.info(f"夾爪移動到位置 {position} 並等待完成 (超時: {timeout}秒)")
        
        if not self._send_command(GripperCommand.MOVE_ABS, position):
            return False
        
        return self._wait_for_completion(timeout)
    
    # ==================== 智能操作API ====================
    
    def smart_grip(self, target_position: int = 420, max_attempts: int = 3) -> bool:
        """
        智能夾取 - 自動判斷夾取成功
        
        基於MVP.py的智能夾取邏輯:
        1. 記錄初始位置
        2. 撐開到目標位置
        3. 智能判斷成功條件:
           - 位置差≤20: 到達目標位置
           - 移動>100且狀態=夾住: 撐開固定物件
           - 位置穩定且移動>50: 撐開成功
        
        Args:
            target_position: 目標撐開位置 (預設420)
            max_attempts: 最大嘗試次數
            
        Returns:
            bool: 夾取是否成功
        """
        self.logger.info(f"智能夾取開始 - 目標位置: {target_position}, 最大嘗試: {max_attempts}")
        
        for attempt in range(max_attempts):
            self.logger.info(f"智能夾取嘗試 {attempt + 1}/{max_attempts}")
            
            # 記錄初始位置
            initial_position = self._get_current_position()
            if initial_position is None:
                self.logger.error("無法讀取初始位置")
                continue
            
            # 發送撐開指令
            if not self._send_command(GripperCommand.MOVE_ABS, target_position):
                self.logger.error("發送撐開指令失敗")
                continue
            
            # 智能等待和判斷
            max_position_reached = initial_position
            start_time = time.time()
            
            while time.time() - start_time < self.operation_timeout:
                current_position = self._get_current_position()
                grip_status = self._read_register('GRIP_STATUS')
                
                if current_position is None or grip_status is None:
                    time.sleep(0.2)
                    continue
                
                # 記錄最大位置
                if current_position > max_position_reached:
                    max_position_reached = current_position
                
                position_diff = abs(current_position - target_position)
                movement_from_start = abs(current_position - initial_position)
                
                # 智能判斷成功條件 (來自MVP.py)
                if position_diff <= 20:
                    self.logger.info(f"夾爪到達目標位置: {current_position}")
                    return True
                elif movement_from_start > 100 and grip_status == GripperStatus.GRIPPED:
                    self.logger.info(f"夾爪撐開固定物件: {current_position}")
                    return True
                elif current_position == max_position_reached and movement_from_start > 50:
                    # 檢查位置穩定性
                    stable_count = 0
                    for _ in range(3):
                        time.sleep(0.1)
                        check_pos = self._get_current_position()
                        if check_pos == current_position:
                            stable_count += 1
                    
                    if stable_count >= 2:
                        self.logger.info(f"夾爪位置穩定，撐開成功: {current_position}")
                        return True
                
                time.sleep(0.2)
            
            # 超時檢查
            final_position = self._get_current_position()
            if final_position:
                final_movement = abs(final_position - initial_position)
                
                if final_movement > 100:
                    self.logger.info(f"超時但有顯著移動，認為成功: {final_position}")
                    return True
                else:
                    self.logger.warning(f"嘗試{attempt + 1}失敗: 移動不足 {final_movement}")
        
        self.logger.error(f"智能夾取失敗 - 已嘗試{max_attempts}次")
        return False
    
    def smart_release(self, release_position: int = 50) -> bool:
        """
        智能釋放 - 移動到釋放位置
        
        Args:
            release_position: 釋放位置 (預設50)
            
        Returns:
            bool: 釋放是否成功
        """
        self.logger.info(f"智能釋放 - 移動到位置: {release_position}")
        return self.move_to_and_wait(release_position)
    
    # ==================== 狀態查詢API ====================
    
    def is_initialized(self) -> bool:
        """檢查夾爪是否已初始化"""
        return self._check_initialized()
    
    def is_connected(self) -> bool:
        """檢查夾爪是否連接"""
        connect_status = self._read_register('CONNECT_STATUS')
        return connect_status == 1 if connect_status is not None else False
    
    def get_position(self) -> Optional[int]:
        """獲取當前位置"""
        return self._get_current_position()
    
    def get_status(self) -> Dict[str, Any]:
        """
        獲取夾爪完整狀態
        
        Returns:
            Dict: 夾爪狀態資訊
        """
        if not self.connected:
            return {
                'connected': False,
                'initialized': False,
                'position': None,
                'grip_status': None,
                'error': '未連接'
            }
        
        try:
            module_status = self._read_register('MODULE_STATUS')
            connect_status = self._read_register('CONNECT_STATUS')
            device_status = self._read_register('DEVICE_STATUS')
            grip_status = self._read_register('GRIP_STATUS')
            position = self._read_register('CURRENT_POSITION')
            error_count = self._read_register('ERROR_COUNT')
            
            # 狀態文字映射
            grip_status_text = {
                0: "運動中", 1: "到達", 2: "夾住", 3: "掉落"
            }.get(grip_status, "未知") if grip_status is not None else "無法讀取"
            
            return {
                'connected': self.connected,
                'module_online': module_status == 1 if module_status is not None else False,
                'device_connected': connect_status == 1 if connect_status is not None else False,
                'initialized': device_status == 1 if device_status is not None else False,
                'position': position,
                'grip_status': grip_status,
                'grip_status_text': grip_status_text,
                'error_count': error_count,
                'modbus_server': f"{self.modbus_host}:{self.modbus_port}"
            }
            
        except Exception as e:
            return {
                'connected': self.connected,
                'error': f"狀態讀取失敗: {e}"
            }
    
    # ==================== 設定參數API ====================
    
    def set_force(self, force: int) -> bool:
        """
        設定夾持力道
        
        Args:
            force: 力道值 (20-100)
            
        Returns:
            bool: 設定是否成功
        """
        if not 20 <= force <= 100:
            self.logger.error(f"力道值超出範圍: {force} (應為20-100)")
            return False
        
        self.logger.info(f"設定夾持力道: {force}")
        return self._send_command(GripperCommand.SET_FORCE, force)
    
    def set_speed(self, speed: int) -> bool:
        """
        設定移動速度
        
        Args:
            speed: 速度值 (1-100)
            
        Returns:
            bool: 設定是否成功
        """
        if not 1 <= speed <= 100:
            self.logger.error(f"速度值超出範圍: {speed} (應為1-100)")
            return False
        
        self.logger.info(f"設定移動速度: {speed}")
        return self._send_command(GripperCommand.SET_SPEED, speed)


# ==================== 使用範例 ====================
def example_usage():
    """使用範例"""
    # 創建夾爪高層API實例
    gripper = GripperHighLevelAPI()
    
    try:
        print("=== 夾爪高層API使用範例 ===")
        
        # 檢查夾爪狀態
        status = gripper.get_status()
        print(f"夾爪狀態: {status}")
        
        # 初始化夾爪
        if not gripper.is_initialized():
            print("夾爪未初始化，開始初始化...")
            if gripper.initialize(wait_completion=True):
                print("✓ 夾爪初始化成功")
            else:
                print("✗ 夾爪初始化失敗")
                return
        
        # 設定參數
        gripper.set_force(50)
        gripper.set_speed(80)
        
        # 智能夾取演示
        print("\n=== 智能夾取演示 ===")
        if gripper.smart_grip(target_position=420):
            print("✓ 智能夾取成功")
            
            time.sleep(2)  # 模擬處理時間
            
            # 智能釋放
            if gripper.smart_release(release_position=50):
                print("✓ 智能釋放成功")
            else:
                print("✗ 智能釋放失敗")
        else:
            print("✗ 智能夾取失敗")
        
        # 快速操作演示
        print("\n=== 快速操作演示 ===")
        gripper.quick_close()  # 發了就走
        time.sleep(1)
        gripper.quick_open()   # 發了就走
        
        # 確認操作演示
        print("\n=== 確認操作演示 ===")
        if gripper.close_and_wait(timeout=5.0):
            print("✓ 關閉並等待完成")
        
        if gripper.open_and_wait(timeout=5.0):
            print("✓ 開啟並等待完成")
        
    finally:
        # 清理資源
        gripper.disconnect()
        print("\n夾爪連接已斷開")


if __name__ == "__main__":
    example_usage()