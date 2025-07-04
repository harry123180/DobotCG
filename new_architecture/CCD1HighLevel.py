# -*- coding: utf-8 -*-
"""
CCD1HighLevel.py - CCD1高層API模組
提供簡化的CCD1功能介面，處理複雜的ModbusTCP握手協議和FIFO佇列管理
適用於其他模組import使用
"""

import time
import threading
from typing import Optional, Tuple, List, Dict, Any
from collections import deque
from enum import IntEnum
import logging
from dataclasses import dataclass

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Modbus Client模組導入失敗: {e}")
    MODBUS_AVAILABLE = False


# ==================== 控制指令枚舉 ====================
class CCD1Command(IntEnum):
    """CCD1控制指令枚舉"""
    CLEAR = 0
    CAPTURE = 8
    CAPTURE_DETECT = 16
    INITIALIZE = 32


# ==================== 狀態位枚舉 ====================
class CCD1StatusBits(IntEnum):
    """CCD1狀態位枚舉"""
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3


# ==================== 圓心座標數據結構 ====================
@dataclass
class CircleWorldCoord:
    """圓心世界座標數據"""
    id: int                    # 圓形ID
    world_x: float            # 世界座標X (mm)
    world_y: float            # 世界座標Y (mm)
    pixel_x: int              # 像素座標X
    pixel_y: int              # 像素座標Y
    radius: int               # 半徑 (像素)
    timestamp: str            # 檢測時間戳


# ==================== CCD1高層API類 ====================
class CCD1HighLevelAPI:
    """
    CCD1高層API - 簡化CCD1功能使用
    
    主要功能:
    1. 拍照+檢測指令 (自動處理握手協議)
    2. 獲取物件圓心世界座標 (FIFO佇列管理)
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化CCD1高層API
        
        Args:
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # CCD1寄存器映射 (基地址200)
        self.REGISTERS = {
            'CONTROL_COMMAND': 200,        # 控制指令
            'STATUS_REGISTER': 201,        # 狀態寄存器
            'CIRCLE_COUNT': 240,           # 檢測圓形數量
            'WORLD_COORD_VALID': 256,      # 世界座標有效標誌
        }
        
        # 圓心座標FIFO佇列
        self.coord_queue = deque()  # 圓心座標佇列
        self.queue_lock = threading.Lock()  # 佇列操作鎖
        
        # 狀態追蹤
        self.last_detection_count = 0
        self.operation_timeout = 10.0  # 操作超時時間(秒)
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1HighLevel")
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
        if not self.connected or not self.modbus_client or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
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
        if not self.connected or not self.modbus_client or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            
            return not result.isError()
                
        except Exception as e:
            self.logger.error(f"寫入寄存器失敗: {e}")
            return False
    
    def _read_multiple_registers(self, start_address: int, count: int) -> Optional[List[int]]:
        """讀取多個寄存器"""
        if not self.connected or not self.modbus_client:
            return None
        
        try:
            result = self.modbus_client.read_holding_registers(start_address, count=count, slave=1)
            
            if not result.isError():
                return result.registers
            else:
                return None
                
        except Exception as e:
            self.logger.error(f"讀取多個寄存器失敗: {e}")
            return None
    
    def _wait_for_ready(self, timeout: float = 10.0) -> bool:
        """
        等待CCD1系統Ready狀態
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 是否Ready
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self._read_register('STATUS_REGISTER')
            if status is not None:
                ready = bool(status & (1 << CCD1StatusBits.READY))
                running = bool(status & (1 << CCD1StatusBits.RUNNING))
                alarm = bool(status & (1 << CCD1StatusBits.ALARM))
                
                if alarm:
                    self.logger.warning("CCD1系統處於Alarm狀態")
                    return False
                
                if ready and not running:
                    return True
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待Ready狀態超時: {timeout}秒")
        return False
    
    def _wait_for_command_complete(self, timeout: float = 10.0) -> bool:
        """
        等待指令執行完成
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 指令是否執行完成
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self._read_register('STATUS_REGISTER')
            if status is not None:
                running = bool(status & (1 << CCD1StatusBits.RUNNING))
                alarm = bool(status & (1 << CCD1StatusBits.ALARM))
                
                if alarm:
                    self.logger.warning("CCD1系統執行中發生Alarm")
                    return False
                
                if not running:
                    return True
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待指令完成超時: {timeout}秒")
        return False
    
    def _read_world_coordinates(self) -> List[CircleWorldCoord]:
        """
        讀取世界座標檢測結果
        
        Returns:
            List[CircleWorldCoord]: 圓心世界座標列表
        """
        # 檢查世界座標有效性
        world_coord_valid = self._read_register('WORLD_COORD_VALID')
        if not world_coord_valid:
            self.logger.warning("世界座標無效，可能缺少標定數據")
            return []
        
        # 讀取檢測到的圓形數量
        circle_count = self._read_register('CIRCLE_COUNT')
        if not circle_count or circle_count == 0:
            self.logger.info("未檢測到圓形")
            return []
        
        # 限制最多5個圓形
        circle_count = min(circle_count, 5)
        
        # 讀取像素座標結果 (240-255)
        pixel_registers = self._read_multiple_registers(241, 15)  # 241-255
        
        # 讀取世界座標結果 (257-276)
        world_registers = self._read_multiple_registers(257, 20)  # 257-276
        
        if not pixel_registers or not world_registers:
            self.logger.error("讀取檢測結果失敗")
            return []
        
        coordinates = []
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        
        for i in range(circle_count):
            # 像素座標 (每個圓形3個寄存器: X, Y, Radius)
            pixel_start_idx = i * 3
            if pixel_start_idx + 2 < len(pixel_registers):
                pixel_x = pixel_registers[pixel_start_idx]
                pixel_y = pixel_registers[pixel_start_idx + 1]
                radius = pixel_registers[pixel_start_idx + 2]
            else:
                continue
            
            # 世界座標 (每個圓形4個寄存器: X高位, X低位, Y高位, Y低位)
            world_start_idx = i * 4
            if world_start_idx + 3 < len(world_registers):
                world_x_high = world_registers[world_start_idx]
                world_x_low = world_registers[world_start_idx + 1]
                world_y_high = world_registers[world_start_idx + 2]
                world_y_low = world_registers[world_start_idx + 3]
                
                # 32位合併並轉換精度
                world_x_int = (world_x_high << 16) | world_x_low
                world_y_int = (world_y_high << 16) | world_y_low
                
                # 處理負數 (32位有符號整數)
                if world_x_int >= 2147483648:
                    world_x_int -= 4294967296
                if world_y_int >= 2147483648:
                    world_y_int -= 4294967296
                
                # 恢復精度 (÷100)
                world_x = world_x_int / 100.0
                world_y = world_y_int / 100.0
            else:
                continue
            
            coord = CircleWorldCoord(
                id=i + 1,
                world_x=world_x,
                world_y=world_y,
                pixel_x=pixel_x,
                pixel_y=pixel_y,
                radius=radius,
                timestamp=current_time
            )
            coordinates.append(coord)
        
        return coordinates
    
    def capture_and_detect(self) -> bool:
        """
        執行拍照+檢測指令
        
        本方法處理完整的握手協議，包括:
        1. 檢查Ready狀態
        2. 發送拍照+檢測指令 (16)
        3. 等待執行完成
        4. 讀取檢測結果並更新FIFO佇列
        
        Returns:
            bool: 操作是否成功
        """
        if not self.connected:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            # 1. 等待Ready狀態
            if not self._wait_for_ready(self.operation_timeout):
                self.logger.error("系統未Ready，無法執行檢測")
                return False
            
            # 2. 發送拍照+檢測指令
            self.logger.info("發送拍照+檢測指令...")
            if not self._write_register('CONTROL_COMMAND', CCD1Command.CAPTURE_DETECT):
                self.logger.error("發送檢測指令失敗")
                return False
            
            # 3. 等待執行完成
            if not self._wait_for_command_complete(self.operation_timeout):
                self.logger.error("檢測指令執行失敗或超時")
                return False
            
            # 4. 讀取檢測結果
            coordinates = self._read_world_coordinates()
            
            # 5. 更新FIFO佇列
            with self.queue_lock:
                for coord in coordinates:
                    self.coord_queue.append(coord)
                
                self.last_detection_count = len(coordinates)
            
            # 6. 清空控制指令 (完成握手)
            self._write_register('CONTROL_COMMAND', CCD1Command.CLEAR)
            
            self.logger.info(f"檢測完成，新增 {len(coordinates)} 個圓心座標到佇列")
            return True
            
        except Exception as e:
            self.logger.error(f"拍照檢測執行異常: {e}")
            return False
    
    def get_next_circle_world_coord(self) -> Optional[CircleWorldCoord]:
        """
        獲取下一個物件圓心世界座標
        
        FIFO佇列管理邏輯:
        1. 如果佇列為空，自動觸發拍照+檢測
        2. 從佇列前端取出一個座標
        3. 返回座標，佇列中移除該座標
        
        Returns:
            CircleWorldCoord: 圓心世界座標，如果無可用座標則返回None
        """
        with self.queue_lock:
            # 檢查佇列是否為空
            if len(self.coord_queue) == 0:
                self.logger.info("佇列為空，觸發新的拍照+檢測...")
                
                # 釋放鎖，執行檢測操作
                with_lock_released = True
        
        # 在鎖外執行檢測 (避免死鎖)
        if 'with_lock_released' in locals():
            success = self.capture_and_detect()
            if not success:
                self.logger.error("自動檢測失敗")
                return None
        
        # 重新獲取鎖並取出座標
        with self.queue_lock:
            if len(self.coord_queue) > 0:
                coord = self.coord_queue.popleft()  # FIFO: 從前端取出
                self.logger.info(f"返回圓心座標: ID={coord.id}, 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm")
                return coord
            else:
                self.logger.warning("佇列仍為空，無可用座標")
                return None
    
    def get_queue_status(self) -> Dict[str, Any]:
        """
        獲取佇列狀態資訊
        
        Returns:
            Dict: 包含佇列長度、最後檢測數量等資訊
        """
        with self.queue_lock:
            queue_length = len(self.coord_queue)
            queue_preview = []
            
            # 獲取前3個座標的預覽
            for i, coord in enumerate(list(self.coord_queue)[:3]):
                queue_preview.append({
                    'id': coord.id,
                    'world_x': coord.world_x,
                    'world_y': coord.world_y,
                    'timestamp': coord.timestamp
                })
        
        return {
            'connected': self.connected,
            'queue_length': queue_length,
            'last_detection_count': self.last_detection_count,
            'queue_preview': queue_preview,
            'modbus_server': f"{self.modbus_host}:{self.modbus_port}"
        }
    
    def clear_queue(self):
        """清空座標佇列"""
        with self.queue_lock:
            self.coord_queue.clear()
            self.logger.info("座標佇列已清空")
    
    def is_ready(self) -> bool:
        """
        檢查CCD1系統是否Ready
        
        Returns:
            bool: 系統是否Ready
        """
        if not self.connected:
            return False
        
        status = self._read_register('STATUS_REGISTER')
        if status is not None:
            ready = bool(status & (1 << CCD1StatusBits.READY))
            alarm = bool(status & (1 << CCD1StatusBits.ALARM))
            return ready and not alarm
        
        return False
    
    def get_system_status(self) -> Dict[str, Any]:
        """
        獲取CCD1系統狀態
        
        Returns:
            Dict: 系統狀態資訊
        """
        if not self.connected:
            return {
                'connected': False,
                'ready': False,
                'running': False,
                'alarm': False,
                'initialized': False,
                'world_coord_valid': False
            }
        
        status = self._read_register('STATUS_REGISTER')
        world_coord_valid = self._read_register('WORLD_COORD_VALID')
        
        if status is not None:
            return {
                'connected': True,
                'ready': bool(status & (1 << CCD1StatusBits.READY)),
                'running': bool(status & (1 << CCD1StatusBits.RUNNING)),
                'alarm': bool(status & (1 << CCD1StatusBits.ALARM)),
                'initialized': bool(status & (1 << CCD1StatusBits.INITIALIZED)),
                'world_coord_valid': bool(world_coord_valid) if world_coord_valid is not None else False,
                'status_register_value': status
            }
        
        return {
            'connected': True,
            'ready': False,
            'running': False,
            'alarm': True,
            'initialized': False,
            'world_coord_valid': False,
            'error': '無法讀取狀態寄存器'
        }


# ==================== 使用範例 ====================
def example_usage():
    """使用範例"""
    # 創建CCD1高層API實例
    ccd1 = CCD1HighLevelAPI()
    
    try:
        print("=== CCD1高層API使用範例 ===")
        
        # 檢查系統狀態
        status = ccd1.get_system_status()
        print(f"系統狀態: {status}")
        
        # 檢查佇列狀態
        queue_status = ccd1.get_queue_status()
        print(f"佇列狀態: {queue_status}")
        
        # 手動執行檢測 (可選)
        print("\n手動執行拍照+檢測...")
        success = ccd1.capture_and_detect()
        print(f"檢測結果: {'成功' if success else '失敗'}")
        
        # 逐一獲取圓心座標
        print("\n逐一獲取圓心座標:")
        for i in range(3):  # 嘗試獲取3個座標
            coord = ccd1.get_next_circle_world_coord()
            if coord:
                print(f"圓心{coord.id}: 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm, "
                      f"像素座標=({coord.pixel_x}, {coord.pixel_y}), 半徑={coord.radius}")
            else:
                print(f"第{i+1}次獲取座標失敗")
                break
        
        # 檢查最終佇列狀態
        final_status = ccd1.get_queue_status()
        print(f"\n最終佇列狀態: {final_status}")
        
    finally:
        # 清理資源
        ccd1.disconnect()
        print("\nCCD1連接已斷開")


if __name__ == "__main__":
    example_usage()