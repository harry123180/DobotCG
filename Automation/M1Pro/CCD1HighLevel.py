# -*- coding: utf-8 -*-
"""
CCD1HighLevel_Enhanced.py - CCD1高層API模組 (修正版 - 自動收取寄存器數據)
提供簡化的CCD1功能介面，處理複雜的ModbusTCP握手協議和FIFO佇列管理
重點功能：
1. 自動檢查是否需要拍照檢測 (檢查240地址是否為0)
2. 自動執行拍照檢測流程 (200=16 → 等待201=8 → 200=0)
3. 全域FIFO佇列管理，自動補充檢測結果
4. 🔥 新增：自動收取現有寄存器數據到FIFO佇列
5. 提供統一的get_next_object()方法供Flow1調用
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


# ==================== 檢測結果狀態枚舉 ====================
class DetectionResult(IntEnum):
    """檢測結果狀態"""
    SUCCESS = 0          # 檢測成功，有物體
    NO_OBJECTS = 1       # 檢測成功，但無物體 (需要補料)
    DETECTION_FAILED = 2 # 檢測失敗 (系統錯誤)
    SYSTEM_NOT_READY = 3 # 系統未準備就緒


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
    r: float = 0.0            # 旋轉角度 (可由Flow1設定)


# ==================== CCD1增強高層API類 ====================
class CCD1HighLevelAPI:
    """
    CCD1高層API - 修正版自動收取寄存器數據
    
    核心功能：
    1. 自動檢測是否需要拍照 (240地址檢查)
    2. 自動執行完整握手協議 (200=16 → 201等待8 → 200=0)
    3. 🔥 自動收取現有寄存器數據到FIFO佇列
    4. 全域FIFO佇列管理
    5. 統一的get_next_object()介面
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
            'PIXEL_COORD_START': 241,      # 像素座標起始地址 (241-255)
            'WORLD_COORD_VALID': 256,      # 世界座標有效標誌
            'WORLD_COORD_START': 257,      # 世界座標起始地址 (257-276)
        }
        
        # === 關鍵：全域FIFO佇列管理 ===
        self.global_coord_queue = deque()         # 全域圓心座標佇列
        self.queue_lock = threading.Lock()        # 佇列操作鎖
        self.detection_in_progress = False        # 檢測進行中標誌
        self.detection_lock = threading.Lock()    # 檢測操作鎖
        
        # 狀態追蹤
        self.last_detection_count = 0
        self.total_detections = 0
        self.operation_timeout = 15.0              # 操作超時時間(秒)
        self.detection_retry_count = 3             # 檢測重試次數
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1HighLevelEnhanced")
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
    
    def _write_multiple_registers(self, start_address: int, values: List[int]) -> bool:
        """寫入多個寄存器"""
        if not self.connected or not self.modbus_client:
            return False
        
        try:
            result = self.modbus_client.write_registers(start_address, values, slave=1)
            return not result.isError()
        except Exception as e:
            self.logger.error(f"寫入多個寄存器失敗: {e}")
            return False
    
    def check_detection_needed(self) -> bool:
        """
        檢查是否需要進行拍照檢測
        
        檢查邏輯：
        1. 檢查240寄存器 (CIRCLE_COUNT) 是否為0
        2. 如果為0表示無檢測結果，需要拍照檢測
        3. 如果非0表示有檢測結果，不需要拍照
        
        Returns:
            bool: True=需要拍照檢測, False=不需要拍照檢測
        """
        try:
            circle_count = self._read_register('CIRCLE_COUNT')
            
            if circle_count is None:
                self.logger.error("無法讀取CIRCLE_COUNT寄存器")
                return True  # 無法讀取時，假設需要檢測
            
            need_detection = (circle_count == 0)
            
            if need_detection:
                self.logger.info(f"檢測需求檢查: 240寄存器={circle_count}, 需要拍照檢測")
            else:
                self.logger.info(f"檢測需求檢查: 240寄存器={circle_count}, 無需拍照檢測")
            
            return need_detection
            
        except Exception as e:
            self.logger.error(f"檢查檢測需求失敗: {e}")
            return True  # 異常時假設需要檢測
    
    def auto_collect_existing_data(self) -> DetectionResult:
        """
        🔥 新增：自動收取現有寄存器數據到FIFO佇列
        
        當發現240寄存器有數值但FIFO佇列為空時調用
        讀取現有的座標數據，加入FIFO佇列，然後清零寄存器
        
        Returns:
            DetectionResult: 收取結果狀態
        """
        try:
            self.logger.info("開始自動收取現有寄存器數據...")
            
            # 1. 讀取圓形數量
            circle_count = self._read_register('CIRCLE_COUNT')
            if circle_count is None or circle_count == 0:
                self.logger.warning("240寄存器為0或無法讀取，無法收取數據")
                return DetectionResult.NO_OBJECTS
            
            self.logger.info(f"發現現有數據: {circle_count}個圓形座標")
            
            # 2. 檢查世界座標有效性
            world_coord_valid = self._read_register('WORLD_COORD_VALID')
            if not world_coord_valid:
                self.logger.warning("世界座標無效，無法收取數據")
                return DetectionResult.DETECTION_FAILED
            
            # 3. 限制最多5個圓形
            circle_count = min(circle_count, 5)
            
            # 4. 讀取像素座標數據 (241-255)
            pixel_registers = self._read_multiple_registers(
                self.REGISTERS['PIXEL_COORD_START'], 15
            )
            
            # 5. 讀取世界座標數據 (257-276)
            world_registers = self._read_multiple_registers(
                self.REGISTERS['WORLD_COORD_START'], 20
            )
            
            if not pixel_registers or not world_registers:
                self.logger.error("讀取座標數據失敗")
                return DetectionResult.DETECTION_FAILED
            
            # 6. 解析座標數據並加入FIFO佇列
            new_coordinates = []
            current_time = time.strftime("%Y-%m-%d %H:%M:%S")
            
            for i in range(circle_count):
                # 解析像素座標 (每個圓形3個寄存器: X, Y, Radius)
                pixel_start_idx = i * 3
                if pixel_start_idx + 2 < len(pixel_registers):
                    pixel_x = pixel_registers[pixel_start_idx]
                    pixel_y = pixel_registers[pixel_start_idx + 1]
                    radius = pixel_registers[pixel_start_idx + 2]
                else:
                    continue
                
                # 解析世界座標 (每個圓形4個寄存器: X高位, X低位, Y高位, Y低位)
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
                
                # 創建座標對象
                coord = CircleWorldCoord(
                    id=self.total_detections + i + 1,  # 全域唯一ID
                    world_x=world_x,
                    world_y=world_y,
                    pixel_x=pixel_x,
                    pixel_y=pixel_y,
                    radius=radius,
                    timestamp=current_time
                )
                new_coordinates.append(coord)
                
                self.logger.info(f"  收取圓形{i+1}: 世界座標=({world_x:.2f}, {world_y:.2f})mm, "
                               f"像素座標=({pixel_x}, {pixel_y}), 半徑={radius}")
            
            # 7. 更新全域FIFO佇列
            with self.queue_lock:
                for coord in new_coordinates:
                    self.global_coord_queue.append(coord)
                
                self.last_detection_count = len(new_coordinates)
                self.total_detections += len(new_coordinates)
            
            # 8. 🔥 清零寄存器數據 (防止重複收取)
            self._clear_detection_registers(circle_count)
            
            self.logger.info(f"自動收取完成: 收取 {len(new_coordinates)} 個座標，已清零寄存器")
            return DetectionResult.SUCCESS
            
        except Exception as e:
            self.logger.error(f"自動收取現有數據失敗: {e}")
            return DetectionResult.DETECTION_FAILED
    
    def _clear_detection_registers(self, circle_count: int):
        """
        清零檢測結果寄存器 (防止重複收取)
        
        Args:
            circle_count: 要清零的圓形數量
        """
        try:
            self.logger.info(f"清零檢測結果寄存器 (圓形數量: {circle_count})...")
            
            # 1. 清零圓形數量寄存器 (240)
            if not self._write_register('CIRCLE_COUNT', 0):
                self.logger.warning("清零240寄存器失敗")
            else:
                self.logger.info("✓ 240寄存器已清零")
            
            # 2. 清零像素座標寄存器 (241-255)
            pixel_clear_count = min(circle_count * 3, 15)  # 每個圓形3個寄存器，最多15個
            if pixel_clear_count > 0:
                pixel_clear_values = [0] * pixel_clear_count
                if self._write_multiple_registers(self.REGISTERS['PIXEL_COORD_START'], pixel_clear_values):
                    self.logger.info(f"✓ 像素座標寄存器已清零 (241-{240+pixel_clear_count})")
                else:
                    self.logger.warning("清零像素座標寄存器失敗")
            
            # 3. 清零世界座標寄存器 (257-276)
            world_clear_count = min(circle_count * 4, 20)  # 每個圓形4個寄存器，最多20個
            if world_clear_count > 0:
                world_clear_values = [0] * world_clear_count
                if self._write_multiple_registers(self.REGISTERS['WORLD_COORD_START'], world_clear_values):
                    self.logger.info(f"✓ 世界座標寄存器已清零 (257-{256+world_clear_count})")
                else:
                    self.logger.warning("清零世界座標寄存器失敗")
            
            self.logger.info("寄存器清零完成，防止下次重複收取")
            
        except Exception as e:
            self.logger.error(f"清零寄存器失敗: {e}")
    
    def execute_capture_and_detect(self) -> DetectionResult:
        """
        執行拍照+檢測指令 (修正版 - 清零後檢查結果)
        
        修正執行流程：
        1. 檢查系統Ready狀態 (201寄存器bit0=1)
        2. 發送拍照+檢測指令 (200寄存器=16)
        3. 等待執行完成 (201寄存器=8, Ready=0且Running=0)
        4. 🔥 清除控制指令 (200寄存器=0) - 此時結果才會出現
        5. 🔥 等待結果穩定 (給CCD1模組時間寫入結果)
        6. 讀取並處理檢測結果
        7. 更新全域FIFO佇列
        
        Returns:
            DetectionResult: 檢測結果狀態
        """
        if not self.connected:
            self.logger.error("Modbus未連接")
            return DetectionResult.SYSTEM_NOT_READY
        
        retry_count = 0
        max_retries = self.detection_retry_count
        
        while retry_count < max_retries:
            try:
                retry_count += 1
                self.logger.info(f"執行拍照+檢測 (第{retry_count}/{max_retries}次)...")
                
                # 1. 等待Ready狀態
                if not self._wait_for_ready(self.operation_timeout):
                    self.logger.error("系統未Ready，無法執行檢測")
                    if retry_count < max_retries:
                        time.sleep(2.0)
                        continue
                    return DetectionResult.SYSTEM_NOT_READY
                
                # 2. 發送拍照+檢測指令
                self.logger.info("發送拍照+檢測指令 (200=16)...")
                if not self._write_register('CONTROL_COMMAND', CCD1Command.CAPTURE_DETECT):
                    self.logger.error("發送檢測指令失敗")
                    if retry_count < max_retries:
                        time.sleep(1.0)
                        continue
                    return DetectionResult.DETECTION_FAILED
                
                # 3. 等待執行完成 (狀態寄存器=8)
                self.logger.info("等待執行完成 (201=8)...")
                if not self._wait_for_completion(self.operation_timeout):
                    self.logger.error("檢測指令執行失敗或超時")
                    # 嘗試清除指令
                    self._write_register('CONTROL_COMMAND', CCD1Command.CLEAR)
                    if retry_count < max_retries:
                        time.sleep(2.0)
                        continue
                    return DetectionResult.DETECTION_FAILED
                
                # 🔥 4. 清除控制指令 (200=0) - 關鍵：此時結果才會出現
                self.logger.info("清除控制指令 (200=0) - 結果將在此時出現...")
                if not self._write_register('CONTROL_COMMAND', CCD1Command.CLEAR):
                    self.logger.error("清除控制指令失敗")
                    if retry_count < max_retries:
                        time.sleep(1.0)
                        continue
                    return DetectionResult.DETECTION_FAILED
                
                # 🔥 5. 等待結果穩定 (給CCD1模組時間寫入結果到240等寄存器)
                self.logger.info("等待CCD1模組寫入檢測結果...")
                time.sleep(0.5)  # 等待500ms讓結果穩定
                
                # 6. 讀取檢測結果並更新FIFO佇列
                result = self._read_and_update_fifo_queue()
                
                if result == DetectionResult.SUCCESS:
                    self.logger.info(f"拍照+檢測成功完成，佇列新增 {self.last_detection_count} 個物體")
                    return DetectionResult.SUCCESS
                elif result == DetectionResult.NO_OBJECTS:
                    self.logger.info("拍照+檢測完成，未檢測到物體 (需要補料)")
                    return DetectionResult.NO_OBJECTS
                else:
                    self.logger.error("讀取檢測結果失敗")
                    if retry_count < max_retries:
                        time.sleep(1.0)
                        continue
                    return DetectionResult.DETECTION_FAILED
                
            except Exception as e:
                self.logger.error(f"執行拍照+檢測異常: {e}")
                if retry_count < max_retries:
                    time.sleep(2.0)
                    continue
                return DetectionResult.DETECTION_FAILED
        
        # 所有重試都失敗
        self.logger.error("拍照+檢測所有重試都失敗")
        return DetectionResult.DETECTION_FAILED
    
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
    
    def _wait_for_completion(self, timeout: float = 15.0) -> bool:
        """
        等待指令執行完成 (等待201寄存器=8)
        
        執行完成判斷：Ready=0且Running=0 (狀態寄存器=8)
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 指令是否執行完成
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self._read_register('STATUS_REGISTER')
            if status is not None:
                ready = bool(status & (1 << CCD1StatusBits.READY))
                running = bool(status & (1 << CCD1StatusBits.RUNNING))
                alarm = bool(status & (1 << CCD1StatusBits.ALARM))
                
                elapsed = time.time() - start_time
                self.logger.debug(f"執行狀態 ({elapsed:.1f}s): 201={status}, Ready={ready}, Running={running}, Alarm={alarm}")
                
                if alarm:
                    self.logger.warning("CCD1系統執行中發生Alarm")
                    return False
                
                # 檢查是否執行完成 (Ready=0且Running=0, 狀態寄存器=8)
                if not ready and not running:
                    self.logger.info(f"指令執行完成 (201={status})")
                    return True
            
            time.sleep(0.2)  # 200ms檢查間隔
        
        self.logger.error(f"等待指令完成超時: {timeout}秒")
        return False
    
    def _read_and_update_fifo_queue(self) -> DetectionResult:
        """
        讀取檢測結果並更新全域FIFO佇列
        
        Returns:
            DetectionResult: 檢測結果狀態
        """
        try:
            # 檢查世界座標有效性
            world_coord_valid = self._read_register('WORLD_COORD_VALID')
            if not world_coord_valid:
                self.logger.warning("世界座標無效，可能缺少標定數據")
                return DetectionResult.DETECTION_FAILED
            
            # 讀取檢測到的圓形數量
            circle_count = self._read_register('CIRCLE_COUNT')
            if circle_count is None:
                self.logger.error("無法讀取圓形數量")
                return DetectionResult.DETECTION_FAILED
            
            self.logger.info(f"檢測結果: 圓形數量={circle_count}")
            
            if circle_count == 0:
                self.last_detection_count = 0
                self.logger.info("檢測完成，未發現物體 (需要補料)")
                return DetectionResult.NO_OBJECTS
            
            # 限制最多5個圓形
            circle_count = min(circle_count, 5)
            
            # 讀取像素座標結果 (241-255)
            pixel_registers = self._read_multiple_registers(241, 15)  # 241-255
            
            # 讀取世界座標結果 (257-276)
            world_registers = self._read_multiple_registers(257, 20)  # 257-276
            
            if not pixel_registers or not world_registers:
                self.logger.error("讀取檢測結果失敗")
                return DetectionResult.DETECTION_FAILED
            
            # 解析檢測結果並更新FIFO佇列
            new_coordinates = []
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
                
                # 創建座標對象
                coord = CircleWorldCoord(
                    id=self.total_detections + i + 1,  # 全域唯一ID
                    world_x=world_x,
                    world_y=world_y,
                    pixel_x=pixel_x,
                    pixel_y=pixel_y,
                    radius=radius,
                    timestamp=current_time
                )
                new_coordinates.append(coord)
            
            # 更新全域FIFO佇列
            with self.queue_lock:
                for coord in new_coordinates:
                    self.global_coord_queue.append(coord)
                
                self.last_detection_count = len(new_coordinates)
                self.total_detections += len(new_coordinates)
            
            self.logger.info(f"FIFO佇列已更新，新增 {len(new_coordinates)} 個座標，佇列總長度: {len(self.global_coord_queue)}")
            return DetectionResult.SUCCESS
            
        except Exception as e:
            self.logger.error(f"讀取和更新FIFO佇列失敗: {e}")
            return DetectionResult.DETECTION_FAILED
    
    def get_next_object(self) -> Optional[CircleWorldCoord]:
        """
        獲取下一個物件圓心世界座標 (核心API方法) - 修正版
        
        自動管理邏輯:
        1. 檢查全域FIFO佇列是否有物體
        2. 如果有，直接返回 (FIFO)
        3. 如果沒有，檢查是否有現有寄存器數據未收取
        4. 🔥 如果有現有數據，自動收取到FIFO佇列
        5. 如果仍沒有，檢查是否需要拍照檢測 (240寄存器=0)
        6. 如果需要，自動執行拍照+檢測
        7. 如果檢測到物體，返回第一個
        8. 如果無物體，返回None (告知需要補料)
        
        Returns:
            CircleWorldCoord: 圓心世界座標，如果無可用座標則返回None
            None: 表示無物體，需要補料
        """
        # 首先檢查FIFO佇列
        with self.queue_lock:
            if len(self.global_coord_queue) > 0:
                coord = self.global_coord_queue.popleft()  # FIFO: 從前端取出
                self.logger.info(f"從FIFO佇列獲取物體: ID={coord.id}, 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm")
                return coord
        
        # 佇列為空，檢查是否有現有數據未收取
        self.logger.info("FIFO佇列為空，檢查是否有現有寄存器數據...")
        
        # 防止多線程同時觸發操作
        with self.detection_lock:
            if self.detection_in_progress:
                self.logger.info("檢測正在進行中，等待完成...")
                # 等待檢測完成
                timeout = 20.0
                start_time = time.time()
                while self.detection_in_progress and (time.time() - start_time) < timeout:
                    time.sleep(0.5)
                
                # 檢測完成後，再次檢查佇列
                with self.queue_lock:
                    if len(self.global_coord_queue) > 0:
                        coord = self.global_coord_queue.popleft()
                        self.logger.info(f"檢測完成後獲取物體: ID={coord.id}")
                        return coord
                
                self.logger.warning("檢測完成但佇列仍為空")
                return None
            
            # 標記操作進行中
            self.detection_in_progress = True
        
        try:
            # 🔥 優先檢查是否有現有數據未收取
            circle_count = self._read_register('CIRCLE_COUNT')
            if circle_count is not None and circle_count > 0:
                self.logger.info(f"發現現有寄存器數據: {circle_count}個圓形，自動收取...")
                
                # 自動收取現有數據
                collect_result = self.auto_collect_existing_data()
                
                if collect_result == DetectionResult.SUCCESS:
                    self.logger.info("自動收取現有數據成功")
                    
                    # 從佇列獲取第一個物體
                    with self.queue_lock:
                        if len(self.global_coord_queue) > 0:
                            coord = self.global_coord_queue.popleft()
                            self.logger.info(f"收取數據後獲取物體: ID={coord.id}, 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm")
                            return coord
                        else:
                            self.logger.error("收取數據成功但佇列為空，這不應該發生")
                            return None
                else:
                    self.logger.error(f"自動收取現有數據失敗: {collect_result}")
                    # 繼續嘗試拍照檢測
            
            # 檢查是否需要拍照檢測
            if self.check_detection_needed():
                self.logger.info("需要拍照檢測，自動觸發...")
                
                # 執行拍照+檢測
                result = self.execute_capture_and_detect()
                
                if result == DetectionResult.SUCCESS:
                    self.logger.info("自動拍照檢測成功")
                    
                    # 從佇列獲取第一個物體
                    with self.queue_lock:
                        if len(self.global_coord_queue) > 0:
                            coord = self.global_coord_queue.popleft()
                            self.logger.info(f"自動檢測後獲取物體: ID={coord.id}, 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm")
                            return coord
                        else:
                            self.logger.error("檢測成功但佇列為空，這不應該發生")
                            return None
                    
                elif result == DetectionResult.NO_OBJECTS:
                    self.logger.info("自動拍照檢測完成，未發現物體 (需要補料)")
                    return None
                
                else:
                    self.logger.error(f"自動拍照檢測失敗: {result}")
                    return None
            else:
                self.logger.info("240寄存器為0，且無現有數據可收取")
                return None
        
        finally:
            # 清除操作進行中標誌
            with self.detection_lock:
                self.detection_in_progress = False
    
    def manual_capture_and_detect(self) -> DetectionResult:
        """
        手動執行拍照+檢測指令 (供外部調用)
        
        Returns:
            DetectionResult: 檢測結果狀態
        """
        self.logger.info("手動觸發拍照+檢測...")
        
        with self.detection_lock:
            if self.detection_in_progress:
                self.logger.warning("檢測正在進行中，無法手動觸發")
                return DetectionResult.DETECTION_FAILED
            
            self.detection_in_progress = True
        
        try:
            result = self.execute_capture_and_detect()
            self.logger.info(f"手動拍照+檢測完成: {result}")
            return result
        finally:
            with self.detection_lock:
                self.detection_in_progress = False
    
    def manual_collect_existing_data(self) -> DetectionResult:
        """
        🔥 新增：手動執行收取現有寄存器數據 (供外部調用)
        
        Returns:
            DetectionResult: 收取結果狀態
        """
        self.logger.info("手動觸發收取現有數據...")
        
        with self.detection_lock:
            if self.detection_in_progress:
                self.logger.warning("檢測正在進行中，無法手動觸發")
                return DetectionResult.DETECTION_FAILED
            
            self.detection_in_progress = True
        
        try:
            result = self.auto_collect_existing_data()
            self.logger.info(f"手動收取現有數據完成: {result}")
            return result
        finally:
            with self.detection_lock:
                self.detection_in_progress = False
    
    def get_queue_status(self) -> Dict[str, Any]:
        """
        獲取佇列狀態資訊
        
        Returns:
            Dict: 包含佇列長度、檢測統計等資訊
        """
        with self.queue_lock:
            queue_length = len(self.global_coord_queue)
            queue_preview = []
            
            # 獲取前3個座標的預覽
            for i, coord in enumerate(list(self.global_coord_queue)[:3]):
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
            'total_detections': self.total_detections,
            'queue_preview': queue_preview,
            'detection_in_progress': self.detection_in_progress,
            'modbus_server': f"{self.modbus_host}:{self.modbus_port}",
            'auto_detection_enabled': True,
            'auto_collection_enabled': True  # 新增：標識自動收取功能已啟用
        }
    
    def clear_queue(self):
        """清空座標佇列"""
        with self.queue_lock:
            self.global_coord_queue.clear()
            self.logger.info("FIFO座標佇列已清空")
    
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
        circle_count = self._read_register('CIRCLE_COUNT')
        
        if status is not None:
            return {
                'connected': True,
                'ready': bool(status & (1 << CCD1StatusBits.READY)),
                'running': bool(status & (1 << CCD1StatusBits.RUNNING)),
                'alarm': bool(status & (1 << CCD1StatusBits.ALARM)),
                'initialized': bool(status & (1 << CCD1StatusBits.INITIALIZED)),
                'world_coord_valid': bool(world_coord_valid) if world_coord_valid is not None else False,
                'current_circle_count': circle_count if circle_count is not None else 0,
                'detection_needed': (circle_count == 0) if circle_count is not None else True,
                'status_register_value': status
            }
        
        return {
            'connected': True,
            'ready': False,
            'running': False,
            'alarm': True,
            'initialized': False,
            'world_coord_valid': False,
            'current_circle_count': 0,
            'detection_needed': True,
            'error': '無法讀取狀態寄存器'
        }


# ==================== 使用範例 ====================
def example_usage():
    """使用範例"""
    # 創建CCD1高層API實例
    ccd1 = CCD1HighLevelAPI()
    
    try:
        print("=== CCD1修正版高層API使用範例 ===")
        
        # 檢查系統狀態
        status = ccd1.get_system_status()
        print(f"系統狀態: {status}")
        
        # 檢查佇列狀態
        queue_status = ccd1.get_queue_status()
        print(f"佇列狀態: {queue_status}")
        
        # 模擬Flow1的使用方式
        print("\n模擬Flow1連續獲取物體:")
        for i in range(5):  # 嘗試獲取5個物體
            print(f"\n--- Flow1第{i+1}次調用 ---")
            
            coord = ccd1.get_next_object()
            
            if coord:
                print(f"✓ 獲取物體成功:")
                print(f"  ID: {coord.id}")
                print(f"  世界座標: ({coord.world_x:.2f}, {coord.world_y:.2f})mm")
                print(f"  像素座標: ({coord.pixel_x}, {coord.pixel_y})")
                print(f"  半徑: {coord.radius}")
                print(f"  時間戳: {coord.timestamp}")
                
                # Flow1可以設定R值
                coord.r = 45.0  # 例如從VP_TOPSIDE繼承R值
                print(f"  R值 (Flow1設定): {coord.r}°")
            else:
                print("✗ 無可用物體，需要補料")
                print("  Flow1應該執行補料流程")
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