# -*- coding: utf-8 -*-
"""
CCD1HighLevel.py - CCD1高層API模組 (修正版 - 寄存器地址一致性修正)
修正問題：
1. 世界座標寄存器地址映射錯誤 (原本使用241-255作為像素座標，應該使用241-255像素座標，257-276世界座標)
2. 世界座標解析順序錯誤 (高位低位合併方式)
3. 與CCD1VisionCode_Enhanced.py的寄存器映射完全一致
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
    CCD1高層API - 簡化CCD1功能使用 (修正版)
    
    修正內容:
    1. 與CCD1VisionCode_Enhanced.py寄存器地址完全一致
    2. 正確的世界座標解析邏輯
    3. 修正32位合併順序和負數處理
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
        
        # 🔥 修正：CCD1寄存器映射 - 與CCD1VisionCode_Enhanced.py完全一致
        self.REGISTERS = {
            # 核心控制寄存器
            'CONTROL_COMMAND': 200,        # 控制指令
            'STATUS_REGISTER': 201,        # 狀態寄存器
            
            # 檢測結果寄存器 (240-255) - 像素座標區域
            'CIRCLE_COUNT': 240,           # 檢測圓形數量
            # 圓形像素座標 (每個圓形3個寄存器: X, Y, Radius)
            'CIRCLE_1_X': 241, 'CIRCLE_1_Y': 242, 'CIRCLE_1_RADIUS': 243,
            'CIRCLE_2_X': 244, 'CIRCLE_2_Y': 245, 'CIRCLE_2_RADIUS': 246,
            'CIRCLE_3_X': 247, 'CIRCLE_3_Y': 248, 'CIRCLE_3_RADIUS': 249,
            'CIRCLE_4_X': 250, 'CIRCLE_4_Y': 251, 'CIRCLE_4_RADIUS': 252,
            'CIRCLE_5_X': 253, 'CIRCLE_5_Y': 254, 'CIRCLE_5_RADIUS': 255,
            
            # 🔥 修正：世界座標寄存器 (256-276) - 與CCD1VisionCode_Enhanced.py一致
            'WORLD_COORD_VALID': 256,      # 世界座標有效標誌
            # 圓形世界座標 (每個圓形4個寄存器: X高位, X低位, Y高位, Y低位)
            'CIRCLE_1_WORLD_X_HIGH': 257, 'CIRCLE_1_WORLD_X_LOW': 258,
            'CIRCLE_1_WORLD_Y_HIGH': 259, 'CIRCLE_1_WORLD_Y_LOW': 260,
            'CIRCLE_2_WORLD_X_HIGH': 261, 'CIRCLE_2_WORLD_X_LOW': 262,
            'CIRCLE_2_WORLD_Y_HIGH': 263, 'CIRCLE_2_WORLD_Y_LOW': 264,
            'CIRCLE_3_WORLD_X_HIGH': 265, 'CIRCLE_3_WORLD_X_LOW': 266,
            'CIRCLE_3_WORLD_Y_HIGH': 267, 'CIRCLE_3_WORLD_Y_LOW': 268,
            'CIRCLE_4_WORLD_X_HIGH': 269, 'CIRCLE_4_WORLD_X_LOW': 270,
            'CIRCLE_4_WORLD_Y_HIGH': 271, 'CIRCLE_4_WORLD_Y_LOW': 272,
            'CIRCLE_5_WORLD_X_HIGH': 273, 'CIRCLE_5_WORLD_X_LOW': 274,
            'CIRCLE_5_WORLD_Y_HIGH': 275, 'CIRCLE_5_WORLD_Y_LOW': 276,
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
        🔥 最終修正：讀取世界座標檢測結果 - 匹配CCD1寫入邏輯
        
        CCD1寫入邏輯：
        world_x_high = (world_x_int >> 16) & 0xFFFF  # 無符號16位
        world_x_low = world_x_int & 0xFFFF           # 無符號16位
        
        但pymodbus讀取時可能解釋為有符號16位，需要轉換回無符號
        
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
        
        coordinates = []
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        
        for i in range(circle_count):
            try:
                # 讀取像素座標
                pixel_x = self._read_register(f'CIRCLE_{i+1}_X')
                pixel_y = self._read_register(f'CIRCLE_{i+1}_Y')
                radius = self._read_register(f'CIRCLE_{i+1}_RADIUS')
                
                if pixel_x is None or pixel_y is None or radius is None:
                    self.logger.warning(f"圓形{i+1}像素座標讀取失敗")
                    continue
                
                # 讀取世界座標寄存器原始值
                world_x_high_raw = self._read_register(f'CIRCLE_{i+1}_WORLD_X_HIGH')
                world_x_low_raw = self._read_register(f'CIRCLE_{i+1}_WORLD_X_LOW')
                world_y_high_raw = self._read_register(f'CIRCLE_{i+1}_WORLD_Y_HIGH')
                world_y_low_raw = self._read_register(f'CIRCLE_{i+1}_WORLD_Y_LOW')
                
                if (world_x_high_raw is None or world_x_low_raw is None or 
                    world_y_high_raw is None or world_y_low_raw is None):
                    self.logger.warning(f"圓形{i+1}世界座標讀取失敗")
                    continue
                
                # 🔥 關鍵修正：將pymodbus的有符號16位讀取轉換為無符號16位
                # 這樣才能正確對應CCD1的寫入邏輯
                def to_unsigned_16bit(signed_value):
                    """將有符號16位轉換為無符號16位"""
                    if signed_value < 0:
                        return signed_value + 65536
                    return signed_value
                
                world_x_high = to_unsigned_16bit(world_x_high_raw)
                world_x_low = to_unsigned_16bit(world_x_low_raw)
                world_y_high = to_unsigned_16bit(world_y_high_raw)
                world_y_low = to_unsigned_16bit(world_y_low_raw)
                
                # 🔥 修正：按照CCD1的邏輯重建32位值
                # 還原CCD1的寫入邏輯：(world_x_int >> 16) & 0xFFFF 和 world_x_int & 0xFFFF
                world_x_int = (world_x_high << 16) | world_x_low
                world_y_int = (world_y_high << 16) | world_y_low
                
                # 🔥 修正：處理32位有符號整數範圍
                # 如果值超過32位有符號整數範圍，轉換為負數
                if world_x_int > 2147483647:  # 2^31 - 1
                    world_x_int = world_x_int - 4294967296  # 2^32
                if world_y_int > 2147483647:
                    world_y_int = world_y_int - 4294967296
                
                # 恢復精度 (÷100)
                world_x = world_x_int / 100.0
                world_y = world_y_int / 100.0
                
                # 記錄詳細調試信息
                self.logger.info(f"圓形{i+1}座標解析詳細:")
                self.logger.info(f"  原始讀取(有符號): X_HIGH={world_x_high_raw}, X_LOW={world_x_low_raw}, Y_HIGH={world_y_high_raw}, Y_LOW={world_y_low_raw}")
                self.logger.info(f"  轉換(無符號): X_HIGH={world_x_high}, X_LOW={world_x_low}, Y_HIGH={world_y_high}, Y_LOW={world_y_low}")
                self.logger.info(f"  32位重建: X_INT={world_x_int}, Y_INT={world_y_int}")
                self.logger.info(f"  最終座標: X={world_x:.2f}mm, Y={world_y:.2f}mm")
                
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
                
            except Exception as e:
                self.logger.error(f"圓形{i+1}座標解析失敗: {e}")
                continue
        
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
    
    def debug_register_values(self) -> Dict[str, Any]:
        """
        🔥 新增：調試寄存器數值 - 驗證讀取是否正確
        
        Returns:
            Dict: 所有相關寄存器的當前數值
        """
        if not self.connected:
            return {'error': 'Modbus未連接'}
        
        debug_info = {}
        
        try:
            # 基本狀態寄存器
            debug_info['control_command'] = self._read_register('CONTROL_COMMAND')
            debug_info['status_register'] = self._read_register('STATUS_REGISTER')
            debug_info['circle_count'] = self._read_register('CIRCLE_COUNT')
            debug_info['world_coord_valid'] = self._read_register('WORLD_COORD_VALID')
            
            # 第一個圓形的詳細資料
            circle_1_raw = {
                'pixel_x': self._read_register('CIRCLE_1_X'),
                'pixel_y': self._read_register('CIRCLE_1_Y'),
                'radius': self._read_register('CIRCLE_1_RADIUS'),
                'world_x_high_raw': self._read_register('CIRCLE_1_WORLD_X_HIGH'),
                'world_x_low_raw': self._read_register('CIRCLE_1_WORLD_X_LOW'),
                'world_y_high_raw': self._read_register('CIRCLE_1_WORLD_Y_HIGH'),
                'world_y_low_raw': self._read_register('CIRCLE_1_WORLD_Y_LOW'),
            }
            
            debug_info['circle_1_raw'] = circle_1_raw
            
            # 處理有符號16位轉換
            def signed_16bit(value):
                if value is None:
                    return None
                if value > 32767:
                    return value - 65536
                return value
            
            circle_1_signed = {
                'world_x_high_signed': signed_16bit(circle_1_raw['world_x_high_raw']),
                'world_x_low_signed': signed_16bit(circle_1_raw['world_x_low_raw']),
                'world_y_high_signed': signed_16bit(circle_1_raw['world_y_high_raw']),
                'world_y_low_signed': signed_16bit(circle_1_raw['world_y_low_raw']),
            }
            
            debug_info['circle_1_signed'] = circle_1_signed
            
            # 計算合併後的世界座標
            if (circle_1_signed['world_x_high_signed'] is not None and 
                circle_1_signed['world_x_low_signed'] is not None):
                
                # 修正的32位合併邏輯
                world_x_int = (circle_1_signed['world_x_high_signed'] << 16) + (circle_1_signed['world_x_low_signed'] & 0xFFFF)
                if world_x_int > 2147483647:
                    world_x_int -= 4294967296
                    
                debug_info['world_x_calculation'] = {
                    'high_shifted': circle_1_signed['world_x_high_signed'] << 16,
                    'low_masked': circle_1_signed['world_x_low_signed'] & 0xFFFF,
                    'combined_32bit': world_x_int,
                    'final_mm': world_x_int / 100.0
                }
            
            if (circle_1_signed['world_y_high_signed'] is not None and 
                circle_1_signed['world_y_low_signed'] is not None):
                
                world_y_int = (circle_1_signed['world_y_high_signed'] << 16) + (circle_1_signed['world_y_low_signed'] & 0xFFFF)
                if world_y_int > 2147483647:
                    world_y_int -= 4294967296
                    
                debug_info['world_y_calculation'] = {
                    'high_shifted': circle_1_signed['world_y_high_signed'] << 16,
                    'low_masked': circle_1_signed['world_y_low_signed'] & 0xFFFF,
                    'combined_32bit': world_y_int,
                    'final_mm': world_y_int / 100.0
                }
            
            # 寄存器地址映射確認
            debug_info['register_addresses'] = {
                'CIRCLE_1_X': self.REGISTERS['CIRCLE_1_X'],
                'CIRCLE_1_Y': self.REGISTERS['CIRCLE_1_Y'],
                'CIRCLE_1_RADIUS': self.REGISTERS['CIRCLE_1_RADIUS'],
                'CIRCLE_1_WORLD_X_HIGH': self.REGISTERS['CIRCLE_1_WORLD_X_HIGH'],
                'CIRCLE_1_WORLD_X_LOW': self.REGISTERS['CIRCLE_1_WORLD_X_LOW'],
                'CIRCLE_1_WORLD_Y_HIGH': self.REGISTERS['CIRCLE_1_WORLD_Y_HIGH'],
                'CIRCLE_1_WORLD_Y_LOW': self.REGISTERS['CIRCLE_1_WORLD_Y_LOW'],
            }
            
            # 根據ModbusPoll實測數據驗證
            debug_info['modbus_poll_validation'] = {
                'expected_x_high': -9242,
                'expected_x_low': -26375,
                'expected_y_high': 29945,
                'expected_y_low': 0,
                'matches_x_high': circle_1_signed['world_x_high_signed'] == -9242,
                'matches_x_low': circle_1_signed['world_x_low_signed'] == -26375,
                'matches_y_high': circle_1_signed['world_y_high_signed'] == 29945,
                'matches_y_low': circle_1_signed['world_y_low_signed'] == 0,
            }
            
            return debug_info
            
        except Exception as e:
            return {'error': f'調試讀取失敗: {str(e)}'}
    
    def manual_calculate_world_coords(self, x_high: int, x_low: int, y_high: int, y_low: int) -> Dict[str, Any]:
        """
        🔥 新增：手動計算世界座標 - 用於驗證計算邏輯 (最終修正版)
        
        Args:
            x_high, x_low, y_high, y_low: ModbusPoll中看到的實際數值 (有符號16位讀取)
            
        Returns:
            Dict: 計算過程和結果
        """
        try:
            # 🔥 關鍵：將pymodbus的有符號16位讀取轉換為無符號16位
            # 這樣才能匹配CCD1的寫入邏輯
            def to_unsigned_16bit(signed_value):
                if signed_value < 0:
                    return signed_value + 65536
                return signed_value
            
            x_high_unsigned = to_unsigned_16bit(x_high)
            x_low_unsigned = to_unsigned_16bit(x_low)
            y_high_unsigned = to_unsigned_16bit(y_high)
            y_low_unsigned = to_unsigned_16bit(y_low)
            
            # 按照CCD1邏輯重建32位值
            world_x_int = (x_high_unsigned << 16) | x_low_unsigned
            world_y_int = (y_high_unsigned << 16) | y_low_unsigned
            
            # 處理32位有符號整數範圍
            if world_x_int > 2147483647:
                world_x_int = world_x_int - 4294967296
            if world_y_int > 2147483647:
                world_y_int = world_y_int - 4294967296
            
            # 恢復精度
            world_x_mm = world_x_int / 100.0
            world_y_mm = world_y_int / 100.0
            
            return {
                'input_signed': {'x_high': x_high, 'x_low': x_low, 'y_high': y_high, 'y_low': y_low},
                'converted_unsigned': {'x_high': x_high_unsigned, 'x_low': x_low_unsigned, 'y_high': y_high_unsigned, 'y_low': y_low_unsigned},
                'combined_32bit': {'x_int': world_x_int, 'y_int': world_y_int},
                'final_result': {'x_mm': world_x_mm, 'y_mm': world_y_mm},
                'verification': {
                    'modbus_poll_input': f"X_HIGH={x_high}, X_LOW={x_low}, Y_HIGH={y_high}, Y_LOW={y_low}",
                    'expected_result': f"X={world_x_mm:.2f}mm, Y={world_y_mm:.2f}mm"
                }
            }
            
        except Exception as e:
            return {'error': f'手動計算失敗: {str(e)}'}
    
    def test_modbus_poll_data(self) -> Dict[str, Any]:
        """
        🔥 新增：使用ModbusPoll實測數據進行測試
        
        測試數據：X_HIGH=-9242, X_LOW=-26375, Y_HIGH=29945, Y_LOW=0
        """
        return self.manual_calculate_world_coords(-9242, -26375, 29945, 0)


# -*- coding: utf-8 -*-
"""
CCD1HighLevel.py - CCD1高層API模組 (狀態機交握修正版)
修正內容：
1. 正確的狀態機交握協議 (201初始=9, 完成=8, 清空後回9)
2. 檢測完成後自動清空240數量寄存器
3. 嚴格的狀態驗證和超時控制
4. 與CCD1VisionCode_Enhanced.py的寄存器映射完全一致
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
    CCD1高層API - 正確狀態機交握版
    
    修正內容:
    1. 正確的CCD1狀態機交握協議
    2. 自動清空檢測數量寄存器
    3. 嚴格的狀態驗證機制
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
        
        # CCD1寄存器映射 - 與CCD1VisionCode_Enhanced.py完全一致
        self.REGISTERS = {
            # 核心控制寄存器
            'CONTROL_COMMAND': 200,        # 控制指令
            'STATUS_REGISTER': 201,        # 狀態寄存器
            
            # 檢測結果寄存器 (240-255) - 像素座標區域
            'CIRCLE_COUNT': 240,           # 檢測圓形數量
            # 圓形像素座標 (每個圓形3個寄存器: X, Y, Radius)
            'CIRCLE_1_X': 241, 'CIRCLE_1_Y': 242, 'CIRCLE_1_RADIUS': 243,
            'CIRCLE_2_X': 244, 'CIRCLE_2_Y': 245, 'CIRCLE_2_RADIUS': 246,
            'CIRCLE_3_X': 247, 'CIRCLE_3_Y': 248, 'CIRCLE_3_RADIUS': 249,
            'CIRCLE_4_X': 250, 'CIRCLE_4_Y': 251, 'CIRCLE_4_RADIUS': 252,
            'CIRCLE_5_X': 253, 'CIRCLE_5_Y': 254, 'CIRCLE_5_RADIUS': 255,
            
            # 世界座標寄存器 (256-276)
            'WORLD_COORD_VALID': 256,      # 世界座標有效標誌
            # 圓形世界座標 (每個圓形4個寄存器: X高位, X低位, Y高位, Y低位)
            'CIRCLE_1_WORLD_X_HIGH': 257, 'CIRCLE_1_WORLD_X_LOW': 258,
            'CIRCLE_1_WORLD_Y_HIGH': 259, 'CIRCLE_1_WORLD_Y_LOW': 260,
            'CIRCLE_2_WORLD_X_HIGH': 261, 'CIRCLE_2_WORLD_X_LOW': 262,
            'CIRCLE_2_WORLD_Y_HIGH': 263, 'CIRCLE_2_WORLD_Y_LOW': 264,
            'CIRCLE_3_WORLD_X_HIGH': 265, 'CIRCLE_3_WORLD_X_LOW': 266,
            'CIRCLE_3_WORLD_Y_HIGH': 267, 'CIRCLE_3_WORLD_Y_LOW': 268,
            'CIRCLE_4_WORLD_X_HIGH': 269, 'CIRCLE_4_WORLD_X_LOW': 270,
            'CIRCLE_4_WORLD_Y_HIGH': 271, 'CIRCLE_4_WORLD_Y_LOW': 272,
            'CIRCLE_5_WORLD_X_HIGH': 273, 'CIRCLE_5_WORLD_X_LOW': 274,
            'CIRCLE_5_WORLD_Y_HIGH': 275, 'CIRCLE_5_WORLD_Y_LOW': 276,
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
    
    def _wait_for_ready_status_9(self, timeout: float = 10.0) -> bool:
        """
        等待CCD1系統Ready狀態 (狀態值=9)
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 是否Ready (狀態值=9)
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self._read_register('STATUS_REGISTER')
            if status is not None:
                if status & 4:  # bit2 = Alarm
                    self.logger.warning("CCD1系統處於Alarm狀態")
                    return False
                
                if status == 9:  # Ready=1, Initialized=1
                    return True
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待Ready狀態(9)超時: {timeout}秒")
        return False
    
    def _wait_for_detection_complete(self, timeout: float = 10.0) -> bool:
        """
        等待檢測完成 (狀態值=8且檢測數量>0)
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 檢測是否完成且有結果
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self._read_register('STATUS_REGISTER')
            count = self._read_register('CIRCLE_COUNT')
            
            if status is not None and count is not None:
                if status & 4:  # bit2 = Alarm
                    self.logger.warning("CCD1檢測過程發生Alarm")
                    return False
                
                # 檢測完成條件: 狀態值=8且檢測到物件
                if status == 8 and count > 0:
                    self.logger.info(f"檢測完成: 狀態={status}, 檢測數量={count}")
                    return True
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待檢測完成超時: {timeout}秒")
        return False
    
    def _wait_for_status_recovery(self, timeout: float = 3.0) -> bool:
        """
        等待狀態恢復到9
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 狀態是否恢復到9
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self._read_register('STATUS_REGISTER')
            if status == 9:
                self.logger.info("狀態已恢復到9")
                return True
            
            time.sleep(0.1)
        
        self.logger.warning("狀態恢復到9超時")
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
        
        coordinates = []
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        
        for i in range(circle_count):
            try:
                # 讀取像素座標
                pixel_x = self._read_register(f'CIRCLE_{i+1}_X')
                pixel_y = self._read_register(f'CIRCLE_{i+1}_Y')
                radius = self._read_register(f'CIRCLE_{i+1}_RADIUS')
                
                if pixel_x is None or pixel_y is None or radius is None:
                    self.logger.warning(f"圓形{i+1}像素座標讀取失敗")
                    continue
                
                # 讀取世界座標寄存器原始值
                world_x_high_raw = self._read_register(f'CIRCLE_{i+1}_WORLD_X_HIGH')
                world_x_low_raw = self._read_register(f'CIRCLE_{i+1}_WORLD_X_LOW')
                world_y_high_raw = self._read_register(f'CIRCLE_{i+1}_WORLD_Y_HIGH')
                world_y_low_raw = self._read_register(f'CIRCLE_{i+1}_WORLD_Y_LOW')
                
                if (world_x_high_raw is None or world_x_low_raw is None or 
                    world_y_high_raw is None or world_y_low_raw is None):
                    self.logger.warning(f"圓形{i+1}世界座標讀取失敗")
                    continue
                
                # 將pymodbus的有符號16位讀取轉換為無符號16位
                def to_unsigned_16bit(signed_value):
                    if signed_value < 0:
                        return signed_value + 65536
                    return signed_value
                
                world_x_high = to_unsigned_16bit(world_x_high_raw)
                world_x_low = to_unsigned_16bit(world_x_low_raw)
                world_y_high = to_unsigned_16bit(world_y_high_raw)
                world_y_low = to_unsigned_16bit(world_y_low_raw)
                
                # 按照CCD1的邏輯重建32位值
                world_x_int = (world_x_high << 16) | world_x_low
                world_y_int = (world_y_high << 16) | world_y_low
                
                # 處理32位有符號整數範圍
                if world_x_int > 2147483647:
                    world_x_int = world_x_int - 4294967296
                if world_y_int > 2147483647:
                    world_y_int = world_y_int - 4294967296
                
                # 恢復精度 (÷100)
                world_x = world_x_int / 100.0
                world_y = world_y_int / 100.0
                
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
                
            except Exception as e:
                self.logger.error(f"圓形{i+1}座標解析失敗: {e}")
                continue
        
        return coordinates
    
    def capture_and_detect(self) -> bool:
        """
        執行拍照+檢測指令 - 修正的狀態機交握版
        
        正確的交握流程：
        1. 檢查狀態=9 (Ready且Initialized)
        2. 發送指令16到200
        3. 等待狀態=8且240>0 (檢測完成)
        4. 讀取檢測結果
        5. 清空200 (寫入0)
        6. 等待狀態回到9
        7. 清空240 (寫入0)
        
        Returns:
            bool: 操作是否成功
        """
        if not self.connected:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            # 步驟1: 等待Ready狀態 (狀態值=9)
            self.logger.info("步驟1: 等待CCD1系統Ready狀態 (期望狀態=9)...")
            if not self._wait_for_ready_status_9(self.operation_timeout):
                self.logger.error("系統未Ready，無法執行檢測")
                return False
            
            # 步驟2: 發送拍照+檢測指令
            self.logger.info("步驟2: 發送拍照+檢測指令16到200...")
            if not self._write_register('CONTROL_COMMAND', CCD1Command.CAPTURE_DETECT):
                self.logger.error("發送檢測指令失敗")
                return False
            
            # 步驟3: 等待檢測完成 (狀態=8且240>0)
            self.logger.info("步驟3: 等待檢測完成 (期望狀態=8且240>0)...")
            if not self._wait_for_detection_complete(self.operation_timeout):
                self.logger.error("檢測指令執行失敗或超時")
                return False
            
            # 步驟4: 讀取檢測結果
            self.logger.info("步驟4: 讀取檢測結果...")
            coordinates = self._read_world_coordinates()
            detection_count = len(coordinates)
            
            # 步驟5: 更新FIFO佇列
            with self.queue_lock:
                for coord in coordinates:
                    self.coord_queue.append(coord)
                self.last_detection_count = detection_count
            
            # 步驟6: 清空控制指令 (200=0)
            self.logger.info("步驟6: 清空控制指令 (200=0)...")
            if not self._write_register('CONTROL_COMMAND', CCD1Command.CLEAR):
                self.logger.warning("清空控制指令失敗")
            
            # 步驟7: 等待狀態恢復到9
            self.logger.info("步驟7: 等待狀態恢復到9...")
            self._wait_for_status_recovery(3.0)
            
            # 步驟8: 清空檢測數量 (240=0)
            self.logger.info("步驟8: 清空檢測數量 (240=0)...")
            if not self._write_register('CIRCLE_COUNT', 0):
                self.logger.warning("清空檢測數量失敗")
            
            self.logger.info(f"檢測完成，新增 {detection_count} 個圓心座標到佇列")
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
        檢查CCD1系統是否Ready (狀態值=9)
        
        Returns:
            bool: 系統是否Ready
        """
        if not self.connected:
            return False
        
        status = self._read_register('STATUS_REGISTER')
        return status == 9
    
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
                'world_coord_valid': False,
                'status_register_value': 0
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
                'status_register_value': status,
                'is_status_9': status == 9
            }
        
        return {
            'connected': True,
            'ready': False,
            'running': False,
            'alarm': True,
            'initialized': False,
            'world_coord_valid': False,
            'status_register_value': 0,
            'is_status_9': False,
            'error': '無法讀取狀態寄存器'
        }


# 便利函數，供快速調用
def test_ccd1_detection(host: str = "127.0.0.1", port: int = 502) -> bool:
    """便利函數：測試CCD1檢測功能
    
    Args:
        host: Modbus服務器IP
        port: Modbus服務器端口
        
    Returns:
        bool: 檢測是否成功
    """
    ccd1_api = CCD1HighLevelAPI(host, port)
    
    if not ccd1_api.connected:
        print("CCD1 API連接失敗")
        return False
    
    try:
        success = ccd1_api.capture_and_detect()
        
        if success:
            status = ccd1_api.get_queue_status()
            print(f"檢測成功，檢測到 {status['last_detection_count']} 個物件")
            return True
        else:
            print("檢測失敗")
            return False
    finally:
        ccd1_api.disconnect()