# -*- coding: utf-8 -*-
"""
CCD1HighLevel.py - CCD1高層API模組 (YOLO版本適配)
修正內容：
1. 適配YOLOv11版本的寄存器映射 (基於paste-2.txt)
2. 支援CG_F/CG_B/STACK三種檢測分類
3. 世界座標寄存器地址從256→260調整
4. 檢測結果寄存器重新定義為YOLO格式
5. 新增模型管理和完成標誌寄存器
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


# ==================== YOLO檢測結果數據結構 ====================
@dataclass
class YOLODetectionCoord:
    """YOLO檢測座標數據 - 支援CG_F類型"""
    id: int                    # 物件ID
    object_type: str          # 物件類型 (CG_F, CG_B, STACK)
    world_x: float            # 世界座標X (mm)
    world_y: float            # 世界座標Y (mm)
    pixel_x: int              # 像素座標X
    pixel_y: int              # 像素座標Y
    confidence: float         # 檢測置信度
    timestamp: str            # 檢測時間戳
    model_id: int             # 使用的模型ID


# ==================== CCD1高層API類 - YOLO版本 ====================
class CCD1HighLevelAPI:
    """
    CCD1高層API - YOLO版本適配
    
    YOLO版本特性:
    1. 支援YOLOv11三種檢測類別 (CG_F, CG_B, STACK)
    2. 新的寄存器映射 (240=CG_F數量, 260=世界座標有效)
    3. 完成標誌寄存器 (203-206)
    4. 模型管理寄存器 (202)
    5. 僅返回CG_F類型的世界座標 (符合Flow1需求)
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化CCD1高層API - YOLO版本
        
        Args:
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 🔥 YOLO版本寄存器映射 - 基於paste-2.txt
        self.REGISTERS = {
            # 核心控制寄存器 (200-201)
            'CONTROL_COMMAND': 200,        # 控制指令
            'STATUS_REGISTER': 201,        # 狀態寄存器
            'MODEL_SELECT': 202,           # 模型選擇 (0=未指定, 1-20=模型ID)
            
            # 完成標誌寄存器 (203-206) - YOLO版本新增
            'CAPTURE_COMPLETE': 203,       # 拍照完成標誌
            'DETECT_COMPLETE': 204,        # 檢測完成標誌
            'OPERATION_SUCCESS': 205,      # 操作成功標誌
            'ERROR_CODE': 206,             # 錯誤代碼
            
            # YOLOv11檢測參數寄存器 (210-219)
            'CONFIDENCE_HIGH': 210,        # 置信度閾值高位
            'CONFIDENCE_LOW': 211,         # 置信度閾值低位
            
            # 🔥 YOLO版本檢測結果寄存器 (240-259)
            'CG_F_COUNT': 240,             # CG_F檢測數量
            'CG_B_COUNT': 241,             # CG_B檢測數量
            'STACK_COUNT': 242,            # STACK檢測數量
            'TOTAL_DETECTIONS': 243,       # 總檢測數量
            'DETECTION_SUCCESS': 244,      # 檢測成功標誌
            
            # CG_F座標寄存器 (245-254) - 最多5個
            'CG_F_1_X': 245, 'CG_F_1_Y': 246,
            'CG_F_2_X': 247, 'CG_F_2_Y': 248,
            'CG_F_3_X': 249, 'CG_F_3_Y': 250,
            'CG_F_4_X': 251, 'CG_F_4_Y': 252,
            'CG_F_5_X': 253, 'CG_F_5_Y': 254,
            
            # 擴展檢測結果 (255-259)
            'CG_B_1_X': 255, 'CG_B_1_Y': 256,
            'STACK_1_X': 257, 'STACK_1_Y': 258,
            'MODEL_ID_USED': 259,           # 本次檢測使用的模型ID
            
            # 🔥 YOLO版本世界座標寄存器 (260-280) - 地址移位
            'WORLD_COORD_VALID': 260,       # 世界座標有效標誌 (原256→260)
            
            # CG_F世界座標 (261-280) - 每個CG_F 4個寄存器
            'CG_F_1_WORLD_X_HIGH': 261, 'CG_F_1_WORLD_X_LOW': 262,
            'CG_F_1_WORLD_Y_HIGH': 263, 'CG_F_1_WORLD_Y_LOW': 264,
            'CG_F_2_WORLD_X_HIGH': 265, 'CG_F_2_WORLD_X_LOW': 266,
            'CG_F_2_WORLD_Y_HIGH': 267, 'CG_F_2_WORLD_Y_LOW': 268,
            'CG_F_3_WORLD_X_HIGH': 269, 'CG_F_3_WORLD_X_LOW': 270,
            'CG_F_3_WORLD_Y_HIGH': 271, 'CG_F_3_WORLD_Y_LOW': 272,
            'CG_F_4_WORLD_X_HIGH': 273, 'CG_F_4_WORLD_X_LOW': 274,
            'CG_F_4_WORLD_Y_HIGH': 275, 'CG_F_4_WORLD_Y_LOW': 276,
            'CG_F_5_WORLD_X_HIGH': 277, 'CG_F_5_WORLD_X_LOW': 278,
            'CG_F_5_WORLD_Y_HIGH': 279, 'CG_F_5_WORLD_Y_LOW': 280,
            
            # 統計資訊寄存器 (281-299)
            'LAST_CAPTURE_TIME': 281,      # 最後拍照耗時
            'LAST_PROCESS_TIME': 282,      # 最後處理耗時
            'LAST_TOTAL_TIME': 283,        # 最後總耗時
            'OPERATION_COUNT': 284,        # 操作計數器
            'ERROR_COUNT': 285,            # 錯誤計數器
            'CONNECTION_COUNT': 286,       # 連接計數器
            'MODEL_SWITCH_COUNT': 287,     # 模型切換次數
        }
        
        # CG_F座標FIFO佇列 (僅處理CG_F類型)
        self.coord_queue = deque()  # CG_F座標佇列
        self.queue_lock = threading.Lock()  # 佇列操作鎖
        
        # 狀態追蹤
        self.last_detection_count = 0
        self.last_cg_f_count = 0
        self.operation_timeout = 10.0  # 操作超時時間(秒)
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1HighLevel_YOLO")
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
    
    def _wait_for_ready_status(self, timeout: float = 10.0) -> bool:
        """
        等待CCD1系統Ready狀態 (YOLO版本)
        
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
                alarm = bool(status & (1 << CCD1StatusBits.ALARM))
                running = bool(status & (1 << CCD1StatusBits.RUNNING))
                
                if alarm:
                    self.logger.warning("CCD1系統處於Alarm狀態")
                    return False
                
                if ready and not running:
                    return True
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待Ready狀態超時: {timeout}秒")
        return False
    
    def _wait_for_detection_complete_yolo(self, timeout: float = 10.0) -> bool:
        """
        等待YOLO檢測完成 (檢查完成標誌寄存器)
        
        Args:
            timeout: 超時時間(秒)
            
        Returns:
            bool: 檢測是否完成
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 檢查完成標誌
            detect_complete = self._read_register('DETECT_COMPLETE')
            operation_success = self._read_register('OPERATION_SUCCESS')
            error_code = self._read_register('ERROR_CODE')
            
            if detect_complete == 1:
                if operation_success == 1:
                    self.logger.info("YOLO檢測完成且成功")
                    return True
                else:
                    error = error_code or 0
                    self.logger.warning(f"YOLO檢測完成但失敗，錯誤代碼: {error}")
                    return False
            
            time.sleep(0.1)  # 100ms檢查間隔
        
        self.logger.error(f"等待YOLO檢測完成超時: {timeout}秒")
        return False
    
    def _read_yolo_world_coordinates(self) -> List[YOLODetectionCoord]:
        """
        讀取YOLO版本世界座標檢測結果 - 僅處理CG_F類型
        
        Returns:
            List[YOLODetectionCoord]: CG_F世界座標列表
        """
        # 檢查世界座標有效性 (YOLO版本地址260)
        world_coord_valid = self._read_register('WORLD_COORD_VALID')
        if not world_coord_valid:
            self.logger.warning("世界座標無效，可能缺少標定數據")
            return []
        
        # 讀取CG_F檢測數量 (YOLO版本地址240)
        cg_f_count = self._read_register('CG_F_COUNT')
        if not cg_f_count or cg_f_count == 0:
            self.logger.info("未檢測到CG_F物件")
            return []
        
        # 讀取使用的模型ID
        model_id = self._read_register('MODEL_ID_USED') or 0
        
        # 限制最多5個CG_F
        cg_f_count = min(cg_f_count, 5)
        
        coordinates = []
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        
        for i in range(cg_f_count):
            try:
                # 讀取CG_F像素座標 (YOLO版本地址245-254)
                pixel_x = self._read_register(f'CG_F_{i+1}_X')
                pixel_y = self._read_register(f'CG_F_{i+1}_Y')
                
                if pixel_x is None or pixel_y is None:
                    self.logger.warning(f"CG_F {i+1}像素座標讀取失敗")
                    continue
                
                # 讀取CG_F世界座標 (YOLO版本地址261-280)
                world_x_high_raw = self._read_register(f'CG_F_{i+1}_WORLD_X_HIGH')
                world_x_low_raw = self._read_register(f'CG_F_{i+1}_WORLD_X_LOW')
                world_y_high_raw = self._read_register(f'CG_F_{i+1}_WORLD_Y_HIGH')
                world_y_low_raw = self._read_register(f'CG_F_{i+1}_WORLD_Y_LOW')
                
                if (world_x_high_raw is None or world_x_low_raw is None or 
                    world_y_high_raw is None or world_y_low_raw is None):
                    self.logger.warning(f"CG_F {i+1}世界座標讀取失敗")
                    continue
                
                # 轉換為無符號16位
                def to_unsigned_16bit(signed_value):
                    if signed_value < 0:
                        return signed_value + 65536
                    return signed_value
                
                world_x_high = to_unsigned_16bit(world_x_high_raw)
                world_x_low = to_unsigned_16bit(world_x_low_raw)
                world_y_high = to_unsigned_16bit(world_y_high_raw)
                world_y_low = to_unsigned_16bit(world_y_low_raw)
                
                # 重建32位值
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
                
                coord = YOLODetectionCoord(
                    id=i + 1,
                    object_type="CG_F",
                    world_x=world_x,
                    world_y=world_y,
                    pixel_x=pixel_x,
                    pixel_y=pixel_y,
                    confidence=0.8,  # YOLO置信度，可從其他寄存器讀取
                    timestamp=current_time,
                    model_id=model_id
                )
                coordinates.append(coord)
                
                self.logger.info(f"CG_F {i+1}: 像素({pixel_x}, {pixel_y}) → 世界({world_x:.2f}, {world_y:.2f})mm")
                
            except Exception as e:
                self.logger.error(f"CG_F {i+1}座標解析失敗: {e}")
                continue
        
        return coordinates
    
    def capture_and_detect(self) -> bool:
        """
        執行拍照+檢測指令 - YOLO版本
        
        YOLO版本交握流程：
        1. 檢查Ready狀態
        2. 發送指令16到200
        3. 等待完成標誌 (204=1且205=1)
        4. 讀取YOLO檢測結果
        5. 清空控制指令 (200=0)
        6. 清空完成標誌
        
        Returns:
            bool: 操作是否成功
        """
        if not self.connected:
            self.logger.error("Modbus未連接")
            return False
        
        try:
            # 步驟1: 等待Ready狀態
            self.logger.info("步驟1: 等待CCD1系統Ready狀態...")
            if not self._wait_for_ready_status(self.operation_timeout):
                self.logger.error("系統未Ready，無法執行檢測")
                return False
            
            # 步驟2: 發送拍照+檢測指令
            self.logger.info("步驟2: 發送YOLO拍照+檢測指令16...")
            if not self._write_register('CONTROL_COMMAND', CCD1Command.CAPTURE_DETECT):
                self.logger.error("發送檢測指令失敗")
                return False
            
            # 步驟3: 等待YOLO檢測完成
            self.logger.info("步驟3: 等待YOLO檢測完成...")
            if not self._wait_for_detection_complete_yolo(self.operation_timeout):
                self.logger.error("YOLO檢測失敗或超時")
                return False
            
            # 步驟4: 讀取YOLO檢測結果
            self.logger.info("步驟4: 讀取YOLO檢測結果...")
            coordinates = self._read_yolo_world_coordinates()
            detection_count = len(coordinates)
            
            # 更新統計
            cg_f_count = self._read_register('CG_F_COUNT') or 0
            cg_b_count = self._read_register('CG_B_COUNT') or 0
            stack_count = self._read_register('STACK_COUNT') or 0
            
            self.logger.info(f"YOLO檢測結果: CG_F={cg_f_count}, CG_B={cg_b_count}, STACK={stack_count}")
            
            # 步驟5: 更新CG_F FIFO佇列
            with self.queue_lock:
                for coord in coordinates:
                    self.coord_queue.append(coord)
                self.last_detection_count = detection_count
                self.last_cg_f_count = cg_f_count
            
            # 步驟6: 清空控制指令
            self.logger.info("步驟6: 清空控制指令...")
            self._write_register('CONTROL_COMMAND', CCD1Command.CLEAR)
            
            # 步驟7: 清空完成標誌 (可選)
            time.sleep(0.2)  # 等待系統處理
            
            self.logger.info(f"YOLO檢測完成，新增 {detection_count} 個CG_F座標到佇列")
            return True
            
        except Exception as e:
            self.logger.error(f"YOLO拍照檢測執行異常: {e}")
            return False
    
    def get_next_circle_world_coord(self) -> Optional[YOLODetectionCoord]:
        """
        獲取下一個CG_F物件世界座標 - YOLO版本
        
        FIFO佇列管理邏輯:
        1. 如果佇列為空，自動觸發YOLO拍照+檢測
        2. 從佇列前端取出一個CG_F座標
        3. 返回座標，佇列中移除該座標
        
        Returns:
            YOLODetectionCoord: CG_F世界座標，如果無可用座標則返回None
        """
        with self.queue_lock:
            # 檢查佇列是否為空
            if len(self.coord_queue) == 0:
                self.logger.info("CG_F佇列為空，觸發新的YOLO檢測...")
                with_lock_released = True
        
        # 在鎖外執行檢測 (避免死鎖)
        if 'with_lock_released' in locals():
            success = self.capture_and_detect()
            if not success:
                self.logger.error("YOLO自動檢測失敗")
                return None
        
        # 重新獲取鎖並取出座標
        with self.queue_lock:
            if len(self.coord_queue) > 0:
                coord = self.coord_queue.popleft()  # FIFO: 從前端取出
                self.logger.info(f"返回CG_F座標: ID={coord.id}, 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm, 模型={coord.model_id}")
                return coord
            else:
                self.logger.warning("CG_F佇列仍為空，無可用座標")
                return None
    
    def get_queue_status(self) -> Dict[str, Any]:
        """
        獲取佇列狀態資訊 - YOLO版本
        
        Returns:
            Dict: 包含佇列長度、檢測統計等資訊
        """
        with self.queue_lock:
            queue_length = len(self.coord_queue)
            queue_preview = []
            
            # 獲取前3個CG_F座標的預覽
            for i, coord in enumerate(list(self.coord_queue)[:3]):
                queue_preview.append({
                    'id': coord.id,
                    'object_type': coord.object_type,
                    'world_x': coord.world_x,
                    'world_y': coord.world_y,
                    'model_id': coord.model_id,
                    'timestamp': coord.timestamp
                })
        
        # 讀取當前檢測統計
        current_stats = {
            'cg_f_count': self._read_register('CG_F_COUNT') or 0,
            'cg_b_count': self._read_register('CG_B_COUNT') or 0,
            'stack_count': self._read_register('STACK_COUNT') or 0,
            'total_detections': self._read_register('TOTAL_DETECTIONS') or 0,
            'current_model_id': self._read_register('MODEL_ID_USED') or 0
        }
        
        return {
            'connected': self.connected,
            'queue_length': queue_length,
            'last_detection_count': self.last_detection_count,
            'last_cg_f_count': self.last_cg_f_count,
            'queue_preview': queue_preview,
            'current_stats': current_stats,
            'modbus_server': f"{self.modbus_host}:{self.modbus_port}",
            'version': 'YOLO'
        }
    
    def clear_queue(self):
        """清空CG_F座標佇列"""
        with self.queue_lock:
            self.coord_queue.clear()
            self.logger.info("CG_F座標佇列已清空")
    
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
            running = bool(status & (1 << CCD1StatusBits.RUNNING))
            return ready and not alarm and not running
        
        return False
    
    def get_system_status(self) -> Dict[str, Any]:
        """
        獲取CCD1系統狀態 - YOLO版本
        
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
                'version': 'YOLO'
            }
        
        status = self._read_register('STATUS_REGISTER')
        world_coord_valid = self._read_register('WORLD_COORD_VALID')
        
        # 讀取完成標誌
        capture_complete = self._read_register('CAPTURE_COMPLETE')
        detect_complete = self._read_register('DETECT_COMPLETE')
        operation_success = self._read_register('OPERATION_SUCCESS')
        error_code = self._read_register('ERROR_CODE')
        
        if status is not None:
            return {
                'connected': True,
                'ready': bool(status & (1 << CCD1StatusBits.READY)),
                'running': bool(status & (1 << CCD1StatusBits.RUNNING)),
                'alarm': bool(status & (1 << CCD1StatusBits.ALARM)),
                'initialized': bool(status & (1 << CCD1StatusBits.INITIALIZED)),
                'world_coord_valid': bool(world_coord_valid) if world_coord_valid is not None else False,
                'status_register_value': status,
                'completion_flags': {
                    'capture_complete': capture_complete,
                    'detect_complete': detect_complete,
                    'operation_success': operation_success,
                    'error_code': error_code
                },
                'version': 'YOLO'
            }
        
        return {
            'connected': True,
            'ready': False,
            'running': False,
            'alarm': True,
            'initialized': False,
            'world_coord_valid': False,
            'error': '無法讀取狀態寄存器',
            'version': 'YOLO'
        }
    
    def switch_model(self, model_id: int) -> bool:
        """
        切換YOLO模型 - YOLO版本新增功能
        
        Args:
            model_id: 模型ID (0=未指定, 1-20=模型ID)
            
        Returns:
            bool: 切換是否成功
        """
        if not self.connected:
            self.logger.error("Modbus未連接，無法切換模型")
            return False
        
        if not (0 <= model_id <= 20):
            self.logger.error(f"無效的模型ID: {model_id} (應為0-20)")
            return False
        
        try:
            success = self._write_register('MODEL_SELECT', model_id)
            if success:
                self.logger.info(f"模型切換指令發送成功: 模型{model_id}")
                return True
            else:
                self.logger.error(f"模型切換指令發送失敗: 模型{model_id}")
                return False
                
        except Exception as e:
            self.logger.error(f"模型切換異常: {e}")
            return False
    
    def get_model_status(self) -> Dict[str, Any]:
        """
        獲取模型狀態 - YOLO版本新增功能
        
        Returns:
            Dict: 模型狀態資訊
        """
        if not self.connected:
            return {'error': 'Modbus未連接'}
        
        try:
            model_select = self._read_register('MODEL_SELECT')
            model_used = self._read_register('MODEL_ID_USED')
            switch_count = self._read_register('MODEL_SWITCH_COUNT')
            
            return {
                'model_select': model_select,
                'model_used': model_used,
                'switch_count': switch_count,
                'model_description': f"模型{model_used}" if model_used and model_used > 0 else "未指定模型"
            }
            
        except Exception as e:
            return {'error': f'讀取模型狀態失敗: {e}'}


# ==================== 相容性包裝 - 保持原有API ====================
# 為了保持與原有代碼的相容性，提供CircleWorldCoord包裝
@dataclass  
class CircleWorldCoord:
    """圓心世界座標數據 - 相容性包裝"""
    id: int
    world_x: float
    world_y: float  
    pixel_x: int
    pixel_y: int
    radius: int = 0  # YOLO版本沒有半徑概念，設為0
    timestamp: str = ""
    
    @classmethod
    def from_yolo_coord(cls, yolo_coord: YOLODetectionCoord) -> 'CircleWorldCoord':
        """從YOLO座標轉換為相容性格式"""
        return cls(
            id=yolo_coord.id,
            world_x=yolo_coord.world_x,
            world_y=yolo_coord.world_y,
            pixel_x=yolo_coord.pixel_x,
            pixel_y=yolo_coord.pixel_y,
            radius=0,  # YOLO版本無半徑
            timestamp=yolo_coord.timestamp
        )


# 便利函數，供快速調用
def test_yolo_detection(host: str = "127.0.0.1", port: int = 502) -> bool:
    """便利函數：測試YOLO版本CCD1檢測功能
    
    Args:
        host: Modbus服務器IP
        port: Modbus服務器端口
        
    Returns:
        bool: 檢測是否成功
    """
    ccd1_api = CCD1HighLevelAPI(host, port)
    
    if not ccd1_api.connected:
        print("CCD1 YOLO API連接失敗")
        return False
    
    try:
        success = ccd1_api.capture_and_detect()
        
        if success:
            status = ccd1_api.get_queue_status()
            print(f"YOLO檢測成功，檢測到 {status['last_cg_f_count']} 個CG_F物件")
            print(f"統計: {status['current_stats']}")
            return True
        else:
            print("YOLO檢測失敗")
            return False
    finally:
        ccd1_api.disconnect()