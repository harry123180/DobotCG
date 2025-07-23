#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoFeeding_main.py - CG版本獨立入料檢測模組 (修正版 - 新增1202進度判斷)
基地址：900-999
功能：持續檢測CG_F，主動監控Flow1，新增進度判斷邏輯
修改：監控1201當前執行Flow，當值為1時根據1202進度判斷是否暫停自動進料程序
"""

import time
import math
import os
import json
import logging
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum
from logging.handlers import RotatingFileHandler

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("[ERROR] pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


def setup_logging(module_name: str) -> logging.Logger:
    """統一設置logging配置"""
    # 日誌目錄：執行檔同層目錄下的logs資料夾
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 格式化器
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 文件處理器 (輪替日誌，保存一週)
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, f'{module_name}.log'),
        maxBytes=10*1024*1024,  # 10MB
        backupCount=7,          # 保留7個檔案
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)
    
    # 控制台處理器
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    
    # 配置logger
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


class AutoFeedingStatus(Enum):
    """AutoFeeding狀態"""
    STOPPED = 0
    RUNNING = 1
    FLOW1_PAUSED = 2  # Flow1執行時暫停
    FLOW1_PROGRESS_PAUSED = 3  # Flow1進度小於44時暫停
    DETECTING = 4
    VP_VIBRATING = 5
    ERROR = 6


class OperationStatus(Enum):
    """操作狀態"""
    IDLE = 0
    CG_DETECTING = 1
    VP_CONTROLLING = 2
    FLOW4_TRIGGERING = 3


@dataclass
class CGDetectionResult:
    """CG檢測結果"""
    cg_f_count: int = 0
    total_detections: int = 0
    cg_f_world_coords: List[Tuple[float, float]] = None
    operation_success: bool = False
    
    def __post_init__(self):
        if self.cg_f_world_coords is None:
            self.cg_f_world_coords = []


class ProtectionZone:
    """CG專案保護區域判斷"""
    
    @staticmethod
    def is_point_in_quad(x_a: float, y_a: float) -> bool:
        """判斷點是否在CG保護區域四邊形內"""
        points = [
            (-86, -349.51),       # x1, y1
            (-112.82, -244.63),   # x2, y2
            (8.07, -244.64),      # x3, y3
            (8.06, -349.52)       # x4, y4
        ]
        
        # 找中心點並按極角排序
        cx = sum(p[0] for p in points) / 4
        cy = sum(p[1] for p in points) / 4
        
        def angle(p):
            return math.atan2(p[1] - cy, p[0] - cx)
        
        sorted_points = sorted(points, key=angle)
        
        # 射線法檢查
        def point_in_polygon(x, y, polygon):
            n = len(polygon)
            inside = False
            px, py = polygon[0]
            for i in range(1, n + 1):
                qx, qy = polygon[i % n]
                if ((py > y) != (qy > y)):
                    cross = (qx - px) * (y - py) / (qy - py + 1e-9) + px
                    if x < cross:
                        inside = not inside
                px, py = qx, qy
            return inside
        
        return point_in_polygon(x_a, y_a, sorted_points)


class AutoFeedingModule:
    """CG版本AutoFeeding獨立模組 (修正版 - 新增進度判斷)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        # 設置logger
        self.logger = setup_logging("AutoFeeding_CG")
        self.logger.info("CG版本AutoFeeding模組初始化開始")
        
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址範圍 900-999
        self.BASE_ADDRESS = 900
        
        # 外部模組地址
        self.CG_BASE = 200  # CG視覺模組基地址
        self.VP_BASE = 300
        self.FLOW4_ADDRESS = 448
        
        # 新增：監控1201當前執行Flow和1202進度
        self.CURRENT_MOTION_FLOW = 1201  # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
        self.MOTION_PROGRESS = 1202      # 運動進度 (0-100百分比)
        self.PROGRESS_THRESHOLD = 44     # 進度門檻值
        
        # 載入配置
        self.config = self.load_config()
        
        # 保護區域判斷
        self.protection_zone = ProtectionZone()
        
        # 系統狀態 - 修正邏輯
        self.status = AutoFeedingStatus.STOPPED
        self.operation_status = OperationStatus.IDLE
        self.running = False
        self.flow1_active = False  # 監控Flow1是否正在執行
        self.flow1_progress = 0    # 新增：Flow1進度
        self.vp_clearing_mode = False
        
        # CG_F狀態 - 核心資訊
        self.cg_f_available = False  # 保護區內是否有CG_F
        self.cg_f_coords = (0.0, 0.0)  # 當前CG_F座標
        self.cg_f_taken = False  # 座標是否已被讀取
        
        # 統計資訊
        self.cycle_count = 0
        self.cg_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        self.flow4_consecutive_count = 0
        self.vp_empty_detection_count = 0
        self.error_code = 0
        self.last_cg_f_count = 0
        self.last_background_count = 0
        
        self.logger.info(f"CG版本AutoFeeding獨立模組初始化 (進度判斷版) - 基地址{self.BASE_ADDRESS}")
        self.logger.info(f"監控當前執行Flow地址: {self.CURRENT_MOTION_FLOW}")
        self.logger.info(f"監控運動進度地址: {self.MOTION_PROGRESS}")
        self.logger.info(f"新邏輯：當1201=1且1202<{self.PROGRESS_THRESHOLD}時暫停自動進料程序")
        self.logger.info(f"新邏輯：當1201=1但1202>={self.PROGRESS_THRESHOLD}時啟動自動進料程序")
        self.logger.info(f"新邏輯：當1201!=1時啟動自動進料程序")
        self.logger.info(f"CG保護區域: (-86,-349.51) 到 (8.07,-244.64)")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autofeeding": {
                "cycle_interval": 1.0,
                "cg_timeout": 5.0,
                "flow4_consecutive_limit": 5,  # 這個鍵必須保留
                "vp_empty_check_count": 3,
                "auto_start": True,
                "progress_threshold": 44
            },
            "vp_params": {
                "spread_action_code": 11,
                "spread_strength": 37,
                "spread_frequency": 42,
                "spread_duration": 0.3,
                "stop_command_code": 3,
                "stop_delay": 0.1
            },
            "flow4_params": {
                "pulse_duration": 0.1,
                "pulse_interval": 0.05
            },
            "timing": {
                "command_delay": 0.05,
                "status_check_interval": 0.05,
                "register_clear_delay": 0.02,
                "vp_stabilize_delay": 0.15,
                "flow1_check_interval": 0.1
            },
            "coordination": {
                "coords_taken_timeout": 10.0
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autofeeding_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    
                    # 深層合併而不是直接替換
                    def deep_update(base_dict, update_dict):
                        for key, value in update_dict.items():
                            if key in base_dict and isinstance(base_dict[key], dict) and isinstance(value, dict):
                                deep_update(base_dict[key], value)
                            else:
                                base_dict[key] = value
                    
                    deep_update(default_config, loaded_config)
                self.logger.info(f"配置檔案已載入: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                self.logger.info(f"預設配置檔案已創建: {config_path}")
        except Exception as e:
            self.logger.error(f"配置檔案處理失敗: {e}", exc_info=True)
            
        # 更新進度門檻值
        self.PROGRESS_THRESHOLD = default_config['autofeeding'].get('progress_threshold', 44)
        
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                self.logger.error("Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                self.logger.info(f"Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_registers()
                return True
            else:
                self.logger.error(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
                return False
        except Exception as e:
            self.logger.error(f"Modbus連接異常: {e}", exc_info=True)
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            self.logger.info("Modbus連接已斷開")
    
    def init_registers(self):
        """初始化寄存器"""
        try:
            # 狀態寄存器 (900-919)
            self.write_register(900, AutoFeedingStatus.STOPPED.value)
            self.write_register(901, self.cycle_count)
            self.write_register(902, self.cg_f_found_count)
            self.write_register(903, self.flow4_trigger_count)
            self.write_register(904, self.vp_vibration_count)
            self.write_register(905, 0)  # 保留
            self.write_register(906, 0)  # 保留
            self.write_register(907, 0)  # 錯誤代碼
            self.write_register(908, OperationStatus.IDLE.value)
            self.write_register(909, 0)  # Flow1監控狀態
            self.write_register(910, 0)  # 新增：Flow1進度
            
            # CG_F狀態寄存器 (940-959)
            self.write_register(940, 0)  # CG_F可用標誌
            self.write_register(941, 0)  # CG_F座標X高位
            self.write_register(942, 0)  # CG_F座標X低位
            self.write_register(943, 0)  # CG_F座標Y高位
            self.write_register(944, 0)  # CG_F座標Y低位
            self.write_register(945, 0)  # 座標已讀取標誌
            
            # 配置參數寄存器 (960-979)
            self.write_register(960, int(self.config['autofeeding']['cycle_interval'] * 1000))
            self.write_register(961, int(self.config['autofeeding']['cg_timeout'] * 1000))
            self.write_register(962, self.config['vp_params']['spread_strength'])
            self.write_register(963, self.config['vp_params']['spread_frequency'])
            self.write_register(964, int(self.config['vp_params']['spread_duration'] * 1000))
            self.write_register(965, self.PROGRESS_THRESHOLD)  # 新增：進度門檻值
            
            self.logger.info("寄存器初始化完成 (CG版本進度判斷版)")
        except Exception as e:
            self.logger.error(f"寄存器初始化失敗: {e}", exc_info=True)
    
    def read_register(self, address: int) -> Optional[int]:
        """讀取單個寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except Exception:
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            return False
    
    def read_32bit_register(self, high_addr: int, low_addr: int) -> float:
        """讀取32位世界座標並轉換為實際值"""
        high_val = self.read_register(high_addr)
        low_val = self.read_register(low_addr)
        
        if high_val is None or low_val is None:
            return 0.0
        
        # 合併32位值
        combined = (high_val << 16) + low_val
        
        # 處理補碼(負數)
        if combined >= 2147483648:  # 2^31
            combined = combined - 4294967296  # 2^32
        
        # 轉換為毫米(除以100)
        return combined / 100.0
    
    def write_32bit_register(self, high_addr: int, low_addr: int, value: float) -> bool:
        """寫入32位世界座標"""
        # 轉換為整數形式(×100)
        value_int = int(value * 100)
        
        # 處理負數(補碼)
        if value_int < 0:
            value_int = value_int + 4294967296  # 2^32
        
        # 分解為高低位
        high_val = (value_int >> 16) & 0xFFFF
        low_val = value_int & 0xFFFF
        
        success = True
        success &= self.write_register(high_addr, high_val)
        success &= self.write_register(low_addr, low_val)
        
        return success
    
    def update_status_registers(self):
        """更新狀態寄存器"""
        try:
            self.write_register(900, self.status.value)
            self.write_register(901, self.cycle_count)
            self.write_register(902, self.cg_f_found_count)
            self.write_register(903, self.flow4_trigger_count)
            self.write_register(904, self.vp_vibration_count)
            self.write_register(907, self.error_code)
            self.write_register(908, self.operation_status.value)
            self.write_register(909, 1 if self.flow1_active else 0)
            self.write_register(910, self.flow1_progress)  # 新增：Flow1進度
            
            # 更新CG_F狀態
            self.write_register(940, 1 if self.cg_f_available else 0)
            if self.cg_f_available:
                self.write_32bit_register(941, 942, self.cg_f_coords[0])
                self.write_32bit_register(943, 944, self.cg_f_coords[1])
        except Exception as e:
            self.logger.error(f"狀態寄存器更新失敗: {e}", exc_info=True)
    
    def should_pause_feeding(self) -> Tuple[bool, str]:
        """判斷是否應該暫停自動進料"""
        try:
            current_motion_flow = self.read_register(self.CURRENT_MOTION_FLOW)
            motion_progress = self.read_register(self.MOTION_PROGRESS)
            
            if current_motion_flow is None or motion_progress is None:
                return False, "寄存器讀取失敗，繼續檢測"
            
            # 更新狀態
            self.flow1_progress = motion_progress
            
            if current_motion_flow == 1:
                # Flow1正在執行，檢查進度
                if motion_progress < self.PROGRESS_THRESHOLD:
                    return True, f"暫停原因：Flow1執行中且進度({motion_progress})<{self.PROGRESS_THRESHOLD}"
                else:
                    return False, f"繼續檢測：Flow1執行中但進度({motion_progress})>={self.PROGRESS_THRESHOLD}"
            else:
                # Flow1未執行，正常檢測
                return False, f"正常檢測：Flow1未執行(當前Flow={current_motion_flow})"
                
        except Exception as e:
            self.logger.error(f"暫停判斷失敗: {e}", exc_info=True)
            return False, "判斷異常，繼續檢測"
    
    def check_flow1_status(self) -> bool:
        """主動監控當前執行Flow狀態和進度"""
        try:
            should_pause, reason = self.should_pause_feeding()
            
            current_motion_flow = self.read_register(self.CURRENT_MOTION_FLOW)
            if current_motion_flow is None:
                return False
            
            flow1_now_active = (current_motion_flow == 1)
            
            # 根據暫停判斷更新狀態
            if should_pause:
                if self.status not in [AutoFeedingStatus.FLOW1_PAUSED, AutoFeedingStatus.FLOW1_PROGRESS_PAUSED]:
                    self.logger.warning(f"暫停自動進料 - {reason}")
                    if current_motion_flow == 1 and self.flow1_progress < self.PROGRESS_THRESHOLD:
                        self.status = AutoFeedingStatus.FLOW1_PROGRESS_PAUSED
                    else:
                        self.status = AutoFeedingStatus.FLOW1_PAUSED
            else:
                # 確保在不暫停時設置為RUNNING狀態
                if self.status in [AutoFeedingStatus.FLOW1_PAUSED, AutoFeedingStatus.FLOW1_PROGRESS_PAUSED]:
                    self.logger.info(f"恢復自動進料 - {reason}")
                    self.status = AutoFeedingStatus.RUNNING
                elif self.status == AutoFeedingStatus.DETECTING:
                    # 檢測完成後也要設置為RUNNING
                    self.status = AutoFeedingStatus.RUNNING
            
            # 更新Flow1狀態
            if flow1_now_active != self.flow1_active:
                self.flow1_active = flow1_now_active
                if self.flow1_active:
                    self.logger.info(f"檢測到Flow1開始執行 (1201=1)")
                else:
                    self.logger.info(f"檢測到Flow1執行完成 (1201=0)")
            
            return True
        except Exception as e:
            self.logger.error(f"Flow1狀態檢查失敗: {e}", exc_info=True)
            return False
    
    def check_coords_taken(self):
        """檢查座標是否被Flow1讀取"""
        if not self.cg_f_available:
            return
        
        try:
            coords_taken = self.read_register(945)  # Flow1設置此標誌表示已讀取座標
            if coords_taken == 1:
                self.logger.info("座標已被Flow1讀取，清除CG_F狀態")
                # 清除CG_F狀態，繼續檢測新的
                self.cg_f_available = False
                self.cg_f_coords = (0.0, 0.0)
                self.cg_f_taken = True
                
                # 清除相關寄存器
                self.write_register(940, 0)  # CG_F可用標誌
                self.write_register(945, 0)  # 座標已讀取標誌
                for addr in [941, 942, 943, 944]:
                    self.write_register(addr, 0)
                
                self.logger.info("CG_F狀態已清除，繼續檢測新的正面物件")
        except Exception as e:
            self.logger.error(f"座標讀取檢查失敗: {e}", exc_info=True)
    
    def check_modules_status(self) -> bool:
        """檢查CG、VP模組狀態"""
        # 檢查CG狀態
        cg_status = self.read_register(201)
        if cg_status is None:
            if self.cycle_count % 50 == 1:
                self.logger.debug("CG模組無回應")
            self.error_code = 101
            return False
        
        cg_ready = bool(cg_status & 0x01)
        cg_alarm = bool(cg_status & 0x04)
        cg_initialized = bool(cg_status & 0x08)
        
        if self.cycle_count % 10 == 1:
            self.logger.debug(f"CG狀態詳細: 原始值={cg_status}, Ready={cg_ready}, Alarm={cg_alarm}, Init={cg_initialized}")
        
        if cg_alarm or not cg_initialized:
            if self.cycle_count % 10 == 1:
                self.logger.warning(f"CG狀態問題: Alarm={cg_alarm}, Initialized={cg_initialized}")
            self.error_code = 102
            return False
        
        if not cg_ready:
            if self.cycle_count % 10 == 1:
                self.logger.warning(f"CG未Ready: status={cg_status}")
            self.error_code = 102
            return False
        
        # 檢查VP狀態
        vp_status = self.read_register(300)
        vp_connected = self.read_register(301)
        
        if vp_status is None or vp_connected is None:
            if self.cycle_count % 50 == 1:
                self.logger.debug("VP模組無回應")
            self.error_code = 103
            return False
        
        if vp_status != 1 or vp_connected != 1:
            if self.cycle_count % 50 == 1:
                self.logger.debug(f"VP模組狀態異常: status={vp_status}, connected={vp_connected}")
            self.error_code = 103
            return False
        
        return True
    
    def trigger_cg_detection(self) -> CGDetectionResult:
        """觸發CG檢測"""
        self.operation_status = OperationStatus.CG_DETECTING
        result = CGDetectionResult()
        
        # 觸發拍照+檢測
        if not self.write_register(200, 16):
            self.error_code = 201
            return result
        
        # 等待檢測完成
        timeout = self.config['autofeeding']['cg_timeout']
        start_time = time.time()
        check_interval = 0.02
        
        while (time.time() - start_time) < timeout:
            capture_complete = self.read_register(203)
            detect_complete = self.read_register(204)
            operation_success = self.read_register(205)
            
            if capture_complete == 1 and detect_complete == 1 and operation_success == 1:
                result.operation_success = True
                break
            
            time.sleep(check_interval)
        
        if not result.operation_success:
            self.error_code = 202
            return result
        
        # 讀取檢測結果
        result.cg_f_count = self.read_register(240) or 0
        result.total_detections = self.read_register(243) or 0
        
        # 提取CG_F世界座標
        if result.cg_f_count > 0:
            for i in range(min(result.cg_f_count, 5)):
                base_addr = 261 + (i * 4)
                world_x = self.read_32bit_register(base_addr, base_addr + 1)
                world_y = self.read_32bit_register(base_addr + 2, base_addr + 3)
                result.cg_f_world_coords.append((world_x, world_y))
        
        # 清空CG寄存器
        self.write_register(200, 0)
        self.write_register(203, 0)
        self.write_register(204, 0)
        self.write_register(205, 0)
        
        return result
    
    def find_cg_f_in_protection_zone(self, detection_result: CGDetectionResult) -> Optional[Tuple[float, float]]:
        """尋找保護區域內的CG_F"""
        if detection_result.cg_f_count == 0:
            return None
        
        for world_x, world_y in detection_result.cg_f_world_coords:
            if self.protection_zone.is_point_in_quad(world_x, world_y):
                return (world_x, world_y)
        
        return None
    
    def trigger_vp_vibration(self) -> bool:
        """觸發VP震動 - 使用CASE版本格式"""
        self.operation_status = OperationStatus.VP_CONTROLLING
        self.logger.debug(f"VP震動參數: action={self.config['vp_params']['spread_action_code']}, strength={self.config['vp_params']['spread_strength']}, frequency={self.config['vp_params']['spread_frequency']}")
        
        spread_action_code = self.config['vp_params']['spread_action_code']  # 11
        spread_strength = self.config['vp_params']['spread_strength']        # 37  
        spread_frequency = self.config['vp_params']['spread_frequency']      # 42
        command_id = int(time.time()) % 65535
        
        # 啟動震動
        success = True
        success &= self.write_register(320, 5)                    # 執行動作指令
        success &= self.write_register(321, spread_action_code)   # 參數1: 動作碼
        success &= self.write_register(322, spread_strength)      # 參數2: 強度  
        success &= self.write_register(323, spread_frequency)     # 參數3: 頻率
        success &= self.write_register(324, command_id)          # 指令ID
        
        if not success:
            self.error_code = 301
            return False
        
        self.logger.info(f"VP震動啟動: 動作={spread_action_code}, 強度={spread_strength}, 頻率={spread_frequency}")
        
        # 震動持續時間
        time.sleep(self.config['vp_params']['spread_duration'])
        
        # 停止震動
        return self.stop_vp_vibration()
    
    def stop_vp_vibration(self) -> bool:
        """停止VP震動"""
        success = True
        success &= self.write_register(320, self.config['vp_params']['stop_command_code'])  # 停止指令
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, 99)  # 指令ID
        
        if success:
            time.sleep(self.config['vp_params']['stop_delay'])
            self.logger.info("VP震動已停止")
        
        return success
    
    def trigger_flow4_feeding(self) -> bool:
        """觸發Flow4送料"""
        self.operation_status = OperationStatus.FLOW4_TRIGGERING
        
        pulse_duration = self.config['flow4_params']['pulse_duration']
        
        if not self.write_register(self.FLOW4_ADDRESS, 1):
            self.error_code = 401
            return False
        
        time.sleep(pulse_duration)
        
        if not self.write_register(self.FLOW4_ADDRESS, 0):
            self.error_code = 402
            return False
        
        return True
    
    def set_cg_f_available(self, coords: Tuple[float, float]):
        """設置CG_F可用狀態"""
        self.cg_f_available = True
        self.cg_f_coords = coords
        self.cg_f_taken = False
        
        # 立即更新寄存器讓Flow1可以讀取
        self.write_register(940, 1)  # CG_F可用標誌
        self.write_32bit_register(941, 942, coords[0])  # X座標
        self.write_32bit_register(943, 944, coords[1])  # Y座標
        
        self.logger.info(f"CG_F已就緒: {coords}, Flow1可直接讀取座標")
    
    def should_recheck_detection(self, current_cg_f: int, current_background: int) -> bool:
        """
        判斷是否需要重新檢測
        
        條件：
        1. 上次CG_F>=3，這次CG_F=0
        2. 上次背景物件>=10，這次背景物件>=2
        
        Args:
            current_cg_f: 當前檢測到的CG_F數量
            current_background: 當前背景物件數量
            
        Returns:
            bool: 是否需要重新檢測
        """
        # 第一次檢測，沒有歷史數據
        if not hasattr(self, 'last_cg_f_count') or not hasattr(self, 'last_background_count'):
            return False
        
        # 檢查CG_F異常歸零
        cg_f_abnormal = (self.last_cg_f_count >= 3 and current_cg_f < 1)
        
        # 檢查背景物件仍然存在
        background_sufficient = (self.last_background_count >= 3 and current_background < 2)
        
        # 兩個條件都滿足才重檢
        return cg_f_abnormal and background_sufficient
    
    def feeding_cycle(self) -> bool:
        """執行一次入料檢測週期 - CG版本完整版"""
        try:
            self.cycle_count += 1
            self.status = AutoFeedingStatus.DETECTING
            self.error_code = 0  # 重置錯誤碼
            
            # 快速檢查模組狀態
            if not self.check_modules_status():
                if self.error_code == 102:  # CG初始化中
                    self.status = AutoFeedingStatus.RUNNING
                    return True
                return False
            
            # CG檢測
            detection_result = self.trigger_cg_detection()
            if not detection_result.operation_success:
                self.logger.warning(f"週期{self.cycle_count} CG檢測失敗")
                return False
            
            # 讀取CG_B和STACK數量計算背景物件
            cg_b_count = self.read_register(241) or 0
            stack_count = self.read_register(242) or 0
            background_count = cg_b_count + stack_count
            
            # 異常檢測重檢邏輯
            need_recheck = self.should_recheck_detection(
                detection_result.cg_f_count, 
                background_count
            )
            
            if need_recheck:
                self.logger.warning(f"檢測結果異常，1秒後重新檢測")
                self.logger.warning(f"  上次CG_F={self.last_cg_f_count}, 這次CG_F={detection_result.cg_f_count}")
                self.logger.warning(f"  上次背景={self.last_background_count}, 這次背景={background_count}")
                
                time.sleep(1.0)
                
                retry_result = self.trigger_cg_detection()
                if retry_result.operation_success:
                    detection_result = retry_result
                    cg_b_count = self.read_register(241) or 0
                    stack_count = self.read_register(242) or 0
                    background_count = cg_b_count + stack_count
                    
                    self.logger.info(f"重檢結果: CG_F={detection_result.cg_f_count}, 背景物件={background_count}")
                else:
                    self.logger.error("重檢失敗，使用原始檢測結果")
            
            # 記錄當前檢測結果供下次比較
            self.last_cg_f_count = detection_result.cg_f_count
            self.last_background_count = background_count
            
            self.logger.info(f"週期{self.cycle_count} 檢測結果:")
            self.logger.info(f"  CG_F={detection_result.cg_f_count}")
            self.logger.info(f"  CG_B={cg_b_count}")
            self.logger.info(f"  STACK={stack_count}")
            self.logger.info(f"  背景物件總數={background_count}")
            self.logger.info(f"  總檢測數={detection_result.total_detections}")
            
            # 尋找保護區域內的CG_F
            target_coords = self.find_cg_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 找到正面物件 - 直接設置可用狀態
                self.cg_f_found_count += 1
                self.flow4_consecutive_count = 0
                self.logger.info(f"找到保護區內CG_F: {target_coords}")
                
                # 設置CG_F可用狀態
                self.set_cg_f_available(target_coords)
                
                # 檢查是否需要供料
                if background_count < 4:
                    self.logger.info(f"背景物件不足({background_count}<4)，但有CG_F可夾取，暫不供料")
                    self.logger.info(f"等待Flow1夾取後再評估是否需要補料")
                else:
                    self.logger.info(f"背景物件充足({background_count}>=4)，CG_F已就緒")
            else:
                # 保護區無CG_F - 根據背景物件數量決定動作
                if background_count < 4:
                    # 背景物件不足且無正面，觸發直振供料
                    self.logger.info(f"背景物件不足({background_count}<4)且無CG_F，觸發Flow4直振供料")
                    
                    if self.trigger_flow4_feeding():
                        self.flow4_trigger_count += 1
                        self.flow4_consecutive_count += 1
                        self.logger.info(f"Flow4直振供料完成 (連續{self.flow4_consecutive_count}次)")
                        
                        # 檢查連續直振限制
                        flow4_limit = self.config['autofeeding'].get('flow4_consecutive_limit', 5)
                        if self.flow4_consecutive_count >= flow4_limit:
                            self.logger.warning("達到連續直振限制，需要VP清空處理")
                    else:
                        self.logger.error("Flow4直振供料失敗")
                else:
                    # 背景物件充足但無正面，VP震動重檢
                    self.logger.info(f"背景物件充足({background_count}>=4)但無CG_F，VP震動散開重檢")
                    self.flow4_consecutive_count = 0
                    
                    if self.trigger_vp_vibration():
                        self.vp_vibration_count += 1
                        self.logger.info("VP震動完成，等待穩定後重新檢測")
                        
                        time.sleep(self.config['timing']['vp_stabilize_delay'])
                        
                        # 立即重新檢測
                        retry_result = self.trigger_cg_detection()
                        if retry_result.operation_success:
                            retry_cg_b = self.read_register(241) or 0
                            retry_stack = self.read_register(242) or 0
                            retry_background = retry_cg_b + retry_stack
                            
                            self.logger.info(f"震動後重檢結果:")
                            self.logger.info(f"  CG_F={retry_result.cg_f_count}")
                            self.logger.info(f"  背景物件={retry_background}")
                            
                            retry_coords = self.find_cg_f_in_protection_zone(retry_result)
                            if retry_coords:
                                self.cg_f_found_count += 1
                                self.logger.info(f"震動後找到CG_F: {retry_coords}")
                                self.set_cg_f_available(retry_coords)
                            else:
                                self.logger.debug("震動後仍無保護區內CG_F")
            
            self.operation_status = OperationStatus.IDLE
            self.status = AutoFeedingStatus.RUNNING
            return True
            
        except Exception as e:
            self.logger.error(f"入料週期異常: {e}", exc_info=True)
            self.error_code = 999
            return False
    
    def start_feeding(self):
        """啟動入料檢測"""
        if self.running:
            return
        
        self.logger.info("啟動持續入料檢測")
        self.logger.info("目標：保持保護區域內始終有CG_F可用")
        
        self.running = True
        self.status = AutoFeedingStatus.RUNNING
        self.error_code = 0
        
        # 重置CG_F狀態
        self.cg_f_available = False
        self.cg_f_coords = (0.0, 0.0)
        self.cg_f_taken = False
    
    def stop_feeding(self):
        """停止入料檢測"""
        self.running = False
        self.status = AutoFeedingStatus.STOPPED
        self.cg_f_available = False
        self.emergency_stop_vp()
        self.logger.info("入料檢測已停止")
    
    def emergency_stop_vp(self):
        """緊急停止VP"""
        try:
            self.stop_vp_vibration()
            self.logger.warning("VP緊急停止")
        except Exception as e:
            self.logger.error(f"VP緊急停止失敗: {e}", exc_info=True)
    
    def main_loop(self):
        """主循環 - CG版本修正版含進度判斷"""
        self.logger.info("CG版本主循環啟動 (進度判斷版)")
        self.logger.info("特性：")
        self.logger.info("  ✓ 主動監控當前執行Flow狀態 (1201)")
        self.logger.info("  ✓ 主動監控運動進度 (1202)")
        self.logger.info(f"  ✓ 當1201=1且1202<{self.PROGRESS_THRESHOLD}時暫停自動進料程序")
        self.logger.info(f"  ✓ 當1201=1但1202>={self.PROGRESS_THRESHOLD}時啟動自動進料程序")
        self.logger.info("  ✓ 當1201!=1時啟動自動進料程序")
        self.logger.info("  ✓ 持續檢測確保CG_F可用")
        self.logger.info("  ✓ CG保護區域判斷")
        self.logger.info("  ✓ Flow1直接讀取座標")
        self.logger.info("  ✓ 異常檢測重檢邏輯")
        
        # 自動啟動檢測
        auto_start = self.config['autofeeding'].get('auto_start', True)
        if auto_start:
            self.start_feeding()
        
        loop_count = 0
        
        while True:
            try:
                loop_count += 1
                
                # 定期打印狀態
                if loop_count % 2000 == 1:
                    should_pause, reason = self.should_pause_feeding()
                    self.logger.debug(f"主循環 {loop_count}: running={self.running}, status={self.status.name}, "
                                    f"flow1_active={self.flow1_active}, progress={self.flow1_progress}, "
                                    f"should_pause={should_pause}, cg_f_available={self.cg_f_available}")
                    self.logger.debug(f"暫停原因: {reason}")
                
                # 檢查連接狀態
                if not self.connected:
                    self.logger.debug("Modbus連接斷開，嘗試重連")
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                
                # 主動監控Flow1狀態和進度
                self.check_flow1_status()
                
                # 檢查座標是否被讀取
                if self.cg_f_available:
                    self.check_coords_taken()
                
                # 更新狀態寄存器
                self.update_status_registers()
                
                # 執行入料檢測 - 只有在運行且未暫停時
                if (self.running and 
                    self.status not in [AutoFeedingStatus.FLOW1_PAUSED, AutoFeedingStatus.FLOW1_PROGRESS_PAUSED] and 
                    not self.vp_clearing_mode):
                    
                    if not self.feeding_cycle():
                        if self.error_code == 0:
                            # 錯誤碼0表示正常狀態下的返回False（如Flow1未執行等）
                            # 不設置ERROR狀態，繼續檢測
                            pass
                        else:
                            self.logger.debug(f"入料檢測失敗，錯誤碼: {self.error_code}")
                            self.status = AutoFeedingStatus.ERROR
                            time.sleep(0.5)
                    else:
                        # 檢測成功，快速進入下一輪
                        cycle_interval = self.config['autofeeding']['cycle_interval']
                        time.sleep(cycle_interval)
                else:
                    # 非運行狀態或暫停中，短間隔檢查
                    time.sleep(self.config['timing']['flow1_check_interval'])
                    
            except KeyboardInterrupt:
                self.logger.info("收到中斷信號，準備退出")
                break
            except Exception as e:
                self.logger.error(f"主循環異常: {e}", exc_info=True)
                time.sleep(1.0)
        
        # 清理資源
        self.stop_feeding()
        self.disconnect()
        self.logger.info("程序已退出")


def main():
    """主程序入口"""
    print("=== CG版本AutoFeeding獨立模組啟動 (進度判斷版) ===")
    print("基地址範圍: 900-999")
    print("主要改進:")
    print("  新增監控運動進度地址(1202)")
    print("  當1201=1且1202<44時暫停自動進料程序")
    print("  當1201=1但1202>=44時啟動自動進料程序")
    print("  當1201!=1時啟動自動進料程序")
    print("  持續檢測保持CG_F可用")
    print("  CG保護區域判斷")
    print("  Flow1直接讀取座標(940-944)")
    print("  自動啟動檢測")
    print("  完整logging系統")
    print("  異常檢測重檢邏輯")
    
    # 創建AutoFeeding模組
    autofeeding = AutoFeedingModule()
    
    # 連接Modbus
    if not autofeeding.connect():
        print("[ERROR] Modbus連接失敗，程序退出")
        return
    
    # 啟動主循環
    autofeeding.main_loop()


if __name__ == "__main__":
    main()