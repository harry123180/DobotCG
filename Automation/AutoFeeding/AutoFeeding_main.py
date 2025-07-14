#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoFeeding_main.py - CG版本獨立入料檢測模組
基地址：900-999
功能：持續檢測CG_F在保護區域內，確保供料充足
特性：
- 檢測CG_F/CG_B/STACK三種物件
- 保護區域判斷
- VP震動盤控制
- Flow4直振送料
- 連續直振監控與VP清空流程
- 獨立模組設計，避免執行緒穩定性問題
"""

import time
import math
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("[ERROR] pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


class AutoFeedingStatus(Enum):
    """AutoFeeding狀態"""
    STOPPED = 0
    RUNNING = 1
    FLOW1_PAUSED = 2  # Flow1執行時暫停
    DETECTING = 3
    VP_VIBRATING = 4
    VP_CLEARING = 5
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
    """CG版本AutoFeeding獨立模組"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
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
        
        # 監控當前執行Flow地址
        self.CURRENT_MOTION_FLOW = 1201  # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
        
        # 載入配置
        self.config = self.load_config()
        
        # 保護區域判斷
        self.protection_zone = ProtectionZone()
        
        # 系統狀態
        self.status = AutoFeedingStatus.STOPPED
        self.operation_status = OperationStatus.IDLE
        self.running = False
        self.flow1_active = False  # 監控Flow1是否正在執行
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
        
        print(f"[AutoFeeding] CG版本獨立模組初始化 - 基地址{self.BASE_ADDRESS}")
        print(f"[AutoFeeding] 監控當前執行Flow地址: {self.CURRENT_MOTION_FLOW}")
        print(f"[AutoFeeding] 當1201=1時暫停自動進料程序")
        print(f"[AutoFeeding] CG保護區域: (-86,-369.51) 到 (8.07,-244.64)")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autofeeding": {
                "cycle_interval": 1.0,     # 1秒檢測週期
                "cg_timeout": 5.0,         # CG檢測超時
                "flow4_consecutive_limit": 5,
                "vp_empty_check_count": 3,
                "auto_start": True         # 啟動後自動開始檢測
            },
            "vp_params": {
                "spread_action_code": 11,
                "spread_strength": 44,
                "spread_frequency": 44,
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
                "flow1_check_interval": 0.1  # Flow1監控間隔
            },
            "coordination": {
                "coords_taken_timeout": 10.0  # 座標被讀取超時
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autofeeding_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"[AutoFeeding] 配置檔案已載入: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"[AutoFeeding] 預設配置檔案已創建: {config_path}")
        except Exception as e:
            print(f"[ERROR] 配置檔案處理失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("[ERROR] Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"[AutoFeeding] Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_registers()
                return True
            else:
                print(f"[ERROR] Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
                return False
        except Exception as e:
            print(f"[ERROR] Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("[AutoFeeding] Modbus連接已斷開")
    
    def init_registers(self):
        """初始化寄存器"""
        try:
            # 狀態寄存器 (900-919)
            self.write_register(900, AutoFeedingStatus.STOPPED.value)  # 模組狀態
            self.write_register(901, self.cycle_count)                  # 週期計數
            self.write_register(902, self.cg_f_found_count)            # CG_F找到次數
            self.write_register(903, self.flow4_trigger_count)         # Flow4觸發次數
            self.write_register(904, self.vp_vibration_count)          # VP震動次數
            self.write_register(905, 0)  # 保留
            self.write_register(906, 0)  # 保留
            self.write_register(907, 0)  # 錯誤代碼
            self.write_register(908, OperationStatus.IDLE.value)       # 操作狀態
            self.write_register(909, 0)  # Flow1監控狀態
            
            # CG_F狀態寄存器 (940-959) - 簡化交握
            self.write_register(940, 0)  # CG_F可用標誌 (0=無, 1=有)
            self.write_register(941, 0)  # CG_F座標X高位
            self.write_register(942, 0)  # CG_F座標X低位
            self.write_register(943, 0)  # CG_F座標Y高位
            self.write_register(944, 0)  # CG_F座標Y低位
            self.write_register(945, 0)  # 座標已讀取標誌 (Flow1設置)
            self.write_register(946, 0)  # 保留
            self.write_register(947, 0)  # 保留
            
            # 配置參數寄存器 (960-979)
            self.write_register(960, int(self.config['autofeeding']['cycle_interval'] * 1000))
            self.write_register(961, int(self.config['autofeeding']['cg_timeout'] * 1000))
            self.write_register(962, self.config['vp_params']['spread_strength'])
            self.write_register(963, self.config['vp_params']['spread_frequency'])
            self.write_register(964, int(self.config['vp_params']['spread_duration'] * 1000))
            
            print("[AutoFeeding] 寄存器初始化完成 (CG版本)")
        except Exception as e:
            print(f"[ERROR] 寄存器初始化失敗: {e}")
    
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
            
            # 更新CG_F狀態
            self.write_register(940, 1 if self.cg_f_available else 0)
            if self.cg_f_available:
                self.write_32bit_register(941, 942, self.cg_f_coords[0])
                self.write_32bit_register(943, 944, self.cg_f_coords[1])
        except Exception as e:
            print(f"[ERROR] 狀態寄存器更新失敗: {e}")
    
    def check_flow1_status(self) -> bool:
        """監控當前執行Flow狀態"""
        try:
            current_motion_flow = self.read_register(self.CURRENT_MOTION_FLOW)
            if current_motion_flow is None:
                return False
            
            # 檢查當前執行Flow狀態變化
            flow1_now_active = (current_motion_flow == 1)
            
            if flow1_now_active != self.flow1_active:
                # Flow1狀態變化
                self.flow1_active = flow1_now_active
                if self.flow1_active:
                    print(f"[AutoFeeding] 檢測到Flow1正在執行 ({self.CURRENT_MOTION_FLOW}=1)，暫停檢測")
                    if self.status == AutoFeedingStatus.RUNNING:
                        self.status = AutoFeedingStatus.FLOW1_PAUSED
                else:
                    print(f"[AutoFeeding] 檢測到Flow1執行完成 ({self.CURRENT_MOTION_FLOW}=0)，恢復檢測")
                    if self.status == AutoFeedingStatus.FLOW1_PAUSED:
                        self.status = AutoFeedingStatus.RUNNING
                        # Flow1完成後，檢查座標是否被讀取
                        self.check_coords_taken()
            
            return True
        except Exception as e:
            print(f"[ERROR] Flow1狀態檢查失敗: {e}")
            return False
    
    def check_coords_taken(self):
        """檢查座標是否被Flow1讀取"""
        if not self.cg_f_available:
            return
        
        try:
            coords_taken = self.read_register(945)  # Flow1設置此標誌表示已讀取座標
            if coords_taken == 1:
                print(f"[AutoFeeding] 座標已被Flow1讀取，清除CG_F狀態")
                # 清除CG_F狀態，繼續檢測新的
                self.cg_f_available = False
                self.cg_f_coords = (0.0, 0.0)
                self.cg_f_taken = True
                
                # 清除相關寄存器
                self.write_register(940, 0)  # CG_F可用標誌
                self.write_register(945, 0)  # 座標已讀取標誌
                for addr in [941, 942, 943, 944]:
                    self.write_register(addr, 0)
                
                print(f"[AutoFeeding] CG_F狀態已清除，繼續檢測新的正面物件")
        except Exception as e:
            print(f"[ERROR] 座標讀取檢查失敗: {e}")
    
    def check_modules_status(self) -> bool:
        """檢查CG、VP模組狀態"""
        # 檢查CG狀態
        cg_status = self.read_register(201)
        if cg_status is None:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] CG模組無回應")
            self.error_code = 101
            return False
        
        cg_ready = bool(cg_status & 0x01)
        cg_alarm = bool(cg_status & 0x04)
        cg_initialized = bool(cg_status & 0x08)
        
        if self.cycle_count % 100 == 1:  # 減少打印頻率
            print(f"[DEBUG] CG狀態: {cg_status} (Ready={cg_ready}, Alarm={cg_alarm}, Init={cg_initialized})")
        
        if cg_alarm or not cg_initialized:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] CG仍在初始化或有警報，等待...")
            self.error_code = 102
            return False
        
        if not cg_ready:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] CG未Ready，等待...")
            self.error_code = 102
            return False
        
        # 檢查VP狀態
        vp_status = self.read_register(300)
        vp_connected = self.read_register(301)
        
        if vp_status is None or vp_connected is None:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] VP模組無回應")
            self.error_code = 103
            return False
        
        if vp_status != 1 or vp_connected != 1:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] VP模組狀態異常: status={vp_status}, connected={vp_connected}")
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
        """觸發VP震動"""
        self.operation_status = OperationStatus.VP_CONTROLLING
        
        # 啟動震動
        success = True
        success &= self.write_register(320, 5)  # execute_action
        success &= self.write_register(321, self.config['vp_params']['spread_action_code'])
        success &= self.write_register(322, self.config['vp_params']['spread_strength'])
        success &= self.write_register(323, self.config['vp_params']['spread_frequency'])
        success &= self.write_register(324, int(time.time()) % 65535)
        
        if not success:
            self.error_code = 301
            return False
        
        # 震動持續時間
        time.sleep(self.config['vp_params']['spread_duration'])
        
        # 停止震動
        return self.stop_vp_vibration()
    
    def stop_vp_vibration(self) -> bool:
        """停止VP震動"""
        success = True
        success &= self.write_register(320, self.config['vp_params']['stop_command_code'])
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, 99)
        
        if success:
            time.sleep(self.config['vp_params']['stop_delay'])
        
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
        
        print(f"[AutoFeeding] CG_F已就緒: {coords}, Flow1可直接讀取座標")
    
    def feeding_cycle(self) -> bool:
        """執行一次入料檢測週期"""
        try:
            self.cycle_count += 1
            self.status = AutoFeedingStatus.DETECTING
            
            # 快速檢查模組狀態
            if not self.check_modules_status():
                if self.error_code == 102:  # CG初始化中
                    self.status = AutoFeedingStatus.RUNNING
                    return True
                return False
            
            # CG檢測
            detection_result = self.trigger_cg_detection()
            if not detection_result.operation_success:
                print(f"[AutoFeeding] 週期{self.cycle_count} CG檢測失敗")
                return False
            
            print(f"[AutoFeeding] 週期{self.cycle_count} 檢測結果: CG_F={detection_result.cg_f_count}, 總數={detection_result.total_detections}")
            
            # 尋找保護區域內的CG_F
            target_coords = self.find_cg_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 找到正面物件
                self.cg_f_found_count += 1
                self.flow4_consecutive_count = 0
                print(f"[AutoFeeding] 找到保護區內CG_F: {target_coords}")
                
                # 設置CG_F可用狀態
                self.set_cg_f_available(target_coords)
                
            elif detection_result.total_detections < 4:
                # 料件不足，觸發Flow4送料
                print(f"[AutoFeeding] 料件不足 (總數={detection_result.total_detections}<4)，觸發Flow4送料")
                
                if self.trigger_flow4_feeding():
                    self.flow4_trigger_count += 1
                    self.flow4_consecutive_count += 1
                    print(f"[AutoFeeding] Flow4送料完成 (連續{self.flow4_consecutive_count}次)")
                    
                    # 檢查連續直振限制
                    if self.flow4_consecutive_count >= self.config['autofeeding']['flow4_consecutive_limit']:
                        print("[AutoFeeding] 達到連續直振限制，需要VP清空")
                        # 這裡可以加入VP清空流程或報警
                else:
                    print(f"[AutoFeeding] Flow4送料失敗")
                
            else:
                # 料件充足但無正面，VP震動重檢
                print(f"[AutoFeeding] 料件充足 (總數={detection_result.total_detections}>=4) 但無正面，VP震動重檢")
                self.flow4_consecutive_count = 0
                
                if self.trigger_vp_vibration():
                    self.vp_vibration_count += 1
                    print(f"[AutoFeeding] VP震動完成，等待穩定後重新檢測")
                    
                    # 等待穩定
                    time.sleep(self.config['timing']['vp_stabilize_delay'])
                    
                    # 立即重新檢測
                    retry_result = self.trigger_cg_detection()
                    if retry_result.operation_success:
                        print(f"[AutoFeeding] 震動後重檢: CG_F={retry_result.cg_f_count}, 總數={retry_result.total_detections}")
                        retry_coords = self.find_cg_f_in_protection_zone(retry_result)
                        if retry_coords:
                            self.cg_f_found_count += 1
                            print(f"[AutoFeeding] 震動後找到CG_F: {retry_coords}")
                            self.set_cg_f_available(retry_coords)
            
            self.operation_status = OperationStatus.IDLE
            self.status = AutoFeedingStatus.RUNNING
            return True
            
        except Exception as e:
            print(f"[ERROR] 入料週期異常: {e}")
            self.error_code = 999
            return False
    
    def start_feeding(self):
        """啟動入料檢測"""
        if self.running:
            return
        
        print("[AutoFeeding] 啟動持續入料檢測")
        print("[AutoFeeding] 目標：保持保護區域內始終有CG_F可用")
        
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
        print("[AutoFeeding] 入料檢測已停止")
    
    def emergency_stop_vp(self):
        """緊急停止VP"""
        try:
            self.stop_vp_vibration()
            print("[AutoFeeding] VP緊急停止")
        except:
            pass
    
    def main_loop(self):
        """主循環"""
        print("[AutoFeeding] CG版本主循環啟動")
        print("[AutoFeeding] 特性：")
        print("  ✓ 主動監控當前執行Flow狀態 (1201)")
        print("  ✓ 當1201=1時暫停自動進料程序")
        print("  ✓ 持續檢測確保CG_F可用")
        print("  ✓ CG保護區域判斷")
        print("  ✓ Flow1直接讀取座標")
        
        # 自動啟動檢測
        auto_start = self.config['autofeeding'].get('auto_start', True)
        if auto_start:
            self.start_feeding()
        
        loop_count = 0
        
        while True:
            try:
                loop_count += 1
                
                # 定期打印狀態
                if loop_count % 200 == 1:
                    print(f"[DEBUG] 主循環 {loop_count}: running={self.running}, status={self.status.name}, flow1_active={self.flow1_active}, cg_f_available={self.cg_f_available}")
                
                # 檢查連接狀態
                if not self.connected:
                    print("[DEBUG] Modbus連接斷開，嘗試重連")
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                
                # 主動監控Flow1狀態
                self.check_flow1_status()
                
                # 檢查座標是否被讀取
                if self.cg_f_available:
                    self.check_coords_taken()
                
                # 更新狀態寄存器
                self.update_status_registers()
                
                # 執行入料檢測 - 只有在運行且Flow1未啟動時
                if self.running and not self.flow1_active and not self.vp_clearing_mode:
                    if not self.feeding_cycle():
                        print(f"[DEBUG] 入料檢測失敗，錯誤碼: {self.error_code}")
                        self.status = AutoFeedingStatus.ERROR
                        time.sleep(0.5)
                    else:
                        # 檢測成功，快速進入下一輪
                        cycle_interval = self.config['autofeeding']['cycle_interval']
                        time.sleep(cycle_interval)
                else:
                    # 非運行狀態或Flow1執行中，短間隔檢查
                    time.sleep(self.config['timing']['flow1_check_interval'])
                    
            except KeyboardInterrupt:
                print("\n[AutoFeeding] 收到中斷信號，準備退出")
                break
            except Exception as e:
                print(f"[ERROR] 主循環異常: {e}")
                time.sleep(1.0)
        
        # 清理資源
        self.stop_feeding()
        self.disconnect()
        print("[AutoFeeding] 程序已退出")


def main():
    """主程序入口"""
    print("=== CG版本AutoFeeding獨立模組啟動 ===")
    print("基地址範圍: 900-999")
    print("特性:")
    print("  ✓ 監控當前執行Flow地址(1201)")
    print("  ✓ 當1201=1時暫停自動進料程序")
    print("  ✓ 持續檢測保持CG_F可用")
    print("  ✓ CG保護區域判斷")
    print("  ✓ Flow1直接讀取座標(940-944)")
    print("  ✓ 自動啟動檢測")
    print("  ✓ 獨立模組設計")
    
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