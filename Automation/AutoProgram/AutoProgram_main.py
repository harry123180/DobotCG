#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_RobotJob_CG.py - CG入料+機械臂協調控制系統 (增強版)
雙執行緒架構：AutoFeeding執行緒 + RobotJob執行緒
基地址：1300-1349
新增功能：
1. 控制地址1350 - 流程總開關
2. 直振供料計數與清空流程
3. VP清空檢測與自動停止
4. CG_F/CG_B分類檢測支援
"""

import time
import math
import os
import json
import threading
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


class SystemStatus(Enum):
    """系統狀態"""
    STOPPED = 0
    AUTO_FEEDING_RUNNING = 1
    ROBOT_JOB_RUNNING = 2
    FLOW1_EXECUTING = 3
    FLOW5_EXECUTING = 4
    VP_CLEARING = 5  # 新增：VP清空中
    ERROR = 6


@dataclass
class CGDetectionResult:
    """CG檢測結果"""
    cg_f_count: int = 0
    total_detections: int = 0
    cg_f_world_coords: List[Tuple[float, float]] = None
    capture_success: bool = False
    detect_success: bool = False
    operation_success: bool = False
    
    def __post_init__(self):
        if self.cg_f_world_coords is None:
            self.cg_f_world_coords = []


class ProtectionZone:
    """保護區域判斷"""
    
    @staticmethod
    def is_point_in_quad(x_a: float, y_a: float) -> bool:
        """判斷點是否在保護區域四邊形內"""
        points = [
            (-86, -369.51),   # x1, y1
            (-112.82, -244.63),   # x2, y2
            (8.07, -244.64),  # x3, y3
            (8.06, -369.52)  # x4, y4
        ]
        
        # 找中心點，並以中心為基準對四點極角排序
        cx = sum(p[0] for p in points) / 4
        cy = sum(p[1] for p in points) / 4
        
        def angle(p):
            return math.atan2(p[1] - cy, p[0] - cx)
        
        sorted_points = sorted(points, key=angle)
        
        # 使用射線法檢查點是否在排序後四邊形內
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


class AutoFeedingThread:
    """自動入料執行緒"""
    
    def __init__(self, modbus_client: ModbusTcpClient, config: Dict):
        self.modbus_client = modbus_client
        self.config = config
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.protection_zone = ProtectionZone()
        
        # 模組基地址 - CG專案使用不同的基地址
        self.CG_BASE = 200  # 假設CG視覺模組使用基地址200
        self.VP_BASE = 300
        self.FLOW4_BASE = 448
        self.CONTROL_ADDRESS = 1350  # 新增：流程控制地址
        
        # 統計資訊
        self.cycle_count = 0
        self.cg_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        
        # 新增：直振供料與清空流程
        self.flow4_consecutive_count = 0  # 連續直振次數
        self.vp_clearing_mode = False     # VP清空模式
        self.vp_empty_detection_count = 0 # VP空檢測次數
        
        # 入料完成標誌
        self.feeding_ready = False
        self.feeding_ready_callback = None
        
        # 暫停標誌 (用於Flow1執行時暫停)
        self.pause_for_robot = False
        
    def set_feeding_ready_callback(self, callback):
        """設置入料完成回調函數"""
        self.feeding_ready_callback = callback
    
    def pause_for_robot_operation(self):
        """為機械臂作業暫停入料檢測"""
        print("[AutoFeeding] 為機械臂作業暫停入料檢測")
        self.pause_for_robot = True
        self.feeding_ready = False
    
    def resume_after_robot_operation(self):
        """機械臂作業完成後恢復入料檢測"""
        print("[AutoFeeding] 機械臂作業完成，恢復入料檢測")
        self.pause_for_robot = False
        self.feeding_ready = False
    
    def check_flow_control(self) -> bool:
        """檢查流程控制開關 (1350地址)"""
        try:
            control_value = self.read_register(self.CONTROL_ADDRESS)
            return control_value == 1
        except Exception:
            return False
    
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
    
    def clear_cg_registers(self) -> bool:
        """清空CG控制寄存器"""
        success = True
        success &= self.write_register(200, 0)  # CONTROL_COMMAND
        success &= self.write_register(203, 0)  # CAPTURE_COMPLETE
        success &= self.write_register(204, 0)  # DETECT_COMPLETE
        success &= self.write_register(205, 0)  # OPERATION_SUCCESS
        return success
    
    def check_modules_status(self) -> bool:
        """檢查VP、CG模組狀態"""
        print("[AutoFeeding] 檢查模組狀態...")
        
        # 檢查CG狀態
        cg_status = self.read_register(201)  # STATUS_REGISTER
        if cg_status is None:
            print("[AutoFeeding] ✗ CG模組無回應")
            return False
        
        cg_ready = bool(cg_status & 0x01)  # bit0=Ready
        cg_initialized = bool(cg_status & 0x08)  # bit3=Initialized
        
        print(f"[AutoFeeding] CG狀態: {cg_status:04b}, Ready={cg_ready}, Initialized={cg_initialized}")
        
        # 如果CG未準備就緒但已初始化，嘗試重置
        if not cg_ready and cg_initialized:
            print("[AutoFeeding] CG未Ready但已初始化，嘗試重置...")
            
            # 清零CG控制寄存器
            reset_success = True
            reset_success &= self.write_register(200, 0)  # CONTROL_COMMAND
            reset_success &= self.write_register(203, 0)  # CAPTURE_COMPLETE
            reset_success &= self.write_register(204, 0)  # DETECT_COMPLETE
            reset_success &= self.write_register(205, 0)  # OPERATION_SUCCESS
            
            if reset_success:
                print("[AutoFeeding] CG寄存器已清零，等待狀態更新...")
                time.sleep(0.5)  # 等待狀態更新
                
                # 重新檢查CG狀態
                cg_status = self.read_register(201)
                if cg_status is not None:
                    cg_ready = bool(cg_status & 0x01)
                    cg_initialized = bool(cg_status & 0x08)
                    print(f"[AutoFeeding] CG重置後狀態: {cg_status:04b}, Ready={cg_ready}, Initialized={cg_initialized}")
                else:
                    print("[AutoFeeding] ✗ CG重置後無回應")
                    return False
            else:
                print("[AutoFeeding] ✗ CG寄存器重置失敗")
        
        if not (cg_ready and cg_initialized):
            print(f"[AutoFeeding] ✗ CG模組未準備就緒 (Ready={cg_ready}, Initialized={cg_initialized})")
            return False
        
        # 檢查VP狀態
        vp_status = self.read_register(300)  # module_status
        vp_connected = self.read_register(301)  # device_connection
        
        print(f"[AutoFeeding] VP狀態: module_status={vp_status}, device_connection={vp_connected}")
        
        if vp_status != 1:
            print(f"[AutoFeeding] ✗ VP模組狀態異常: {vp_status}")
            return False
        
        if vp_connected != 1:
            print(f"[AutoFeeding] ✗ VP設備未連接: {vp_connected}")
            return False
        
        print("[AutoFeeding] ✓ 所有模組狀態正常")
        return True
    
    def trigger_cg_detection(self) -> CGDetectionResult:
        """觸發CG拍照檢測並獲取結果"""
        print("[AutoFeeding] 觸發CG拍照檢測...")
        
        result = CGDetectionResult()
        
        # 向200地址寫入16觸發拍照+檢測
        if not self.write_register(200, 16):
            print("[AutoFeeding] ✗ CG指令寫入失敗")
            return result
        
        print("[AutoFeeding] CG指令已發送，等待檢測完成...")
        
        # 等待拍照、檢測、操作完成標誌
        timeout = self.config['auto_program']['cg_timeout']
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            capture_complete = self.read_register(203)  # CAPTURE_COMPLETE
            detect_complete = self.read_register(204)   # DETECT_COMPLETE  
            operation_success = self.read_register(205) # OPERATION_SUCCESS
            
            # 每秒輸出一次等待狀態
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0:
                print(f"[AutoFeeding] 等待CG檢測... {elapsed:.1f}s (capture={capture_complete}, detect={detect_complete}, success={operation_success})")
            
            if capture_complete == 1 and detect_complete == 1 and operation_success == 1:
                result.capture_success = True
                result.detect_success = True
                result.operation_success = True
                print("[AutoFeeding] ✓ CG檢測完成")
                break
            
            time.sleep(self.config['timing']['status_check_interval'])
        
        if not result.operation_success:
            elapsed = time.time() - start_time
            print(f"[AutoFeeding] ✗ CG檢測超時或失敗 (耗時{elapsed:.1f}s)")
            return result
        
        # 讀取檢測結果 - 適應CG專案的寄存器映射
        result.cg_f_count = self.read_register(240) or 0      # CG_F_COUNT
        result.total_detections = self.read_register(243) or 0  # TOTAL_DETECTIONS
        
        print(f"[AutoFeeding] CG檢測結果: CG_F={result.cg_f_count}, 總檢測={result.total_detections}")
        
        # 提取CG_F世界座標
        if result.cg_f_count > 0:
            max_check = min(result.cg_f_count, self.config['auto_program']['max_cg_f_check'])
            
            for i in range(max_check):
                base_addr = 261 + (i * 4)  # 每個物件佔4個寄存器
                world_x = self.read_32bit_register(base_addr, base_addr + 1)
                world_y = self.read_32bit_register(base_addr + 2, base_addr + 3)
                result.cg_f_world_coords.append((world_x, world_y))
                print(f"[AutoFeeding] CG_F{i+1}座標: ({world_x:.2f}, {world_y:.2f})")
        
        return result
    
    def find_cg_f_in_protection_zone(self, detection_result: CGDetectionResult) -> Optional[Tuple[float, float]]:
        """尋找保護區域內的CG_F物件"""
        if detection_result.cg_f_count == 0:
            return None
        
        for i, (world_x, world_y) in enumerate(detection_result.cg_f_world_coords):
            if self.protection_zone.is_point_in_quad(world_x, world_y):
                return (world_x, world_y)
        
        return None
    
    def update_first_cg_f_coordinates(self, target_coords: Tuple[float, float]) -> bool:
        """將目標座標覆蓋到第一個CG_F位置(261-264)"""
        world_x, world_y = target_coords
        
        # 轉換為整數形式(×100)
        world_x_int = int(world_x * 100)
        world_y_int = int(world_y * 100)
        
        # 處理負數(補碼)
        if world_x_int < 0:
            world_x_int = world_x_int + 4294967296  # 2^32
        if world_y_int < 0:
            world_y_int = world_y_int + 4294967296  # 2^32
        
        # 分解為高低位
        x_high = (world_x_int >> 16) & 0xFFFF
        x_low = world_x_int & 0xFFFF
        y_high = (world_y_int >> 16) & 0xFFFF
        y_low = world_y_int & 0xFFFF
        
        # 寫入寄存器261-264
        success = True
        success &= self.write_register(261, x_high)  # CG_F_1_WORLD_X_HIGH
        success &= self.write_register(262, x_low)   # CG_F_1_WORLD_X_LOW
        success &= self.write_register(263, y_high)  # CG_F_1_WORLD_Y_HIGH
        success &= self.write_register(264, y_low)   # CG_F_1_WORLD_Y_LOW
        
        return success
    
    def stop_vp_vibration(self) -> bool:
        """停止VP震動盤"""
        success = True
        success &= self.write_register(320, 3)  # stop_all指令
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, 99)  # emergency stop id
        
        if success:
            time.sleep(self.config['auto_program']['vp_stop_delay'])
        
        return success
    
    def trigger_vp_vibration_and_redetect(self) -> Optional[Tuple[float, float]]:
        """觸發VP震動並重新檢測CG_F"""
        # VP指令參數
        command_code = 5  # execute_action
        action_code = self.config['vp_params']['spread_action_code']  # spread=11
        strength = self.config['vp_params']['spread_strength']        # 50
        frequency = self.config['vp_params']['spread_frequency']      # 43
        duration = self.config['vp_params']['spread_duration']        # 0.5秒
        
        # 啟動VP震動
        success = True
        success &= self.write_register(320, command_code)
        success &= self.write_register(321, action_code)
        success &= self.write_register(322, strength)
        success &= self.write_register(323, frequency)
        success &= self.write_register(324, int(time.time()) % 65535)  # command_id
        
        if not success:
            return None
        
        # 等待震動
        time.sleep(duration)
        
        # 停止震動
        if not self.stop_vp_vibration():
            return None
        
        # 等待穩定
        time.sleep(0.3)
        
        # 重新檢測
        detection_result = self.trigger_cg_detection()
        
        if not detection_result.operation_success:
            return None
        
        target_coords = self.find_cg_f_in_protection_zone(detection_result)
        
        if target_coords:
            self.cg_f_found_count += 1
            return target_coords
        
        return None
    
    def trigger_flow4_feeding(self) -> bool:
        """觸發Flow4送料"""
        if not self.write_register(448, 1):
            return False
        
        time.sleep(self.config['auto_program']['flow4_pulse_duration'])
        
        if not self.write_register(448, 0):
            return False
        
        return True
    
    def trigger_vp_clear_vibration(self) -> bool:
        """觸發VP清空震動 - 翻正所有料件"""
        print("[AutoFeeding] 觸發VP清空震動 (spread強度50頻率43)")
        
        command_code = 5   # execute_action
        action_code = 11   # spread
        strength = 50      # 強度50
        frequency = 43     # 頻率43
        duration = 0.5     # 持續0.5秒
        
        # 啟動VP震動
        success = True
        success &= self.write_register(320, command_code)
        success &= self.write_register(321, action_code)
        success &= self.write_register(322, strength)
        success &= self.write_register(323, frequency)
        success &= self.write_register(324, int(time.time()) % 65535)  # command_id
        
        if not success:
            print("[AutoFeeding] ✗ VP清空震動指令發送失敗")
            return False
        
        # 等待震動完成
        time.sleep(duration)
        
        # 停止震動
        if not self.stop_vp_vibration():
            print("[AutoFeeding] ✗ VP清空震動停止失敗")
            return False
        
        print("[AutoFeeding] ✓ VP清空震動完成")
        return True
    
    def check_vp_empty_condition(self) -> bool:
        """檢查VP是否為空 (總檢測數為0)"""
        detection_result = self.trigger_cg_detection()
        
        if not detection_result.operation_success:
            print("[AutoFeeding] ✗ VP空檢測失敗")
            return False
        
        is_empty = (detection_result.total_detections == 0)
        print(f"[AutoFeeding] VP空檢測: 總檢測數={detection_result.total_detections}, 是否為空={is_empty}")
        
        return is_empty
    
    def execute_vp_clearing_process(self) -> bool:
        """執行VP清空流程"""
        print("\n[AutoFeeding] === 開始VP清空流程 ===")
        print("[AutoFeeding] 料桶無料，執行VP清空：翻正所有料件 + Flow1清空")
        
        self.vp_clearing_mode = True
        self.vp_empty_detection_count = 0
        
        try:
            # 步驟1：VP震動翻正所有料件
            if not self.trigger_vp_clear_vibration():
                print("[AutoFeeding] ✗ VP清空震動失敗")
                return False
            
            # 步驟2：重複執行Flow1直到VP空
            flow1_attempts = 0
            max_flow1_attempts = 20  # 防止無限循環
            
            while flow1_attempts < max_flow1_attempts:
                flow1_attempts += 1
                print(f"\n[AutoFeeding] Flow1清空嘗試 {flow1_attempts}/{max_flow1_attempts}")
                
                # 觸發Flow1
                if not self.write_register(1240, 1):  # FLOW1_CONTROL
                    print("[AutoFeeding] ✗ Flow1觸發失敗")
                    break
                
                # 等待Flow1完成 (最多30秒)
                flow1_timeout = 30.0
                start_time = time.time()
                flow1_completed = False
                
                while (time.time() - start_time) < flow1_timeout:
                    flow1_status = self.read_register(1204)  # FLOW1_COMPLETE
                    if flow1_status == 1:
                        flow1_completed = True
                        print(f"[AutoFeeding] ✓ Flow1清空完成 (耗時{time.time() - start_time:.1f}s)")
                        break
                    time.sleep(0.5)
                
                if not flow1_completed:
                    print("[AutoFeeding] ✗ Flow1清空超時")
                    break
                
                # 清除Flow1控制狀態
                self.write_register(1240, 0)
                time.sleep(0.5)
                
                # VP震動 + 檢測VP是否為空
                self.trigger_vp_clear_vibration()
                time.sleep(0.5)  # 等待0.5秒
                
                if self.check_vp_empty_condition():
                    self.vp_empty_detection_count += 1
                    print(f"[AutoFeeding] VP空檢測 {self.vp_empty_detection_count}/5")
                    
                    if self.vp_empty_detection_count >= 5:
                        print("[AutoFeeding] ✓ VP連續5次檢測為空，清空流程完成")
                        
                        # 清空流程完成，停止AutoProgram
                        print("[AutoFeeding] 設置控制地址1350=0，停止AutoProgram")
                        self.write_register(self.CONTROL_ADDRESS, 0)
                        
                        return True
                else:
                    self.vp_empty_detection_count = 0  # 重置空檢測計數
                
                time.sleep(1.0)  # 等待下一輪
            
            print(f"[AutoFeeding] ✗ VP清空流程失敗 (Flow1嘗試{flow1_attempts}次)")
            return False
            
        except Exception as e:
            print(f"[AutoFeeding] ✗ VP清空流程異常: {e}")
            return False
        
        finally:
            self.vp_clearing_mode = False
            print("[AutoFeeding] === VP清空流程結束 ===\n")
    
    def auto_feeding_cycle(self) -> bool:
        """執行一次自動入料週期 (增強版)"""
        try:
            # 檢查流程控制開關
            if not self.check_flow_control():
                print("[AutoFeeding] 流程控制開關關閉 (1350=0)，跳過週期")
                return False
            
            self.cycle_count += 1
            print(f"\n[AutoFeeding] === 週期 {self.cycle_count} 開始 ===")
            
            # 檢查模組狀態
            if not self.check_modules_status():
                print(f"[AutoFeeding] 週期 {self.cycle_count} 跳過: 模組狀態檢查失敗")
                return False
            
            # 觸發CG檢測
            detection_result = self.trigger_cg_detection()
            
            if not detection_result.operation_success:
                print(f"[AutoFeeding] 週期 {self.cycle_count} 跳過: CG檢測失敗")
                return False
            
            # 尋找保護區域內的CG_F
            target_coords = self.find_cg_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 找到保護區域內的CG_F
                self.cg_f_found_count += 1
                self.flow4_consecutive_count = 0  # 重置連續直振計數
                
                if self.update_first_cg_f_coordinates(target_coords):
                    print(f"[AutoFeeding] ✓ 找到CG_F在保護區域: {target_coords}")
                    self.feeding_ready = True
                    # 通知RobotJob
                    if self.feeding_ready_callback:
                        self.feeding_ready_callback()
                else:
                    print("[AutoFeeding] ✗ CG_F座標更新失敗")
            else:
                # 沒有CG_F在保護區域內
                if detection_result.total_detections < 4:
                    # 料件不足，觸發Flow4送料
                    if self.trigger_flow4_feeding():
                        self.flow4_trigger_count += 1
                        self.flow4_consecutive_count += 1  # 增加連續直振計數
                        
                        print(f"[AutoFeeding] 料件不足，觸發Flow4送料 (總檢測={detection_result.total_detections})")
                        print(f"[AutoFeeding] 連續直振次數: {self.flow4_consecutive_count}/5")
                        
                        # 檢查是否達到連續直振5次
                        if self.flow4_consecutive_count >= 5:
                            print("[AutoFeeding] 連續直振5次，料桶無料，開始VP清空流程")
                            self.execute_vp_clearing_process()
                            return True  # 清空流程會自動停止系統
                
                elif detection_result.total_detections >= 4:
                    # 料件充足但沒有CG_F在保護區，震動散開並重新檢測
                    print(f"[AutoFeeding] 料件充足但無CG_F在保護區 (總檢測={detection_result.total_detections})")
                    self.flow4_consecutive_count = 0  # 重置連續直振計數
                    
                    target_coords_after_vp = self.trigger_vp_vibration_and_redetect()
                    if target_coords_after_vp:
                        # 震動後找到CG_F，更新座標
                        if self.update_first_cg_f_coordinates(target_coords_after_vp):
                            print(f"[AutoFeeding] ✓ 震動後找到CG_F: {target_coords_after_vp}")
                            self.feeding_ready = True
                            # 通知RobotJob
                            if self.feeding_ready_callback:
                                self.feeding_ready_callback()
                        self.vp_vibration_count += 1
                    else:
                        print("[AutoFeeding] 震動後仍未找到保護區域內的CG_F")
                        self.vp_vibration_count += 1
            
            # 清空CG寄存器
            self.clear_cg_registers()
            time.sleep(self.config['timing']['register_clear_delay'])
            
            print(f"[AutoFeeding] 週期 {self.cycle_count} 完成")
            return True
            
        except Exception as e:
            print(f"[AutoFeeding] 週期 {self.cycle_count} 異常: {e}")
            return False
    
    def start(self):
        """啟動自動入料執行緒"""
        if self.running:
            return
        
        print("[AutoFeeding] 啟動自動入料執行緒")
        self.running = True
        self.cycle_count = 0
        self.cg_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        self.flow4_consecutive_count = 0  # 重置連續直振計數
        self.vp_clearing_mode = False
        self.vp_empty_detection_count = 0
        self.feeding_ready = False
        self.pause_for_robot = False
        
        self.thread = threading.Thread(target=self._auto_feeding_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """停止自動入料執行緒"""
        if not self.running:
            return
        
        print("[AutoFeeding] 停止自動入料執行緒")
        self.running = False
        self.feeding_ready = False
        
        # 緊急停止VP
        try:
            self.stop_vp_vibration()
        except:
            pass
        
        # 只有在外部執行緒調用時才join，避免自己等待自己
        if self.thread and self.thread.is_alive() and threading.current_thread() != self.thread:
            self.thread.join(timeout=2.0)
        elif threading.current_thread() == self.thread:
            print("[AutoFeeding] 執行緒內部停止，跳過join操作")
    
    def _auto_feeding_loop(self):
        """自動入料主循環 (增強版)"""
        cycle_interval = self.config['auto_program']['cycle_interval']
        
        while self.running:
            try:
                # 檢查流程控制開關 (100ms高頻輪詢)
                if not self.check_flow_control():
                    print("[AutoFeeding] 流程控制關閉，等待重新啟動...")
                    time.sleep(0.1)  # 100ms高頻輪詢
                    continue
                
                # 檢查是否需要暫停
                if self.pause_for_robot:
                    print("[AutoFeeding] 已暫停，等待機械臂作業完成...")
                    time.sleep(0.5)
                    continue
                
                # 如果在VP清空模式，不執行正常檢測
                if self.vp_clearing_mode:
                    print("[AutoFeeding] VP清空模式中，跳過正常檢測...")
                    time.sleep(1.0)
                    continue
                
                # 只有在feeding_ready=False時才執行檢測
                if not self.feeding_ready:
                    self.auto_feeding_cycle()
                
                time.sleep(cycle_interval)
                
            except Exception as e:
                print(f"[AutoFeeding] 循環異常: {e}")
                time.sleep(1.0)


class RobotJobThread:
    """機械臂作業執行緒"""
    
    def __init__(self, modbus_client: ModbusTcpClient, config: Dict):
        self.modbus_client = modbus_client
        self.config = config
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # 狀態變數
        self.prepare_done = False
        
        # AutoFeeding執行緒參考 (用於重啟)
        self.auto_feeding_ref = None
        
        # 模組基地址
        self.FLOW1_CONTROL = 1240  # Flow1控制
        self.FLOW1_STATUS = 1204   # Flow1完成狀態
        self.FLOW5_STATUS = 1206   # Flow5完成狀態
        self.CONTROL_ADDRESS = 1350  # 流程控制地址
        
        # 統計資訊
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        
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
    
    def check_flow_control(self) -> bool:
        """檢查流程控制開關 (1350地址)"""
        try:
            control_value = self.read_register(self.CONTROL_ADDRESS)
            return control_value == 1
        except Exception:
            return False
    
    def on_feeding_ready(self):
        """自動入料完成回調"""
        print("[RobotJob] 收到自動入料完成通知")
        
        # 檢查流程控制開關
        if not self.check_flow_control():
            print("[RobotJob] 流程控制關閉，忽略入料完成通知")
            return
        
        # 只要prepare_done=False，就觸發Flow1
        if not self.prepare_done:
            print("[RobotJob] prepare_done=False，觸發Flow1去VP拿料")
            
            # 使用暫停標誌而不是停止執行緒
            if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                print("[RobotJob] 暫停AutoFeeding檢測避免干擾Flow1")
                self.auto_feeding_ref.pause_for_robot_operation()
            
            if self.write_register(self.FLOW1_CONTROL, 1):
                self.flow1_trigger_count += 1
                print("[RobotJob] Flow1觸發成功")
            else:
                print("[RobotJob] Flow1觸發失敗")
        else:
            # prepare_done=True，表示機台已準備好，等待出料指令
            print(f"[RobotJob] prepare_done=True，機台已準備好接受出料指令")
    
    def robot_job_cycle(self):
        """機械臂作業週期 (增強版)"""
        try:
            # 檢查流程控制開關
            if not self.check_flow_control():
                return
            
            # 檢查機械臂是否Ready (1200=9)
            robot_status = self.read_register(1200)  # 機械臂狀態寄存器
            robot_ready = (robot_status == 9) if robot_status is not None else False
            
            # 檢查Flow1完成狀態 (但不清空，供其他模組使用)
            flow1_status = self.read_register(self.FLOW1_STATUS)
            if flow1_status == 1:
                print("[RobotJob] 檢測到Flow1完成")
                
                # Flow1完成，設置prepare_done=True
                self.prepare_done = True
                
                # 恢復AutoFeeding檢測
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    print("[RobotJob] Flow1完成，恢復AutoFeeding檢測")
                    self.auto_feeding_ref.resume_after_robot_operation()
                
                # 只清空Flow1控制狀態，保持Flow1完成狀態供其他模組使用
                self.write_register(self.FLOW1_CONTROL, 0)
                print("[RobotJob] Flow1控制狀態已清零，prepare_done=True，等待其他模組執行Flow5")
            
            # 檢查Flow5完成狀態
            flow5_status = self.read_register(self.FLOW5_STATUS)
            if flow5_status == 1:
                print("[RobotJob] 檢測到Flow5完成")
                
                # Flow5完成，重置prepare_done=False，準備下一輪
                self.prepare_done = False
                self.flow5_complete_count += 1
                
                # 重置AutoFeeding狀態開始新的週期
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    print("[RobotJob] Flow5完成，重置AutoFeeding狀態開始新週期")
                    # 重置所有狀態
                    self.auto_feeding_ref.feeding_ready = False
                    self.auto_feeding_ref.pause_for_robot = False
                
                # 清除Flow5完成狀態
                self.write_register(self.FLOW5_STATUS, 0)
                print("[RobotJob] Flow5完成狀態已清零，prepare_done=False，系統準備新週期")
            
            # 核心邏輯：當prepare_done=False且機械臂Ready時，執行AutoFeeding+Flow1
            if not self.prepare_done and robot_ready:
                print(f"[RobotJob] 系統條件滿足：prepare_done=False, 機械臂Ready={robot_ready}")
                print("[RobotJob] 開始AutoFeeding+Flow1週期")
                
                # 確保AutoFeeding正在運行且未暫停
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    if not self.auto_feeding_ref.running:
                        print("[RobotJob] 啟動AutoFeeding執行緒")
                        self.auto_feeding_ref.start()
                    
                    if self.auto_feeding_ref.pause_for_robot:
                        print("[RobotJob] 恢復AutoFeeding檢測")
                        self.auto_feeding_ref.resume_after_robot_operation()
        
        except Exception as e:
            print(f"[RobotJob] 機械臂作業週期異常: {e}")
    
    def start(self):
        """啟動機械臂作業執行緒"""
        if self.running:
            return
        
        print("[RobotJob] 啟動機械臂作業執行緒")
        self.running = True
        self.prepare_done = False
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        
        # 初始化Flow1完成狀態為0
        self.write_register(self.FLOW1_STATUS, 0)
        
        self.thread = threading.Thread(target=self._robot_job_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """停止機械臂作業執行緒"""
        if not self.running:
            return
        
        print("[RobotJob] 停止機械臂作業執行緒")
        self.running = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
    
    def _robot_job_loop(self):
        """機械臂作業主循環"""
        while self.running:
            try:
                self.robot_job_cycle()
                time.sleep(0.5)  # 0.5秒檢查間隔
                
            except Exception as e:
                print(f"[RobotJob] 循環異常: {e}")
                time.sleep(1.0)


class AutoProgramRobotJobController:
    """CG入料+機械臂協調控制系統 (增強版)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址 1300-1349
        self.base_address = 1300
        
        # 載入配置
        self.config = self.load_config()
        
        # 建立執行緒
        self.auto_feeding_thread = None
        self.robot_job_thread = None
        
        # 系統狀態
        self.system_status = SystemStatus.STOPPED
        
        print("CG入料+機械臂協調控制系統初始化完成 (增強版)")
        print(f"Modbus服務器: {modbus_host}:{modbus_port}")
        print(f"系統基地址: {self.base_address}")
        print(f"流程控制地址: 1350")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "auto_program": {
                "cycle_interval": 2.0,
                "cg_timeout": 10.0,
                "vp_vibration_duration": 0.5,
                "vp_stop_delay": 0.2,
                "flow4_pulse_duration": 0.1,
                "max_cg_f_check": 5,
                "flow4_consecutive_limit": 5,      # 新增：連續直振限制
                "vp_empty_check_count": 5          # 新增：VP空檢測計數限制
            },
            "vp_params": {
                "spread_action_code": 11,
                "spread_strength": 50,
                "spread_frequency": 43,
                "spread_duration": 0.5,
                "stop_command_code": 3,
                "clear_strength": 50,              # 新增：清空震動強度
                "clear_frequency": 43              # 新增：清空震動頻率
            },
            "timing": {
                "command_delay": 0.1,
                "status_check_interval": 0.1,
                "register_clear_delay": 0.05,
                "flow_control_poll_interval": 0.1  # 新增：流程控制輪詢間隔
            },
            "modbus_mapping": {
                "base_address": 1300,
                "control_address": 1350            # 新增：控制地址
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autoprogram_robotjob_cg_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            print(f"配置檔案處理失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_system_registers()
            else:
                print(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def init_system_registers(self):
        """初始化系統寄存器"""
        try:
            # 1300: 系統狀態
            # 1301: AutoFeeding執行緒狀態 
            # 1302: RobotJob執行緒狀態
            # 1303: 系統錯誤代碼
            # 1304: AutoFeeding週期計數
            # 1305: CG_F找到次數
            # 1306: Flow4觸發次數  
            # 1307: VP震動次數
            # 1308: Flow1觸發次數
            # 1309: Flow5完成次數
            # 1310: 連續直振次數       # 新增
            # 1311: VP空檢測次數       # 新增
            # 1350: 流程控制開關       # 新增
            
            self.modbus_client.write_register(1300, SystemStatus.STOPPED.value, slave=1)
            self.modbus_client.write_register(1301, 0, slave=1)  # AutoFeeding停止
            self.modbus_client.write_register(1302, 0, slave=1)  # RobotJob停止
            self.modbus_client.write_register(1303, 0, slave=1)  # 無錯誤
            self.modbus_client.write_register(1310, 0, slave=1)  # 連續直振次數
            self.modbus_client.write_register(1311, 0, slave=1)  # VP空檢測次數
            self.modbus_client.write_register(1350, 0, slave=1)  # 流程控制關閉
            print("系統寄存器初始化完成")
        except Exception as e:
            print(f"系統寄存器初始化失敗: {e}")
    
    def update_system_registers(self):
        """更新系統寄存器"""
        try:
            if not self.connected:
                return
            
            # 更新系統狀態
            self.modbus_client.write_register(1300, self.system_status.value, slave=1)
            
            # 更新執行緒狀態
            auto_feeding_running = 1 if (self.auto_feeding_thread and self.auto_feeding_thread.running) else 0
            robot_job_running = 1 if (self.robot_job_thread and self.robot_job_thread.running) else 0
            
            self.modbus_client.write_register(1301, auto_feeding_running, slave=1)
            self.modbus_client.write_register(1302, robot_job_running, slave=1)
            
            # 更新統計資訊
            if self.auto_feeding_thread:
                self.modbus_client.write_register(1304, self.auto_feeding_thread.cycle_count, slave=1)
                self.modbus_client.write_register(1305, self.auto_feeding_thread.cg_f_found_count, slave=1)
                self.modbus_client.write_register(1306, self.auto_feeding_thread.flow4_trigger_count, slave=1)
                self.modbus_client.write_register(1307, self.auto_feeding_thread.vp_vibration_count, slave=1)
                self.modbus_client.write_register(1310, self.auto_feeding_thread.flow4_consecutive_count, slave=1)  # 新增
                self.modbus_client.write_register(1311, self.auto_feeding_thread.vp_empty_detection_count, slave=1)  # 新增
            
            if self.robot_job_thread:
                self.modbus_client.write_register(1308, self.robot_job_thread.flow1_trigger_count, slave=1)
                self.modbus_client.write_register(1309, self.robot_job_thread.flow5_complete_count, slave=1)
                
        except Exception as e:
            print(f"系統寄存器更新失敗: {e}")
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def start_system(self):
        """啟動協調控制系統"""
        if not self.connected:
            print("請先連接Modbus服務器")
            return
        
        print("=== 啟動CG入料+機械臂協調控制系統 (增強版) ===")
        
        # 建立AutoFeeding執行緒
        self.auto_feeding_thread = AutoFeedingThread(self.modbus_client, self.config)
        
        # 建立RobotJob執行緒
        self.robot_job_thread = RobotJobThread(self.modbus_client, self.config)
        
        # 設置執行緒間的相互參考
        self.robot_job_thread.auto_feeding_ref = self.auto_feeding_thread
        
        # 設置回調連接
        self.auto_feeding_thread.set_feeding_ready_callback(self.robot_job_thread.on_feeding_ready)
        
        # 啟動執行緒
        self.auto_feeding_thread.start()
        self.robot_job_thread.start()
        
        # 更新系統狀態
        self.system_status = SystemStatus.AUTO_FEEDING_RUNNING
        
        print("協調控制系統已啟動")
        print("流程控制: 寫入1350=1啟動流程，1350=0停止流程")
    
    def stop_system(self):
        """停止協調控制系統"""
        print("=== 停止CG入料+機械臂協調控制系統 ===")
        
        self.system_status = SystemStatus.STOPPED
        
        # 停止執行緒
        if self.auto_feeding_thread:
            self.auto_feeding_thread.stop()
        
        if self.robot_job_thread:
            self.robot_job_thread.stop()
        
        # 更新系統寄存器
        self.update_system_registers()
        
        print("協調控制系統已停止")
        self.print_statistics()
    
    def enable_flow_control(self):
        """啟動流程控制 (1350=1)"""
        if self.connected:
            try:
                self.modbus_client.write_register(1350, 1, slave=1)
                print("流程控制已啟動 (1350=1)")
            except Exception as e:
                print(f"流程控制啟動失敗: {e}")
    
    def disable_flow_control(self):
        """停止流程控制 (1350=0)"""
        if self.connected:
            try:
                self.modbus_client.write_register(1350, 0, slave=1)
                print("流程控制已停止 (1350=0)")
            except Exception as e:
                print(f"流程控制停止失敗: {e}")
    
    def print_statistics(self):
        """輸出統計資訊 (增強版)"""
        print(f"\n=== CG協調控制系統統計 (增強版) ===")
        
        if self.auto_feeding_thread:
            print(f"AutoFeeding統計:")
            print(f"  總週期數: {self.auto_feeding_thread.cycle_count}")
            print(f"  CG_F找到次數: {self.auto_feeding_thread.cg_f_found_count}")
            print(f"  Flow4觸發次數: {self.auto_feeding_thread.flow4_trigger_count}")
            print(f"  VP震動次數: {self.auto_feeding_thread.vp_vibration_count}")
            print(f"  連續直振次數: {self.auto_feeding_thread.flow4_consecutive_count}")  # 新增
            print(f"  VP空檢測次數: {self.auto_feeding_thread.vp_empty_detection_count}")  # 新增
            print(f"  VP清空模式: {'是' if self.auto_feeding_thread.vp_clearing_mode else '否'}")  # 新增
            if self.auto_feeding_thread.cycle_count > 0:
                success_rate = (self.auto_feeding_thread.cg_f_found_count / self.auto_feeding_thread.cycle_count) * 100
                print(f"  CG_F找到率: {success_rate:.1f}%")
        
        if self.robot_job_thread:
            print(f"RobotJob統計:")
            print(f"  Flow1觸發次數: {self.robot_job_thread.flow1_trigger_count}")
            print(f"  Flow5完成次數: {self.robot_job_thread.flow5_complete_count}")
    
    def get_status_info(self) -> Dict[str, Any]:
        """獲取狀態資訊 (增強版)"""
        # 讀取流程控制開關狀態
        flow_control_enabled = False
        if self.connected:
            try:
                result = self.modbus_client.read_holding_registers(1350, count=1, slave=1)
                if not result.isError():
                    flow_control_enabled = result.registers[0] == 1
            except:
                pass
        
        status = {
            "connected": self.connected,
            "system_status": self.system_status.name,
            "flow_control_enabled": flow_control_enabled,  # 新增
            "auto_feeding_running": self.auto_feeding_thread.running if self.auto_feeding_thread else False,
            "robot_job_running": self.robot_job_thread.running if self.robot_job_thread else False
        }
        
        if self.auto_feeding_thread:
            status.update({
                "auto_feeding_cycle_count": self.auto_feeding_thread.cycle_count,
                "cg_f_found_count": self.auto_feeding_thread.cg_f_found_count,
                "flow4_trigger_count": self.auto_feeding_thread.flow4_trigger_count,
                "vp_vibration_count": self.auto_feeding_thread.vp_vibration_count,
                "flow4_consecutive_count": self.auto_feeding_thread.flow4_consecutive_count,  # 新增
                "vp_clearing_mode": self.auto_feeding_thread.vp_clearing_mode,              # 新增
                "vp_empty_detection_count": self.auto_feeding_thread.vp_empty_detection_count,  # 新增
                "feeding_ready": self.auto_feeding_thread.feeding_ready
            })
        
        if self.robot_job_thread:
            status.update({
                "prepare_done": self.robot_job_thread.prepare_done,
                "flow1_trigger_count": self.robot_job_thread.flow1_trigger_count,
                "flow5_complete_count": self.robot_job_thread.flow5_complete_count
            })
        
        return status


def main():
    """主程序"""
    print("CG入料+機械臂協調控制系統啟動 (增強版)")
    
    # 創建控制器
    controller = AutoProgramRobotJobController()
    
    # 連接Modbus
    if not controller.connect():
        print("Modbus連接失敗，程序退出")
        return
    
    try:
        # 啟動協調控制系統
        controller.start_system()
        
        # 定期更新系統寄存器
        def update_registers():
            while True:
                controller.update_system_registers()
                time.sleep(2.0)
        
        update_thread = threading.Thread(target=update_registers, daemon=True)
        update_thread.start()
        
        # 主循環 - 等待用戶操作
        print("\n指令說明:")
        print("  s - 顯示狀態")
        print("  r - 重啟系統")
        print("  start - 啟動流程控制 (1350=1)")
        print("  stop - 停止流程控制 (1350=0)")
        print("  pause - 暫停自動入料")
        print("  resume - 恢復自動入料")
        print("  q - 退出程序")
        
        while True:
            try:
                cmd = input("\n請輸入指令: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    status = controller.get_status_info()
                    print(f"\n系統狀態:")
                    for key, value in status.items():
                        print(f"  {key}: {value}")
                elif cmd == 'r':
                    controller.stop_system()
                    time.sleep(1.0)
                    controller.start_system()
                elif cmd == 'start':
                    controller.enable_flow_control()
                elif cmd == 'stop':
                    controller.disable_flow_control()
                elif cmd == 'pause':
                    if controller.auto_feeding_thread:
                        controller.auto_feeding_thread.pause_for_robot_operation()
                elif cmd == 'resume':
                    if controller.auto_feeding_thread:
                        controller.auto_feeding_thread.resume_after_robot_operation()
                else:
                    print("無效指令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    finally:
        # 清理資源
        controller.stop_system()
        controller.disconnect()
        print("程序已退出")


if __name__ == "__main__":
    main()#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_RobotJob_CG.py - CG入料+機械臂協調控制系統 (修正版)
修正Flow1暫停時序問題：
1. Flow1觸發前就先暫停AutoFeeding
2. Flow1完成後恢復AutoFeeding持續供料
3. Flow5觸發前再次暫停AutoFeeding
4. Flow5完成後恢復AutoFeeding
"""

import time
import math
import os
import json
import threading
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


class SystemStatus(Enum):
    """系統狀態"""
    STOPPED = 0
    AUTO_FEEDING_RUNNING = 1
    ROBOT_JOB_RUNNING = 2
    FLOW1_EXECUTING = 3
    FLOW5_EXECUTING = 4
    VP_CLEARING = 5
    ERROR = 6


@dataclass
class CGDetectionResult:
    """CG檢測結果"""
    cg_f_count: int = 0
    total_detections: int = 0
    cg_f_world_coords: List[Tuple[float, float]] = None
    capture_success: bool = False
    detect_success: bool = False
    operation_success: bool = False
    
    def __post_init__(self):
        if self.cg_f_world_coords is None:
            self.cg_f_world_coords = []


class ProtectionZone:
    """保護區域判斷"""
    
    @staticmethod
    def is_point_in_quad(x_a: float, y_a: float) -> bool:
        """判斷點是否在保護區域四邊形內"""
        points = [
            (-86, -369.51),   # x1, y1
            (-112.82, -244.63),   # x2, y2
            (8.07, -244.64),  # x3, y3
            (8.06, -369.52)  # x4, y4
        ]
        
        # 找中心點，並以中心為基準對四點極角排序
        cx = sum(p[0] for p in points) / 4
        cy = sum(p[1] for p in points) / 4
        
        def angle(p):
            return math.atan2(p[1] - cy, p[0] - cx)
        
        sorted_points = sorted(points, key=angle)
        
        # 使用射線法檢查點是否在排序後四邊形內
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


class AutoFeedingThread:
    """自動入料執行緒 - 修正版"""
    
    def __init__(self, modbus_client: ModbusTcpClient, config: Dict):
        self.modbus_client = modbus_client
        self.config = config
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.protection_zone = ProtectionZone()
        
        # 模組基地址
        self.CG_BASE = 200
        self.VP_BASE = 300
        self.FLOW4_BASE = 448
        self.CONTROL_ADDRESS = 1350
        
        # 統計資訊
        self.cycle_count = 0
        self.cg_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        
        # 直振供料與清空流程
        self.flow4_consecutive_count = 0
        self.vp_clearing_mode = False
        self.vp_empty_detection_count = 0
        
        # 入料完成標誌
        self.feeding_ready = False
        self.feeding_ready_callback = None
        
        # === 修正版：更精確的暫停控制 ===
        self.pause_for_robot = False  # 暫停標誌
        self.pause_reason = ""        # 暫停原因記錄
        
    def set_feeding_ready_callback(self, callback):
        """設置入料完成回調函數"""
        self.feeding_ready_callback = callback
    
    def pause_for_flow1_preparation(self):
        """為Flow1準備階段暫停入料檢測 - 修正版"""
        print("[AutoFeeding] *** Flow1準備階段暫停 - 防止CCD1拍攝到機械手臂 ***")
        self.pause_for_robot = True
        self.pause_reason = "Flow1準備中"
        self.feeding_ready = False
    
    def pause_for_flow5_preparation(self):
        """為Flow5準備階段暫停入料檢測 - 新增"""
        print("[AutoFeeding] *** Flow5準備階段暫停 - 防止干擾出料流程 ***")
        self.pause_for_robot = True
        self.pause_reason = "Flow5準備中"
        self.feeding_ready = False
    
    def resume_after_flow1_complete(self):
        """Flow1完成後恢復入料檢測 - 修正版"""
        print("[AutoFeeding] *** Flow1完成，恢復AutoFeeding持續供料 ***")
        self.pause_for_robot = False
        self.pause_reason = ""
        self.feeding_ready = False  # 重置狀態，開始新的檢測週期
    
    def resume_after_flow5_complete(self):
        """Flow5完成後恢復入料檢測 - 新增"""
        print("[AutoFeeding] *** Flow5完成，恢復AutoFeeding持續供料 ***")
        self.pause_for_robot = False
        self.pause_reason = ""
        self.feeding_ready = False  # 重置狀態，開始新的檢測週期
    
    def check_flow_control(self) -> bool:
        """檢查流程控制開關 (1350地址)"""
        try:
            control_value = self.read_register(self.CONTROL_ADDRESS)
            return control_value == 1
        except Exception:
            return False
    
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
    
    def clear_cg_registers(self) -> bool:
        """清空CG控制寄存器"""
        success = True
        success &= self.write_register(200, 0)  # CONTROL_COMMAND
        success &= self.write_register(203, 0)  # CAPTURE_COMPLETE
        success &= self.write_register(204, 0)  # DETECT_COMPLETE
        success &= self.write_register(205, 0)  # OPERATION_SUCCESS
        return success
    
    def check_modules_status(self) -> bool:
        """檢查VP、CG模組狀態"""
        print("[AutoFeeding] 檢查模組狀態...")
        
        # 檢查CG狀態
        cg_status = self.read_register(201)  # STATUS_REGISTER
        if cg_status is None:
            print("[AutoFeeding] ✗ CG模組無回應")
            return False
        
        cg_ready = bool(cg_status & 0x01)  # bit0=Ready
        cg_initialized = bool(cg_status & 0x08)  # bit3=Initialized
        
        print(f"[AutoFeeding] CG狀態: {cg_status:04b}, Ready={cg_ready}, Initialized={cg_initialized}")
        
        # 如果CG未準備就緒但已初始化，嘗試重置
        if not cg_ready and cg_initialized:
            print("[AutoFeeding] CG未Ready但已初始化，嘗試重置...")
            
            # 清零CG控制寄存器
            reset_success = True
            reset_success &= self.write_register(200, 0)  # CONTROL_COMMAND
            reset_success &= self.write_register(203, 0)  # CAPTURE_COMPLETE
            reset_success &= self.write_register(204, 0)  # DETECT_COMPLETE
            reset_success &= self.write_register(205, 0)  # OPERATION_SUCCESS
            
            if reset_success:
                print("[AutoFeeding] CG寄存器已清零，等待狀態更新...")
                time.sleep(0.5)  # 等待狀態更新
                
                # 重新檢查CG狀態
                cg_status = self.read_register(201)
                if cg_status is not None:
                    cg_ready = bool(cg_status & 0x01)
                    cg_initialized = bool(cg_status & 0x08)
                    print(f"[AutoFeeding] CG重置後狀態: {cg_status:04b}, Ready={cg_ready}, Initialized={cg_initialized}")
                else:
                    print("[AutoFeeding] ✗ CG重置後無回應")
                    return False
            else:
                print("[AutoFeeding] ✗ CG寄存器重置失敗")
        
        if not (cg_ready and cg_initialized):
            print(f"[AutoFeeding] ✗ CG模組未準備就緒 (Ready={cg_ready}, Initialized={cg_initialized})")
            return False
        
        # 檢查VP狀態
        vp_status = self.read_register(300)  # module_status
        vp_connected = self.read_register(301)  # device_connection
        
        print(f"[AutoFeeding] VP狀態: module_status={vp_status}, device_connection={vp_connected}")
        
        if vp_status != 1:
            print(f"[AutoFeeding] ✗ VP模組狀態異常: {vp_status}")
            return False
        
        if vp_connected != 1:
            print(f"[AutoFeeding] ✗ VP設備未連接: {vp_connected}")
            return False
        
        print("[AutoFeeding] ✓ 所有模組狀態正常")
        return True
    
    def trigger_cg_detection(self) -> CGDetectionResult:
        """觸發CG拍照檢測並獲取結果"""
        print("[AutoFeeding] 觸發CG拍照檢測...")
        
        result = CGDetectionResult()
        
        # 向200地址寫入16觸發拍照+檢測
        if not self.write_register(200, 16):
            print("[AutoFeeding] ✗ CG指令寫入失敗")
            return result
        
        print("[AutoFeeding] CG指令已發送，等待檢測完成...")
        
        # 等待拍照、檢測、操作完成標誌
        timeout = self.config['auto_program']['cg_timeout']
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            capture_complete = self.read_register(203)  # CAPTURE_COMPLETE
            detect_complete = self.read_register(204)   # DETECT_COMPLETE  
            operation_success = self.read_register(205) # OPERATION_SUCCESS
            
            # 每秒輸出一次等待狀態
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0:
                print(f"[AutoFeeding] 等待CG檢測... {elapsed:.1f}s (capture={capture_complete}, detect={detect_complete}, success={operation_success})")
            
            if capture_complete == 1 and detect_complete == 1 and operation_success == 1:
                result.capture_success = True
                result.detect_success = True
                result.operation_success = True
                print("[AutoFeeding] ✓ CG檢測完成")
                break
            
            time.sleep(self.config['timing']['status_check_interval'])
        
        if not result.operation_success:
            elapsed = time.time() - start_time
            print(f"[AutoFeeding] ✗ CG檢測超時或失敗 (耗時{elapsed:.1f}s)")
            return result
        
        # 讀取檢測結果
        result.cg_f_count = self.read_register(240) or 0
        result.total_detections = self.read_register(243) or 0
        
        print(f"[AutoFeeding] CG檢測結果: CG_F={result.cg_f_count}, 總檢測={result.total_detections}")
        
        # 提取CG_F世界座標
        if result.cg_f_count > 0:
            max_check = min(result.cg_f_count, self.config['auto_program']['max_cg_f_check'])
            
            for i in range(max_check):
                base_addr = 261 + (i * 4)  # 每個物件佔4個寄存器
                world_x = self.read_32bit_register(base_addr, base_addr + 1)
                world_y = self.read_32bit_register(base_addr + 2, base_addr + 3)
                result.cg_f_world_coords.append((world_x, world_y))
                print(f"[AutoFeeding] CG_F{i+1}座標: ({world_x:.2f}, {world_y:.2f})")
        
        return result
    
    def find_cg_f_in_protection_zone(self, detection_result: CGDetectionResult) -> Optional[Tuple[float, float]]:
        """尋找保護區域內的CG_F物件"""
        if detection_result.cg_f_count == 0:
            return None
        
        for i, (world_x, world_y) in enumerate(detection_result.cg_f_world_coords):
            if self.protection_zone.is_point_in_quad(world_x, world_y):
                return (world_x, world_y)
        
        return None
    
    def update_first_cg_f_coordinates(self, target_coords: Tuple[float, float]) -> bool:
        """將目標座標覆蓋到第一個CG_F位置(261-264)"""
        world_x, world_y = target_coords
        
        # 轉換為整數形式(×100)
        world_x_int = int(world_x * 100)
        world_y_int = int(world_y * 100)
        
        # 處理負數(補碼)
        if world_x_int < 0:
            world_x_int = world_x_int + 4294967296  # 2^32
        if world_y_int < 0:
            world_y_int = world_y_int + 4294967296  # 2^32
        
        # 分解為高低位
        x_high = (world_x_int >> 16) & 0xFFFF
        x_low = world_x_int & 0xFFFF
        y_high = (world_y_int >> 16) & 0xFFFF
        y_low = world_y_int & 0xFFFF
        
        # 寫入寄存器261-264
        success = True
        success &= self.write_register(261, x_high)  # CG_F_1_WORLD_X_HIGH
        success &= self.write_register(262, x_low)   # CG_F_1_WORLD_X_LOW
        success &= self.write_register(263, y_high)  # CG_F_1_WORLD_Y_HIGH
        success &= self.write_register(264, y_low)   # CG_F_1_WORLD_Y_LOW
        
        return success
    
    def stop_vp_vibration(self) -> bool:
        """停止VP震動盤"""
        success = True
        success &= self.write_register(320, 3)  # stop_all指令
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, 99)  # emergency stop id
        
        if success:
            time.sleep(self.config['auto_program']['vp_stop_delay'])
        
        return success
    
    def trigger_vp_vibration_and_redetect(self) -> Optional[Tuple[float, float]]:
        """觸發VP震動並重新檢測CG_F"""
        # VP指令參數
        command_code = 5  # execute_action
        action_code = self.config['vp_params']['spread_action_code']  # spread=11
        strength = self.config['vp_params']['spread_strength']        # 50
        frequency = self.config['vp_params']['spread_frequency']      # 43
        duration = self.config['vp_params']['spread_duration']        # 0.5秒
        
        # 啟動VP震動
        success = True
        success &= self.write_register(320, command_code)
        success &= self.write_register(321, action_code)
        success &= self.write_register(322, strength)
        success &= self.write_register(323, frequency)
        success &= self.write_register(324, int(time.time()) % 65535)  # command_id
        
        if not success:
            return None
        
        # 等待震動
        time.sleep(duration)
        
        # 停止震動
        if not self.stop_vp_vibration():
            return None
        
        # 等待穩定
        time.sleep(0.3)
        
        # 重新檢測
        detection_result = self.trigger_cg_detection()
        
        if not detection_result.operation_success:
            return None
        
        target_coords = self.find_cg_f_in_protection_zone(detection_result)
        
        if target_coords:
            self.cg_f_found_count += 1
            return target_coords
        
        return None
    
    def trigger_flow4_feeding(self) -> bool:
        """觸發Flow4送料"""
        if not self.write_register(448, 1):
            return False
        
        time.sleep(self.config['auto_program']['flow4_pulse_duration'])
        
        if not self.write_register(448, 0):
            return False
        
        return True
    
    def trigger_vp_clear_vibration(self) -> bool:
        """觸發VP清空震動 - 翻正所有料件"""
        print("[AutoFeeding] 觸發VP清空震動 (spread強度50頻率43)")
        
        command_code = 5   # execute_action
        action_code = 11   # spread
        strength = 50      # 強度50
        frequency = 43     # 頻率43
        duration = 0.5     # 持續0.5秒
        
        # 啟動VP震動
        success = True
        success &= self.write_register(320, command_code)
        success &= self.write_register(321, action_code)
        success &= self.write_register(322, strength)
        success &= self.write_register(323, frequency)
        success &= self.write_register(324, int(time.time()) % 65535)  # command_id
        
        if not success:
            print("[AutoFeeding] ✗ VP清空震動指令發送失敗")
            return False
        
        # 等待震動完成
        time.sleep(duration)
        
        # 停止震動
        if not self.stop_vp_vibration():
            print("[AutoFeeding] ✗ VP清空震動停止失敗")
            return False
        
        print("[AutoFeeding] ✓ VP清空震動完成")
        return True
    
    def check_vp_empty_condition(self) -> bool:
        """檢查VP是否為空 (總檢測數為0)"""
        detection_result = self.trigger_cg_detection()
        
        if not detection_result.operation_success:
            print("[AutoFeeding] ✗ VP空檢測失敗")
            return False
        
        is_empty = (detection_result.total_detections == 0)
        print(f"[AutoFeeding] VP空檢測: 總檢測數={detection_result.total_detections}, 是否為空={is_empty}")
        
        return is_empty
    
    def execute_vp_clearing_process(self) -> bool:
        """執行VP清空流程"""
        print("\n[AutoFeeding] === 開始VP清空流程 ===")
        print("[AutoFeeding] 料桶無料，執行VP清空：翻正所有料件 + Flow1清空")
        
        self.vp_clearing_mode = True
        self.vp_empty_detection_count = 0
        
        try:
            # 步驟1：VP震動翻正所有料件
            if not self.trigger_vp_clear_vibration():
                print("[AutoFeeding] ✗ VP清空震動失敗")
                return False
            
            # 步驟2：重複執行Flow1直到VP空
            flow1_attempts = 0
            max_flow1_attempts = 20  # 防止無限循環
            
            while flow1_attempts < max_flow1_attempts:
                flow1_attempts += 1
                print(f"\n[AutoFeeding] Flow1清空嘗試 {flow1_attempts}/{max_flow1_attempts}")
                
                # 觸發Flow1
                if not self.write_register(1240, 1):  # FLOW1_CONTROL
                    print("[AutoFeeding] ✗ Flow1觸發失敗")
                    break
                
                # 等待Flow1完成 (最多30秒)
                flow1_timeout = 30.0
                start_time = time.time()
                flow1_completed = False
                
                while (time.time() - start_time) < flow1_timeout:
                    flow1_status = self.read_register(1204)  # FLOW1_COMPLETE
                    if flow1_status == 1:
                        flow1_completed = True
                        print(f"[AutoFeeding] ✓ Flow1清空完成 (耗時{time.time() - start_time:.1f}s)")
                        break
                    time.sleep(0.5)
                
                if not flow1_completed:
                    print("[AutoFeeding] ✗ Flow1清空超時")
                    break
                
                # 清除Flow1控制狀態
                self.write_register(1240, 0)
                time.sleep(0.5)
                
                # VP震動 + 檢測VP是否為空
                self.trigger_vp_clear_vibration()
                time.sleep(0.5)  # 等待0.5秒
                
                if self.check_vp_empty_condition():
                    self.vp_empty_detection_count += 1
                    print(f"[AutoFeeding] VP空檢測 {self.vp_empty_detection_count}/5")
                    
                    if self.vp_empty_detection_count >= 5:
                        print("[AutoFeeding] ✓ VP連續5次檢測為空，清空流程完成")
                        
                        # 清空流程完成，停止AutoProgram
                        print("[AutoFeeding] 設置控制地址1350=0，停止AutoProgram")
                        self.write_register(self.CONTROL_ADDRESS, 0)
                        
                        return True
                else:
                    self.vp_empty_detection_count = 0  # 重置空檢測計數
                
                time.sleep(1.0)  # 等待下一輪
            
            print(f"[AutoFeeding] ✗ VP清空流程失敗 (Flow1嘗試{flow1_attempts}次)")
            return False
            
        except Exception as e:
            print(f"[AutoFeeding] ✗ VP清空流程異常: {e}")
            return False
        
        finally:
            self.vp_clearing_mode = False
            print("[AutoFeeding] === VP清空流程結束 ===\n")
    
    def auto_feeding_cycle(self) -> bool:
        """執行一次自動入料週期"""
        try:
            # 檢查流程控制開關
            if not self.check_flow_control():
                print("[AutoFeeding] 流程控制開關關閉 (1350=0)，跳過週期")
                return False
            
            self.cycle_count += 1
            print(f"\n[AutoFeeding] === 週期 {self.cycle_count} 開始 ===")
            
            # 檢查模組狀態
            if not self.check_modules_status():
                print(f"[AutoFeeding] 週期 {self.cycle_count} 跳過: 模組狀態檢查失敗")
                return False
            
            # 觸發CG檢測
            detection_result = self.trigger_cg_detection()
            
            if not detection_result.operation_success:
                print(f"[AutoFeeding] 週期 {self.cycle_count} 跳過: CG檢測失敗")
                return False
            
            # 尋找保護區域內的CG_F
            target_coords = self.find_cg_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 找到保護區域內的CG_F
                self.cg_f_found_count += 1
                self.flow4_consecutive_count = 0  # 重置連續直振計數
                
                if self.update_first_cg_f_coordinates(target_coords):
                    print(f"[AutoFeeding] ✓ 找到CG_F在保護區域: {target_coords}")
                    self.feeding_ready = True
                    # 通知RobotJob
                    if self.feeding_ready_callback:
                        self.feeding_ready_callback()
                else:
                    print("[AutoFeeding] ✗ CG_F座標更新失敗")
            else:
                # 沒有CG_F在保護區域內
                if detection_result.total_detections < 4:
                    # 料件不足，觸發Flow4送料
                    if self.trigger_flow4_feeding():
                        self.flow4_trigger_count += 1
                        self.flow4_consecutive_count += 1  # 增加連續直振計數
                        
                        print(f"[AutoFeeding] 料件不足，觸發Flow4送料 (總檢測={detection_result.total_detections})")
                        print(f"[AutoFeeding] 連續直振次數: {self.flow4_consecutive_count}/5")
                        
                        # 檢查是否達到連續直振5次
                        if self.flow4_consecutive_count >= 5:
                            print("[AutoFeeding] 連續直振5次，料桶無料，開始VP清空流程")
                            self.execute_vp_clearing_process()
                            return True  # 清空流程會自動停止系統
                
                elif detection_result.total_detections >= 4:
                    # 料件充足但沒有CG_F在保護區，震動散開並重新檢測
                    print(f"[AutoFeeding] 料件充足但無CG_F在保護區 (總檢測={detection_result.total_detections})")
                    self.flow4_consecutive_count = 0  # 重置連續直振計數
                    
                    target_coords_after_vp = self.trigger_vp_vibration_and_redetect()
                    if target_coords_after_vp:
                        # 震動後找到CG_F，更新座標
                        if self.update_first_cg_f_coordinates(target_coords_after_vp):
                            print(f"[AutoFeeding] ✓ 震動後找到CG_F: {target_coords_after_vp}")
                            self.feeding_ready = True
                            # 通知RobotJob
                            if self.feeding_ready_callback:
                                self.feeding_ready_callback()
                        self.vp_vibration_count += 1
                    else:
                        print("[AutoFeeding] 震動後仍未找到保護區域內的CG_F")
                        self.vp_vibration_count += 1
            
            # 清空CG寄存器
            self.clear_cg_registers()
            time.sleep(self.config['timing']['register_clear_delay'])
            
            print(f"[AutoFeeding] 週期 {self.cycle_count} 完成")
            return True
            
        except Exception as e:
            print(f"[AutoFeeding] 週期 {self.cycle_count} 異常: {e}")
            return False
    
    def start(self):
        """啟動自動入料執行緒"""
        if self.running:
            return
        
        print("[AutoFeeding] 啟動自動入料執行緒")
        self.running = True
        self.cycle_count = 0
        self.cg_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        self.flow4_consecutive_count = 0
        self.vp_clearing_mode = False
        self.vp_empty_detection_count = 0
        self.feeding_ready = False
        self.pause_for_robot = False
        self.pause_reason = ""
        
        self.thread = threading.Thread(target=self._auto_feeding_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """停止自動入料執行緒"""
        if not self.running:
            return
        
        print("[AutoFeeding] 停止自動入料執行緒")
        self.running = False
        self.feeding_ready = False
        
        # 緊急停止VP
        try:
            self.stop_vp_vibration()
        except:
            pass
        
        if self.thread and self.thread.is_alive() and threading.current_thread() != self.thread:
            self.thread.join(timeout=2.0)
        elif threading.current_thread() == self.thread:
            print("[AutoFeeding] 執行緒內部停止，跳過join操作")
    
    def _auto_feeding_loop(self):
        """自動入料主循環 - 修正版"""
        cycle_interval = self.config['auto_program']['cycle_interval']
        
        while self.running:
            try:
                # 檢查流程控制開關 (100ms高頻輪詢)
                if not self.check_flow_control():
                    print("[AutoFeeding] 流程控制關閉，等待重新啟動...")
                    time.sleep(0.1)  # 100ms高頻輪詢
                    continue
                
                # === 修正版：精確的暫停檢查 ===
                if self.pause_for_robot:
                    print(f"[AutoFeeding] 已暫停 - {self.pause_reason}，等待恢復...")
                    time.sleep(0.5)
                    continue
                
                # 如果在VP清空模式，不執行正常檢測
                if self.vp_clearing_mode:
                    print("[AutoFeeding] VP清空模式中，跳過正常檢測...")
                    time.sleep(1.0)
                    continue
                
                # 只有在feeding_ready=False時才執行檢測
                if not self.feeding_ready:
                    self.auto_feeding_cycle()
                
                time.sleep(cycle_interval)
                
            except Exception as e:
                print(f"[AutoFeeding] 循環異常: {e}")
                time.sleep(1.0)


class RobotJobThread:
    """機械臂作業執行緒 - 修正版"""
    
    def __init__(self, modbus_client: ModbusTcpClient, config: Dict):
        self.modbus_client = modbus_client
        self.config = config
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # 狀態變數
        self.prepare_done = False
        
        # AutoFeeding執行緒參考
        self.auto_feeding_ref = None
        
        # 模組基地址
        self.FLOW1_CONTROL = 1240  # Flow1控制
        self.FLOW1_STATUS = 1204   # Flow1完成狀態
        self.FLOW5_STATUS = 1206   # Flow5完成狀態
        self.CONTROL_ADDRESS = 1350  # 流程控制地址
        
        # 統計資訊
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        
        # === 新增：Flow5狀態跟蹤 ===
        self.flow5_detected = False  # 是否檢測到Flow5執行
        
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
    
    def check_flow_control(self) -> bool:
        """檢查流程控制開關 (1350地址)"""
        try:
            control_value = self.read_register(self.CONTROL_ADDRESS)
            return control_value == 1
        except Exception:
            return False
    
    def on_feeding_ready(self):
        """自動入料完成回調 - 修正版"""
        print("[RobotJob] 收到自動入料完成通知")
        
        # 檢查流程控制開關
        if not self.check_flow_control():
            print("[RobotJob] 流程控制關閉，忽略入料完成通知")
            return
        
        # 只要prepare_done=False，就觸發Flow1
        if not self.prepare_done:
            print("[RobotJob] prepare_done=False，準備觸發Flow1去VP拿料")
            
            # === 修正版：Flow1觸發前先暫停AutoFeeding ===
            if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                print("[RobotJob] *** Flow1觸發前先暫停AutoFeeding - 防止CCD1拍攝干擾 ***")
                self.auto_feeding_ref.pause_for_flow1_preparation()
                time.sleep(0.2)  # 確保暫停生效
            
            # 觸發Flow1
            if self.write_register(self.FLOW1_CONTROL, 1):
                self.flow1_trigger_count += 1
                print("[RobotJob] ✓ Flow1觸發成功")
            else:
                print("[RobotJob] ✗ Flow1觸發失敗")
        else:
            # prepare_done=True，表示機台已準備好，等待出料指令
            print(f"[RobotJob] prepare_done=True，機台已準備好接受出料指令")
    
    def robot_job_cycle(self):
        """機械臂作業週期 - 修正版"""
        try:
            # 檢查流程控制開關
            if not self.check_flow_control():
                return
            
            # 檢查機械臂是否Ready (1200=9)
            robot_status = self.read_register(1200)  # 機械臂狀態寄存器
            robot_ready = (robot_status == 9) if robot_status is not None else False
            
            # === 修正版：檢查Flow1完成狀態 ===
            flow1_status = self.read_register(self.FLOW1_STATUS)
            if flow1_status == 1 and not self.prepare_done:
                print("[RobotJob] 檢測到Flow1完成")
                
                # Flow1完成，設置prepare_done=True
                self.prepare_done = True
                
                # === 修正版：Flow1完成後恢復AutoFeeding持續供料 ===
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    print("[RobotJob] *** Flow1完成，恢復AutoFeeding持續供料 ***")
                    self.auto_feeding_ref.resume_after_flow1_complete()
                
                # 只清空Flow1控制狀態，保持Flow1完成狀態供其他模組使用
                self.write_register(self.FLOW1_CONTROL, 0)
                print("[RobotJob] Flow1控制狀態已清零，prepare_done=True，等待其他模組執行Flow5")
            
            # === 新增：檢查Flow5執行狀態 ===
            # 讀取Flow5狀態寄存器，檢測Flow5是否開始執行
            flow5_running_status = self.read_register(1202)  # 假設1202是Flow5運行狀態
            if flow5_running_status == 1 and not self.flow5_detected:
                print("[RobotJob] 檢測到Flow5開始執行")
                self.flow5_detected = True
                
                # === Flow5執行前暫停AutoFeeding ===
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    print("[RobotJob] *** Flow5執行前暫停AutoFeeding - 防止干擾出料流程 ***")
                    self.auto_feeding_ref.pause_for_flow5_preparation()
            
            # === 修正版：檢查Flow5完成狀態 ===
            flow5_status = self.read_register(self.FLOW5_STATUS)
            if flow5_status == 1:
                print("[RobotJob] 檢測到Flow5完成")
                
                # Flow5完成，重置prepare_done=False，準備下一輪
                self.prepare_done = False
                self.flow5_complete_count += 1
                self.flow5_detected = False  # 重置Flow5檢測狀態
                
                # === 修正版：Flow5完成後恢復AutoFeeding ===
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    print("[RobotJob] *** Flow5完成，恢復AutoFeeding持續供料 ***")
                    self.auto_feeding_ref.resume_after_flow5_complete()
                
                # 清除Flow5完成狀態
                self.write_register(self.FLOW5_STATUS, 0)
                print("[RobotJob] Flow5完成狀態已清零，prepare_done=False，系統準備新週期")
            
            # === 修正版：核心邏輯 ===
            # 當prepare_done=False且機械臂Ready時，確保AutoFeeding運行
            if not self.prepare_done and robot_ready:
                print(f"[RobotJob] 系統條件滿足：prepare_done=False, 機械臂Ready={robot_ready}")
                
                # 確保AutoFeeding正在運行且未暫停
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    if not self.auto_feeding_ref.running:
                        print("[RobotJob] 啟動AutoFeeding執行緒")
                        self.auto_feeding_ref.start()
                    
                    # 只有在非Flow1/Flow5執行期間才恢復
                    if (self.auto_feeding_ref.pause_for_robot and 
                        flow1_status != 1 and flow5_running_status != 1):
                        print("[RobotJob] 恢復AutoFeeding檢測 - 系統準備新週期")
                        self.auto_feeding_ref.resume_after_flow5_complete()
        
        except Exception as e:
            print(f"[RobotJob] 機械臂作業週期異常: {e}")
    
    def start(self):
        """啟動機械臂作業執行緒"""
        if self.running:
            return
        
        print("[RobotJob] 啟動機械臂作業執行緒")
        self.running = True
        self.prepare_done = False
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        self.flow5_detected = False
        
        # 初始化Flow1完成狀態為0
        self.write_register(self.FLOW1_STATUS, 0)
        
        self.thread = threading.Thread(target=self._robot_job_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """停止機械臂作業執行緒"""
        if not self.running:
            return
        
        print("[RobotJob] 停止機械臂作業執行緒")
        self.running = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
    
    def _robot_job_loop(self):
        """機械臂作業主循環"""
        while self.running:
            try:
                self.robot_job_cycle()
                time.sleep(0.5)  # 0.5秒檢查間隔
                
            except Exception as e:
                print(f"[RobotJob] 循環異常: {e}")
                time.sleep(1.0)


class AutoProgramRobotJobController:
    """CG入料+機械臂協調控制系統 - 修正版"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址 1300-1349
        self.base_address = 1300
        
        # 載入配置
        self.config = self.load_config()
        
        # 建立執行緒
        self.auto_feeding_thread = None
        self.robot_job_thread = None
        
        # 系統狀態
        self.system_status = SystemStatus.STOPPED
        
        print("CG入料+機械臂協調控制系統初始化完成 (修正版)")
        print(f"Modbus服務器: {modbus_host}:{modbus_port}")
        print(f"系統基地址: {self.base_address}")
        print(f"流程控制地址: 1350")
        print("修正項目: Flow1/Flow5執行時精確暫停AutoFeeding機制")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "auto_program": {
                "cycle_interval": 2.0,
                "cg_timeout": 10.0,
                "vp_vibration_duration": 0.5,
                "vp_stop_delay": 0.2,
                "flow4_pulse_duration": 0.1,
                "max_cg_f_check": 5,
                "flow4_consecutive_limit": 5,
                "vp_empty_check_count": 5
            },
            "vp_params": {
                "spread_action_code": 11,
                "spread_strength": 50,
                "spread_frequency": 43,
                "spread_duration": 0.5,
                "stop_command_code": 3,
                "clear_strength": 50,
                "clear_frequency": 43
            },
            "timing": {
                "command_delay": 0.1,
                "status_check_interval": 0.1,
                "register_clear_delay": 0.05,
                "flow_control_poll_interval": 0.1
            },
            "modbus_mapping": {
                "base_address": 1300,
                "control_address": 1350
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autoprogram_robotjob_cg_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            print(f"配置檔案處理失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_system_registers()
            else:
                print(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def init_system_registers(self):
        """初始化系統寄存器"""
        try:
            # 系統狀態寄存器映射
            self.modbus_client.write_register(1300, SystemStatus.STOPPED.value, slave=1)
            self.modbus_client.write_register(1301, 0, slave=1)  # AutoFeeding停止
            self.modbus_client.write_register(1302, 0, slave=1)  # RobotJob停止
            self.modbus_client.write_register(1303, 0, slave=1)  # 無錯誤
            self.modbus_client.write_register(1310, 0, slave=1)  # 連續直振次數
            self.modbus_client.write_register(1311, 0, slave=1)  # VP空檢測次數
            self.modbus_client.write_register(1350, 0, slave=1)  # 流程控制關閉
            print("系統寄存器初始化完成")
        except Exception as e:
            print(f"系統寄存器初始化失敗: {e}")
    
    def update_system_registers(self):
        """更新系統寄存器"""
        try:
            if not self.connected:
                return
            
            # 更新系統狀態
            self.modbus_client.write_register(1300, self.system_status.value, slave=1)
            
            # 更新執行緒狀態
            auto_feeding_running = 1 if (self.auto_feeding_thread and self.auto_feeding_thread.running) else 0
            robot_job_running = 1 if (self.robot_job_thread and self.robot_job_thread.running) else 0
            
            self.modbus_client.write_register(1301, auto_feeding_running, slave=1)
            self.modbus_client.write_register(1302, robot_job_running, slave=1)
            
            # 更新統計資訊
            if self.auto_feeding_thread:
                self.modbus_client.write_register(1304, self.auto_feeding_thread.cycle_count, slave=1)
                self.modbus_client.write_register(1305, self.auto_feeding_thread.cg_f_found_count, slave=1)
                self.modbus_client.write_register(1306, self.auto_feeding_thread.flow4_trigger_count, slave=1)
                self.modbus_client.write_register(1307, self.auto_feeding_thread.vp_vibration_count, slave=1)
                self.modbus_client.write_register(1310, self.auto_feeding_thread.flow4_consecutive_count, slave=1)
                self.modbus_client.write_register(1311, self.auto_feeding_thread.vp_empty_detection_count, slave=1)
            
            if self.robot_job_thread:
                self.modbus_client.write_register(1308, self.robot_job_thread.flow1_trigger_count, slave=1)
                self.modbus_client.write_register(1309, self.robot_job_thread.flow5_complete_count, slave=1)
                
        except Exception as e:
            print(f"系統寄存器更新失敗: {e}")
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def start_system(self):
        """啟動協調控制系統"""
        if not self.connected:
            print("請先連接Modbus服務器")
            return
        
        print("=== 啟動CG入料+機械臂協調控制系統 (修正版) ===")
        
        # 建立AutoFeeding執行緒
        self.auto_feeding_thread = AutoFeedingThread(self.modbus_client, self.config)
        
        # 建立RobotJob執行緒
        self.robot_job_thread = RobotJobThread(self.modbus_client, self.config)
        
        # 設置執行緒間的相互參考
        self.robot_job_thread.auto_feeding_ref = self.auto_feeding_thread
        
        # 設置回調連接
        self.auto_feeding_thread.set_feeding_ready_callback(self.robot_job_thread.on_feeding_ready)
        
        # 啟動執行緒
        self.auto_feeding_thread.start()
        self.robot_job_thread.start()
        
        # 更新系統狀態
        self.system_status = SystemStatus.AUTO_FEEDING_RUNNING
        
        print("協調控制系統已啟動")
        print("流程控制: 寫入1350=1啟動流程，1350=0停止流程")
        print("修正機制: Flow1/Flow5執行時自動暫停AutoFeeding，完成後自動恢復")
    
    def stop_system(self):
        """停止協調控制系統"""
        print("=== 停止CG入料+機械臂協調控制系統 ===")
        
        self.system_status = SystemStatus.STOPPED
        
        # 停止執行緒
        if self.auto_feeding_thread:
            self.auto_feeding_thread.stop()
        
        if self.robot_job_thread:
            self.robot_job_thread.stop()
        
        # 更新系統寄存器
        self.update_system_registers()
        
        print("協調控制系統已停止")
        self.print_statistics()
    
    def enable_flow_control(self):
        """啟動流程控制 (1350=1)"""
        if self.connected:
            try:
                self.modbus_client.write_register(1350, 1, slave=1)
                print("流程控制已啟動 (1350=1)")
            except Exception as e:
                print(f"流程控制啟動失敗: {e}")
    
    def disable_flow_control(self):
        """停止流程控制 (1350=0)"""
        if self.connected:
            try:
                self.modbus_client.write_register(1350, 0, slave=1)
                print("流程控制已停止 (1350=0)")
            except Exception as e:
                print(f"流程控制停止失敗: {e}")
    
    def print_statistics(self):
        """輸出統計資訊"""
        print(f"\n=== CG協調控制系統統計 (修正版) ===")
        
        if self.auto_feeding_thread:
            print(f"AutoFeeding統計:")
            print(f"  總週期數: {self.auto_feeding_thread.cycle_count}")
            print(f"  CG_F找到次數: {self.auto_feeding_thread.cg_f_found_count}")
            print(f"  Flow4觸發次數: {self.auto_feeding_thread.flow4_trigger_count}")
            print(f"  VP震動次數: {self.auto_feeding_thread.vp_vibration_count}")
            print(f"  連續直振次數: {self.auto_feeding_thread.flow4_consecutive_count}")
            print(f"  VP空檢測次數: {self.auto_feeding_thread.vp_empty_detection_count}")
            print(f"  VP清空模式: {'是' if self.auto_feeding_thread.vp_clearing_mode else '否'}")
            print(f"  目前暫停狀態: {'是' if self.auto_feeding_thread.pause_for_robot else '否'}")
            if self.auto_feeding_thread.pause_for_robot:
                print(f"  暫停原因: {self.auto_feeding_thread.pause_reason}")
            if self.auto_feeding_thread.cycle_count > 0:
                success_rate = (self.auto_feeding_thread.cg_f_found_count / self.auto_feeding_thread.cycle_count) * 100
                print(f"  CG_F找到率: {success_rate:.1f}%")
        
        if self.robot_job_thread:
            print(f"RobotJob統計:")
            print(f"  Flow1觸發次數: {self.robot_job_thread.flow1_trigger_count}")
            print(f"  Flow5完成次數: {self.robot_job_thread.flow5_complete_count}")
            print(f"  目前prepare_done狀態: {self.robot_job_thread.prepare_done}")
    
    def get_status_info(self) -> Dict[str, Any]:
        """獲取狀態資訊"""
        # 讀取流程控制開關狀態
        flow_control_enabled = False
        if self.connected:
            try:
                result = self.modbus_client.read_holding_registers(1350, count=1, slave=1)
                if not result.isError():
                    flow_control_enabled = result.registers[0] == 1
            except:
                pass
        
        status = {
            "connected": self.connected,
            "system_status": self.system_status.name,
            "flow_control_enabled": flow_control_enabled,
            "auto_feeding_running": self.auto_feeding_thread.running if self.auto_feeding_thread else False,
            "robot_job_running": self.robot_job_thread.running if self.robot_job_thread else False
        }
        
        if self.auto_feeding_thread:
            status.update({
                "auto_feeding_cycle_count": self.auto_feeding_thread.cycle_count,
                "cg_f_found_count": self.auto_feeding_thread.cg_f_found_count,
                "flow4_trigger_count": self.auto_feeding_thread.flow4_trigger_count,
                "vp_vibration_count": self.auto_feeding_thread.vp_vibration_count,
                "flow4_consecutive_count": self.auto_feeding_thread.flow4_consecutive_count,
                "vp_clearing_mode": self.auto_feeding_thread.vp_clearing_mode,
                "vp_empty_detection_count": self.auto_feeding_thread.vp_empty_detection_count,
                "feeding_ready": self.auto_feeding_thread.feeding_ready,
                "pause_for_robot": self.auto_feeding_thread.pause_for_robot,  # 新增
                "pause_reason": self.auto_feeding_thread.pause_reason         # 新增
            })
        
        if self.robot_job_thread:
            status.update({
                "prepare_done": self.robot_job_thread.prepare_done,
                "flow1_trigger_count": self.robot_job_thread.flow1_trigger_count,
                "flow5_complete_count": self.robot_job_thread.flow5_complete_count,
                "flow5_detected": self.robot_job_thread.flow5_detected  # 新增
            })
        
        return status


def main():
    """主程序"""
    print("CG入料+機械臂協調控制系統啟動 (修正版)")
    print("修正項目: 精確的Flow1/Flow5執行時暫停AutoFeeding機制")
    
    # 創建控制器
    controller = AutoProgramRobotJobController()
    
    # 連接Modbus
    if not controller.connect():
        print("Modbus連接失敗，程序退出")
        return
    
    try:
        # 啟動協調控制系統
        controller.start_system()
        
        # 定期更新系統寄存器
        def update_registers():
            while True:
                controller.update_system_registers()
                time.sleep(2.0)
        
        update_thread = threading.Thread(target=update_registers, daemon=True)
        update_thread.start()
        
        # 主循環 - 等待用戶操作
        print("\n指令說明:")
        print("  s - 顯示狀態")
        print("  r - 重啟系統")
        print("  start - 啟動流程控制 (1350=1)")
        print("  stop - 停止流程控制 (1350=0)")
        print("  pause - 手動暫停自動入料")
        print("  resume - 手動恢復自動入料")
        print("  q - 退出程序")
        
        while True:
            try:
                cmd = input("\n請輸入指令: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    status = controller.get_status_info()
                    print(f"\n=== 系統狀態 (修正版) ===")
                    print(f"連接狀態: {status['connected']}")
                    print(f"系統狀態: {status['system_status']}")
                    print(f"流程控制: {'啟動' if status['flow_control_enabled'] else '停止'}")
                    print(f"AutoFeeding運行: {'是' if status['auto_feeding_running'] else '否'}")
                    print(f"RobotJob運行: {'是' if status['robot_job_running'] else '否'}")
                    
                    if 'pause_for_robot' in status:
                        print(f"AutoFeeding暫停: {'是' if status['pause_for_robot'] else '否'}")
                        if status['pause_for_robot'] and 'pause_reason' in status:
                            print(f"暫停原因: {status['pause_reason']}")
                    
                    if 'prepare_done' in status:
                        print(f"prepare_done狀態: {status['prepare_done']}")
                    
                    if 'feeding_ready' in status:
                        print(f"feeding_ready狀態: {status['feeding_ready']}")
                    
                    # 顯示統計資訊
                    if 'auto_feeding_cycle_count' in status:
                        print(f"\nAutoFeeding統計:")
                        print(f"  週期數: {status['auto_feeding_cycle_count']}")
                        print(f"  CG_F找到: {status['cg_f_found_count']}")
                        print(f"  Flow4觸發: {status['flow4_trigger_count']}")
                        print(f"  VP震動: {status['vp_vibration_count']}")
                        print(f"  連續直振: {status['flow4_consecutive_count']}")
                    
                    if 'flow1_trigger_count' in status:
                        print(f"\nRobotJob統計:")
                        print(f"  Flow1觸發: {status['flow1_trigger_count']}")
                        print(f"  Flow5完成: {status['flow5_complete_count']}")
                        
                elif cmd == 'r':
                    print("重啟系統...")
                    controller.stop_system()
                    time.sleep(1.0)
                    controller.start_system()
                    print("系統重啟完成")
                    
                elif cmd == 'start':
                    controller.enable_flow_control()
                    
                elif cmd == 'stop':
                    controller.disable_flow_control()
                    
                elif cmd == 'pause':
                    if controller.auto_feeding_thread:
                        controller.auto_feeding_thread.pause_for_flow1_preparation()
                        print("手動暫停AutoFeeding")
                    else:
                        print("AutoFeeding執行緒未運行")
                        
                elif cmd == 'resume':
                    if controller.auto_feeding_thread:
                        controller.auto_feeding_thread.resume_after_flow1_complete()
                        print("手動恢復AutoFeeding")
                    else:
                        print("AutoFeeding執行緒未運行")
                        
                else:
                    print("無效指令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    finally:
        # 清理資源
        controller.stop_system()
        controller.disconnect()
        print("程序已退出")


if __name__ == "__main__":
    main()