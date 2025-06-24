# -*- coding: utf-8 -*-
"""
CCD1VisionCode.py - CCD視覺控制系統 (Modbus TCP Client版本)
基於工業設備控制架構的視覺辨識Web控制介面
作為Modbus TCP Client連接外部PLC/HMI設備
"""

import sys
import os
import time
import threading
import json
import base64
from typing import Optional, Dict, Any, Tuple, List
import numpy as np
import cv2
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import logging
from dataclasses import dataclass, asdict
from datetime import datetime

# 導入Modbus TCP Client服務 (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
    PYMODBUS_VERSION = "3.9.2"
    print("✅ Modbus Client模組導入成功 (pymodbus 3.9.2)")
except ImportError as e:
    print(f"⚠️ Modbus Client模組導入失敗: {e}")
    print("💡 請確認pymodbus版本: pip install pymodbus>=3.0.0")
    MODBUS_AVAILABLE = False
    PYMODBUS_VERSION = "unavailable"

# 導入相機管理模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
try:
    from camera_manager import OptimizedCameraManager, CameraConfig, CameraMode, PixelFormat
    CAMERA_MANAGER_AVAILABLE = True
except ImportError as e:
    print(f"❌ 無法導入 camera_manager 模組: {e}")
    CAMERA_MANAGER_AVAILABLE = False


@dataclass
class DetectionParams:
    """檢測參數配置"""
    min_area: float = 50000.0
    min_roundness: float = 0.8
    gaussian_kernel_size: int = 9
    gaussian_sigma: float = 2.0
    canny_low: int = 20
    canny_high: int = 60


@dataclass
class VisionResult:
    """視覺辨識結果"""
    circle_count: int
    circles: List[Dict[str, Any]]
    processing_time: float
    capture_time: float
    total_time: float
    timestamp: str
    success: bool
    error_message: Optional[str] = None


class ModbusTcpClientService:
    """Modbus TCP Client服務 - 連接外部PLC/HMI設備"""
    
    def __init__(self, server_ip="192.168.1.100", server_port=502):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.running = False
        self.vision_controller = None
        
        # 連接參數
        self.reconnect_delay = 5.0  # 重連延遲
        self.read_timeout = 3.0     # 讀取超時
        self.write_timeout = 3.0    # 寫入超時
        
        # 新增：同步控制
        self.sync_enabled = False           # 同步開關
        self.sync_thread = None            # 同步線程
        self.sync_running = False          # 同步線程運行狀態
        self.sync_interval = 0.1           # 同步間隔 (100ms)
        self.status_sync_counter = 0       # 狀態同步計數器
        self.status_sync_interval = 10     # 每10次循環同步一次狀態 (1秒)
        
        # Modbus寄存器映射 (CCD1專用地址段: 200-299)
        self.REGISTERS = {
            # 控制寄存器 (200-209) - 從外部PLC讀取控制指令
            'EXTERNAL_CONTROL_ENABLE': 200,    # 外部控制啟用 (0=禁用, 1=啟用)
            'CAPTURE_TRIGGER': 201,            # 拍照觸發 (讀取到1時觸發)
            'DETECT_TRIGGER': 202,             # 拍照+檢測觸發 (讀取到1時觸發)
            'SYSTEM_RESET': 203,               # 系統重置 (讀取到1時重置)
            'PARAM_UPDATE_TRIGGER': 204,       # 參數更新觸發
            
            # 參數設定寄存器 (210-219) - 從外部PLC讀取參數設定
            'MIN_AREA_HIGH': 210,              # 最小面積設定 (高16位)
            'MIN_AREA_LOW': 211,               # 最小面積設定 (低16位)
            'MIN_ROUNDNESS': 212,              # 最小圓度設定 (乘以1000)
            'GAUSSIAN_KERNEL': 213,            # 高斯核大小
            'CANNY_LOW': 214,                  # Canny低閾值
            'CANNY_HIGH': 215,                 # Canny高閾值
            
            # 狀態回報寄存器 (220-239) - 寫入狀態到外部PLC
            'SYSTEM_STATUS': 220,              # 系統狀態 (0=斷線, 1=已連接, 2=處理中)
            'CAMERA_CONNECTED': 221,           # 相機連接狀態 (0=斷線, 1=已連接)
            'LAST_OPERATION_STATUS': 222,      # 最後操作狀態 (0=失敗, 1=成功)
            'PROCESSING_PROGRESS': 223,        # 處理進度 (0-100)
            
            # 結果寄存器 (240-279) - 寫入檢測結果到外部PLC
            'CIRCLE_COUNT': 240,               # 檢測到的圓形數量
            'CIRCLE_1_X': 241,                 # 圓形1 X座標
            'CIRCLE_1_Y': 242,                 # 圓形1 Y座標
            'CIRCLE_1_RADIUS': 243,            # 圓形1 半徑
            'CIRCLE_2_X': 244,                 # 圓形2 X座標
            'CIRCLE_2_Y': 245,                 # 圓形2 Y座標
            'CIRCLE_2_RADIUS': 246,            # 圓形2 半徑
            'CIRCLE_3_X': 247,                 # 圓形3 X座標
            'CIRCLE_3_Y': 248,                 # 圓形3 Y座標
            'CIRCLE_3_RADIUS': 249,            # 圓形3 半徑
            'CIRCLE_4_X': 250,                 # 圓形4 X座標
            'CIRCLE_4_Y': 251,                 # 圓形4 Y座標
            'CIRCLE_4_RADIUS': 252,            # 圓形4 半徑
            'CIRCLE_5_X': 253,                 # 圓形5 X座標
            'CIRCLE_5_Y': 254,                 # 圓形5 Y座標
            'CIRCLE_5_RADIUS': 255,            # 圓形5 半徑
            
            # 統計資訊寄存器 (280-299) - 寫入統計到外部PLC
            'LAST_CAPTURE_TIME': 280,          # 最後拍照耗時 (ms)
            'LAST_PROCESS_TIME': 281,          # 最後處理耗時 (ms)
            'LAST_TOTAL_TIME': 282,            # 最後總耗時 (ms)
            'OPERATION_COUNT': 283,            # 操作計數器
            'ERROR_COUNT': 284,                # 錯誤計數器
            'CONNECTION_COUNT': 285,           # 連接計數器
            'VERSION_MAJOR': 290,              # 軟體版本主版號
            'VERSION_MINOR': 291,              # 軟體版本次版號
            'UPTIME_HOURS': 292,               # 系統運行時間 (小時)
            'UPTIME_MINUTES': 293,             # 系統運行時間 (分鐘)
        }
        
        # 狀態追蹤
        self.last_trigger_states = {}
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        
        # 外部控制狀態
        self.external_control_enabled = False
        self.last_params_hash = None
        
    def set_vision_controller(self, controller):
        """設置視覺控制器引用"""
        self.vision_controller = controller
        
    def set_server_address(self, ip: str, port: int = 502):
        """設置Modbus服務器地址"""
        self.server_ip = ip
        self.server_port = port
        print(f"🔧 Modbus服務器地址設置為: {ip}:{port}")
    
    def connect(self) -> bool:
        """連接到Modbus TCP服務器"""
        if not MODBUS_AVAILABLE:
            print("❌ Modbus Client不可用")
            return False
        
        try:
            if self.client:
                self.client.close()
            
            print(f"🔗 正在連接Modbus TCP服務器: {self.server_ip}:{self.server_port}")
            
            self.client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=self.read_timeout
            )
            
            # 嘗試連接
            if self.client.connect():
                self.connected = True
                self.connection_count += 1
                
                # 寫入初始狀態
                self._write_initial_status()
                
                print(f"✅ Modbus TCP Client連接成功: {self.server_ip}:{self.server_port}")
                return True
            else:
                print(f"❌ Modbus TCP連接失敗: {self.server_ip}:{self.server_port}")
                self.connected = False
                return False
                
        except Exception as e:
            print(f"❌ Modbus TCP連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        # 先停止同步線程
        self.stop_sync()
        
        if self.client and self.connected:
            try:
                # 寫入斷線狀態
                self.write_register('SYSTEM_STATUS', 0)
                self.write_register('CAMERA_CONNECTED', 0)
                
                self.client.close()
                print("🔌 Modbus TCP Client已斷開連接")
            except:
                pass
        
        self.connected = False
        self.client = None
    
    def enable_external_control(self, enable: bool):
        """啟用/禁用外部控制"""
        self.external_control_enabled = enable
        
        if enable and self.connected:
            # 啟用外部控制時開始同步
            self.start_sync()
            print("🔄 外部控制已啟用，開始同步線程")
        else:
            # 禁用外部控制時停止同步
            self.stop_sync()
            print("⏹️ 外部控制已禁用，停止同步線程")
    
    def start_sync(self):
        """啟動同步線程"""
        if self.sync_running:
            return  # 已經在運行
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._sync_loop, daemon=True)
        self.sync_thread.start()
        print("✅ Modbus同步線程已啟動")
    
    def stop_sync(self):
        """停止同步線程"""
        if self.sync_running:
            self.sync_running = False
            if self.sync_thread and self.sync_thread.is_alive():
                self.sync_thread.join(timeout=2.0)  # 等待最多2秒
            print("🛑 Modbus同步線程已停止")
    
    def _sync_loop(self):
        """同步循環 - 在獨立線程中運行"""
        print("🔄 同步線程開始運行...")
        
        while self.sync_running and self.connected:
            try:
                # 1. 檢查外部控制狀態變化
                self._check_external_control_changes()
                
                # 2. 如果外部控制啟用，進行觸發檢測
                if self.external_control_enabled:
                    self._check_all_triggers()
                    self._check_parameter_updates()
                else:
                    # 如果外部控制被禁用，停止同步線程
                    print("⚠️ 外部控制已禁用，同步線程將退出")
                    break
                
                # 3. 定期同步狀態（每1秒一次）
                self.status_sync_counter += 1
                if self.status_sync_counter >= self.status_sync_interval:
                    self._sync_status_to_plc()
                    self._update_uptime()
                    self.status_sync_counter = 0
                
                # 短暫休眠
                time.sleep(self.sync_interval)
                
            except ConnectionException:
                print("❌ Modbus連接中斷，同步線程退出")
                self.connected = False
                break
                
            except Exception as e:
                print(f"❌ 同步線程錯誤: {e}")
                self.error_count += 1
                time.sleep(1.0)  # 錯誤時延長休眠
        
        self.sync_running = False
        print("⏹️ 同步線程已退出")
    
    def _check_all_triggers(self):
        """檢查所有觸發信號 (增加詳細日誌)"""
        try:
            # 檢查拍照觸發
            capture_trigger = self.read_register('CAPTURE_TRIGGER')
            if capture_trigger is not None:
                if (capture_trigger > 0 and 
                    capture_trigger != self.last_trigger_states.get('capture', 0)):
                    
                    print(f"📸 檢測到拍照觸發: {capture_trigger} (上次: {self.last_trigger_states.get('capture', 0)})")
                    self.last_trigger_states['capture'] = capture_trigger
                    self._handle_capture_trigger()
                    # 處理完成後清除觸發信號
                    self.write_register('CAPTURE_TRIGGER', 0)
            
            # 檢查檢測觸發
            detect_trigger = self.read_register('DETECT_TRIGGER')
            if detect_trigger is not None:
                if (detect_trigger > 0 and 
                    detect_trigger != self.last_trigger_states.get('detect', 0)):
                    
                    print(f"🔍 檢測到檢測觸發: {detect_trigger} (上次: {self.last_trigger_states.get('detect', 0)})")
                    self.last_trigger_states['detect'] = detect_trigger
                    self._handle_detect_trigger()
                    # 處理完成後清除觸發信號
                    self.write_register('DETECT_TRIGGER', 0)
            
            # 檢查重置觸發
            reset_trigger = self.read_register('SYSTEM_RESET')
            if reset_trigger is not None:
                if (reset_trigger > 0 and 
                    reset_trigger != self.last_trigger_states.get('reset', 0)):
                    
                    print(f"🔄 檢測到重置觸發: {reset_trigger} (上次: {self.last_trigger_states.get('reset', 0)})")
                    self.last_trigger_states['reset'] = reset_trigger
                    self._handle_reset_trigger()
                    # 處理完成後清除觸發信號
                    self.write_register('SYSTEM_RESET', 0)
                    
        except Exception as e:
            print(f"❌ 檢查觸發信號失敗: {e}")
    
    def get_debug_info(self) -> Dict[str, Any]:
        """獲取調試信息"""
        return {
            'connected': self.connected,
            'external_control_enabled': self.external_control_enabled,
            'sync_running': self.sync_running,
            'sync_thread_alive': self.sync_thread.is_alive() if self.sync_thread else False,
            'last_trigger_states': self.last_trigger_states.copy(),
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'server_address': f"{self.server_ip}:{self.server_port}"
        }
    
    def _check_external_control_changes(self):
        """檢查外部控制狀態變化"""
        try:
            control_value = self.read_register('EXTERNAL_CONTROL_ENABLE')
            if control_value is not None:
                new_state = bool(control_value)
                if new_state != self.external_control_enabled:
                    self.external_control_enabled = new_state
                    print(f"🔄 外部控制狀態同步: {'啟用' if new_state else '停用'}")
        except:
            pass
    
    def _check_all_triggers(self):
        """檢查所有觸發信號"""
        try:
            # 檢查拍照觸發
            capture_trigger = self.read_register('CAPTURE_TRIGGER')
            if (capture_trigger is not None and capture_trigger > 0 and 
                capture_trigger != self.last_trigger_states.get('capture', 0)):
                
                self.last_trigger_states['capture'] = capture_trigger
                self._handle_capture_trigger()
                # 處理完成後清除觸發信號
                self.write_register('CAPTURE_TRIGGER', 0)
            
            # 檢查檢測觸發
            detect_trigger = self.read_register('DETECT_TRIGGER')
            if (detect_trigger is not None and detect_trigger > 0 and 
                detect_trigger != self.last_trigger_states.get('detect', 0)):
                
                self.last_trigger_states['detect'] = detect_trigger
                self._handle_detect_trigger()
                # 處理完成後清除觸發信號
                self.write_register('DETECT_TRIGGER', 0)
            
            # 檢查重置觸發
            reset_trigger = self.read_register('SYSTEM_RESET')
            if (reset_trigger is not None and reset_trigger > 0 and 
                reset_trigger != self.last_trigger_states.get('reset', 0)):
                
                self.last_trigger_states['reset'] = reset_trigger
                self._handle_reset_trigger()
                # 處理完成後清除觸發信號
                self.write_register('SYSTEM_RESET', 0)
                
        except Exception as e:
            print(f"❌ 檢查觸發信號失敗: {e}")
    
    def _sync_status_to_plc(self):
        """同步狀態到PLC"""
        try:
            # 同步系統狀態
            if self.vision_controller and self.vision_controller.is_connected:
                self.write_register('SYSTEM_STATUS', 1)
                self.write_register('CAMERA_CONNECTED', 1)
            else:
                self.write_register('SYSTEM_STATUS', 0)
                self.write_register('CAMERA_CONNECTED', 0)
            
            # 同步計數器
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
        except Exception as e:
            print(f"❌ 同步狀態到PLC失敗: {e}")
    
    def _write_initial_status(self):
        """寫入初始狀態到PLC"""
        try:
            # 版本資訊
            self.write_register('VERSION_MAJOR', 2)
            self.write_register('VERSION_MINOR', 1)
            
            # 系統狀態
            camera_status = 1 if (self.vision_controller and self.vision_controller.is_connected) else 0
            self.write_register('SYSTEM_STATUS', camera_status)
            self.write_register('CAMERA_CONNECTED', camera_status)
            self.write_register('LAST_OPERATION_STATUS', 1)
            
            # 計數器
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            print("📊 初始狀態已寫入PLC")
            
        except Exception as e:
            print(f"❌ 寫入初始狀態失敗: {e}")
    
    def _handle_capture_trigger(self):
        """處理拍照觸發 (在同步線程中執行)"""
        if not self.vision_controller:
            return
        
        try:
            self.write_register('PROCESSING_PROGRESS', 50)
            print("📸 外部觸發: 執行拍照")
            
            image, capture_time = self.vision_controller.capture_image()
            
            if image is not None:
                self.write_register('LAST_OPERATION_STATUS', 1)
                self.write_register('LAST_CAPTURE_TIME', int(capture_time * 1000))
                print(f"✅ 拍照成功，耗時: {capture_time*1000:.2f}ms")
            else:
                self.write_register('LAST_OPERATION_STATUS', 0)
                self.error_count += 1
                print("❌ 拍照失敗")
            
            self.operation_count += 1
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('PROCESSING_PROGRESS', 100)
            
        except Exception as e:
            print(f"❌ 處理拍照觸發失敗: {e}")
            self.write_register('LAST_OPERATION_STATUS', 0)
            self.write_register('PROCESSING_PROGRESS', 0)
            self.error_count += 1
    
    def _handle_detect_trigger(self):
        """處理檢測觸發 (在同步線程中執行)"""
        if not self.vision_controller:
            return
        
        try:
            self.write_register('PROCESSING_PROGRESS', 20)
            print("🔍 外部觸發: 執行拍照+檢測")
            
            result = self.vision_controller.capture_and_detect()
            
            if result.success:
                # 更新檢測結果到PLC
                self.update_detection_results(result)
                self.write_register('LAST_OPERATION_STATUS', 1)
                print(f"✅ 檢測成功，找到 {result.circle_count} 個圓形")
            else:
                self.write_register('LAST_OPERATION_STATUS', 0)
                self.error_count += 1
                print(f"❌ 檢測失敗: {result.error_message}")
            
            self.operation_count += 1
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('PROCESSING_PROGRESS', 100)
            
        except Exception as e:
            print(f"❌ 處理檢測觸發失敗: {e}")
            self.write_register('LAST_OPERATION_STATUS', 0)
            self.write_register('PROCESSING_PROGRESS', 0)
            self.error_count += 1
    
    def _handle_reset_trigger(self):
        """處理重置觸發 (在同步線程中執行)"""
        try:
            print("🔄 外部觸發: 系統重置")
            
            # 重置計數器
            self.operation_count = 0
            self.error_count = 0
            
            # 清空檢測結果
            self.write_register('CIRCLE_COUNT', 0)
            for i in range(1, 6):
                self.write_register(f'CIRCLE_{i}_X', 0)
                self.write_register(f'CIRCLE_{i}_Y', 0)
                self.write_register(f'CIRCLE_{i}_RADIUS', 0)
            
            # 更新計數器
            self.write_register('OPERATION_COUNT', 0)
            self.write_register('ERROR_COUNT', 0)
            
            print("✅ 系統重置完成")
            
        except Exception as e:
            print(f"❌ 處理重置觸發失敗: {e}")
    
    def _handle_parameter_update(self):
        """處理參數更新觸發 (在同步線程中執行)"""
        if not self.vision_controller:
            return
        
        try:
            print("📊 外部觸發: 參數更新")
            
            # 讀取新參數
            area_high = self.read_register('MIN_AREA_HIGH') or 0
            area_low = self.read_register('MIN_AREA_LOW') or 50000
            min_area = (area_high << 16) + area_low
            
            roundness_int = self.read_register('MIN_ROUNDNESS') or 800
            min_roundness = roundness_int / 1000.0
            
            gaussian_kernel = self.read_register('GAUSSIAN_KERNEL') or 9
            canny_low = self.read_register('CANNY_LOW') or 20
            canny_high = self.read_register('CANNY_HIGH') or 60
            
            # 更新視覺控制器參數
            self.vision_controller.update_detection_params(
                min_area=min_area,
                min_roundness=min_roundness,
                gaussian_kernel=gaussian_kernel,
                canny_low=canny_low,
                canny_high=canny_high
            )
            
            print(f"✅ 參數更新完成: 面積>={min_area}, 圓度>={min_roundness}")
            
        except Exception as e:
            print(f"❌ 處理參數更新失敗: {e}")
    
    def _check_parameter_updates(self):
        """檢查參數更新 (在同步線程中執行)"""
        try:
            param_trigger = self.read_register('PARAM_UPDATE_TRIGGER')
            if (param_trigger is not None and param_trigger > 0 and 
                param_trigger != self.last_trigger_states.get('param', 0)):
                
                self.last_trigger_states['param'] = param_trigger
                self._handle_parameter_update()
                # 處理完成後清除觸發信號
                self.write_register('PARAM_UPDATE_TRIGGER', 0)
                
        except Exception as e:
            print(f"❌ 檢查參數更新失敗: {e}")
    
    def _update_uptime(self):
        """更新運行時間 (在同步線程中執行)"""
        try:
            uptime_total_minutes = int((time.time() - self.start_time) / 60)
            uptime_hours = uptime_total_minutes // 60
            uptime_minutes = uptime_total_minutes % 60
            
            self.write_register('UPTIME_HOURS', uptime_hours)
            self.write_register('UPTIME_MINUTES', uptime_minutes)
        except:
            pass
    def start_monitoring(self):
        """啟動基礎監控 (已棄用，改為同步線程)"""
        print("⚠️ start_monitoring已棄用，請使用start_sync")
        return True
    
    def stop_monitoring(self):
        """停止基礎監控 (已棄用，改為同步線程)"""
        print("⚠️ stop_monitoring已棄用，請使用stop_sync")
    
    def _monitor_loop(self):
        """舊監控循環 (已棄用)"""
        pass
    
    def _monitor_loop(self):
        """舊監控循環 (已棄用)"""
        pass
    
    def _update_uptime(self):
        """更新運行時間"""
        try:
            uptime_total_minutes = int((time.time() - self.start_time) / 60)
            uptime_hours = uptime_total_minutes // 60
            uptime_minutes = uptime_total_minutes % 60
            
            self.write_register('UPTIME_HOURS', uptime_hours)
            self.write_register('UPTIME_MINUTES', uptime_minutes)
        except:
            pass
    
    def _check_external_control(self):
        """檢查外部控制狀態 (已棄用)"""
        pass
    
    def _check_triggers(self):
        """檢查觸發信號 (已棄用)"""
        pass
    
    def _check_parameter_updates(self):
        """檢查參數更新"""
        try:
            param_trigger = self.read_register('PARAM_UPDATE_TRIGGER')
            if (param_trigger is not None and param_trigger > 0 and 
                param_trigger != self.last_trigger_states.get('param', 0)):
                
                self.last_trigger_states['param'] = param_trigger
                self._handle_parameter_update()
                # 處理完成後清除觸發信號
                self.write_register('PARAM_UPDATE_TRIGGER', 0)
                
        except Exception as e:
            print(f"❌ 檢查參數更新失敗: {e}")
    
    def _handle_detect_trigger(self):
        """處理檢測觸發 (在同步線程中執行)"""
        if not self.vision_controller:
            print("❌ 檢測觸發失敗: vision_controller 不存在")
            self.error_count += 1
            return
        
        try:
            print("🔍 外部觸發: 開始執行拍照+檢測")
            self.write_register('PROCESSING_PROGRESS', 20)
            
            # 檢查相機連接狀態
            if not self.vision_controller.is_connected:
                print("❌ 檢測觸發失敗: 相機未連接")
                self.write_register('LAST_OPERATION_STATUS', 0)
                self.write_register('PROCESSING_PROGRESS', 0)
                self.error_count += 1
                return
            
            print("📸 正在執行拍照+檢測...")
            result = self.vision_controller.capture_and_detect()
            
            if result and result.success:
                # 更新檢測結果到PLC
                print(f"✅ 檢測成功，找到 {result.circle_count} 個圓形")
                self.update_detection_results(result)
                self.write_register('LAST_OPERATION_STATUS', 1)
                self.write_register('PROCESSING_PROGRESS', 100)
            else:
                error_msg = result.error_message if result else "檢測結果為空"
                print(f"❌ 檢測失敗: {error_msg}")
                self.write_register('LAST_OPERATION_STATUS', 0)
                self.write_register('PROCESSING_PROGRESS', 0)
                self.error_count += 1
            
            self.operation_count += 1
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            
        except Exception as e:
            print(f"❌ 處理檢測觸發異常: {e}")
            print(f"❌ 異常類型: {type(e).__name__}")
            import traceback
            print(f"❌ 詳細堆疊: {traceback.format_exc()}")
            
            self.write_register('LAST_OPERATION_STATUS', 0)
            self.write_register('PROCESSING_PROGRESS', 0)
            self.error_count += 1
            self.write_register('ERROR_COUNT', self.error_count)
    
    def _handle_capture_trigger(self):
        """處理拍照觸發 (在同步線程中執行)"""
        if not self.vision_controller:
            print("❌ 拍照觸發失敗: vision_controller 不存在")
            self.error_count += 1
            return
        
        try:
            print("📸 外部觸發: 開始執行拍照")
            self.write_register('PROCESSING_PROGRESS', 50)
            
            # 檢查相機連接狀態
            if not self.vision_controller.is_connected:
                print("❌ 拍照觸發失敗: 相機未連接")
                self.write_register('LAST_OPERATION_STATUS', 0)
                self.write_register('PROCESSING_PROGRESS', 0)
                self.error_count += 1
                return
            
            print("📸 正在執行拍照...")
            image, capture_time = self.vision_controller.capture_image()
            
            if image is not None:
                self.write_register('LAST_OPERATION_STATUS', 1)
                self.write_register('LAST_CAPTURE_TIME', int(capture_time * 1000))
                self.write_register('PROCESSING_PROGRESS', 100)
                print(f"✅ 拍照成功，耗時: {capture_time*1000:.2f}ms")
            else:
                self.write_register('LAST_OPERATION_STATUS', 0)
                self.write_register('PROCESSING_PROGRESS', 0)
                self.error_count += 1
                print("❌ 拍照失敗: 返回圖像為空")
            
            self.operation_count += 1
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            
        except Exception as e:
            print(f"❌ 處理拍照觸發異常: {e}")
            print(f"❌ 異常類型: {type(e).__name__}")
            import traceback
            print(f"❌ 詳細堆疊: {traceback.format_exc()}")
            
            self.write_register('LAST_OPERATION_STATUS', 0)
            self.write_register('PROCESSING_PROGRESS', 0)
            self.error_count += 1
            self.write_register('ERROR_COUNT', self.error_count)
    
    def _handle_reset_trigger(self):
        """處理重置觸發"""
        try:
            print("🔄 外部觸發: 系統重置")
            
            # 重置計數器
            self.operation_count = 0
            self.error_count = 0
            
            # 清空檢測結果
            self.write_register('CIRCLE_COUNT', 0)
            for i in range(1, 6):
                self.write_register(f'CIRCLE_{i}_X', 0)
                self.write_register(f'CIRCLE_{i}_Y', 0)
                self.write_register(f'CIRCLE_{i}_RADIUS', 0)
            
            # 更新計數器
            self.write_register('OPERATION_COUNT', 0)
            self.write_register('ERROR_COUNT', 0)
            
            print("✅ 系統重置完成")
            
        except Exception as e:
            print(f"❌ 處理重置觸發失敗: {e}")
    
    def _handle_parameter_update(self):
        """處理參數更新觸發"""
        if not self.vision_controller:
            return
        
        try:
            print("📊 外部觸發: 參數更新")
            
            # 讀取新參數
            area_high = self.read_register('MIN_AREA_HIGH') or 0
            area_low = self.read_register('MIN_AREA_LOW') or 50000
            min_area = (area_high << 16) + area_low
            
            roundness_int = self.read_register('MIN_ROUNDNESS') or 800
            min_roundness = roundness_int / 1000.0
            
            gaussian_kernel = self.read_register('GAUSSIAN_KERNEL') or 9
            canny_low = self.read_register('CANNY_LOW') or 20
            canny_high = self.read_register('CANNY_HIGH') or 60
            
            # 更新視覺控制器參數
            self.vision_controller.update_detection_params(
                min_area=min_area,
                min_roundness=min_roundness,
                gaussian_kernel=gaussian_kernel,
                canny_low=canny_low,
                canny_high=canny_high
            )
            
            print(f"✅ 參數更新完成: 面積>={min_area}, 圓度>={min_roundness}")
            
        except Exception as e:
            print(f"❌ 處理參數更新失敗: {e}")
    
    def _update_system_status(self):
        """更新系統狀態到PLC (已棄用)"""
        pass
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器 (pymodbus 3.x 語法修正)"""
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            # pymodbus 3.x: 使用關鍵字參數
            result = self.client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                print(f"❌ 讀取寄存器失敗 {register_name}: {result}")
                return None
                
        except Exception as e:
            print(f"❌ 讀取寄存器異常 {register_name}: {e}")
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器 (pymodbus 3.x 語法修正)"""
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            # pymodbus 3.x: 使用關鍵字參數
            result = self.client.write_register(address, value, slave=1)
            
            if not result.isError():
                return True
            else:
                print(f"❌ 寫入寄存器失敗 {register_name}: {result}")
                return False
                
        except Exception as e:
            print(f"❌ 寫入寄存器異常 {register_name}: {e}")
            return False
    
    def read_multiple_registers(self, start_address: int, count: int) -> Optional[List[int]]:
        """讀取多個連續寄存器 (pymodbus 3.x 語法修正)"""
        if not self.connected or not self.client:
            return None
        
        try:
            # pymodbus 3.x: 使用關鍵字參數
            result = self.client.read_holding_registers(start_address, count=count, slave=1)
            
            if not result.isError():
                return result.registers
            else:
                print(f"❌ 讀取多個寄存器失敗: {result}")
                return None
                
        except Exception as e:
            print(f"❌ 讀取多個寄存器異常: {e}")
            return None
    
    def write_multiple_registers(self, start_address: int, values: List[int]) -> bool:
        """寫入多個連續寄存器 (pymodbus 3.x 語法修正)"""
        if not self.connected or not self.client:
            return False
        
        try:
            # pymodbus 3.x: 使用關鍵字參數
            result = self.client.write_registers(start_address, values, slave=1)
            
            if not result.isError():
                return True
            else:
                print(f"❌ 寫入多個寄存器失敗: {result}")
                return False
                
        except Exception as e:
            print(f"❌ 寫入多個寄存器異常: {e}")
            return False
    
    def update_detection_results(self, result: VisionResult):
        """更新檢測結果到PLC"""
        try:
            # 寫入圓形數量
            self.write_register('CIRCLE_COUNT', result.circle_count)
            
            # 寫入圓形座標和半徑 (最多5個)
            for i in range(5):
                if i < len(result.circles):
                    circle = result.circles[i]
                    self.write_register(f'CIRCLE_{i+1}_X', int(circle['center'][0]))
                    self.write_register(f'CIRCLE_{i+1}_Y', int(circle['center'][1]))
                    self.write_register(f'CIRCLE_{i+1}_RADIUS', int(circle['radius']))
                else:
                    # 清空未使用的寄存器
                    self.write_register(f'CIRCLE_{i+1}_X', 0)
                    self.write_register(f'CIRCLE_{i+1}_Y', 0)
                    self.write_register(f'CIRCLE_{i+1}_RADIUS', 0)
            
            # 寫入時間統計
            self.write_register('LAST_CAPTURE_TIME', int(result.capture_time * 1000))
            self.write_register('LAST_PROCESS_TIME', int(result.processing_time * 1000))
            self.write_register('LAST_TOTAL_TIME', int(result.total_time * 1000))
            
        except Exception as e:
            print(f"❌ 更新檢測結果到PLC失敗: {e}")
    
    def get_connection_status(self) -> Dict[str, Any]:
        """獲取連接狀態"""
        return {
            'connected': self.connected,
            'server_ip': self.server_ip,
            'server_port': self.server_port,
            'external_control_enabled': self.external_control_enabled,
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'connection_count': self.connection_count,
            'uptime_seconds': int(time.time() - self.start_time)
        }


class MockModbusTcpClientService:
    """Modbus TCP Client的模擬實現 (當pymodbus不可用時使用)"""
    
    def __init__(self, server_ip="192.168.1.100", server_port=502):
        self.server_ip = server_ip
        self.server_port = server_port
        self.connected = False
        self.running = False
        self.vision_controller = None
        
        # 模擬寄存器存儲
        self.registers = {}
        
        # 同步控制 (與真實版本相同)
        self.sync_enabled = False
        self.sync_thread = None
        self.sync_running = False
        self.sync_interval = 0.1
        self.status_sync_counter = 0
        self.status_sync_interval = 10
    
    def get_debug_info(self) -> Dict[str, Any]:
        """獲取調試信息 (模擬版本)"""
        return {
            'connected': self.connected,
            'external_control_enabled': self.external_control_enabled,
            'sync_running': self.sync_running,
            'sync_thread_alive': self.sync_thread.is_alive() if self.sync_thread else False,
            'last_trigger_states': {},
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'server_address': f"{self.server_ip}:{self.server_port}",
            'mock_mode': True
        }
        
        # 寄存器映射 (與真實版本相同)
        self.REGISTERS = {
            'EXTERNAL_CONTROL_ENABLE': 200,
            'CAPTURE_TRIGGER': 201,
            'DETECT_TRIGGER': 202,
            'SYSTEM_RESET': 203,
            'PARAM_UPDATE_TRIGGER': 204,
            'MIN_AREA_HIGH': 210,
            'MIN_AREA_LOW': 211,
            'MIN_ROUNDNESS': 212,
            'GAUSSIAN_KERNEL': 213,
            'CANNY_LOW': 214,
            'CANNY_HIGH': 215,
            'SYSTEM_STATUS': 220,
            'CAMERA_CONNECTED': 221,
            'LAST_OPERATION_STATUS': 222,
            'PROCESSING_PROGRESS': 223,
            'CIRCLE_COUNT': 240,
            'CIRCLE_1_X': 241,
            'CIRCLE_1_Y': 242,
            'CIRCLE_1_RADIUS': 243,
            'CIRCLE_2_X': 244,
            'CIRCLE_2_Y': 245,
            'CIRCLE_2_RADIUS': 246,
            'CIRCLE_3_X': 247,
            'CIRCLE_3_Y': 248,
            'CIRCLE_3_RADIUS': 249,
            'CIRCLE_4_X': 250,
            'CIRCLE_4_Y': 251,
            'CIRCLE_4_RADIUS': 252,
            'CIRCLE_5_X': 253,
            'CIRCLE_5_Y': 254,
            'CIRCLE_5_RADIUS': 255,
            'LAST_CAPTURE_TIME': 280,
            'LAST_PROCESS_TIME': 281,
            'LAST_TOTAL_TIME': 282,
            'OPERATION_COUNT': 283,
            'ERROR_COUNT': 284,
            'CONNECTION_COUNT': 285,
            'VERSION_MAJOR': 290,
            'VERSION_MINOR': 291,
            'UPTIME_HOURS': 292,
            'UPTIME_MINUTES': 293,
        }
        
        # 初始化寄存器
        for name, address in self.REGISTERS.items():
            self.registers[address] = 0
        
        # 狀態追蹤
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        self.external_control_enabled = False
    
    def set_vision_controller(self, controller):
        self.vision_controller = controller
    
    def set_server_address(self, ip: str, port: int = 502):
        self.server_ip = ip
        self.server_port = port
        print(f"⚠️ 模擬模式: Modbus服務器地址設置為: {ip}:{port}")
    
    def connect(self) -> bool:
        print(f"⚠️ 模擬連接到Modbus TCP服務器: {self.server_ip}:{self.server_port}")
        self.connected = True
        self.connection_count += 1
        
        # 設置初始值
        self.write_register('VERSION_MAJOR', 2)
        self.write_register('VERSION_MINOR', 1)
        self.write_register('MIN_AREA_LOW', 50000)
        self.write_register('MIN_ROUNDNESS', 800)
        
        return True
    
    def disconnect(self):
        print("⚠️ 模擬Modbus TCP Client已斷開連接")
        self.stop_sync()
        self.connected = False
    
    def enable_external_control(self, enable: bool):
        """啟用/禁用外部控制 (模擬版本)"""
        self.external_control_enabled = enable
        
        if enable and self.connected:
            self.start_sync()
            print("⚠️ 模擬外部控制已啟用，開始模擬同步")
        else:
            self.stop_sync()
            print("⚠️ 模擬外部控制已禁用，停止模擬同步")
    
    def start_sync(self):
        """啟動模擬同步線程"""
        if self.sync_running:
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._mock_sync_loop, daemon=True)
        self.sync_thread.start()
        print("⚠️ 模擬同步線程已啟動")
    
    def stop_sync(self):
        """停止模擬同步線程"""
        if self.sync_running:
            self.sync_running = False
            if self.sync_thread and self.sync_thread.is_alive():
                self.sync_thread.join(timeout=1.0)
            print("⚠️ 模擬同步線程已停止")
    
    def _mock_sync_loop(self):
        """模擬同步循環"""
        print("⚠️ 模擬同步線程開始運行...")
        
        while self.sync_running and self.connected:
            try:
                # 模擬基本的狀態更新
                if self.vision_controller and self.vision_controller.is_connected:
                    self.write_register('SYSTEM_STATUS', 1)
                    self.write_register('CAMERA_CONNECTED', 1)
                else:
                    self.write_register('SYSTEM_STATUS', 0)
                    self.write_register('CAMERA_CONNECTED', 0)
                
                # 更新計數器
                self.write_register('OPERATION_COUNT', self.operation_count)
                self.write_register('ERROR_COUNT', self.error_count)
                
                time.sleep(self.sync_interval)
                
            except Exception as e:
                print(f"⚠️ 模擬同步錯誤: {e}")
                time.sleep(1.0)
        
        print("⚠️ 模擬同步線程已退出")
    
    def start_monitoring(self):
        print("⚠️ 模擬start_monitoring，請使用start_sync")
        return True
    
    def stop_monitoring(self):
        print("⚠️ 模擬stop_monitoring，請使用stop_sync")
    
    def read_register(self, register_name: str) -> Optional[int]:
        if register_name in self.REGISTERS:
            address = self.REGISTERS[register_name]
            return self.registers.get(address, 0)
        return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        if register_name in self.REGISTERS:
            address = self.REGISTERS[register_name]
            self.registers[address] = value
            return True
        return False
    
    def read_multiple_registers(self, start_address: int, count: int) -> Optional[List[int]]:
        return [self.registers.get(start_address + i, 0) for i in range(count)]
    
    def write_multiple_registers(self, start_address: int, values: List[int]) -> bool:
        for i, value in enumerate(values):
            self.registers[start_address + i] = value
        return True
    
    def update_detection_results(self, result: VisionResult):
        self.write_register('CIRCLE_COUNT', result.circle_count)
        
        for i in range(5):
            if i < len(result.circles):
                circle = result.circles[i]
                self.write_register(f'CIRCLE_{i+1}_X', int(circle['center'][0]))
                self.write_register(f'CIRCLE_{i+1}_Y', int(circle['center'][1]))
                self.write_register(f'CIRCLE_{i+1}_RADIUS', int(circle['radius']))
            else:
                self.write_register(f'CIRCLE_{i+1}_X', 0)
                self.write_register(f'CIRCLE_{i+1}_Y', 0)
                self.write_register(f'CIRCLE_{i+1}_RADIUS', 0)
        
        self.write_register('LAST_CAPTURE_TIME', int(result.capture_time * 1000))
        self.write_register('LAST_PROCESS_TIME', int(result.processing_time * 1000))
        self.write_register('LAST_TOTAL_TIME', int(result.total_time * 1000))
    
    def get_connection_status(self) -> Dict[str, Any]:
        return {
            'connected': self.connected,
            'server_ip': self.server_ip,
            'server_port': self.server_port,
            'external_control_enabled': self.external_control_enabled,
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'connection_count': self.connection_count,
            'uptime_seconds': int(time.time() - self.start_time),
            'mock_mode': True
        }


class CircleDetector:
    """圓形檢測器"""
    
    def __init__(self, params: DetectionParams = None):
        self.params = params or DetectionParams()
    
    def update_params(self, params: DetectionParams):
        """更新檢測參數"""
        self.params = params
    
    def is_circle(self, contour, tolerance=0.2):
        """判斷輪廓是否為圓形"""
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        return 1 - tolerance < circularity < 1 + tolerance
    
    def detect_circles(self, image: np.ndarray) -> Tuple[List[Dict], np.ndarray]:
        """檢測圓形並返回結果和標註圖像"""
        if image is None:
            return [], None
        
        try:
            # 確保是灰度圖像
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image.copy()
            
            # 創建彩色輸出圖像
            if len(image.shape) == 3:
                result_image = image.copy()
            else:
                result_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            
            # 使用參數進行處理
            kernel_size = (self.params.gaussian_kernel_size, self.params.gaussian_kernel_size)
            blurred = cv2.GaussianBlur(gray, kernel_size, self.params.gaussian_sigma)
            
            # Canny 邊緣檢測
            edges = cv2.Canny(blurred, self.params.canny_low, self.params.canny_high)
            
            # 輪廓檢測
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            circles = []
            circle_id = 1
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # 使用設定的參數進行篩選
                if area < self.params.min_area:
                    continue
                
                # 計算圓度
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                    
                roundness = (4 * np.pi * area) / (perimeter ** 2)
                
                # 檢查圓度條件
                if roundness < self.params.min_roundness:
                    continue
                
                if self.is_circle(contour):
                    # 計算圓心和半徑
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    
                    circle_info = {
                        'id': circle_id,
                        'center': center,
                        'radius': radius,
                        'area': float(area),
                        'roundness': float(roundness)
                    }
                    circles.append(circle_info)
                    
                    # 在圖像上繪製圓形和編號
                    cv2.circle(result_image, center, radius, (0, 255, 0), 3)  # 綠色圓圈
                    cv2.circle(result_image, center, 5, (0, 0, 255), -1)      # 紅色圓心
                    
                    # 繪製編號
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 2.0
                    thickness = 3
                    text = str(circle_id)
                    
                    # 計算文字大小和位置
                    (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
                    text_x = center[0] - text_width // 2
                    text_y = center[1] - radius - 10
                    
                    # 確保文字不會超出圖像邊界
                    text_x = max(10, min(text_x, result_image.shape[1] - text_width - 10))
                    text_y = max(text_height + 10, min(text_y, result_image.shape[0] - 10))
                    
                    # 繪製文字背景
                    cv2.rectangle(result_image, 
                                (text_x - 5, text_y - text_height - 5),
                                (text_x + text_width + 5, text_y + 5),
                                (255, 255, 255), -1)
                    
                    # 繪製文字
                    cv2.putText(result_image, text, (text_x, text_y), 
                              font, font_scale, (0, 0, 0), thickness)
                    
                    circle_id += 1
            
            return circles, result_image
            
        except Exception as e:
            print(f"圓形檢測失敗: {e}")
            return [], image


class CCD1VisionController:
    """CCD1 視覺控制器 (Modbus TCP Client版本)"""
    
    def __init__(self):
        self.camera_manager: Optional[OptimizedCameraManager] = None
        self.detection_params = DetectionParams()
        self.detector = CircleDetector(self.detection_params)
        self.camera_name = "cam_1"
        self.camera_ip = "192.168.1.8"
        self.is_connected = False
        self.last_image: Optional[np.ndarray] = None
        self.last_result: Optional[VisionResult] = None
        self.lock = threading.Lock()
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1Vision")
        self.logger.setLevel(logging.INFO)
        
        # 選擇合適的Modbus Client服務
        if MODBUS_AVAILABLE:
            self.modbus_client = ModbusTcpClientService()
            print("✅ 使用完整Modbus TCP Client服務")
        else:
            self.modbus_client = MockModbusTcpClientService()
            print("⚠️ 使用模擬Modbus TCP Client服務 (功能受限)")
            
        self.modbus_client.set_vision_controller(self)
        
        # 初始化相機配置
        self.camera_config = CameraConfig(
            name=self.camera_name,
            ip=self.camera_ip,
            exposure_time=20000.0,
            gain=200.0,
            frame_rate=30.0,
            pixel_format=PixelFormat.BAYER_GR8,
            width=2592,
            height=1944,
            trigger_mode=CameraMode.CONTINUOUS,
            auto_reconnect=True
        )
    
    def set_modbus_server(self, ip: str, port: int = 502) -> Dict[str, Any]:
        """設置Modbus服務器地址"""
        try:
            # 如果已連接，先斷開
            if self.modbus_client.connected:
                self.modbus_client.stop_monitoring()
                self.modbus_client.disconnect()
            
            # 設置新地址
            self.modbus_client.set_server_address(ip, port)
            
            return {
                'success': True,
                'message': f'Modbus服務器地址已設置: {ip}:{port}',
                'server_ip': ip,
                'server_port': port
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'設置Modbus服務器地址失敗: {str(e)}'
            }
    
    def connect_modbus(self) -> Dict[str, Any]:
        """連接Modbus TCP服務器"""
        try:
            if self.modbus_client.connect():
                # 連接成功後，檢查外部控制狀態並啟動同步
                # 讀取當前外部控制狀態
                current_control = self.modbus_client.read_register('EXTERNAL_CONTROL_ENABLE')
                if current_control == 1:
                    # 如果PLC端已經啟用外部控制，則自動啟動同步
                    self.modbus_client.enable_external_control(True)
                    print("🔄 檢測到PLC端外部控制已啟用，自動啟動同步線程")
                
                return {
                    'success': True,
                    'message': f'Modbus TCP連接成功: {self.modbus_client.server_ip}:{self.modbus_client.server_port}',
                    'connection_status': self.modbus_client.get_connection_status(),
                    'auto_sync_started': current_control == 1
                }
            else:
                return {
                    'success': False,
                    'message': f'無法連接到Modbus服務器: {self.modbus_client.server_ip}:{self.modbus_client.server_port}'
                }
                
        except Exception as e:
            return {
                'success': False,
                'message': f'Modbus連接異常: {str(e)}'
            }
    
    def disconnect_modbus(self) -> Dict[str, Any]:
        """斷開Modbus連接"""
        try:
            self.modbus_client.disconnect()  # 這會自動停止同步線程
            
            return {
                'success': True,
                'message': 'Modbus連接已斷開，同步線程已停止'
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'斷開Modbus連接失敗: {str(e)}'
            }
    
    def update_detection_params(self, min_area: float = None, min_roundness: float = None, 
                              gaussian_kernel: int = None, canny_low: int = None, canny_high: int = None):
        """更新檢測參數"""
        if min_area is not None:
            self.detection_params.min_area = min_area
        if min_roundness is not None:
            self.detection_params.min_roundness = min_roundness
        if gaussian_kernel is not None:
            self.detection_params.gaussian_kernel_size = gaussian_kernel
        if canny_low is not None:
            self.detection_params.canny_low = canny_low
        if canny_high is not None:
            self.detection_params.canny_high = canny_high
            
        self.detector.update_params(self.detection_params)
        
        self.logger.info(f"檢測參數已更新: 面積>={self.detection_params.min_area}, 圓度>={self.detection_params.min_roundness}")
    
    def initialize_camera(self, ip_address: str = None) -> Dict[str, Any]:
        """初始化相機連接"""
        try:
            if ip_address:
                self.camera_ip = ip_address
                self.camera_config.ip = ip_address
            
            self.logger.info(f"正在初始化相機 {self.camera_name} (IP: {self.camera_ip})")
            
            if self.camera_manager:
                self.camera_manager.shutdown()
            
            self.camera_manager = OptimizedCameraManager()
            
            success = self.camera_manager.add_camera(self.camera_name, self.camera_config)
            if not success:
                raise Exception("添加相機失敗")
            
            connect_result = self.camera_manager.connect_camera(self.camera_name)
            if not connect_result:
                raise Exception("相機連接失敗")
            
            stream_result = self.camera_manager.start_streaming([self.camera_name])
            if not stream_result.get(self.camera_name, False):
                raise Exception("開始串流失敗")
            
            # 設置增益為200
            camera = self.camera_manager.cameras[self.camera_name]
            camera.camera.MV_CC_SetFloatValue("Gain", 200.0)
            
            self.is_connected = True
            self.logger.info(f"相機 {self.camera_name} 初始化成功")
            
            return {
                'success': True,
                'message': f'相機 {self.camera_name} 連接成功',
                'camera_ip': self.camera_ip,
                'gain_set': 200.0
            }
            
        except Exception as e:
            self.is_connected = False
            error_msg = f"相機初始化失敗: {str(e)}"
            self.logger.error(error_msg)
            
            return {
                'success': False,
                'message': error_msg,
                'camera_ip': self.camera_ip
            }
    
    def capture_image(self) -> Tuple[Optional[np.ndarray], float]:
        """捕獲圖像"""
        if not self.is_connected or not self.camera_manager:
            return None, 0.0
        
        capture_start = time.time()
        
        try:
            frame_data = self.camera_manager.get_image_data(self.camera_name, timeout=3000)
            
            if frame_data is None:
                return None, 0.0
            
            capture_time = time.time() - capture_start
            
            image_array = frame_data.data
            
            if len(image_array.shape) == 2:
                display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
            else:
                display_image = image_array
            
            self.last_image = display_image
            return display_image, capture_time
            
        except Exception as e:
            self.logger.error(f"捕獲圖像失敗: {e}")
            return None, 0.0
    
    def capture_and_detect(self) -> VisionResult:
        """拍照並進行圓形檢測"""
        total_start = time.time()
        
        try:
            image, capture_time = self.capture_image()
            
            if image is None:
                result = VisionResult(
                    circle_count=0,
                    circles=[],
                    processing_time=0.0,
                    capture_time=capture_time,
                    total_time=time.time() - total_start,
                    timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    success=False,
                    error_message="圖像捕獲失敗"
                )
            else:
                process_start = time.time()
                circles, annotated_image = self.detector.detect_circles(image)
                processing_time = time.time() - process_start
                total_time = time.time() - total_start
                
                self.last_image = annotated_image
                
                result = VisionResult(
                    circle_count=len(circles),
                    circles=circles,
                    processing_time=processing_time,
                    capture_time=capture_time,
                    total_time=total_time,
                    timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    success=True
                )
            
            self.last_result = result
            
            # 更新Modbus結果 (如果連接)
            if self.modbus_client.connected:
                self.modbus_client.update_detection_results(result)
            
            return result
            
        except Exception as e:
            error_msg = f"檢測失敗: {str(e)}"
            self.logger.error(error_msg)
            
            result = VisionResult(
                circle_count=0,
                circles=[],
                processing_time=0.0,
                capture_time=0.0,
                total_time=time.time() - total_start,
                timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                success=False,
                error_message=error_msg
            )
            
            if self.modbus_client.connected:
                self.modbus_client.update_detection_results(result)
                
            return result
    
    def get_image_base64(self) -> Optional[str]:
        """獲取當前圖像的base64編碼"""
        if self.last_image is None:
            return None
        
        try:
            height, width = self.last_image.shape[:2]
            if width > 800:
                scale = 800 / width
                new_width = 800
                new_height = int(height * scale)
                display_image = cv2.resize(self.last_image, (new_width, new_height))
            else:
                display_image = self.last_image
            
            _, buffer = cv2.imencode('.jpg', display_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            return f"data:image/jpeg;base64,{image_base64}"
            
        except Exception as e:
            self.logger.error(f"圖像編碼失敗: {e}")
            return None
    
    def get_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        status = {
            'connected': self.is_connected,
            'camera_name': self.camera_name,
            'camera_ip': self.camera_ip,
            'has_image': self.last_image is not None,
            'last_result': asdict(self.last_result) if self.last_result else None,
            'detection_params': asdict(self.detection_params),
            'modbus_enabled': MODBUS_AVAILABLE,
            'modbus_connection': self.modbus_client.get_connection_status()
        }
        
        if self.camera_manager and self.is_connected:
            try:
                stats = self.camera_manager.get_camera_statistics(self.camera_name)
                status['camera_stats'] = stats
            except:
                pass
        
        return status
    
    def disconnect(self):
        """斷開所有連接"""
        # 斷開相機連接
        if self.camera_manager:
            self.camera_manager.shutdown()
            self.camera_manager = None
        
        self.is_connected = False
        self.last_image = None
        
        # 斷開Modbus連接
        try:
            self.modbus_client.stop_monitoring()
            self.modbus_client.disconnect()
        except:
            pass
        
        self.logger.info("所有連接已斷開")


# Flask應用設置
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ccd_vision_control_client_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# 創建控制器實例
vision_controller = CCD1VisionController()

# 設置日誌
logging.basicConfig(level=logging.INFO)


@app.route('/')
def index():
    """主頁面"""
    return render_template('ccd_vision_client.html')


@app.route('/api/status')
def get_status():
    """獲取系統狀態"""
    return jsonify(vision_controller.get_status())


@app.route('/api/modbus/set_server', methods=['POST'])
def set_modbus_server():
    """設置Modbus服務器地址"""
    data = request.get_json()
    ip = data.get('ip', '192.168.1.100')
    port = data.get('port', 502)
    
    result = vision_controller.set_modbus_server(ip, port)
    socketio.emit('status_update', vision_controller.get_status())
    
    return jsonify(result)


@app.route('/api/modbus/connect', methods=['POST'])
def connect_modbus():
    """連接Modbus TCP服務器"""
    result = vision_controller.connect_modbus()
    socketio.emit('status_update', vision_controller.get_status())
    
    return jsonify(result)


@app.route('/api/modbus/disconnect', methods=['POST'])
def disconnect_modbus():
    """斷開Modbus連接"""
    result = vision_controller.disconnect_modbus()
    socketio.emit('status_update', vision_controller.get_status())
    
    return jsonify(result)


@app.route('/api/modbus/registers', methods=['GET'])
def get_modbus_registers():
    """獲取所有Modbus寄存器的即時數值"""
    modbus_client = vision_controller.modbus_client
    
    if not modbus_client.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus Client未連接',
            'registers': {}
        })
    
    try:
        registers = {}
        
        # 控制寄存器 (200-209) - 從PLC讀取
        control_registers = {
            '200_外部控制啟用': modbus_client.read_register('EXTERNAL_CONTROL_ENABLE'),
            '201_拍照觸發': modbus_client.read_register('CAPTURE_TRIGGER'),
            '202_拍照檢測觸發': modbus_client.read_register('DETECT_TRIGGER'),
            '203_系統重置': modbus_client.read_register('SYSTEM_RESET'),
            '204_參數更新觸發': modbus_client.read_register('PARAM_UPDATE_TRIGGER'),
        }
        
        # 參數設定寄存器 (210-219) - 從PLC讀取
        area_high = modbus_client.read_register('MIN_AREA_HIGH') or 0
        area_low = modbus_client.read_register('MIN_AREA_LOW') or 0
        combined_area = (area_high << 16) + area_low
        roundness_raw = modbus_client.read_register('MIN_ROUNDNESS') or 0
        roundness_value = roundness_raw / 1000.0
        
        param_registers = {
            '210_最小面積_高16位': area_high,
            '211_最小面積_低16位': area_low,
            '211_合併面積值': combined_area,
            '212_最小圓度_x1000': roundness_raw,
            '212_圓度實際值': round(roundness_value, 3),
            '213_高斯核大小': modbus_client.read_register('GAUSSIAN_KERNEL'),
            '214_Canny低閾值': modbus_client.read_register('CANNY_LOW'),
            '215_Canny高閾值': modbus_client.read_register('CANNY_HIGH'),
        }
        
        # 狀態回報寄存器 (220-239) - 寫入到PLC
        status_registers = {
            '220_系統狀態': modbus_client.read_register('SYSTEM_STATUS'),
            '221_相機連接狀態': modbus_client.read_register('CAMERA_CONNECTED'),
            '222_最後操作狀態': modbus_client.read_register('LAST_OPERATION_STATUS'),
            '223_處理進度': modbus_client.read_register('PROCESSING_PROGRESS'),
        }
        
        # 檢測結果寄存器 (240-279) - 寫入到PLC
        result_registers = {
            '240_檢測圓形數量': modbus_client.read_register('CIRCLE_COUNT'),
        }
        
        # 圓形詳細資料
        for i in range(1, 6):
            x_val = modbus_client.read_register(f'CIRCLE_{i}_X')
            y_val = modbus_client.read_register(f'CIRCLE_{i}_Y')
            r_val = modbus_client.read_register(f'CIRCLE_{i}_RADIUS')
            result_registers[f'{240+i*3-2}_圓形{i}_X座標'] = x_val
            result_registers[f'{240+i*3-1}_圓形{i}_Y座標'] = y_val
            result_registers[f'{240+i*3}_圓形{i}_半徑'] = r_val
        
        # 統計資訊寄存器 (280-299) - 寫入到PLC
        stats_registers = {
            '280_最後拍照耗時ms': modbus_client.read_register('LAST_CAPTURE_TIME'),
            '281_最後處理耗時ms': modbus_client.read_register('LAST_PROCESS_TIME'),
            '282_最後總耗時ms': modbus_client.read_register('LAST_TOTAL_TIME'),
            '283_操作計數器': modbus_client.read_register('OPERATION_COUNT'),
            '284_錯誤計數器': modbus_client.read_register('ERROR_COUNT'),
            '285_連接計數器': modbus_client.read_register('CONNECTION_COUNT'),
            '290_軟體版本主號': modbus_client.read_register('VERSION_MAJOR'),
            '291_軟體版本次號': modbus_client.read_register('VERSION_MINOR'),
            '292_運行時間小時': modbus_client.read_register('UPTIME_HOURS'),
            '293_運行時間分鐘': modbus_client.read_register('UPTIME_MINUTES'),
        }
        
        # 組合所有寄存器
        registers.update(control_registers)
        registers.update(param_registers)
        registers.update(status_registers)
        registers.update(result_registers)
        registers.update(stats_registers)
        
        return jsonify({
            'success': True,
            'message': 'Modbus寄存器讀取成功',
            'registers': registers,
            'external_control_enabled': modbus_client.external_control_enabled,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'total_registers': len(registers),
            'server_info': f"{modbus_client.server_ip}:{modbus_client.server_port}"
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'讀取寄存器失敗: {str(e)}',
            'registers': {},
            'error': str(e)
        })


@app.route('/api/modbus/test', methods=['GET'])
def test_modbus():
    """測試Modbus Client連接狀態"""
    if not MODBUS_AVAILABLE:
        return jsonify({
            'success': False,
            'message': 'Modbus Client模組不可用',
            'available': False,
            'connected': False,
            'pymodbus_version': PYMODBUS_VERSION,
            'install_command': 'pip install pymodbus>=3.0.0'
        })
    
    modbus_client = vision_controller.modbus_client
    
    if not modbus_client.connected:
        return jsonify({
            'success': False,
            'message': f'未連接到Modbus服務器: {modbus_client.server_ip}:{modbus_client.server_port}',
            'available': True,
            'connected': False,
            'pymodbus_version': PYMODBUS_VERSION,
            'suggestion': '請先連接到Modbus TCP服務器'
        })
    
    try:
        # 檢查pymodbus版本
        import pymodbus
        actual_version = pymodbus.__version__
        
        # 測試讀寫操作
        test_success = False
        error_message = ""
        
        # 測試寫入版本號
        write_success = modbus_client.write_register('VERSION_MAJOR', 99)
        if write_success:
            # 測試讀取
            read_value = modbus_client.read_register('VERSION_MAJOR')
            if read_value == 99:
                test_success = True
                # 恢復正確值
                modbus_client.write_register('VERSION_MAJOR', 2)
            else:
                error_message = f"讀取值不匹配: 期望99, 實際{read_value}"
        else:
            error_message = "寫入操作失敗"
        
        # 獲取連接狀態
        connection_status = modbus_client.get_connection_status()
        
        return jsonify({
            'success': test_success,
            'message': f'✅ Modbus Client正常 (pymodbus {actual_version})' if test_success else f'❌ Modbus測試失敗: {error_message}',
            'available': True,
            'connected': True,
            'pymodbus_version': actual_version,
            'expected_version': PYMODBUS_VERSION,
            'write_success': write_success,
            'test_passed': test_success,
            'error_message': error_message,
            'connection_status': connection_status,
            'register_count': len(modbus_client.REGISTERS)
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Modbus測試異常: {str(e)}',
            'available': True,
            'connected': modbus_client.connected,
            'pymodbus_version': PYMODBUS_VERSION,
            'error': str(e),
            'error_type': type(e).__name__
        })


@app.route('/api/initialize', methods=['POST'])
def initialize_camera():
    """初始化相機"""
    data = request.get_json()
    ip_address = data.get('ip_address') if data else None
    
    result = vision_controller.initialize_camera(ip_address)
    socketio.emit('status_update', vision_controller.get_status())
    
    return jsonify(result)


@app.route('/api/update_params', methods=['POST'])
def update_detection_params():
    """更新檢測參數"""
    data = request.get_json()
    min_area = data.get('min_area')
    min_roundness = data.get('min_roundness')
    gaussian_kernel = data.get('gaussian_kernel')
    canny_low = data.get('canny_low')
    canny_high = data.get('canny_high')
    
    vision_controller.update_detection_params(
        min_area=min_area,
        min_roundness=min_roundness,
        gaussian_kernel=gaussian_kernel,
        canny_low=canny_low,
        canny_high=canny_high
    )
    
    return jsonify({
        'success': True,
        'message': '參數已更新',
        'params': asdict(vision_controller.detection_params)
    })


@app.route('/api/capture', methods=['POST'])
def capture_image():
    """拍照"""
    image, capture_time = vision_controller.capture_image()
    
    if image is None:
        return jsonify({
            'success': False,
            'message': '圖像捕獲失敗',
            'capture_time_ms': 0
        })
    
    image_base64 = vision_controller.get_image_base64()
    capture_time_ms = capture_time * 1000
    
    result = {
        'success': True,
        'message': '圖像捕獲成功',
        'capture_time_ms': round(capture_time_ms, 2),
        'image': image_base64,
        'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    
    socketio.emit('image_update', result)
    return jsonify(result)


@app.route('/api/capture_and_detect', methods=['POST'])
def capture_and_detect():
    """拍照並檢測"""
    result = vision_controller.capture_and_detect()
    
    response = {
        'success': result.success,
        'circle_count': result.circle_count,
        'circles': result.circles,
        'capture_time_ms': round(result.capture_time * 1000, 2),
        'processing_time_ms': round(result.processing_time * 1000, 2),
        'total_time_ms': round(result.total_time * 1000, 2),
        'timestamp': result.timestamp,
        'image': vision_controller.get_image_base64() if result.success else None,
        'error_message': result.error_message
    }
    
    socketio.emit('detection_result', response)
    return jsonify(response)


@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """斷開所有連接"""
    vision_controller.disconnect()
    socketio.emit('status_update', vision_controller.get_status())
    
    return jsonify({'success': True, 'message': '所有連接已斷開'})


@app.route('/api/modbus/toggle_external_control', methods=['POST'])
def toggle_external_control():
    """切換外部控制模式"""
    data = request.get_json()
    enable = data.get('enable', False)
    
    modbus_client = vision_controller.modbus_client
    
    # 檢查服務是否可用
    if not modbus_client.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus Client未連接到服務器'
        })
    
    try:
        # 寫入外部控制啟用寄存器到PLC
        value = 1 if enable else 0
        success = modbus_client.write_register('EXTERNAL_CONTROL_ENABLE', value)
        
        if success:
            # 啟用/禁用同步線程
            modbus_client.enable_external_control(enable)
            
            # 驗證寫入 (從PLC讀回確認)
            read_back = modbus_client.read_register('EXTERNAL_CONTROL_ENABLE')
            
            # 記錄日誌
            action = '啟用' if enable else '停用'
            sync_status = '同步線程已啟動' if enable else '同步線程已停止'
            print(f"🔄 WebUI設定外部控制: {action}, {sync_status}")
            
            service_type = "Modbus TCP Client" if MODBUS_AVAILABLE else "模擬Client"
            
            return jsonify({
                'success': True,
                'external_control_enabled': enable,
                'message': f'外部控制已{action} ({service_type}), {sync_status}',
                'register_value': value,
                'read_back_value': read_back,
                'verified': (read_back == value),
                'service_type': service_type,
                'sync_thread_status': '運行中' if enable else '已停止'
            })
        else:
            return jsonify({
                'success': False,
                'message': 'Modbus寄存器寫入失敗'
            })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'操作失敗: {str(e)}'
        })


@app.route('/api/modbus/debug', methods=['GET'])
def get_modbus_debug():
    """獲取Modbus調試信息"""
    modbus_client = vision_controller.modbus_client
    
    if not modbus_client:
        return jsonify({
            'success': False,
            'message': 'Modbus Client不存在'
        })
    
    try:
        debug_info = modbus_client.get_debug_info()
        
        # 額外檢查當前寄存器狀態
        if modbus_client.connected:
            current_registers = {
                'EXTERNAL_CONTROL_ENABLE': modbus_client.read_register('EXTERNAL_CONTROL_ENABLE'),
                'CAPTURE_TRIGGER': modbus_client.read_register('CAPTURE_TRIGGER'),
                'DETECT_TRIGGER': modbus_client.read_register('DETECT_TRIGGER'),
                'SYSTEM_RESET': modbus_client.read_register('SYSTEM_RESET'),
                'CIRCLE_COUNT': modbus_client.read_register('CIRCLE_COUNT'),
                'SYSTEM_STATUS': modbus_client.read_register('SYSTEM_STATUS'),
                'CAMERA_CONNECTED': modbus_client.read_register('CAMERA_CONNECTED')
            }
            debug_info['current_registers'] = current_registers
        
        return jsonify({
            'success': True,
            'debug_info': debug_info,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'獲取調試信息失敗: {str(e)}',
            'error': str(e)
        })


@app.route('/api/modbus/reset_trigger_states', methods=['POST'])
def reset_trigger_states():
    """重置觸發狀態記錄"""
    modbus_client = vision_controller.modbus_client
    
    try:
        # 清除觸發狀態記錄
        old_states = modbus_client.last_trigger_states.copy()
        modbus_client.last_trigger_states.clear()
        
        # 重置錯誤計數（可選）
        reset_errors = request.get_json().get('reset_errors', False) if request.get_json() else False
        if reset_errors:
            modbus_client.error_count = 0
            modbus_client.write_register('ERROR_COUNT', 0)
        
        return jsonify({
            'success': True,
            'message': '觸發狀態已重置',
            'old_states': old_states,
            'error_count_reset': reset_errors,
            'current_error_count': modbus_client.error_count
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'重置觸發狀態失敗: {str(e)}'
        })


@app.route('/api/modbus/clear_triggers', methods=['POST'])
def clear_triggers():
    """清除所有觸發信號"""
    modbus_client = vision_controller.modbus_client
    
    if not modbus_client.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus未連接'
        })
    
    try:
        # 清除所有觸發信號
        triggers_cleared = {}
        triggers_cleared['CAPTURE_TRIGGER'] = modbus_client.write_register('CAPTURE_TRIGGER', 0)
        triggers_cleared['DETECT_TRIGGER'] = modbus_client.write_register('DETECT_TRIGGER', 0)
        triggers_cleared['SYSTEM_RESET'] = modbus_client.write_register('SYSTEM_RESET', 0)
        triggers_cleared['PARAM_UPDATE_TRIGGER'] = modbus_client.write_register('PARAM_UPDATE_TRIGGER', 0)
        
        # 重置處理進度
        modbus_client.write_register('PROCESSING_PROGRESS', 0)
        
        success_count = sum(triggers_cleared.values())
        
        return jsonify({
            'success': True,
            'message': f'已清除 {success_count}/4 個觸發信號',
            'triggers_cleared': triggers_cleared
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'清除觸發信號失敗: {str(e)}'
        })


@app.route('/api/modbus/manual_trigger', methods=['POST'])
def manual_trigger():
    """手動觸發檢測 (繞過Modbus，直接調用)"""
    data = request.get_json()
    action = data.get('action', 'detect')  # 'capture' 或 'detect'
    
    modbus_client = vision_controller.modbus_client
    
    try:
        if action == 'capture':
            print("🔧 手動觸發: 拍照")
            modbus_client._handle_capture_trigger()
        elif action == 'detect':
            print("🔧 手動觸發: 拍照+檢測")
            modbus_client._handle_detect_trigger()
        else:
            return jsonify({
                'success': False,
                'message': '無效的操作類型'
            })
        
        return jsonify({
            'success': True,
            'message': f'手動觸發 {action} 完成',
            'operation_count': modbus_client.operation_count,
            'error_count': modbus_client.error_count
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'手動觸發失敗: {str(e)}'
        })


@app.route('/api/modbus/force_sync', methods=['POST'])
def force_start_sync():
    """強制啟動同步線程 (調試用)"""
    modbus_client = vision_controller.modbus_client
    
    if not modbus_client.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus未連接'
        })
    
    try:
        # 強制啟動同步
        modbus_client.external_control_enabled = True
        modbus_client.start_sync()
        
        return jsonify({
            'success': True,
            'message': '同步線程已強制啟動',
            'sync_running': modbus_client.sync_running,
            'external_control': modbus_client.external_control_enabled
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'強制啟動同步失敗: {str(e)}'
        })


@app.route('/api/modbus/info', methods=['GET'])
def get_modbus_info():
    """獲取Modbus Client資訊"""
    try:
        import pymodbus
        current_version = pymodbus.__version__
        version_info = f"當前版本: {current_version}"
    except:
        current_version = "未安裝"
        version_info = "pymodbus未安裝"
    
    return jsonify({
        'pymodbus_available': MODBUS_AVAILABLE,
        'current_version': current_version,
        'target_version': PYMODBUS_VERSION,
        'version_info': version_info,
        'client_mode': True,
        'server_mode': False,
        'install_commands': [
            'pip install pymodbus>=3.0.0',
            'pip install "pymodbus[serial]>=3.0.0"'
        ],
        'verify_command': 'python -c "import pymodbus; print(f\'pymodbus {pymodbus.__version__}\')"',
        'architecture': 'Modbus TCP Client (連接外部PLC/HMI)',
        'register_mapping': {
            '控制寄存器 (200-209)': '從PLC讀取控制指令',
            '參數設定 (210-219)': '從PLC讀取檢測參數',
            '狀態回報 (220-239)': '寫入系統狀態到PLC',
            '檢測結果 (240-279)': '寫入檢測結果到PLC',
            '統計資訊 (280-299)': '寫入統計資料到PLC'
        },
        'features': [
            '自動重連機制',
            '外部觸發控制',
            '參數動態更新',
            '狀態即時回報',
            '錯誤計數追蹤'
        ],
        'restart_required': True,
        'compatibility': {
            'python_min': '3.7',
            'recommended_python': '3.8+',
            'async_support': True,
            'sync_support': True
        }
    })


@socketio.on('connect')
def handle_connect():
    """客戶端連接"""
    emit('status_update', vision_controller.get_status())


@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開"""
    pass


def main():
    """主函數"""
    print("🚀 CCD1 視覺控制系統啟動中 (Modbus TCP Client版本)...")
    
    if not CAMERA_MANAGER_AVAILABLE:
        print("❌ 相機管理器不可用，請檢查SDK導入")
        return
    
    try:
        print("🔧 系統架構: Modbus TCP Client")
        print("📡 連接模式: 主動連接外部PLC/HMI設備")
        
        if MODBUS_AVAILABLE:
            print(f"✅ Modbus TCP Client模組可用 (pymodbus {PYMODBUS_VERSION})")
            print("📊 CCD1 Modbus寄存器映射 (Client模式):")
            print("   ┌─ 控制寄存器 (200-209) ← 從PLC讀取")
            print("   │  • 200: 外部控制啟用")
            print("   │  • 201: 拍照觸發")
            print("   │  • 202: 拍照+檢測觸發")
            print("   │  • 203: 系統重置")
            print("   │  • 204: 參數更新觸發")
            print("   ├─ 參數設定 (210-219) ← 從PLC讀取")
            print("   │  • 210-211: 最小面積設定")
            print("   │  • 212: 最小圓度設定")
            print("   │  • 213-215: 圖像處理參數")
            print("   ├─ 狀態回報 (220-239) → 寫入到PLC")
            print("   │  • 220: 系統狀態")
            print("   │  • 221: 相機連接狀態")
            print("   │  • 222: 最後操作狀態")
            print("   │  • 223: 處理進度")
            print("   ├─ 檢測結果 (240-279) → 寫入到PLC")
            print("   │  • 240: 檢測圓形數量")
            print("   │  • 241-255: 圓形1-5的座標和半徑")
            print("   └─ 統計資訊 (280-299) → 寫入到PLC")
            print("      • 280-282: 時間統計")
            print("      • 283-285: 計數器")
            print("      • 290-293: 版本與運行時間")
        else:
            print("⚠️ Modbus Client功能不可用 (使用模擬模式)")
        
        print("🌐 Web介面啟動中...")
        print("📱 訪問地址: http://localhost:5051")
        print("🎯 系統功能:")
        print("   • 相機連接管理")
        print("   • 參數調整介面")
        print("   • 圓形檢測與標註")
        print("   • Modbus TCP Client外部控制")
        print("   • 即時狀態監控")
        print("🔗 使用說明:")
        print("   1. 先設置Modbus服務器IP地址")
        print("   2. 連接到外部PLC/HMI設備")
        print("   3. 初始化相機連接")
        print("   4. 啟用外部控制模式")
        print("   5. 通過PLC控制拍照和檢測")
        print("=" * 60)
        
        socketio.run(app, host='0.0.0.0', port=5051, debug=False)
        
    except KeyboardInterrupt:
        print("\n🛑 用戶中斷，正在關閉系統...")
    except Exception as e:
        print(f"❌ 系統運行錯誤: {e}")
    finally:
        try:
            vision_controller.disconnect()
        except:
            pass
        print("✅ 系統已安全關閉")


if __name__ == "__main__":
    main()