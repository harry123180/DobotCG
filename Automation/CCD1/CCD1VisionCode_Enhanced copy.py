# -*- coding: utf-8 -*-
"""
CCD1VisionCode_Enhanced.py - CCD視覺控制系統 (運動控制握手版本 + 世界座標轉換)
實現運動控制握手、輪詢式狀態監控、狀態機通信、指令/狀態模式
新增：內外參管理、像素座標到世界座標轉換功能
適用於自動化設備對接流程
"""

import sys
import os
import time
import threading
import json
import base64
import glob
from typing import Optional, Dict, Any, Tuple, List
import numpy as np
import cv2
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import logging
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import IntEnum

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


# ==================== 控制指令枚舉 ====================
class ControlCommand(IntEnum):
    """控制指令枚舉"""
    CLEAR = 0          # 清空控制
    CAPTURE = 8        # 拍照
    DETECT = 16        # 拍照+檢測
    INITIALIZE = 32    # 重新初始化


# ==================== 狀態位枚舉 ====================
class StatusBits(IntEnum):
    """狀態位枚舉"""
    READY = 0      # bit0: Ready狀態
    RUNNING = 1    # bit1: Running狀態  
    ALARM = 2      # bit2: Alarm狀態
    INITIALIZED = 3 # bit3: 初始化狀態


# ==================== 相機內外參管理 ====================
@dataclass
class CameraCalibrationData:
    """相機標定數據結構"""
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None
    is_valid: bool = False
    intrinsic_file: Optional[str] = None
    extrinsic_file: Optional[str] = None
    load_time: Optional[str] = None


class CameraCoordinateTransformer:
    """相機座標轉換器"""
    
    def __init__(self, camera_matrix=None, dist_coeffs=None, rvec=None, tvec=None):
        self.K = camera_matrix
        self.D = dist_coeffs
        self.rvec = rvec.reshape(3, 1) if rvec is not None and rvec.shape != (3, 1) else rvec
        self.tvec = tvec.reshape(3, 1) if tvec is not None and tvec.shape != (3, 1) else tvec
        self.R = None
        
        if self.rvec is not None:
            self.R, _ = cv2.Rodrigues(self.rvec)
    
    def is_valid(self) -> bool:
        """檢查轉換器是否有效"""
        return all([
            self.K is not None,
            self.D is not None,
            self.rvec is not None,
            self.tvec is not None,
            self.R is not None
        ])
    
    def pixel_to_world(self, pixel_coords) -> Optional[np.ndarray]:
        """像素座標轉世界座標（假設Z=0平面）"""
        if not self.is_valid():
            return None
            
        try:
            pixel_coords = np.array(pixel_coords)
            if pixel_coords.ndim == 1:
                pixel_coords = pixel_coords.reshape(1, -1)
                
            world_points = []
            
            for uv in pixel_coords:
                # 步驟1：去畸變
                undistorted_uv = cv2.undistortPoints(
                    uv.reshape(1, 1, 2), self.K, self.D, P=self.K
                ).reshape(-1)
                
                # 步驟2：歸一化座標
                uv_homogeneous = np.array([undistorted_uv[0], undistorted_uv[1], 1.0])
                normalized_coords = np.linalg.inv(self.K) @ uv_homogeneous
                
                # 步驟3：計算深度係數（Z=0平面）
                denominator = self.R[2] @ normalized_coords
                if abs(denominator) < 1e-8:
                    raise ValueError("相機平行於Z=0平面，無法計算交點")
                    
                s = (0 - self.tvec[2, 0]) / denominator
                
                # 步驟4：計算世界座標
                camera_point = s * normalized_coords
                world_point = np.linalg.inv(self.R) @ (camera_point - self.tvec.ravel())
                
                world_points.append(world_point[:2])  # 只返回X,Y座標
                
            return np.array(world_points).squeeze()
            
        except Exception as e:
            print(f"❌ 座標轉換失敗: {e}")
            return None


class CalibrationManager:
    """相機標定檔案管理器"""
    
    def __init__(self, working_dir=None):
        self.working_dir = working_dir or os.path.dirname(os.path.abspath(__file__))
        self.calibration_data = CameraCalibrationData()
        self.transformer = None
        
    def scan_calibration_files(self) -> Dict[str, Any]:
        """掃描標定檔案"""
        result = {
            'intrinsic_files': [],
            'extrinsic_files': [],
            'found_intrinsic': False,
            'found_extrinsic': False,
            'working_dir': self.working_dir
        }
        
        try:
            # 掃描內參檔案 (規範命名)
            camera_matrix_files = glob.glob(os.path.join(self.working_dir, "camera_matrix_*.npy"))
            dist_coeffs_files = glob.glob(os.path.join(self.working_dir, "dist_coeffs_*.npy"))
            
            for matrix_file in camera_matrix_files:
                timestamp = matrix_file.split("camera_matrix_")[1].replace(".npy", "")
                dist_file = os.path.join(self.working_dir, f"dist_coeffs_{timestamp}.npy")
                
                if os.path.exists(dist_file):
                    result['intrinsic_files'].append({
                        'matrix_file': matrix_file,
                        'dist_file': dist_file,
                        'timestamp': timestamp
                    })
                    result['found_intrinsic'] = True
            
            # 掃描外參檔案 (較寬鬆的命名規則)
            extrinsic_patterns = [
                "extrinsic_*.npy",
                "*extrinsic*.npy", 
                "*外參*.npy",
                "*rvec*.npy"
            ]
            
            for pattern in extrinsic_patterns:
                files = glob.glob(os.path.join(self.working_dir, pattern))
                for file in files:
                    if file not in [item['file'] for item in result['extrinsic_files']]:
                        result['extrinsic_files'].append({
                            'file': file,
                            'name': os.path.basename(file)
                        })
                        result['found_extrinsic'] = True
            
            return result
            
        except Exception as e:
            print(f"❌ 掃描標定檔案失敗: {e}")
            return result
    
    def load_calibration_data(self, intrinsic_file=None, extrinsic_file=None) -> Dict[str, Any]:
        """載入標定數據"""
        try:
            # 如果未指定，自動選擇最新的檔案
            if intrinsic_file is None or extrinsic_file is None:
                scan_result = self.scan_calibration_files()
                
                if not scan_result['found_intrinsic']:
                    return {
                        'success': False,
                        'message': '未找到內參檔案（camera_matrix_*.npy, dist_coeffs_*.npy）',
                        'details': f'檢查目錄: {self.working_dir}'
                    }
                
                if not scan_result['found_extrinsic']:
                    return {
                        'success': False,
                        'message': '未找到外參檔案（extrinsic_*.npy 或類似命名）',
                        'details': f'檢查目錄: {self.working_dir}'
                    }
                
                # 選擇最新的內參檔案
                latest_intrinsic = max(scan_result['intrinsic_files'], 
                                     key=lambda x: x['timestamp'])
                intrinsic_file = latest_intrinsic['matrix_file']
                dist_file = latest_intrinsic['dist_file']
                
                # 選擇第一個外參檔案（可以後續擴展為選擇最新的）
                extrinsic_file = scan_result['extrinsic_files'][0]['file']
            else:
                # 根據內參檔案找對應的畸變檔案
                if "camera_matrix_" in intrinsic_file:
                    timestamp = intrinsic_file.split("camera_matrix_")[1].replace(".npy", "")
                    dist_file = os.path.join(self.working_dir, f"dist_coeffs_{timestamp}.npy")
                    if not os.path.exists(dist_file):
                        return {
                            'success': False,
                            'message': f'找不到對應的畸變係數檔案: dist_coeffs_{timestamp}.npy'
                        }
                else:
                    return {
                        'success': False,
                        'message': '內參檔案命名不符合規範（camera_matrix_YYYYMMDD_HHMMSS.npy）'
                    }
            
            # 載入內參
            camera_matrix = np.load(intrinsic_file)
            dist_coeffs = np.load(dist_file)
            
            # 驗證內參格式
            if camera_matrix.shape != (3, 3):
                return {
                    'success': False,
                    'message': f'內參矩陣格式錯誤: 期望(3,3), 實際{camera_matrix.shape}'
                }
            
            if dist_coeffs.shape[0] < 4:
                return {
                    'success': False,
                    'message': f'畸變係數格式錯誤: 期望至少4個參數, 實際{dist_coeffs.shape[0]}個'
                }
            
            # 載入外參
            extrinsic_data = np.load(extrinsic_file, allow_pickle=True)
            
            if isinstance(extrinsic_data, dict):
                # 字典格式
                rvec = np.array(extrinsic_data.get('rvec', extrinsic_data.get('rotation_vector')))
                tvec = np.array(extrinsic_data.get('tvec', extrinsic_data.get('translation_vector')))
            else:
                return {
                    'success': False,
                    'message': '外參檔案格式錯誤: 期望字典格式包含rvec和tvec'
                }
            
            # 驗證外參格式
            if rvec is None or tvec is None:
                return {
                    'success': False,
                    'message': '外參檔案缺少rvec或tvec數據'
                }
            
            if rvec.shape != (3, 1) and rvec.shape != (3,):
                return {
                    'success': False,
                    'message': f'旋轉向量格式錯誤: 期望(3,1)或(3,), 實際{rvec.shape}'
                }
            
            if tvec.shape != (3, 1) and tvec.shape != (3,):
                return {
                    'success': False,
                    'message': f'平移向量格式錯誤: 期望(3,1)或(3,), 實際{tvec.shape}'
                }
            
            # 更新標定數據
            self.calibration_data.camera_matrix = camera_matrix
            self.calibration_data.dist_coeffs = dist_coeffs.flatten()  # 確保為1D
            self.calibration_data.rvec = rvec
            self.calibration_data.tvec = tvec
            self.calibration_data.is_valid = True
            self.calibration_data.intrinsic_file = intrinsic_file
            self.calibration_data.extrinsic_file = extrinsic_file
            self.calibration_data.load_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # 創建座標轉換器
            self.transformer = CameraCoordinateTransformer(
                camera_matrix, dist_coeffs.flatten(), rvec, tvec
            )
            
            return {
                'success': True,
                'message': '內外參載入成功',
                'details': {
                    'intrinsic_file': os.path.basename(intrinsic_file),
                    'extrinsic_file': os.path.basename(extrinsic_file),
                    'camera_matrix_shape': camera_matrix.shape,
                    'dist_coeffs_count': len(dist_coeffs),
                    'load_time': self.calibration_data.load_time
                }
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'載入標定數據失敗: {str(e)}',
                'error_type': type(e).__name__
            }
    
    def get_status(self) -> Dict[str, Any]:
        """獲取標定狀態"""
        return {
            'is_valid': self.calibration_data.is_valid,
            'has_transformer': self.transformer is not None and self.transformer.is_valid(),
            'intrinsic_file': os.path.basename(self.calibration_data.intrinsic_file) if self.calibration_data.intrinsic_file else None,
            'extrinsic_file': os.path.basename(self.calibration_data.extrinsic_file) if self.calibration_data.extrinsic_file else None,
            'load_time': self.calibration_data.load_time,
            'working_dir': self.working_dir
        }


# ==================== 系統狀態管理 ====================
class SystemStateMachine:
    """系統狀態機管理"""
    
    def __init__(self):
        self.status_register = 0b0000  # 4位狀態寄存器
        self.lock = threading.Lock()
        
    def get_bit(self, bit_pos: StatusBits) -> bool:
        """獲取指定位的狀態"""
        with self.lock:
            return bool(self.status_register & (1 << bit_pos))
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        """設置指定位的狀態"""
        with self.lock:
            if value:
                self.status_register |= (1 << bit_pos)
            else:
                self.status_register &= ~(1 << bit_pos)
    
    def get_status_register(self) -> int:
        """獲取完整狀態寄存器值"""
        with self.lock:
            return self.status_register
    
    def is_ready(self) -> bool:
        """檢查是否Ready狀態"""
        return self.get_bit(StatusBits.READY)
    
    def is_running(self) -> bool:
        """檢查是否Running狀態"""
        return self.get_bit(StatusBits.RUNNING)
    
    def is_alarm(self) -> bool:
        """檢查是否Alarm狀態"""
        return self.get_bit(StatusBits.ALARM)
    
    def is_initialized(self) -> bool:
        """檢查是否已初始化"""
        return self.get_bit(StatusBits.INITIALIZED)
    
    def set_ready(self, ready: bool = True):
        """設置Ready狀態"""
        self.set_bit(StatusBits.READY, ready)
    
    def set_running(self, running: bool = True):
        """設置Running狀態"""
        self.set_bit(StatusBits.RUNNING, running)
    
    def set_alarm(self, alarm: bool = True):
        """設置Alarm狀態"""
        self.set_bit(StatusBits.ALARM, alarm)
        if alarm:
            # Alarm時，初始化狀態設為0
            self.set_bit(StatusBits.INITIALIZED, False)
    
    def set_initialized(self, initialized: bool = True):
        """設置初始化狀態"""
        self.set_bit(StatusBits.INITIALIZED, initialized)
    
    def reset_to_idle(self):
        """重置到空閒狀態"""
        with self.lock:
            self.status_register = 0b0001  # 只保留Ready=1，其他位清零
    
    def get_status_description(self) -> Dict[str, Any]:
        """獲取狀態描述"""
        return {
            'ready': self.is_ready(),
            'running': self.is_running(),
            'alarm': self.is_alarm(),
            'initialized': self.is_initialized(),
            'status_register': self.get_status_register(),
            'binary_representation': f"{self.get_status_register():04b}"
        }


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
    """視覺辨識結果（擴展世界座標）"""
    circle_count: int
    circles: List[Dict[str, Any]]
    processing_time: float
    capture_time: float
    total_time: float
    timestamp: str
    success: bool
    has_world_coords: bool = False  # 新增：是否包含世界座標
    error_message: Optional[str] = None


class EnhancedModbusTcpClientService:
    """增強型Modbus TCP Client服務 - 運動控制握手版本（擴展世界座標寄存器）"""
    
    def __init__(self, server_ip="192.168.1.100", server_port=502):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.vision_controller = None
        
        # 狀態機管理
        self.state_machine = SystemStateMachine()
        
        # 連接參數
        self.reconnect_delay = 5.0
        self.read_timeout = 3.0
        self.write_timeout = 3.0
        
        # 同步控制
        self.sync_enabled = False
        self.sync_thread = None
        self.sync_running = False
        self.sync_interval = 0.05  # 50ms輪詢間隔，更快響應
        
        # 握手控制
        self.last_control_command = 0
        self.command_processing = False
        
        # 擴展的寄存器映射 (運動控制握手模式 + 世界座標)
        self.REGISTERS = {
            # ===== 核心控制握手寄存器 =====
            'CONTROL_COMMAND': 200,        # 控制指令寄存器 (0=清空, 8=拍照, 16=拍照+檢測, 32=重新初始化)
            'STATUS_REGISTER': 201,        # 狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)
            
            # ===== 檢測參數寄存器 (210-219) =====
            'MIN_AREA_HIGH': 210,          # 最小面積設定 (高16位)
            'MIN_AREA_LOW': 211,           # 最小面積設定 (低16位)
            'MIN_ROUNDNESS': 212,          # 最小圓度設定 (乘以1000)
            'GAUSSIAN_KERNEL': 213,        # 高斯核大小
            'CANNY_LOW': 214,              # Canny低閾值
            'CANNY_HIGH': 215,             # Canny高閾值
            
            # ===== 像素座標檢測結果寄存器 (240-255) =====
            'CIRCLE_COUNT': 240,           # 檢測到的圓形數量
            'CIRCLE_1_X': 241,             # 圓形1 X座標
            'CIRCLE_1_Y': 242,             # 圓形1 Y座標
            'CIRCLE_1_RADIUS': 243,        # 圓形1 半徑
            'CIRCLE_2_X': 244,             # 圓形2 X座標
            'CIRCLE_2_Y': 245,             # 圓形2 Y座標
            'CIRCLE_2_RADIUS': 246,        # 圓形2 半徑
            'CIRCLE_3_X': 247,             # 圓形3 X座標
            'CIRCLE_3_Y': 248,             # 圓形3 Y座標
            'CIRCLE_3_RADIUS': 249,        # 圓形3 半徑
            'CIRCLE_4_X': 250,             # 圓形4 X座標
            'CIRCLE_4_Y': 251,             # 圓形4 Y座標
            'CIRCLE_4_RADIUS': 252,        # 圓形4 半徑
            'CIRCLE_5_X': 253,             # 圓形5 X座標
            'CIRCLE_5_Y': 254,             # 圓形5 Y座標
            'CIRCLE_5_RADIUS': 255,        # 圓形5 半徑
            
            # ===== 世界座標結果寄存器 (256-275) - 新增 =====
            'WORLD_COORD_VALID': 256,      # 世界座標有效性標誌 (0=無效, 1=有效)
            'CIRCLE_1_WORLD_X_HIGH': 257,  # 圓形1 世界X座標高位 (乘以100)
            'CIRCLE_1_WORLD_X_LOW': 258,   # 圓形1 世界X座標低位
            'CIRCLE_1_WORLD_Y_HIGH': 259,  # 圓形1 世界Y座標高位 (乘以100)
            'CIRCLE_1_WORLD_Y_LOW': 260,   # 圓形1 世界Y座標低位
            'CIRCLE_2_WORLD_X_HIGH': 261,  # 圓形2 世界X座標高位
            'CIRCLE_2_WORLD_X_LOW': 262,   # 圓形2 世界X座標低位
            'CIRCLE_2_WORLD_Y_HIGH': 263,  # 圓形2 世界Y座標高位
            'CIRCLE_2_WORLD_Y_LOW': 264,   # 圓形2 世界Y座標低位
            'CIRCLE_3_WORLD_X_HIGH': 265,  # 圓形3 世界X座標高位
            'CIRCLE_3_WORLD_X_LOW': 266,   # 圓形3 世界X座標低位
            'CIRCLE_3_WORLD_Y_HIGH': 267,  # 圓形3 世界Y座標高位
            'CIRCLE_3_WORLD_Y_LOW': 268,   # 圓形3 世界Y座標低位
            'CIRCLE_4_WORLD_X_HIGH': 269,  # 圓形4 世界X座標高位
            'CIRCLE_4_WORLD_X_LOW': 270,   # 圓形4 世界X座標低位
            'CIRCLE_4_WORLD_Y_HIGH': 271,  # 圓形4 世界Y座標高位
            'CIRCLE_4_WORLD_Y_LOW': 272,   # 圓形4 世界Y座標低位
            'CIRCLE_5_WORLD_X_HIGH': 273,  # 圓形5 世界X座標高位
            'CIRCLE_5_WORLD_X_LOW': 274,   # 圓形5 世界X座標低位
            'CIRCLE_5_WORLD_Y_HIGH': 275,  # 圓形5 世界Y座標高位
            'CIRCLE_5_WORLD_Y_LOW': 276,   # 圓形5 世界Y座標低位
            
            # ===== 統計資訊寄存器 (280-299) =====
            'LAST_CAPTURE_TIME': 280,      # 最後拍照耗時 (ms)
            'LAST_PROCESS_TIME': 281,      # 最後處理耗時 (ms)
            'LAST_TOTAL_TIME': 282,        # 最後總耗時 (ms)
            'OPERATION_COUNT': 283,        # 操作計數器
            'ERROR_COUNT': 284,            # 錯誤計數器
            'CONNECTION_COUNT': 285,       # 連接計數器
            'VERSION_MAJOR': 290,          # 軟體版本主版號
            'VERSION_MINOR': 291,          # 軟體版本次版號
            'UPTIME_HOURS': 292,           # 系統運行時間 (小時)
            'UPTIME_MINUTES': 293,         # 系統運行時間 (分鐘)
        }
        
        # 統計計數
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
    
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
            self.state_machine.set_alarm(True)
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
            
            if self.client.connect():
                self.connected = True
                self.connection_count += 1
                
                # 初始化狀態寄存器
                self._initialize_status_registers()
                
                # 檢查相機狀態並設置初始化位
                self._update_initialization_status()
                
                print(f"✅ Modbus TCP Client連接成功: {self.server_ip}:{self.server_port}")
                return True
            else:
                print(f"❌ Modbus TCP連接失敗: {self.server_ip}:{self.server_port}")
                self.connected = False
                self.state_machine.set_alarm(True)
                return False
                
        except Exception as e:
            print(f"❌ Modbus TCP連接異常: {e}")
            self.connected = False
            self.state_machine.set_alarm(True)
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        self.stop_handshake_sync()
        
        if self.client and self.connected:
            try:
                # 設置斷線狀態
                self.state_machine.set_alarm(True)
                self.write_register('STATUS_REGISTER', self.state_machine.get_status_register())
                
                self.client.close()
                print("🔌 Modbus TCP Client已斷開連接")
            except:
                pass
        
        self.connected = False
        self.client = None
    
    def start_handshake_sync(self):
        """啟動握手同步線程"""
        if self.sync_running:
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
        self.sync_thread.start()
        print("✅ 運動控制握手同步線程已啟動")
    
    def stop_handshake_sync(self):
        """停止握手同步線程"""
        if self.sync_running:
            self.sync_running = False
            if self.sync_thread and self.sync_thread.is_alive():
                self.sync_thread.join(timeout=2.0)
            print("🛑 運動控制握手同步線程已停止")
    
    def _handshake_sync_loop(self):
        """握手同步循環 - 高頻輪詢式狀態監控"""
        print("🔄 運動控制握手同步線程開始運行...")
        
        while self.sync_running and self.connected:
            try:
                # 1. 更新狀態寄存器到PLC
                self._update_status_to_plc()
                
                # 2. 讀取控制指令並處理握手邏輯
                self._process_handshake_control()
                
                # 3. 定期更新統計資訊和系統狀態
                self._update_system_statistics()
                
                # 短暫休眠 (50ms輪詢間隔)
                time.sleep(self.sync_interval)
                
            except ConnectionException:
                print("❌ Modbus連接中斷，同步線程退出")
                self.connected = False
                self.state_machine.set_alarm(True)
                break
                
            except Exception as e:
                print(f"❌ 握手同步線程錯誤: {e}")
                self.error_count += 1
                time.sleep(0.5)  # 錯誤時稍長休眠
        
        self.sync_running = False
        print("⏹️ 運動控制握手同步線程已退出")
    
    def _process_handshake_control(self):
        """處理握手控制邏輯"""
        try:
            # 讀取控制指令
            control_command = self.read_register('CONTROL_COMMAND')
            if control_command is None:
                return
            
            # 檢查是否有新的控制指令
            if control_command != self.last_control_command:
                print(f"🎯 收到新控制指令: {control_command} (上次: {self.last_control_command})")
                
                # 根據控制指令處理
                if control_command == ControlCommand.CLEAR:
                    self._handle_clear_command()
                elif control_command in [ControlCommand.CAPTURE, ControlCommand.DETECT, ControlCommand.INITIALIZE]:
                    self._handle_action_command(control_command)
                
                self.last_control_command = control_command
            
            # 檢查Running狀態完成後的Ready恢復邏輯
            if (not self.state_machine.is_running() and 
                control_command == ControlCommand.CLEAR and
                not self.command_processing):
                
                if not self.state_machine.is_ready() and not self.state_machine.is_alarm():
                    print("🟢 恢復Ready狀態")
                    self.state_machine.set_ready(True)
                    
        except Exception as e:
            print(f"❌ 處理握手控制失敗: {e}")
            self.error_count += 1
    
    def _handle_clear_command(self):
        """處理清空控制指令"""
        if self.command_processing:
            return  # 正在處理指令，不處理清空
            
        # 清空控制指令不需要Ready檢查，直接清空相關狀態
        print("🗑️ 處理清空控制指令")
        # 這裡不設置任何狀態，等待握手邏輯自然恢復Ready
    
    def _handle_action_command(self, command: ControlCommand):
        """處理動作指令 (拍照、檢測、初始化)"""
        # 檢查Ready狀態
        if not self.state_machine.is_ready():
            print(f"⚠️ 系統未Ready，忽略控制指令 {command}")
            return
        
        if self.command_processing:
            print(f"⚠️ 正在處理指令，忽略新指令 {command}")
            return
        
        # 設置Running狀態，清除Ready狀態
        print(f"🚀 開始處理控制指令: {command}")
        self.state_machine.set_ready(False)
        self.state_machine.set_running(True)
        self.command_processing = True
        
        # 在獨立線程中執行命令，避免阻塞同步循環
        command_thread = threading.Thread(
            target=self._execute_command_async,
            args=(command,),
            daemon=True
        )
        command_thread.start()
    
    def _execute_command_async(self, command: ControlCommand):
        """異步執行控制指令"""
        try:
            if command == ControlCommand.CAPTURE:
                self._execute_capture()
            elif command == ControlCommand.DETECT:
                self._execute_detect()
            elif command == ControlCommand.INITIALIZE:
                self._execute_initialize()
            
        except Exception as e:
            print(f"❌ 執行控制指令失敗: {e}")
            self.error_count += 1
            self.state_machine.set_alarm(True)
        
        finally:
            # 無論成功失敗，都要清除Running狀態
            print(f"✅ 控制指令 {command} 執行完成")
            self.state_machine.set_running(False)
            self.command_processing = False
            self.operation_count += 1
    
    def _execute_capture(self):
        """執行拍照指令"""
        if not self.vision_controller:
            raise Exception("視覺控制器未設置")
        
        print("📸 執行拍照指令")
        image, capture_time = self.vision_controller.capture_image()
        
        if image is not None:
            self.write_register('LAST_CAPTURE_TIME', int(capture_time * 1000))
            print(f"✅ 拍照成功，耗時: {capture_time*1000:.2f}ms")
        else:
            raise Exception("拍照失敗")
    
    def _execute_detect(self):
        """執行拍照+檢測指令"""
        if not self.vision_controller:
            raise Exception("視覺控制器未設置")
        
        print("🔍 執行拍照+檢測指令")
        result = self.vision_controller.capture_and_detect()
        
        if result.success:
            self.update_detection_results(result)
            print(f"✅ 檢測成功，找到 {result.circle_count} 個圓形")
        else:
            raise Exception(f"檢測失敗: {result.error_message}")
    
    def _execute_initialize(self):
        """執行重新初始化指令"""
        print("🔄 執行重新初始化指令")
        
        # 清除Alarm狀態
        self.state_machine.set_alarm(False)
        
        # 重新初始化相機
        if self.vision_controller:
            init_result = self.vision_controller.initialize_camera()
            if not init_result.get('success', False):
                raise Exception("相機初始化失敗")
        
        # 更新初始化狀態
        self._update_initialization_status()
        
        print("✅ 重新初始化完成")
    
    def _initialize_status_registers(self):
        """初始化狀態寄存器"""
        try:
            # 寫入版本資訊
            self.write_register('VERSION_MAJOR', 4)  # 版本升級到4.0（新增世界座標功能）
            self.write_register('VERSION_MINOR', 0)
            
            # 強制重置狀態機到初始狀態
            self.state_machine.reset_to_idle()
            
            # 確保狀態寄存器固定為初始值
            initial_status = 0b0001  # Ready=1, 其他位=0，確保狀態寄存器值為1
            self.state_machine.status_register = initial_status
            
            # 寫入固定的初始狀態到PLC
            self.write_register('STATUS_REGISTER', initial_status)
            self.write_register('CONTROL_COMMAND', 0)  # 清空控制指令
            
            # 初始化世界座標有效性標誌
            self.write_register('WORLD_COORD_VALID', 0)  # 預設為無效
            
            # 初始化計數器
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            print(f"📊 狀態寄存器初始化完成，固定初始值: {initial_status} (Ready=1)")
            
        except Exception as e:
            print(f"❌ 初始化狀態寄存器失敗: {e}")
    
    def _update_status_to_plc(self):
        """更新狀態到PLC"""
        try:
            # 更新狀態寄存器
            status_value = self.state_machine.get_status_register()
            self.write_register('STATUS_REGISTER', status_value)
            
            # 更新計數器
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            
            # 更新世界座標有效性
            if self.vision_controller:
                calib_valid = (self.vision_controller.calibration_manager.calibration_data.is_valid and
                             self.vision_controller.calibration_manager.transformer and
                             self.vision_controller.calibration_manager.transformer.is_valid())
                self.write_register('WORLD_COORD_VALID', 1 if calib_valid else 0)
            
        except Exception as e:
            print(f"❌ 更新狀態到PLC失敗: {e}")
    
    def _update_system_statistics(self):
        """更新系統統計資訊"""
        try:
            # 更新運行時間
            uptime_total_minutes = int((time.time() - self.start_time) / 60)
            uptime_hours = uptime_total_minutes // 60
            uptime_minutes = uptime_total_minutes % 60
            
            self.write_register('UPTIME_HOURS', uptime_hours)
            self.write_register('UPTIME_MINUTES', uptime_minutes)
            
        except Exception as e:
            pass  # 統計更新失敗不影響主流程
    
    def _update_initialization_status(self):
        """更新初始化狀態"""
        try:
            # 檢查系統初始化狀態
            modbus_ok = self.connected
            camera_ok = (self.vision_controller and 
                        self.vision_controller.is_connected)
            
            if modbus_ok and camera_ok:
                # 系統完全初始化：Ready=1, Initialized=1, Alarm=0, Running=0
                self.state_machine.set_initialized(True)
                self.state_machine.set_alarm(False)
                self.state_machine.set_ready(True)
                self.state_machine.set_running(False)
                print("✅ 系統完全初始化，狀態寄存器固定為: Ready=1, Initialized=1")
            else:
                # 系統未完全初始化：設置Alarm=1, Initialized=0
                self.state_machine.set_initialized(False)
                self.state_machine.set_alarm(True)
                self.state_machine.set_ready(False)
                print(f"⚠️ 系統未完全初始化 - Modbus: {modbus_ok}, Camera: {camera_ok}")
                
        except Exception as e:
            print(f"❌ 更新初始化狀態失敗: {e}")
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.client.write_register(address, value, slave=1)
            
            return not result.isError()
                
        except Exception as e:
            return False
    
    def update_detection_results(self, result: VisionResult):
        """更新檢測結果到PLC（包含世界座標）"""
        try:
            # 寫入圓形數量
            self.write_register('CIRCLE_COUNT', result.circle_count)
            
            # 寫入像素座標和半徑 (最多5個)
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
            
            # 寫入世界座標（如果有效）
            world_coord_valid = result.has_world_coords
            self.write_register('WORLD_COORD_VALID', 1 if world_coord_valid else 0)
            
            if world_coord_valid:
                for i in range(5):
                    if i < len(result.circles) and 'world_coords' in result.circles[i]:
                        world_x, world_y = result.circles[i]['world_coords']
                        
                        # 轉換為整數（乘以100保留2位小數精度）
                        world_x_int = int(world_x * 100)
                        world_y_int = int(world_y * 100)
                        
                        # 分解為32位高低位
                        world_x_high = (world_x_int >> 16) & 0xFFFF
                        world_x_low = world_x_int & 0xFFFF
                        world_y_high = (world_y_int >> 16) & 0xFFFF
                        world_y_low = world_y_int & 0xFFFF
                        
                        # 寫入世界座標寄存器
                        self.write_register(f'CIRCLE_{i+1}_WORLD_X_HIGH', world_x_high)
                        self.write_register(f'CIRCLE_{i+1}_WORLD_X_LOW', world_x_low)
                        self.write_register(f'CIRCLE_{i+1}_WORLD_Y_HIGH', world_y_high)
                        self.write_register(f'CIRCLE_{i+1}_WORLD_Y_LOW', world_y_low)
                    else:
                        # 清空未使用的世界座標寄存器
                        self.write_register(f'CIRCLE_{i+1}_WORLD_X_HIGH', 0)
                        self.write_register(f'CIRCLE_{i+1}_WORLD_X_LOW', 0)
                        self.write_register(f'CIRCLE_{i+1}_WORLD_Y_HIGH', 0)
                        self.write_register(f'CIRCLE_{i+1}_WORLD_Y_LOW', 0)
            else:
                # 清空所有世界座標寄存器
                for i in range(1, 6):
                    self.write_register(f'CIRCLE_{i}_WORLD_X_HIGH', 0)
                    self.write_register(f'CIRCLE_{i}_WORLD_X_LOW', 0)
                    self.write_register(f'CIRCLE_{i}_WORLD_Y_HIGH', 0)
                    self.write_register(f'CIRCLE_{i}_WORLD_Y_LOW', 0)
            
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
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'connection_count': self.connection_count,
            'uptime_seconds': int(time.time() - self.start_time),
            'state_machine': self.state_machine.get_status_description(),
            'last_control_command': self.last_control_command,
            'command_processing': self.command_processing,
            'handshake_mode': True,
            'world_coord_supported': True
        }
    
    def get_debug_info(self) -> Dict[str, Any]:
        """獲取調試信息"""
        return {
            'connected': self.connected,
            'sync_running': self.sync_running,
            'sync_thread_alive': self.sync_thread.is_alive() if self.sync_thread else False,
            'last_control_command': self.last_control_command,
            'command_processing': self.command_processing,
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'server_address': f"{self.server_ip}:{self.server_port}",
            'state_machine': self.state_machine.get_status_description(),
            'handshake_mode': True,
            'sync_interval_ms': self.sync_interval * 1000,
            'world_coord_supported': True
        }


# ==================== 模擬版本 (當pymodbus不可用時) ====================
class MockEnhancedModbusTcpClientService(EnhancedModbusTcpClientService):
    """模擬增強型Modbus TCP Client服務（支援世界座標）"""
    
    def __init__(self, server_ip="192.168.1.100", server_port=502):
        # 調用父類初始化，但跳過Modbus相關部分
        self.server_ip = server_ip
        self.server_port = server_port
        self.client = None
        self.connected = False
        self.vision_controller = None
        
        # 狀態機管理
        self.state_machine = SystemStateMachine()
        
        # 模擬寄存器存儲
        self.registers = {}
        
        # 其他屬性與父類相同
        self.reconnect_delay = 5.0
        self.read_timeout = 3.0
        self.write_timeout = 3.0
        self.sync_enabled = False
        self.sync_thread = None
        self.sync_running = False
        self.sync_interval = 0.05
        self.last_control_command = 0
        self.command_processing = False
        
        # 初始化擴展寄存器映射（包含世界座標）
        self.REGISTERS = {
            'CONTROL_COMMAND': 200,
            'STATUS_REGISTER': 201,
            'MIN_AREA_HIGH': 210,
            'MIN_AREA_LOW': 211,
            'MIN_ROUNDNESS': 212,
            'GAUSSIAN_KERNEL': 213,
            'CANNY_LOW': 214,
            'CANNY_HIGH': 215,
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
            # 世界座標寄存器
            'WORLD_COORD_VALID': 256,
            'CIRCLE_1_WORLD_X_HIGH': 257,
            'CIRCLE_1_WORLD_X_LOW': 258,
            'CIRCLE_1_WORLD_Y_HIGH': 259,
            'CIRCLE_1_WORLD_Y_LOW': 260,
            'CIRCLE_2_WORLD_X_HIGH': 261,
            'CIRCLE_2_WORLD_X_LOW': 262,
            'CIRCLE_2_WORLD_Y_HIGH': 263,
            'CIRCLE_2_WORLD_Y_LOW': 264,
            'CIRCLE_3_WORLD_X_HIGH': 265,
            'CIRCLE_3_WORLD_X_LOW': 266,
            'CIRCLE_3_WORLD_Y_HIGH': 267,
            'CIRCLE_3_WORLD_Y_LOW': 268,
            'CIRCLE_4_WORLD_X_HIGH': 269,
            'CIRCLE_4_WORLD_X_LOW': 270,
            'CIRCLE_4_WORLD_Y_HIGH': 271,
            'CIRCLE_4_WORLD_Y_LOW': 272,
            'CIRCLE_5_WORLD_X_HIGH': 273,
            'CIRCLE_5_WORLD_X_LOW': 274,
            'CIRCLE_5_WORLD_Y_HIGH': 275,
            'CIRCLE_5_WORLD_Y_LOW': 276,
            # 統計寄存器
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
        
        # 統計計數
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
    
    def connect(self) -> bool:
        """模擬連接"""
        print(f"⚠️ 模擬連接到Modbus TCP服務器: {self.server_ip}:{self.server_port}")
        self.connected = True
        self.connection_count += 1
        
        # 初始化模擬寄存器，確保狀態寄存器固定值
        self._initialize_status_registers()
        self._update_initialization_status()
        
        # 強制設置狀態寄存器為固定值 1 (Ready=1)
        self.state_machine.reset_to_idle()
        self.registers[self.REGISTERS['STATUS_REGISTER']] = 1
        print(f"📊 模擬狀態寄存器固定為: 1 (Ready=1)")
        
        return True
    
    def read_register(self, register_name: str) -> Optional[int]:
        """模擬讀取寄存器"""
        if register_name in self.REGISTERS:
            address = self.REGISTERS[register_name]
            return self.registers.get(address, 0)
        return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """模擬寫入寄存器"""
        if register_name in self.REGISTERS:
            address = self.REGISTERS[register_name]
            self.registers[address] = value
            return True
        return False


# ==================== 圓形檢測器 ====================
class CircleDetector:
    """圓形檢測器 (保持原有邏輯)"""
    
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
                    cv2.circle(result_image, center, radius, (0, 255, 0), 3)
                    cv2.circle(result_image, center, 5, (0, 0, 255), -1)
                    
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


# ==================== CCD1視覺控制器 (新增世界座標功能) ====================
class CCD1VisionController:
    """CCD1 視覺控制器 (適配增強型Modbus服務 + 世界座標轉換)"""
    
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
        
        # 新增：相機標定管理器
        self.calibration_manager = CalibrationManager()
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1Vision")
        self.logger.setLevel(logging.INFO)
        
        # 選擇合適的Modbus Client服務
        if MODBUS_AVAILABLE:
            self.modbus_client = EnhancedModbusTcpClientService()
            print("✅ 使用增強型Modbus TCP Client服務 (運動控制握手模式 + 世界座標)")
        else:
            self.modbus_client = MockEnhancedModbusTcpClientService()
            print("⚠️ 使用模擬增強型Modbus TCP Client服務 (功能受限)")
            
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
                self.modbus_client.stop_handshake_sync()
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
                # 連接成功後自動啟動握手同步
                self.modbus_client.start_handshake_sync()
                
                return {
                    'success': True,
                    'message': f'Modbus TCP連接成功，運動控制握手模式已啟動: {self.modbus_client.server_ip}:{self.modbus_client.server_port}',
                    'connection_status': self.modbus_client.get_connection_status(),
                    'handshake_mode': True,
                    'world_coord_supported': True
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
            self.modbus_client.disconnect()  # 這會自動停止握手同步線程
            
            return {
                'success': True,
                'message': 'Modbus連接已斷開，運動控制握手同步已停止'
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'斷開Modbus連接失敗: {str(e)}'
            }
    
    def scan_calibration_files(self) -> Dict[str, Any]:
        """掃描內外參檔案"""
        return self.calibration_manager.scan_calibration_files()
    
    def load_calibration_data(self, intrinsic_file=None, extrinsic_file=None) -> Dict[str, Any]:
        """載入內外參數據"""
        return self.calibration_manager.load_calibration_data(intrinsic_file, extrinsic_file)
    
    def get_calibration_status(self) -> Dict[str, Any]:
        """獲取標定狀態"""
        return self.calibration_manager.get_status()
    
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
            
            # 更新Modbus的初始化狀態
            if self.modbus_client.connected:
                self.modbus_client._update_initialization_status()
            
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
            
            # 更新Modbus的Alarm狀態
            if self.modbus_client.connected:
                self.modbus_client.state_machine.set_alarm(True)
                self.modbus_client._update_initialization_status()
            
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
            # 設置Alarm狀態
            if self.modbus_client.connected:
                self.modbus_client.state_machine.set_alarm(True)
            return None, 0.0
    
    def capture_and_detect(self) -> VisionResult:
        """拍照並進行圓形檢測（包含世界座標轉換）"""
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
                    has_world_coords=False,
                    error_message="圖像捕獲失敗"
                )
            else:
                process_start = time.time()
                circles, annotated_image = self.detector.detect_circles(image)
                
                # 檢查是否可以進行世界座標轉換
                can_transform = (self.calibration_manager.calibration_data.is_valid and
                               self.calibration_manager.transformer and
                               self.calibration_manager.transformer.is_valid())
                
                # 如果可以轉換，計算世界座標
                if can_transform and circles:
                    try:
                        for circle in circles:
                            pixel_coords = [circle['center']]
                            world_coords = self.calibration_manager.transformer.pixel_to_world(pixel_coords)
                            
                            if world_coords is not None and len(world_coords) > 0:
                                circle['world_coords'] = (float(world_coords[0]), float(world_coords[1]))
                            else:
                                circle['world_coords'] = None
                                can_transform = False  # 轉換失敗
                                break
                    except Exception as e:
                        print(f"⚠️ 世界座標轉換失敗: {e}")
                        can_transform = False
                        # 移除已設置的world_coords
                        for circle in circles:
                            if 'world_coords' in circle:
                                del circle['world_coords']
                
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
                    success=True,
                    has_world_coords=can_transform
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
                has_world_coords=False,
                error_message=error_msg
            )
            
            # 設置Alarm狀態
            if self.modbus_client.connected:
                self.modbus_client.state_machine.set_alarm(True)
                self.modbus_client.update_detection_results(result)
            
            return result
    
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
        """獲取系統狀態（包含標定狀態）"""
        status = {
            'connected': self.is_connected,
            'camera_name': self.camera_name,
            'camera_ip': self.camera_ip,
            'has_image': self.last_image is not None,
            'last_result': asdict(self.last_result) if self.last_result else None,
            'detection_params': asdict(self.detection_params),
            'modbus_enabled': MODBUS_AVAILABLE,
            'modbus_connection': self.modbus_client.get_connection_status(),
            'handshake_mode': True,
            'world_coord_supported': True,
            'calibration_status': self.calibration_manager.get_status()
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
        
        # 更新Modbus狀態
        if self.modbus_client.connected:
            self.modbus_client.state_machine.set_alarm(True)
            self.modbus_client._update_initialization_status()
        
        # 斷開Modbus連接
        try:
            self.modbus_client.stop_handshake_sync()
            self.modbus_client.disconnect()
        except:
            pass
        
        self.logger.info("所有連接已斷開")


# ==================== Flask應用設置 ====================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ccd_vision_enhanced_handshake_world_coord_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# 創建控制器實例
vision_controller = CCD1VisionController()

# 設置日誌
logging.basicConfig(level=logging.INFO)


# ==================== 路由定義 ====================
@app.route('/')
def index():
    """主頁面"""
    return render_template('ccd_vision_enhanced_world_coord.html')


@app.route('/api/status')
def get_status():
    """獲取系統狀態"""
    return jsonify(vision_controller.get_status())


# ===== Modbus相關API =====
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


@app.route('/api/modbus/status', methods=['GET'])
def get_modbus_status():
    """獲取Modbus狀態機資訊"""
    modbus_client = vision_controller.modbus_client
    
    if not modbus_client.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus Client未連接',
            'status': {}
        })
    
    try:
        # 讀取當前寄存器狀態
        control_command = modbus_client.read_register('CONTROL_COMMAND')
        status_register = modbus_client.read_register('STATUS_REGISTER')
        world_coord_valid = modbus_client.read_register('WORLD_COORD_VALID')
        
        status_info = {
            'control_command': control_command,
            'status_register': status_register,
            'world_coord_valid': world_coord_valid,
            'state_machine': modbus_client.state_machine.get_status_description(),
            'last_control_command': modbus_client.last_control_command,
            'command_processing': modbus_client.command_processing,
            'sync_running': modbus_client.sync_running,
            'operation_count': modbus_client.operation_count,
            'error_count': modbus_client.error_count
        }
        
        return jsonify({
            'success': True,
            'message': '成功獲取Modbus狀態',
            'status': status_info,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'獲取Modbus狀態失敗: {str(e)}',
            'status': {}
        })


@app.route('/api/modbus/registers', methods=['GET'])
def get_modbus_registers():
    """獲取所有Modbus寄存器的即時數值（包含世界座標）"""
    modbus_client = vision_controller.modbus_client
    
    if not modbus_client.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus Client未連接',
            'registers': {}
        })
    
    try:
        registers = {}
        
        # 核心控制握手寄存器
        control_registers = {
            '200_控制指令': modbus_client.read_register('CONTROL_COMMAND'),
            '201_狀態寄存器': modbus_client.read_register('STATUS_REGISTER'),
        }
        
        # 解析狀態寄存器的各個位
        status_value = control_registers['201_狀態寄存器'] or 0
        status_bits = {
            '201_Ready狀態_bit0': (status_value >> 0) & 1,
            '201_Running狀態_bit1': (status_value >> 1) & 1,
            '201_Alarm狀態_bit2': (status_value >> 2) & 1,
            '201_初始化狀態_bit3': (status_value >> 3) & 1,
        }
        
        # 檢測結果寄存器（像素座標）
        result_registers = {
            '240_檢測圓形數量': modbus_client.read_register('CIRCLE_COUNT'),
        }
        
        # 圓形像素座標詳細資料
        for i in range(1, 6):
            x_val = modbus_client.read_register(f'CIRCLE_{i}_X')
            y_val = modbus_client.read_register(f'CIRCLE_{i}_Y')
            r_val = modbus_client.read_register(f'CIRCLE_{i}_RADIUS')
            result_registers[f'{240+i*3-2}_圓形{i}_X像素'] = x_val
            result_registers[f'{240+i*3-1}_圓形{i}_Y像素'] = y_val
            result_registers[f'{240+i*3}_圓形{i}_半徑'] = r_val
        
        # 世界座標寄存器
        world_coord_registers = {
            '256_世界座標有效標誌': modbus_client.read_register('WORLD_COORD_VALID'),
        }
        
        # 圓形世界座標詳細資料
        for i in range(1, 6):
            x_high = modbus_client.read_register(f'CIRCLE_{i}_WORLD_X_HIGH')
            x_low = modbus_client.read_register(f'CIRCLE_{i}_WORLD_X_LOW')
            y_high = modbus_client.read_register(f'CIRCLE_{i}_WORLD_Y_HIGH')
            y_low = modbus_client.read_register(f'CIRCLE_{i}_WORLD_Y_LOW')
            
            # 計算實際世界座標值（從32位整數恢復到浮點數）
            if x_high is not None and x_low is not None:
                x_world_int = (x_high << 16) | x_low
                x_world = x_world_int / 100.0  # 恢復小數點
            else:
                x_world = 0.0
                
            if y_high is not None and y_low is not None:
                y_world_int = (y_high << 16) | y_low
                y_world = y_world_int / 100.0  # 恢復小數點
            else:
                y_world = 0.0
            
            world_coord_registers[f'{257+i*4-4}_圓形{i}_世界X_高位'] = x_high
            world_coord_registers[f'{257+i*4-3}_圓形{i}_世界X_低位'] = x_low
            world_coord_registers[f'{257+i*4-2}_圓形{i}_世界Y_高位'] = y_high
            world_coord_registers[f'{257+i*4-1}_圓形{i}_世界Y_低位'] = y_low
            world_coord_registers[f'圓形{i}_世界座標_X'] = f"{x_world:.2f}mm"
            world_coord_registers[f'圓形{i}_世界座標_Y'] = f"{y_world:.2f}mm"
        
        # 統計資訊寄存器
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
        registers.update(status_bits)
        registers.update(result_registers)
        registers.update(world_coord_registers)
        registers.update(stats_registers)
        
        return jsonify({
            'success': True,
            'message': 'Modbus寄存器讀取成功 (運動控制握手模式 + 世界座標)',
            'registers': registers,
            'handshake_mode': True,
            'world_coord_mode': True,
            'state_machine': modbus_client.state_machine.get_status_description(),
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


@app.route('/api/modbus/manual_command', methods=['POST'])
def manual_command():
    """手動發送控制指令 (模擬PLC操作)"""
    data = request.get_json()
    command = data.get('command', 0)
    
    modbus_client = vision_controller.modbus_client
    
    if not modbus_client.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus未連接'
        })
    
    try:
        # 驗證控制指令
        valid_commands = [0, 8, 16, 32]
        if command not in valid_commands:
            return jsonify({
                'success': False,
                'message': f'無效的控制指令: {command}，有效值: {valid_commands}'
            })
        
        # 寫入控制指令
        success = modbus_client.write_register('CONTROL_COMMAND', command)
        
        if success:
            command_names = {
                0: "清空控制",
                8: "拍照", 
                16: "拍照+檢測",
                32: "重新初始化"
            }
            
            return jsonify({
                'success': True,
                'message': f'手動控制指令已發送: {command} ({command_names.get(command, "未知")})',
                'command': command,
                'command_name': command_names.get(command, "未知"),
                'state_machine': modbus_client.state_machine.get_status_description()
            })
        else:
            return jsonify({
                'success': False,
                'message': '寫入控制指令失敗'
            })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'發送手動指令失敗: {str(e)}'
        })


# ===== 標定相關API =====
@app.route('/api/calibration/scan', methods=['GET'])
def scan_calibration_files():
    """掃描標定檔案"""
    result = vision_controller.scan_calibration_files()
    return jsonify(result)


@app.route('/api/calibration/load', methods=['POST'])
def load_calibration():
    """載入標定數據"""
    data = request.get_json() or {}
    intrinsic_file = data.get('intrinsic_file')
    extrinsic_file = data.get('extrinsic_file')
    
    result = vision_controller.load_calibration_data(intrinsic_file, extrinsic_file)
    socketio.emit('status_update', vision_controller.get_status())
    
    return jsonify(result)


@app.route('/api/calibration/status', methods=['GET'])
def get_calibration_status():
    """獲取標定狀態"""
    return jsonify(vision_controller.get_calibration_status())


# ===== 其他現有API =====
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
    """拍照並檢測（包含世界座標）"""
    result = vision_controller.capture_and_detect()
    
    response = {
        'success': result.success,
        'circle_count': result.circle_count,
        'circles': result.circles,
        'capture_time_ms': round(result.capture_time * 1000, 2),
        'processing_time_ms': round(result.processing_time * 1000, 2),
        'total_time_ms': round(result.total_time * 1000, 2),
        'timestamp': result.timestamp,
        'has_world_coords': result.has_world_coords,
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


# ===== Socket.IO 事件處理 =====
@socketio.on('connect')
def handle_connect():
    """客戶端連接"""
    emit('status_update', vision_controller.get_status())


@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開"""
    pass


# ==================== 主函數 ====================
def main():
    """主函數"""
    print("🚀 CCD1 視覺控制系統啟動中 (運動控制握手版本 + 世界座標轉換)...")
    
    if not CAMERA_MANAGER_AVAILABLE:
        print("❌ 相機管理器不可用，請檢查SDK導入")
        return
    
    try:
        print("🔧 系統架構: Modbus TCP Client - 運動控制握手模式 + 世界座標轉換")
        print("📡 連接模式: 主動連接外部PLC/HMI設備")
        print("🤝 握手協議: 指令/狀態模式，50ms高頻輪詢")
        print("🌍 新功能: 內外參管理 + 像素座標到世界座標轉換")
        
        if MODBUS_AVAILABLE:
            print(f"✅ Modbus TCP Client模組可用 (pymodbus {PYMODBUS_VERSION})")
            print("📊 CCD1 運動控制握手寄存器映射 (擴展版本):")
            print("   ┌─ 控制指令寄存器 (200)")
            print("   │  • 0: 清空控制")
            print("   │  • 8: 拍照")
            print("   │  • 16: 拍照+檢測")
            print("   │  • 32: 重新初始化")
            print("   ├─ 狀態寄存器 (201) - 固定初始值")
            print("   │  • bit0: Ready狀態")
            print("   │  • bit1: Running狀態")
            print("   │  • bit2: Alarm狀態")
            print("   │  • bit3: Initialized狀態")
            print("   │  • 初始值: 1 (Ready=1, 其他=0)")
            print("   │  • 完全初始化後: 9 (Ready=1, Initialized=1)")
            print("   ├─ 檢測參數 (210-219)")
            print("   │  • 面積、圓度、圖像處理參數")
            print("   ├─ 像素座標檢測結果 (240-255)")
            print("   │  • 圓形數量、像素座標、半徑")
            print("   ├─ 世界座標檢測結果 (256-276) - 新增")
            print("   │  • 世界座標有效標誌 (256)")
            print("   │  • 5個圓形世界座標 (X,Y各32位)")
            print("   │  • 座標精度: 乘以100保留2位小數")
            print("   └─ 統計資訊 (280-299)")
            print("      • 時間統計、計數器、版本信息")
            print("")
            print("🌍 世界座標轉換功能:")
            print("   1. 自動掃描內外參檔案（camera_matrix_*.npy, dist_coeffs_*.npy, extrinsic_*.npy）")
            print("   2. 支援NPY格式檔案導入與驗證")
            print("   3. 像素座標到世界座標轉換（Z=0平面）")
            print("   4. 世界座標寄存器映射（32位精度）")
            print("   5. UI顯示世界座標（保留2位小數）")
            print("")
            print("🤝 握手邏輯:")
            print("   1. 系統初始化完成 → Ready=1")
            print("   2. PLC下控制指令 → 檢查Ready=1")
            print("   3. 開始執行 → Ready=0, Running=1")
            print("   4. 執行完成 → Running=0")
            print("   5. PLC清零指令 → Ready=1 (準備下次)")
            print("   6. 異常發生 → Alarm=1, Initialized=0")
        else:
            print("⚠️ Modbus Client功能不可用 (使用模擬模式)")
        
        print("🌐 Web介面啟動中...")
        print("📱 訪問地址: http://localhost:5051")
        print("🎯 系統功能:")
        print("   • 相機連接管理")
        print("   • 內外參檔案管理")
        print("   • 參數調整介面")
        print("   • 圓形檢測與標註")
        print("   • 世界座標轉換")
        print("   • 運動控制握手協議")
        print("   • 即時狀態監控")
        print("   • 狀態機管理")
        print("🔗 使用說明:")
        print("   1. 設置Modbus服務器IP地址")
        print("   2. 連接到外部PLC/HMI設備")
        print("   3. 初始化相機連接")
        print("   4. 將內外參NPY檔案放入同層目錄")
        print("   5. 點擊「確認導入」載入標定數據")
        print("   6. 系統自動進入握手模式")
        print("   7. PLC通過控制指令操作系統")
        print("   8. 監控狀態寄存器確認執行狀態")
        print("   9. 檢測結果包含像素座標和世界座標")
        print("=" * 80)
        
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