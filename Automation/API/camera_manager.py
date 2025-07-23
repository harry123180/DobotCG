# -*- coding: utf-8 -*-
"""
Optimized Camera Manager - 專為頻寬控制和最新幀獲取優化
基於海康威視SDK的高性能相機管理系統
特點：200Mbps頻寬限制、5FPS、無緩存最新幀模式
"""

import threading
import time
import socket
import struct
import numpy as np
from typing import Dict, List, Optional, Tuple, Callable, Any
from enum import Enum
from ctypes import *
from dataclasses import dataclass
import logging

# 導入海康SDK
try:
    from MvCameraControl_class import *
    from CameraParams_const import *
    from CameraParams_header import *
    from MvErrorDefine_const import *
    from PixelType_header import *
except ImportError as e:
    raise ImportError(f"無法導入海康SDK模組: {e}")


class CameraState(Enum):
    """相機狀態枚舉"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    STREAMING = "streaming"
    ERROR = "error"


class CameraMode(Enum):
    """相機模式枚舉"""
    CONTINUOUS = "continuous"
    TRIGGER = "trigger"
    SOFTWARE_TRIGGER = "software_trigger"


class PixelFormat(Enum):
    """像素格式枚舉"""
    MONO8 = PixelType_Gvsp_Mono8
    BAYER_GR8 = PixelType_Gvsp_BayerGR8
    BAYER_RG8 = PixelType_Gvsp_BayerRG8
    BAYER_GB8 = PixelType_Gvsp_BayerGB8
    BAYER_BG8 = PixelType_Gvsp_BayerBG8
    RGB8 = PixelType_Gvsp_RGB8_Packed
    BGR8 = PixelType_Gvsp_BGR8_Packed


@dataclass
class CameraConfig:
    """相機配置類 - 針對頻寬控制優化"""
    name: str
    ip: str
    port: int = 0
    timeout: int = 5000
    exposure_time: float = 50000.0  # 增加曝光時間配合低FPS
    gain: float = 0.0
    frame_rate: float = 5.0  # 固定5FPS
    pixel_format: PixelFormat = PixelFormat.BAYER_GR8
    width: int = 2592
    height: int = 1944
    
    # 頻寬控制相關參數
    bandwidth_limit_mbps: int = 200  # 200Mbps頻寬限制
    packet_size: int = 1500  # 降低包大小減少網路負載
    packet_delay: int = 5000  # 包間延遲 (ticks)
    enable_bandwidth_control: bool = True
    
    # 最新幀模式 - 關閉緩存
    buffer_count: int = 1  # 最小緩存
    use_latest_frame_only: bool = True  # 只保留最新幀
    
    trigger_mode: CameraMode = CameraMode.SOFTWARE_TRIGGER
    auto_reconnect: bool = True


@dataclass
class FrameData:
    """幀數據類"""
    timestamp: float
    frame_number: int
    width: int
    height: int
    pixel_format: int
    data: np.ndarray
    camera_name: str
    capture_time: float = 0.0


class OptimizedCamera:
    """優化的單相機管理類 - 頻寬控制版"""
    
    def __init__(self, config: CameraConfig, logger: logging.Logger):
        self.config = config
        self.logger = logger
        self.name = config.name
        
        # 海康SDK對象
        self.camera = MvCamera()
        self.device_info = None
        
        # 狀態管理
        self.state = CameraState.DISCONNECTED
        self.is_streaming = False
        
        # 線程同步
        self._lock = threading.RLock()
        
        # 最新幀管理 - 無緩存模式
        self.latest_frame: Optional[FrameData] = None
        self.frame_update_time: float = 0.0
        
        # 回調函數
        self.frame_callback: Optional[Callable] = None
        self.error_callback: Optional[Callable] = None
        
        # 統計信息
        self.stats = {
            'frames_captured': 0,
            'frames_dropped': 0,
            'connection_attempts': 0,
            'last_error': None,
            'average_fps': 0.0,
            'bandwidth_usage_mbps': 0.0
        }
        
        # FPS計算
        self.fps_counter = []
        self.fps_window_size = 10
    
    def connect(self) -> bool:
        """連接相機"""
        with self._lock:
            if self.state in [CameraState.CONNECTED, CameraState.STREAMING]:
                return True
            
            self.state = CameraState.CONNECTING
            self.stats['connection_attempts'] += 1
            
            try:
                # 查找設備
                if not self._find_device():
                    raise Exception(f"未找到IP為 {self.config.ip} 的設備")
                
                # 創建句柄
                ret = self.camera.MV_CC_CreateHandle(self.device_info)
                if ret != MV_OK:
                    raise Exception(f"創建句柄失敗: 0x{ret:08x}")
                
                # 打開設備
                ret = self.camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
                if ret != MV_OK:
                    raise Exception(f"打開設備失敗: 0x{ret:08x}")
                
                # 應用頻寬控制設定
                self._apply_bandwidth_control()
                
                # 設置相機參數
                self._configure_camera()
                
                self.state = CameraState.CONNECTED
                self.logger.info(f"相機 {self.name} 連接成功 - 頻寬限制: {self.config.bandwidth_limit_mbps}Mbps")
                return True
                
            except Exception as e:
                self.state = CameraState.ERROR
                self.stats['last_error'] = str(e)
                self.logger.error(f"相機 {self.name} 連接失敗: {e}")
                
                if self.error_callback:
                    self.error_callback(self.name, str(e))
                
                return False
    
    def _find_device(self) -> bool:
        """查找指定IP的設備"""
        device_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)
        
        if ret != MV_OK:
            raise Exception(f"枚舉設備失敗: 0x{ret:08x}")
        
        for i in range(device_list.nDeviceNum):
            device_info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            
            if device_info.nTLayerType == MV_GIGE_DEVICE:
                ip_int = device_info.SpecialInfo.stGigEInfo.nCurrentIp
                ip_str = socket.inet_ntoa(struct.pack('>I', ip_int))
                
                if ip_str == self.config.ip:
                    self.device_info = device_info
                    return True
        
        return False
    
    def _apply_bandwidth_control(self):
        """應用頻寬控制設定 - 200Mbps限制"""
        try:
            self.logger.info(f"開始配置頻寬控制: {self.config.bandwidth_limit_mbps}Mbps")
            
            # 1. 設置包大小 - 較小的包大小有助於頻寬控制
            ret = self.camera.MV_CC_SetIntValue("GevSCPSPacketSize", self.config.packet_size)
            if ret == MV_OK:
                self.logger.info(f"✓ 設置包大小: {self.config.packet_size} bytes")
            else:
                self.logger.warning(f"✗ 設置包大小失敗: 0x{ret:08x}")
            
            # 2. 設置包間延遲 - 主要頻寬控制手段
            if self.config.packet_delay > 0:
                ret = self.camera.MV_CC_SetIntValue("GevSCPD", self.config.packet_delay)
                if ret == MV_OK:
                    self.logger.info(f"✓ 設置包間延遲: {self.config.packet_delay} ticks")
                else:
                    self.logger.warning(f"✗ 設置包間延遲失敗: 0x{ret:08x}")
            
            # 3. 計算並設置適當的幀率以滿足頻寬限制
            max_fps_for_bandwidth = self._calculate_max_fps_for_bandwidth()
            actual_fps = min(self.config.frame_rate, max_fps_for_bandwidth)
            
            self.logger.info(f"頻寬計算: 最大FPS={max_fps_for_bandwidth:.2f}, 實際FPS={actual_fps:.2f}")
            
            # 4. 設置重傳參數 - 優化網路穩定性
            ret = self.camera.MV_GIGE_SetResend(1, 5, 100)  # 啟用重傳，5%重傳率，100ms超時
            if ret == MV_OK:
                self.logger.info("✓ 重傳參數設置成功")
            
            # 5. 設置GVSP超時
            ret = self.camera.MV_GIGE_SetGvspTimeout(self.config.timeout)
            if ret == MV_OK:
                self.logger.info(f"✓ GVSP超時設置: {self.config.timeout}ms")
            
            # 6. 嘗試直接設置頻寬限制 (某些型號支援)
            try:
                bandwidth_bytes_per_sec = self.config.bandwidth_limit_mbps * 125000  # Mbps to Bytes/s
                ret = self.camera.MV_CC_SetIntValue("GevSCBWA", bandwidth_bytes_per_sec)
                if ret == MV_OK:
                    self.logger.info(f"✓ 直接頻寬限制設置成功: {self.config.bandwidth_limit_mbps}Mbps")
                else:
                    self.logger.info("○ 設備不支援直接頻寬限制，使用其他方法控制")
            except Exception as e:
                self.logger.debug(f"直接頻寬限制嘗試失敗: {e}")
            
            self.logger.info("頻寬控制配置完成")
            
        except Exception as e:
            self.logger.error(f"頻寬控制配置失敗: {e}")
    
    def _calculate_max_fps_for_bandwidth(self) -> float:
        """根據頻寬限制計算最大幀率"""
        # 計算單幀數據量 (bytes)
        bytes_per_pixel = 1  # BAYER_GR8 = 1 byte per pixel
        frame_size_bytes = self.config.width * self.config.height * bytes_per_pixel
        
        # 考慮網路開銷 (大約20%額外開銷)
        frame_size_with_overhead = frame_size_bytes * 1.2
        
        # 計算頻寬限制下的最大FPS
        bandwidth_bytes_per_sec = self.config.bandwidth_limit_mbps * 125000  # Mbps to Bytes/s
        max_fps = bandwidth_bytes_per_sec / frame_size_with_overhead
        
        self.logger.debug(f"頻寬計算: 幀大小={frame_size_bytes/1024/1024:.2f}MB, 最大FPS={max_fps:.2f}")
        
        return max_fps
    
    def _configure_camera(self):
        """配置相機參數"""
        try:
            # 設置圖像尺寸
            ret = self.camera.MV_CC_SetIntValue("Width", self.config.width)
            if ret != MV_OK:
                self.logger.warning(f"設置寬度失敗: 0x{ret:08x}")
            
            ret = self.camera.MV_CC_SetIntValue("Height", self.config.height)
            if ret != MV_OK:
                self.logger.warning(f"設置高度失敗: 0x{ret:08x}")
            
            # 設置像素格式
            ret = self.camera.MV_CC_SetEnumValue("PixelFormat", self.config.pixel_format.value)
            if ret != MV_OK:
                self.logger.warning(f"設置像素格式失敗: 0x{ret:08x}")
            
            # 設置曝光參數
            ret = self.camera.MV_CC_SetEnumValue("ExposureAuto", 0)  # 關閉自動曝光
            if ret == MV_OK:
                ret = self.camera.MV_CC_SetFloatValue("ExposureTime", self.config.exposure_time)
                if ret == MV_OK:
                    self.logger.info(f"✓ 曝光時間設置: {self.config.exposure_time}μs")
            
            # 設置增益
            ret = self.camera.MV_CC_SetFloatValue("Gain", self.config.gain)
            if ret == MV_OK:
                self.logger.info(f"✓ 增益設置: {self.config.gain}")
            
            # 設置幀率 - 關鍵設定
            ret = self.camera.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True)
            if ret == MV_OK:
                ret = self.camera.MV_CC_SetFloatValue("AcquisitionFrameRate", self.config.frame_rate)
                if ret == MV_OK:
                    self.logger.info(f"✓ 幀率設置: {self.config.frame_rate} FPS")
                else:
                    self.logger.warning(f"✗ 幀率設置失敗: 0x{ret:08x}")
            
            # 設置觸發模式
            if self.config.trigger_mode == CameraMode.CONTINUOUS:
                ret = self.camera.MV_CC_SetEnumValue("TriggerMode", 0)
                if ret == MV_OK:
                    self.logger.info("✓ 連續採集模式")
            else:
                self.camera.MV_CC_SetEnumValue("TriggerMode", 1)
                self.camera.MV_CC_SetEnumValue("TriggerSource", 7)  # 軟觸發
                self.logger.info("✓ 觸發模式")
            
            # 設置最小緩存 - 最新幀模式
            ret = self.camera.MV_CC_SetImageNodeNum(self.config.buffer_count)
            if ret == MV_OK:
                self.logger.info(f"✓ 緩存節點: {self.config.buffer_count} (最新幀模式)")
            
            # 設置抓取策略 - 總是獲取最新幀
            ret = self.camera.MV_CC_SetGrabStrategy(MV_GrabStrategy_LatestImagesOnly)
            if ret == MV_OK:
                self.logger.info("✓ 抓取策略: 僅最新幀")
            
            self.logger.info(f"相機 {self.name} 參數配置完成")
            
        except Exception as e:
            raise Exception(f"配置相機參數失敗: {e}")
    
    def start_streaming(self) -> bool:
        """開始串流"""
        with self._lock:
            if self.state != CameraState.CONNECTED:
                return False
            
            try:
                ret = self.camera.MV_CC_StartGrabbing()
                if ret != MV_OK:
                    raise Exception(f"開始串流失敗: 0x{ret:08x}")
                
                self.is_streaming = True
                self.state = CameraState.STREAMING
                self.logger.info(f"相機 {self.name} 開始串流 - {self.config.frame_rate}FPS")
                return True
                
            except Exception as e:
                self.state = CameraState.ERROR
                self.stats['last_error'] = str(e)
                self.logger.error(f"相機 {self.name} 開始串流失敗: {e}")
                return False
    
    def stop_streaming(self) -> bool:
        """停止串流"""
        with self._lock:
            if not self.is_streaming:
                return True
            
            try:
                ret = self.camera.MV_CC_StopGrabbing()
                if ret != MV_OK:
                    self.logger.warning(f"停止串流警告: 0x{ret:08x}")
                
                self.is_streaming = False
                self.state = CameraState.CONNECTED
                self.logger.info(f"相機 {self.name} 停止串流")
                return True
                
            except Exception as e:
                self.logger.error(f"相機 {self.name} 停止串流失敗: {e}")
                return False
    
    def capture_latest_frame(self, timeout: int = None) -> Optional[FrameData]:
        """捕獲最新幀 - 無緩存模式"""
        if not self.is_streaming:
            return None
        
        if timeout is None:
            timeout = self.config.timeout
        
        capture_start = time.time()
        
        try:
            # 清除舊的緩存數據，確保獲取最新幀
            self.camera.MV_CC_ClearImageBuffer()
            
            # 使用GetImageBuffer獲取圖像
            frame_out = MV_FRAME_OUT()
            memset(byref(frame_out), 0, sizeof(frame_out))
            
            ret = self.camera.MV_CC_GetImageBuffer(frame_out, timeout)
            if ret != MV_OK:
                if ret == MV_E_NODATA:
                    return None  # 無數據，正常情況
                else:
                    raise Exception(f"獲取圖像失敗: 0x{ret:08x}")
            
            try:
                # 複製圖像數據
                data_size = frame_out.stFrameInfo.nFrameLen
                data_buffer = (c_ubyte * data_size)()
                cdll.msvcrt.memcpy(byref(data_buffer), frame_out.pBufAddr, data_size)
                
                # 轉換為numpy數組
                raw_data = np.frombuffer(data_buffer, dtype=np.uint8)
                
                # 根據像素格式重塑數組
                if frame_out.stFrameInfo.enPixelType in [PixelType_Gvsp_Mono8, PixelType_Gvsp_BayerGR8]:
                    image_data = raw_data.reshape((frame_out.stFrameInfo.nHeight, frame_out.stFrameInfo.nWidth))
                else:
                    # 其他格式待擴展
                    image_data = raw_data
                
                capture_time = time.time() - capture_start
                
                # 創建幀數據對象
                frame_data = FrameData(
                    timestamp=time.time(),
                    frame_number=frame_out.stFrameInfo.nFrameNum,
                    width=frame_out.stFrameInfo.nWidth,
                    height=frame_out.stFrameInfo.nHeight,
                    pixel_format=frame_out.stFrameInfo.enPixelType,
                    data=image_data,
                    camera_name=self.name,
                    capture_time=capture_time
                )
                
                # 更新統計
                self.stats['frames_captured'] += 1
                self._update_fps_stats()
                self._calculate_bandwidth_usage(frame_data)
                
                # 更新最新幀
                self.latest_frame = frame_data
                self.frame_update_time = time.time()
                
                # 調用回調
                if self.frame_callback:
                    self.frame_callback(frame_data)
                
                return frame_data
                
            finally:
                # 釋放圖像緩存
                self.camera.MV_CC_FreeImageBuffer(frame_out)
                
        except Exception as e:
            self.stats['last_error'] = str(e)
            self.logger.error(f"相機 {self.name} 捕獲幀失敗: {e}")
            return None
    
    def get_latest_frame(self) -> Optional[FrameData]:
        """獲取最新幀（無等待）"""
        return self.latest_frame
    
    def get_latest_image(self) -> Optional[np.ndarray]:
        """獲取最新圖像數據"""
        frame = self.get_latest_frame()
        return frame.data if frame else None
    
    def _update_fps_stats(self):
        """更新FPS統計"""
        current_time = time.time()
        self.fps_counter.append(current_time)
        
        # 保持窗口大小
        if len(self.fps_counter) > self.fps_window_size:
            self.fps_counter.pop(0)
        
        # 計算FPS
        if len(self.fps_counter) >= 2:
            time_span = self.fps_counter[-1] - self.fps_counter[0]
            if time_span > 0:
                self.stats['average_fps'] = (len(self.fps_counter) - 1) / time_span
    
    def _calculate_bandwidth_usage(self, frame_data: FrameData):
        """計算頻寬使用量"""
        frame_size_mb = frame_data.data.nbytes / (1024 * 1024)
        fps = self.stats.get('average_fps', 0)
        self.stats['bandwidth_usage_mbps'] = frame_size_mb * fps * 8  # 轉換為Mbps
    
    def trigger_software(self) -> bool:
        """軟觸發"""
        # 修正：檢查條件，應該允許SOFTWARE_TRIGGER和TRIGGER模式
        if self.config.trigger_mode not in [CameraMode.TRIGGER, CameraMode.SOFTWARE_TRIGGER]:
            self.logger.error(f"相機 {self.name} 觸發模式不正確: {self.config.trigger_mode}")
            return False
        
        try:
            ret = self.camera.MV_CC_SetCommandValue("TriggerSoftware")
            if ret == MV_OK:
                self.logger.debug(f"相機 {self.name} 軟體觸發成功")
                return True
            else:
                self.logger.error(f"相機 {self.name} 軟體觸發失敗: 0x{ret:08x}")
                return False
        except Exception as e:
            self.logger.error(f"相機 {self.name} 軟觸發異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        with self._lock:
            try:
                # 停止串流
                if self.is_streaming:
                    self.stop_streaming()
                
                # 關閉設備
                if self.state in [CameraState.CONNECTED, CameraState.STREAMING]:
                    self.camera.MV_CC_CloseDevice()
                
                # 銷毀句柄
                self.camera.MV_CC_DestroyHandle()
                
                self.state = CameraState.DISCONNECTED
                self.logger.info(f"相機 {self.name} 已斷開")
                
            except Exception as e:
                self.logger.error(f"相機 {self.name} 斷開失敗: {e}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """獲取統計信息"""
        return {
            'name': self.name,
            'state': self.state.value,
            'is_streaming': self.is_streaming,
            'config': {
                'ip': self.config.ip,
                'frame_rate': self.config.frame_rate,
                'bandwidth_limit_mbps': self.config.bandwidth_limit_mbps,
                'resolution': f"{self.config.width}x{self.config.height}"
            },
            'stats': self.stats,
            'latest_frame_age': time.time() - self.frame_update_time if self.frame_update_time else None
        }


class OptimizedCameraManager:
    """優化的相機管理器主類 - 頻寬控制版"""
    
    def __init__(self, config_dict: Dict[str, CameraConfig] = None, 
                 log_level: int = logging.INFO):
        # 配置日誌
        self.logger = self._setup_logger(log_level)
        
        # 初始化SDK
        ret = MvCamera.MV_CC_Initialize()
        if ret != MV_OK:
            raise Exception(f"初始化SDK失敗: 0x{ret:08x}")
        
        # 相機管理
        self.cameras: Dict[str, OptimizedCamera] = {}
        self.config_dict = config_dict or {}
        
        # 全局回調
        self.global_frame_callback: Optional[Callable] = None
        self.global_error_callback: Optional[Callable] = None
        
        self.logger.info("相機管理器初始化完成 - 頻寬控制版")
    
    def _setup_logger(self, log_level: int) -> logging.Logger:
        """設置日誌"""
        logger = logging.getLogger("CameraManager")
        logger.setLevel(log_level)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        
        return logger
    
    def add_camera(self, name: str, config: CameraConfig) -> bool:
        """添加相機"""
        if name in self.cameras:
            self.logger.warning(f"相機 {name} 已存在")
            return False
        
        try:
            camera = OptimizedCamera(config, self.logger)
            
            # 設置回調
            if self.global_frame_callback:
                camera.frame_callback = self.global_frame_callback
            if self.global_error_callback:
                camera.error_callback = self.global_error_callback
            
            self.cameras[name] = camera
            self.config_dict[name] = config
            
            self.logger.info(f"添加相機 {name} - IP: {config.ip}, 頻寬: {config.bandwidth_limit_mbps}Mbps")
            return True
            
        except Exception as e:
            self.logger.error(f"添加相機 {name} 失敗: {e}")
            return False
    
    def connect_camera(self, name: str) -> bool:
        """連接指定相機"""
        if name not in self.cameras:
            self.logger.error(f"相機 {name} 不存在")
            return False
        
        return self.cameras[name].connect()
    
    def connect_all_cameras(self) -> Dict[str, bool]:
        """連接所有相機"""
        self.logger.info("開始連接所有相機...")
        
        results = {}
        for name in self.cameras:
            results[name] = self.connect_camera(name)
        
        success_count = sum(results.values())
        self.logger.info(f"相機連接完成: {success_count}/{len(self.cameras)} 成功")
        
        return results
    
    def start_streaming(self, camera_names: List[str] = None) -> Dict[str, bool]:
        """開始指定相機串流"""
        if camera_names is None:
            camera_names = list(self.cameras.keys())
        
        results = {}
        for name in camera_names:
            if name in self.cameras:
                results[name] = self.cameras[name].start_streaming()
            else:
                results[name] = False
        
        return results
    
    def stop_streaming(self, camera_names: List[str] = None) -> Dict[str, bool]:
        """停止指定相機串流"""
        if camera_names is None:
            camera_names = list(self.cameras.keys())
        
        results = {}
        for name in camera_names:
            if name in self.cameras:
                results[name] = self.cameras[name].stop_streaming()
            else:
                results[name] = False
        
        return results
    
    def get_latest_image(self, camera_name: str) -> Optional[np.ndarray]:
        """獲取指定相機的最新圖像"""
        if camera_name not in self.cameras:
            self.logger.error(f"相機 {camera_name} 不存在")
            return None
        
        return self.cameras[camera_name].get_latest_image()
    
    def capture_new_frame(self, camera_name: str, timeout: int = None) -> Optional[FrameData]:
        """主動捕獲新幀"""
        if camera_name not in self.cameras:
            return None
        
        return self.cameras[camera_name].capture_latest_frame(timeout)
    
    def get_camera_statistics(self, camera_name: str = None) -> Dict[str, Dict[str, Any]]:
        """獲取相機統計信息"""
        if camera_name:
            if camera_name in self.cameras:
                return {camera_name: self.cameras[camera_name].get_statistics()}
            return {}
        else:
            stats = {}
            for name, camera in self.cameras.items():
                stats[name] = camera.get_statistics()
            return stats
    
    def trigger_software(self, camera_names: List[str] = None) -> Dict[str, bool]:
        """軟觸發指定相機"""
        if camera_names is None:
            camera_names = [name for name, cam in self.cameras.items() 
                          if cam.config.trigger_mode == CameraMode.SOFTWARE_TRIGGER]
        
        results = {}
        for name in camera_names:
            if name in self.cameras:
                results[name] = self.cameras[name].trigger_software()
            else:
                results[name] = False
        
        return results
    
    def shutdown(self):
        """關閉管理器"""
        self.logger.info("開始關閉相機管理器...")
        
        # 斷開所有相機
        for name, camera in self.cameras.items():
            try:
                camera.disconnect()
            except Exception as e:
                self.logger.error(f"斷開相機 {name} 失敗: {e}")
        
        # 反初始化SDK
        try:
            MvCamera.MV_CC_Finalize()
        except Exception as e:
            self.logger.error(f"反初始化SDK失敗: {e}")
        
        self.logger.info("相機管理器已關閉")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()


# ==================== 便利函數 ====================

# 全局管理器實例
_global_manager: Optional[OptimizedCameraManager] = None

# 默認配置 - 200Mbps, 5FPS
CAMERA_CONFIG = {
    "cam_1": CameraConfig(
        name="cam_1",
        ip="192.168.1.8",
        bandwidth_limit_mbps=200,
        frame_rate=5.0,
        exposure_time=50000.0,
        use_latest_frame_only=True
    ),
}


def initialize_camera(ip: str = "192.168.1.8", name: str = "cam_1") -> bool:
    """初始化單個相機（兼容原API）"""
    global _global_manager
    
    try:
        config = CameraConfig(
            name=name,
            ip=ip,
            bandwidth_limit_mbps=200,
            frame_rate=5.0,
            exposure_time=50000.0,
            use_latest_frame_only=True
        )
        
        _global_manager = OptimizedCameraManager()
        _global_manager.add_camera(name, config)
        
        # 連接並開始串流
        if _global_manager.connect_camera(name):
            results = _global_manager.start_streaming([name])
            return results.get(name, False)
        
        return False
        
    except Exception as e:
        print(f"初始化相機失敗: {e}")
        return False


def get_latest_image(camera_name: str = "cam_1") -> Optional[np.ndarray]:
    """獲取最新圖像（兼容原API）"""
    global _global_manager
    
    if _global_manager is None:
        raise RuntimeError("相機管理器未初始化")
    
    return _global_manager.get_latest_image(camera_name)


def capture_new_image(camera_name: str = "cam_1") -> Optional[np.ndarray]:
    """主動捕獲新圖像"""
    global _global_manager
    
    if _global_manager is None:
        raise RuntimeError("相機管理器未初始化")
    
    frame_data = _global_manager.capture_new_frame(camera_name)
    return frame_data.data if frame_data else None


def get_camera_stats(camera_name: str = "cam_1") -> Dict:
    """獲取相機統計信息"""
    global _global_manager
    
    if _global_manager is None:
        return {}
    
    return _global_manager.get_camera_statistics(camera_name)


def shutdown_camera():
    """關閉相機（兼容原API）"""
    global _global_manager
    
    if _global_manager is not None:
        _global_manager.shutdown()
        _global_manager = None


# ==================== 測試代碼 ====================
