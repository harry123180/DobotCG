# -*- coding: utf-8 -*-
"""
Optimized Camera Manager - 高度整合海康SDK的優化相機管理器
基於海康威視SDK的高性能多相機管理系統
"""

import threading
import time
import queue
import socket
import struct
import numpy as np
from typing import Dict, List, Optional, Tuple, Callable, Any
from enum import Enum
from ctypes import *
from dataclasses import dataclass
import logging
from concurrent.futures import ThreadPoolExecutor

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
    PAUSED = "paused"


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
    """相機配置類"""
    name: str
    ip: str
    port: int = 0
    timeout: int = 3000
    exposure_time: float = 20000.0  # 微秒
    gain: float = 0.0
    frame_rate: float = 30.0
    pixel_format: PixelFormat = PixelFormat.BAYER_GR8
    width: int = 2592
    height: int = 1944
    packet_size: int = 8192
    auto_reconnect: bool = True
    buffer_count: int = 5
    trigger_mode: CameraMode = CameraMode.CONTINUOUS


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


class CameraPerformanceMonitor:
    """相機性能監控器"""
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.frame_times = []
        self.capture_times = []
        self.error_count = 0
        self.total_frames = 0
        self.start_time = time.time()
        self._lock = threading.Lock()
    
    def record_frame(self, capture_time: float):
        """記錄幀數據"""
        with self._lock:
            current_time = time.time()
            self.frame_times.append(current_time)
            self.capture_times.append(capture_time)
            self.total_frames += 1
            
            # 保持窗口大小
            if len(self.frame_times) > self.window_size:
                self.frame_times.pop(0)
                self.capture_times.pop(0)
    
    def record_error(self):
        """記錄錯誤"""
        with self._lock:
            self.error_count += 1
    
    def get_fps(self) -> float:
        """獲取當前FPS"""
        with self._lock:
            if len(self.frame_times) < 2:
                return 0.0
            
            time_span = self.frame_times[-1] - self.frame_times[0]
            if time_span <= 0:
                return 0.0
            
            return (len(self.frame_times) - 1) / time_span
    
    def get_average_capture_time(self) -> float:
        """獲取平均捕獲時間"""
        with self._lock:
            if not self.capture_times:
                return 0.0
            return sum(self.capture_times) / len(self.capture_times)
    
    def get_statistics(self) -> Dict[str, Any]:
        """獲取統計信息"""
        with self._lock:
            return {
                'fps': self.get_fps(),
                'avg_capture_time': self.get_average_capture_time(),
                'total_frames': self.total_frames,
                'error_count': self.error_count,
                'error_rate': self.error_count / max(1, self.total_frames),
                'uptime': time.time() - self.start_time
            }


class OptimizedCamera:
    """優化的單相機管理類"""
    
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
        self.last_frame_time = 0.0
        
        # 線程同步
        self._lock = threading.RLock()
        self._frame_event = threading.Event()
        
        # 幀緩存管理
        self.frame_buffer = queue.Queue(maxsize=config.buffer_count)
        self.latest_frame: Optional[FrameData] = None
        
        # 性能監控
        self.performance = CameraPerformanceMonitor()
        
        # 回調函數
        self.frame_callback: Optional[Callable] = None
        self.error_callback: Optional[Callable] = None
        
        # 自動重連
        self.reconnect_thread: Optional[threading.Thread] = None
        self.should_reconnect = False
        
        # 統計信息
        self.stats = {
            'frames_captured': 0,
            'frames_dropped': 0,
            'connection_attempts': 0,
            'last_error': None
        }
    
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
                
                # 優化網絡設置
                self._optimize_network_settings()
                
                # 設置相機參數
                self._configure_camera()
                
                self.state = CameraState.CONNECTED
                self.logger.info(f"相機 {self.name} 連接成功")
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
    
    def _optimize_network_settings(self):
        """優化網絡設置"""
        try:
            # 設置最佳包大小
            optimal_packet_size = self.camera.MV_CC_GetOptimalPacketSize()
            if optimal_packet_size > 0:
                packet_size = min(optimal_packet_size, self.config.packet_size)
                ret = self.camera.MV_CC_SetIntValue("GevSCPSPacketSize", packet_size)
                if ret == MV_OK:
                    self.logger.debug(f"設置包大小: {packet_size}")
            
            # 設置重傳參數
            self.camera.MV_GIGE_SetResend(1, 10, 50)  # 啟用重傳，10%重傳率，50ms超時
            
            # 設置GVSP超時
            self.camera.MV_GIGE_SetGvspTimeout(self.config.timeout)
            
        except Exception as e:
            self.logger.warning(f"優化網絡設置失敗: {e}")
    
    def _configure_camera(self):
        """配置相機參數"""
        try:
            # 設置圖像尺寸
            self.camera.MV_CC_SetIntValue("Width", self.config.width)
            self.camera.MV_CC_SetIntValue("Height", self.config.height)
            
            # 設置像素格式
            self.camera.MV_CC_SetEnumValue("PixelFormat", self.config.pixel_format.value)
            
            # 設置曝光參數
            self.camera.MV_CC_SetEnumValue("ExposureAuto", 0)  # 關閉自動曝光
            self.camera.MV_CC_SetFloatValue("ExposureTime", self.config.exposure_time)
            
            # 設置增益
            self.camera.MV_CC_SetFloatValue("Gain", self.config.gain)
            
            # 設置幀率
            self.camera.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True)
            self.camera.MV_CC_SetFloatValue("AcquisitionFrameRate", self.config.frame_rate)
            
            # 設置觸發模式
            if self.config.trigger_mode == CameraMode.CONTINUOUS:
                self.camera.MV_CC_SetEnumValue("TriggerMode", 0)
            else:
                self.camera.MV_CC_SetEnumValue("TriggerMode", 1)
                self.camera.MV_CC_SetEnumValue("TriggerSource", 7)  # 軟觸發
            
            # 設置圖像緩存
            self.camera.MV_CC_SetImageNodeNum(self.config.buffer_count)
            
            self.logger.debug(f"相機 {self.name} 參數配置完成")
            
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
                self.logger.info(f"相機 {self.name} 開始串流")
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
    
    def capture_frame(self, timeout: int = None) -> Optional[FrameData]:
        """捕獲單幀"""
        if not self.is_streaming:
            return None
        
        if timeout is None:
            timeout = self.config.timeout
        
        capture_start = time.time()
        
        try:
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
                self.performance.record_frame(capture_time)
                self.latest_frame = frame_data
                
                # 更新幀緩存
                try:
                    self.frame_buffer.put_nowait(frame_data)
                except queue.Full:
                    # 緩存滿，丟棄最舊的幀
                    try:
                        self.frame_buffer.get_nowait()
                        self.frame_buffer.put_nowait(frame_data)
                        self.stats['frames_dropped'] += 1
                    except queue.Empty:
                        pass
                
                # 調用回調
                if self.frame_callback:
                    self.frame_callback(frame_data)
                
                return frame_data
                
            finally:
                # 釋放圖像緩存
                self.camera.MV_CC_FreeImageBuffer(frame_out)
                
        except Exception as e:
            self.performance.record_error()
            self.stats['last_error'] = str(e)
            self.logger.error(f"相機 {self.name} 捕獲幀失敗: {e}")
            return None
    
    def trigger_software(self) -> bool:
        """軟觸發"""
        if self.config.trigger_mode != CameraMode.TRIGGER:
            return False
        
        try:
            ret = self.camera.MV_CC_SetCommandValue("TriggerSoftware")
            return ret == MV_OK
        except Exception as e:
            self.logger.error(f"相機 {self.name} 軟觸發失敗: {e}")
            return False
    
    def get_latest_frame(self) -> Optional[FrameData]:
        """獲取最新幀"""
        return self.latest_frame
    
    def get_buffered_frame(self, timeout: float = 0.1) -> Optional[FrameData]:
        """從緩存獲取幀"""
        try:
            return self.frame_buffer.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def disconnect(self):
        """斷開連接"""
        with self._lock:
            try:
                # 停止自動重連
                self.should_reconnect = False
                
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
        perf_stats = self.performance.get_statistics()
        return {
            'name': self.name,
            'state': self.state.value,
            'is_streaming': self.is_streaming,
            'config': self.config.__dict__,
            'stats': self.stats,
            'performance': perf_stats
        }


class OptimizedCameraManager:
    """優化的相機管理器主類"""
    
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
        
        # 線程管理
        self.executor = ThreadPoolExecutor(max_workers=10, thread_name_prefix="CamMgr")
        self._shutdown_flag = threading.Event()
        
        # 全局回調
        self.global_frame_callback: Optional[Callable] = None
        self.global_error_callback: Optional[Callable] = None
        
        # 監控線程
        self.monitor_thread: Optional[threading.Thread] = None
        self.start_monitoring = False
        
        self.logger.info("相機管理器初始化完成")
    
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
            
            self.logger.info(f"添加相機 {name}")
            return True
            
        except Exception as e:
            self.logger.error(f"添加相機 {name} 失敗: {e}")
            return False
    
    def remove_camera(self, name: str) -> bool:
        """移除相機"""
        if name not in self.cameras:
            return False
        
        try:
            camera = self.cameras[name]
            camera.disconnect()
            del self.cameras[name]
            
            if name in self.config_dict:
                del self.config_dict[name]
            
            self.logger.info(f"移除相機 {name}")
            return True
            
        except Exception as e:
            self.logger.error(f"移除相機 {name} 失敗: {e}")
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
        
        # 並行連接
        futures = {}
        for name in self.cameras:
            future = self.executor.submit(self.connect_camera, name)
            futures[name] = future
        
        # 收集結果
        results = {}
        for name, future in futures.items():
            try:
                results[name] = future.result(timeout=10)
            except Exception as e:
                self.logger.error(f"連接相機 {name} 超時: {e}")
                results[name] = False
        
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
    
    def get_image(self, camera_name: str, timeout: int = None) -> Optional[np.ndarray]:
        """獲取指定相機的圖像"""
        if camera_name not in self.cameras:
            self.logger.error(f"相機 {camera_name} 不存在")
            return None
        
        camera = self.cameras[camera_name]
        frame_data = camera.capture_frame(timeout)
        
        if frame_data:
            return frame_data.data
        return None
    
    def get_image_data(self, camera_name: str, timeout: int = None) -> Optional[FrameData]:
        """獲取指定相機的完整幀數據"""
        if camera_name not in self.cameras:
            return None
        
        return self.cameras[camera_name].capture_frame(timeout)
    
    def get_latest_image(self, camera_name: str) -> Optional[np.ndarray]:
        """獲取最新圖像（無等待）"""
        if camera_name not in self.cameras:
            return None
        
        frame_data = self.cameras[camera_name].get_latest_frame()
        return frame_data.data if frame_data else None
    
    def trigger_software(self, camera_names: List[str] = None) -> Dict[str, bool]:
        """軟觸發指定相機"""
        if camera_names is None:
            camera_names = [name for name, cam in self.cameras.items() 
                          if cam.config.trigger_mode == CameraMode.TRIGGER]
        
        results = {}
        for name in camera_names:
            if name in self.cameras:
                results[name] = self.cameras[name].trigger_software()
            else:
                results[name] = False
        
        return results
    
    def get_all_statistics(self) -> Dict[str, Dict[str, Any]]:
        """獲取所有相機統計信息"""
        stats = {}
        for name, camera in self.cameras.items():
            stats[name] = camera.get_statistics()
        return stats
    
    def get_camera_list(self) -> List[str]:
        """獲取相機名稱列表"""
        return list(self.cameras.keys())
    
    def is_camera_streaming(self, camera_name: str) -> bool:
        """檢查相機是否在串流"""
        if camera_name not in self.cameras:
            return False
        return self.cameras[camera_name].is_streaming
    
    def set_global_frame_callback(self, callback: Callable):
        """設置全局幀回調"""
        self.global_frame_callback = callback
        for camera in self.cameras.values():
            camera.frame_callback = callback
    
    def set_global_error_callback(self, callback: Callable):
        """設置全局錯誤回調"""
        self.global_error_callback = callback
        for camera in self.cameras.values():
            camera.error_callback = callback
    
    def shutdown(self):
        """關閉管理器"""
        self.logger.info("開始關閉相機管理器...")
        
        # 停止監控
        self._shutdown_flag.set()
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2)
        
        # 斷開所有相機
        for name, camera in self.cameras.items():
            try:
                camera.disconnect()
            except Exception as e:
                self.logger.error(f"斷開相機 {name} 失敗: {e}")
        
        # 關閉線程池
        self.executor.shutdown(wait=True, timeout=5)
        
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

# 默認配置
CAMERA_CONFIG = {
    "cam_1": CameraConfig(
        name="cam_1",
        ip="192.168.1.8",
        exposure_time=20000.0,
        gain=0.0,
        frame_rate=30.0
    ),
    "cam_2": CameraConfig(
        name="cam_2", 
        ip="192.168.1.9",
        exposure_time=20000.0,
        gain=0.0,
        frame_rate=30.0
    ),
    "cam_3": CameraConfig(
        name="cam_3",
        ip="192.168.1.10", 
        exposure_time=20000.0,
        gain=0.0,
        frame_rate=30.0
    ),
}


def initialize_all_cameras(config_dict: Dict[str, CameraConfig] = None) -> bool:
    """初始化所有相機（兼容原API）"""
    global _global_manager
    
    if _global_manager is not None:
        return True
    
    try:
        if config_dict is None:
            config_dict = CAMERA_CONFIG
        
        _global_manager = OptimizedCameraManager(config_dict)
        
        # 添加所有相機
        for name, config in config_dict.items():
            _global_manager.add_camera(name, config)
        
        # 連接所有相機
        results = _global_manager.connect_all_cameras()
        
        # 開始串流
        _global_manager.start_streaming()
        
        return all(results.values())
        
    except Exception as e:
        print(f"初始化相機失敗: {e}")
        return False


def get_image(camera_name: str) -> Optional[bytes]:
    """獲取圖像數據（兼容原API）"""
    global _global_manager
    
    if _global_manager is None:
        raise RuntimeError("相機管理器未初始化")
    
    image_data = _global_manager.get_image(camera_name)
    if image_data is not None:
        return image_data.tobytes()
    return None


def get_image_array(camera_name: str) -> Optional[np.ndarray]:
    """獲取圖像數組"""
    global _global_manager
    
    if _global_manager is None:
        raise RuntimeError("相機管理器未初始化")
    
    return _global_manager.get_image(camera_name)


def shutdown_all():
    """關閉所有相機（兼容原API）"""
    global _global_manager
    
    if _global_manager is not None:
        _global_manager.shutdown()
        _global_manager = None


def get_manager() -> Optional[OptimizedCameraManager]:
    """獲取全局管理器實例"""
    return _global_manager


def get_camera_statistics(camera_name: str = None) -> Dict:
    """獲取相機統計信息"""
    global _global_manager
    
    if _global_manager is None:
        return {}
    
    if camera_name:
        if camera_name in _global_manager.cameras:
            return _global_manager.cameras[camera_name].get_statistics()
        return {}
    else:
        return _global_manager.get_all_statistics()


# ==================== 測試代碼 ====================

if __name__ == "__main__":
    # 測試配置
    test_config = {
        "test_cam": CameraConfig(
            name="test_cam",
            ip="192.168.1.8",  # 修改為您的相機IP
            exposure_time=10000.0,
            frame_rate=10.0,
            trigger_mode=CameraMode.CONTINUOUS
        )
    }
    
    print("🚀 測試優化相機管理器")
    
    # 創建管理器
    with OptimizedCameraManager(test_config, logging.DEBUG) as manager:
        # 添加相機
        success = manager.add_camera("test_cam", test_config["test_cam"])
        print(f"添加相機: {'成功' if success else '失敗'}")
        
        # 連接相機
        results = manager.connect_all_cameras()
        print(f"連接結果: {results}")
        
        if results.get("test_cam", False):
            # 開始串流
            stream_results = manager.start_streaming()
            print(f"串流結果: {stream_results}")
            
            # 捕獲幾幀
            for i in range(5):
                frame_data = manager.get_image_data("test_cam")
                if frame_data:
                    print(f"幀 {i+1}: {frame_data.width}x{frame_data.height}, "
                          f"捕獲耗時: {frame_data.capture_time:.3f}s")
                else:
                    print(f"幀 {i+1}: 捕獲失敗")
                
                time.sleep(0.1)
            
            # 獲取統計信息
            stats = manager.get_all_statistics()
            print(f"統計信息: {stats}")
    
    print("✅ 測試完成")