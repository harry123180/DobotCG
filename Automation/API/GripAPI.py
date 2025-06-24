"""
夾爪控制API - 自動化生產線系統
=================================

此API提供完整的夾爪控制功能，包含基本控制、狀態監控、安全檢查等功能
適用於透過USB轉RS485介面的Modbus RTU通訊協定控制夾爪

作者: 自動化系統團隊
版本: v1.0.0
更新日期: 2025-06-05
"""

import time
import threading
import logging
from typing import Optional, Dict, Any, Tuple, Union, Callable
from dataclasses import dataclass
from enum import Enum
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import json


class GripperStatus(Enum):
    """夾爪運行狀態枚舉"""
    OFFLINE = "OFFLINE"
    IDLE = "IDLE"
    MOVING = "MOVING"
    GRIPPING = "GRIPPING"
    ERROR = "ERROR"
    INITIALIZING = "INITIALIZING"


class GripperMode(Enum):
    """夾爪操作模式"""
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    MAINTENANCE = "MAINTENANCE"


@dataclass
class GripperConfig:
    """夾爪運行參數配置"""
    max_position: int = 1000          # 最大開啟位置 (千分比)
    min_position: int = 0             # 最小關閉位置 (千分比)
    default_force: int = 50           # 預設夾持力道 (20-100%)
    default_speed: int = 40           # 預設移動速度 (1-100%)
    max_force: int = 100              # 最大允許力道
    min_force: int = 20               # 最小有效力道
    timeout_seconds: float = 5.0      # 動作超時時間 (秒)
    safety_interval: float = 0.1      # 指令間安全間隔 (秒)


@dataclass
class GripperState:
    """夾爪即時狀態資訊"""
    current_position: int = 0         # 當前位置 (千分比)
    target_position: int = 0          # 目標位置 (千分比)
    current_force: int = 0            # 當前力道設定
    current_speed: int = 0            # 當前速度設定
    is_moving: bool = False           # 是否正在移動
    is_gripping: bool = False         # 是否正在夾持物件
    temperature: float = 0.0          # 夾爪溫度 (如果支援)
    current: float = 0.0              # 工作電流 (如果支援)
    error_code: int = 0               # 錯誤代碼
    uptime: float = 0.0               # 累計運行時間
    last_update: float = 0.0          # 狀態最後更新時間


class GripperController:
    """
    夾爪控制API主類
    ============
    
    提供完整的夾爪控制功能，包含：
    - 基本運動控制 (開啟/關閉/移動到指定位置)
    - 參數設定 (力道/速度調整)
    - 狀態監控 (位置/力道/錯誤狀態)
    - 安全功能 (緊急停止/限位保護)
    - 高級功能 (軟夾持/漸進控制)
    """
    
    def __init__(self, 
                 port: str = 'COM3',
                 baudrate: int = 115200,
                 parity: str = 'N',
                 stopbits: int = 1,
                 unit_id: int = 6,
                 config: Optional[GripperConfig] = None,
                 logger: Optional[logging.Logger] = None):
        """
        初始化夾爪控制API
        
        參數:
            port (str): RS485串口埠號，預設 'COM3'
            baudrate (int): 通訊鮑率，預設 115200
            parity (str): 奇偶校驗位，預設 'N' (無校驗)
            stopbits (int): 停止位數，預設 1
            unit_id (int): Modbus從站號，預設 6
            config (GripperConfig): 夾爪運行參數，若無則使用預設值
            logger (Logger): 日誌記錄器，若無則建立新的
        
        異常:
            ConnectionError: 當無法連接到夾爪時拋出
        """
        
        # 基本參數設定
        self.port = port
        self.baudrate = baudrate
        self.unit_id = unit_id
        self.config = config or GripperConfig()
        
        # 日誌設定
        self.logger = logger or self._create_logger()
        
        # 狀態管理
        self.status = GripperStatus.OFFLINE
        self.mode = GripperMode.MANUAL
        self.state = GripperState()
        self.is_connected = False
        
        # 執行緒控制
        self._monitor_thread = None
        self._stop_monitoring = threading.Event()
        self._state_lock = threading.Lock()
        
        # 回調函式
        self._status_change_callback = None
        self._error_callback = None
        
        # 建立Modbus客戶端
        self.modbus_client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            stopbits=stopbits,
            parity=parity,
            timeout=2.0
        )
        
        # 建立連線
        self._establish_connection()
        
    def _create_logger(self) -> logging.Logger:
        """建立專用日誌記錄器"""
        logger = logging.getLogger(f'gripper_controller_{self.unit_id}')
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(levelname)s - Gripper API: %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
            logger.setLevel(logging.INFO)
        return logger
        
    def _establish_connection(self) -> None:
        """建立與夾爪的通訊連線"""
        try:
            if self.modbus_client.connect():
                self.is_connected = True
                self.status = GripperStatus.IDLE
                self.logger.info(f"Connected to gripper - Port: {self.port}, Unit ID: {self.unit_id}")
                
                # 啟動狀態監控
                self._start_monitoring()
                
                # 讀取初始狀態
                self._update_state()
                
            else:
                raise ConnectionError(f"Failed to connect to gripper on port: {self.port}")
                
        except Exception as e:
            self.logger.error(f"Connection failed: {str(e)}")
            raise ConnectionError(f"Gripper connection failed: {str(e)}")
    
    def _write_register(self, address: int, value: int) -> bool:
        """
        寫入Modbus暫存器
        
        參數:
            address (int): 暫存器位址
            value (int): 要寫入的數值
            
        回傳:
            bool: 寫入是否成功
        """
        try:
            if not self.is_connected:
                self.logger.warning("Gripper not connected, cannot write register")
                return False
                
            result = self.modbus_client.write_register(
                address=address, 
                value=value, 
                slave=self.unit_id
            )
            
            if result.isError():
                self.logger.error(f"Write failed - Address: 0x{address:04X}, Value: {value}")
                return False
            else:
                self.logger.debug(f"Write successful - Address: 0x{address:04X}, Value: {value}")
                time.sleep(self.config.safety_interval)  # 安全間隔
                return True
                
        except Exception as e:
            self.logger.error(f"Register write exception: {str(e)}")
            return False
    
    def _read_register(self, address: int, count: int = 1) -> Optional[Union[int, list]]:
        """
        讀取Modbus暫存器
        
        參數:
            address (int): 起始暫存器位址
            count (int): 要讀取的暫存器數量
            
        回傳:
            int|list|None: 讀取的數值，失敗時回傳None
        """
        try:
            if not self.is_connected:
                return None
                
            result = self.modbus_client.read_holding_registers(
                address=address,
                count=count,
                slave=self.unit_id
            )
            
            if result.isError():
                self.logger.warning(f"Read failed - Address: 0x{address:04X}")
                return None
            else:
                if count == 1:
                    return result.registers[0]
                else:
                    return result.registers
                    
        except Exception as e:
            self.logger.error(f"Register read exception: {str(e)}")
            return None
    
    # ==================== 基本控制功能 ====================
    
    def initialize(self, mode: int = 0x01) -> bool:
        """
        初始化夾爪 (歸零校準)
        
        參數:
            mode (int): 初始化模式
                      0x01 = 回零校準 (預設)
                      0xA5 = 完全重置
        
        回傳:
            bool: 初始化是否成功
        """
        self.logger.info(f"Initializing gripper... (mode: 0x{mode:02X})")
        
        with self._state_lock:
            self.status = GripperStatus.INITIALIZING
            
        if self._write_register(0x0100, mode):
            # 等待初始化完成
            start_time = time.time()
            while time.time() - start_time < 10.0:  # 最多等待10秒
                time.sleep(0.5)
                if self._read_register(0x0201):  # 檢查狀態暫存器
                    break
                    
            with self._state_lock:
                self.status = GripperStatus.IDLE
                self.state.current_position = 0
                self.state.target_position = 0
                
            self.logger.info("Gripper initialization completed")
            return True
        else:
            with self._state_lock:
                self.status = GripperStatus.ERROR
            self.logger.error("Gripper initialization failed")
            return False
    
    def emergency_stop(self) -> bool:
        """
        緊急停止夾爪所有動作
        
        回傳:
            bool: 停止指令是否成功發送
        """
        self.logger.warning("Emergency stop executed!")
        
        success = self._write_register(0x0100, 0)
        
        if success:
            with self._state_lock:
                self.status = GripperStatus.IDLE
                self.state.is_moving = False
            self.logger.info("Emergency stop completed")
        else:
            self.logger.error("Emergency stop failed")
            
        return success
    
    def set_position(self, position: int, wait_completion: bool = False) -> bool:
        """
        設定夾爪位置
        
        參數:
            position (int): 目標位置 (0-1000千分比)
                           0 = 完全關閉
                           1000 = 完全開啟
            wait_completion (bool): 是否等待動作完成
            
        回傳:
            bool: 指令是否成功執行
        """
        # 參數驗證
        position = max(self.config.min_position, min(position, self.config.max_position))
        
        self.logger.info(f"Setting gripper position: {position}/1000")
        
        with self._state_lock:
            self.status = GripperStatus.MOVING
            self.state.target_position = position
            self.state.is_moving = True
        
        success = self._write_register(0x0103, position)
        
        if success and wait_completion:
            return self._wait_for_completion()
        
        return success
    
    def set_force(self, force: int) -> bool:
        """
        設定夾爪夾持力道
        
        參數:
            force (int): 夾持力道 (20-100百分比)
            
        回傳:
            bool: 設定是否成功
        """
        # 參數驗證
        force = max(self.config.min_force, min(force, self.config.max_force))
        
        self.logger.info(f"Setting gripper force: {force}%")
        
        success = self._write_register(0x0101, force)
        
        if success:
            with self._state_lock:
                self.state.current_force = force
                
        return success
    
    def set_speed(self, speed: int) -> bool:
        """
        設定夾爪移動速度
        
        參數:
            speed (int): 移動速度 (1-100百分比)
            
        回傳:
            bool: 設定是否成功
        """
        # 參數驗證
        speed = max(1, min(speed, 100))
        
        self.logger.info(f"Setting gripper speed: {speed}%")
        
        success = self._write_register(0x0104, speed)
        
        if success:
            with self._state_lock:
                self.state.current_speed = speed
                
        return success
    

    
    # ==================== 狀態監控功能 ====================
    
    def get_current_position(self) -> Optional[int]:
        """
        取得夾爪當前位置
        
        回傳:
            int|None: 當前位置 (0-1000)，失敗時回傳None
        """
        position = self._read_register(0x0202)
        if position is not None:
            with self._state_lock:
                self.state.current_position = position
        return position
    
    def get_gripper_state(self) -> GripperState:
        """
        取得完整的夾爪狀態資訊
        
        回傳:
            GripperState: 當前狀態資訊副本
        """
        with self._state_lock:
            return GripperState(
                current_position=self.state.current_position,
                target_position=self.state.target_position,
                current_force=self.state.current_force,
                current_speed=self.state.current_speed,
                is_moving=self.state.is_moving,
                is_gripping=self.state.is_gripping,
                temperature=self.state.temperature,
                current=self.state.current,
                error_code=self.state.error_code,
                uptime=self.state.uptime,
                last_update=time.time()
            )
    
    def is_ready(self) -> bool:
        """
        檢查夾爪是否準備好接受新指令
        
        回傳:
            bool: 是否準備就緒
        """
        return (self.is_connected and 
                self.status in [GripperStatus.IDLE, GripperStatus.GRIPPING] and
                not self.state.is_moving)
    
    def is_moving(self) -> bool:
        """
        檢查夾爪是否正在移動
        
        回傳:
            bool: 是否正在移動
        """
        return self.state.is_moving
    
    def get_connection_status(self) -> Dict[str, Any]:
        """
        取得連線狀態資訊
        
        回傳:
            Dict: 連線狀態詳細資訊
        """
        return {
            "connected": self.is_connected,
            "port": self.port,
            "baudrate": self.baudrate,
            "unit_id": self.unit_id,
            "status": self.status.value,
            "mode": self.mode.value,
            "last_communication": self.state.last_update
        }
    
    # ==================== 安全與監控功能 ====================
    
    def set_status_change_callback(self, callback: Callable[[GripperStatus, GripperState], None]) -> None:
        """
        設定狀態變更回調函式
        
        參數:
            callback: 當狀態變更時呼叫的函式 (新狀態, 狀態資訊)
        """
        self._status_change_callback = callback
        self.logger.info("Status change callback set")
    
    def set_error_callback(self, callback: Callable[[int, str], None]) -> None:
        """
        設定錯誤回調函式
        
        參數:
            callback: 當發生錯誤時呼叫的函式 (錯誤代碼, 錯誤訊息)
        """
        self._error_callback = callback
        self.logger.info("Error callback set")
    
    def check_safety_status(self) -> Tuple[bool, list]:
        """
        檢查夾爪安全狀態
        
        回傳:
            Tuple[bool, list]: (是否安全, 警告訊息列表)
        """
        warnings = []
        
        # 檢查連線狀態
        if not self.is_connected:
            warnings.append("Gripper not connected")
        
        # 檢查位置範圍
        current_pos = self.get_current_position()
        if current_pos is not None:
            if current_pos < self.config.min_position or current_pos > self.config.max_position:
                warnings.append(f"Position out of safe range: {current_pos}")
        
        # 檢查錯誤代碼
        if self.state.error_code != 0:
            warnings.append(f"System error code: {self.state.error_code}")
        
        # 檢查溫度 (如果支援)
        if self.state.temperature > 70.0:  # 假設70度為警告溫度
            warnings.append(f"High temperature: {self.state.temperature:.1f}°C")
        
        return len(warnings) == 0, warnings
    
    def run_self_diagnostic(self) -> Dict[str, Any]:
        """
        執行夾爪自我診斷
        
        回傳:
            Dict: 診斷結果
        """
        self.logger.info("Starting gripper self-diagnostic...")
        
        diagnostic_result = {
            "timestamp": time.time(),
            "communication_test": False,
            "movement_test": False,
            "force_test": False,
            "position_accuracy": None,
            "issues": []
        }
        
        try:
            # 通訊測試
            position = self.get_current_position()
            if position is not None:
                diagnostic_result["communication_test"] = True
                self.logger.info("Communication test passed")
            else:
                diagnostic_result["issues"].append("Communication test failed")
            
            # 移動測試 (小範圍移動)
            if diagnostic_result["communication_test"]:
                original_pos = position
                test_pos = original_pos + 50 if original_pos < 950 else original_pos - 50
                
                if self.set_position(test_pos, wait_completion=True):
                    new_pos = self.get_current_position()
                    if new_pos is not None and abs(new_pos - test_pos) < 10:
                        diagnostic_result["movement_test"] = True
                        diagnostic_result["position_accuracy"] = abs(new_pos - test_pos)
                        self.logger.info("Movement test passed")
                        
                        # 恢復原始位置
                        self.set_position(original_pos, wait_completion=True)
                    else:
                        diagnostic_result["issues"].append("Position accuracy insufficient")
                else:
                    diagnostic_result["issues"].append("Movement command failed")
            
            # 力道測試
            if self.set_force(30) and self.set_force(self.config.default_force):
                diagnostic_result["force_test"] = True
                self.logger.info("Force test passed")
            else:
                diagnostic_result["issues"].append("Force setting failed")
                
        except Exception as e:
            diagnostic_result["issues"].append(f"Diagnostic exception: {str(e)}")
            self.logger.error(f"Self-diagnostic exception: {str(e)}")
        
        # 總結
        passed_tests = sum([diagnostic_result["communication_test"], 
                          diagnostic_result["movement_test"], 
                          diagnostic_result["force_test"]])
        diagnostic_result["score"] = f"{passed_tests}/3"
        diagnostic_result["overall_status"] = "PASS" if passed_tests == 3 else "FAIL"
        
        self.logger.info(f"Self-diagnostic completed - Score: {diagnostic_result['score']}")
        
        return diagnostic_result
    
    # ==================== 私有輔助方法 ====================
    
    def _wait_for_completion(self, timeout: Optional[float] = None) -> bool:
        """等待夾爪移動完成"""
        timeout = timeout or self.config.timeout_seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            current_pos = self.get_current_position()
            if current_pos is not None:
                with self._state_lock:
                    if abs(current_pos - self.state.target_position) < 5:  # 允許5單位誤差
                        self.state.is_moving = False
                        self.status = GripperStatus.IDLE
                        return True
            time.sleep(0.1)
        
        self.logger.warning("Movement timeout")
        with self._state_lock:
            self.state.is_moving = False
            self.status = GripperStatus.ERROR
        return False
    
    def _check_grip_success(self) -> bool:
        """檢查是否成功夾持物件 (可擴展加入感測器回饋)"""
        # 基本檢查：位置是否在預期範圍內
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
            
        # 如果夾爪停在目標位置附近，假設夾持成功
        # 實際應用中可以加入力回饋感測器或其他感測器
        return abs(current_pos - self.state.target_position) < 20
    
    def _start_monitoring(self) -> None:
        """啟動狀態監控執行緒"""
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._stop_monitoring.clear()
            self._monitor_thread = threading.Thread(
                target=self._monitoring_loop,
                daemon=True,
                name=f"gripper_monitor_{self.unit_id}"
            )
            self._monitor_thread.start()
            self.logger.info("Status monitoring thread started")
    
    def _monitoring_loop(self) -> None:
        """狀態監控主迴圈"""
        while not self._stop_monitoring.is_set():
            try:
                self._update_state()
                time.sleep(0.5)  # 每0.5秒更新一次
            except Exception as e:
                self.logger.error(f"Monitoring loop error: {str(e)}")
                time.sleep(1.0)
    
    def _update_state(self) -> None:
        """更新夾爪狀態資訊"""
        try:
            # 更新位置
            current_pos = self.get_current_position()
            
            # 更新其他狀態 (根據實際硬體支援情況調整)
            # 這裡可以添加更多狀態讀取邏輯
            
            with self._state_lock:
                self.state.last_update = time.time()
                
                # 檢查是否還在移動
                if self.state.is_moving and current_pos is not None:
                    if abs(current_pos - self.state.target_position) < 5:
                        self.state.is_moving = False
                        if self.status == GripperStatus.MOVING:
                            self.status = GripperStatus.IDLE
                            
                # 調用狀態變更回調
                if self._status_change_callback:
                    try:
                        self._status_change_callback(self.status, self.state)
                    except Exception as e:
                        self.logger.error(f"Status callback error: {str(e)}")
                        
        except Exception as e:
            self.logger.error(f"State update error: {str(e)}")
            with self._state_lock:
                self.status = GripperStatus.ERROR
    
    def disconnect(self) -> None:
        """斷開夾爪連線"""
        self.logger.info("Disconnecting gripper...")
        
        # 停止監控
        self._stop_monitoring.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)
        
        # 關閉連線
        if self.modbus_client:
            self.modbus_client.close()
            
        self.is_connected = False
        self.status = GripperStatus.OFFLINE
        self.logger.info("Gripper disconnected")
    
    def __enter__(self):
        """上下文管理器進入"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()


# ==================== 工廠函式 ====================

def create_gripper_controller(port: str = 'COM3', 
                             unit_id: int = 6,
                             **kwargs) -> GripperController:
    """
    工廠函式：建立夾爪控制器實例
    
    參數:
        port (str): 串口埠號
        unit_id (int): Modbus從站號
        **kwargs: 其他初始化參數
        
    回傳:
        GripperController: 夾爪控制器實例
    """
    return GripperController(port=port, unit_id=unit_id, **kwargs)


# ==================== 使用範例 ====================

def main():
    """使用範例"""
    # 建立夾爪控制器
    with create_gripper_controller(port='COM4', unit_id=6) as gripper:
        
        # 初始化夾爪
        gripper.initialize()
        
        # 設定基本參數
        gripper.set_force(50)
        gripper.set_speed(40)
        
        # 基本動作測試
        gripper.set_position(400, wait_completion=True)
        gripper.set_position(500, wait_completion=True)
        gripper.set_position(0, wait_completion=True)
        
        # 自我診斷
        diagnostic = gripper.run_self_diagnostic()
        print(f"Diagnostic result: {diagnostic}")


if __name__ == "__main__":
    main()