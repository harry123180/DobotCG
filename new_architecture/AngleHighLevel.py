import time
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

# PyModbus imports
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# 設置logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AngleOperationResult(Enum):
    """角度操作結果枚舉"""
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"
    NOT_READY = "NOT_READY"
    CONNECTION_ERROR = "CONNECTION_ERROR"
    SYSTEM_ERROR = "SYSTEM_ERROR"

@dataclass
class AngleCorrectionResult:
    """角度校正結果數據類"""
    result: AngleOperationResult
    message: str
    original_angle: Optional[float] = None
    angle_diff: Optional[float] = None  
    motor_position: Optional[int] = None
    execution_time: Optional[float] = None
    error_details: Optional[str] = None

class AngleHighLevel:
    """角度調整系統高階API
    
    提供簡潔的方法供Flow流程調用，隱藏底層Modbus通訊細節
    專注於執行90度角度校正功能
    """
    
    def __init__(self, host: str = "127.0.0.1", port: int = 502):
        """初始化角度調整高階API
        
        Args:
            host: Modbus服務器IP
            port: Modbus服務器端口
        """
        self.host = host
        self.port = port
        self.base_address = 700
        self.modbus_client = None
        self.timeout = 3.0
        
        # 操作超時設定
        self.correction_timeout = 15.0  # 角度校正總超時15秒
        self.status_check_interval = 0.5  # 狀態檢查間隔500ms
        
        logger.info(f"AngleHighLevel初始化: {host}:{port}, 基地址:{self.base_address}")
    
    def connect(self) -> bool:
        """連接到角度調整模組
        
        Returns:
            bool: 連接成功返回True
        """
        try:
            logger.info("正在連接角度調整模組...")
            
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.host,
                port=self.port,
                timeout=self.timeout
            )
            
            if self.modbus_client.connect():
                # 驗證模組回應
                status = self._read_system_status()
                if status:
                    logger.info(f"角度調整模組連接成功 - Ready:{status.get('ready')}, Initialized:{status.get('initialized')}")
                    return True
                else:
                    logger.error("角度調整模組無回應")
                    return False
            else:
                logger.error(f"無法連接到角度調整模組: {self.host}:{self.port}")
                return False
                
        except Exception as e:
            logger.error(f"連接角度調整模組失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client:
            self.modbus_client.close()
            self.modbus_client = None
            logger.info("角度調整模組連接已斷開")
    
    def is_system_ready(self) -> bool:
        """檢查系統是否準備就緒
        
        Returns:
            bool: 系統Ready且無Alarm時返回True
        """
        status = self._read_system_status()
        if not status:
            return False
        
        ready = status.get('ready', False)
        alarm = status.get('alarm', False)
        initialized = status.get('initialized', False)
        
        logger.debug(f"系統狀態檢查: Ready={ready}, Alarm={alarm}, Initialized={initialized}")
        
        return ready and not alarm and initialized
    
    def adjust_to_90_degrees(self) -> AngleCorrectionResult:
        """執行角度校正到90度
        
        這是主要的公開方法，供Flow流程調用
        執行完整的CCD3檢測 → 角度計算 → 馬達移動流程
        
        Returns:
            AngleCorrectionResult: 包含執行結果的完整資訊
        """
        start_time = time.time()
        
        try:
            logger.info("=== 開始執行角度校正到90度 ===")
            
            # 步驟1: 檢查連接狀態
            if not self.modbus_client or not self.modbus_client.connected:
                return AngleCorrectionResult(
                    result=AngleOperationResult.CONNECTION_ERROR,
                    message="Modbus連接未建立，請先調用connect()"
                )
            
            # 步驟2: 檢查系統狀態
            if not self.is_system_ready():
                return AngleCorrectionResult(
                    result=AngleOperationResult.NOT_READY,
                    message="角度調整系統未準備就緒，請檢查系統狀態"
                )
            
            # 步驟3: 發送角度校正指令
            logger.info("發送角度校正指令...")
            if not self._send_angle_correction_command():
                return AngleCorrectionResult(
                    result=AngleOperationResult.FAILED,
                    message="發送角度校正指令失敗"
                )
            
            # 步驟4: 等待執行完成
            logger.info("等待角度校正執行完成...")
            execution_result = self._wait_for_completion()
            
            if execution_result.result != AngleOperationResult.SUCCESS:
                return execution_result
            
            # 步驟5: 讀取執行結果
            result_data = self._read_correction_results()
            execution_time = time.time() - start_time
            
            if result_data and result_data.get('success', False):
                logger.info(f"角度校正成功完成，耗時: {execution_time:.2f}秒")
                logger.info(f"檢測角度: {result_data.get('original_angle'):.2f}度")
                logger.info(f"角度差: {result_data.get('angle_diff'):.2f}度")
                logger.info(f"馬達位置: {result_data.get('motor_position')}")
                
                return AngleCorrectionResult(
                    result=AngleOperationResult.SUCCESS,
                    message="角度校正完成",
                    original_angle=result_data.get('original_angle'),
                    angle_diff=result_data.get('angle_diff'),
                    motor_position=result_data.get('motor_position'),
                    execution_time=execution_time
                )
            else:
                return AngleCorrectionResult(
                    result=AngleOperationResult.FAILED,
                    message="角度校正執行失敗，無有效結果",
                    execution_time=execution_time
                )
            
        except Exception as e:
            execution_time = time.time() - start_time
            logger.error(f"角度校正過程發生異常: {e}")
            return AngleCorrectionResult(
                result=AngleOperationResult.SYSTEM_ERROR,
                message="角度校正系統異常",
                execution_time=execution_time,
                error_details=str(e)
            )
    
    def reset_motor(self) -> AngleOperationResult:
        """馬達重置
        
        Returns:
            AngleOperationResult: 重置結果
        """
        try:
            logger.info("執行馬達重置...")
            
            if not self.is_system_ready():
                return AngleOperationResult.NOT_READY
            
            # 發送馬達重置指令
            result = self.modbus_client.write_register(
                address=self.base_address + 40, value=2, slave=1
            )
            
            if result.isError():
                logger.error("馬達重置指令發送失敗")
                return AngleOperationResult.FAILED
            
            # 等待指令處理
            time.sleep(1.0)
            
            # 清除指令
            self.modbus_client.write_register(
                address=self.base_address + 40, value=0, slave=1
            )
            
            logger.info("馬達重置完成")
            return AngleOperationResult.SUCCESS
            
        except Exception as e:
            logger.error(f"馬達重置異常: {e}")
            return AngleOperationResult.SYSTEM_ERROR
    
    def reset_errors(self) -> AngleOperationResult:
        """錯誤重置
        
        Returns:
            AngleOperationResult: 重置結果
        """
        try:
            logger.info("執行錯誤重置...")
            
            # 發送錯誤重置指令
            result = self.modbus_client.write_register(
                address=self.base_address + 40, value=7, slave=1
            )
            
            if result.isError():
                logger.error("錯誤重置指令發送失敗")
                return AngleOperationResult.FAILED
            
            # 等待指令處理
            time.sleep(1.0)
            
            # 清除指令
            self.modbus_client.write_register(
                address=self.base_address + 40, value=0, slave=1
            )
            
            logger.info("錯誤重置完成")
            return AngleOperationResult.SUCCESS
            
        except Exception as e:
            logger.error(f"錯誤重置異常: {e}")
            return AngleOperationResult.SYSTEM_ERROR
    
    def get_system_status(self) -> Optional[Dict[str, Any]]:
        """獲取系統狀態資訊
        
        Returns:
            Dict: 系統狀態字典，包含詳細狀態資訊
        """
        return self._read_system_status()
    
    def get_last_result(self) -> Optional[Dict[str, Any]]:
        """獲取最後一次角度校正結果
        
        Returns:
            Dict: 校正結果字典
        """
        return self._read_correction_results()
    
    def _send_angle_correction_command(self) -> bool:
        """發送角度校正指令 (私有方法)"""
        try:
            result = self.modbus_client.write_register(
                address=self.base_address + 40, value=1, slave=1
            )
            return not result.isError()
        except Exception as e:
            logger.error(f"發送角度校正指令異常: {e}")
            return False
    
    def _wait_for_completion(self) -> AngleCorrectionResult:
        """等待角度校正完成 (私有方法)"""
        start_time = time.time()
        
        while time.time() - start_time < self.correction_timeout:
            try:
                status = self._read_system_status()
                if not status:
                    time.sleep(self.status_check_interval)
                    continue
                
                ready = status.get('ready', False)
                running = status.get('running', False)
                alarm = status.get('alarm', False)
                
                logger.debug(f"執行狀態: Ready={ready}, Running={running}, Alarm={alarm}")
                
                # 檢查是否有錯誤
                if alarm:
                    return AngleCorrectionResult(
                        result=AngleOperationResult.FAILED,
                        message="角度校正過程發生錯誤，系統進入Alarm狀態"
                    )
                
                # 檢查是否完成 (Ready=True且Running=False)
                if ready and not running:
                    logger.info("角度校正執行完成")
                    return AngleCorrectionResult(
                        result=AngleOperationResult.SUCCESS,
                        message="角度校正執行完成"
                    )
                
                time.sleep(self.status_check_interval)
                
            except Exception as e:
                logger.error(f"狀態檢查異常: {e}")
                time.sleep(self.status_check_interval)
        
        logger.error(f"角度校正執行超時 ({self.correction_timeout}秒)")
        return AngleCorrectionResult(
            result=AngleOperationResult.TIMEOUT,
            message=f"角度校正執行超時 ({self.correction_timeout}秒)"
        )
    
    def _read_system_status(self) -> Optional[Dict[str, Any]]:
        """讀取系統狀態 (私有方法)"""
        try:
            result = self.modbus_client.read_holding_registers(
                address=self.base_address, count=15, slave=1
            )
            
            if result.isError():
                return None
            
            registers = result.registers
            status_register = registers[0]
            
            return {
                'status_register': status_register,
                'ready': bool(status_register & (1 << 0)),
                'running': bool(status_register & (1 << 1)),
                'alarm': bool(status_register & (1 << 2)),
                'initialized': bool(status_register & (1 << 3)),
                'ccd_detecting': bool(status_register & (1 << 4)),
                'motor_moving': bool(status_register & (1 << 5)),
                'modbus_connected': bool(registers[1]),
                'motor_connected': bool(registers[2]),
                'error_code': registers[3],
                'operation_count': (registers[5] << 16) | registers[4],
                'error_count': registers[6]
            }
            
        except Exception as e:
            logger.error(f"讀取系統狀態異常: {e}")
            return None
    
    def _read_correction_results(self) -> Optional[Dict[str, Any]]:
        """讀取角度校正結果 (私有方法)"""
        try:
            result = self.modbus_client.read_holding_registers(
                address=self.base_address + 20, count=20, slave=1
            )
            
            if result.isError():
                return None
            
            registers = result.registers
            success = bool(registers[0])
            
            if not success:
                return {'success': False}
            
            # 原始角度 (32位，保留2位小數)
            angle_int = (registers[1] << 16) | registers[2]
            if angle_int >= 2**31:
                angle_int -= 2**32
            original_angle = angle_int / 100.0
            
            # 角度差 (32位，保留2位小數)
            diff_int = (registers[3] << 16) | registers[4]
            if diff_int >= 2**31:
                diff_int -= 2**32
            angle_diff = diff_int / 100.0
            
            # 馬達位置 (32位)
            pos_int = (registers[5] << 16) | registers[6]
            if pos_int >= 2**31:
                pos_int -= 2**32
            motor_position = pos_int
            
            return {
                'success': True,
                'original_angle': original_angle,
                'angle_diff': angle_diff,
                'motor_position': motor_position,
                'operation_count': (registers[11] << 16) | registers[10],
                'error_count': registers[12],
                'runtime': registers[13]
            }
            
        except Exception as e:
            logger.error(f"讀取校正結果異常: {e}")
            return None

# 便利函數，供快速調用
def correct_angle_to_90_degrees(host: str = "127.0.0.1", port: int = 502) -> AngleCorrectionResult:
    """便利函數：一鍵執行角度校正到90度
    
    自動處理連接/斷開，適合簡單的一次性調用
    
    Args:
        host: Modbus服務器IP
        port: Modbus服務器端口
        
    Returns:
        AngleCorrectionResult: 校正結果
    """
    angle_controller = AngleHighLevel(host, port)
    
    if not angle_controller.connect():
        return AngleCorrectionResult(
            result=AngleOperationResult.CONNECTION_ERROR,
            message="無法連接到角度調整模組"
        )
    
    try:
        result = angle_controller.adjust_to_90_degrees()
        return result
    finally:
        angle_controller.disconnect()

# 使用範例
if __name__ == '__main__':
    # 範例1: 使用便利函數 (一次性調用)
    print("=== 範例1: 便利函數調用 ===")
    result = correct_angle_to_90_degrees()
    print(f"結果: {result.result.value}")
    print(f"訊息: {result.message}")
    if result.original_angle:
        print(f"檢測角度: {result.original_angle:.2f}度")
        print(f"角度差: {result.angle_diff:.2f}度")
    
    print("\n" + "="*50 + "\n")
    
    # 範例2: 使用類別實例 (持續性操作)
    print("=== 範例2: 類別實例調用 ===")
    angle_api = AngleHighLevel()
    
    if angle_api.connect():
        print("✓ 連接成功")
        
        # 檢查系統狀態
        if angle_api.is_system_ready():
            print("✓ 系統準備就緒")
            
            # 執行角度校正
            correction_result = angle_api.adjust_to_90_degrees()
            print(f"校正結果: {correction_result.result.value}")
            print(f"訊息: {correction_result.message}")
            
            if correction_result.result == AngleOperationResult.SUCCESS:
                print(f"執行時間: {correction_result.execution_time:.2f}秒")
                print(f"檢測角度: {correction_result.original_angle:.2f}度")
                print(f"角度差: {correction_result.angle_diff:.2f}度")
                print(f"馬達位置: {correction_result.motor_position}")
        else:
            print("✗ 系統未準備就緒")
            
            # 嘗試重置錯誤
            reset_result = angle_api.reset_errors()
            if reset_result == AngleOperationResult.SUCCESS:
                print("✓ 錯誤重置成功")
        
        angle_api.disconnect()
    else:
        print("✗ 連接失敗")