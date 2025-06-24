import time
import threading
import json
import os
import logging
from typing import Dict, Any, Optional
from enum import Enum
import serial
import struct
from dataclasses import dataclass

# PyModbus imports
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# 設置logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StatusBits(Enum):
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3
    CCD_DETECTING = 4
    MOTOR_MOVING = 5

class ControlCommand(Enum):
    CLEAR = 0
    ANGLE_CORRECTION = 1  # 角度校正指令
    MOTOR_RESET = 2       # 馬達重置
    ERROR_RESET = 7       # 錯誤重置

@dataclass
class AngleResult:
    success: bool
    original_angle: Optional[float]  # CCD3返回的原始角度
    angle_diff: Optional[float]      # 與90度的角度差
    motor_position: Optional[int]    # 計算出的馬達位置
    error_message: Optional[str] = None

class ModbusRTU:
    """馬達驅動器RTU通訊類 - 完全參考paste.txt實現"""
    def __init__(self):
        self.serial_conn = None
        
    def connect(self, port, baudrate=115200):
        try:
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1
            )
            print(f"馬達驅動器RTU連接成功: {port}, 波特率: {baudrate}")
            return True
        except Exception as e:
            print(f"馬達驅動器RTU連接失敗: {e}")
            return False
    
    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("馬達驅動器RTU連接已關閉")
    
    def crc16(self, data):
        """計算CRC-16校驗 - 完全參考paste.txt"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def write_single_register(self, slave_id, address, value):
        """寫入單個保持寄存器 - 完全參考paste.txt"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
            
        frame = bytearray()
        frame.append(slave_id)           # 從站地址
        frame.append(0x06)               # 功能碼
        frame.extend(address.to_bytes(2, 'big'))    # 寄存器地址
        frame.extend(value.to_bytes(2, 'big'))      # 寄存器值
        
        # 計算並添加CRC
        crc = self.crc16(frame)
        frame.extend(crc.to_bytes(2, 'little'))
        
        try:
            self.serial_conn.write(frame)
            response = self.serial_conn.read(8)  # 預期響應長度
            success = len(response) >= 8
            if success:
                print(f"寫入寄存器成功: 地址{address}, 值{value}")
            else:
                print(f"寫入寄存器失敗: 地址{address}, 值{value}")
            return success
        except Exception as e:
            print(f"寫入寄存器異常: {e}")
            return False
    
    def read_holding_registers(self, slave_id, start_addr, count):
        """讀取保持寄存器 - 完全參考paste.txt"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
            
        frame = bytearray()
        frame.append(slave_id)           # 從站地址
        frame.append(0x03)               # 功能碼
        frame.extend(start_addr.to_bytes(2, 'big'))  # 起始地址
        frame.extend(count.to_bytes(2, 'big'))       # 寄存器數量
        
        # 計算並添加CRC
        crc = self.crc16(frame)
        frame.extend(crc.to_bytes(2, 'little'))
        
        try:
            self.serial_conn.write(frame)
            response = self.serial_conn.read(5 + count * 2)  # 響應長度
            
            if len(response) >= 5 and response[1] == 0x03:
                # 解析數據
                byte_count = response[2]
                data = response[3:3+byte_count]
                values = []
                for i in range(0, byte_count, 2):
                    value = struct.unpack('>H', data[i:i+2])[0]
                    values.append(value)
                return values
            return None
        except Exception as e:
            print(f"讀取寄存器異常: {e}")
            return None

class AngleSystemStateMachine:
    """角度調整系統狀態機"""
    def __init__(self):
        self.status_register = 0b0001  # 初始狀態: Ready=1
        self.lock = threading.Lock()
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        with self.lock:
            if value:
                self.status_register |= (1 << bit_pos.value)
            else:
                self.status_register &= ~(1 << bit_pos.value)
    
    def get_bit(self, bit_pos: StatusBits) -> bool:
        with self.lock:
            return bool(self.status_register & (1 << bit_pos.value))
    
    def is_ready(self) -> bool:
        return self.get_bit(StatusBits.READY)
    
    def is_running(self) -> bool:
        return self.get_bit(StatusBits.RUNNING)
    
    def is_alarm(self) -> bool:
        return self.get_bit(StatusBits.ALARM)
    
    def is_initialized(self) -> bool:
        return self.get_bit(StatusBits.INITIALIZED)
    
    def is_ccd_detecting(self) -> bool:
        return self.get_bit(StatusBits.CCD_DETECTING)
    
    def is_motor_moving(self) -> bool:
        return self.get_bit(StatusBits.MOTOR_MOVING)
    
    def set_ready(self, ready: bool):
        self.set_bit(StatusBits.READY, ready)
    
    def set_running(self, running: bool):
        self.set_bit(StatusBits.RUNNING, running)
    
    def set_alarm(self, alarm: bool):
        self.set_bit(StatusBits.ALARM, alarm)
    
    def set_initialized(self, initialized: bool):
        self.set_bit(StatusBits.INITIALIZED, initialized)
    
    def set_ccd_detecting(self, detecting: bool):
        self.set_bit(StatusBits.CCD_DETECTING, detecting)
    
    def set_motor_moving(self, moving: bool):
        self.set_bit(StatusBits.MOTOR_MOVING, moving)
    
    def reset_to_idle(self):
        with self.lock:
            self.status_register = 0b1001  # Ready=1, Initialized=1

class AngleAdjustmentService:
    """角度調整主服務"""
    def __init__(self):
        # 基本配置
        self.base_address = 700  # 角度調整模組基地址
        self.ccd3_base_address = 800  # CCD3模組基地址
        
        # Modbus TCP 連接 (主服務器)
        self.modbus_client = None
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        
        # 馬達驅動器RTU連接
        self.motor_rtu = ModbusRTU()
        self.motor_slave_id = 3
        
        # 狀態機
        self.state_machine = AngleSystemStateMachine()
        
        # 控制變量
        self.last_control_command = 0
        self.command_processing = False
        self.handshake_thread = None
        self.stop_handshake = False
        
        # 統計資訊
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        
        # 配置檔案
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'angle_config.json')
        self.load_config()
    
    def load_config(self):
        """載入配置檔案"""
        default_config = {
            "module_id": "Angle_Adjustment_System",
            "modbus_tcp": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 3.0
            },
            "motor_rtu": {
                "port": "COM5",
                "baudrate": 115200,
                "parity": "N",
                "stopbits": 1,
                "bytesize": 8,
                "timeout": 1.0,
                "slave_id": 3
            },
            "modbus_mapping": {
                "base_address": 700,
                "ccd3_base_address": 800
            },
            "angle_calculation": {
                "target_angle": 90.0,
                "motor_base_position": 9000,
                "angle_multiplier": 10
            },
            "timing": {
                "handshake_interval": 0.05,
                "motor_settle_delay": 0.1,
                "ccd_timeout": 10.0,
                "motor_timeout": 30.0
            }
        }
        
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                print(f"配置檔案載入成功: {self.config_file}")
            else:
                config = default_config
                with open(self.config_file, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                print(f"配置檔案已創建: {self.config_file}")
            
            # 應用配置
            self.server_ip = config['modbus_tcp']['host']
            self.server_port = config['modbus_tcp']['port']
            self.base_address = config['modbus_mapping']['base_address']
            self.ccd3_base_address = config['modbus_mapping']['ccd3_base_address']
            self.motor_slave_id = config['motor_rtu']['slave_id']
            
        except Exception as e:
            print(f"配置檔案載入錯誤: {e}")
    
    def connect_modbus(self) -> bool:
        """連接Modbus TCP主服務器"""
        try:
            print("正在連接Modbus TCP主服務器...")
            
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3
            )
            
            if self.modbus_client.connect():
                self.connection_count += 1
                print(f"角度調整模組已連接到Modbus服務器: {self.server_ip}:{self.server_port}")
                return True
            else:
                print(f"Modbus連接失敗: 無法連接到 {self.server_ip}:{self.server_port}")
                self.state_machine.set_alarm(True)
                return False
                
        except Exception as e:
            print(f"Modbus連接錯誤: {e}")
            self.state_machine.set_alarm(True)
            return False
    
    def send_motor_init_params(self) -> bool:
        """發送馬達初始化參數 - 參考GUI工具的設定"""
        try:
            print("發送馬達初始化參數...")
            
            # 按照GUI工具的參數設置 (寄存器6144-6155)
            init_values = [
                0,      # 6144: 0
                2,      # 6145: 2  
                0,      # 6146: 0
                9000,   # 6147: 9000 (目標位置)
                0,      # 6148: 0
                0,      # 6149: 0 (運轉速度，修正為0)
                15,     # 6150: 15
                16960,  # 6151: 16960
                15,     # 6152: 15
                16960,  # 6153: 16960
                0,      # 6154: 0
                1000    # 6155: 1000
            ]
            
            # 使用write_multiple_registers批次寫入
            success = self.write_multiple_registers(6144, init_values)
            
            if success:
                print("馬達初始化參數發送成功")
                return True
            else:
                print("馬達初始化參數發送失敗")
                return False
                
        except Exception as e:
            print(f"發送馬達初始化參數錯誤: {e}")
            return False
    
    def write_multiple_registers(self, start_addr: int, values: list) -> bool:
        """寫入多個保持寄存器 - 參考GUI工具實現"""
        try:
            if not self.motor_rtu.serial_conn or not self.motor_rtu.serial_conn.is_open:
                return False
                
            # 構建請求幀
            register_count = len(values)
            byte_count = register_count * 2
            
            frame = bytearray()
            frame.append(self.motor_slave_id)        # 從站地址
            frame.append(0x10)                       # 功能碼 (寫入多個寄存器)
            frame.extend(start_addr.to_bytes(2, 'big'))  # 起始地址
            frame.extend(register_count.to_bytes(2, 'big'))  # 寄存器數量
            frame.append(byte_count)                 # 字節數
            
            # 添加數據
            for value in values:
                frame.extend(value.to_bytes(2, 'big'))
            
            # 計算並添加CRC
            crc = self.motor_rtu.crc16(frame)
            frame.extend(crc.to_bytes(2, 'little'))
            
            # 發送請求
            self.motor_rtu.serial_conn.write(frame)
            response = self.motor_rtu.serial_conn.read(8)  # 預期響應長度
            
            success = len(response) >= 8
            if success:
                print(f"批次寫入寄存器成功: 起始地址{start_addr}, 數量{register_count}")
            else:
                print(f"批次寫入寄存器失敗: 起始地址{start_addr}, 數量{register_count}")
            
            return success
            
        except Exception as e:
            print(f"批次寫入寄存器異常: {e}")
            return False
    
    def connect_motor(self, port: str = "COM5", baudrate: int = 115200) -> bool:
        """連接馬達驅動器"""
        try:
            print(f"正在連接馬達驅動器: {port}, 波特率: {baudrate}")
            success = self.motor_rtu.connect(port, baudrate)
            
            if success:
                # 測試讀取馬達狀態
                test_result = self.motor_rtu.read_holding_registers(self.motor_slave_id, 127, 1)
                if test_result is not None:
                    print(f"馬達驅動器連接測試成功，狀態寄存器: {test_result[0]}")
                    
                    # 跳過初始化參數發送，使用預設參數
                    print("使用預先設定的馬達參數，跳過初始化")
                    
                    self.state_machine.set_initialized(True)
                    self.state_machine.set_alarm(False)
                    return True
                else:
                    print("馬達驅動器連接測試失敗")
                    self.state_machine.set_alarm(True)
                    return False
            else:
                print("馬達驅動器連接失敗")
                self.state_machine.set_alarm(True)
                return False
                
        except Exception as e:
            print(f"馬達驅動器連接錯誤: {e}")
            self.state_machine.set_alarm(True)
            return False
    
    def read_ccd3_angle(self) -> Optional[float]:
        """讀取CCD3檢測到的角度"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                print("Modbus未連接，無法讀取CCD3角度")
                return None
            
            # 修正：讀取CCD3檢測結果寄存器 (840-844)
            result = self.modbus_client.read_holding_registers(
                address=self.ccd3_base_address + 40, count=5, slave=1  # 修正為+40
            )
            
            if result.isError():
                print("讀取CCD3角度失敗")
                return None
            
            registers = result.registers
            success_flag = registers[0]  # 840: 檢測成功標誌
            center_x = registers[1]      # 841: 中心X座標
            center_y = registers[2]      # 842: 中心Y座標
            angle_high = registers[3]    # 843: 角度高位
            angle_low = registers[4]     # 844: 角度低位
            
            print(f"CCD3檢測結果: success={success_flag}, center=({center_x},{center_y}), angle_data=({angle_high},{angle_low})")
            
            if success_flag != 1:
                print("CCD3檢測未成功，無有效角度數據")
                return None
            
            # 合併32位角度數據
            angle_int = (angle_high << 16) | angle_low
            
            # 處理有符號數值
            if angle_int >= 2**31:
                angle_int -= 2**32
            
            angle_degrees = angle_int / 100.0  # 恢復2位小數精度
            
            print(f"CCD3角度讀取成功: {angle_degrees:.2f}度")
            return angle_degrees
            
        except Exception as e:
            print(f"讀取CCD3角度錯誤: {e}")
            return None
    
    def calculate_motor_position(self, ccd_angle: float) -> int:
        """計算馬達目標位置 - 按照需求公式計算"""
        try:
            # 角度計算邏輯: (9000 - CCD3角度*100)
            motor_position = 9000 - int(ccd_angle * 100)
            
            print(f"角度計算: CCD3角度={ccd_angle:.2f}度, 馬達位置={motor_position}")
            print(f"計算公式: 9000 - ({ccd_angle:.2f} × 100) = {motor_position}")
            
            return motor_position
            
        except Exception as e:
            print(f"角度計算錯誤: {e}")
            return 0
    
    def clear_motor_command(self) -> bool:
        """清除馬達指令寄存器 - 確保馬達回到Ready狀態"""
        try:
            print("清除馬達指令寄存器...")
            success = self.motor_rtu.write_single_register(self.motor_slave_id, 125, 0)
            if success:
                print("馬達指令寄存器已清除")
            else:
                print("清除馬達指令寄存器失敗")
            return success
        except Exception as e:
            print(f"清除馬達指令錯誤: {e}")
            return False
    
    def wait_motor_ready(self, timeout: float = 10.0, check_interval: float = 0.1) -> bool:
        """等待馬達準備就緒 - Ready=1, Alarm=0, Moving=0"""
        try:
            print(f"等待馬達準備就緒 (超時: {timeout}秒)...")
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                # 讀取馬達狀態
                status = self.motor_rtu.read_holding_registers(self.motor_slave_id, 127, 1)
                
                if status is None:
                    print("讀取馬達狀態失敗，重試...")
                    time.sleep(check_interval)
                    continue
                
                status_word = status[0]
                ready = bool(status_word & (1 << 5))    # bit 5: 準備就緒
                alarm = bool(status_word & (1 << 7))    # bit 7: 警報狀態
                moving = bool(status_word & (1 << 13))  # bit 13: 運動中
                
                elapsed = time.time() - start_time
                print(f"狀態檢查 ({elapsed:.1f}s): Ready={ready}, Alarm={alarm}, Moving={moving}, status_word={status_word}")
                
                # 如果出現警報，嘗試清除警報
                if alarm:
                    print("檢測到警報狀態，嘗試清除警報...")
                    alm_rst_success = self.motor_rtu.write_single_register(self.motor_slave_id, 125, 128)
                    if alm_rst_success:
                        print("清除警報指令已發送")
                        time.sleep(0.2)
                        # 清除指令寄存器
                        self.clear_motor_command()
                        time.sleep(0.3)
                        continue  # 重新檢查狀態
                    else:
                        print("清除警報指令發送失敗")
                        return False
                
                # 檢查是否滿足發送指令的條件: Ready=1, Alarm=0, Moving=0
                if ready and not alarm and not moving:
                    print("馬達已準備就緒，可以發送指令")
                    return True
                
                # 如果未Ready但沒有運動和警報，嘗試清除指令狀態
                if not ready and not alarm and not moving:
                    print("馬達未Ready但無運動和警報，嘗試清除指令狀態...")
                    if self.clear_motor_command():
                        time.sleep(0.2)  # 給馬達一點時間更新狀態
                        continue
                
                time.sleep(check_interval)
            
            print(f"等待馬達準備就緒超時 ({timeout}秒)")
            return False
            
        except Exception as e:
            print(f"等待馬達準備就緒錯誤: {e}")
            return False

    def send_motor_command(self, position: int) -> bool:
        """發送馬達移動指令 - 先等待Ready再發送"""
        try:
            print(f"準備發送馬達移動指令: 位置={position}")
            
            # 步驟1: 等待馬達準備就緒
            if not self.wait_motor_ready(timeout=10.0):
                print("馬達未準備就緒，無法發送移動指令")
                return False
            
            # 步驟2: 設置目標位置 (寄存器6147)
            success1 = self.motor_rtu.write_single_register(self.motor_slave_id, 6147, position)
            
            if not success1:
                print("設置馬達目標位置失敗")
                return False
            
            print(f"目標位置已設置: {position}")
            
            # 步驟3: 發送移動指令 (寄存器125, 值8)
            success2 = self.motor_rtu.write_single_register(self.motor_slave_id, 125, 8)
            
            if not success2:
                print("發送馬達移動指令失敗")
                return False
            
            print(f"馬達移動指令發送成功: 目標位置={position}")
            return True
            
        except Exception as e:
            print(f"發送馬達指令錯誤: {e}")
            return False
    
    def wait_motor_complete(self, timeout: float = 30.0) -> bool:
        """等待馬達運動完成 - 等待Moving=0且Ready=1，確保清除指令"""
        clear_success = False
        try:
            print("等待馬達運動完成...")
            start_time = time.time()
            
            # 等待運動完成：Moving=0
            while time.time() - start_time < timeout:
                status = self.motor_rtu.read_holding_registers(self.motor_slave_id, 127, 1)
                
                if status is None:
                    print("讀取馬達狀態失敗，重試...")
                    time.sleep(0.2)
                    continue
                
                status_word = status[0]
                moving = bool(status_word & (1 << 13))  # bit 13: 運動中
                ready = bool(status_word & (1 << 5))    # bit 5: 準備就緒
                alarm = bool(status_word & (1 << 7))    # bit 7: 警報狀態
                
                elapsed = time.time() - start_time
                print(f"運動狀態檢查 ({elapsed:.1f}s): Moving={moving}, Ready={ready}, Alarm={alarm}, status_word={status_word}")
                
                # 如果有警報，嘗試清除警報
                if alarm:
                    print("檢測到警報狀態，發送清除警報指令...")
                    alm_rst_success = self.motor_rtu.write_single_register(self.motor_slave_id, 125, 128)
                    
                    if alm_rst_success:
                        print("清除警報指令已發送")
                        time.sleep(0.2)
                        clear_success = self.clear_motor_command()
                        time.sleep(0.3)
                        continue
                    else:
                        print("發送清除警報指令失敗")
                        time.sleep(0.2)
                        continue
                
                # 檢查運動是否完成：Moving=0
                if not moving:
                    print("馬達運動已完成")
                    
                    # 等待Ready狀態穩定
                    stable_count = 0
                    for i in range(5):  # 檢查5次，每次200ms
                        time.sleep(0.2)
                        status_check = self.motor_rtu.read_holding_registers(self.motor_slave_id, 127, 1)
                        
                        if status_check:
                            status_word_check = status_check[0]
                            ready_check = bool(status_word_check & (1 << 5))
                            alarm_check = bool(status_word_check & (1 << 7))
                            moving_check = bool(status_word_check & (1 << 13))
                            
                            print(f"穩定性檢查 {i+1}/5: Moving={moving_check}, Ready={ready_check}, Alarm={alarm_check}")
                            
                            # 如果狀態穩定 (不運動且無警報)
                            if not moving_check and not alarm_check:
                                stable_count += 1
                            else:
                                stable_count = 0
                                break
                    
                    if stable_count >= 3:  # 至少3次穩定檢查
                        print("馬達狀態穩定，運動完成")
                        clear_success = self.clear_motor_command()
                        return True
                
                time.sleep(0.1)  # 100ms檢查間隔
            
            print(f"等待馬達運動完成超時 ({timeout}秒)")
            return False
            
        except Exception as e:
            print(f"等待馬達完成錯誤: {e}")
            return False
        
        finally:
            # 確保無論如何都清除指令寄存器
            if not clear_success:
                try:
                    clear_success = self.clear_motor_command()
                    if clear_success:
                        print("finally塊: 馬達指令寄存器清除成功")
                    else:
                        print("finally塊: 馬達指令寄存器清除失敗")
                except Exception as clear_error:
                    print(f"finally塊: 清除馬達指令錯誤: {clear_error}")
    
    def trigger_ccd3_detection(self) -> bool:
        """觸發CCD3角度檢測 - 修正：確保Ready後才發送指令"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                print("Modbus未連接，無法觸發CCD3檢測")
                return False
            
            # 步驟1：先確保CCD3指令寄存器已清零
            print("步驟1：確保CCD3指令寄存器已清零...")
            clear_result = self.modbus_client.write_register(
                address=self.ccd3_base_address, value=0, slave=1
            )
            if clear_result.isError():
                print("清零CCD3指令失敗")
                return False
            
            # 步驟2：等待CCD3回到Ready狀態
            print("步驟2：等待CCD3回到Ready狀態...")
            ready_timeout = 3.0  # 3秒超時
            start_time = time.time()
            
            while time.time() - start_time < ready_timeout:
                # 讀取CCD3狀態
                status_result = self.modbus_client.read_holding_registers(
                    address=self.ccd3_base_address + 1, count=1, slave=1
                )
                
                if not status_result.isError():
                    status_register = status_result.registers[0]
                    ready = bool(status_register & (1 << 0))    # bit 0: Ready
                    running = bool(status_register & (1 << 1))  # bit 1: Running
                    
                    print(f"CCD3準備狀態檢查: Ready={ready}, Running={running}, status_register={status_register}")
                    
                    if ready and not running:
                        print("CCD3已準備就緒")
                        break
                else:
                    print("讀取CCD3狀態失敗")
                
                time.sleep(0.2)  # 200ms檢查間隔
            else:
                print("等待CCD3準備就緒超時")
                return False
            
            # 步驟3：發送拍照+角度檢測指令
            print("步驟3：發送CCD3拍照+角度檢測指令...")
            result = self.modbus_client.write_register(
                address=self.ccd3_base_address, value=16, slave=1
            )
            
            if result.isError():
                print("發送CCD3檢測指令失敗")
                return False
            
            print("CCD3檢測指令發送成功，等待檢測完成...")
            
            # 步驟4：確認CCD3開始Running
            time.sleep(0.2)  # 給CCD3一點時間開始處理
            confirm_result = self.modbus_client.read_holding_registers(
                address=self.ccd3_base_address + 1, count=1, slave=1
            )
            
            if not confirm_result.isError():
                confirm_status = confirm_result.registers[0]
                confirm_ready = bool(confirm_status & (1 << 0))
                confirm_running = bool(confirm_status & (1 << 1))
                print(f"指令發送後狀態確認: Ready={confirm_ready}, Running={confirm_running}, status_register={confirm_status}")
                
                if not confirm_running:
                    print("警告：CCD3未進入Running狀態，可能指令未被接受")
            
            return True
            
        except Exception as e:
            print(f"觸發CCD3檢測錯誤: {e}")
            return False
    
    def wait_ccd3_complete(self, timeout: float = 10.0) -> bool:
        """等待CCD3檢測完成 - 修正：手動清零指令確保Ready"""
        try:
            print("等待CCD3檢測完成...")
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                # 讀取CCD3狀態寄存器801
                result = self.modbus_client.read_holding_registers(
                    address=self.ccd3_base_address + 1, count=1, slave=1
                )
                
                if result.isError():
                    print("讀取CCD3狀態失敗")
                    time.sleep(0.5)
                    continue
                
                status_register = result.registers[0]
                ready = bool(status_register & (1 << 0))    # bit 0: Ready
                running = bool(status_register & (1 << 1))  # bit 1: Running
                
                print(f"CCD3狀態檢查: Ready={ready}, Running={running}, status_register={status_register}")
                
                if ready and not running:
                    print("CCD3檢測完成")
                    # 關鍵修正：手動清零CCD3指令寄存器，確保下次能重新檢測
                    print("清零CCD3指令寄存器，確保下次重新檢測...")
                    clear_result = self.modbus_client.write_register(
                        address=self.ccd3_base_address, value=0, slave=1
                    )
                    if clear_result.isError():
                        print("清零CCD3指令失敗")
                    else:
                        print("CCD3指令已清零，系統Ready可接受新指令")
                    return True
                
                time.sleep(0.5)  # 500ms檢查間隔
            
            print(f"CCD3檢測超時 ({timeout}秒)")
            # 超時也要嘗試清零指令
            try:
                self.modbus_client.write_register(
                    address=self.ccd3_base_address, value=0, slave=1
                )
                print("超時情況下已清零CCD3指令")
            except:
                pass
            return False
            
        except Exception as e:
            print(f"等待CCD3完成錯誤: {e}")
            # 異常情況也要嘗試清零指令
            try:
                self.modbus_client.write_register(
                    address=self.ccd3_base_address, value=0, slave=1
                )
                print("異常情況下已清零CCD3指令")
            except:
                pass
            return False
    
    def execute_angle_correction(self) -> AngleResult:
        """執行完整的角度校正流程"""
        try:
            print("=== 開始執行角度校正流程 ===")
            
            # 步驟1: 觸發CCD3檢測
            self.state_machine.set_ccd_detecting(True)
            
            if not self.trigger_ccd3_detection():
                raise Exception("觸發CCD3檢測失敗")
            
            # 步驟2: 等待CCD3檢測完成
            if not self.wait_ccd3_complete():
                raise Exception("CCD3檢測超時或失敗")
            
            self.state_machine.set_ccd_detecting(False)
            
            # 步驟3: 讀取CCD3檢測角度
            ccd_angle = self.read_ccd3_angle()
            if ccd_angle is None:
                raise Exception("無法讀取CCD3檢測角度")
            
            # 步驟4: 計算馬達目標位置
            motor_position = self.calculate_motor_position(ccd_angle)
            
            # 步驟5: 發送馬達移動指令
            self.state_machine.set_motor_moving(True)
            
            if not self.send_motor_command(motor_position):
                raise Exception("發送馬達移動指令失敗")
            
            # 步驟6: 等待馬達運動完成
            if not self.wait_motor_complete():
                raise Exception("馬達運動超時或失敗")
            
            self.state_machine.set_motor_moving(False)
            
            # 計算角度差
            angle_diff = abs(90.0 - ccd_angle)
            
            print("=== 角度校正流程完成 ===")
            print(f"檢測角度: {ccd_angle:.2f}度")
            print(f"角度差: {angle_diff:.2f}度")
            print(f"馬達位置: {motor_position}")
            
            self.operation_count += 1
            
            return AngleResult(
                success=True,
                original_angle=ccd_angle,
                angle_diff=angle_diff,
                motor_position=motor_position
            )
            
        except Exception as e:
            print(f"角度校正流程失敗: {e}")
            self.error_count += 1
            self.state_machine.set_ccd_detecting(False)
            self.state_machine.set_motor_moving(False)
            self.state_machine.set_alarm(True)
            
            return AngleResult(
                success=False,
                original_angle=None,
                angle_diff=None,
                motor_position=None,
                error_message=str(e)
            )
    
    def write_status_registers(self):
        """更新狀態寄存器到主服務器"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                return
            
            # 狀態寄存器 (700-714)
            status_registers = [
                self.state_machine.status_register,  # 700: 狀態位
                1 if (self.modbus_client and self.modbus_client.connected) else 0,  # 701: Modbus連接
                1 if self.motor_rtu.serial_conn and self.motor_rtu.serial_conn.is_open else 0,  # 702: 馬達連接
                0,  # 703: 錯誤代碼
                self.operation_count & 0xFFFF,  # 704: 操作計數低位
                (self.operation_count >> 16) & 0xFFFF,  # 705: 操作計數高位
                self.error_count,  # 706: 錯誤計數
                0, 0, 0, 0, 0, 0, 0, 0  # 707-714: 保留
            ]
            
            # 批次寫入狀態寄存器
            self.modbus_client.write_registers(
                address=self.base_address, values=status_registers, slave=1
            )
            
        except Exception as e:
            print(f"寫入狀態寄存器錯誤: {e}")
    
    def write_result_registers(self, result: AngleResult):
        """寫入檢測結果到寄存器"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                return
            
            # 結果寄存器 (720-739)
            result_registers = [0] * 20
            
            if result.success:
                result_registers[0] = 1  # 720: 成功標誌
                
                if result.original_angle is not None:
                    # 原始角度32位存儲 (保留2位小數)
                    angle_int = int(result.original_angle * 100)
                    result_registers[1] = (angle_int >> 16) & 0xFFFF  # 721: 角度高位
                    result_registers[2] = angle_int & 0xFFFF          # 722: 角度低位
                
                if result.angle_diff is not None:
                    # 角度差32位存儲
                    diff_int = int(result.angle_diff * 100)
                    result_registers[3] = (diff_int >> 16) & 0xFFFF   # 723: 角度差高位
                    result_registers[4] = diff_int & 0xFFFF           # 724: 角度差低位
                
                if result.motor_position is not None:
                    # 馬達位置32位存儲
                    result_registers[5] = (result.motor_position >> 16) & 0xFFFF  # 725: 位置高位
                    result_registers[6] = result.motor_position & 0xFFFF          # 726: 位置低位
            
            # 統計資訊
            result_registers[10] = self.operation_count & 0xFFFF      # 730: 成功次數低位
            result_registers[11] = (self.operation_count >> 16) & 0xFFFF  # 731: 成功次數高位
            result_registers[12] = self.error_count                   # 732: 錯誤次數
            result_registers[13] = int(time.time() - self.start_time) # 733: 運行時間
            
            # 批次寫入結果寄存器
            self.modbus_client.write_registers(
                address=self.base_address + 20, values=result_registers, slave=1
            )
            
            print(f"檢測結果已寫入寄存器: 成功={result.success}")
            
        except Exception as e:
            print(f"寫入結果寄存器錯誤: {e}")
    
    def _handshake_sync_loop(self):
        """握手同步循環 - 50ms輪詢"""
        print("角度調整握手同步線程啟動")
        
        while not self.stop_handshake:
            try:
                if self.modbus_client and self.modbus_client.connected:
                    # 更新狀態寄存器
                    self.write_status_registers()
                    
                    # 處理控制指令
                    self._process_control_commands()
                
                time.sleep(0.05)  # 50ms循環
                
            except Exception as e:
                print(f"握手同步錯誤: {e}")
                time.sleep(1)
        
        print("角度調整握手同步線程停止")
    
    def _process_control_commands(self):
        """處理控制指令"""
        try:
            # 讀取控制指令寄存器 (740)
            result = self.modbus_client.read_holding_registers(
                address=self.base_address + 40, count=1, slave=1
            )
            
            if result.isError():
                return
            
            control_command = result.registers[0]
            
            # 檢查新指令 - 修正：允許重複執行相同指令
            if control_command != 0 and not self.command_processing:
                # 只要有非零指令且當前未處理，就執行
                print(f"收到控制指令: {control_command} (上次: {self.last_control_command})")
                self._handle_control_command(control_command)
                self.last_control_command = control_command
            
            # PLC清零指令後恢復Ready
            elif control_command == 0 and self.last_control_command != 0:
                print("PLC已清零指令，恢復Ready狀態")
                self.state_machine.set_ready(True)
                self.last_control_command = 0
                
        except Exception as e:
            print(f"控制指令處理錯誤: {e}")
    
    def _handle_control_command(self, command: int):
        """處理控制指令"""
        # 檢查系統是否完全初始化
        if not self.state_machine.is_initialized():
            print(f"系統未完全初始化，無法執行指令 {command}")
            return
            
        if not self.state_machine.is_ready():
            print(f"系統未Ready，無法執行指令 {command}")
            return
        
        # 檢查馬達連接狀態
        if not (self.motor_rtu.serial_conn and self.motor_rtu.serial_conn.is_open):
            print(f"馬達驅動器未連接，無法執行指令 {command}")
            self.state_machine.set_alarm(True)
            return
        
        print(f"開始處理控制指令: {command}")
        self.command_processing = True
        self.state_machine.set_ready(False)
        self.state_machine.set_running(True)
        
        # 異步執行指令
        threading.Thread(target=self._execute_command_async, args=(command,), daemon=True).start()
    
    def _execute_command_async(self, command: int):
        """異步執行指令"""
        try:
            if command == ControlCommand.ANGLE_CORRECTION.value:
                # 角度校正指令
                print("執行角度校正指令")
                result = self.execute_angle_correction()
                self.write_result_registers(result)
                
            elif command == ControlCommand.MOTOR_RESET.value:
                # 馬達重置指令 - 等待Ready後再清除
                print("執行馬達重置指令")
                if self.wait_motor_ready(timeout=5.0):
                    self.clear_motor_command()
                    print("馬達重置完成")
                else:
                    print("馬達重置失敗：等待Ready超時")
                
            elif command == ControlCommand.ERROR_RESET.value:
                # 錯誤重置指令
                print("執行錯誤重置指令")
                self.state_machine.set_alarm(False)
                self.error_count = 0
                
            else:
                print(f"未知指令: {command}")
                
        except Exception as e:
            print(f"指令執行錯誤: {e}")
            self.error_count += 1
            self.state_machine.set_alarm(True)
        
        finally:
            print(f"控制指令 {command} 執行完成")
            self.command_processing = False
            self.state_machine.set_running(False)
            self.state_machine.set_ccd_detecting(False)
            self.state_machine.set_motor_moving(False)
            if not self.state_machine.is_alarm():
                self.state_machine.set_ready(True)
    
    def start_handshake_service(self):
        """啟動握手服務"""
        if not self.handshake_thread or not self.handshake_thread.is_alive():
            self.stop_handshake = False
            self.handshake_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
            self.handshake_thread.start()
            print("角度調整握手服務已啟動")
    
    def stop_handshake_service(self):
        """停止握手服務"""
        print("正在停止角度調整握手服務...")
        self.stop_handshake = True
        if self.handshake_thread:
            self.handshake_thread.join(timeout=2)
    
    def disconnect(self):
        """斷開所有連接"""
        print("正在斷開角度調整系統所有連接...")
        self.stop_handshake_service()
        
        if self.motor_rtu:
            print("正在關閉馬達驅動器RTU連接...")
            # 在斷線前確保清除指令
            try:
                self.clear_motor_command()
            except:
                pass
            self.motor_rtu.disconnect()
        
        if self.modbus_client:
            print("正在關閉Modbus TCP連接...")
            self.modbus_client.close()
            self.modbus_client = None
        
        print("角度調整系統已斷開所有連接")

def auto_initialize_system():
    """系統自動初始化 - 修正順序"""
    print("=== 角度調整系統自動初始化開始 ===")
    
    # 1. 先連接馬達驅動器 (避免Modbus連接後立即接收指令)
    print("步驟1: 自動連接馬達驅動器...")
    motor_success = angle_service.connect_motor("COM5", 115200)
    if motor_success:
        print("✓ 馬達驅動器連接成功")
    else:
        print("✗ 馬達驅動器連接失敗")
        return False
    
    # 2. 後連接Modbus服務器
    print("步驟2: 自動連接Modbus服務器...")
    modbus_success = angle_service.connect_modbus()
    if modbus_success:
        print("✓ Modbus服務器連接成功")
        # 啟動握手服務
        angle_service.start_handshake_service()
        print("✓ 握手服務已啟動")
        
        # 3. 設置系統Ready狀態
        angle_service.state_machine.set_ready(True)
        print("✓ 系統狀態設置為Ready")
    else:
        print("✗ Modbus服務器連接失敗")
        return False
    
    print("=== 角度調整系統自動初始化完成 ===")
    print(f"狀態: Ready={angle_service.state_machine.is_ready()}")
    print(f"狀態: Initialized={angle_service.state_machine.is_initialized()}")
    print(f"狀態: Alarm={angle_service.state_machine.is_alarm()}")
    return True

# 全局服務實例
angle_service = AngleAdjustmentService()

if __name__ == '__main__':
    print("角度調整系統啟動中...")
    print(f"系統架構: Modbus TCP Client + RTU橋接 - 狀態機交握模式")
    print(f"基地址: {angle_service.base_address}")
    print(f"CCD3基地址: {angle_service.ccd3_base_address}")
    print(f"Modbus服務器: {angle_service.server_ip}:{angle_service.server_port}")
    print(f"馬達驅動器: COM5, 115200, Slave {angle_service.motor_slave_id}")
    print(f"功能: CCD3拍照 → 角度計算 → 馬達補正")
    
    # 執行自動初始化
    auto_success = auto_initialize_system()
    if auto_success:
        print("系統已就緒，等待PLC指令...")
        print("控制指令:")
        print("  1 = 角度校正 (拍照→計算→馬達移動)")
        print("  2 = 馬達重置")
        print("  7 = 錯誤重置")
    else:
        print("系統初始化失敗")
    
    try:
        # 保持主程序運行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n正在關閉角度調整系統...")
        angle_service.disconnect()
    except Exception as e:
        print(f"系統錯誤: {e}")
        angle_service.disconnect()