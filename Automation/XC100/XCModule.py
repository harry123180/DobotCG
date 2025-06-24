#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XC100 超高速動作模組 - 穩定修復版本
修復了競爭條件和狀態管理問題
"""

import os
import time
import threading
import json
import logging
from datetime import datetime
from enum import Enum
from typing import Optional, Dict, Any
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient, ModbusTcpClient

# 確保XC100目錄存在
XC100_DIR = "XC100"
if not os.path.exists(XC100_DIR):
    os.makedirs(XC100_DIR)

class XCState(Enum):
    """XC100狀態枚舉"""
    OFFLINE = 0
    IDLE = 1
    MOVING = 2
    HOMING = 3
    ERROR = 4
    SERVO_OFF = 5
    EMERGENCY = 6

class XCCommand(Enum):
    """XC100指令枚舉"""
    NOP = 0
    SERVO_ON = 1
    SERVO_OFF = 2
    HOME = 3
    MOVE_ABS = 4
    MOVE_REL = 5
    EMERGENCY_STOP = 6
    RESET_ERROR = 7

class UltraFastXCModule:
    """超高速XC100模組"""
    
    def __init__(self, config_file=None):
        if config_file is None:
            config_file = os.path.join(XC100_DIR, "xc_ultrafast_config.json")
        self.config_file = config_file
        self.config = self.load_config()
        
        self.setup_logging()
        self.module_id = self.config.get("module_id", "XC100_FAST")
        
        # 連接客戶端
        self.xc_client: Optional[ModbusSerialClient] = None
        self.tcp_client: Optional[ModbusTcpClient] = None
        self.xc_connected = False
        self.tcp_connected = False
        
        # 狀態變量 - 使用鎖保護
        self._state_lock = threading.Lock()
        self.current_state = XCState.OFFLINE
        self.servo_status = False
        self.error_code = 0
        self.current_position = 0
        self.target_position = 0
        self.command_executing = False
        self.command_start_time = 0
        
        # 位置設定
        self.position_A = self.config.get("positions", {}).get("A", 400)
        self.position_B = self.config.get("positions", {}).get("B", 2682)
        
        # 超高速線程
        self.fast_loop_thread = None
        self.fast_loop_running = False
        
        # 指令管理
        self.last_command_id = 0
        self.modbus_base_address = self.config.get("modbus_mapping", {}).get("base_address", 1000)
        
        # 連接重試計數器
        self.tcp_retry_count = 0
        self.xc_retry_count = 0
        self.max_retry = 3
        
        self.logger.info(f"超高速XC100模組初始化: {self.module_id}")
    
    def load_config(self) -> Dict[str, Any]:
        """載入超高速配置"""
        default_config = {
            "module_id": "XC100_FAST",
            "description": "XC100超高速模組",
            "xc_connection": {
                "port": "COM5",
                "baudrate": 115200,
                "unit_id": 2,
                "timeout": 0.2,
                "retry_count": 2,
                "retry_delay": 0.01
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "modbus_mapping": {
                "base_address": 1000,
                "register_count": 50
            },
            "positions": {
                "A": 400,
                "B": 2682
            },
            "timing": {
                "fast_loop_interval": 0.02,  # 調整為20ms，更穩定
                "movement_delay": 0.1,       # 調整為100ms
                "command_delay": 0.02,       # 調整為20ms
                "register_delay": 0.01,      # 調整為10ms
                "no_wait_mode": True
            }
        }
        
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                loaded_config = json.load(f)
                for key, value in default_config.items():
                    if key not in loaded_config:
                        loaded_config[key] = value
                    elif isinstance(value, dict):
                        for sub_key, sub_value in value.items():
                            if sub_key not in loaded_config[key]:
                                loaded_config[key][sub_key] = sub_value
                return loaded_config
        except FileNotFoundError:
            self.save_config(default_config)
        except Exception as e:
            print(f"載入配置失敗: {e}")
            
        return default_config
    
    def save_config(self, config=None):
        """保存配置"""
        try:
            config_to_save = config or self.config
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config_to_save, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"保存配置失敗: {e}")
    
    def setup_logging(self):
        """設置精簡日誌"""
        log_file = os.path.join(XC100_DIR, f'xc_fast_{datetime.now().strftime("%Y%m%d")}.log')
        logging.basicConfig(
            level=logging.ERROR,  # 只記錄錯誤
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file, encoding='utf-8'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(f'FastXC.{self.config.get("module_id", "XC100")}')
    
    def safe_modbus_operation(self, operation_func, *args, **kwargs):
        """安全的Modbus操作包裝器"""
        try:
            return operation_func(*args, **kwargs)
        except Exception as e:
            self.logger.error(f"Modbus操作失敗: {e}")
            return None
    
    def connect_main_server(self) -> bool:
        """連接主服務器 - 增加錯誤處理"""
        try:
            if self.tcp_client:
                try:
                    self.tcp_client.close()
                except:
                    pass
            
            tcp_config = self.config["tcp_server"]
            self.tcp_client = ModbusTcpClient(
                host=tcp_config["host"],
                port=tcp_config["port"],
                timeout=tcp_config["timeout"]
            )
            
            if self.tcp_client.connect():
                self.tcp_connected = True
                self.tcp_retry_count = 0
                return True
            return False
        except Exception as e:
            self.logger.error(f"TCP連接失敗: {e}")
            self.tcp_connected = False
            return False
    
    def connect_xc100(self) -> bool:
        """連接XC100 - 增加錯誤處理"""
        try:
            if self.xc_client:
                try:
                    self.xc_client.close()
                except:
                    pass
            
            xc_config = self.config["xc_connection"]
            self.xc_client = ModbusSerialClient(
                port=xc_config["port"],
                baudrate=xc_config["baudrate"],
                stopbits=1,
                parity='N',
                timeout=xc_config["timeout"]
            )
            
            if self.xc_client.connect():
                self.xc_connected = True
                self.xc_retry_count = 0
                with self._state_lock:
                    self.current_state = XCState.IDLE
                return True
            return False
        except Exception as e:
            self.logger.error(f"XC100連接失敗: {e}")
            self.xc_connected = False
            return False
    
    def ultra_fast_rtu_write(self, address, values):
        """超高速RTU寫入 - 增加安全檢查"""
        if not self.xc_connected or not self.xc_client:
            return False
        
        try:
            unit_id = self.config["xc_connection"]["unit_id"]
            
            if isinstance(values, list):
                result = self.xc_client.write_registers(
                    address=address, 
                    values=values, 
                    slave=unit_id
                )
            else:
                result = self.xc_client.write_register(
                    address=address, 
                    value=values, 
                    slave=unit_id
                )
            
            if result.isError():
                self.logger.error(f"RTU寫入失敗: {result}")
                return False
            return True
            
        except Exception as e:
            self.logger.error(f"RTU寫入異常: {e}")
            return False
    
    def execute_xc_command_fast(self, command: XCCommand, param1=0, param2=0) -> bool:
        """超高速指令執行 - 改進狀態管理"""
        with self._state_lock:
            if not self.xc_connected or self.command_executing:
                return False
            
            # 設置執行狀態
            self.command_executing = True
            self.command_start_time = time.time()
        
        try:
            success = False
            
            if command == XCCommand.SERVO_ON:
                success = self.ultra_fast_rtu_write(0x2011, 0)
                if success:
                    with self._state_lock:
                        self.current_state = XCState.IDLE
                        self.servo_status = True
                
            elif command == XCCommand.SERVO_OFF:
                success = self.ultra_fast_rtu_write(0x2011, 1)
                if success:
                    with self._state_lock:
                        self.current_state = XCState.SERVO_OFF
                        self.servo_status = False
                
            elif command == XCCommand.HOME:
                success = self.ultra_fast_rtu_write(0x201E, 3)
                if success:
                    with self._state_lock:
                        self.current_state = XCState.HOMING
                
            elif command == XCCommand.MOVE_ABS:
                position = (param2 << 16) | param1
                with self._state_lock:
                    self.target_position = position
                
                position_high = (position >> 16) & 0xFFFF
                position_low = position & 0xFFFF
                
                # 第一個指令：寫入位置
                success1 = self.ultra_fast_rtu_write(0x2002, [position_high, position_low])
                
                if success1:
                    # 延遲控制
                    if not self.config["timing"].get("no_wait_mode", False):
                        time.sleep(self.config["timing"]["command_delay"])
                    
                    # 第二個指令：執行移動
                    success2 = self.ultra_fast_rtu_write(0x201E, 1)
                    if success2:
                        with self._state_lock:
                            self.current_state = XCState.MOVING
                        success = True
                        # 移動指令需要等待完成，不在這裡清除執行狀態
                        return True
                
            elif command == XCCommand.EMERGENCY_STOP:
                success = self.ultra_fast_rtu_write(0x2020, 1)
                if success:
                    with self._state_lock:
                        self.current_state = XCState.EMERGENCY
            
            # 對於非移動指令，立即清除執行狀態
            if command != XCCommand.MOVE_ABS:
                with self._state_lock:
                    self.command_executing = False
                    self.command_start_time = 0
            
            return success
            
        except Exception as e:
            self.logger.error(f"指令執行失敗: {e}")
            with self._state_lock:
                self.command_executing = False
                self.command_start_time = 0
            return False
    
    def ultra_fast_loop(self):
        """超高速主循環 - 改進穩定性"""
        print("超高速循環開始")
        
        loop_interval = self.config["timing"]["fast_loop_interval"]
        movement_delay = self.config["timing"]["movement_delay"]
        
        error_count = 0
        max_errors = 10
        
        while self.fast_loop_running:
            try:
                start_time = time.time()
                
                # 1. 檢查連接狀態
                if not self.tcp_connected and self.tcp_retry_count < self.max_retry:
                    if self.connect_main_server():
                        print("TCP重新連接成功")
                    else:
                        self.tcp_retry_count += 1
                
                # 2. 讀取指令 - 增加錯誤處理
                if self.tcp_connected and self.tcp_client:
                    try:
                        base_addr = self.modbus_base_address + 20
                        unit_id = self.config["tcp_server"]["unit_id"]
                        
                        result = self.tcp_client.read_holding_registers(
                            address=base_addr, count=5, slave=unit_id
                        )
                        
                        if not result.isError() and len(result.registers) >= 4:
                            command_code = result.registers[0]
                            param1 = result.registers[1]
                            param2 = result.registers[2]
                            command_id = result.registers[3]
                            
                            # 檢查新指令
                            if command_code != 0 and command_id != self.last_command_id:
                                self.last_command_id = command_id
                                
                                try:
                                    command = XCCommand(command_code)
                                    
                                    # 執行指令
                                    if self.execute_xc_command_fast(command, param1, param2):
                                        # 清除指令
                                        self.safe_modbus_operation(
                                            self.tcp_client.write_registers,
                                            address=base_addr,
                                            values=[0, 0, 0, 0, 0],
                                            slave=unit_id
                                        )
                                        
                                except ValueError:
                                    pass
                                except Exception as e:
                                    self.logger.error(f"指令處理失敗: {e}")
                        else:
                            if result.isError():
                                error_count += 1
                                if error_count > max_errors:
                                    self.tcp_connected = False
                                    error_count = 0
                                    
                    except Exception as e:
                        self.logger.error(f"指令讀取失敗: {e}")
                        error_count += 1
                        if error_count > max_errors:
                            self.tcp_connected = False
                            error_count = 0
                
                # 3. 檢查移動完成
                with self._state_lock:
                    if (self.command_executing and 
                        self.command_start_time > 0 and 
                        time.time() - self.command_start_time > movement_delay):
                        self.command_executing = False
                        if self.current_state == XCState.MOVING:
                            self.current_state = XCState.IDLE
                        self.command_start_time = 0
                
                # 4. 更新狀態到主服務器
                if self.tcp_connected and self.tcp_client:
                    try:
                        base_addr = self.modbus_base_address
                        unit_id = self.config["tcp_server"]["unit_id"]
                        
                        with self._state_lock:
                            key_data = [
                                self.current_state.value,
                                1 if self.xc_connected else 0,
                                1 if self.servo_status else 0,
                                self.error_code,
                                self.current_position & 0xFFFF,
                                (self.current_position >> 16) & 0xFFFF,
                                1 if self.command_executing else 0
                            ]
                        
                        self.safe_modbus_operation(
                            self.tcp_client.write_registers,
                            address=base_addr,
                            values=key_data,
                            slave=unit_id
                        )
                        
                    except Exception as e:
                        self.logger.error(f"狀態更新失敗: {e}")
                
                # 5. 精確計時
                elapsed = time.time() - start_time
                if elapsed < loop_interval:
                    time.sleep(loop_interval - elapsed)
                    
            except Exception as e:
                self.logger.error(f"主循環異常: {e}")
                time.sleep(loop_interval)
        
        print("超高速循環停止")
    
    def start_fast_loop(self):
        """啟動超高速循環"""
        if not self.fast_loop_running:
            self.fast_loop_running = True
            self.fast_loop_thread = threading.Thread(target=self.ultra_fast_loop, daemon=True)
            self.fast_loop_thread.start()
    
    def stop_fast_loop(self):
        """停止超高速循環"""
        self.fast_loop_running = False
        if self.fast_loop_thread:
            self.fast_loop_thread.join(timeout=2)
    
    def start(self):
        """啟動超高速模組"""
        print(f"啟動超高速XC100模組: {self.module_id}")
        
        if not self.connect_main_server():
            print("主服務器連接失敗")
            return False
        
        if not self.connect_xc100():
            print("XC100連接失敗，但繼續運行")
        
        self.start_fast_loop()
        print(f"超高速模組啟動成功 (循環間隔: {self.config['timing']['fast_loop_interval']*1000}ms)")
        return True
    
    def stop(self):
        """停止模組"""
        print(f"停止超高速模組: {self.module_id}")
        self.stop_fast_loop()
        
        try:
            if self.xc_client and self.xc_connected:
                self.xc_client.close()
        except:
            pass
            
        try:
            if self.tcp_client and self.tcp_connected:
                self.tcp_client.close()
        except:
            pass
    
    def get_status(self):
        """獲取狀態"""
        with self._state_lock:
            return {
                "module_id": self.module_id,
                "tcp_connected": self.tcp_connected,
                "xc_connected": self.xc_connected,
                "current_state": self.current_state.name,
                "current_position": self.current_position,
                "command_executing": self.command_executing,
                "fast_mode": True
            }

def main():
    """超高速主函數"""
    import argparse
    
    parser = argparse.ArgumentParser(description='XC100超高速動作模組')
    parser.add_argument('--config', type=str, help='配置文件路徑')
    parser.add_argument('--port', type=str, help='XC100串口號')
    args = parser.parse_args()
    
    print("XC100超高速動作模組 - 穩定修復版本")
    print("=" * 50)
    
    module = UltraFastXCModule(args.config)
    
    if args.port:
        module.config["xc_connection"]["port"] = args.port
        module.save_config()
    
    try:
        if module.start():
            print(f"超高速模組運行中: {module.module_id}")
            print(f"循環間隔: {module.config['timing']['fast_loop_interval']*1000}ms")
            print(f"移動延遲: {module.config['timing']['movement_delay']*1000}ms")
            print("按 Ctrl+C 停止")
            
            while True:
                status = module.get_status()
                print(f"\r超高速模組: {status['current_state']} | "
                      f"TCP: {'連接' if status['tcp_connected'] else '斷開'} | "
                      f"XC100: {'連接' if status['xc_connected'] else '斷開'} | "
                      f"執行中: {'是' if status['command_executing'] else '否'}", end="")
                time.sleep(0.2)  # 調整為200ms，減少輸出頻率
        else:
            print("模組啟動失敗")
            
    except KeyboardInterrupt:
        print("\n正在停止超高速模組...")
        module.stop()
        print("已停止")

if __name__ == "__main__":
    main()