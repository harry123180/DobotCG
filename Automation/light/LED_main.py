#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED_main.py - LED控制器RS232轉ModbusTCP Client主程序
實現LED控制器RS232轉TCP橋接，狀態機交握，自動重連
參考VP_main.py和XCModule.py架構
"""

import sys
import os
import time
import threading
import json
import signal
import logging
import serial
import serial.tools.list_ports
from typing import Dict, Any, Optional
from datetime import datetime
from pymodbus.client import ModbusTcpClient

logger = logging.getLogger(__name__)

class LEDControllerModbusClient:
    """LED控制器Modbus TCP Client - RS232轉TCP橋接模組"""
    
    def __init__(self, config_file="led_config.json"):
        # 載入配置
        self.config = self.load_config(config_file)
        
        # 核心組件
        self.serial_connection: Optional[serial.Serial] = None
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.running = False
        
        # 狀態變數
        self.connected_to_server = False
        self.connected_to_device = False
        self.last_command_id = 0
        self.executing_command = False
        
        # LED狀態管理
        self.led_states = [False, False, False, False]  # L1-L4開關狀態
        self.led_brightness = [0, 0, 0, 0]  # L1-L4亮度 (0-511)
        self.device_error_code = 0
        self.last_error_response = ""
        
        # 執行緒控制
        self.main_loop_thread = None
        self.loop_lock = threading.Lock()
        
        # 統計計數
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        
        # 寄存器映射 (基地址 + 偏移)
        self.base_address = self.config['modbus_mapping']['base_address']
        self.init_register_mapping()
        
        # 設置日誌
        self.setup_logging()
        
    def setup_logging(self):
        """設置日誌"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
    def load_config(self, config_file: str) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "module_id": "LED控制器模組",
            "serial_connection": {
                "port": "COM6",
                "baudrate": 9600,
                "parity": "N",
                "stopbits": 1,
                "bytesize": 8,
                "timeout": 1.0
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 3.0
            },
            "modbus_mapping": {
                "base_address": 600
            },
            "timing": {
                "fast_loop_interval": 0.05,
                "command_delay": 0.1,
                "serial_delay": 0.05
            }
        }
        
        try:
            # 取得當前執行檔案的目錄
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(current_dir, config_file)
            
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    # 合併配置
                    default_config.update(loaded_config)
                print(f"已載入配置檔案: {config_path}")
            else:
                # 創建預設配置檔案在執行檔案同層目錄
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            print(f"載入配置檔案失敗: {e}")
            
        return default_config
    
    def init_register_mapping(self):
        """初始化寄存器映射"""
        base = self.base_address
        
        # 狀態寄存器區 (只讀) base+0 ~ base+15
        self.status_registers = {
            'module_status': base + 0,          # 模組狀態 (0=離線, 1=閒置, 2=執行中, 3=初始化, 4=錯誤)
            'device_connection': base + 1,      # 設備連接狀態 (0=斷開, 1=已連接)
            'active_channels': base + 2,        # 開啟通道數量
            'error_code': base + 3,             # 錯誤代碼
            'l1_state': base + 4,               # L1狀態 (0=OFF, 1=ON)
            'l2_state': base + 5,               # L2狀態
            'l3_state': base + 6,               # L3狀態
            'l4_state': base + 7,               # L4狀態
            'l1_brightness': base + 8,          # L1亮度 (0-511)
            'l2_brightness': base + 9,          # L2亮度
            'l3_brightness': base + 10,         # L3亮度
            'l4_brightness': base + 11,         # L4亮度
            'operation_count': base + 12,       # 操作計數
            'error_count': base + 13,           # 錯誤計數
            'reserved_14': base + 14,           # 保留
            'timestamp': base + 15              # 時間戳
        }
        
        # 指令寄存器區 (讀寫) base+20 ~ base+24
        self.command_registers = {
            'command_code': base + 20,          # 指令代碼
            'param1': base + 21,                # 參數1 (通道號/亮度值)
            'param2': base + 22,                # 參數2 (亮度值)
            'command_id': base + 23,            # 指令ID
            'reserved': base + 24               # 保留
        }
        
        # 所有寄存器
        self.all_registers = {**self.status_registers, **self.command_registers}
        
        logger.info(f"寄存器映射初始化完成 - 基地址: {base}")
        print(f"LED控制器模組寄存器映射:")
        print(f"  基地址: {base}")
        print(f"  狀態寄存器: {base} ~ {base + 15}")
        print(f"  指令寄存器: {base + 20} ~ {base + 24}")
        print(f"  指令映射:")
        print(f"    0: NOP (無操作)")
        print(f"    1: 全部開啟")
        print(f"    2: 全部關閉") 
        print(f"    3: 重置設備")
        print(f"    4: 設定單一通道亮度 (param1=通道1-4, param2=亮度0-511)")
        print(f"    5: 開啟單一通道 (param1=通道1-4)")
        print(f"    6: 關閉單一通道 (param1=通道1-4)")
        print(f"    7: 錯誤重置")
    
    def connect_main_server(self) -> bool:
        """連接到主Modbus TCP服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            server_config = self.config['tcp_server']
            self.modbus_client = ModbusTcpClient(
                host=server_config['host'],
                port=server_config['port'],
                timeout=server_config['timeout']
            )
            
            if self.modbus_client.connect():
                self.connected_to_server = True
                self.connection_count += 1
                print(f"連接到主服務器成功: {server_config['host']}:{server_config['port']}")
                
                # 初始化寄存器
                self.init_status_registers()
                return True
            else:
                print("連接到主服務器失敗")
                return False
                
        except Exception as e:
            print(f"連接主服務器異常: {e}")
            self.connected_to_server = False
            return False
    
    def connect_serial_device(self) -> bool:
        """連接到LED控制器串口設備"""
        try:
            if self.serial_connection:
                self.serial_connection.close()
            
            serial_config = self.config['serial_connection']
            self.serial_connection = serial.Serial(
                port=serial_config['port'],
                baudrate=serial_config['baudrate'],
                parity=serial_config['parity'],
                stopbits=serial_config['stopbits'],
                bytesize=serial_config['bytesize'],
                timeout=serial_config['timeout']
            )
            
            if self.serial_connection.is_open:
                self.connected_to_device = True
                print(f"連接到LED控制器成功: {serial_config['port']}")
                
                # 測試通訊 - 發送VERSION查詢
                test_result = self.send_serial_command("VERSION?")
                if test_result:
                    print("LED控制器通訊測試成功")
                else:
                    print("LED控制器通訊測試無回應，但保持連接")
                
                return True
            else:
                print("LED控制器串口開啟失敗")
                return False
                
        except Exception as e:
            print(f"連接LED控制器異常: {e}")
            self.connected_to_device = False
            return False
    
    def send_serial_command(self, command: str) -> bool:
        """發送RS232指令到LED控制器"""
        if not self.connected_to_device or not self.serial_connection:
            self.logger.error("串口設備未連接")
            return False
        
        try:
            # 添加換行符 (根據手冊要求)
            full_command = command + "\r\n"
            self.serial_connection.write(full_command.encode('ascii'))
            
            # 讀取回應
            time.sleep(self.config['timing']['serial_delay'])
            response = ""
            if self.serial_connection.in_waiting > 0:
                response = self.serial_connection.read(self.serial_connection.in_waiting).decode('ascii', errors='ignore')
                self.logger.debug(f"發送: {command} | 回應: {response.strip()}")
            else:
                self.logger.debug(f"發送: {command} | 無回應")
            
            self.operation_count += 1
            return True
            
        except Exception as e:
            self.logger.error(f"發送串口指令失敗: {e}")
            self.error_count += 1
            return False
    
    def init_status_registers(self):
        """初始化狀態寄存器"""
        try:
            # 寫入模組基本資訊
            self.write_register('module_status', 1)  # 閒置狀態
            self.write_register('error_code', 0)     # 無錯誤
            self.write_register('operation_count', self.operation_count)
            self.write_register('error_count', self.error_count)
            
            print("狀態寄存器初始化完成")
        except Exception as e:
            print(f"初始化狀態寄存器失敗: {e}")
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected_to_server or register_name not in self.all_registers:
            return None
        
        try:
            address = self.all_registers[register_name]
            result = self.modbus_client.read_holding_registers(
                address=address,
                count=1,
                slave=self.config['tcp_server']['unit_id']
            )
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            pass  # 靜默處理讀取錯誤
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected_to_server or register_name not in self.all_registers:
            return False
        
        try:
            address = self.all_registers[register_name]
            result = self.modbus_client.write_register(
                address=address,
                value=value,
                slave=self.config['tcp_server']['unit_id']
            )
            
            return not result.isError()
                
        except Exception as e:
            pass  # 靜默處理寫入錯誤
            return False
    
    def execute_command(self, command: int, param1: int, param2: int) -> bool:
        """執行LED控制指令"""
        if not self.connected_to_device:
            self.logger.error("LED設備未連接，無法執行指令")
            return False
        
        try:
            success = False
            
            if command == 0:  # NOP
                success = True
                
            elif command == 1:  # 全部開啟
                success = True
                for i in range(4):
                    cmd = f"CH{i+1}:255"
                    if self.send_serial_command(cmd):
                        self.led_states[i] = True
                        self.led_brightness[i] = 255
                    else:
                        success = False
                    time.sleep(self.config['timing']['serial_delay'])
                
            elif command == 2:  # 全部關閉
                success = True
                for i in range(4):
                    cmd = f"CH{i+1}:0"
                    if self.send_serial_command(cmd):
                        self.led_states[i] = False
                        self.led_brightness[i] = 0
                    else:
                        success = False
                    time.sleep(self.config['timing']['serial_delay'])
                
            elif command == 3:  # 重置設備
                success = self.send_serial_command("RESET")
                if success:
                    # 重置本地狀態
                    for i in range(4):
                        self.led_states[i] = False
                        self.led_brightness[i] = 0
                
            elif command == 4:  # 設定單一通道亮度
                if 1 <= param1 <= 4 and 0 <= param2 <= 511:
                    cmd = f"CH{param1}:{param2}"
                    success = self.send_serial_command(cmd)
                    if success:
                        channel_idx = param1 - 1
                        self.led_brightness[channel_idx] = param2
                        self.led_states[channel_idx] = param2 > 0
                
            elif command == 5:  # 開啟單一通道
                if 1 <= param1 <= 4:
                    channel_idx = param1 - 1
                    brightness = self.led_brightness[channel_idx] if self.led_brightness[channel_idx] > 0 else 255
                    cmd = f"CH{param1}:{brightness}"
                    success = self.send_serial_command(cmd)
                    if success:
                        self.led_states[channel_idx] = True
                        self.led_brightness[channel_idx] = brightness
                
            elif command == 6:  # 關閉單一通道
                if 1 <= param1 <= 4:
                    cmd = f"CH{param1}:0"
                    success = self.send_serial_command(cmd)
                    if success:
                        channel_idx = param1 - 1
                        self.led_states[channel_idx] = False
                        self.led_brightness[channel_idx] = 0
                
            elif command == 7:  # 錯誤重置
                self.error_count = 0
                self.device_error_code = 0
                success = True
            
            if success:
                print(f"指令執行成功: cmd={command}, p1={param1}, p2={param2}")
            else:
                self.error_count += 1
                print(f"指令執行失敗: cmd={command}, p1={param1}, p2={param2}")
                
            return success
            
        except Exception as e:
            print(f"執行指令異常: {e}")
            self.error_count += 1
            return False
    
    def update_status_registers(self):
        """更新狀態寄存器"""
        try:
            # 更新連接狀態
            self.write_register('device_connection', 1 if self.connected_to_device else 0)
            
            # 更新LED狀態
            for i in range(4):
                self.write_register(f'l{i+1}_state', 1 if self.led_states[i] else 0)
                self.write_register(f'l{i+1}_brightness', self.led_brightness[i])
            
            # 更新活動通道數量
            active_count = sum(1 for state in self.led_states if state)
            self.write_register('active_channels', active_count)
            
            # 更新模組狀態
            if self.executing_command:
                self.write_register('module_status', 2)  # 執行中
            elif not self.connected_to_device:
                self.write_register('module_status', 0)  # 離線
            elif self.error_count > 10:
                self.write_register('module_status', 4)  # 錯誤
            else:
                self.write_register('module_status', 1)  # 閒置
            
            # 更新統計信息
            self.write_register('operation_count', self.operation_count % 65536)
            self.write_register('error_count', self.error_count % 65536)
            self.write_register('error_code', self.device_error_code)
            self.write_register('timestamp', int(time.time()) % 65536)
            
        except Exception as e:
            pass  # 靜默處理狀態更新錯誤
    
    def process_commands(self):
        """處理指令 (狀態機交握)"""
        try:
            # 讀取新指令ID
            new_command_id = self.read_register('command_id')
            if new_command_id is None or new_command_id == self.last_command_id:
                return
            
            # 檢測到新指令
            command_code = self.read_register('command_code')
            param1 = self.read_register('param1')
            param2 = self.read_register('param2')
            
            if command_code is None:
                return
            
            print(f"收到新指令: ID={new_command_id}, CMD={command_code}, P1={param1}, P2={param2}")
            
            # 設置執行狀態
            self.executing_command = True
            
            # 執行指令
            success = self.execute_command(command_code, param1 or 0, param2 or 0)
            
            # 更新錯誤狀態
            if not success:
                self.device_error_code = 1  # 指令執行失敗
            else:
                self.device_error_code = 0  # 執行成功
            
            # 清除執行狀態
            self.executing_command = False
            
            # 清除指令寄存器
            self.write_register('command_code', 0)
            self.write_register('param1', 0)
            self.write_register('param2', 0)
            self.write_register('command_id', 0)
            
            # 更新指令ID
            self.last_command_id = new_command_id
            
        except Exception as e:
            print(f"處理指令失敗: {e}")
            self.executing_command = False
            self.error_count += 1
    
    def main_loop(self):
        """主循環"""
        loop_interval = self.config['timing']['fast_loop_interval']
        
        while self.running:
            try:
                with self.loop_lock:
                    # 檢查連接狀態
                    if not self.connected_to_server:
                        if not self.connect_main_server():
                            time.sleep(1)
                            continue
                    
                    if not self.connected_to_device:
                        if not self.connect_serial_device():
                            time.sleep(1)
                            continue
                    
                    # 處理指令
                    self.process_commands()
                    
                    # 更新狀態
                    self.update_status_registers()
                
                time.sleep(loop_interval)
                
            except Exception as e:
                print(f"主循環異常: {e}")
                self.error_count += 1
                time.sleep(0.5)
    
    def start(self) -> bool:
        """啟動模組"""
        if self.running:
            print("模組已在運行中")
            return False
        
        try:
            # 連接服務器和設備
            if not self.connect_main_server():
                print("無法連接到主服務器")
                return False
            
            if not self.connect_serial_device():
                print("無法連接到LED設備，將在主循環中重試")
            
            # 啟動主循環
            self.running = True
            self.main_loop_thread = threading.Thread(target=self.main_loop, daemon=True)
            self.main_loop_thread.start()
            
            print("LED控制器模組啟動成功")
            return True
            
        except Exception as e:
            print(f"啟動模組失敗: {e}")
            return False
    
    def stop(self):
        """停止模組"""
        print("正在停止LED控制器模組...")
        
        self.running = False
        
        # 關閉LED (如果連接)
        if self.connected_to_device:
            try:
                for i in range(4):
                    self.send_serial_command(f"CH{i+1}:0")
                    time.sleep(0.05)
            except:
                pass
        
        # 更新狀態為離線
        if self.connected_to_server:
            try:
                self.write_register('module_status', 0)  # 離線
                self.write_register('device_connection', 0)
            except:
                pass
        
        # 關閉連接
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except:
                pass
        
        if self.modbus_client:
            try:
                self.modbus_client.close()
            except:
                pass
        
        # 等待線程結束
        if self.main_loop_thread and self.main_loop_thread.is_alive():
            self.main_loop_thread.join(timeout=2)
        
        print("LED控制器模組已停止")
    
    def get_status(self) -> Dict[str, Any]:
        """獲取模組狀態"""
        uptime = time.time() - self.start_time
        
        status = {
            'module_id': self.config['module_id'],
            'running': self.running,
            'connected_to_server': self.connected_to_server,
            'connected_to_device': self.connected_to_device,
            'executing_command': self.executing_command,
            'led_states': self.led_states.copy(),
            'led_brightness': self.led_brightness.copy(),
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'connection_count': self.connection_count,
            'uptime_seconds': int(uptime),
            'last_command_id': self.last_command_id,
            'device_error_code': self.device_error_code,
            'config': self.config,
            'register_mapping': {
                'base_address': self.base_address,
                'status_registers': self.status_registers,
                'command_registers': self.command_registers
            }
        }
        
        return status


def main():
    """主函數"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    print("LED控制器Modbus TCP Client啟動中...")
    print(f"執行目錄: {current_dir}")
    
    # 創建模組實例
    led_client = LEDControllerModbusClient()
    
    # 信號處理
    def signal_handler(sig, frame):
        print("收到停止信號")
        led_client.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # 啟動模組
        if led_client.start():
            print(f"LED控制器模組運行中 - 基地址: {led_client.base_address}")
            print("寄存器映射:")
            print(f"  狀態寄存器: {led_client.base_address} ~ {led_client.base_address + 15}")
            print(f"  指令寄存器: {led_client.base_address + 20} ~ {led_client.base_address + 24}")
            print("按 Ctrl+C 停止程序")
            
            # 保持運行
            while led_client.running:
                time.sleep(1)
        else:
            print("模組啟動失敗")
            
    except KeyboardInterrupt:
        print("\n收到中斷信號")
    except Exception as e:
        print(f"運行異常: {e}")
    finally:
        led_client.stop()


if __name__ == '__main__':
    main()