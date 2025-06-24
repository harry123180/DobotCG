#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RS485Gateway.py - RS485網關主程序
實現RTU到TCP的橋接，專門處理PGC、PGHL、PGE三款夾爪
"""

import os
import json
import time
import threading
from datetime import datetime
from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.exceptions import ModbusException, ConnectionException

class RS485Gateway:
    def __init__(self, config_file='rs485_config.json'):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # RTU客戶端
        self.rtu_client = None
        self.rtu_connected = False
        
        # TCP客戶端（連接主服務器）
        self.tcp_client = None
        self.tcp_connected = False
        
        # 運行狀態
        self.running = False
        self.loop_thread = None
        self.last_command_ids = {}
        
        # 設備映射
        self.device_mapping = {
            'PGC': {
                'unit_id': 6,
                'status_base': 500,
                'command_base': 520
            },
            'PGHL': {
                'unit_id': 5,
                'status_base': 530,
                'command_base': 550
            },
            'PGE': {
                'unit_id': 4,
                'status_base': 560,
                'command_base': 580
            }
        }
        
        print("RS485Gateway啟動")
        print(f"RTU端口: {self.config['rtu_connection']['port']}")
        print("設備映射:")
        for name, mapping in self.device_mapping.items():
            print(f"  {name} (unit_id={mapping['unit_id']}): 狀態 {mapping['status_base']}-{mapping['status_base']+19}, 指令 {mapping['command_base']}-{mapping['command_base']+9}")

    def load_config(self):
        """載入配置檔案"""
        default_config = {
            "module_id": "RS485Gateway",
            "rtu_connection": {
                "port": "COM5",
                "baudrate": 115200,
                "timeout": 2.0,
                "stopbits": 1,
                "bytesize": 8,
                "parity": "N"
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 3.0
            },
            "timing": {
                "fast_loop_interval": 0.1,
                "device_test_interval": 10
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                print(f"已載入配置檔案: {self.config_file}")
                return config
            except Exception as e:
                print(f"載入配置檔案失敗: {e}")
        
        # 生成默認配置檔案
        with open(self.config_file, 'w', encoding='utf-8') as f:
            json.dump(default_config, f, indent=2, ensure_ascii=False)
        print(f"已建立默認配置檔案: {self.config_file}")
        return default_config

    def connect_tcp_server(self) -> bool:
        """連接主Modbus TCP服務器"""
        try:
            if self.tcp_client:
                self.tcp_client.close()
            
            host = self.config['tcp_server']['host']
            port = self.config['tcp_server']['port']
            timeout = self.config['tcp_server']['timeout']
            
            self.tcp_client = ModbusTcpClient(
                host=host,
                port=port,
                timeout=timeout
            )
            
            if self.tcp_client.connect():
                self.tcp_connected = True
                print(f"已連接主服務器: {host}:{port}")
                return True
            else:
                self.tcp_connected = False
                return False
                
        except Exception as e:
            print(f"連接主服務器失敗: {e}")
            self.tcp_connected = False
            return False

    def connect_rtu_device(self) -> bool:
        """連接RTU設備"""
        try:
            if self.rtu_client:
                self.rtu_client.close()
            
            rtu_config = self.config['rtu_connection']
            self.rtu_client = ModbusSerialClient(
                port=rtu_config['port'],
                baudrate=rtu_config['baudrate'],
                timeout=rtu_config['timeout'],
                stopbits=rtu_config['stopbits'],
                bytesize=rtu_config['bytesize'],
                parity=rtu_config['parity']
            )
            
            if self.rtu_client.connect():
                self.rtu_connected = True
                print(f"已連接RTU設備: {rtu_config['port']}")
                self.clear_rtu_buffer()
                return True
            else:
                self.rtu_connected = False
                return False
                
        except Exception as e:
            print(f"連接RTU設備失敗: {e}")
            self.rtu_connected = False
            return False

    def clear_rtu_buffer(self):
        """清理RTU接收緩衝區"""
        try:
            if hasattr(self.rtu_client, 'socket') and self.rtu_client.socket:
                # 清空接收緩衝區
                if hasattr(self.rtu_client.socket, 'reset_input_buffer'):
                    self.rtu_client.socket.reset_input_buffer()
                if hasattr(self.rtu_client.socket, 'reset_output_buffer'):
                    self.rtu_client.socket.reset_output_buffer()
        except Exception as e:
            print(f"清理RTU緩衝區錯誤: {e}")

    def test_device_connection(self, device_name: str, unit_id: int) -> bool:
        """測試特定設備連接"""
        if not self.rtu_connected:
            return False
        
        try:
            self.clear_rtu_buffer()
            time.sleep(0.01)
            
            # 讀取單個寄存器測試連接
            result = self.rtu_client.read_holding_registers(
                address=0,
                count=1,
                slave=unit_id
            )
            
            if not result.isError():
                return True
            else:
                return False
                
        except Exception as e:
            print(f"測試設備 {device_name} (ID:{unit_id}) 連接失敗: {e}")
            return False

    def read_device_status(self, device_name: str, unit_id: int) -> dict:
        """讀取設備狀態"""
        status = {
            'connected': False,
            'status': 0,
            'position': 0,
            'error_code': 0
        }
        
        if not self.rtu_connected:
            return status
        
        try:
            self.clear_rtu_buffer()
            time.sleep(0.01)
            
            # 讀取設備基本狀態寄存器
            result = self.rtu_client.read_holding_registers(
                address=0,
                count=10,
                slave=unit_id
            )
            
            if not result.isError():
                registers = result.registers
                status['connected'] = True
                status['status'] = registers[0] if len(registers) > 0 else 0
                status['position'] = registers[1] if len(registers) > 1 else 0
                status['error_code'] = registers[2] if len(registers) > 2 else 0
            
        except Exception as e:
            print(f"讀取設備 {device_name} 狀態失敗: {e}")
        
        return status

    def update_device_status_to_server(self, device_name: str, status_data: dict):
        """更新設備狀態到主服務器"""
        if not self.tcp_connected:
            return
        
        try:
            mapping = self.device_mapping[device_name]
            base_addr = mapping['status_base']
            
            # 準備狀態寄存器數據
            status_registers = [
                1 if status_data['connected'] else 0,  # 連接狀態
                status_data['status'],                  # 設備狀態
                status_data['position'],                # 位置
                status_data['error_code'],              # 錯誤代碼
                int(time.time()) & 0xFFFF               # 時間戳
            ]
            
            # 寫入狀態寄存器
            result = self.tcp_client.write_registers(
                address=base_addr,
                values=status_registers,
                slave=1
            )
            
            if result.isError():
                print(f"更新 {device_name} 狀態到服務器失敗")
                
        except Exception as e:
            print(f"更新 {device_name} 狀態錯誤: {e}")

    def process_device_commands(self, device_name: str, unit_id: int):
        """處理設備指令"""
        if not self.tcp_connected:
            return
        
        try:
            mapping = self.device_mapping[device_name]
            command_addr = mapping['command_base']
            
            # 讀取指令寄存器
            result = self.tcp_client.read_holding_registers(
                address=command_addr,
                count=5,
                slave=1
            )
            
            if result.isError():
                return
            
            registers = result.registers
            command_code = registers[0]
            param1 = registers[1]
            param2 = registers[2]
            command_id = registers[3]
            
            # 檢查是否有新指令
            last_id = self.last_command_ids.get(device_name, 0)
            if command_id != 0 and command_id != last_id:
                self.last_command_ids[device_name] = command_id
                
                # 執行指令
                success = self.execute_device_command(device_name, unit_id, command_code, param1, param2)
                
                # 清除指令寄存器
                self.tcp_client.write_registers(
                    address=command_addr,
                    values=[0, 0, 0, 0, 0],
                    slave=1
                )
                
        except Exception as e:
            print(f"處理 {device_name} 指令錯誤: {e}")

    def execute_device_command(self, device_name: str, unit_id: int, command: int, param1: int, param2: int) -> bool:
        """執行設備指令"""
        if not self.rtu_connected:
            return False
        
        try:
            self.clear_rtu_buffer()
            time.sleep(0.01)
            
            print(f"執行 {device_name} 指令: {command}, 參數: {param1}, {param2}")
            
            success = False
            
            if command == 1:  # 開啟設備
                result = self.rtu_client.write_register(
                    address=0,
                    value=1,
                    slave=unit_id
                )
                success = not result.isError()
                
            elif command == 2:  # 關閉設備
                result = self.rtu_client.write_register(
                    address=0,
                    value=0,
                    slave=unit_id
                )
                success = not result.isError()
                
            elif command == 3:  # 移動到位置
                result = self.rtu_client.write_register(
                    address=1,
                    value=param1,
                    slave=unit_id
                )
                success = not result.isError()
                
            elif command == 4:  # 緊急停止
                result = self.rtu_client.write_register(
                    address=2,
                    value=1,
                    slave=unit_id
                )
                success = not result.isError()
                
            return success
            
        except Exception as e:
            print(f"執行 {device_name} 指令失敗: {e}")
            return False

    def main_loop(self):
        """主循環"""
        print("RS485Gateway主循環啟動")
        loop_count = 0
        
        while self.running:
            try:
                # 檢查連接狀態
                if not self.tcp_connected:
                    self.connect_tcp_server()
                    time.sleep(1)
                    continue
                
                if not self.rtu_connected:
                    self.connect_rtu_device()
                    time.sleep(1)
                    continue
                
                # 每10個循環測試一次設備連接
                if loop_count % self.config['timing']['device_test_interval'] == 0:
                    for device_name, mapping in self.device_mapping.items():
                        unit_id = mapping['unit_id']
                        
                        # 測試設備連接並讀取狀態
                        is_connected = self.test_device_connection(device_name, unit_id)
                        if is_connected:
                            status_data = self.read_device_status(device_name, unit_id)
                        else:
                            status_data = {
                                'connected': False,
                                'status': 0,
                                'position': 0,
                                'error_code': 0
                            }
                        
                        # 更新狀態到主服務器
                        self.update_device_status_to_server(device_name, status_data)
                        
                        # 處理指令
                        self.process_device_commands(device_name, unit_id)
                        
                        time.sleep(0.02)  # 設備間延遲
                
                loop_count += 1
                time.sleep(self.config['timing']['fast_loop_interval'])
                
            except Exception as e:
                print(f"主循環錯誤: {e}")
                time.sleep(1)

    def start(self):
        """啟動網關"""
        if self.running:
            return
        
        self.running = True
        
        # 連接服務器和設備
        self.connect_tcp_server()
        self.connect_rtu_device()
        
        # 啟動主循環線程
        self.loop_thread = threading.Thread(target=self.main_loop, daemon=True)
        self.loop_thread.start()
        
        print("RS485Gateway已啟動")

    def stop(self):
        """停止網關"""
        self.running = False
        
        if self.loop_thread:
            self.loop_thread.join(timeout=3)
        
        if self.tcp_client:
            self.tcp_client.close()
        
        if self.rtu_client:
            self.rtu_client.close()
        
        print("RS485Gateway已停止")

def main():
    gateway = RS485Gateway()
    
    try:
        gateway.start()
        
        # 保持運行
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n收到停止信號")
    finally:
        gateway.stop()

if __name__ == "__main__":
    main()