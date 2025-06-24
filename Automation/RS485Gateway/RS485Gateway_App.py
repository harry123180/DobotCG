#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RS485Gateway_App.py - RS485網關監控應用 (修正版)
移除XC100映射，專注於三款夾爪監控
"""

import os
import json
import time
import threading
from datetime import datetime
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

class RS485GatewayApp:
    """RS485網關監控應用"""
    
    def __init__(self, config_file="rs485_gateway_app_config.json"):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # Flask應用
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'rs485_gateway_secret_key'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Modbus客戶端
        self.modbus_client = None
        self.is_connected = False
        
        # 監控線程控制
        self.monitoring_active = False
        self.monitor_thread = None
        
        # 修正的設備映射 - 移除XC100
        self.device_mapping = {
            'PGC': {'status_base': 500, 'command_base': 520, 'unit_id': 6},
            'PGHL': {'status_base': 530, 'command_base': 550, 'unit_id': 5},
            'PGE': {'status_base': 560, 'command_base': 580, 'unit_id': 4}
        }
        
        # 設備狀態
        self.device_status = {
            'PGC': {
                'connected': False,
                'init_status': 0,
                'grip_status': 0,
                'position': 0,
                'error_count': 0
            },
            'PGHL': {
                'connected': False,
                'home_status': 0,
                'running_status': 0,
                'position': 0,
                'current': 0,
                'error_count': 0
            },
            'PGE': {
                'connected': False,
                'init_status': 0,
                'grip_status': 0,
                'position': 0,
                'error_count': 0
            }
        }
        
        # 網關統計
        self.gateway_stats = {
            'start_time': datetime.now(),
            'total_commands': 0,
            'successful_commands': 0,
            'failed_commands': 0,
            'communication_errors': 0
        }
        
        # 設置路由
        self.setup_routes()
        self.setup_socketio()
        
        print("RS485Gateway監控應用啟動中...")
        print(f"Modbus服務器地址: {self.config['modbus_tcp']['host']}:{self.config['modbus_tcp']['port']}")
        print("設備寄存器映射 (修正版):")
        for device, mapping in self.device_mapping.items():
            print(f"  {device} (unit_id={mapping['unit_id']}): 狀態 {mapping['status_base']}-{mapping['status_base']+19}, 指令 {mapping['command_base']}-{mapping['command_base']+9}")

    def load_config(self):
        """載入配置"""
        default_config = {
            "module_id": "RS485Gateway監控",
            "modbus_tcp": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "web_server": {
                "host": "0.0.0.0",
                "port": 5055,
                "debug": False
            },
            "ui_settings": {
                "refresh_interval": 2.0,
                "manual_mode": False,
                "show_debug_info": True
            },
            "devices": {
                "PGC": {
                    "name": "PGC夾爪",
                    "enabled": True,
                    "color": "#38a169"
                },
                "PGHL": {
                    "name": "PGHL夾爪",
                    "enabled": True,
                    "color": "#d69e2e"
                },
                "PGE": {
                    "name": "PGE夾爪",
                    "enabled": True,
                    "color": "#e53e3e"
                }
            }
        }
        
        if os.path.exists(self.config_file):
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
            except Exception as e:
                print(f"配置檔案讀取錯誤: {e}")
                return default_config
        else:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(default_config, f, indent=2, ensure_ascii=False)
            return default_config

    def connect_modbus(self):
        """連接Modbus服務器"""
        try:
            if self.modbus_client and self.modbus_client.connected:
                return True
                
            self.modbus_client = ModbusTcpClient(
                host=self.config["modbus_tcp"]["host"],
                port=self.config["modbus_tcp"]["port"],
                timeout=self.config["modbus_tcp"]["timeout"]
            )
            
            if self.modbus_client.connect():
                self.is_connected = True
                print(f"已連接Modbus服務器: {self.config['modbus_tcp']['host']}:{self.config['modbus_tcp']['port']}")
                return True
            else:
                self.is_connected = False
                return False
        except Exception as e:
            print(f"Modbus連接錯誤: {e}")
            self.is_connected = False
            return False

    def read_device_status(self, device_name):
        """讀取設備狀態"""
        try:
            if not self.is_connected:
                return None
                
            status_base = self.device_mapping[device_name]['status_base']
            unit_id = self.config["modbus_tcp"]["unit_id"]
            
            result = self.modbus_client.read_holding_registers(
                address=status_base,
                count=20,
                slave=unit_id
            )
            
            if result.isError():
                return None
                
            registers = result.registers
            
            # 夾爪狀態解析
            status_mappings = {
                'init_status': {0: "未初始化", 1: "成功", 2: "進行中"},
                'grip_status': {0: "運動中", 1: "到達", 2: "夾住", 3: "掉落"},
                'home_status': {0: "未初始化", 1: "成功", 2: "進行中"},
                'running_status': {0: "運動中", 1: "到達", 2: "堵轉", 3: "掉落"}
            }
            
            status_data = {
                'connected': registers[1] == 1,
                'device_status': registers[2],
                'error_count': registers[3],
                'grip_status': registers[4],
                'position': registers[5],
                'timestamp': registers[14] if len(registers) > 14 else 0
            }
            
            if device_name == 'PGHL':
                status_data['current'] = registers[6] if len(registers) > 6 else 0
                status_data['home_status'] = status_data['device_status']
                status_data['running_status'] = status_data['grip_status']
                status_data['home_status_text'] = status_mappings['home_status'].get(status_data['home_status'], str(status_data['home_status']))
                status_data['running_status_text'] = status_mappings['running_status'].get(status_data['running_status'], str(status_data['running_status']))
            else:
                status_data['init_status'] = status_data['device_status']
                status_data['init_status_text'] = status_mappings['init_status'].get(status_data['init_status'], str(status_data['init_status']))
                status_data['grip_status_text'] = status_mappings['grip_status'].get(status_data['grip_status'], str(status_data['grip_status']))
            
            return status_data
            
        except Exception as e:
            print(f"讀取{device_name}狀態錯誤: {e}")
            self.gateway_stats['communication_errors'] += 1
            return None

    def send_device_command(self, device_name, command, param1=0, param2=0):
        """發送設備指令"""
        try:
            if not self.is_connected:
                return False
                
            command_base = self.device_mapping[device_name]['command_base']
            unit_id = self.config["modbus_tcp"]["unit_id"]
            command_id = int(time.time() * 1000) % 65535  # 生成唯一指令ID
            
            values = [command, param1, param2, command_id, 0, 0, 0, 0, 0, 0]
            
            result = self.modbus_client.write_registers(
                address=command_base,
                values=values,
                slave=unit_id
            )
            
            self.gateway_stats['total_commands'] += 1
            
            if not result.isError():
                self.gateway_stats['successful_commands'] += 1
                return True
            else:
                self.gateway_stats['failed_commands'] += 1
                return False
            
        except Exception as e:
            print(f"發送{device_name}指令錯誤: {e}")
            self.gateway_stats['failed_commands'] += 1
            self.gateway_stats['communication_errors'] += 1
            return False

    def setup_routes(self):
        """設置Flask路由"""
        
        @self.app.route('/')
        def index():
            """主頁"""
            return render_template('gateway_index.html', 
                                 config=self.config,
                                 device_mapping=self.device_mapping)

        @self.app.route('/api/connect', methods=['POST'])
        def connect():
            """連接API"""
            success = self.connect_modbus()
            return jsonify({
                'success': success,
                'message': '連接成功' if success else '連接失敗'
            })

        @self.app.route('/api/status')
        def get_status():
            """獲取所有設備狀態"""
            if not self.is_connected:
                return jsonify({'success': False, 'message': '未連接到服務器'})
            
            all_status = {}
            for device_name in ['PGC', 'PGHL', 'PGE']:
                if self.config["devices"][device_name]["enabled"]:
                    status = self.read_device_status(device_name)
                    all_status[device_name] = status
            
            # 計算成功率
            success_rate = 0
            if self.gateway_stats['total_commands'] > 0:
                success_rate = (self.gateway_stats['successful_commands'] / self.gateway_stats['total_commands']) * 100
            
            uptime = datetime.now() - self.gateway_stats['start_time']
            
            return jsonify({
                'success': True,
                'connected': self.is_connected,
                'devices': all_status,
                'statistics': {
                    'uptime': str(uptime).split('.')[0],
                    'total_commands': self.gateway_stats['total_commands'],
                    'successful_commands': self.gateway_stats['successful_commands'],
                    'failed_commands': self.gateway_stats['failed_commands'],
                    'success_rate': f'{success_rate:.1f}%',
                    'communication_errors': self.gateway_stats['communication_errors']
                },
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            })

        @self.app.route('/api/command/<device_name>', methods=['POST'])
        def send_command(device_name):
            """發送設備指令API"""
            if device_name not in self.device_mapping:
                return jsonify({'error': '無效的設備名稱'}), 400
                
            data = request.get_json()
            command = data.get('command', 0)
            param1 = data.get('param1', 0)
            param2 = data.get('param2', 0)
            
            success = self.send_device_command(device_name, command, param1, param2)
            return jsonify({
                'success': success,
                'message': f'{device_name}指令發送成功' if success else '指令發送失敗'
            })

        @self.app.route('/api/emergency_stop', methods=['POST'])
        def emergency_stop():
            """緊急停止所有設備"""
            results = {}
            for device_name in ['PGC', 'PGHL', 'PGE']:
                if self.config["devices"][device_name]["enabled"]:
                    results[device_name] = self.send_device_command(device_name, 2)  # 停止指令
            
            all_success = all(results.values())
            return jsonify({
                'success': all_success,
                'results': results,
                'message': '緊急停止指令已發送' if all_success else '部分設備停止失敗'
            })

        @self.app.route('/api/reset_stats', methods=['POST'])
        def reset_stats():
            """重置統計"""
            self.gateway_stats = {
                'start_time': datetime.now(),
                'total_commands': 0,
                'successful_commands': 0,
                'failed_commands': 0,
                'communication_errors': 0
            }
            return jsonify({'success': True, 'message': '統計已重置'})

        @self.app.route('/api/device_details/<device_name>')
        def get_device_details(device_name):
            """獲取設備詳細資訊"""
            if device_name not in self.device_mapping:
                return jsonify({'error': '無效的設備名稱'}), 400
            
            status = self.read_device_status(device_name)
            return jsonify({
                'success': status is not None,
                'device': device_name,
                'status': status,
                'config': self.config["devices"][device_name]
            })

        # 新增夾爪專用API
        @self.app.route('/api/initialize/<device_name>', methods=['POST'])
        def initialize_gripper(device_name):
            """初始化夾爪"""
            if device_name not in self.device_mapping:
                return jsonify({'error': '無效的設備名稱'}), 400
            
            success = self.send_device_command(device_name, 1)  # 初始化指令
            return jsonify({
                'success': success,
                'message': f'{device_name}初始化指令已發送' if success else '初始化指令發送失敗'
            })

        @self.app.route('/api/move/<device_name>', methods=['POST'])
        def move_gripper(device_name):
            """移動夾爪到指定位置"""
            if device_name not in self.device_mapping:
                return jsonify({'error': '無效的設備名稱'}), 400
            
            data = request.get_json()
            position = data.get('position', 0)
            
            success = self.send_device_command(device_name, 3, position)  # 絕對位置指令
            return jsonify({
                'success': success,
                'message': f'{device_name}移動指令已發送 (位置:{position})' if success else '移動指令發送失敗'
            })

        @self.app.route('/api/set_force/<device_name>', methods=['POST'])
        def set_force(device_name):
            """設定夾爪力道"""
            if device_name not in self.device_mapping:
                return jsonify({'error': '無效的設備名稱'}), 400
            
            data = request.get_json()
            force = data.get('force', 50)
            
            success = self.send_device_command(device_name, 5, force)  # 設定力道指令
            return jsonify({
                'success': success,
                'message': f'{device_name}力道設定已發送 (力道:{force})' if success else '力道設定失敗'
            })

        @self.app.route('/api/open/<device_name>', methods=['POST'])
        def open_gripper(device_name):
            """開啟夾爪"""
            if device_name not in self.device_mapping:
                return jsonify({'error': '無效的設備名稱'}), 400
            
            success = self.send_device_command(device_name, 7)  # 開啟指令
            return jsonify({
                'success': success,
                'message': f'{device_name}開啟指令已發送' if success else '開啟指令發送失敗'
            })

        @self.app.route('/api/close/<device_name>', methods=['POST'])
        def close_gripper(device_name):
            """關閉夾爪"""
            if device_name not in self.device_mapping:
                return jsonify({'error': '無效的設備名稱'}), 400
            
            success = self.send_device_command(device_name, 8)  # 關閉指令
            return jsonify({
                'success': success,
                'message': f'{device_name}關閉指令已發送' if success else '關閉指令發送失敗'
            })

    def setup_socketio(self):
        """設置SocketIO事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            """客戶端連接"""
            print('Web客戶端已連接')
            emit('status', {'connected': True})

        @self.socketio.on('disconnect')
        def handle_disconnect():
            """客戶端斷開"""
            print('Web客戶端已斷開')

        @self.socketio.on('start_monitoring')
        def handle_start_monitoring():
            """啟動監控"""
            self.start_monitoring()
            emit('monitoring_status', {'active': True})

        @self.socketio.on('stop_monitoring')
        def handle_stop_monitoring():
            """停止監控"""
            self.stop_monitoring()
            emit('monitoring_status', {'active': False})

        @self.socketio.on('request_status')
        def handle_request_status():
            """請求狀態更新"""
            self.emit_status_update()

    def start_monitoring(self):
        """啟動狀態監控"""
        if not self.monitoring_active:
            self.monitoring_active = True
            self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
            self.monitor_thread.start()
            print("狀態監控已啟動")

    def stop_monitoring(self):
        """停止狀態監控"""
        self.monitoring_active = False
        print("狀態監控已停止")

    def monitor_loop(self):
        """監控循環"""
        while self.monitoring_active:
            try:
                if self.connect_modbus():
                    self.emit_status_update()
                time.sleep(self.config["ui_settings"]["refresh_interval"])
            except Exception as e:
                print(f"監控循環錯誤: {e}")
                time.sleep(1)

    def emit_status_update(self):
        """發送狀態更新"""
        try:
            all_status = {}
            for device_name in ['PGC', 'PGHL', 'PGE']:
                if self.config["devices"][device_name]["enabled"]:
                    status = self.read_device_status(device_name)
                    all_status[device_name] = status
            
            uptime = datetime.now() - self.gateway_stats['start_time']
            success_rate = 0
            if self.gateway_stats['total_commands'] > 0:
                success_rate = (self.gateway_stats['successful_commands'] / self.gateway_stats['total_commands']) * 100
            
            self.socketio.emit('status_update', {
                'timestamp': time.time(),
                'connected': self.is_connected,
                'devices': all_status,
                'statistics': {
                    'uptime': str(uptime).split('.')[0],
                    'success_rate': f'{success_rate:.1f}%',
                    'total_commands': self.gateway_stats['total_commands'],
                    'communication_errors': self.gateway_stats['communication_errors']
                }
            })
            
        except Exception as e:
            print(f"狀態更新錯誤: {e}")

    def run(self):
        """啟動Web應用"""
        print(f"Web服務器啟動 - http://{self.config['web_server']['host']}:{self.config['web_server']['port']}")
        
        # 嘗試連接Modbus
        self.connect_modbus()
        
        # 啟動狀態監控
        if not self.config["ui_settings"]["manual_mode"]:
            self.start_monitoring()
        
        self.socketio.run(
            self.app,
            host=self.config["web_server"]["host"],
            port=self.config["web_server"]["port"],
            debug=self.config["web_server"]["debug"]
        )

if __name__ == "__main__":
    app = RS485GatewayApp()
    app.run()