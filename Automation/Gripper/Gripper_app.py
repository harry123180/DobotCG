# Gripper_app.py - 夾爪Web控制應用
import os
import json
import time
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

class GripperWebApp:
    def __init__(self, config_file="gripper_app_config.json"):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # Flask應用
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'gripper_secret_key'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Modbus客戶端
        self.modbus_client = None
        self.is_connected = False
        
        # 監控線程控制
        self.monitoring_active = False
        self.monitor_thread = None
        
        # 寄存器映射
        self.register_mapping = {
            'PGC': {'status_base': 500, 'command_base': 520},
            'PGHL': {'status_base': 530, 'command_base': 550},
            'PGE': {'status_base': 560, 'command_base': 580}
        }
        
        # 設置路由
        self.setup_routes()
        self.setup_socketio()
        
        print("夾爪Web控制應用啟動中...")
        print(f"Modbus服務器地址: {self.config['modbus_tcp']['host']}:{self.config['modbus_tcp']['port']}")
        print("夾爪寄存器映射:")
        for gripper, mapping in self.register_mapping.items():
            print(f"  {gripper}: 狀態 {mapping['status_base']}-{mapping['status_base']+19}, 指令 {mapping['command_base']}-{mapping['command_base']+9}")

    def load_config(self):
        default_config = {
            "module_id": "夾爪Web UI",
            "modbus_tcp": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "web_server": {
                "host": "0.0.0.0",
                "port": 5054,
                "debug": False
            },
            "ui_settings": {
                "refresh_interval": 3.0,
                "manual_mode": False
            },
            "grippers": {
                "PGC": {
                    "name": "PGC夾爪",
                    "enabled": True,
                    "positions": {"open": 1000, "close": 0},
                    "max_force": 100,
                    "max_speed": 100
                },
                "PGHL": {
                    "name": "PGHL夾爪", 
                    "enabled": True,
                    "positions": {"open": 5000, "close": 0},
                    "max_force": 100,
                    "max_speed": 100
                },
                "PGE": {
                    "name": "PGE夾爪",
                    "enabled": True,
                    "positions": {"open": 1000, "close": 0},
                    "max_force": 100,
                    "max_speed": 100
                }
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                print(f"配置檔案讀取錯誤: {e}")
                return default_config
        else:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(default_config, f, indent=2, ensure_ascii=False)
            return default_config

    def connect_modbus(self):
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

    def read_gripper_status(self, gripper_type):
        """讀取夾爪狀態"""
        try:
            if not self.is_connected:
                return None
                
            status_base = self.register_mapping[gripper_type]['status_base']
            
            result = self.modbus_client.read_holding_registers(
                address=status_base,
                count=20,
                slave=self.config["modbus_tcp"]["unit_id"]
            )
            
            if result.isError():
                return None
                
            registers = result.registers
            
            status_data = {
                'module_status': registers[0],
                'connected': bool(registers[1]),
                'device_status': registers[2],
                'error_count': registers[3],
                'grip_status': registers[4],
                'position': registers[5],
                'current': registers[6] if len(registers) > 6 else 0,
                'timestamp': registers[14] if len(registers) > 14 else 0
            }
            
            return status_data
            
        except Exception as e:
            print(f"讀取{gripper_type}狀態錯誤: {e}")
            return None

    def send_gripper_command(self, gripper_type, command, param1=0, param2=0):
        """發送夾爪指令"""
        try:
            if not self.is_connected:
                return False
                
            command_base = self.register_mapping[gripper_type]['command_base']
            command_id = int(time.time() * 1000) % 65535  # 生成唯一指令ID
            
            values = [command, param1, param2, command_id, 0, 0, 0, 0, 0, 0]
            
            result = self.modbus_client.write_registers(
                address=command_base,
                values=values,
                slave=self.config["modbus_tcp"]["unit_id"]
            )
            
            return not result.isError() if result else False
            
        except Exception as e:
            print(f"發送{gripper_type}指令錯誤: {e}")
            return False

    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html', 
                                 config=self.config,
                                 register_mapping=self.register_mapping)

        @self.app.route('/api/connect', methods=['POST'])
        def connect():
            success = self.connect_modbus()
            return jsonify({
                'success': success,
                'message': '連接成功' if success else '連接失敗'
            })

        @self.app.route('/api/status/<gripper_type>')
        def get_status(gripper_type):
            if gripper_type not in self.register_mapping:
                return jsonify({'error': '無效的夾爪類型'}), 400
                
            status = self.read_gripper_status(gripper_type)
            return jsonify({
                'success': status is not None,
                'status': status,
                'connected': self.is_connected
            })

        @self.app.route('/api/command/<gripper_type>', methods=['POST'])
        def send_command(gripper_type):
            if gripper_type not in self.register_mapping:
                return jsonify({'error': '無效的夾爪類型'}), 400
                
            data = request.get_json()
            command = data.get('command', 0)
            param1 = data.get('param1', 0)
            param2 = data.get('param2', 0)
            
            success = self.send_gripper_command(gripper_type, command, param1, param2)
            return jsonify({
                'success': success,
                'message': '指令發送成功' if success else '指令發送失敗'
            })

        @self.app.route('/api/initialize/<gripper_type>', methods=['POST'])
        def initialize_gripper(gripper_type):
            success = self.send_gripper_command(gripper_type, 1)  # 指令1: 初始化
            return jsonify({
                'success': success,
                'message': f'{gripper_type}初始化指令已發送' if success else '指令發送失敗'
            })

        @self.app.route('/api/stop/<gripper_type>', methods=['POST'])
        def stop_gripper(gripper_type):
            success = self.send_gripper_command(gripper_type, 2)  # 指令2: 停止
            return jsonify({
                'success': success,
                'message': f'{gripper_type}停止指令已發送' if success else '指令發送失敗'
            })

        @self.app.route('/api/move/<gripper_type>', methods=['POST'])
        def move_gripper(gripper_type):
            data = request.get_json()
            position = data.get('position', 0)
            
            success = self.send_gripper_command(gripper_type, 3, position)  # 指令3: 絕對位置
            return jsonify({
                'success': success,
                'message': f'{gripper_type}移動指令已發送' if success else '指令發送失敗'
            })

        @self.app.route('/api/set_force/<gripper_type>', methods=['POST'])
        def set_force(gripper_type):
            data = request.get_json()
            force = data.get('force', 50)
            
            success = self.send_gripper_command(gripper_type, 5, force)  # 指令5: 設定力道
            return jsonify({
                'success': success,
                'message': f'{gripper_type}力道設定已發送' if success else '指令發送失敗'
            })

        @self.app.route('/api/set_speed/<gripper_type>', methods=['POST'])
        def set_speed(gripper_type):
            data = request.get_json()
            speed = data.get('speed', 50)
            
            success = self.send_gripper_command(gripper_type, 6, speed)  # 指令6: 設定速度
            return jsonify({
                'success': success,
                'message': f'{gripper_type}速度設定已發送' if success else '指令發送失敗'
            })

        @self.app.route('/api/open/<gripper_type>', methods=['POST'])
        def open_gripper(gripper_type):
            success = self.send_gripper_command(gripper_type, 7)  # 指令7: 開啟
            return jsonify({
                'success': success,
                'message': f'{gripper_type}開啟指令已發送' if success else '指令發送失敗'
            })

        @self.app.route('/api/close/<gripper_type>', methods=['POST'])
        def close_gripper(gripper_type):
            success = self.send_gripper_command(gripper_type, 8)  # 指令8: 關閉
            return jsonify({
                'success': success,
                'message': f'{gripper_type}關閉指令已發送' if success else '指令發送失敗'
            })

    def setup_socketio(self):
        @self.socketio.on('connect')
        def handle_connect():
            print('客戶端已連接')
            emit('status', {'connected': True})

        @self.socketio.on('disconnect')
        def handle_disconnect():
            print('客戶端已斷開')

        @self.socketio.on('start_monitoring')
        def handle_start_monitoring():
            self.start_monitoring()
            emit('monitoring_status', {'active': True})

        @self.socketio.on('stop_monitoring')
        def handle_stop_monitoring():
            self.stop_monitoring()
            emit('monitoring_status', {'active': False})

        @self.socketio.on('request_status')
        def handle_request_status():
            self.emit_all_status()

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
                    self.emit_all_status()
                time.sleep(self.config["ui_settings"]["refresh_interval"])
            except Exception as e:
                print(f"監控循環錯誤: {e}")
                time.sleep(1)

    def emit_all_status(self):
        """發送所有夾爪狀態"""
        try:
            status_data = {}
            
            for gripper_type in ['PGC', 'PGHL', 'PGE']:
                if self.config["grippers"][gripper_type]["enabled"]:
                    status = self.read_gripper_status(gripper_type)
                    status_data[gripper_type] = status
            
            self.socketio.emit('status_update', {
                'timestamp': time.time(),
                'connected': self.is_connected,
                'grippers': status_data
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
    app = GripperWebApp()
    app.run()