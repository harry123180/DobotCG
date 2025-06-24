import time
import threading
import json
import os
from typing import Dict, Any

# PyModbus imports
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# Flask imports
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# 設置Flask應用
app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'angle_adjustment_web_app'
socketio = SocketIO(app, cors_allowed_origins="*")

class AngleAppService:
    """角度調整Web應用服務"""
    def __init__(self):
        # 基本配置
        self.base_address = 700
        self.ccd3_base_address = 800
        
        # Modbus TCP 連接
        self.modbus_client = None
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        
        # 狀態監控
        self.monitoring = False
        self.monitor_thread = None
        
        # 配置檔案
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'angle_app_config.json')
        self.load_config()
    
    def load_config(self):
        """載入配置檔案"""
        default_config = {
            "module_id": "Angle_Adjustment_Web_App",
            "modbus_tcp": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 3.0
            },
            "web_server": {
                "host": "localhost",
                "port": 5087,  # 修正為5087端口
                "debug": False
            },
            "modbus_mapping": {
                "base_address": 700,
                "ccd3_base_address": 800
            },
            "ui_settings": {
                "refresh_interval": 1.0,  # 改為1秒更新
                "auto_refresh": True
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
            self.refresh_interval = config['ui_settings']['refresh_interval']
            
        except Exception as e:
            print(f"配置檔案載入錯誤: {e}")
            self.refresh_interval = 1.0  # 預設1秒
    
    def connect_modbus(self) -> bool:
        """連接Modbus TCP服務器"""
        try:
            print(f"正在連接Modbus TCP服務器: {self.server_ip}:{self.server_port}")
            
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3
            )
            
            if self.modbus_client.connect():
                print(f"角度調整Web應用已連接到Modbus服務器")
                self.start_monitoring()
                return True
            else:
                print(f"Modbus連接失敗")
                return False
                
        except Exception as e:
            print(f"Modbus連接錯誤: {e}")
            return False
    
    def disconnect_modbus(self):
        """斷開Modbus連接"""
        self.stop_monitoring()
        if self.modbus_client:
            self.modbus_client.close()
            self.modbus_client = None
            print("Modbus連接已斷開")
    
    def read_status_registers(self) -> Dict[str, Any]:
        """讀取狀態寄存器"""
        status = {
            'connected': False,
            'status_register': 0,
            'modbus_connected': False,
            'motor_connected': False,
            'error_code': 0,
            'operation_count': 0,
            'error_count': 0,
            'ready': False,
            'running': False,
            'alarm': False,
            'initialized': False,
            'ccd_detecting': False,
            'motor_moving': False
        }
        
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                return status
            
            # 讀取狀態寄存器 (700-714)
            result = self.modbus_client.read_holding_registers(
                address=self.base_address, count=15, slave=1
            )
            
            if result.isError():
                return status
            
            registers = result.registers
            status['connected'] = True
            
            # 解析狀態寄存器
            status_register = registers[0]
            status['status_register'] = status_register
            status['ready'] = bool(status_register & (1 << 0))
            status['running'] = bool(status_register & (1 << 1))
            status['alarm'] = bool(status_register & (1 << 2))
            status['initialized'] = bool(status_register & (1 << 3))
            status['ccd_detecting'] = bool(status_register & (1 << 4))
            status['motor_moving'] = bool(status_register & (1 << 5))
            
            # 其他狀態
            status['modbus_connected'] = bool(registers[1])
            status['motor_connected'] = bool(registers[2])
            status['error_code'] = registers[3]
            
            # 統計資訊 (32位)
            status['operation_count'] = (registers[5] << 16) | registers[4]
            status['error_count'] = registers[6]
            
        except Exception as e:
            print(f"讀取狀態寄存器錯誤: {e}")
            
        return status
    
    def read_result_registers(self) -> Dict[str, Any]:
        """讀取結果寄存器"""
        result = {
            'success': False,
            'original_angle': None,
            'angle_diff': None,
            'motor_position': None,
            'operation_count': 0,
            'error_count': 0,
            'runtime': 0
        }
        
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                return result
            
            # 讀取結果寄存器 (720-739)
            res = self.modbus_client.read_holding_registers(
                address=self.base_address + 20, count=20, slave=1
            )
            
            if res.isError():
                return result
            
            registers = res.registers
            
            # 解析結果
            result['success'] = bool(registers[0])
            
            if result['success']:
                # 原始角度 (32位，保留2位小數)
                angle_int = (registers[1] << 16) | registers[2]
                if angle_int >= 2**31:
                    angle_int -= 2**32
                result['original_angle'] = angle_int / 100.0
                
                # 角度差 (32位，保留2位小數)
                diff_int = (registers[3] << 16) | registers[4]
                if diff_int >= 2**31:
                    diff_int -= 2**32
                result['angle_diff'] = diff_int / 100.0
                
                # 馬達位置 (32位)
                pos_int = (registers[5] << 16) | registers[6]
                if pos_int >= 2**31:
                    pos_int -= 2**32
                result['motor_position'] = pos_int
            
            # 統計資訊
            result['operation_count'] = (registers[11] << 16) | registers[10]
            result['error_count'] = registers[12]
            result['runtime'] = registers[13]
            
        except Exception as e:
            print(f"讀取結果寄存器錯誤: {e}")
            
        return result
    
    def send_command(self, command: int) -> bool:
        """發送控制指令"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                print("Modbus未連接，無法發送指令")
                return False
            
            # 寫入控制指令寄存器 (740)
            result = self.modbus_client.write_register(
                address=self.base_address + 40, value=command, slave=1
            )
            
            if result.isError():
                print(f"發送指令失敗: {command}")
                return False
            
            print(f"指令已發送: {command}")
            return True
            
        except Exception as e:
            print(f"發送指令錯誤: {e}")
            return False
    
    def start_monitoring(self):
        """開始狀態監控"""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            print("狀態監控已啟動")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)
            print("狀態監控已停止")
    
    def _monitor_loop(self):
        """監控循環 - 修正為1秒更新間隔"""
        while self.monitoring:
            try:
                if self.modbus_client and self.modbus_client.connected:
                    status = self.read_status_registers()
                    result = self.read_result_registers()
                    
                    # 通過SocketIO發送狀態更新
                    socketio.emit('status_update', {
                        'status': status,
                        'result': result,
                        'timestamp': time.time()
                    })
                
                time.sleep(self.refresh_interval)  # 使用配置的更新間隔
                
            except Exception as e:
                print(f"監控循環錯誤: {e}")
                time.sleep(2.0)

# 全局服務實例
angle_app_service = AngleAppService()

# Web Routes
@app.route('/')
def index():
    return render_template('angle_index.html')

@app.route('/api/modbus/connect', methods=['POST'])
def connect_modbus():
    data = request.json
    ip = data.get('ip', '127.0.0.1')
    port = data.get('port', 502)
    
    angle_app_service.server_ip = ip
    angle_app_service.server_port = port
    
    success = angle_app_service.connect_modbus()
    if success:
        return jsonify({'success': True, 'message': f'已連接到Modbus服務器 {ip}:{port}'})
    else:
        return jsonify({'success': False, 'message': 'Modbus連接失敗'})

@app.route('/api/modbus/disconnect', methods=['POST'])
def disconnect_modbus():
    angle_app_service.disconnect_modbus()
    return jsonify({'success': True, 'message': 'Modbus連接已斷開'})

@app.route('/api/status', methods=['GET'])
def get_status():
    status = angle_app_service.read_status_registers()
    result = angle_app_service.read_result_registers()
    
    return jsonify({
        'status': status,
        'result': result,
        'timestamp': time.time()
    })

@app.route('/api/command/angle_correction', methods=['POST'])
def angle_correction():
    success = angle_app_service.send_command(1)  # 角度校正指令
    if success:
        # 發送指令後，等待一段時間再自動清零，允許重複執行
        import threading
        def auto_clear_command():
            import time
            time.sleep(0.5)  # 等待0.5秒讓主程序接收指令
            angle_app_service.send_command(0)  # 清零指令
        
        threading.Thread(target=auto_clear_command, daemon=True).start()
        
        return jsonify({'success': True, 'message': '角度校正指令已發送'})
    else:
        return jsonify({'success': False, 'message': '指令發送失敗'})

@app.route('/api/command/motor_reset', methods=['POST'])
def motor_reset():
    success = angle_app_service.send_command(2)  # 馬達重置指令
    if success:
        # 發送指令後自動清零
        import threading
        def auto_clear_command():
            import time
            time.sleep(0.5)
            angle_app_service.send_command(0)
        
        threading.Thread(target=auto_clear_command, daemon=True).start()
        
        return jsonify({'success': True, 'message': '馬達重置指令已發送'})
    else:
        return jsonify({'success': False, 'message': '指令發送失敗'})

@app.route('/api/command/error_reset', methods=['POST'])
def error_reset():
    success = angle_app_service.send_command(7)  # 錯誤重置指令
    if success:
        # 發送指令後自動清零
        import threading
        def auto_clear_command():
            import time
            time.sleep(0.5)
            angle_app_service.send_command(0)
        
        threading.Thread(target=auto_clear_command, daemon=True).start()
        
        return jsonify({'success': True, 'message': '錯誤重置指令已發送'})
    else:
        return jsonify({'success': False, 'message': '指令發送失敗'})

@app.route('/api/command/clear', methods=['POST'])
def clear_command():
    success = angle_app_service.send_command(0)  # 清除指令
    if success:
        return jsonify({'success': True, 'message': '指令已清除'})
    else:
        return jsonify({'success': False, 'message': '指令清除失敗'})

# SocketIO Events
@socketio.on('connect')
def handle_connect():
    print("Web客戶端已連接")
    emit('message', {'data': '角度調整系統Web介面已連接'})

@socketio.on('disconnect')
def handle_disconnect():
    print("Web客戶端已斷開")

@socketio.on('get_status')
def handle_get_status():
    status = angle_app_service.read_status_registers()
    result = angle_app_service.read_result_registers()
    
    emit('status_update', {
        'status': status,
        'result': result,
        'timestamp': time.time()
    })

@socketio.on('send_command')
def handle_send_command(data):
    command = data.get('command', 0)
    success = angle_app_service.send_command(command)
    
    emit('command_response', {
        'success': success,
        'command': command,
        'message': f'指令 {command} {"已發送" if success else "發送失敗"}'
    })

if __name__ == '__main__':
    print("角度調整系統Web應用啟動中...")
    print(f"Web服務器: http://localhost:5087")  # 修正端口為5087
    print(f"Modbus服務器地址: {angle_app_service.server_ip}:{angle_app_service.server_port}")
    print(f"角度調整模組基地址: {angle_app_service.base_address}")
    print(f"CCD3模組基地址: {angle_app_service.ccd3_base_address}")
    
    try:
        socketio.run(app, host='localhost', port=5087, debug=False)  # 修正端口為5087
    except KeyboardInterrupt:
        print("\n正在關閉角度調整Web應用...")
        angle_app_service.disconnect_modbus()
    except Exception as e:
        print(f"Web應用錯誤: {e}")
        angle_app_service.disconnect_modbus()