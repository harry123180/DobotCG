#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import psutil
import socket
import subprocess
import threading
import time
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import serial.tools.list_ports

# 配置檔案路徑
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

class ConfigManager:
    """配置檔案管理器"""
    
    @staticmethod
    def read_com_port(config_path, module_type):
        """讀取config檔案中的COM口設定"""
        try:
            if not os.path.exists(config_path):
                return None
                
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            com_port_mappings = {
                "Gripper": ['rtu_connection', 'port'],
                "LED": ['serial_connection', 'port'],
                "XC100": ['xc_connection', 'port']
            }
            
            if module_type in com_port_mappings:
                keys = com_port_mappings[module_type]
                data = config
                for key in keys:
                    if key in data:
                        data = data[key]
                    else:
                        return None
                return data
            return None
            
        except Exception as e:
            print(f"讀取配置檔案錯誤: {e}")
            return None
    
    @staticmethod
    def update_com_port(config_path, module_type, new_com):
        """更新config檔案中的COM口設定"""
        try:
            if not os.path.exists(config_path):
                return False
                
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            com_port_mappings = {
                "Gripper": ['rtu_connection', 'port'],
                "LED": ['serial_connection', 'port'],
                "XC100": ['xc_connection', 'port']
            }
            
            if module_type in com_port_mappings:
                keys = com_port_mappings[module_type]
                data = config
                for key in keys[:-1]:
                    if key not in data:
                        data[key] = {}
                    data = data[key]
                data[keys[-1]] = new_com
                
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                return True
            return False
            
        except Exception as e:
            print(f"更新配置檔案錯誤: {e}")
            return False

class ModuleController:
    """單一模組控制器"""
    
    def __init__(self, name, script_path, config_path=None, needs_com=False):
        self.name = name
        self.script_path = script_path
        self.config_path = config_path
        self.needs_com = needs_com
        self.process = None
        
    def start(self, com_port=None):
        """啟動模組"""
        try:
            if self.is_running():
                return False, "模組已運行中"
            
            # 如果需要COM口，先更新配置
            if self.needs_com and com_port and self.config_path:
                module_type = self.name.replace("_app", "")
                success = ConfigManager.update_com_port(self.config_path, module_type, com_port)
                if not success:
                    return False, f"COM口配置更新失敗: {com_port}"
            
            # 啟動程序
            script_full_path = os.path.join(PROJECT_ROOT, self.script_path)
            if not os.path.exists(script_full_path):
                return False, f"腳本檔案不存在: {script_full_path}"
            
            # 設置環境變量來處理Unicode編碼問題
            env = os.environ.copy()
            env['PYTHONIOENCODING'] = 'utf-8'
            
            self.process = subprocess.Popen(
                [sys.executable, script_full_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=os.path.dirname(script_full_path),
                env=env,  # 添加環境變量
                encoding='utf-8',  # 設置編碼
                errors='replace'   # 替換無法編碼的字符
            )
            
            # 等待一小段時間確認啟動成功
            time.sleep(2)  # 增加等待時間
            if self.process.poll() is not None:
                stdout, stderr = self.process.communicate()
                error_msg = stderr if stderr else stdout
                return False, f"程序啟動失敗: {error_msg}"
            
            return True, "模組啟動成功"
            
        except Exception as e:
            return False, f"啟動異常: {str(e)}"
    
    def stop(self):
        """停止模組"""
        try:
            if not self.is_running():
                return True, "模組未運行"
            
            self.process.terminate()
            
            # 等待程序結束
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            
            self.process = None
            return True, "模組停止成功"
            
        except Exception as e:
            return False, f"停止異常: {str(e)}"
    
    def is_running(self):
        """檢查模組是否運行中"""
        if self.process is None:
            return False
        
        return self.process.poll() is None
    
    def get_status(self):
        """獲取模組狀態"""
        if self.is_running():
            return "運行中"
        else:
            return "停止"

class StatusMonitor:
    """狀態監控器"""
    
    def __init__(self, socketio, startup_manager):
        self.socketio = socketio
        self.startup_manager = startup_manager
        self.monitoring = False
        self.monitor_thread = None
    
    def start_monitoring(self):
        """開始狀態監控"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """停止狀態監控"""
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2)
    
    def _monitor_loop(self):
        """監控循環"""
        while self.monitoring:
            try:
                status_data = self._collect_status()
                self.socketio.emit('status_update', status_data)
                time.sleep(2)  # 縮短到2秒間隔，提高響應性
            except Exception as e:
                print(f"狀態監控錯誤: {e}")
                time.sleep(2)
    
    def _collect_status(self):
        """收集所有狀態"""
        # 每次狀態更新時打印摘要
        modbus_running = self._check_modbus_server()
        running_modules = []
        running_webapps = []
        
        status = {
            'modbus_server': modbus_running,
            'modules': {},
            'web_apps': {},
            'com_ports': self._scan_com_ports(),
            'timestamp': time.time()
        }
        
        # 檢查主模組狀態
        for name, controller in self.startup_manager.modules.items():
            is_running = controller.is_running()
            status['modules'][name] = {
                'status': controller.get_status(),
                'running': is_running
            }
            if is_running:
                running_modules.append(name)
        
        # 檢查WebUI狀態
        for name, controller in self.startup_manager.web_apps.items():
            is_running = controller.is_running()
            module_key = name.replace('_app', '')
            port = self.startup_manager.web_ports.get(module_key, 0)
            port_active = self._check_port(port)
            
            status['web_apps'][name] = {
                'status': controller.get_status(),
                'running': is_running,
                'port_active': port_active
            }
            if is_running:
                running_webapps.append(f"{name}({port})")
        
        # 打印狀態摘要（每30秒一次，避免太頻繁）
        current_time = time.time()
        if not hasattr(self, '_last_status_print') or (current_time - self._last_status_print) > 30:
            print(f"\n=== 系統狀態摘要 ===")
            print(f"ModbusTCP服務器: {'✅ 運行中' if modbus_running else '❌ 停止'}")
            print(f"運行中的主模組 ({len(running_modules)}): {', '.join(running_modules) if running_modules else '無'}")
            print(f"運行中的WebUI ({len(running_webapps)}): {', '.join(running_webapps) if running_webapps else '無'}")
            print(f"可用COM口: {', '.join(status['com_ports']) if status['com_ports'] else '無'}")
            print(f"==================")
            self._last_status_print = current_time
        
        return status
    
    def _check_modbus_server(self):
        """檢查ModbusTCP服務器狀態"""
        return self._check_port(502)
    
    def _check_port(self, port):
        """檢查端口是否被佔用"""
        if port == 0:
            return False
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1)
                result = s.connect_ex(('127.0.0.1', port))
                return result == 0
        except Exception:
            return False
    
    def _scan_com_ports(self):
        """掃描可用COM口"""
        try:
            ports = serial.tools.list_ports.comports()
            return [port.device for port in ports]
        except Exception:
            return []

class StartupManager:
    """啟動管理器主類別"""
    
    def __init__(self):
        self.modbus_server = None
        self.modules = {}
        self.web_apps = {}
        self.web_ports = {
            'CCD1': 5051,
            'CCD3': 5052,
            'VP': 5053,
            'Gripper': 5054,
            'XC100': 5007,
            'LED': 5008
        }
        
        self._initialize_modules()
    
    def _initialize_modules(self):
        """初始化模組控制器"""
        # 主模組配置
        module_configs = {
            'CCD1': {
                'script': 'Automation/CCD1/CCD1VisionCode_Enhanced.py',
                'config': None,
                'needs_com': False
            },
            'CCD3': {
                'script': 'Automation/CCD3/CCD3AngleDetection.py',
                'config': None,
                'needs_com': False
            },
            'Gripper': {
                'script': 'Automation/Gripper/Gripper.py',
                'config': 'Automation/Gripper/gripper_config.json',
                'needs_com': True
            },
            'LED': {
                'script': 'Automation/light/LED_main.py',
                'config': 'Automation/light/led_config.json',
                'needs_com': True
            },
            'VP': {
                'script': 'Automation/VP/VP_main.py',
                'config': 'Automation/VP/vp_config.json',
                'needs_com': False
            },
            'XC100': {
                'script': 'Automation/XC100/XCModule.py',
                'config': 'Automation/XC100/xc_module_config.json',
                'needs_com': True
            }
        }
        
        # WebUI模組配置
        web_configs = {
            'Gripper_app': {
                'script': 'Automation/Gripper/Gripper_app.py',
                'config': 'Automation/Gripper/gripper_app_config.json',
                'needs_com': False
            },
            'LED_app': {
                'script': 'Automation/light/LED_app.py',
                'config': None,
                'needs_com': False
            },
            'VP_app': {
                'script': 'Automation/VP/VP_app.py',
                'config': 'Automation/VP/vp_app_config.json',
                'needs_com': False
            },
            'XC_app': {
                'script': 'Automation/XC100/XCApp.py',
                'config': None,
                'needs_com': False
            }
        }
        
        # 創建主模組控制器
        for name, config in module_configs.items():
            config_path = os.path.join(PROJECT_ROOT, config['config']) if config['config'] else None
            self.modules[name] = ModuleController(
                name, config['script'], config_path, config['needs_com']
            )
        
        # 創建WebUI控制器
        for name, config in web_configs.items():
            config_path = os.path.join(PROJECT_ROOT, config['config']) if config['config'] else None
            self.web_apps[name] = ModuleController(
                name, config['script'], config_path, config['needs_com']
            )
        
        # ModbusTCP服務器控制器
        self.modbus_server = ModuleController(
            'ModbusTCP_Server', 'ModbusServer/TCPServer.py'
        )
    
    def start_modbus_server(self):
        """啟動ModbusTCP服務器"""
        try:
            print(f"\n=== 嘗試啟動ModbusTCP服務器 ===")
            print(f"TCPServer路徑: {self.modbus_server.script_path}")
            
            # 檢查TCPServer.py是否存在
            full_path = os.path.join(PROJECT_ROOT, self.modbus_server.script_path)
            print(f"完整路徑: {full_path}")
            
            if not os.path.exists(full_path):
                error_msg = f"TCPServer.py不存在: {full_path}"
                print(f"❌ {error_msg}")
                return False, error_msg
            
            print(f"✅ TCPServer.py檔案存在")
            
            # 檢查端口502是否被佔用
            print(f"檢查端口502狀態...")
            if self._check_port_occupied(502):
                error_msg = "端口502已被佔用，請先停止其他ModbusTCP服務"
                print(f"❌ {error_msg}")
                return False, error_msg
            
            print(f"✅ 端口502可用")
            
            # 嘗試啟動
            print(f"正在啟動ModbusTCP服務器...")
            success, message = self.modbus_server.start()
            
            if success:
                print(f"✅ ModbusTCP服務器啟動成功")
                print(f"   PID: {self.modbus_server.process.pid if self.modbus_server.process else 'N/A'}")
                
                # 等待一下再檢查端口
                time.sleep(2)
                if self._check_port_occupied(502):
                    print(f"✅ 端口502正在監聽")
                else:
                    print(f"⚠️  端口502未檢測到監聽狀態")
            else:
                print(f"❌ ModbusTCP服務器啟動失敗: {message}")
            
            return success, message
            
        except Exception as e:
            error_msg = f"啟動異常: {str(e)}"
            print(f"❌ {error_msg}")
            return False, error_msg
    
    def stop_modbus_server(self):
        """停止ModbusTCP服務器"""
        return self.modbus_server.stop()
    
    def start_module(self, module_name, com_port=None):
        """啟動模組"""
        print(f"\n=== 嘗試啟動模組: {module_name} ===")
        
        if module_name in self.modules:
            controller = self.modules[module_name]
            module_type = "主模組"
        elif module_name in self.web_apps:
            controller = self.web_apps[module_name]
            module_type = "WebUI模組"
        else:
            error_msg = f"未知模組: {module_name}"
            print(f"❌ {error_msg}")
            return False, error_msg
        
        print(f"模組類型: {module_type}")
        print(f"腳本路徑: {controller.script_path}")
        
        # 檢查腳本檔案是否存在
        full_path = os.path.join(PROJECT_ROOT, controller.script_path)
        print(f"完整路徑: {full_path}")
        
        if not os.path.exists(full_path):
            error_msg = f"腳本檔案不存在: {full_path}"
            print(f"❌ {error_msg}")
            return False, error_msg
        
        print(f"✅ 腳本檔案存在")
        
        # COM口配置
        if controller.needs_com:
            if com_port:
                print(f"需要COM口: {com_port}")
                print(f"配置檔案: {controller.config_path}")
            else:
                print(f"⚠️  模組需要COM口但未提供")
        else:
            print(f"無需COM口配置")
        
        # 嘗試啟動
        print(f"正在啟動模組...")
        success, message = controller.start(com_port)
        
        if success:
            print(f"✅ 模組 {module_name} 啟動成功")
            print(f"   PID: {controller.process.pid if controller.process else 'N/A'}")
            
            # 如果是WebUI模組，檢查端口
            if module_name in self.web_apps:
                module_key = module_name.replace('_app', '')
                port = self.web_ports.get(module_key, 0)
                if port > 0:
                    print(f"   預期端口: {port}")
                    time.sleep(2)
                    if self._check_port_occupied(port):
                        print(f"✅ 端口 {port} 正在監聽")
                    else:
                        print(f"⚠️  端口 {port} 未檢測到監聽狀態")
        else:
            print(f"❌ 模組 {module_name} 啟動失敗: {message}")
        
        return success, message
    
    def stop_module(self, module_name):
        """停止模組"""
        print(f"\n=== 嘗試停止模組: {module_name} ===")
        
        if module_name in self.modules:
            controller = self.modules[module_name]
            module_type = "主模組"
        elif module_name in self.web_apps:
            controller = self.web_apps[module_name]
            module_type = "WebUI模組"
        else:
            error_msg = f"未知模組: {module_name}"
            print(f"❌ {error_msg}")
            return False, error_msg
        
        print(f"模組類型: {module_type}")
        
        if not controller.is_running():
            message = "模組未運行"
            print(f"⚠️  {message}")
            return True, message
        
        print(f"正在停止模組...")
        success, message = controller.stop()
        
        if success:
            print(f"✅ 模組 {module_name} 停止成功")
        else:
            print(f"❌ 模組 {module_name} 停止失敗: {message}")
        
        return success, message
    
    def get_module_status(self, module_name):
        """獲取模組狀態"""
        if module_name in self.modules:
            return self.modules[module_name].get_status()
        elif module_name in self.web_apps:
            return self.web_apps[module_name].get_status()
        else:
            return "未知"
    
    def get_com_port_config(self, module_name):
        """獲取模組COM口配置"""
        if module_name in self.modules:
            controller = self.modules[module_name]
            if controller.config_path and controller.needs_com:
                return ConfigManager.read_com_port(controller.config_path, module_name)
        return None
    
    def scan_com_ports(self):
        """掃描可用COM口"""
        try:
            ports = serial.tools.list_ports.comports()
            return [port.device for port in ports]
        except Exception:
            return []
    
    def _check_port_occupied(self, port):
        """檢查端口是否被佔用"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1)
                result = s.connect_ex(('127.0.0.1', port))
                return result == 0
        except Exception:
            return False

# Flask應用
app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'dobot_m1_startup_tool_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# 全局管理器
startup_manager = StartupManager()
status_monitor = StatusMonitor(socketio, startup_manager)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/modbus_server/start', methods=['POST'])
def start_modbus_server():
    print(f"\n[API] 收到啟動ModbusTCP服務器請求")
    success, message = startup_manager.start_modbus_server()
    print(f"[API] 回應: success={success}, message={message}")
    return jsonify({'success': success, 'message': message})

@app.route('/api/modbus_server/stop', methods=['POST'])
def stop_modbus_server():
    print(f"\n[API] 收到停止ModbusTCP服務器請求")
    success, message = startup_manager.stop_modbus_server()
    print(f"[API] 回應: success={success}, message={message}")
    return jsonify({'success': success, 'message': message})

@app.route('/api/modbus_server/status', methods=['GET'])
def get_modbus_server_status():
    return jsonify({
        'running': startup_manager.modbus_server.is_running(),
        'status': startup_manager.modbus_server.get_status()
    })

@app.route('/api/modules/<module_name>/start', methods=['POST'])
def start_module(module_name):
    data = request.json or {}
    com_port = data.get('com_port')
    
    print(f"\n[API] 收到啟動模組請求: {module_name}, COM口: {com_port}")
    success, message = startup_manager.start_module(module_name, com_port)
    print(f"[API] 回應: success={success}, message={message}")
    return jsonify({'success': success, 'message': message})

@app.route('/api/modules/<module_name>/stop', methods=['POST'])
def stop_module(module_name):
    print(f"\n[API] 收到停止模組請求: {module_name}")
    success, message = startup_manager.stop_module(module_name)
    print(f"[API] 回應: success={success}, message={message}")
    return jsonify({'success': success, 'message': message})

@app.route('/api/modules/<module_name>/status', methods=['GET'])
def get_module_status(module_name):
    status = startup_manager.get_module_status(module_name)
    com_config = startup_manager.get_com_port_config(module_name)
    
    return jsonify({
        'status': status,
        'com_port': com_config
    })

@app.route('/api/com_ports', methods=['GET'])
def get_com_ports():
    ports = startup_manager.scan_com_ports()
    return jsonify({'ports': ports})

@app.route('/api/config/<module_name>/com', methods=['PUT'])
def update_com_config(module_name):
    data = request.json
    new_com = data.get('com_port')
    
    if module_name in startup_manager.modules:
        controller = startup_manager.modules[module_name]
        if controller.config_path and controller.needs_com:
            success = ConfigManager.update_com_port(controller.config_path, module_name, new_com)
            return jsonify({
                'success': success,
                'message': f'COM口配置{"更新成功" if success else "更新失敗"}'
            })
    
    return jsonify({'success': False, 'message': '模組不支援COM口配置'})

@socketio.on('connect')
def handle_connect():
    print(f"[WebSocket] 客戶端連接")
    emit('connected', {'message': 'DobotM1專案啟動工具已連接'})

@socketio.on('start_monitoring')
def handle_start_monitoring():
    print(f"[WebSocket] 開始狀態監控")
    status_monitor.start_monitoring()
    emit('monitoring_started', {'message': '狀態監控已啟動'})
    
    # 立即發送一次狀態更新
    try:
        status_data = status_monitor._collect_status()
        emit('status_update', status_data)
        print(f"[WebSocket] 立即發送狀態更新")
    except Exception as e:
        print(f"[WebSocket] 立即狀態更新失敗: {e}")

@socketio.on('stop_monitoring')
def handle_stop_monitoring():
    print(f"[WebSocket] 停止狀態監控")
    status_monitor.stop_monitoring()
    emit('monitoring_stopped', {'message': '狀態監控已停止'})

@socketio.on('disconnect')
def handle_disconnect():
    print(f"[WebSocket] 客戶端斷開連接")

if __name__ == '__main__':
    print("=" * 80)
    print("DobotM1專案啟動工具啟動中...")
    print("=" * 80)
    print(f"Python版本: {sys.version}")
    print(f"工作目錄: {os.getcwd()}")
    print(f"腳本目錄: {SCRIPT_DIR}")
    print(f"專案根目錄: {PROJECT_ROOT}")
    print(f"Web介面: http://localhost:8081")
    print("=" * 80)
    
    # 初始化狀態檢查
    print("\n=== 初始化狀態檢查 ===")
    
    # 檢查各模組腳本檔案
    print("檢查模組腳本檔案...")
    for name, controller in startup_manager.modules.items():
        full_path = os.path.join(PROJECT_ROOT, controller.script_path)
        exists = os.path.exists(full_path)
        print(f"  {name}: {'✅' if exists else '❌'} {controller.script_path}")
    
    print("檢查WebUI腳本檔案...")
    for name, controller in startup_manager.web_apps.items():
        full_path = os.path.join(PROJECT_ROOT, controller.script_path)
        exists = os.path.exists(full_path)
        print(f"  {name}: {'✅' if exists else '❌'} {controller.script_path}")
    
    # 檢查ModbusTCP服務器
    tcpserver_path = os.path.join(PROJECT_ROOT, 'ModbusServer/TCPServer.py')
    tcpserver_exists = os.path.exists(tcpserver_path)
    print(f"ModbusTCP服務器: {'✅' if tcpserver_exists else '❌'} {tcpserver_path}")
    
    # 檢查端口狀態
    print("檢查關鍵端口狀態...")
    ports_to_check = [502, 8081] + list(startup_manager.web_ports.values())
    for port in ports_to_check:
        occupied = startup_manager._check_port_occupied(port)
        print(f"  端口 {port}: {'❌ 被佔用' if occupied else '✅ 可用'}")
    
    # 檢查COM口
    com_ports = startup_manager.scan_com_ports()
    print(f"可用COM口: {', '.join(com_ports) if com_ports else '無'}")
    
    print("=" * 80)
    print("啟動Flask應用...")
    
    try:
        socketio.run(app, host='0.0.0.0', port=8081, debug=False)
    except KeyboardInterrupt:
        print("\n正在關閉啟動工具...")
        status_monitor.stop_monitoring()
        print("啟動工具已關閉")
    except Exception as e:
        print(f"啟動工具錯誤: {e}")
        status_monitor.stop_monitoring()