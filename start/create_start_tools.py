#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DobotM1專案啟動工具檔案結構創建腳本
執行此腳本會自動創建完整的啟動工具目錄結構
"""

import os
import json

def create_directory_structure():
    """創建目錄結構"""
    directories = [
        'start',
        'start/modules',
        'start/templates', 
        'start/static',
        'start/static/css',
        'start/static/js',
        'start/logs'
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
        print(f"創建目錄: {directory}")

def create_modules_init():
    """創建modules/__init__.py"""
    content = '''"""
DobotM1專案啟動工具模組
"""

__version__ = "1.0.0"
__author__ = "DobotM1Project Team"
'''
    
    with open('start/modules/__init__.py', 'w', encoding='utf-8') as f:
        f.write(content)
    print("創建: start/modules/__init__.py")

def create_config_manager():
    """創建config_manager.py"""
    content = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json

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
    
    @staticmethod
    def validate_config(config_path):
        """驗證config檔案格式正確性"""
        try:
            if not os.path.exists(config_path):
                return False, "配置檔案不存在"
            
            with open(config_path, 'r', encoding='utf-8') as f:
                json.load(f)
            
            return True, "配置檔案格式正確"
            
        except json.JSONDecodeError as e:
            return False, f"JSON格式錯誤: {e}"
        except Exception as e:
            return False, f"驗證錯誤: {e}"
'''
    
    with open('start/modules/config_manager.py', 'w', encoding='utf-8') as f:
        f.write(content)
    print("創建: start/modules/config_manager.py")

def create_module_controller():
    """創建module_controller.py"""
    content = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import time
from .config_manager import ConfigManager

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
            
            # 檢查腳本檔案存在
            if not os.path.exists(self.script_path):
                return False, f"腳本檔案不存在: {self.script_path}"
            
            # 啟動程序
            self.process = subprocess.Popen(
                [sys.executable, self.script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=os.path.dirname(self.script_path)
            )
            
            # 等待一小段時間確認啟動成功
            time.sleep(2)
            if self.process.poll() is not None:
                stdout, stderr = self.process.communicate()
                error_msg = stderr.decode('utf-8', errors='ignore')
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
                self.process.wait(timeout=10)
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
'''
    
    with open('start/modules/module_controller.py', 'w', encoding='utf-8') as f:
        f.write(content)
    print("創建: start/modules/module_controller.py")

def create_status_monitor():
    """創建status_monitor.py"""
    content = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import socket
import serial.tools.list_ports

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
                time.sleep(3)  # 3秒間隔
            except Exception as e:
                print(f"狀態監控錯誤: {e}")
                time.sleep(3)
    
    def _collect_status(self):
        """收集所有狀態"""
        status = {
            'modbus_server': self._check_modbus_server(),
            'modules': {},
            'web_apps': {},
            'com_ports': self._scan_com_ports(),
            'timestamp': time.time()
        }
        
        # 檢查主模組狀態
        for name, controller in self.startup_manager.modules.items():
            status['modules'][name] = {
                'status': controller.get_status(),
                'running': controller.is_running()
            }
        
        # 檢查WebUI狀態
        for name, controller in self.startup_manager.web_apps.items():
            module_name = name.replace('_app', '')
            port = self.startup_manager.web_ports.get(module_name, 0)
            
            status['web_apps'][name] = {
                'status': controller.get_status(),
                'running': controller.is_running(),
                'port_active': self._check_port(port),
                'port': port
            }
        
        return status
    
    def _check_modbus_server(self):
        """檢查ModbusTCP服務器狀態"""
        return {
            'running': self.startup_manager.modbus_server.is_running(),
            'port_active': self._check_port(502)
        }
    
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
'''
    
    with open('start/modules/status_monitor.py', 'w', encoding='utf-8') as f:
        f.write(content)
    print("創建: start/modules/status_monitor.py")

def create_startup_manager():
    """創建startup_manager.py"""
    content = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import socket
import serial.tools.list_ports
from .module_controller import ModuleController
from .config_manager import ConfigManager

class StartupManager:
    """啟動管理器主類別"""
    
    def __init__(self, project_root):
        self.project_root = project_root
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
            config_path = os.path.join(self.project_root, config['config']) if config['config'] else None
            self.modules[name] = ModuleController(
                name, 
                os.path.join(self.project_root, config['script']), 
                config_path, 
                config['needs_com']
            )
        
        # 創建WebUI控制器
        for name, config in web_configs.items():
            config_path = os.path.join(self.project_root, config['config']) if config['config'] else None
            self.web_apps[name] = ModuleController(
                name, 
                os.path.join(self.project_root, config['script']), 
                config_path, 
                config['needs_com']
            )
        
        # ModbusTCP服務器控制器
        self.modbus_server = ModuleController(
            'ModbusTCP_Server', 
            os.path.join(self.project_root, 'ModbusServer/TCPServer.py')
        )
    
    def start_modbus_server(self):
        """啟動ModbusTCP服務器"""
        if self._check_port_occupied(502):
            return False, "端口502已被佔用"
        
        return self.modbus_server.start()
    
    def stop_modbus_server(self):
        """停止ModbusTCP服務器"""
        return self.modbus_server.stop()
    
    def start_module(self, module_name, com_port=None):
        """啟動模組"""
        if module_name in self.modules:
            return self.modules[module_name].start(com_port)
        elif module_name in self.web_apps:
            return self.web_apps[module_name].start(com_port)
        else:
            return False, f"未知模組: {module_name}"
    
    def stop_module(self, module_name):
        """停止模組"""
        if module_name in self.modules:
            return self.modules[module_name].stop()
        elif module_name in self.web_apps:
            return self.web_apps[module_name].stop()
        else:
            return False, f"未知模組: {module_name}"
    
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
'''
    
    with open('start/modules/startup_manager.py', 'w', encoding='utf-8') as f:
        f.write(content)
    print("創建: start/modules/startup_manager.py")

def main():
    """主函數"""
    print("開始創建DobotM1專案啟動工具檔案結構...")
    
    # 創建目錄結構
    create_directory_structure()
    
    # 創建模組檔案
    create_modules_init()
    create_config_manager()
    create_module_controller()
    create_status_monitor()
    create_startup_manager()
    
    print("\n檔案結構創建完成！")
    print("\n接下來的步驟：")
    print("1. 將 start.py 主程序檔案複製到 start/ 目錄")
    print("2. 將 index.html 複製到 start/templates/ 目錄")
    print("3. 將 style.css 複製到 start/static/css/ 目錄")
    print("4. 將 app.js 複製到 start/static/js/ 目錄")
    print("5. 將 requirements.txt 複製到 start/ 目錄")
    print("6. 執行: cd start && pip install -r requirements.txt")
    print("7. 執行: python start.py")
    print("8. 訪問: http://localhost:8081")

if __name__ == "__main__":
    main()