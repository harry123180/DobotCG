#!/usr/bin/env python3
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
