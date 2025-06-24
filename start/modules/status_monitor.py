#!/usr/bin/env python3
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
