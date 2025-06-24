#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XCApp.py - XC100 可視化控制應用 - 修正版本
基於Flask的Web界面，通過Modbus TCP與XCModule通訊
修正寄存器地址映射和頁面刷新問題
"""

import json
import time
import threading
import os
from datetime import datetime
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient
import logging

# 禁用Flask的日誌輸出
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

class XCApp:
    """XC100 Web應用 - 修正版本"""
    
    def __init__(self, config_file="xc_app_config.json"):
        # 獲取執行檔案目錄
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # Modbus TCP客戶端
        self.modbus_client = None
        self.connected = False
        self.connection_retry_count = 0
        self.max_retry_count = 5
        
        # 修正: 使用正確的基地址
        self.base_address = 1000  # 與XCModule.py一致
        
        # 設備狀態
        self.device_status = {
            "state": "未知",
            "servo_status": False,
            "error_code": 0,
            "current_position": 0,
            "target_position": 0,
            "command_executing": False,
            "position_A": 400,
            "position_B": 2682,
            "module_connected": False,
            "communication_health": 100,
            "last_update": datetime.now().strftime("%H:%M:%S")
        }
        
        # 應用狀態
        self.app_stats = {
            "total_commands": 0,
            "successful_commands": 0,
            "failed_commands": 0,
            "uptime_start": datetime.now(),
            "communication_errors": 0
        }
        
        # Flask應用
        self.app = Flask(__name__)
        self.app.secret_key = 'xc100_app_secret_key_v3'
        
        # 修正: 添加SocketIO支持
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        self.setup_routes()
        self.setup_socketio_events()
        
        # 監控線程控制
        self.monitor_thread = None
        self.monitor_running = False
        self.auto_refresh_enabled = True  # 新增: 自動刷新控制
        self.manual_refresh_mode = False  # 新增: 手動刷新模式
        
        print("XCApp初始化完成")
    
    def load_config(self):
        """載入配置"""
        default_config = {
            "modbus_tcp": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 3.0,
                "retry_on_failure": True,
                "max_retries": 3
            },
            "web_server": {
                "host": "0.0.0.0",
                "port": 5007,
                "debug": False
            },
            "xc_module": {
                "base_address": 1000,  # 修正: 明確指定基地址
                "register_count": 50
            },
            "ui_settings": {
                "auto_refresh": True,
                "refresh_interval": 3.0,  # 修正: 增加到3秒
                "show_debug_info": True,
                "command_confirmation": False,
                "manual_mode": False  # 新增: 手動模式
            }
        }
        
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    # 深度合併配置
                    for key, value in default_config.items():
                        if key not in loaded_config:
                            loaded_config[key] = value
                        elif isinstance(value, dict):
                            for sub_key, sub_value in value.items():
                                if sub_key not in loaded_config[key]:
                                    loaded_config[key][sub_key] = sub_value
                    return loaded_config
            else:
                self.save_config(default_config)
                return default_config
        except Exception as e:
            print(f"載入配置失敗: {e}")
            
        return default_config
    
    def save_config(self, config=None):
        """保存配置"""
        try:
            config_to_save = config or self.config
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config_to_save, f, indent=2, ensure_ascii=False)
            print(f"配置已保存: {self.config_file}")
        except Exception as e:
            print(f"保存配置失敗: {e}")
    
    def connect_modbus(self):
        """連線到XCModule with retry logic"""
        try:
            modbus_config = self.config["modbus_tcp"]
            
            if self.modbus_client:
                try:
                    self.modbus_client.close()
                except:
                    pass
            
            self.modbus_client = ModbusTcpClient(
                host=modbus_config["host"],
                port=modbus_config["port"],
                timeout=modbus_config["timeout"]
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self.connection_retry_count = 0
                print(f"已連線到XCModule: {modbus_config['host']}:{modbus_config['port']}")
                return True
            else:
                self.connected = False
                self.connection_retry_count += 1
                print(f"連線XCModule失敗 (重試 {self.connection_retry_count}/{self.max_retry_count})")
                return False
                
        except Exception as e:
            self.connected = False
            self.connection_retry_count += 1
            print(f"連線XCModule異常 (重試 {self.connection_retry_count}/{self.max_retry_count}): {e}")
            return False
    
    def disconnect_modbus(self):
        """斷開Modbus連線"""
        try:
            if self.modbus_client and self.connected:
                self.modbus_client.close()
                self.connected = False
                print("已斷開XCModule連線")
        except Exception as e:
            print(f"斷開連線異常: {e}")
    
    def read_device_status(self):
        """讀取設備狀態 - 修正寄存器地址"""
        if not self.connected:
            return False
        
        try:
            unit_id = self.config["modbus_tcp"]["unit_id"]
            
            # 修正: 使用正確的基地址讀取狀態寄存器 (1000-1014)
            result = self.modbus_client.read_holding_registers(
                address=self.base_address, 
                count=15, 
                slave=unit_id
            )
            
            if not result.isError():
                registers = result.registers
                
                # 狀態映射
                state_map = {
                    0: "離線", 1: "閒置", 2: "移動中", 3: "原點復歸中", 
                    4: "錯誤", 5: "Servo關閉", 6: "緊急停止"
                }
                
                # 錯誤代碼描述
                error_map = {
                    0: "正常",
                    1: "在動作中接收動作指令",
                    2: "上下限錯誤", 
                    3: "位置錯誤",
                    4: "格式錯誤",
                    5: "控制模式錯誤",
                    6: "斷電重開",
                    7: "初始化未完成",
                    8: "Servo ON/OFF 錯誤",
                    9: "LOCK",
                    10: "軟體極限",
                    11: "參數寫入權限不足",
                    12: "原點復歸未完成",
                    13: "剎車已解除",
                    999: "通訊異常"
                }
                
                self.device_status.update({
                    "state": state_map.get(registers[0], f"未知({registers[0]})"),
                    "xc_connected": registers[1] == 1,  # 修正: XC設備連接狀態
                    "servo_status": registers[2] == 1,
                    "error_code": registers[3],
                    "error_description": error_map.get(registers[3], f"未知錯誤({registers[3]})"),
                    "current_position": (registers[5] << 16) | registers[4],  # 修正: 32位位置合併
                    "target_position": (registers[7] << 16) | registers[6],   # 修正: 32位位置合併
                    "command_executing": registers[8] == 1,
                    "comm_errors": registers[9],
                    "position_A": (registers[11] << 16) | registers[10],      # 修正: A點位置
                    "position_B": (registers[13] << 16) | registers[12],      # 修正: B點位置
                    "module_connected": True,  # 能讀取到數據說明模組已連接
                    "last_update": datetime.now().strftime("%H:%M:%S")
                })
                
                # 計算通訊健康度
                if self.app_stats["communication_errors"] == 0:
                    self.device_status["communication_health"] = 100
                else:
                    health = max(0, 100 - (self.app_stats["communication_errors"] * 10))
                    self.device_status["communication_health"] = health
                
                return True
                
        except Exception as e:
            self.app_stats["communication_errors"] += 1
            print(f"讀取設備狀態異常: {e}")
            self.device_status["module_connected"] = False
            return False
    
    def send_command(self, command, param1=0, param2=0):
        """發送指令到XCModule - 修正寄存器地址"""
        if not self.connected:
            self.app_stats["failed_commands"] += 1
            return False
        
        try:
            self.app_stats["total_commands"] += 1
            unit_id = self.config["modbus_tcp"]["unit_id"]
            
            # 先檢查是否有指令正在執行
            status_result = self.modbus_client.read_holding_registers(
                address=self.base_address + 8, count=1, slave=unit_id
            )
            if not status_result.isError() and status_result.registers[0] == 1:
                print("有指令正在執行中，請稍候")
                self.app_stats["failed_commands"] += 1
                return False
            
            # 修正: 使用正確的指令寄存器地址 (1020-1024)
            command_address = self.base_address + 20  # 1020
            command_id = int(time.time()) % 65536  # 生成唯一ID
            
            # 寫入指令寄存器
            values = [command, param1, param2, command_id, 0]
            result = self.modbus_client.write_registers(
                address=command_address, 
                values=values, 
                slave=unit_id
            )
            
            if not result.isError():
                command_names = {
                    1: 'Servo ON', 2: 'Servo OFF', 3: '原點復歸',
                    4: '絕對移動', 6: '緊急停止', 7: '錯誤重置'
                }
                print(f"指令發送成功: {command_names.get(command, f'指令{command}')} (ID: {command_id})")
                self.app_stats["successful_commands"] += 1
                return True
            else:
                print(f"指令發送失敗: {result}")
                self.app_stats["failed_commands"] += 1
                return False
                
        except Exception as e:
            print(f"發送指令異常: {e}")
            self.app_stats["failed_commands"] += 1
            self.app_stats["communication_errors"] += 1
            return False
    
    def update_position(self, pos_type, position):
        """更新位置設定 - 修正寄存器地址"""
        if not self.connected:
            return False
        
        try:
            # 位置範圍檢查
            if not (-999999 <= position <= 999999):
                print(f"位置超出範圍: {position}")
                return False
            
            unit_id = self.config["modbus_tcp"]["unit_id"]
            
            # 32位位置分解
            pos_low = position & 0xFFFF
            pos_high = (position >> 16) & 0xFFFF
            
            if pos_type == 'A':
                # 修正: 更新A點位置 (1010-1011)
                address = self.base_address + 10
                result = self.modbus_client.write_registers(
                    address=address, values=[pos_low, pos_high], slave=unit_id
                )
            elif pos_type == 'B':
                # 修正: 更新B點位置 (1012-1013)
                address = self.base_address + 12
                result = self.modbus_client.write_registers(
                    address=address, values=[pos_low, pos_high], slave=unit_id
                )
            else:
                return False
            
            if not result.isError():
                print(f"{pos_type}點位置已更新為: {position}")
                # 更新本地狀態
                if pos_type == 'A':
                    self.device_status["position_A"] = position
                else:
                    self.device_status["position_B"] = position
                return True
            else:
                print(f"位置更新失敗: {result}")
                return False
            
        except Exception as e:
            print(f"更新位置異常: {e}")
            return False
    
    def monitor_loop(self):
        """監控循環 - 修正刷新頻率"""
        print("開始設備狀態監控")
        
        refresh_interval = self.config["ui_settings"]["refresh_interval"]
        
        while self.monitor_running:
            try:
                if self.connected:
                    if not self.read_device_status():
                        # 讀取失敗，可能需要重連
                        self.connected = False
                else:
                    # 嘗試重新連線
                    if self.connection_retry_count < self.max_retry_count:
                        print(f"嘗試重新連線... ({self.connection_retry_count + 1}/{self.max_retry_count})")
                        self.connect_modbus()
                    elif self.connection_retry_count >= self.max_retry_count:
                        print("達到最大重試次數，停止重連嘗試")
                        time.sleep(10)  # 等待10秒後重置重試計數
                        self.connection_retry_count = 0
                
                # 修正: 只有在自動刷新模式下才發送更新
                if self.auto_refresh_enabled and not self.manual_refresh_mode:
                    self.socketio.emit('status_update', self.get_full_status())
                
                time.sleep(refresh_interval)
                
            except Exception as e:
                print(f"監控循環異常: {e}")
                time.sleep(5)
        
        print("設備狀態監控停止")
    
    def start_monitoring(self):
        """開始監控"""
        if not self.monitor_running:
            self.monitor_running = True
            self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
            self.monitor_thread.start()
    
    def stop_monitoring(self):
        """停止監控"""
        self.monitor_running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=3)
    
    def get_app_statistics(self):
        """獲取應用統計"""
        uptime = datetime.now() - self.app_stats["uptime_start"]
        success_rate = 0
        if self.app_stats["total_commands"] > 0:
            success_rate = (self.app_stats["successful_commands"] / self.app_stats["total_commands"]) * 100
        
        return {
            "uptime": str(uptime).split('.')[0],
            "total_commands": self.app_stats["total_commands"],
            "successful_commands": self.app_stats["successful_commands"],
            "failed_commands": self.app_stats["failed_commands"],
            "success_rate": f"{success_rate:.1f}%",
            "communication_errors": self.app_stats["communication_errors"]
        }
    
    def get_full_status(self):
        """獲取完整狀態"""
        return {
            "success": True,
            "connected": self.connected,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "data": self.device_status,
            "statistics": self.get_app_statistics(),
            "config": {
                "base_address": self.base_address,
                "auto_refresh": self.auto_refresh_enabled,
                "manual_mode": self.manual_refresh_mode,
                "refresh_interval": self.config["ui_settings"]["refresh_interval"]
            }
        }
    
    def setup_socketio_events(self):
        """設置SocketIO事件 - 新增手動控制功能"""
        
        @self.socketio.on('connect')
        def handle_connect():
            """客戶端連接"""
            print("Web客戶端已連接")
            emit('status_update', self.get_full_status())
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            """客戶端斷開"""
            print("Web客戶端已斷開")
        
        @self.socketio.on('request_status')
        def handle_status_request():
            """手動請求狀態更新"""
            emit('status_update', self.get_full_status())
        
        @self.socketio.on('toggle_auto_refresh')
        def handle_toggle_auto_refresh(data):
            """切換自動刷新"""
            self.auto_refresh_enabled = data.get('enabled', True)
            print(f"自動刷新: {'開啟' if self.auto_refresh_enabled else '關閉'}")
            emit('auto_refresh_status', {'enabled': self.auto_refresh_enabled})
        
        @self.socketio.on('set_manual_mode')
        def handle_manual_mode(data):
            """設置手動模式"""
            self.manual_refresh_mode = data.get('manual', False)
            print(f"手動模式: {'開啟' if self.manual_refresh_mode else '關閉'}")
            emit('manual_mode_status', {'manual': self.manual_refresh_mode})
    
    def setup_routes(self):
        """設置Flask路由 - 增強功能"""
        
        @self.app.route('/')
        def index():
            """主頁"""
            return render_template('index.html')
        
        @self.app.route('/api/status')
        def get_status():
            """獲取設備狀態API"""
            return jsonify(self.get_full_status())
        
        @self.app.route('/api/command', methods=['POST'])
        def send_command_api():
            """發送指令API"""
            try:
                data = request.get_json()
                command = data.get('command', 0)
                param1 = data.get('param1', 0)
                param2 = data.get('param2', 0)
                
                # 指令驗證
                valid_commands = [1, 2, 3, 4, 6, 7]
                if command not in valid_commands:
                    return jsonify({"success": False, "message": f"無效的指令代碼: {command}"})
                
                if self.send_command(command, param1, param2):
                    # 指令發送後立即更新狀態
                    time.sleep(0.1)
                    self.read_device_status()
                    return jsonify({"success": True, "message": "指令發送成功"})
                else:
                    return jsonify({"success": False, "message": "指令發送失敗"})
                    
            except Exception as e:
                return jsonify({"success": False, "message": f"發送指令異常: {e}"})
        
        @self.app.route('/api/position', methods=['POST'])
        def update_position_api():
            """更新位置API"""
            try:
                data = request.get_json()
                pos_type = data.get('type')  # 'A' or 'B'
                position = int(data.get('position', 0))
                
                if pos_type not in ['A', 'B']:
                    return jsonify({"success": False, "message": "無效的位置類型"})
                
                if self.update_position(pos_type, position):
                    # 位置更新後立即刷新狀態
                    time.sleep(0.1)
                    self.read_device_status()
                    return jsonify({"success": True, "message": f"{pos_type}點位置更新成功"})
                else:
                    return jsonify({"success": False, "message": "位置更新失敗"})
                    
            except ValueError:
                return jsonify({"success": False, "message": "位置必須是數字"})
            except Exception as e:
                return jsonify({"success": False, "message": f"更新位置異常: {e}"})
        
        @self.app.route('/api/connect', methods=['POST'])
        def connect_api():
            """連線API"""
            self.connection_retry_count = 0
            if self.connect_modbus():
                return jsonify({"success": True, "message": "連線成功"})
            else:
                return jsonify({"success": False, "message": "連線失敗"})
        
        @self.app.route('/api/disconnect', methods=['POST'])
        def disconnect_api():
            """斷開連線API"""
            self.disconnect_modbus()
            return jsonify({"success": True, "message": "已斷開連線"})
        
        @self.app.route('/api/manual_refresh', methods=['POST'])
        def manual_refresh():
            """手動刷新狀態"""
            if self.read_device_status():
                return jsonify(self.get_full_status())
            else:
                return jsonify({"success": False, "message": "讀取狀態失敗"})
        
        @self.app.route('/api/settings', methods=['GET', 'POST'])
        def settings_api():
            """設置API"""
            if request.method == 'GET':
                return jsonify({
                    "success": True,
                    "settings": self.config["ui_settings"]
                })
            else:
                try:
                    data = request.get_json()
                    
                    # 更新設置
                    if 'auto_refresh' in data:
                        self.config["ui_settings"]["auto_refresh"] = data['auto_refresh']
                        self.auto_refresh_enabled = data['auto_refresh']
                    
                    if 'refresh_interval' in data:
                        self.config["ui_settings"]["refresh_interval"] = float(data['refresh_interval'])
                    
                    if 'manual_mode' in data:
                        self.config["ui_settings"]["manual_mode"] = data['manual_mode']
                        self.manual_refresh_mode = data['manual_mode']
                    
                    self.save_config()
                    return jsonify({"success": True, "message": "設置已更新"})
                    
                except Exception as e:
                    return jsonify({"success": False, "message": f"更新設置失敗: {e}"})
        
        @self.app.route('/api/debug')
        def debug_info():
            """調試信息API"""
            return jsonify({
                "success": True,
                "debug_info": {
                    "base_address": self.base_address,
                    "config_file": self.config_file,
                    "current_dir": self.current_dir,
                    "connected": self.connected,
                    "auto_refresh": self.auto_refresh_enabled,
                    "manual_mode": self.manual_refresh_mode,
                    "monitor_running": self.monitor_running
                }
            })
    
    def run(self):
        """運行Web應用"""
        # 檢查XCModule是否在運行
        if not self.connect_modbus():
            print("無法連線到XCModule")
            print("請確保XCModule.py正在運行，然後重試")
            print("\n故障排除步驟：")
            print("1. 確認XCModule.py已啟動並顯示'模組啟動成功'")
            print("2. 檢查Modbus TCP Server是否在127.0.0.1:502運行")
            print("3. 確認防火牆沒有阻擋端口502")
            print("4. 檢查XC100設備是否正確連接")
            print("\n將以離線模式啟動Web界面...")
        
        # 檢查templates目錄
        templates_dir = os.path.join(self.current_dir, 'templates')
        if not os.path.exists(templates_dir):
            print("找不到templates目錄")
            print(f"請在 {self.current_dir} 目錄下創建templates文件夾")
            return
        
        index_file = os.path.join(templates_dir, 'index.html')
        if not os.path.exists(index_file):
            print("找不到templates/index.html文件")
            print("請將index.html放置在templates目錄中")
            return
        
        # 開始監控
        self.start_monitoring()
        
        try:
            web_config = self.config["web_server"]
            print(f"\nXCApp啟動")
            print(f"Web界面: http://localhost:{web_config['port']}")
            print(f"配置文件: {self.config_file}")
            print(f"刷新間隔: {self.config['ui_settings']['refresh_interval']}秒")
            print(f"寄存器基地址: {self.base_address}")
            print(f"模板目錄: {templates_dir}")
            print("\n修正功能:")
            print("  修正寄存器地址映射")
            print("  優化頁面刷新頻率")
            print("  新增手動刷新模式")
            print("  改善位置輸入體驗")
            print("  SocketIO即時通訊")
            print("  配置文件自動保存")
            print("\n按 Ctrl+C 停止應用")
            
            # 啟動Flask應用
            self.socketio.run(
                self.app,
                host=web_config["host"],
                port=web_config["port"],
                debug=web_config["debug"],
                allow_unsafe_werkzeug=True
            )
            
        except KeyboardInterrupt:
            print("\n正在停止應用...")
        except Exception as e:
            print(f"\nWeb應用運行異常: {e}")
        finally:
            self.stop_monitoring()
            self.disconnect_modbus()
            print("XCApp已停止")

def main():
    """主函數"""
    import argparse
    
    parser = argparse.ArgumentParser(description='XC100 Web控制應用')
    parser.add_argument('--config', type=str, default="xc_app_config.json", help='配置文件路徑')
    parser.add_argument('--port', type=int, help='Web服務器端口')
    parser.add_argument('--host', type=str, help='Web服務器主機')
    parser.add_argument('--modbus-host', type=str, help='XCModule主機地址')
    parser.add_argument('--modbus-port', type=int, help='XCModule端口')
    parser.add_argument('--debug', action='store_true', help='啟用調試模式')
    args = parser.parse_args()
    
    print("XCApp - XC100 Web控制應用 修正版本")
    print("=" * 50)
    
    # 創建應用實例
    app = XCApp(args.config)
    
    # 覆蓋命令行參數
    if args.port:
        app.config["web_server"]["port"] = args.port
    if args.host:
        app.config["web_server"]["host"] = args.host
    if args.modbus_host:
        app.config["modbus_tcp"]["host"] = args.modbus_host
    if args.modbus_port:
        app.config["modbus_tcp"]["port"] = args.modbus_port
    if args.debug:
        app.config["web_server"]["debug"] = True
    
    # 保存更新後的配置
    app.save_config()
    
    # 運行應用
    app.run()

if __name__ == "__main__":
    main()