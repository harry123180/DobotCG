#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main_app.py - Dobot主控制器Web介面 (增強版)
增加Angle系統、CCD1系統、CCD3系統狀態監控
增加強制Ready按鈕測試功能
"""

import os
import json
import time
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client.tcp import ModbusTcpClient

# 配置檔案
CONFIG_FILE = "dobot_main_app_config.json"

# Modbus寄存器映射 (狀態機交握版本)
class DobotRegisters:
    # 狀態寄存器 (400-419) - 只讀
    STATUS_REGISTER = 400     # 主狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)
    ROBOT_STATE = 401         # 機械臂狀態
    CURRENT_FLOW = 402        # 當前流程ID
    FLOW_PROGRESS = 403       # 流程執行進度
    ERROR_CODE = 404          # 錯誤代碼
    ROBOT_MODE = 405          # 機械臂模式
    POS_X = 406              # 當前X座標
    POS_Y = 407              # 當前Y座標
    POS_Z = 408              # 當前Z座標
    POS_R = 409              # 當前R座標
    JOINT_J1 = 410           # J1角度
    JOINT_J2 = 411           # J2角度
    JOINT_J3 = 412           # J3角度
    JOINT_J4 = 413           # J4角度
    DI_STATUS = 414          # 數位輸入狀態
    DO_STATUS = 415          # 數位輸出狀態
    OP_COUNTER = 416         # 操作計數器
    ERR_COUNTER = 417        # 錯誤計數器
    RUN_TIME = 418           # 運行時間(分鐘)
    GLOBAL_SPEED = 419       # 全局速度設定值
    
    # 控制寄存器 (440-449) - 讀寫
    VP_CONTROL = 440         # VP視覺取料控制
    UNLOAD_CONTROL = 441     # 出料控制
    CLEAR_ALARM = 442        # 清除警報控制
    EMERGENCY_STOP = 443     # 緊急停止控制
    MANUAL_COMMAND = 444     # 手動指令 (Web端使用)
    SPEED_COMMAND = 445      # 速度控制指令
    SPEED_VALUE = 446        # 速度數值
    SPEED_CMD_ID = 447       # 速度指令ID
    FORCE_READY = 448        # 強制Ready測試 (新增)

# 外部系統寄存器映射
class ExternalSystemRegisters:
    # CCD1系統 (基地址200)
    CCD1_CONTROL = 200           # 控制指令
    CCD1_STATUS = 201            # 狀態寄存器
    CCD1_CIRCLE_COUNT = 240      # 檢測圓形數量 (佇列物體數量)
    CCD1_WORLD_COORD_VALID = 256 # 世界座標有效標誌
    
    # Angle系統 (基地址700)
    ANGLE_STATUS = 700           # 狀態寄存器
    ANGLE_CONTROL = 740          # 控制指令
    ANGLE_SUCCESS = 720          # 成功標誌
    ANGLE_ORIGINAL_HIGH = 721    # 原始角度高位
    ANGLE_ORIGINAL_LOW = 722     # 原始角度低位
    ANGLE_DIFF_HIGH = 723        # 角度差高位
    ANGLE_DIFF_LOW = 724         # 角度差低位
    ANGLE_MOTOR_POS_HIGH = 725   # 馬達位置高位
    ANGLE_MOTOR_POS_LOW = 726    # 馬達位置低位
    
    # CCD3系統 (基地址800)
    CCD3_CONTROL = 800           # 控制指令
    CCD3_STATUS = 801            # 狀態寄存器

# 狀態映射
ROBOT_STATES = {
    0: "空閒", 1: "運行中", 2: "暫停", 3: "錯誤", 4: "緊急停止"
}

FLOW_TYPES = {
    0: "無流程", 1: "VP視覺抓取(FIFO)", 2: "出料流程", 3: "完整加工流程"
}

SYSTEM_STATUS_BITS = {
    "ready": "準備就緒",
    "running": "執行中",
    "alarm": "警報",
    "initialized": "已初始化"
}

class DobotHandshakeController:
    """Dobot狀態機交握控制器 (增強版)"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        self.modbus_client = None
        self.is_connected = False
        self.monitoring = False
        self.monitor_thread = None
        self.speed_cmd_id_counter = 1
        
    def _load_config(self) -> dict:
        """載入配置檔案"""
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.config_file)
        
        default_config = {
            "modbus": {
                "server_ip": "127.0.0.1",
                "server_port": 502,
                "timeout": 3.0
            },
            "web_server": {
                "host": "0.0.0.0",
                "port": 5009,
                "debug": False
            },
            "monitoring": {
                "refresh_interval": 1.0,
                "auto_start": True
            },
            "handshake_settings": {
                "command_timeout": 5.0,
                "retry_count": 3,
                "auto_clear_commands": True
            },
            "speed_control": {
                "min_speed": 1,
                "max_speed": 100,
                "default_speed": 50
            },
            "external_systems": {
                "monitor_ccd1": True,
                "monitor_angle": True,
                "monitor_ccd3": True
            }
        }
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    self._deep_update(default_config, user_config)
            except Exception as e:
                print(f"載入配置檔案失敗，使用預設配置: {e}")
        else:
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"創建預設配置檔案: {config_path}")
            except Exception as e:
                print(f"創建配置檔案失敗: {e}")
                
        return default_config
    
    def _deep_update(self, base_dict: dict, update_dict: dict):
        """深度更新字典"""
        for key, value in update_dict.items():
            if key in base_dict and isinstance(base_dict[key], dict) and isinstance(value, dict):
                self._deep_update(base_dict[key], value)
            else:
                base_dict[key] = value
    
    def connect_modbus(self) -> bool:
        """連接Modbus服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
                
            self.modbus_client = ModbusTcpClient(
                self.config["modbus"]["server_ip"],
                port=self.config["modbus"]["server_port"]
            )
            
            if self.modbus_client.connect():
                self.is_connected = True
                print(f"Modbus連接成功: {self.config['modbus']['server_ip']}:{self.config['modbus']['server_port']}")
                return True
            else:
                self.is_connected = False
                print("Modbus連接失敗")
                return False
                
        except Exception as e:
            self.is_connected = False
            print(f"Modbus連接異常: {e}")
            return False
    
    def read_register(self, address: int) -> int:
        """讀取單個寄存器"""
        try:
            if not self.is_connected:
                return None
                
            result = self.modbus_client.read_holding_registers(address, count=1)
            if hasattr(result, 'registers') and len(result.registers) > 0:
                return result.registers[0]
            return None
        except Exception as e:
            print(f"讀取寄存器{address}失敗: {e}")
            return None
    
    def read_multiple_registers(self, start_address: int, count: int) -> list:
        """讀取多個寄存器"""
        try:
            if not self.is_connected:
                return None
                
            result = self.modbus_client.read_holding_registers(start_address, count=count)
            if hasattr(result, 'registers') and len(result.registers) >= count:
                return result.registers
            return None
        except Exception as e:
            print(f"讀取寄存器{start_address}(數量{count})失敗: {e}")
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            if not self.is_connected:
                return False
                
            result = self.modbus_client.write_register(address, value)
            return not (hasattr(result, 'isError') and result.isError())
        except Exception as e:
            print(f"寫入寄存器{address}={value}失敗: {e}")
            return False
    
    def get_status_bits(self, status_register: int) -> dict:
        """解析狀態寄存器位元"""
        if status_register is None:
            return {}
        
        return {
            "ready": bool(status_register & 0x01),      # bit0
            "running": bool(status_register & 0x02),    # bit1
            "alarm": bool(status_register & 0x04),      # bit2
            "initialized": bool(status_register & 0x08), # bit3
            "status_register_value": status_register,
            "status_register_binary": f"{status_register:04b}"
        }
    
    def is_ready_for_command(self) -> bool:
        """檢查系統是否準備好接受指令"""
        status_register = self.read_register(DobotRegisters.STATUS_REGISTER)
        status_bits = self.get_status_bits(status_register)
        return (status_bits.get("ready", False) and 
                not status_bits.get("running", False) and 
                not status_bits.get("alarm", False))
    
    def read_external_systems_status(self) -> dict:
        """讀取外部系統狀態"""
        external_status = {}
        
        try:
            # CCD1系統狀態
            if self.config["external_systems"]["monitor_ccd1"]:
                ccd1_status_reg = self.read_register(ExternalSystemRegisters.CCD1_STATUS)
                ccd1_circle_count = self.read_register(ExternalSystemRegisters.CCD1_CIRCLE_COUNT)
                ccd1_world_valid = self.read_register(ExternalSystemRegisters.CCD1_WORLD_COORD_VALID)
                
                external_status["ccd1"] = {
                    "status_bits": self.get_status_bits(ccd1_status_reg),
                    "circle_count": ccd1_circle_count or 0,
                    "world_coord_valid": bool(ccd1_world_valid),
                    "connected": ccd1_status_reg is not None
                }
            
            # Angle系統狀態
            if self.config["external_systems"]["monitor_angle"]:
                angle_status_reg = self.read_register(ExternalSystemRegisters.ANGLE_STATUS)
                angle_success = self.read_register(ExternalSystemRegisters.ANGLE_SUCCESS)
                
                # 讀取角度檢測結果 (32位數值恢復)
                angle_data = {}
                if angle_success == 1:
                    angle_regs = self.read_multiple_registers(ExternalSystemRegisters.ANGLE_ORIGINAL_HIGH, 6)
                    if angle_regs and len(angle_regs) >= 6:
                        # 原始角度 (32位, ×100精度)
                        original_angle_int = (angle_regs[0] << 16) | angle_regs[1]
                        if original_angle_int >= 2**31:
                            original_angle_int -= 2**32
                        angle_data["original_angle"] = original_angle_int / 100.0
                        
                        # 角度差 (32位, ×100精度)
                        angle_diff_int = (angle_regs[2] << 16) | angle_regs[3]
                        if angle_diff_int >= 2**31:
                            angle_diff_int -= 2**32
                        angle_data["angle_diff"] = angle_diff_int / 100.0
                        
                        # 馬達位置 (32位)
                        motor_pos_int = (angle_regs[4] << 16) | angle_regs[5]
                        if motor_pos_int >= 2**31:
                            motor_pos_int -= 2**32
                        angle_data["motor_position"] = motor_pos_int
                
                external_status["angle"] = {
                    "status_bits": self.get_status_bits(angle_status_reg),
                    "detection_success": bool(angle_success),
                    "angle_data": angle_data,
                    "connected": angle_status_reg is not None
                }
            
            # CCD3系統狀態
            if self.config["external_systems"]["monitor_ccd3"]:
                ccd3_status_reg = self.read_register(ExternalSystemRegisters.CCD3_STATUS)
                
                external_status["ccd3"] = {
                    "status_bits": self.get_status_bits(ccd3_status_reg),
                    "connected": ccd3_status_reg is not None
                }
                
        except Exception as e:
            print(f"讀取外部系統狀態失敗: {e}")
        
        return external_status
    
    def read_all_status(self) -> dict:
        """讀取所有狀態寄存器"""
        try:
            if not self.is_connected:
                return {}
                
            # 讀取Dobot狀態寄存器 (400-419)
            result = self.modbus_client.read_holding_registers(DobotRegisters.STATUS_REGISTER, count=20)
            
            if not (hasattr(result, 'registers') and len(result.registers) >= 20):
                return {}
            
            registers = result.registers
            status_bits = self.get_status_bits(registers[0])
            
            # 基本狀態數據
            status_data = {
                # 狀態機交握狀態
                "handshake_status": status_bits,
                "ready_for_command": self.is_ready_for_command(),
                
                # 機械臂狀態
                "robot_state": registers[1],
                "robot_state_text": ROBOT_STATES.get(registers[1], "未知"),
                "current_flow": registers[2],
                "current_flow_text": FLOW_TYPES.get(registers[2], "未知"),
                "flow_progress": registers[3],
                "error_code": registers[4],
                "robot_mode": registers[5],
                "global_speed": registers[19],
                
                # 位置資訊
                "position": {
                    "x": registers[6],
                    "y": registers[7],
                    "z": registers[8],
                    "r": registers[9]
                },
                
                # 關節角度
                "joints": {
                    "j1": registers[10] / 100.0,
                    "j2": registers[11] / 100.0,
                    "j3": registers[12] / 100.0,
                    "j4": registers[13] / 100.0
                },
                
                # IO狀態
                "io_status": {
                    "di": registers[14],
                    "do": registers[15]
                },
                
                # 統計資訊
                "statistics": {
                    "operation_count": registers[16],
                    "error_count": registers[17],
                    "run_time_minutes": registers[18]
                },
                
                "connection_status": self.is_connected,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            
            # 新增: 外部系統狀態
            external_systems = self.read_external_systems_status()
            status_data["external_systems"] = external_systems
            
            return status_data
            
        except Exception as e:
            print(f"讀取狀態失敗: {e}")
            return {}
    
    def read_control_registers(self) -> dict:
        """讀取控制寄存器狀態"""
        try:
            if not self.is_connected:
                return {}
            
            control_data = {
                "vp_control": self.read_register(DobotRegisters.VP_CONTROL),
                "unload_control": self.read_register(DobotRegisters.UNLOAD_CONTROL),
                "clear_alarm": self.read_register(DobotRegisters.CLEAR_ALARM),
                "emergency_stop": self.read_register(DobotRegisters.EMERGENCY_STOP),
                "manual_command": self.read_register(DobotRegisters.MANUAL_COMMAND),
                "speed_command": self.read_register(DobotRegisters.SPEED_COMMAND),
                "speed_value": self.read_register(DobotRegisters.SPEED_VALUE),
                "speed_cmd_id": self.read_register(DobotRegisters.SPEED_CMD_ID),
                "force_ready": self.read_register(DobotRegisters.FORCE_READY)
            }
            
            return control_data
            
        except Exception as e:
            print(f"讀取控制寄存器失敗: {e}")
            return {}
    
    # === 狀態機交握指令方法 ===
    
    def execute_vp_pickup(self) -> dict:
        """執行VP視覺取料 (Flow1)"""
        try:
            if not self.is_ready_for_command():
                return {
                    "success": False,
                    "message": "系統未Ready，無法執行VP視覺取料"
                }
            
            success = self.write_register(DobotRegisters.VP_CONTROL, 1)
            return {
                "success": success,
                "message": "VP視覺取料指令已發送" if success else "VP視覺取料指令發送失敗",
                "flow_description": "從CCD1佇列獲取物體座標並執行抓取動作"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"VP視覺取料執行失敗: {e}"
            }
    
    def execute_unload_flow(self) -> dict:
        """執行出料流程 (Flow2)"""
        try:
            if not self.is_ready_for_command():
                return {
                    "success": False,
                    "message": "系統未Ready，無法執行出料流程"
                }
            
            vp_control = self.read_register(DobotRegisters.VP_CONTROL)
            if vp_control != 0:
                return {
                    "success": False,
                    "message": f"VP控制未清零({vp_control})，無法執行出料流程"
                }
            
            success = self.write_register(DobotRegisters.UNLOAD_CONTROL, 1)
            return {
                "success": success,
                "message": "出料流程指令已發送" if success else "出料流程指令發送失敗",
                "flow_description": "從standby點開始的完整出料作業流程"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"出料流程執行失敗: {e}"
            }
    
    def clear_vp_pickup(self) -> dict:
        """清零VP視覺取料指令"""
        try:
            success = self.write_register(DobotRegisters.VP_CONTROL, 0)
            return {
                "success": success,
                "message": "VP視覺取料指令已清零" if success else "VP視覺取料指令清零失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"清零VP視覺取料指令失敗: {e}"
            }
    
    def clear_unload_flow(self) -> dict:
        """清零出料流程指令"""
        try:
            success = self.write_register(DobotRegisters.UNLOAD_CONTROL, 0)
            return {
                "success": success,
                "message": "出料流程指令已清零" if success else "出料流程指令清零失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"清零出料流程指令失敗: {e}"
            }
    
    def clear_alarm(self) -> dict:
        """清除警報"""
        try:
            success = self.write_register(DobotRegisters.CLEAR_ALARM, 1)
            return {
                "success": success,
                "message": "清除警報指令已發送" if success else "清除警報指令發送失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"清除警報失敗: {e}"
            }
    
    def emergency_stop(self) -> dict:
        """緊急停止"""
        try:
            success = self.write_register(DobotRegisters.EMERGENCY_STOP, 1)
            return {
                "success": success,
                "message": "緊急停止指令已發送" if success else "緊急停止指令發送失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"緊急停止失敗: {e}"
            }
    
    def manual_execute_flow(self, flow_id: int) -> dict:
        """Web端手動執行流程"""
        try:
            if not self.is_ready_for_command():
                return {
                    "success": False,
                    "message": "系統未Ready，無法執行手動指令"
                }
            
            success = self.write_register(DobotRegisters.MANUAL_COMMAND, flow_id)
            flow_name = FLOW_TYPES.get(flow_id, f"流程{flow_id}")
            
            return {
                "success": success,
                "message": f"手動執行{flow_name}指令已發送" if success else f"手動執行{flow_name}指令發送失敗"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"手動執行流程{flow_id}失敗: {e}"
            }
    
    def force_ready(self) -> dict:
        """強制設置Ready狀態 (測試用)"""
        try:
            success = self.write_register(DobotRegisters.FORCE_READY, 1)
            return {
                "success": success,
                "message": "強制Ready指令已發送 (測試用)" if success else "強制Ready指令發送失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"強制Ready失敗: {e}"
            }
    
    def set_global_speed(self, speed: int) -> dict:
        """設定全局速度"""
        try:
            min_speed = self.config["speed_control"]["min_speed"]
            max_speed = self.config["speed_control"]["max_speed"]
            
            if speed < min_speed or speed > max_speed:
                return {
                    "success": False,
                    "message": f"速度值超出範圍({min_speed}-{max_speed})"
                }
            
            self.speed_cmd_id_counter += 1
            if self.speed_cmd_id_counter > 65535:
                self.speed_cmd_id_counter = 1
            
            success1 = self.write_register(DobotRegisters.SPEED_VALUE, speed)
            success2 = self.write_register(DobotRegisters.SPEED_CMD_ID, self.speed_cmd_id_counter)
            success3 = self.write_register(DobotRegisters.SPEED_COMMAND, 1)
            
            success = success1 and success2 and success3
            
            return {
                "success": success,
                "message": f"全局速度設定為{speed}%" if success else "全局速度設定失敗",
                "speed_value": speed,
                "command_id": self.speed_cmd_id_counter
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"設定全局速度失敗: {e}"
            }
    
    def get_global_speed(self) -> dict:
        """獲取當前全局速度"""
        try:
            current_speed = self.read_register(DobotRegisters.GLOBAL_SPEED)
            if current_speed is None:
                return {
                    "success": False,
                    "message": "讀取全局速度失敗"
                }
            
            return {
                "success": True,
                "current_speed": current_speed,
                "message": f"當前全局速度: {current_speed}%"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"獲取全局速度失敗: {e}"
            }
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            print("狀態監控已啟動 (增強版含外部系統)")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        print("狀態監控已停止")
    
    def _monitor_loop(self):
        """監控循環"""
        while self.monitoring:
            try:
                if self.is_connected:
                    status = self.read_all_status()
                    if status:
                        socketio.emit('status_update', status)
                        
                time.sleep(self.config["monitoring"]["refresh_interval"])
                
            except Exception as e:
                print(f"監控循環錯誤: {e}")
                time.sleep(1)

# 初始化Flask應用
app = Flask(__name__)
app.config['SECRET_KEY'] = 'dobot_handshake_controller_enhanced_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# 初始化控制器
controller = DobotHandshakeController()

@app.route('/')
def index():
    """主頁面"""
    return render_template('dobot_handshake_index.html')

@app.route('/api/connect', methods=['POST'])
def connect():
    """連接Modbus服務器"""
    success = controller.connect_modbus()
    if success and controller.config["monitoring"]["auto_start"]:
        controller.start_monitoring()
    
    return jsonify({
        'success': success,
        'message': 'Modbus連接成功' if success else 'Modbus連接失敗'
    })

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """斷開Modbus連接"""
    controller.stop_monitoring()
    if controller.modbus_client:
        controller.modbus_client.close()
    controller.is_connected = False
    
    return jsonify({
        'success': True,
        'message': 'Modbus連接已斷開'
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    """獲取系統狀態"""
    status = controller.read_all_status()
    return jsonify(status)

@app.route('/api/control_registers', methods=['GET'])
def get_control_registers():
    """獲取控制寄存器狀態"""
    control_status = controller.read_control_registers()
    return jsonify(control_status)

@app.route('/api/external_systems', methods=['GET'])
def get_external_systems():
    """獲取外部系統狀態"""
    external_status = controller.read_external_systems_status()
    return jsonify(external_status)

# === 狀態機交握API路由 ===

@app.route('/api/handshake/vp_pickup', methods=['POST'])
def handshake_vp_pickup():
    """執行VP視覺取料"""
    result = controller.execute_vp_pickup()
    return jsonify(result)

@app.route('/api/handshake/unload_flow', methods=['POST'])
def handshake_unload_flow():
    """執行出料流程"""
    result = controller.execute_unload_flow()
    return jsonify(result)

@app.route('/api/handshake/clear_vp', methods=['POST'])
def handshake_clear_vp():
    """清零VP視覺取料指令"""
    result = controller.clear_vp_pickup()
    return jsonify(result)

@app.route('/api/handshake/clear_unload', methods=['POST'])
def handshake_clear_unload():
    """清零出料流程指令"""
    result = controller.clear_unload_flow()
    return jsonify(result)

@app.route('/api/handshake/clear_alarm', methods=['POST'])
def handshake_clear_alarm():
    """清除警報"""
    result = controller.clear_alarm()
    return jsonify(result)

@app.route('/api/handshake/emergency_stop', methods=['POST'])
def handshake_emergency_stop():
    """緊急停止"""
    result = controller.emergency_stop()
    return jsonify(result)

@app.route('/api/handshake/force_ready', methods=['POST'])
def handshake_force_ready():
    """強制Ready (測試用)"""
    result = controller.force_ready()
    return jsonify(result)

@app.route('/api/handshake/manual_flow/<int:flow_id>', methods=['POST'])
def handshake_manual_flow(flow_id):
    """手動執行流程"""
    result = controller.manual_execute_flow(flow_id)
    return jsonify(result)

@app.route('/api/handshake/set_speed', methods=['POST'])
def handshake_set_speed():
    """設定全局速度"""
    data = request.get_json()
    speed = data.get('speed')
    
    if speed is None:
        return jsonify({
            'success': False,
            'message': '缺少speed參數'
        })
    
    try:
        speed = int(speed)
        result = controller.set_global_speed(speed)
        return jsonify(result)
    except ValueError:
        return jsonify({
            'success': False,
            'message': 'speed必須是數字'
        })

@app.route('/api/handshake/get_speed', methods=['GET'])
def handshake_get_speed():
    """獲取當前全局速度"""
    result = controller.get_global_speed()
    return jsonify(result)

@app.route('/api/handshake/check_preconditions/<int:flow_id>', methods=['GET'])
def handshake_check_preconditions(flow_id):
    """檢查流程執行前置條件"""
    result = controller.check_flow_preconditions(flow_id)
    return jsonify(result)

# === SocketIO事件處理 ===

@socketio.on('connect')
def on_connect():
    """客戶端連接"""
    print('客戶端已連接')
    emit('connected', {'message': '已連接到Dobot狀態機交握控制器 (增強版)'})

@socketio.on('disconnect')
def on_disconnect():
    """客戶端斷開"""
    print('客戶端已斷開')

@socketio.on('request_status')
def on_request_status():
    """請求狀態更新"""
    status = controller.read_all_status()
    emit('status_update', status)

@socketio.on('request_control_status')
def on_request_control_status():
    """請求控制寄存器狀態"""
    control_status = controller.read_control_registers()
    emit('control_status_update', control_status)

@socketio.on('request_external_systems')
def on_request_external_systems():
    """請求外部系統狀態"""
    external_status = controller.read_external_systems_status()
    emit('external_systems_update', external_status)

@socketio.on('handshake_vp_pickup')
def on_handshake_vp_pickup():
    """SocketIO VP視覺取料"""
    result = controller.execute_vp_pickup()
    emit('handshake_result', result)

@socketio.on('handshake_unload_flow')
def on_handshake_unload_flow():
    """SocketIO 出料流程"""
    result = controller.execute_unload_flow()
    emit('handshake_result', result)

@socketio.on('handshake_force_ready')
def on_handshake_force_ready():
    """SocketIO 強制Ready"""
    result = controller.force_ready()
    emit('handshake_result', result)

@socketio.on('handshake_clear_alarm')
def on_handshake_clear_alarm():
    """SocketIO 清除警報"""
    result = controller.clear_alarm()
    emit('handshake_result', result)

@socketio.on('handshake_emergency_stop')
def on_handshake_emergency_stop():
    """SocketIO 緊急停止"""
    result = controller.emergency_stop()
    emit('handshake_result', result)

@socketio.on('handshake_manual_flow')
def on_handshake_manual_flow(data):
    """SocketIO 手動執行流程"""
    flow_id = data.get('flow_id')
    if flow_id is not None:
        result = controller.manual_execute_flow(int(flow_id))
        emit('handshake_result', result)

@socketio.on('handshake_set_speed')
def on_handshake_set_speed(data):
    """SocketIO 設定全局速度"""
    speed = data.get('speed')
    if speed is not None:
        result = controller.set_global_speed(int(speed))
        emit('speed_update_result', result)

@socketio.on('handshake_get_speed')
def on_handshake_get_speed():
    """SocketIO 獲取全局速度"""
    result = controller.get_global_speed()
    emit('speed_status_update', result)

def main():
    """主函數"""
    print("=== Dobot狀態機交握控制器Web應用啟動中 (增強版) ===")
    print(f"Web服務器: http://{controller.config['web_server']['host']}:{controller.config['web_server']['port']}")
    print(f"Modbus服務器: {controller.config['modbus']['server_ip']}:{controller.config['modbus']['server_port']}")
    
    print("\n=== Dobot狀態機交握寄存器映射 ===")
    print(f"主狀態寄存器: {DobotRegisters.STATUS_REGISTER} (bit0=Ready, bit1=Running, bit2=Alarm)")
    print(f"VP視覺取料控制: {DobotRegisters.VP_CONTROL}")
    print(f"出料控制: {DobotRegisters.UNLOAD_CONTROL}")
    print(f"強制Ready控制: {DobotRegisters.FORCE_READY} (新增測試功能)")
    
    print("\n=== 外部系統寄存器映射 ===")
    print(f"CCD1狀態: {ExternalSystemRegisters.CCD1_STATUS}, 佇列數量: {ExternalSystemRegisters.CCD1_CIRCLE_COUNT}")
    print(f"Angle狀態: {ExternalSystemRegisters.ANGLE_STATUS}, 檢測結果: {ExternalSystemRegisters.ANGLE_SUCCESS}")
    print(f"Angle原始角度: {ExternalSystemRegisters.ANGLE_ORIGINAL_HIGH}-{ExternalSystemRegisters.ANGLE_ORIGINAL_LOW}")
    print(f"Angle角度差: {ExternalSystemRegisters.ANGLE_DIFF_HIGH}-{ExternalSystemRegisters.ANGLE_DIFF_LOW}")
    print(f"CCD3狀態: {ExternalSystemRegisters.CCD3_STATUS}")
    
    print("\n=== 功能特色 ===")
    print("1. 完整的Dobot狀態機交握控制")
    print("2. CCD1佇列物體數量監控")
    print("3. Angle系統角度檢測結果監控")
    print("4. CCD3系統狀態監控")
    print("5. 強制Ready按鈕 (測試用)")
    print("6. 實時外部系統狀態顯示")
    print("7. 寄存器地址按鈕提示")
    
    print("\n請在瀏覽器中打開Web介面進行控制")
    
    try:
        socketio.run(
            app,
            host=controller.config['web_server']['host'],
            port=controller.config['web_server']['port'],
            debug=controller.config['web_server']['debug']
        )
    except KeyboardInterrupt:
        print("\n收到中斷信號，正在關閉...")
    finally:
        controller.stop_monitoring()
        if controller.modbus_client:
            controller.modbus_client.close()
        print("Web應用已關閉")

if __name__ == "__main__":
    main()