#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_app.py - DobotFlow控制介面 (New Architecture版)
整合CCD1檢測、CCD3角度檢測、Flow控制、夾爪控制
寄存器映射: Dobot(400), CCD1(200), CCD3(800), 夾爪(500)
"""

import os
import json
import time
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient

# 配置檔案
CONFIG_FILE = "dobot_flow_config.json"

# ==================== 寄存器映射 ====================

class DobotRegisters:
    """Dobot新架構寄存器映射 - 混合交握協議"""
    
    # === 運動類Flow寄存器 (基地址1200-1249) ===
    # 運動狀態寄存器 (1200-1219) - 只讀
    MOTION_STATUS = 1200      # 運動狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)
    CURRENT_MOTION_FLOW = 1201 # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
    MOTION_PROGRESS = 1202    # 運動進度 (0-100百分比)
    MOTION_ERROR_CODE = 1203  # 運動錯誤碼
    FLOW1_COMPLETE = 1204     # Flow1完成狀態 (0=未完成, 1=完成且角度校正成功)
    FLOW2_COMPLETE = 1205     # Flow2完成狀態
    FLOW5_COMPLETE = 1206     # Flow5完成狀態
    MOTION_OP_COUNT = 1207    # 運動操作計數
    MOTION_ERR_COUNT = 1208   # 運動錯誤計數
    MOTION_RUN_TIME = 1209    # 運動系統運行時間
    
    # 運動控制寄存器 (1240-1249) - 讀寫
    FLOW1_CONTROL = 1240      # Flow1控制 (VP視覺取料) - 運動類
    FLOW2_CONTROL = 1241      # Flow2控制 (CV出料流程) - 運動類
    FLOW5_CONTROL = 1242      # Flow5控制 (機械臂運轉流程) - 運動類
    MOTION_CLEAR_ALARM = 1243 # 運動清除警報
    MOTION_EMERGENCY_STOP = 1244 # 運動緊急停止
    
    # === IO類Flow寄存器 (447-449) - 併行執行 ===
    FLOW3_CONTROL = 447       # Flow3控制 (翻轉站) - IO類併行
    FLOW4_CONTROL = 448       # Flow4控制 (震動投料) - IO類併行

class CCD1Registers:
    """CCD1視覺檢測寄存器 (基地址200)"""
    CONTROL_COMMAND = 200     # 控制指令 (0=清空, 16=拍照+檢測)
    STATUS_REGISTER = 201     # 狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm)
    CIRCLE_COUNT = 240        # 檢測圓形數量
    WORLD_COORD_VALID = 256   # 世界座標有效標誌

class CCD3Registers:
    """CCD3角度檢測寄存器 (基地址800)"""
    CONTROL_COMMAND = 800     # 控制指令 (0=清空, 16=拍照+檢測)
    STATUS_REGISTER = 801     # 狀態寄存器
    SUCCESS_FLAG = 840        # 檢測成功標誌
    CENTER_X = 841           # 物體中心X座標
    CENTER_Y = 842           # 物體中心Y座標
    ANGLE_HIGH = 843         # 角度高位
    ANGLE_LOW = 844          # 角度低位
    CONTOUR_AREA = 849       # 輪廓面積

class GripperRegisters:
    """夾爪控制寄存器 (基地址500)"""
    MODULE_STATUS = 500       # 模組狀態
    CONNECT_STATUS = 501      # 連接狀態
    CURRENT_POSITION = 505    # 當前位置
    COMMAND = 520            # 指令代碼
    PARAM1 = 521             # 參數1
    COMMAND_ID = 523         # 指令ID

# ==================== Web應用類 ====================

class DobotFlowController:
    """DobotFlow控制器"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        self.modbus_client = None
        self.is_connected = False
        self.monitoring = False
        self.monitor_thread = None
        self.command_id_counter = 1
        
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
                "port": 5058,
                "debug": False
            },
            "monitoring": {
                "refresh_interval": 2.0,
                "auto_start": True
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
            
            print(f"連接Modbus服務器: {self.config['modbus']['server_ip']}:{self.config['modbus']['server_port']}")
            
            self.modbus_client = ModbusTcpClient(
                host=self.config['modbus']['server_ip'],
                port=self.config['modbus']['server_port'],
                timeout=self.config['modbus']['timeout']
            )
            
            if self.modbus_client.connect():
                self.is_connected = True
                print("Modbus連接成功")
                
                if self.config['monitoring']['auto_start']:
                    self.start_monitoring()
                
                return True
            else:
                print("Modbus連接失敗")
                return False
                
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            return False
    
    def disconnect_modbus(self):
        """斷開Modbus連接"""
        self.stop_monitoring()
        
        if self.modbus_client and self.is_connected:
            try:
                self.modbus_client.close()
                print("Modbus連接已斷開")
            except:
                pass
        
        self.is_connected = False
        self.modbus_client = None
    
    def read_register(self, address: int) -> int:
        """讀取寄存器"""
        if not self.is_connected or not self.modbus_client:
            return None
        
        try:
            result = self.modbus_client.read_holding_registers(address, count=1)
            if not result.isError():
                return result.registers[0]
            return None
        except:
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入寄存器"""
        if not self.is_connected or not self.modbus_client:
            return False
        
        try:
            result = self.modbus_client.write_register(address, value)
            return not result.isError()
        except:
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
            self.monitor_thread.join(timeout=1)
            print("狀態監控已停止")
    
    def _monitor_loop(self):
        """監控循環"""
        while self.monitoring:
            try:
                status_data = self.get_system_status()
                socketio.emit('status_update', status_data)
                time.sleep(self.config['monitoring']['refresh_interval'])
            except Exception as e:
                print(f"監控循環異常: {e}")
                time.sleep(1)
    
    def get_system_status(self) -> dict:
        """獲取系統狀態"""
        try:
            # 新架構運動系統狀態 (基地址1200)
            motion_status = self.read_register(DobotRegisters.MOTION_STATUS) or 0
            motion_ready = bool(motion_status & 1)
            motion_running = bool(motion_status & 2)
            motion_alarm = bool(motion_status & 4)
            motion_initialized = bool(motion_status & 8)
            
            current_motion_flow = self.read_register(DobotRegisters.CURRENT_MOTION_FLOW) or 0
            motion_progress = self.read_register(DobotRegisters.MOTION_PROGRESS) or 0
            
            # CCD1狀態
            ccd1_status = self.read_register(CCD1Registers.STATUS_REGISTER) or 0
            ccd1_ready = bool(ccd1_status & 1)
            ccd1_running = bool(ccd1_status & 2)
            ccd1_count = self.read_register(CCD1Registers.CIRCLE_COUNT) or 0
            
            # CCD3狀態
            ccd3_status = self.read_register(CCD3Registers.STATUS_REGISTER) or 0
            ccd3_ready = bool(ccd3_status & 1)
            ccd3_running = bool(ccd3_status & 2)
            ccd3_result = self.read_register(CCD3Registers.SUCCESS_FLAG) or 0
            
            # 夾爪狀態
            gripper_connected = self.read_register(GripperRegisters.CONNECT_STATUS) or 0
            gripper_position = self.read_register(GripperRegisters.CURRENT_POSITION) or 0
            
            # Flow完成狀態 (新架構)
            flow1_complete = self.read_register(DobotRegisters.FLOW1_COMPLETE) or 0
            flow2_complete = self.read_register(DobotRegisters.FLOW2_COMPLETE) or 0
            flow5_complete = self.read_register(DobotRegisters.FLOW5_COMPLETE) or 0
            
            return {
                'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                'motion': {
                    'status': motion_status,
                    'ready': motion_ready,
                    'running': motion_running,
                    'alarm': motion_alarm,
                    'initialized': motion_initialized,
                    'current_flow': current_motion_flow,
                    'progress': motion_progress
                },
                'ccd1': {
                    'connected': self.is_connected,
                    'ready': ccd1_ready,
                    'running': ccd1_running,
                    'count': ccd1_count
                },
                'ccd3': {
                    'connected': self.is_connected,
                    'ready': ccd3_ready,
                    'running': ccd3_running,
                    'result': ccd3_result
                },
                'gripper': {
                    'connected': bool(gripper_connected),
                    'position': gripper_position
                },
                'flows': {
                    'flow1_complete': flow1_complete,
                    'flow2_complete': flow2_complete,
                    'flow5_complete': flow5_complete
                }
            }
            
        except Exception as e:
            print(f"獲取系統狀態失敗: {e}")
            return {
                'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                'error': str(e)
            }
    
    # ==================== CCD控制方法 ====================
    
    def execute_ccd1_detection(self) -> dict:
        """執行CCD1拍照檢測"""
        try:
            # 檢查CCD1 Ready狀態
            ccd1_status = self.read_register(CCD1Registers.STATUS_REGISTER) or 0
            if not (ccd1_status & 1):  # Ready bit
                return {'success': False, 'message': 'CCD1系統未Ready'}
            
            # 發送檢測指令
            if not self.write_register(CCD1Registers.CONTROL_COMMAND, 16):
                return {'success': False, 'message': 'CCD1指令發送失敗'}
            
            # 等待執行完成 (最多10秒)
            for _ in range(100):
                time.sleep(0.1)
                status = self.read_register(CCD1Registers.STATUS_REGISTER) or 0
                if not (status & 2):  # Running bit清除
                    break
            else:
                return {'success': False, 'message': 'CCD1檢測超時'}
            
            # 讀取檢測結果
            count = self.read_register(CCD1Registers.CIRCLE_COUNT) or 0
            
            return {
                'success': True,
                'message': f'CCD1檢測完成',
                'count': count
            }
            
        except Exception as e:
            return {'success': False, 'message': f'CCD1檢測異常: {str(e)}'}
    
    def clear_ccd1_control(self) -> dict:
        """清除CCD1控制寄存器"""
        try:
            if self.write_register(CCD1Registers.CONTROL_COMMAND, 0):
                return {'success': True, 'message': 'CCD1控制寄存器已清除'}
            else:
                return {'success': False, 'message': 'CCD1控制寄存器清除失敗'}
        except Exception as e:
            return {'success': False, 'message': f'CCD1清除異常: {str(e)}'}
    
    def execute_ccd3_detection(self) -> dict:
        """執行CCD3角度檢測"""
        try:
            # 檢查CCD3 Ready狀態
            ccd3_status = self.read_register(CCD3Registers.STATUS_REGISTER) or 0
            if not (ccd3_status & 1):  # Ready bit
                return {'success': False, 'message': 'CCD3系統未Ready'}
            
            # 發送檢測指令
            if not self.write_register(CCD3Registers.CONTROL_COMMAND, 16):
                return {'success': False, 'message': 'CCD3指令發送失敗'}
            
            # 等待執行完成
            for _ in range(100):
                time.sleep(0.1)
                status = self.read_register(CCD3Registers.STATUS_REGISTER) or 0
                if not (status & 2):  # Running bit清除
                    break
            else:
                return {'success': False, 'message': 'CCD3檢測超時'}
            
            # 讀取檢測結果
            success_flag = self.read_register(CCD3Registers.SUCCESS_FLAG) or 0
            center_x = self.read_register(CCD3Registers.CENTER_X) or 0
            center_y = self.read_register(CCD3Registers.CENTER_Y) or 0
            angle_high = self.read_register(CCD3Registers.ANGLE_HIGH) or 0
            angle_low = self.read_register(CCD3Registers.ANGLE_LOW) or 0
            area = self.read_register(CCD3Registers.CONTOUR_AREA) or 0
            
            # 處理32位角度值
            angle_int = (angle_high << 16) | angle_low
            if angle_int > 2147483647:
                angle_int -= 4294967296
            angle = angle_int / 100.0
            
            if success_flag:
                return {
                    'success': True,
                    'message': 'CCD3角度檢測完成',
                    'angle': angle,
                    'center': [center_x, center_y],
                    'area': area
                }
            else:
                return {'success': False, 'message': 'CCD3檢測失敗，未找到有效物體'}
            
        except Exception as e:
            return {'success': False, 'message': f'CCD3檢測異常: {str(e)}'}
    
    # ==================== Flow控制方法 ====================
    
    def execute_flow(self, flow_number: int) -> dict:
        """執行Flow控制 - 新架構混合交握協議"""
        try:
            # 新架構Flow分類處理
            if flow_number in [1, 2, 5]:
                # 運動類Flow (需要狀態機交握)
                return self._execute_motion_flow(flow_number)
            elif flow_number in [3, 4]:
                # IO類Flow (併行執行)
                return self._execute_io_flow(flow_number)
            else:
                return {'success': False, 'message': f'不支援的Flow編號: {flow_number}'}
                
        except Exception as e:
            return {'success': False, 'message': f'Flow{flow_number}執行異常: {str(e)}'}
    
    def _execute_motion_flow(self, flow_number: int) -> dict:
        """執行運動類Flow (狀態機交握)"""
        # 運動類Flow寄存器映射 (基地址1200)
        motion_flow_registers = {
            1: DobotRegisters.FLOW1_CONTROL,  # 1240: VP視覺取料
            2: DobotRegisters.FLOW2_CONTROL,  # 1241: CV出料流程  
            5: DobotRegisters.FLOW5_CONTROL   # 1242: 機械臂運轉流程
        }
        
        flow_names = {
            1: 'VP視覺取料',
            2: 'CV出料流程', 
            5: '機械臂運轉流程'
        }
        
        # 檢查運動系統Ready狀態 (1200寄存器)
        motion_status = self.read_register(DobotRegisters.MOTION_STATUS) or 0
        if not (motion_status & 1):  # Ready bit
            return {
                'success': False, 
                'message': f'運動系統未Ready (狀態={motion_status})，無法執行{flow_names[flow_number]}'
            }
        
        # 發送運動Flow指令
        register_addr = motion_flow_registers[flow_number]
        if not self.write_register(register_addr, 1):
            return {
                'success': False, 
                'message': f'運動Flow{flow_number}({flow_names[flow_number]})指令發送失敗'
            }
        
        return {
            'success': True, 
            'message': f'運動Flow{flow_number}({flow_names[flow_number]})指令已發送，等待狀態機交握'
        }
    
    def _execute_io_flow(self, flow_number: int) -> dict:
        """執行IO類Flow (併行執行)"""
        # IO類Flow寄存器映射 (447-448)
        io_flow_registers = {
            3: DobotRegisters.FLOW3_CONTROL,  # 447: 翻轉站
            4: DobotRegisters.FLOW4_CONTROL   # 448: 震動投料
        }
        
        flow_names = {
            3: '翻轉站',
            4: '震動投料'
        }
        
        # IO類Flow可併行執行，無需檢查狀態機Ready
        register_addr = io_flow_registers[flow_number]
        if not self.write_register(register_addr, 1):
            return {
                'success': False, 
                'message': f'IO Flow{flow_number}({flow_names[flow_number]})指令發送失敗'
            }
        
        return {
            'success': True, 
            'message': f'IO Flow{flow_number}({flow_names[flow_number]})指令已發送，併行執行中'
        }
    
    def clear_flow(self, flow_number: int) -> dict:
        """清除Flow控制寄存器 - 新架構混合交握協議"""
        try:
            # 新架構Flow分類處理
            if flow_number in [1, 2, 5]:
                # 運動類Flow清除
                return self._clear_motion_flow(flow_number)
            elif flow_number in [3, 4]:
                # IO類Flow清除
                return self._clear_io_flow(flow_number)
            else:
                return {'success': False, 'message': f'不支援的Flow編號: {flow_number}'}
                
        except Exception as e:
            return {'success': False, 'message': f'Flow{flow_number}清除異常: {str(e)}'}
    
    def _clear_motion_flow(self, flow_number: int) -> dict:
        """清除運動類Flow控制寄存器"""
        motion_flow_registers = {
            1: DobotRegisters.FLOW1_CONTROL,  # 1240
            2: DobotRegisters.FLOW2_CONTROL,  # 1241
            5: DobotRegisters.FLOW5_CONTROL   # 1242
        }
        
        flow_names = {
            1: 'VP視覺取料',
            2: 'CV出料流程',
            5: '機械臂運轉流程'
        }
        
        register_addr = motion_flow_registers[flow_number]
        if self.write_register(register_addr, 0):
            return {
                'success': True, 
                'message': f'運動Flow{flow_number}({flow_names[flow_number]})控制寄存器已清除'
            }
        else:
            return {
                'success': False, 
                'message': f'運動Flow{flow_number}({flow_names[flow_number]})控制寄存器清除失敗'
            }
    
    def _clear_io_flow(self, flow_number: int) -> dict:
        """清除IO類Flow控制寄存器"""
        io_flow_registers = {
            3: DobotRegisters.FLOW3_CONTROL,  # 447
            4: DobotRegisters.FLOW4_CONTROL   # 448
        }
        
        flow_names = {
            3: '翻轉站',
            4: '震動投料'
        }
        
        register_addr = io_flow_registers[flow_number]
        if self.write_register(register_addr, 0):
            return {
                'success': True, 
                'message': f'IO Flow{flow_number}({flow_names[flow_number]})控制寄存器已清除'
            }
        else:
            return {
                'success': False, 
                'message': f'IO Flow{flow_number}({flow_names[flow_number]})控制寄存器清除失敗'
            }
    
    # ==================== 夾爪控制方法 ====================
    
    def gripper_quick_close(self) -> dict:
        """夾爪快速關閉"""
        try:
            # 檢查夾爪連接狀態
            connect_status = self.read_register(GripperRegisters.CONNECT_STATUS) or 0
            if not connect_status:
                return {'success': False, 'message': '夾爪未連接'}
            
            # 發送快速關閉指令 (指令碼8)
            cmd_id = self.command_id_counter
            self.command_id_counter += 1
            
            # 寫入指令參數
            if not self.write_register(GripperRegisters.PARAM1, 0):
                return {'success': False, 'message': '夾爪參數寫入失敗'}
            
            if not self.write_register(GripperRegisters.COMMAND_ID, cmd_id):
                return {'success': False, 'message': '夾爪指令ID寫入失敗'}
            
            # 發送快速關閉指令
            if not self.write_register(GripperRegisters.COMMAND, 8):
                return {'success': False, 'message': '夾爪快速關閉指令發送失敗'}
            
            return {'success': True, 'message': '夾爪快速關閉指令已發送'}
            
        except Exception as e:
            return {'success': False, 'message': f'夾爪快速關閉異常: {str(e)}'}

# ==================== Flask應用設置 ====================

app = Flask(__name__)
app.config['SECRET_KEY'] = 'dobot_flow_secret_key_2024'
socketio = SocketIO(app, cors_allowed_origins="*")

# 全域控制器實例
controller = DobotFlowController()

@app.route('/')
def index():
    """首頁"""
    return render_template('DobotFlowUI.html')

@app.route('/connect', methods=['POST'])
def connect():
    """連接Modbus服務器"""
    success = controller.connect_modbus()
    return jsonify({'success': success})

@app.route('/disconnect', methods=['POST'])
def disconnect():
    """斷開Modbus連接"""
    controller.disconnect_modbus()
    return jsonify({'success': True})

@app.route('/status')
def get_status():
    """獲取系統狀態"""
    status = controller.get_system_status()
    return jsonify(status)

# ==================== SocketIO事件處理 ====================

@socketio.on('connect')
def handle_connect():
    """客戶端連接事件"""
    print('客戶端已連接')
    emit('connected', {'message': '連接成功'})

@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開事件"""
    print('客戶端已斷開')

@socketio.on('get_status')
def handle_get_status():
    """獲取狀態事件"""
    status = controller.get_system_status()
    emit('status_update', status)

@socketio.on('ccd1_detection')
def handle_ccd1_detection():
    """CCD1檢測事件"""
    result = controller.execute_ccd1_detection()
    emit('ccd1_result', result)

@socketio.on('ccd1_clear_control')
def handle_ccd1_clear():
    """CCD1清除控制事件"""
    result = controller.clear_ccd1_control()
    emit('ccd1_clear_result', result)

@socketio.on('ccd3_detection')
def handle_ccd3_detection():
    """CCD3檢測事件"""
    result = controller.execute_ccd3_detection()
    emit('ccd3_result', result)

@socketio.on('flow_control')
def handle_flow_control(data):
    """Flow控制事件"""
    flow_number = data.get('flow_number')
    action = data.get('action', 'start')
    
    if action == 'start':
        result = controller.execute_flow(flow_number)
    elif action == 'clear':
        result = controller.clear_flow(flow_number)
    else:
        result = {'success': False, 'message': f'不支援的動作: {action}'}
    
    emit('flow_result', result)

@socketio.on('gripper_quick_close')
def handle_gripper_quick_close():
    """夾爪快速關閉事件"""
    result = controller.gripper_quick_close()
    emit('gripper_result', result)

# ==================== 主程序 ====================

def main():
    """主程序"""
    print("DobotFlow控制介面啟動中...")
    print(f"Modbus服務器: {controller.config['modbus']['server_ip']}:{controller.config['modbus']['server_port']}")
    print(f"Web服務器: http://{controller.config['web_server']['host']}:{controller.config['web_server']['port']}")
    print("系統架構: New Architecture 混合交握協議")
    print("  運動類Flow: Flow1(1240), Flow2(1241), Flow5(1242) - 狀態機交握")
    print("  IO類Flow: Flow3(447), Flow4(448) - 併行執行")
    print("  CCD1視覺: 基地址200 | CCD3角度: 基地址800 | 夾爪: 基地址500")
    
    try:
        # 自動連接Modbus
        if controller.connect_modbus():
            print("Modbus自動連接成功")
        else:
            print("Modbus自動連接失敗，請在Web介面手動連接")
        
        # 啟動Web服務器
        socketio.run(
            app,
            host=controller.config['web_server']['host'],
            port=controller.config['web_server']['port'],
            debug=controller.config['web_server']['debug']
        )
        
    except KeyboardInterrupt:
        print("\n收到中斷信號，正在關閉...")
    except Exception as e:
        print(f"應用程式異常: {e}")
    finally:
        controller.disconnect_modbus()
        print("DobotFlow控制介面已關閉")

if __name__ == '__main__':
    main()