#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MVP_app.py - 機械臂狀態機Web控制界面
提供簡單的Web界面來觸發Modbus控制指令
"""

from flask import Flask, render_template, jsonify, request
from pymodbus.client.tcp import ModbusTcpClient
import threading
import time
import json

app = Flask(__name__)

# Modbus配置
MODBUS_IP = "127.0.0.1"
MODBUS_PORT = 502
SLAVE_ID = 1

# 機械臂狀態機地址（基地址400）
CONTROL_ADDR = 400  # 控制指令地址
RUNNING_ADDR = 401  # 運行狀態地址
STANDBY_ADDR = 402  # 準備狀態地址

class ModbusController:
    def __init__(self):
        self.client = None
        self.connected = False
        
    def connect(self):
        """連接Modbus服務器"""
        try:
            self.client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
            if self.client.connect():
                self.connected = True
                print(f"Modbus連接成功: {MODBUS_IP}:{MODBUS_PORT}")
                return True
            else:
                self.connected = False
                print("Modbus連接失敗")
                return False
        except Exception as e:
            self.connected = False
            print(f"Modbus連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.client:
            self.client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def read_register(self, address):
        """讀取寄存器"""
        if not self.connected:
            return None
            
        try:
            # 根據專案經驗，使用正確的API格式
            result = self.client.read_holding_registers(address, count=1)
            
            if hasattr(result, 'isError') and result.isError():
                return None
            elif hasattr(result, 'registers') and len(result.registers) > 0:
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            print(f"讀取寄存器{address}異常: {e}")
            return None
    
    def write_register(self, address, value):
        """寫入寄存器"""
        if not self.connected:
            return False
            
        try:
            # 根據專案經驗，使用正確的API格式
            result = self.client.write_register(address, value)
            
            if hasattr(result, 'isError') and result.isError():
                return False
            else:
                return True
                
        except Exception as e:
            print(f"寫入寄存器{address}={value}異常: {e}")
            return False

# 全局Modbus控制器
modbus_ctrl = ModbusController()

@app.route('/')
def index():
    """主頁面"""
    return render_template('index.html')

@app.route('/api/connect', methods=['POST'])
def connect_modbus():
    """連接Modbus服務器"""
    success = modbus_ctrl.connect()
    return jsonify({
        'success': success,
        'message': 'Modbus連接成功' if success else 'Modbus連接失敗'
    })

@app.route('/api/disconnect', methods=['POST'])
def disconnect_modbus():
    """斷開Modbus連接"""
    modbus_ctrl.disconnect()
    return jsonify({
        'success': True,
        'message': 'Modbus連接已斷開'
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    """獲取當前狀態"""
    if not modbus_ctrl.connected:
        return jsonify({
            'connected': False,
            'control': 0,
            'running': 0,
            'standby': 0
        })
    
    control_value = modbus_ctrl.read_register(CONTROL_ADDR)
    running_value = modbus_ctrl.read_register(RUNNING_ADDR)
    standby_value = modbus_ctrl.read_register(STANDBY_ADDR)
    
    return jsonify({
        'connected': True,
        'control': control_value if control_value is not None else 0,
        'running': running_value if running_value is not None else 0,
        'standby': standby_value if standby_value is not None else 0
    })

@app.route('/api/trigger', methods=['POST'])
def trigger_action():
    """觸發機械臂動作"""
    if not modbus_ctrl.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus未連接'
        })
    
    # 檢查是否處於準備狀態
    standby_value = modbus_ctrl.read_register(STANDBY_ADDR)
    running_value = modbus_ctrl.read_register(RUNNING_ADDR)
    
    if running_value == 1:
        return jsonify({
            'success': False,
            'message': '機械臂正在運行中，請等待完成'
        })
    
    if standby_value != 1:
        return jsonify({
            'success': False,
            'message': '機械臂未處於準備狀態'
        })
    
    # 發送控制指令
    success = modbus_ctrl.write_register(CONTROL_ADDR, 1)
    
    return jsonify({
        'success': success,
        'message': '指令發送成功' if success else '指令發送失敗'
    })

@app.route('/api/stop', methods=['POST'])
def stop_action():
    """停止動作（清除控制指令）"""
    if not modbus_ctrl.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus未連接'
        })
    
    success = modbus_ctrl.write_register(CONTROL_ADDR, 0)
    
    return jsonify({
        'success': success,
        'message': '停止指令發送成功' if success else '停止指令發送失敗'
    })

@app.route('/api/reset', methods=['POST'])
def reset_status():
    """重置狀態寄存器"""
    if not modbus_ctrl.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus未連接'
        })
    
    # 清除所有狀態
    success1 = modbus_ctrl.write_register(CONTROL_ADDR, 0)
    success2 = modbus_ctrl.write_register(RUNNING_ADDR, 0)
    success3 = modbus_ctrl.write_register(STANDBY_ADDR, 1)
    
    success = success1 and success2 and success3
    
    return jsonify({
        'success': success,
        'message': '狀態重置成功' if success else '狀態重置失敗'
    })

@app.route('/api/registers', methods=['GET'])
def get_registers():
    """獲取詳細寄存器資訊"""
    if not modbus_ctrl.connected:
        return jsonify({
            'success': False,
            'message': 'Modbus未連接'
        })
    
    registers = {}
    for addr in range(400, 410):  # 讀取400-409範圍的寄存器
        value = modbus_ctrl.read_register(addr)
        registers[addr] = value if value is not None else 'Error'
    
    return jsonify({
        'success': True,
        'registers': registers
    })

def cleanup():
    """程序退出時清理資源"""
    modbus_ctrl.disconnect()

if __name__ == '__main__':
    try:
        print("=== 機械臂狀態機Web控制界面 ===")
        print(f"Modbus目標: {MODBUS_IP}:{MODBUS_PORT}")
        print(f"控制地址: {CONTROL_ADDR}")
        print(f"運行地址: {RUNNING_ADDR}")
        print(f"準備地址: {STANDBY_ADDR}")
        print("啟動Web服務器...")
        
        app.run(host='0.0.0.0', port=5100, debug=True)
        
    except KeyboardInterrupt:
        print("\n程序中斷")
    finally:
        cleanup()