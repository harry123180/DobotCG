#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_CG_app.py - CG AutoProgram Web控制界面
提供CG AutoProgram流程控制、狀態監控、手動操作等功能
基於Flask + SocketIO架構
針對1350地址控制、Flow1/Flow5狀態監控與控制
"""

import os
import time
import json
import threading
from typing import Dict, Any, Optional
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException, ConnectionException

# 創建Flask應用
app = Flask(__name__)
app.config['SECRET_KEY'] = 'autoprogram_cg_v1.0'
socketio = SocketIO(app, cors_allowed_origins="*")

class AutoProgramCGWebController:
    """CG AutoProgram Web控制器"""
    
    def __init__(self, modbus_host="127.0.0.1", modbus_port=502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 狀態監控執行緒
        self.monitor_thread = None
        self.monitoring = False
        
        # 寄存器地址映射
        self.REGISTERS = {
            # AutoProgram控制
            'AUTO_PROGRAM_CONTROL': 1350,    # AutoProgram流程控制
            
            # 系統狀態
            'SYSTEM_STATUS': 1300,           # 系統狀態
            'AUTO_FEEDING_STATUS': 1301,     # AutoFeeding執行緒狀態
            'ROBOT_JOB_STATUS': 1302,        # RobotJob執行緒狀態
            
            # 統計資訊
            'CYCLE_COUNT': 1304,             # 週期計數
            'CG_F_FOUND_COUNT': 1305,        # CG_F找到次數
            'FLOW4_TRIGGER_COUNT': 1306,     # Flow4觸發次數
            'VP_VIBRATION_COUNT': 1307,      # VP震動次數
            'FLOW1_TRIGGER_COUNT': 1308,     # Flow1觸發次數
            'FLOW5_COMPLETE_COUNT': 1309,    # Flow5完成次數
            'FLOW4_CONSECUTIVE_COUNT': 1310, # 連續直振次數
            'VP_EMPTY_DETECTION_COUNT': 1311,# VP空檢測次數
            
            # 機械臂控制
            'MOTION_STATUS': 1200,           # 運動狀態寄存器
            'CURRENT_MOTION_FLOW': 1201,     # 當前運動Flow
            'MOTION_PROGRESS': 1202,         # 運動進度
            'FLOW1_CONTROL': 1240,           # Flow1控制
            'FLOW5_CONTROL': 1242,           # Flow5控制
            'FLOW1_COMPLETE': 1204,          # Flow1完成狀態
            'FLOW5_COMPLETE': 1206,          # Flow5完成狀態
            
            # CG檢測結果
            'CG_STATUS': 201,                # CG狀態
            'CG_F_COUNT': 240,               # CG_F數量
            'CG_B_COUNT': 241,               # CG_B數量
            'STACK_COUNT': 242,              # STACK數量
            'TOTAL_DETECTIONS': 243,         # 總檢測數量
            
            # VP狀態
            'VP_STATUS': 300,                # VP模組狀態
            'VP_DEVICE_CONNECTION': 301,     # VP設備連接
            'VP_VIBRATION_STATUS': 312,      # VP震動狀態
            
            # Flow4直振供應
            'FLOW4_CONTROL': 448,            # Flow4控制
        }
        
        # 自動交握狀態
        self.auto_handshake_running = False
        self.auto_handshake_thread = None
        
        print("CG AutoProgram Web控制器初始化完成")
    
    def connect_modbus(self) -> bool:
        """連接Modbus服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"✓ Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                # 啟動狀態監控
                self.start_monitoring()
            else:
                print(f"✗ Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
            
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect_modbus(self):
        """斷開Modbus連接"""
        self.stop_monitoring()
        
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            return None
            
        except Exception:
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            
            return not result.isError()
            
        except Exception:
            return False
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        status = {
            # 連接狀態
            'connected': self.connected,
            'modbus_host': self.modbus_host,
            'modbus_port': self.modbus_port,
            
            # AutoProgram控制
            'auto_program_enabled': bool(self.read_register('AUTO_PROGRAM_CONTROL')),
            
            # 執行緒狀態
            'auto_feeding_running': bool(self.read_register('AUTO_FEEDING_STATUS')),
            'robot_job_running': bool(self.read_register('ROBOT_JOB_STATUS')),
            
            # 機械臂狀態
            'motion_status': self.read_register('MOTION_STATUS') or 0,
            'current_motion_flow': self.read_register('CURRENT_MOTION_FLOW') or 0,
            'motion_progress': self.read_register('MOTION_PROGRESS') or 0,
            'flow1_complete': bool(self.read_register('FLOW1_COMPLETE')),
            'flow5_complete': bool(self.read_register('FLOW5_COMPLETE')),
            
            # 統計資訊
            'cycle_count': self.read_register('CYCLE_COUNT') or 0,
            'cg_f_found_count': self.read_register('CG_F_FOUND_COUNT') or 0,
            'flow4_trigger_count': self.read_register('FLOW4_TRIGGER_COUNT') or 0,
            'vp_vibration_count': self.read_register('VP_VIBRATION_COUNT') or 0,
            'flow1_trigger_count': self.read_register('FLOW1_TRIGGER_COUNT') or 0,
            'flow5_complete_count': self.read_register('FLOW5_COMPLETE_COUNT') or 0,
            'flow4_consecutive_count': self.read_register('FLOW4_CONSECUTIVE_COUNT') or 0,
            'vp_empty_detection_count': self.read_register('VP_EMPTY_DETECTION_COUNT') or 0,
            
            # CG檢測結果
            'cg_status': self.read_register('CG_STATUS') or 0,
            'cg_f_count': self.read_register('CG_F_COUNT') or 0,
            'cg_b_count': self.read_register('CG_B_COUNT') or 0,
            'stack_count': self.read_register('STACK_COUNT') or 0,
            'total_detections': self.read_register('TOTAL_DETECTIONS') or 0,
            
            # VP狀態
            'vp_status': self.read_register('VP_STATUS') or 0,
            'vp_device_connection': bool(self.read_register('VP_DEVICE_CONNECTION')),
            'vp_vibration_status': self.read_register('VP_VIBRATION_STATUS') or 0,
            
            # Flow4狀態
            'flow4_control': self.read_register('FLOW4_CONTROL') or 0,
            
            # 自動交握狀態
            'auto_handshake_running': self.auto_handshake_running,
            
            # 時間戳
            'timestamp': time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        # 判斷自動供料流程狀態
        status['feeding_process_status'] = self._get_feeding_process_status(status)
        
        return status
    
    def _get_feeding_process_status(self, status: Dict) -> str:
        """判斷自動供料流程狀態"""
        if not status['auto_program_enabled']:
            return "系統停止"
        
        if not status['auto_feeding_running']:
            return "入料程序未啟動"
        
        # 檢查VP震動狀態
        if status['vp_vibration_status'] > 0:
            return "VP震動中"
        
        # 檢查Flow4直振供應
        if status['flow4_control'] > 0:
            return "直振供應中"
        
        # 檢查CG辨識狀態
        cg_status = status['cg_status']
        if cg_status & 0x02:  # bit1=Running
            return "CG辨識中"
        
        # 默認狀態
        if status['auto_feeding_running']:
            return "自動供料運行中"
        
        return "等待中"
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("✓ 狀態監控已啟動")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        if self.monitoring:
            self.monitoring = False
            if self.monitor_thread and self.monitor_thread.is_alive():
                self.monitor_thread.join(timeout=2.0)
            print("狀態監控已停止")
    
    def _monitor_loop(self):
        """狀態監控循環"""
        while self.monitoring and self.connected:
            try:
                # 獲取系統狀態
                status = self.get_system_status()
                
                # 通過SocketIO發送狀態更新
                socketio.emit('status_update', status)
                
                # 2秒間隔
                time.sleep(2.0)
                
            except Exception as e:
                print(f"狀態監控異常: {e}")
                time.sleep(5.0)
    
    def auto_handshake_flow(self):
        """自動交握流程"""
        try:
            self.auto_handshake_running = True
            print("開始執行自動交握流程...")
            
            # 步驟1: 確認Flow1完成狀態為1
            step = 1
            print(f"步驟{step}: 檢查Flow1完成狀態...")
            flow1_complete = self.read_register('FLOW1_COMPLETE')
            
            if flow1_complete != 1:
                print(f"✗ Flow1完成狀態為{flow1_complete}，不等於1，自動交握中止")
                return False
            
            print(f"✓ Flow1完成狀態確認為1")
            
            # 步驟2: 清除Flow1完成狀態
            step += 1
            print(f"步驟{step}: 清除Flow1完成狀態...")
            if not self.write_register('FLOW1_COMPLETE', 0):
                print(f"✗ 清除Flow1完成狀態失敗")
                return False
            
            print(f"✓ Flow1完成狀態已清除")
            time.sleep(0.1)
            
            # 步驟3: 向Flow5地址寫1
            step += 1
            print(f"步驟{step}: 觸發Flow5...")
            if not self.write_register('FLOW5_CONTROL', 1):
                print(f"✗ 觸發Flow5失敗")
                return False
            
            print(f"✓ Flow5已觸發")
            
            # 步驟4: 等待當前運動Flow變為5 (確認Flow5開始執行)
            step += 1
            print(f"步驟{step}: 等待Flow5開始執行...")
            
            timeout = 10.0  # 10秒超時
            start_time = time.time()
            
            while (time.time() - start_time) < timeout:
                current_flow = self.read_register('CURRENT_MOTION_FLOW')
                if current_flow == 5:
                    print(f"✓ Flow5已開始執行 (當前運動Flow: {current_flow})")
                    break
                
                time.sleep(0.5)
            else:
                print(f"✗ 等待Flow5開始執行超時")
                return False
            
            # 步驟5: 清除Flow5控制地址
            step += 1
            print(f"步驟{step}: 清除Flow5控制地址...")
            if not self.write_register('FLOW5_CONTROL', 0):
                print(f"✗ 清除Flow5控制地址失敗")
                return False
            
            print(f"✓ Flow5控制地址已清除")
            print("✅ 自動交握流程執行完成")
            return True
            
        except Exception as e:
            print(f"自動交握流程異常: {e}")
            return False
        finally:
            self.auto_handshake_running = False
    
    def start_auto_handshake(self):
        """啟動自動交握流程"""
        if self.auto_handshake_running:
            return {'success': False, 'message': '自動交握流程已在執行中'}
        
        self.auto_handshake_thread = threading.Thread(
            target=self.auto_handshake_flow, 
            daemon=True
        )
        self.auto_handshake_thread.start()
        
        return {'success': True, 'message': '自動交握流程已啟動'}

# 創建全局控制器實例
controller = AutoProgramCGWebController()

# ==================== Flask路由 ====================

@app.route('/')
def index():
    """主頁面"""
    return render_template('AutoProgram.html')

@app.route('/test')
def test():
    return "CG AutoProgram Web Server is running!"

@app.route('/api/connect', methods=['POST'])
def connect_modbus():
    """連接Modbus服務器"""
    try:
        success = controller.connect_modbus()
        
        return jsonify({
            'success': success,
            'message': '連接成功' if success else '連接失敗',
            'status': controller.get_system_status() if success else None
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/disconnect', methods=['POST'])
def disconnect_modbus():
    """斷開Modbus連接"""
    try:
        controller.disconnect_modbus()
        
        return jsonify({
            'success': True,
            'message': '連接已斷開'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/status', methods=['GET'])
def get_status():
    """獲取系統狀態"""
    try:
        status = controller.get_system_status()
        return jsonify({'success': True, 'status': status})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/autoprogram', methods=['POST'])
def control_autoprogram():
    """控制AutoProgram流程"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'start' or 'stop'
        
        if action == 'start':
            success = controller.write_register('AUTO_PROGRAM_CONTROL', 1)
            message = 'CG AutoProgram流程已啟動 (1350=1)' if success else 'CG AutoProgram流程啟動失敗'
        elif action == 'stop':
            success = controller.write_register('AUTO_PROGRAM_CONTROL', 0)
            message = 'CG AutoProgram流程已停止 (1350=0)' if success else 'CG AutoProgram流程停止失敗'
        else:
            return jsonify({'success': False, 'message': '無效的操作'})
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/flow1', methods=['POST'])
def control_flow1():
    """控制Flow1完成狀態"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'clear'
        
        if action == 'clear':
            success = controller.write_register('FLOW1_COMPLETE', 0)
            message = 'Flow1完成狀態已清除 (1204=0)' if success else 'Flow1完成狀態清除失敗'
        else:
            return jsonify({'success': False, 'message': '無效的操作'})
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/flow5', methods=['POST'])
def control_flow5():
    """控制Flow5"""
    try:
        data = request.get_json()
        action = data.get('action')  # 'start' or 'clear'
        
        if action == 'start':
            success = controller.write_register('FLOW5_CONTROL', 1)
            message = 'Flow5已觸發 (1242=1)' if success else 'Flow5觸發失敗'
        elif action == 'clear':
            success = controller.write_register('FLOW5_CONTROL', 0)
            message = 'Flow5控制地址已清除 (1242=0)' if success else 'Flow5控制地址清除失敗'
        else:
            return jsonify({'success': False, 'message': '無效的操作'})
        
        return jsonify({
            'success': success,
            'message': message
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/api/control/auto_handshake', methods=['POST'])
def auto_handshake():
    """自動交握"""
    try:
        result = controller.start_auto_handshake()
        return jsonify(result)
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

# ==================== SocketIO事件處理 ====================

@socketio.on('connect')
def handle_connect():
    """客戶端連接"""
    print("客戶端已連接")
    emit('status_update', controller.get_system_status())

@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開"""
    print("客戶端已斷開連接")

@socketio.on('request_status')
def handle_request_status():
    """請求狀態更新"""
    emit('status_update', controller.get_system_status())

def main():
    """主函數"""
    print("=" * 60)
    print("CG AutoProgram Web控制界面啟動中...")
    print("CG AutoProgram流程控制與監控")
    print("=" * 60)
    
    # 檢查模板文件
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    template_file = os.path.join(template_dir, 'AutoProgram.html')
    
    if not os.path.exists(template_dir):
        os.makedirs(template_dir)
        print(f"已創建模板目錄: {template_dir}")
    
    if not os.path.exists(template_file):
        print(f"⚠️ 警告: 模板文件不存在 - {template_file}")
        print("請確保AutoProgram.html文件在templates目錄中")
    
    try:
        print("🌐 Web服務器啟動中...")
        print("📱 訪問地址: http://localhost:8062")
        print("🎯 功能特性:")
        print("   • CG AutoProgram流程控制 (1350地址)")
        print("   • Flow1/Flow5狀態監控與控制")
        print("   • 自動交握流程")
        print("   • 自動供料狀態監控")
        print("   • CG檢測結果顯示")
        print("   • 即時運動進度條")
        print("=" * 60)
        
        # 啟動Web服務器
        socketio.run(
            app,
            host='0.0.0.0',
            port=8062,
            debug=False
        )
        
    except KeyboardInterrupt:
        print("\n收到中斷信號，正在關閉...")
    except Exception as e:
        print(f"Web服務器錯誤: {e}")
    finally:
        # 清理資源
        controller.disconnect_modbus()
        print("Web服務器已關閉")

if __name__ == '__main__':
    main()