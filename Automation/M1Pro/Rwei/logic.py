#logic
import os
import json
import time
import socket
import threading
from datetime import datetime
from threading import Thread
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal, Qt
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from pymodbus.client import ModbusTcpClient

# 機械臂模式定義
LABEL_ROBOT_MODE = {
    1: "ROBOT_MODE_INIT",
    2: "ROBOT_MODE_BRAKE_OPEN", 
    3: "",
    4: "ROBOT_MODE_DISABLED",
    5: "ROBOT_MODE_ENABLE",
    6: "ROBOT_MODE_BACKDRIVE",
    7: "ROBOT_MODE_RUNNING",
    8: "ROBOT_MODE_RECORDING",
    9: "ROBOT_MODE_ERROR",
    10: "ROBOT_MODE_PAUSE",
    11: "ROBOT_MODE_JOG"
}

class RobotController(QObject):
    """機械臂控制邏輯"""
    
    # 信號定義
    feedback_update = pyqtSignal(dict)
    log_update = pyqtSignal(str)
    error_update = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    enable_changed = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        
        # 狀態變量
        self.global_state = {
            'connect': False,
            'enable': False
        }
        
        # 連接客戶端
        self.client_dash = None
        self.client_move = None
        self.client_feed = None
        self.modbus_client = None
        self.feedback_thread = None
        
        # 反饋狀態追蹤
        self.feedback_count = 0
        self.last_feedback_time = time.time()
        self.feedback_active = False
        
        # 性能監控
        self.performance_timer = None
        self.last_feedback_count = 0
        
        # 實際機械臂位置數據
        self.current_position = {
            'cartesian': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'r': 0.0},
            'joint': {'j1': 0.0, 'j2': 0.0, 'j3': 0.0, 'j4': 0.0}
        }
        
        # 點位數據管理
        self.saved_points = []
        self.points_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                       'saved_points', 'robot_points.json')
        
        # 確保資料夾存在
        os.makedirs(os.path.dirname(self.points_file), exist_ok=True)
        self.load_points()
        
    def emit_log(self, message):
        """發送日誌信號"""
        self.log_update.emit(message)
        
    def emit_error(self, message):
        """發送錯誤信號"""
        self.error_update.emit(message)
        
    # ==================== 連接管理 ====================
    
    def connect_robot(self, ip, dash_port, move_port, feed_port):
        """連接機械臂 - 增強版本"""
        try:
            self.emit_log("正在連接機械臂...")
            
            # 建立連接
            self.client_dash = DobotApiDashboard(ip, dash_port)
            self.client_move = DobotApiMove(ip, move_port)
            self.client_feed = DobotApi(ip, feed_port)
            
            # 測試Dashboard連接
            try:
                result = self.client_dash.RobotMode()
                self.emit_log(f"Dashboard連接測試成功: {result}")
            except Exception as e:
                raise Exception(f"Dashboard連接失敗: {str(e)}")
            
            # 檢查反饋連接的socket狀態
            if not hasattr(self.client_feed, 'socket_dobot'):
                raise Exception("反饋客戶端socket未正確建立")
            
            # 測試反饋數據接收
            try:
                self.client_feed.socket_dobot.settimeout(5.0)
                test_data = self.client_feed.socket_dobot.recv(1440)
                if len(test_data) == 1440:
                    self.emit_log("反饋端口數據接收測試成功")
                    # 解析測試數據驗證格式
                    a = np.frombuffer(test_data, dtype=MyType)
                    test_value = a['test_value'][0]
                    self.emit_log(f"反饋數據格式驗證 - test_value: {hex(test_value)}")
                else:
                    self.emit_log(f"反饋數據長度異常: {len(test_data)}")
            except Exception as e:
                self.emit_log(f"反饋連接測試警告: {str(e)}")
            
            # 連接Modbus TCP用於PGC夾爪控制
            try:
                self.modbus_client = ModbusTcpClient(host='127.0.0.1', port=502)
                if self.modbus_client.connect():
                    self.emit_log("Modbus TCP連接成功")
                    # 測試讀取PGC狀態寄存器
                    result = self.modbus_client.read_holding_registers(address=500, count=1, slave=1)
                    if not result.isError():
                        self.emit_log("PGC夾爪寄存器讀取成功")
                    else:
                        self.emit_log(f"PGC夾爪寄存器讀取失敗: {result}")
                else:
                    self.emit_log("Modbus TCP連接失敗")
            except Exception as e:
                self.emit_log(f"Modbus TCP連接錯誤: {str(e)}")
            
            self.global_state['connect'] = True
            self.connection_changed.emit(True)
            
            # 啟動反饋線程
            self.start_feedback_thread()
            
            self.emit_log("機械臂連接成功")
            return True
            
        except Exception as e:
            self.emit_log(f"連接失敗: {str(e)}")
            self.global_state['connect'] = False
            self.connection_changed.emit(False)
            return False
            
    def disconnect_robot(self):
        """斷開機械臂連接"""
        try:
            # 停止反饋線程
            self.global_state['connect'] = False
            self.feedback_active = False
            
            # 停止性能監控
            self.stop_performance_monitor()
            
            # 等待反饋線程結束
            if self.feedback_thread and self.feedback_thread.is_alive():
                self.feedback_thread.join(timeout=2.0)
            
            if self.client_dash:
                self.client_dash.close()
            if self.client_move:
                self.client_move.close()
            if self.client_feed:
                self.client_feed.close()
            if self.modbus_client:
                self.modbus_client.close()
                
            self.global_state['enable'] = False
            self.connection_changed.emit(False)
            self.enable_changed.emit(False)
            
            self.emit_log("機械臂連接已斷開")
            return True
            
        except Exception as e:
            self.emit_log(f"斷開連接錯誤: {str(e)}")
            return False
    
    # ==================== 機械臂控制 ====================
    
    def toggle_enable(self):
        """切換使能狀態"""
        if not self.global_state['connect']:
            self.emit_log("機械臂未連接")
            return False
            
        try:
            if self.global_state['enable']:
                self.client_dash.DisableRobot()
                self.global_state['enable'] = False
                self.emit_log("機械臂已下使能")
            else:
                self.client_dash.EnableRobot()
                self.global_state['enable'] = True
                self.emit_log("機械臂已使能")
                
            self.enable_changed.emit(self.global_state['enable'])
            return True
            
        except Exception as e:
            self.emit_log(f"使能切換失敗: {str(e)}")
            return False
    
    def emergency_stop(self):
        """緊急停止功能"""
        try:
            if self.global_state['connect'] and self.client_dash:
                # 發送緊急停止指令到機械臂
                result = self.client_dash.EmergencyStop()
                self.emit_log(f"緊急停止指令已發送: {result}")
                
                # 停止所有點動操作
                if self.client_move:
                    self.client_move.MoveJog("")
                
                # 發送夾爪緊急停止指令
                if self.modbus_client and self.modbus_client.connected:
                    try:
                        command_id = int(time.time() * 1000) % 65535
                        self.modbus_client.write_register(address=520, value=2, slave=1)
                        self.modbus_client.write_register(address=523, value=command_id, slave=1)
                        self.emit_log("PGC夾爪緊急停止指令已發送")
                    except Exception as e:
                        self.emit_log(f"PGC夾爪緊急停止失敗: {str(e)}")
                
                # 自動下使能機械臂
                if self.global_state['enable']:
                    self.global_state['enable'] = False
                    self.enable_changed.emit(False)
                
                return True
            else:
                self.emit_log("緊急停止按鈕被按下 (機械臂未連接)")
                return False
                
        except Exception as e:
            self.emit_log(f"緊急停止執行失敗: {str(e)}")
            return False
    
    def reset_robot(self):
        """重置機械臂"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_dash.ResetRobot()
            self.emit_log("機械臂重置")
            return True
        except Exception as e:
            self.emit_log(f"機械臂重置失敗: {str(e)}")
            return False
    
    def clear_error(self):
        """清除錯誤"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_dash.ClearError()
            self.emit_log("錯誤已清除")
            return True
        except Exception as e:
            self.emit_log(f"清除錯誤失敗: {str(e)}")
            return False
    
    def set_speed_factor(self, speed):
        """設定全局速度比例"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_dash.SpeedFactor(speed)
            self.emit_log(f"全局速度比例設定為 {speed}%")
            return True
        except Exception as e:
            self.emit_log(f"全局速度設定失敗: {str(e)}")
            return False
    
    def set_speed_j(self, speed):
        """設定關節運動速度比例"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_dash.SpeedJ(speed)
            self.emit_log(f"關節運動速度比例設定為 {speed}%")
            return True
        except Exception as e:
            self.emit_log(f"關節運動速度設定失敗: {str(e)}")
            return False
    
    def set_speed_l(self, speed):
        """設定直線運動速度比例"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_dash.SpeedL(speed)
            self.emit_log(f"直線運動速度比例設定為 {speed}%")
            return True
        except Exception as e:
            self.emit_log(f"直線運動速度設定失敗: {str(e)}")
            return False
    
    def set_acc_j(self, speed):
        """設定關節運動加速度比例"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_dash.AccJ(speed)
            self.emit_log(f"關節運動加速度比例設定為 {speed}%")
            return True
        except Exception as e:
            self.emit_log(f"關節運動加速度設定失敗: {str(e)}")
            return False
    
    def set_acc_l(self, speed):
        """設定直線運動加速度比例"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_dash.AccL(speed)
            self.emit_log(f"直線運動加速度比例設定為 {speed}%")
            return True
        except Exception as e:
            self.emit_log(f"直線運動加速度設定失敗: {str(e)}")
            return False
    
    def set_do(self, index, status):
        """設定數位輸出 - 隊列指令"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_dash.DO(index, status)
            status_text = '高電平' if status else '低電平'
            self.emit_log(f"DO{index} 設定為 {status_text}")
            return True
        except Exception as e:
            self.emit_log(f"DO設定失敗: {str(e)}")
            return False
    
    def set_do_execute(self, index, status):
        """設定數位輸出 - 立即執行"""
        if not self.global_state['connect']:
            return False
        try:
            # 確保你的dobot_api.py有DOExecute方法
            result = self.client_dash.DOExecute(index, status)
            status_text = '高電平' if status else '低電平'
            self.emit_log(f"DO{index} 立即設定為 {status_text} - 回應: {result}")
            return True
        except AttributeError:
            # 如果沒有DOExecute方法，使用一般DO方法
            self.emit_log(f"警告：DOExecute方法不存在，使用一般DO指令")
            return self.set_do(index, status)
        except Exception as e:
            self.emit_log(f"DO立即執行失敗: {str(e)}")
            return False
    
    # ==================== 運動控制 ====================
    
    def movj(self, x, y, z, r):
        """關節運動 - 增強診斷版本"""
        if not self.global_state['connect'] or not self.global_state['enable']:
            self.emit_log("機械臂未連接或未使能")
            return False
        try:
            result = self.client_move.MovJ(x, y, z, r)
            self.emit_log(f"MovJ指令回應: {result}")
            
            # 檢查回應是否包含錯誤
            if "0," in str(result):  # 0表示成功
                self.emit_log(f"MovJ to ({x}, {y}, {z}, {r}) - 指令接受成功")
                return True
            else:
                self.emit_log(f"MovJ指令可能失敗，回應: {result}")
                return False
        except Exception as e:
            self.emit_log(f"MovJ發送失敗: {str(e)}")
            return False
    
    def movl(self, x, y, z, r, speed):
        """直線運動 - 增強診斷版本，增加CP參數避免手勢切換"""
        if not self.global_state['connect'] or not self.global_state['enable']:
            self.emit_log("機械臂未連接或未使能")
            return False
        try:
            # 添加CP參數幫助避免手勢切換問題
            speed_param = f"SpeedL={speed}"
            cp_param = "CP=50"  # 連續路徑參數，幫助平滑過渡
            
            result = self.client_move.MovL(x, y, z, r, speed_param, cp_param)
            self.emit_log(f"MovL指令回應: {result}")
            
            # 檢查回應是否包含錯誤
            if "0," in str(result):  # 0表示成功
                self.emit_log(f"MovL to ({x}, {y}, {z}, {r}) at {speed}% speed - 指令接受成功")
                return True
            else:
                self.emit_log(f"MovL指令可能失敗，回應: {result}")
                # 如果MovL失敗，建議使用MovJ
                self.emit_log("建議：如果是手勢切換問題，請使用MovJ或JointMovJ")
                return False
        except Exception as e:
            self.emit_log(f"MovL發送失敗: {str(e)}")
            return False
    
    def joint_movj(self, j1, j2, j3, j4):
        """關節座標運動 - 增強診斷版本"""
        if not self.global_state['connect'] or not self.global_state['enable']:
            self.emit_log("機械臂未連接或未使能")
            return False
        try:
            result = self.client_move.JointMovJ(j1, j2, j3, j4)
            self.emit_log(f"JointMovJ指令回應: {result}")
            
            # 檢查回應是否包含錯誤
            if "0," in str(result):  # 0表示成功
                self.emit_log(f"JointMovJ to ({j1}, {j2}, {j3}, {j4}) - 指令接受成功")
                return True
            else:
                self.emit_log(f"JointMovJ指令可能失敗，回應: {result}")
                return False
        except Exception as e:
            self.emit_log(f"JointMovJ發送失敗: {str(e)}")
            return False
    
    def start_jog(self, command):
        """開始點動"""
        if not self.global_state['connect'] or not self.global_state['enable']:
            return False
        try:
            self.client_move.MoveJog(command)
            return True
        except Exception as e:
            self.emit_log(f"點動啟動失敗: {str(e)}")
            return False
    
    def stop_jog(self):
        """停止點動"""
        if not self.global_state['connect']:
            return False
        try:
            self.client_move.MoveJog("")
            return True
        except Exception as e:
            self.emit_log(f"點動停止失敗: {str(e)}")
            return False
    
    # ==================== 夾爪控制 ====================
    
    def send_gripper_command(self, cmd, param1=0, param2=0):
        """發送PGC夾爪指令"""
        if not self.modbus_client or not self.modbus_client.connected:
            self.emit_log("Modbus連接未建立")
            return False
            
        try:
            command_id = int(time.time() * 1000) % 65535
            
            result = self.modbus_client.write_register(address=520, value=cmd, slave=1)
            if result.isError():
                self.emit_log(f"寫入指令代碼失敗: {result}")
                return False
                
            result = self.modbus_client.write_register(address=521, value=param1, slave=1)
            if result.isError():
                self.emit_log(f"寫入參數1失敗: {result}")
                return False
                
            result = self.modbus_client.write_register(address=522, value=param2, slave=1)
            if result.isError():
                self.emit_log(f"寫入參數2失敗: {result}")
                return False
                
            result = self.modbus_client.write_register(address=523, value=command_id, slave=1)
            if result.isError():
                self.emit_log(f"寫入指令ID失敗: {result}")
                return False
            
            self.emit_log(f"PGC夾爪指令已發送: cmd={cmd}, param1={param1}, ID={command_id}")
            return True
            
        except Exception as e:
            self.emit_log(f"PGC夾爪指令發送失敗: {str(e)}")
            return False
    
    def get_gripper_status(self):
        """獲取PGC夾爪狀態"""
        if not self.modbus_client or not self.modbus_client.connected:
            return None
            
        try:
            result = self.modbus_client.read_holding_registers(address=500, count=20, slave=1)
            if result.isError():
                return None
                
            registers = result.registers
            
            return {
                'module_status': registers[0],
                'connection_status': registers[1],
                'hold_status': registers[4],
                'current_pos': registers[5]
            }
            
        except Exception as e:
            return None
    
    # ==================== 點位管理 ====================
    
    def save_current_point(self, name):
        """保存當前點位"""
        if not self.global_state['connect']:
            self.emit_log("機械臂未連接")
            return False
            
        # 檢查是否有有效的位置數據
        if (self.current_position['cartesian']['x'] == 0 and 
            self.current_position['cartesian']['y'] == 0 and 
            self.current_position['cartesian']['z'] == 0):
            self.emit_log("尚未獲取到機械臂位置反饋")
            return False
            
        # 使用實際機械臂反饋的位置數據
        cartesian = self.current_position['cartesian'].copy()
        joint = self.current_position['joint'].copy()
        
        point = {
            'id': len(self.saved_points),
            'name': name.strip(),
            'cartesian': cartesian,
            'joint': joint,
            'created_time': datetime.now().isoformat(),
            'modified_time': datetime.now().isoformat()
        }
        
        self.saved_points.append(point)
        self.save_points()
        
        # 詳細日誌信息
        self.emit_log(f"點位 '{name}' 已保存 - 位置: X:{cartesian['x']:.2f}, Y:{cartesian['y']:.2f}, Z:{cartesian['z']:.2f}, R:{cartesian['r']:.2f}")
        self.emit_log(f"關節角度: J1:{joint['j1']:.2f}, J2:{joint['j2']:.2f}, J3:{joint['j3']:.2f}, J4:{joint['j4']:.2f}")
        return True
    
    def update_point(self, index, point_data):
        """更新點位數據"""
        if 0 <= index < len(self.saved_points):
            point_data['modified_time'] = datetime.now().isoformat()
            self.saved_points[index] = point_data
            self.save_points()
            self.emit_log(f"點位 '{point_data['name']}' 已更新")
            return True
        return False
    
    def delete_point(self, index):
        """刪除點位"""
        if 0 <= index < len(self.saved_points):
            point = self.saved_points[index]
            del self.saved_points[index]
            # 重新分配ID
            for i, p in enumerate(self.saved_points):
                p['id'] = i
            self.save_points()
            self.emit_log(f"點位 '{point['name']}' 已刪除")
            return True
        return False
    
    def execute_point_movement(self, point_index, motion_type, speed=30):
        """執行點位移動 - 增強診斷版本"""
        if not (0 <= point_index < len(self.saved_points)):
            self.emit_log("無效的點位索引")
            return False
            
        if not self.global_state['connect']:
            self.emit_log("機械臂未連接")
            return False
            
        if not self.global_state['enable']:
            self.emit_log("機械臂未使能")
            return False
        
        point = self.saved_points[point_index]
        cartesian = point['cartesian']
        joint = point['joint']
        
        # 檢查點位數據完整性
        required_keys = ['x', 'y', 'z', 'r']
        for key in required_keys:
            if key not in cartesian:
                self.emit_log(f"點位數據不完整，缺少{key}座標")
                return False
        
        # 安全範圍檢查
        x, y, z, r = cartesian['x'], cartesian['y'], cartesian['z'], cartesian['r']
        
        if not (-800 <= x <= 800) or not (-800 <= y <= 800):
            self.emit_log(f"座標超出安全範圍: X={x}, Y={y}")
            
        if not (50 <= z <= 600):
            self.emit_log(f"Z座標可能不安全: {z}")
        
        try:
            self.emit_log(f"準備移動到點位 '{point['name']}'")
            
            # 檢查機械臂狀態
            robot_mode_result = self.client_dash.RobotMode()
            self.emit_log(f"當前機械臂狀態: {robot_mode_result}")
            
            # 確保運動隊列沒有暫停
            try:
                continue_result = self.client_dash.Continue()
                self.emit_log(f"運動隊列繼續指令: {continue_result}")
            except Exception as e:
                self.emit_log(f"運動隊列繼續指令失敗: {str(e)}")
            
            if "MovL" in motion_type:
                self.emit_log(f"使用MovL - 目標座標: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, R={r:.2f}")
                self.emit_log(f"移動速度: {speed}%")
                result = self.movl(x, y, z, r, speed)
                
            elif "MovJ" in motion_type and "Joint" not in motion_type:
                self.emit_log(f"使用MovJ - 目標座標: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, R={r:.2f}")
                result = self.movj(x, y, z, r)
                
            elif "JointMovJ" in motion_type:
                # 檢查關節數據完整性
                required_joint_keys = ['j1', 'j2', 'j3', 'j4']
                for key in required_joint_keys:
                    if key not in joint:
                        self.emit_log(f"關節數據不完整，缺少{key}角度")
                        return False
                
                j1, j2, j3, j4 = joint['j1'], joint['j2'], joint['j3'], joint['j4']
                self.emit_log(f"使用JointMovJ - 關節角度: J1={j1:.2f}, J2={j2:.2f}, J3={j3:.2f}, J4={j4:.2f}")
                result = self.joint_movj(j1, j2, j3, j4)
            else:
                self.emit_log(f"未知的運動類型: {motion_type}")
                return False
            
            # 檢查指令回應
            if result:
                self.emit_log(f"運動指令發送成功，結果: {result}")
                
                # 發送同步指令等待完成
                try:
                    sync_result = self.client_move.Sync()
                    self.emit_log(f"同步等待指令: {sync_result}")
                except Exception as e:
                    self.emit_log(f"同步等待失敗: {str(e)}")
                
                return True
            else:
                self.emit_log("運動指令發送失敗")
                return False
            
        except Exception as e:
            self.emit_log(f"移動失敗: {str(e)}")
            return False
    
    def load_points(self):
        """載入點位數據 - 增強版本"""
        try:
            if os.path.exists(self.points_file):
                with open(self.points_file, 'r', encoding='utf-8') as f:
                    loaded_points = json.load(f)
                
                # 驗證和修正點位數據
                self.saved_points = []
                for i, point in enumerate(loaded_points):
                    # 確保基本結構完整
                    if not isinstance(point, dict):
                        self.emit_log(f"跳過無效點位數據: {point}")
                        continue
                    
                    # 修正基本欄位
                    if 'id' not in point:
                        point['id'] = i
                    if 'name' not in point:
                        point['name'] = f"Point_{i}"
                    if 'created_time' not in point:
                        point['created_time'] = datetime.now().isoformat()
                    if 'modified_time' not in point:
                        point['modified_time'] = datetime.now().isoformat()
                    
                    # 修正cartesian數據
                    if 'cartesian' not in point:
                        point['cartesian'] = {'x': 0, 'y': 0, 'z': 0, 'r': 0}
                    else:
                        cartesian = point['cartesian']
                        required_keys = ['x', 'y', 'z', 'r']
                        for key in required_keys:
                            if key not in cartesian:
                                cartesian[key] = 0.0
                            # 確保數值類型正確
                            try:
                                cartesian[key] = float(cartesian[key])
                            except (ValueError, TypeError):
                                cartesian[key] = 0.0
                    
                    # 修正joint數據
                    if 'joint' not in point:
                        point['joint'] = {'j1': 0, 'j2': 0, 'j3': 0, 'j4': 0}
                    else:
                        joint = point['joint']
                        required_joint_keys = ['j1', 'j2', 'j3', 'j4']
                        for key in required_joint_keys:
                            if key not in joint:
                                joint[key] = 0.0
                            # 確保數值類型正確
                            try:
                                joint[key] = float(joint[key])
                            except (ValueError, TypeError):
                                joint[key] = 0.0
                    
                    self.saved_points.append(point)
                
                # 重新分配ID確保連續性
                for i, point in enumerate(self.saved_points):
                    point['id'] = i
                
                # 保存修正後的數據
                self.save_points()
                
                self.emit_log(f"載入並修正 {len(self.saved_points)} 個點位")
            else:
                self.saved_points = []
                self.emit_log("點位檔案不存在，建立新的點位列表")
                
        except Exception as e:
            self.emit_log(f"載入點位失敗: {str(e)}")
            self.saved_points = []
    
    def save_points(self):
        """保存點位數據"""
        try:
            with open(self.points_file, 'w', encoding='utf-8') as f:
                json.dump(self.saved_points, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.emit_log(f"保存點位失敗: {str(e)}")
    
    # ==================== 狀態反饋 ====================
    
    def start_feedback_thread(self):
        """啟動反饋線程 - 高頻率版本"""
        if not self.global_state['connect']:
            self.emit_log("錯誤：機械臂未連接，無法啟動反饋線程")
            return
        
        # 初始化反饋計數器
        self.feedback_count = 0
        self.feedback_active = True
        
        # 設置線程為高優先級
        self.feedback_thread = Thread(target=self.feedback_loop, daemon=True)
        self.feedback_thread.start()
        self.emit_log("高頻率狀態反饋線程已啟動 (200Hz目標頻率)")
        
        # 啟動性能監控定時器
        self.start_performance_monitor()
    
    def feedback_loop(self):
        """反饋循環 - 高頻率版本"""
        hasRead = 0
        error_count = 0
        max_errors = 10
        log_interval = 1000  # 每1000次循環才輸出一次日誌
        
        # 只在啟動時輸出一次日誌
        self.emit_log("高頻率狀態反饋循環已啟動 (8ms週期)")
        
        while self.global_state['connect'] and self.feedback_active:
            try:
                data = bytes()
                hasRead = 0
                
                # 檢查客戶端是否有效 - 減少日誌輸出
                if not self.client_feed or not hasattr(self.client_feed, 'socket_dobot'):
                    if self.feedback_count % 100 == 0:  # 只每100次輸出一次警告
                        self.emit_log("警告：反饋客戶端無效")
                    time.sleep(0.1)
                    continue
                
                # 設置socket超時 - 移除日誌輸出
                try:
                    self.client_feed.socket_dobot.settimeout(0.5)  # 減少超時時間
                except:
                    if error_count == 0:  # 只在第一次錯誤時輸出
                        self.emit_log("Socket超時設置失敗")
                    break
                
                # 讀取1440字節的反饋數據
                while hasRead < 1440:
                    try:
                        remaining = 1440 - hasRead
                        temp = self.client_feed.socket_dobot.recv(remaining)
                        
                        if len(temp) == 0:
                            raise ConnectionError("接收到空數據")
                            
                        hasRead += len(temp)
                        data += temp
                        
                    except socket.timeout:
                        raise
                    except Exception as e:
                        raise
                
                # 驗證數據完整性 - 移除日誌輸出
                if len(data) != 1440:
                    continue
                
                # 解析反饋數據
                try:
                    a = np.frombuffer(data, dtype=MyType)
                    
                    # 驗證測試值 - 移除日誌輸出
                    test_value = a['test_value'][0]
                    expected_test = 0x123456789abcdef
                    
                    if test_value != expected_test:
                        continue
                    
                    self.feedback_count += 1
                    self.last_feedback_time = time.time()
                    error_count = 0
                    
                    # 準備反饋數據
                    feedback_data = {
                        'speed_scaling': float(a["speed_scaling"][0]),
                        'robot_mode': int(a["robot_mode"][0]),
                        'digital_input_bits': int(a["digital_input_bits"][0]),
                        'digital_outputs': int(a["digital_outputs"][0]),
                        'q_actual': [float(x) for x in a["q_actual"][0][:4]],
                        'tool_vector_actual': [float(x) for x in a["tool_vector_actual"][0][:4]]
                    }
                    
                    # 更新當前位置數據
                    self.update_current_position(feedback_data)
                    
                    # 發送信號更新UI
                    self.feedback_update.emit(feedback_data)
                    
                    # 檢查錯誤狀態
                    if a["robot_mode"][0] == 9:
                        self.handle_robot_error()
                    
                    # 大幅減少日誌輸出頻率
                    if self.feedback_count % log_interval == 0:
                        self.emit_log(f"反饋循環正常 - 計數: {self.feedback_count}, 頻率: {1000/8:.1f}Hz")
                    
                except Exception as e:
                    # 減少錯誤日誌輸出
                    if error_count < 3:  # 只輸出前3次錯誤
                        self.emit_log(f"數據解析錯誤: {str(e)}")
                    continue
                    
                # 高頻率循環，最小延遲
                time.sleep(0.005)  # 5ms週期，提高到200Hz
                
            except Exception as e:
                error_count += 1
                
                # 減少錯誤日誌頻率
                if error_count <= 3 and self.global_state['connect']:
                    self.emit_log(f"反饋線程錯誤 #{error_count}: {str(e)}")
                    
                if error_count >= max_errors:
                    self.emit_log(f"反饋線程錯誤次數超過 {max_errors} 次，停止線程")
                    break
                    
                time.sleep(0.1)  # 錯誤時短暫延遲
                
        self.emit_log("狀態反饋線程已停止")
        self.feedback_active = False
    
    def update_current_position(self, feedback_data):
        """更新當前位置數據"""
        pos = feedback_data.get('tool_vector_actual', [0, 0, 0, 0])
        joints = feedback_data.get('q_actual', [0, 0, 0, 0])
        
        if len(pos) >= 4:
            self.current_position['cartesian'] = {
                'x': float(pos[0]),
                'y': float(pos[1]),
                'z': float(pos[2]),
                'r': float(pos[3])
            }
        
        if len(joints) >= 4:
            self.current_position['joint'] = {
                'j1': float(joints[0]),
                'j2': float(joints[1]),
                'j3': float(joints[2]),
                'j4': float(joints[3])
            }
    
    def handle_robot_error(self):
        """處理機械臂錯誤"""
        try:
            error_info = self.client_dash.GetErrorID()
            self.parse_and_display_error(error_info)
        except Exception as e:
            self.emit_error(f"獲取錯誤信息失敗: {str(e)}")
    
    def parse_and_display_error(self, error_response):
        """解析並顯示錯誤信息"""
        try:
            if "{" in error_response:
                error_data = error_response.split("{")[1].split("}")[0]
                error_list = json.loads("{" + error_data + "}")
                
                error_messages = {
                    22: "手勢切換錯誤 - 請嘗試使用關節運動(JointMovJ)或調整目標點位",
                    23: "直線運動過程中規劃點超出工作空間 - 重新選取運動點位",
                    24: "圓弧運動過程中規劃點超出工作空間 - 重新選取運動點位",
                    32: "運動過程逆解算奇異 - 重新選取運動點位",
                    33: "運動過程逆解算無解 - 重新選取運動點位",
                    34: "運動過程逆解算限位 - 重新選取運動點位"
                }
                
                if len(error_list) > 0 and error_list[0]:
                    for error_id in error_list[0]:
                        if error_id in error_messages:
                            self.emit_error(f"錯誤 {error_id}: {error_messages[error_id]}")
                        else:
                            self.emit_error(f"未知錯誤 {error_id}")
            else:
                self.emit_error(f"機械臂錯誤: {error_response}")
                
        except Exception as e:
            self.emit_error(f"錯誤解析失敗: {str(e)}")
            self.emit_error(f"原始錯誤: {error_response}")

    def diagnose_feedback_issue(self):
        """診斷反饋問題"""
        self.emit_log("開始診斷反饋連接問題...")
        
        # 檢查連接狀態
        if not self.global_state['connect']:
            self.emit_log("診斷結果：機械臂未連接")
            return
            
        # 檢查socket狀態
        if not self.client_feed or not hasattr(self.client_feed, 'socket_dobot'):
            self.emit_log("診斷結果：反饋客戶端未建立")
            return
            
        # 檢查端口狀態
        try:
            sock = self.client_feed.socket_dobot
            self.emit_log(f"Socket狀態: {sock.getsockname()} -> {sock.getpeername()}")
        except Exception as e:
            self.emit_log(f"Socket狀態異常: {str(e)}")
            
        # 檢查反饋計數
        self.emit_log(f"反饋數據計數: {self.feedback_count}")
        
        if self.feedback_count == 0:
            self.emit_log("診斷結果：未接收到反饋數據，可能是網路或端口問題")
        else:
            self.emit_log("診斷結果：反饋接收正常")
    
    def diagnose_movement_issue(self):
        """診斷運動問題"""
        self.emit_log("開始診斷運動控制問題...")
        
        if not self.global_state['connect']:
            self.emit_log("診斷結果：機械臂未連接")
            return
            
        if not self.global_state['enable']:
            self.emit_log("診斷結果：機械臂未使能")
            return
            
        try:
            # 檢查機械臂狀態
            robot_mode = self.client_dash.RobotMode()
            self.emit_log(f"機械臂狀態檢查: {robot_mode}")
            
            # 檢查錯誤狀態
            error_id = self.client_dash.GetErrorID()
            self.emit_log(f"錯誤狀態檢查: {error_id}")
            
            # 檢查運動隊列狀態
            try:
                continue_result = self.client_dash.Continue()
                self.emit_log(f"運動隊列狀態: {continue_result}")
            except Exception as e:
                self.emit_log(f"運動隊列檢查失敗: {str(e)}")
            
            # 測試簡單運動指令
            try:
                # 獲取當前位置
                current_pos = self.client_dash.GetPose()
                self.emit_log(f"當前位置: {current_pos}")
                
                # 嘗試小幅移動測試
                test_result = self.client_move.MovJ(200, 200, 200, 0)
                self.emit_log(f"測試運動指令回應: {test_result}")
                
            except Exception as e:
                self.emit_log(f"測試運動指令失敗: {str(e)}")
                
        except Exception as e:
            self.emit_log(f"診斷過程發生錯誤: {str(e)}")
    
    def check_robot_ready_for_movement(self):
        """檢查機械臂是否準備好運動"""
        try:
            # 檢查基本狀態
            if not self.global_state['connect']:
                self.emit_log("機械臂未連接")
                return False
                
            if not self.global_state['enable']:
                self.emit_log("機械臂未使能")
                return False
            
            # 檢查機械臂模式
            robot_mode_result = self.client_dash.RobotMode()
            self.emit_log(f"機械臂模式檢查: {robot_mode_result}")
            
            # 從回應中提取模式值
            try:
                if "{" in robot_mode_result and "}" in robot_mode_result:
                    mode_str = robot_mode_result.split("{")[1].split("}")[0]
                    mode = int(mode_str)
                    
                    mode_descriptions = {
                        1: "初始化中",
                        2: "抱閘已松開", 
                        4: "未使能",
                        5: "使能且空閒 - 準備運動",
                        6: "拖拽模式",
                        7: "運行中",
                        8: "錄製模式",
                        9: "錯誤狀態",
                        10: "暫停狀態",
                        11: "點動中"
                    }
                    
                    description = mode_descriptions.get(mode, f"未知模式({mode})")
                    self.emit_log(f"機械臂當前模式: {description}")
                    
                    if mode == 5:  # 使能且空閒
                        self.emit_log("機械臂準備就緒，可以運動")
                        return True
                    elif mode == 9:  # 錯誤狀態
                        self.emit_log("機械臂處於錯誤狀態，需要清除錯誤")
                        return False
                    elif mode == 10:  # 暫停狀態
                        self.emit_log("機械臂處於暫停狀態，嘗試繼續...")
                        continue_result = self.client_dash.Continue()
                        self.emit_log(f"繼續指令結果: {continue_result}")
                        return True
                    else:
                        self.emit_log(f"機械臂模式不適合運動: {description}")
                        return False
                        
            except Exception as e:
                self.emit_log(f"解析機械臂模式失敗: {str(e)}")
                return False
                
        except Exception as e:
            self.emit_log(f"檢查機械臂準備狀態失敗: {str(e)}")
            return False
    
    def start_performance_monitor(self):
        """啟動性能監控定時器"""
        from threading import Timer
        
        def monitor_performance():
            if self.feedback_active:
                current_count = self.feedback_count
                count_diff = current_count - self.last_feedback_count
                actual_freq = count_diff / 5.0  # 每5秒監控一次
                
                # 只在頻率異常時輸出日誌
                if actual_freq < 50:  # 低於50Hz時警告
                    self.emit_log(f"反饋頻率較低: {actual_freq:.1f}Hz (目標200Hz)")
                elif self.feedback_count % 5000 == 0:  # 每5000次輸出一次正常狀態
                    self.emit_log(f"反饋頻率: {actual_freq:.1f}Hz, 總計: {current_count}")
                
                self.last_feedback_count = current_count
                
                # 繼續監控
                self.performance_timer = Timer(5.0, monitor_performance)
                self.performance_timer.daemon = True
                self.performance_timer.start()
        
        # 5秒後開始第一次監控
        self.performance_timer = Timer(5.0, monitor_performance)
        self.performance_timer.daemon = True
        self.performance_timer.start()
    
    def stop_performance_monitor(self):
        """停止性能監控"""
        if self.performance_timer:
            self.performance_timer.cancel()
            self.performance_timer = None

class PointDataValidator:
    """點位數據驗證器"""
    
    @staticmethod
    def validate_position_change(original, new, max_change=100.0):
        """驗證位置變化是否安全"""
        if not original:
            return True
            
        orig_cart = original.get('cartesian', {})
        new_cart = new.get('cartesian', {})
        
        # 檢查變化幅度
        for key in ['x', 'y', 'z']:
            old_val = orig_cart.get(key, 0)
            new_val = new_cart.get(key, 0)
            if abs(new_val - old_val) > max_change:
                return False
                
        # 檢查是否正負相反
        for key in ['x', 'y', 'z']:
            old_val = orig_cart.get(key, 0)
            new_val = new_cart.get(key, 0)
            if old_val != 0 and new_val != 0:
                if (old_val > 0) != (new_val > 0):
                    return False
        return True
    
    @staticmethod
    def validate_cartesian_range(x, y, z, r):
        """驗證笛卡爾座標範圍"""
        warnings = []
        
        if not (-800 <= x <= 800):
            warnings.append(f"X座標 {x} 可能超出安全範圍(-800~800)")
        if not (-800 <= y <= 800):
            warnings.append(f"Y座標 {y} 可能超出安全範圍(-800~800)")
        if not (50 <= z <= 600):
            warnings.append(f"Z座標 {z} 可能不安全(建議50~600)")
        if not (-180 <= r <= 180):
            warnings.append(f"R角度 {r} 超出範圍(-180~180)")
            
        return warnings
    
    @staticmethod
    def validate_joint_range(j1, j2, j3, j4):
        """驗證關節角度範圍"""
        warnings = []
        
        if not (-180 <= j1 <= 180):
            warnings.append(f"J1角度 {j1} 超出範圍(-180~180)")
        if not (-135 <= j2 <= 135):
            warnings.append(f"J2角度 {j2} 超出範圍(-135~135)")
        if not (-135 <= j3 <= 135):
            warnings.append(f"J3角度 {j3} 超出範圍(-135~135)")
        if not (-180 <= j4 <= 180):
            warnings.append(f"J4角度 {j4} 超出範圍(-180~180)")
            
        return warnings