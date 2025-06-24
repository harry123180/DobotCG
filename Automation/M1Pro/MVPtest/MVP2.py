#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MVP2.0 - 機械臂+CCD1視覺整合系統
基地址400規劃，整合PGC夾爪控制、CCD1視覺檢測
版本：優化夾取流程，20%速度執行，耗時統計
"""

import json
import time
import threading
from pymodbus.client.tcp import ModbusTcpClient
from dobot_api import DobotApiDashboard, DobotApiMove

# Modbus地址配置
MODBUS_BASE_ADDR = 400
CONTROL_ADDR = MODBUS_BASE_ADDR + 0      # 400: 控制指令地址
RUNNING_ADDR = MODBUS_BASE_ADDR + 1      # 401: 運行狀態地址  
STANDBY_ADDR = MODBUS_BASE_ADDR + 2      # 402: 準備狀態地址

# CCD1視覺模組地址 (基地址200)
CCD1_CONTROL_ADDR = 200                  # CCD1控制指令
CCD1_STATUS_ADDR = 201                   # CCD1狀態寄存器
CCD1_CIRCLE_COUNT_ADDR = 240             # 檢測到的圓形數量
CCD1_WORLD_COORD_VALID_ADDR = 256        # 世界座標有效標誌

# PGC夾爪寄存器地址 (基地址500)
PGC_STATUS_BASE = 500    # PGC狀態寄存器起始
PGC_COMMAND_BASE = 520   # PGC指令寄存器起始
PGC_COMMAND_ADDR = 520   # 指令代碼
PGC_PARAM1_ADDR = 521    # 參數1 (位置值)
PGC_PARAM2_ADDR = 522    # 參數2 (保留)
PGC_CMD_ID_ADDR = 523    # 指令ID

PGC_MODULE_STATUS_ADDR = 500  # 模組狀態
PGC_CONNECT_STATUS_ADDR = 501 # 連接狀態
PGC_DEVICE_STATUS_ADDR = 502  # 設備狀態
PGC_GRIP_STATUS_ADDR = 504    # 夾持狀態
PGC_POSITION_ADDR = 505       # 當前位置

# 機械臂連接配置
ROBOT_IP = "192.168.1.6"
DASHBOARD_PORT = 29999
MOVE_PORT = 30003

# Modbus連接配置
MODBUS_IP = "127.0.0.1"
MODBUS_PORT = 502
SLAVE_ID = 1

# 系統參數
SPEED_RATIO = 50  # 全局速度50%
POINT_DELAY = 0.5  # 兩點間等待0.5秒
PICKUP_HEIGHT = 137.52  # 夾取高度

# PGC夾爪指令映射
PGC_CMD_INIT = 1        # 初始化
PGC_CMD_STOP = 2        # 停止
PGC_CMD_ABS_POS = 3     # 絕對位置
PGC_CMD_QUICK_OPEN = 7  # 快速開啟
PGC_CMD_QUICK_CLOSE = 8 # 快速關閉

# PGC夾持狀態映射
PGC_STATUS_MOVING = 0   # 運動中
PGC_STATUS_REACHED = 1  # 到達
PGC_STATUS_GRIPPED = 2  # 夾住
PGC_STATUS_DROPPED = 3  # 掉落

# CCD1控制指令
CCD1_CMD_CLEAR = 0      # 清空控制
CCD1_CMD_CAPTURE = 8    # 拍照
CCD1_CMD_DETECT = 16    # 拍照+檢測
CCD1_CMD_INIT = 32      # 重新初始化

class IntegratedRobotSystem:
    def __init__(self):
        self.modbus_client = None
        self.dashboard_api = None
        self.move_api = None
        self.robot_points = {}
        self.running = False
        self.command_id_counter = 1
        
    def load_robot_points(self):
        """載入機械臂點位數據"""
        try:
            with open("Automation/M1Pro/saved_points/robot_points.json", "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            for point in points_list:
                self.robot_points[point["name"]] = point
                
            print(f"載入點位數據成功，共{len(self.robot_points)}個點位")
            return True
            
        except Exception as e:
            print(f"載入點位數據失敗: {e}")
            return False
    
    def connect_modbus(self):
        """連接Modbus TCP Server"""
        try:
            self.modbus_client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
            if self.modbus_client.connect():
                print(f"Modbus連接成功: {MODBUS_IP}:{MODBUS_PORT}")
                self.write_modbus(RUNNING_ADDR, 0)
                self.write_modbus(STANDBY_ADDR, 1)
                return True
            else:
                print("Modbus連接失敗")
                return False
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            return False
    
    def connect_robot(self):
        """連接機械臂"""
        try:
            self.dashboard_api = DobotApiDashboard(ROBOT_IP, DASHBOARD_PORT)
            print(f"Dashboard連接成功: {ROBOT_IP}:{DASHBOARD_PORT}")
            
            self.move_api = DobotApiMove(ROBOT_IP, MOVE_PORT)
            print(f"Move API連接成功: {ROBOT_IP}:{MOVE_PORT}")
            
            # 機械臂初始化
            self.dashboard_api.ClearError()
            self.dashboard_api.EnableRobot()
            self.dashboard_api.SpeedFactor(SPEED_RATIO)
            self.dashboard_api.SpeedJ(SPEED_RATIO)
            self.dashboard_api.AccJ(SPEED_RATIO)
            
            print(f"機械臂初始化完成，全局速度設定: {SPEED_RATIO}%")
            return True
            
        except Exception as e:
            print(f"機械臂連接失敗: {e}")
            return False
    
    def read_modbus(self, address):
        """讀取Modbus寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(address, count=1, slave=SLAVE_ID)
            
            if hasattr(result, 'isError') and result.isError():
                return None
            elif hasattr(result, 'registers') and len(result.registers) > 0:
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            print(f"讀取寄存器{address}異常: {e}")
            return None
    
    def write_modbus(self, address, value):
        """寫入Modbus寄存器"""
        try:
            result = self.modbus_client.write_register(address, value, slave=SLAVE_ID)
            
            if hasattr(result, 'isError') and result.isError():
                return False
            else:
                return True
                
        except Exception as e:
            print(f"寫入寄存器{address}={value}異常: {e}")
            return False
    
    def read_world_coordinate(self, circle_index=1):
        """讀取CCD1第一個圓的世界座標 (mm)"""
        try:
            # 檢查世界座標有效性
            world_valid = self.read_modbus(CCD1_WORLD_COORD_VALID_ADDR)
            if world_valid != 1:
                print("世界座標數據無效")
                return None, None
            
            # 讀取第一個圓的世界座標 (寄存器257-260)
            x_high = self.read_modbus(257)  # 圓形1世界X座標高位
            x_low = self.read_modbus(258)   # 圓形1世界X座標低位
            y_high = self.read_modbus(259)  # 圓形1世界Y座標高位
            y_low = self.read_modbus(260)   # 圓形1世界Y座標低位
            
            if any(val is None for val in [x_high, x_low, y_high, y_low]):
                print("讀取世界座標寄存器失敗")
                return None, None
            
            # 恢復32位世界座標 (×100精度)
            world_x_int = (x_high << 16) | x_low
            world_y_int = (y_high << 16) | y_low
            
            # 處理負數
            if world_x_int >= 2147483648:  # 2^31
                world_x_int -= 4294967296  # 2^32
            if world_y_int >= 2147483648:
                world_y_int -= 4294967296
            
            # 恢復小數精度
            world_x_mm = world_x_int / 100.0
            world_y_mm = world_y_int / 100.0
            
            print(f"讀取世界座標: X={world_x_mm:.2f}mm, Y={world_y_mm:.2f}mm")
            return world_x_mm, world_y_mm
            
        except Exception as e:
            print(f"讀取世界座標失敗: {e}")
            return None, None
    
    def send_ccd1_command(self, command, timeout=15.0):
        """發送CCD1控制指令並等待完成"""
        try:
            start_time = time.time()
            
            # 檢查CCD1是否Ready
            status = self.read_modbus(CCD1_STATUS_ADDR)
            if status is None or (status & 0x01) == 0:  # bit0 = Ready
                print("CCD1系統未Ready")
                return False
            
            print(f"發送CCD1指令: {command}")
            
            # 發送控制指令
            if not self.write_modbus(CCD1_CONTROL_ADDR, command):
                print("CCD1指令發送失敗")
                return False
            
            # 等待Running狀態清除
            while time.time() - start_time < timeout:
                status = self.read_modbus(CCD1_STATUS_ADDR)
                if status is not None:
                    running = (status & 0x02) >> 1  # bit1 = Running
                    if running == 0:
                        # 清空控制指令
                        self.write_modbus(CCD1_CONTROL_ADDR, CCD1_CMD_CLEAR)
                        elapsed_time = time.time() - start_time
                        print(f"CCD1指令執行完成，耗時: {elapsed_time*1000:.1f}ms")
                        return True
                
                time.sleep(0.05)
            
            print(f"CCD1指令執行超時")
            return False
            
        except Exception as e:
            print(f"CCD1指令執行失敗: {e}")
            return False
    
    def get_next_command_id(self):
        """獲取下一個指令ID"""
        self.command_id_counter += 1
        return self.command_id_counter
    
    def send_gripper_command_batch(self, command, param1=0, param2=0, timeout=15.0):
        """使用批量寫入方式發送夾爪指令"""
        try:
            cmd_id = self.get_next_command_id()
            
            # 構建指令數組 (10個寄存器)
            values = [command, param1, param2, cmd_id, 0, 0, 0, 0, 0, 0]
            
            print(f"發送PGC指令: cmd={command}, param1={param1}, id={cmd_id}")
            
            result = self.modbus_client.write_registers(
                address=PGC_COMMAND_BASE,
                values=values,
                slave=SLAVE_ID
            )
            
            if result and not result.isError():
                # 等待指令ID清零確認執行完成
                start_time = time.time()
                while time.time() - start_time < timeout:
                    current_cmd_id = self.read_modbus(PGC_CMD_ID_ADDR)
                    if current_cmd_id == 0:
                        elapsed_time = time.time() - start_time
                        print(f"PGC指令執行完成，耗時: {elapsed_time*1000:.1f}ms")
                        return True
                    time.sleep(0.05)
                
                print(f"PGC指令執行超時")
                return False
            else:
                print(f"PGC指令發送失敗: {result}")
                return False
                
        except Exception as e:
            print(f"發送PGC指令失敗: {e}")
            return False
    
    def initialize_gripper(self):
        """初始化PGC夾爪並關閉"""
        print("=== 初始化PGC夾爪 ===")
        
        # 發送初始化指令
        result = self.send_gripper_command_batch(PGC_CMD_INIT)
        if not result:
            print("夾爪初始化失敗")
            return False
        
        # 等待初始化完成
        print("等待夾爪初始化...")
        init_start_time = time.time()
        init_completed = False
        
        while time.time() - init_start_time < 10.0:
            device_status = self.read_modbus(PGC_DEVICE_STATUS_ADDR)
            if device_status == 1:
                init_time = time.time() - init_start_time
                print(f"夾爪初始化成功，耗時: {init_time:.2f}秒")
                init_completed = True
                break
            time.sleep(0.2)
        
        if not init_completed:
            print("夾爪初始化超時")
            return False
        
        # 設定最大速度和力道
        print("設定夾爪參數...")
        self.send_gripper_command_batch(6, 100)  # 設定速度
        time.sleep(0.5)
        self.send_gripper_command_batch(5, 100)  # 設定力道
        time.sleep(0.5)
        
        # 初始化完畢後才下達關閉指令
        print("初始化完畢，下達夾爪關閉指令...")
        close_result = self.send_gripper_command_batch(PGC_CMD_QUICK_CLOSE)
        if close_result:
            print("夾爪關閉指令發送成功，等待到達位置79...")
        else:
            print("夾爪關閉指令發送失敗")
            return False
        
        # 確實等待夾爪到達位置79 (關閉完成位置)
        close_start_time = time.time()
        position_reached = False
        
        while time.time() - close_start_time < 15.0:  # 最多等待15秒
            current_position = self.read_modbus(PGC_POSITION_ADDR)
            grip_status = self.read_modbus(PGC_GRIP_STATUS_ADDR)
            
            if current_position is not None:
                print(f"當前夾爪位置: {current_position}, 狀態: {grip_status}")
                
                # 檢查是否到達位置79 (允許5個單位的誤差)
                if abs(current_position - 79) <= 5:
                    close_time = time.time() - close_start_time
                    print(f"✓ 夾爪成功關閉到位置79，最終位置: {current_position}，耗時: {close_time:.2f}秒")
                    position_reached = True
                    break
            
            time.sleep(0.2)  # 每0.2秒檢查一次
        
        if not position_reached:
            final_position = self.read_modbus(PGC_POSITION_ADDR)
            print(f"✗ 夾爪關閉超時，最終位置: {final_position}")
            return False
        
        print("PGC夾爪完整初始化和關閉完成")
        return True
    
    def gripper_close(self):
        """夾爪關閉"""
        print("=== 夾爪關閉 ===")
        return self.send_gripper_command_batch(PGC_CMD_QUICK_CLOSE)
    
    def gripper_open_to_position(self, position):
        """夾爪張開到指定位置"""
        print(f"=== 夾爪張開到位置 {position} ===")
        return self.send_gripper_command_batch(PGC_CMD_ABS_POS, position)
    
    def check_gripper_status(self):
        """檢查夾爪狀態"""
        grip_status = self.read_modbus(PGC_GRIP_STATUS_ADDR)
        current_position = self.read_modbus(PGC_POSITION_ADDR)
        
        status_desc = {
            PGC_STATUS_MOVING: "運動中",
            PGC_STATUS_REACHED: "已到達",
            PGC_STATUS_GRIPPED: "夾住物體",
            PGC_STATUS_DROPPED: "物體掉落"
        }.get(grip_status, f"未知狀態({grip_status})")
        
        print(f"夾爪狀態: {status_desc}, 當前位置: {current_position}")
        return grip_status, current_position
    
    def move_to_point(self, point_name):
        """移動到指定點位"""
        if point_name not in self.robot_points:
            print(f"點位{point_name}不存在")
            return False, 0.0
            
        point = self.robot_points[point_name]
        joint = point["joint"]
        
        try:
            move_start = time.time()
            result = self.move_api.JointMovJ(
                joint["j1"], 
                joint["j2"], 
                joint["j3"], 
                joint["j4"]
            )
            
            self.move_api.Sync()
            move_time = time.time() - move_start
            
            print(f"移動到{point_name}: J1={joint['j1']:.2f}, J2={joint['j2']:.2f}, J3={joint['j3']:.2f}, J4={joint['j4']:.2f}, 耗時: {move_time*1000:.1f}ms")
            return True, move_time
            
        except Exception as e:
            print(f"移動到{point_name}失敗: {e}")
            return False, 0.0
    
    def move_to_world_coordinate(self, world_x, world_y, z_height):
        """移動到世界座標位置 (使用JointMovJ避免手勢切換)"""
        try:
            move_start = time.time()
            
            # 使用MovL進行笛卡爾座標移動
            result = self.move_api.MovL(world_x, world_y, z_height, 0)
            self.move_api.Sync()
            
            move_time = time.time() - move_start
            print(f"移動到世界座標: X={world_x:.2f}, Y={world_y:.2f}, Z={z_height:.2f}, 耗時: {move_time*1000:.1f}ms")
            return True, move_time
            
        except Exception as e:
            print(f"移動到世界座標失敗: {e}")
            return False, 0.0
    
    def get_current_joint_position(self):
        """獲取當前關節位置"""
        try:
            # 使用GetAngle()獲取當前關節角度
            result = self.dashboard_api.GetAngle()
            print(f"當前關節位置查詢結果: {result}")
            
            # 解析返回結果，通常格式為字符串，需要解析
            # 假設返回格式類似: "{1.0,2.0,3.0,4.0}"
            if result and isinstance(result, str):
                # 移除大括號並分割
                result_str = result.strip('{}')
                joint_values = [float(x.strip()) for x in result_str.split(',')]
                
                if len(joint_values) >= 4:
                    return joint_values[:4]  # 返回前4個關節角度
                else:
                    print(f"關節角度數據不完整: {joint_values}")
                    return None
            else:
                print(f"無效的關節位置返回格式: {result}")
                return None
                
        except Exception as e:
            print(f"獲取當前關節位置失敗: {e}")
            return None
    
    def move_z_axis_only(self, target_z):
        """僅移動Z軸到指定高度 (使用JointMovJ避免手勢切換)"""
        try:
            move_start = time.time()
            
            # 獲取當前關節位置
            current_joints = self.get_current_joint_position()
            if current_joints is None:
                print("無法獲取當前關節位置，使用相對移動備用方案")
                return self.move_z_axis_relative(target_z)
            
            j1, j2, j3, j4 = current_joints
            print(f"當前關節位置: J1={j1:.2f}, J2={j2:.2f}, J3={j3:.2f}, J4={j4:.2f}")
            
            # 保持J1, J2, J4不變，只修改J3到目標Z軸高度
            target_j3 = target_z
            
            print(f"移動到目標關節位置: J1={j1:.2f}, J2={j2:.2f}, J3={target_j3:.2f}, J4={j4:.2f}")
            
            # 使用JointMovJ移動
            result = self.move_api.JointMovJ(j1, j2, target_j3, j4)
            self.move_api.Sync()
            
            move_time = time.time() - move_start
            print(f"Z軸移動完成: J3={target_j3:.2f}, 耗時: {move_time*1000:.1f}ms")
            return True, move_time
            
        except Exception as e:
            print(f"Z軸移動失敗: {e}")
            return False, 0.0
    
    def move_z_axis_relative(self, target_z):
        """備用Z軸移動方案 (使用相對關節移動)"""
        try:
            move_start = time.time()
            
            # 獲取當前笛卡爾座標
            current_pose_result = self.dashboard_api.GetPose()
            print(f"當前笛卡爾座標: {current_pose_result}")
            
            # 解析當前Z軸位置
            if current_pose_result and isinstance(current_pose_result, str):
                pose_str = current_pose_result.strip('{}')
                pose_values = [float(x.strip()) for x in pose_str.split(',')]
                
                if len(pose_values) >= 3:
                    current_z = pose_values[2]  # Z軸是第3個值
                    z_offset = target_z - current_z
                    
                    print(f"當前Z軸位置: {current_z:.2f}, 目標Z軸: {target_z:.2f}, 偏移量: {z_offset:.2f}")
                    
                    # 使用RelJointMovJ進行相對移動 (只移動J3)
                    result = self.move_api.RelJointMovJ(0, 0, z_offset, 0)
                    self.move_api.Sync()
                    
                    move_time = time.time() - move_start
                    print(f"Z軸相對移動完成: 偏移={z_offset:.2f}, 耗時: {move_time*1000:.1f}ms")
                    return True, move_time
                else:
                    print(f"笛卡爾座標數據不完整: {pose_values}")
                    return False, 0.0
            else:
                print("無法獲取當前笛卡爾座標")
                return False, 0.0
                
        except Exception as e:
            print(f"備用Z軸移動失敗: {e}")
            return False, 0.0
    
    def execute_integrated_cycle(self):
        """執行整合視覺夾取循環"""
        print(f"\n{'='*60}")
        print(f"開始執行整合視覺夾取循環 (50%速度)")
        print(f"{'='*60}")
        
        total_start_time = time.time()
        step_times = {}
        
        try:
            # 1. 初始化：移動到stanby點，初始化夾爪，夾爪閉合
            print(f"\n--- 步驟1: 系統初始化 ---")
            step_start = time.time()
            
            success, move_time = self.move_to_point("stanby")
            if not success:
                print("移動到stanby點失敗")
                return False
            
            if not self.initialize_gripper():
                print("夾爪初始化失敗")
                return False
            
            # 初始化過程中已包含夾爪關閉
            
            step_times["初始化"] = time.time() - step_start
            print(f"步驟1完成，耗時: {step_times['初始化']*1000:.1f}ms")
            
            # 2. CCD1拍照+檢測
            print(f"\n--- 步驟2: CCD1視覺檢測 ---")
            step_start = time.time()
            
            if not self.send_ccd1_command(CCD1_CMD_DETECT):
                print("CCD1檢測失敗")
                return False
            
            # 檢查檢測結果
            circle_count = self.read_modbus(CCD1_CIRCLE_COUNT_ADDR)
            print(f"檢測到圓形數量: {circle_count}")
            
            if circle_count is None or circle_count < 1:
                print("未檢測到圓形，動作結束")
                self.write_modbus(CONTROL_ADDR, 0)  # 清除控制狀態
                return True
            
            step_times["視覺檢測"] = time.time() - step_start
            print(f"步驟2完成，耗時: {step_times['視覺檢測']*1000:.1f}ms")
            
            # 3. 讀取第一個圓的世界座標
            print(f"\n--- 步驟3: 讀取目標座標 ---")
            step_start = time.time()
            
            world_x, world_y = self.read_world_coordinate()
            if world_x is None or world_y is None:
                print("無法獲取有效世界座標")
                return False
            
            step_times["座標讀取"] = time.time() - step_start
            print(f"步驟3完成，目標座標: X={world_x:.2f}mm, Y={world_y:.2f}mm，耗時: {step_times['座標讀取']*1000:.1f}ms")
            
            # 4. 夾取模式開始
            print(f"\n--- 步驟4: 夾取模式 ---")
            print(f"4.1 移動到VP_TOPSIDE")
            step_start = time.time()
            
            success, move_time = self.move_to_point("VP_TOPSIDE")
            if not success:
                print("移動到VP_TOPSIDE失敗")
                return False
            
            print(f"4.2 維持Z軸高度移動到圓心上方")
            vp_topside_point = self.robot_points["VP_TOPSIDE"]
            vp_z_height = vp_topside_point["cartesian"]["z"]
            
            success, move_time = self.move_to_world_coordinate(world_x, world_y, vp_z_height)
            if not success:
                print("移動到圓心上方失敗")
                return False
            
            print(f"4.3 下降到夾取高度並張開夾爪")
            success, move_time = self.move_z_axis_only(PICKUP_HEIGHT)
            if not success:
                print("下降到夾取高度失敗")
                return False
            
            if not self.gripper_open_to_position(370):
                print("夾爪張開失敗")
                return False
            
            print(f"4.4 智慧檢測夾爪狀態")
            time.sleep(1.0)  # 等待夾爪穩定
            grip_status, current_position = self.check_gripper_status()
            
            pickup_success = (grip_status in [PGC_STATUS_REACHED, PGC_STATUS_GRIPPED])
            if pickup_success:
                print("✓ 夾取成功")
            else:
                print("✗ 夾取失敗")
                return False
            
            step_times["夾取操作"] = time.time() - step_start
            print(f"步驟4完成，耗時: {step_times['夾取操作']*1000:.1f}ms")
            
            # 5. 運送流程
            print(f"\n--- 步驟5: 運送流程 ---")
            step_start = time.time()
            
            points_sequence = [
                "VP_TOPSIDE",  # 4.5
                "stanby",      # 4.6
                "Rotate_V2",   # 4.7
                "Rotate_top",  # 4.8
                "Rotate_down"  # 4.9
            ]
            
            for i, point_name in enumerate(points_sequence):
                print(f"4.{5+i} 移動到{point_name}")
                success, move_time = self.move_to_point(point_name)
                if not success:
                    print(f"移動到{point_name}失敗")
                    return False
                time.sleep(POINT_DELAY)
            
            print(f"4.10 夾爪關閉")
            if not self.gripper_close():
                print("夾爪關閉失敗")
                return False
            
            print(f"4.11 移動到Rotate_top")
            success, move_time = self.move_to_point("Rotate_top")
            if not success:
                print("移動到Rotate_top失敗")
                return False
            time.sleep(POINT_DELAY)
            
            print(f"4.12 移動到Rotate_V2")
            success, move_time = self.move_to_point("Rotate_V2")
            if not success:
                print("移動到Rotate_V2失敗")
                return False
            time.sleep(POINT_DELAY)
            
            print(f"4.13 移動到stanby")
            success, move_time = self.move_to_point("stanby")
            if not success:
                print("移動到stanby失敗")
                return False
            
            step_times["運送流程"] = time.time() - step_start
            print(f"步驟5完成，耗時: {step_times['運送流程']*1000:.1f}ms")
            
            # 6. 結束動作
            print(f"\n--- 步驟6: 動作完成 ---")
            total_time = time.time() - total_start_time
            
            print(f"\n{'='*60}")
            print(f"整合視覺夾取循環執行完成！")
            print(f"總執行時間: {total_time:.2f}秒")
            print(f"各步驟耗時分析:")
            for step_name, step_time in step_times.items():
                percentage = (step_time / total_time) * 100
                print(f"  {step_name}: {step_time:.2f}秒 ({percentage:.1f}%)")
            print(f"{'='*60}")
            
            return True
            
        except Exception as e:
            print(f"整合循環執行失敗: {e}")
            return False
    
    def state_machine_loop(self):
        """狀態機主循環"""
        print("整合系統狀態機開始運行...")
        
        while self.running:
            try:
                control_value = self.read_modbus(CONTROL_ADDR)
                
                if control_value == 1:
                    print("\n收到控制指令，開始執行整合動作...")
                    
                    # 設定運行狀態
                    self.write_modbus(RUNNING_ADDR, 1)
                    self.write_modbus(STANDBY_ADDR, 0)
                    
                    # 執行整合視覺夾取循環
                    success = self.execute_integrated_cycle()
                    
                    # 清除運行狀態
                    self.write_modbus(RUNNING_ADDR, 0)
                    
                    if success:
                        print("整合動作執行完成，等待控制指令清除...")
                    else:
                        print("整合動作執行失敗！")
                    
                    # 等待對方清除控制指令
                    while self.read_modbus(CONTROL_ADDR) == 1:
                        time.sleep(0.1)
                    
                    # 設定準備狀態
                    self.write_modbus(STANDBY_ADDR, 1)
                    print("準備好接收下次指令\n")
                
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print("\n收到中斷信號，停止狀態機...")
                break
            except Exception as e:
                print(f"狀態機異常: {e}")
                time.sleep(1)
    
    def start(self):
        """啟動整合系統"""
        print("=== MVP2.0機械臂+CCD1視覺整合系統 ===")
        
        # 載入點位數據
        if not self.load_robot_points():
            return
        
        # 檢查必要點位
        required_points = ["stanby", "VP_TOPSIDE", "Rotate_V2", "Rotate_top", "Rotate_down"]
        for point_name in required_points:
            if point_name not in self.robot_points:
                print(f"缺少必要點位: {point_name}")
                return
        
        # 連接Modbus
        if not self.connect_modbus():
            return
        
        # 連接機械臂
        if not self.connect_robot():
            return
        
        print(f"\n系統配置:")
        print(f"機械臂控制地址: {CONTROL_ADDR}")
        print(f"運行狀態地址: {RUNNING_ADDR}")
        print(f"準備狀態地址: {STANDBY_ADDR}")
        print(f"全局速度: {SPEED_RATIO}%")
        print(f"夾取高度: {PICKUP_HEIGHT}mm")
        print(f"CCD1視覺模組: 基地址200")
        print(f"PGC夾爪模組: 基地址500")
        print(f"\n等待控制指令 (向地址{CONTROL_ADDR}寫入1開始執行)...")
        
        # 啟動狀態機
        self.running = True
        try:
            self.state_machine_loop()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理資源"""
        print("清理資源...")
        
        self.running = False
        
        if self.modbus_client:
            self.write_modbus(RUNNING_ADDR, 0)
            self.write_modbus(STANDBY_ADDR, 0)
            self.modbus_client.close()
         
        if self.dashboard_api:
            self.dashboard_api.close()
        if self.move_api:
            self.move_api.close()
        
        print("資源清理完成")

def main():
    integrated_system = IntegratedRobotSystem()
    integrated_system.start()

if __name__ == "__main__":
    main()