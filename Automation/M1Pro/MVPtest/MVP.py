#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MVP2.0 - 機械臂狀態機+夾爪整合控制程序
基地址400規劃，整合PGC夾爪控制
優化版本：調整夾爪撐開檢測邏輯，關閉動作快速執行
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

# PGC夾爪寄存器地址 (基地址500, 指令區520-529)
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

# 測試參數
SPEED_RATIO = 100  # 全局速度100% (最大速度)
CYCLE_COUNT = 10  # 10個完整循環
POINT_DELAY = 0.5  # 兩點間等待0.5秒

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

class RobotGripperStateMachine:
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
            result = self.modbus_client.read_holding_registers(address, count=1)
            
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
            result = self.modbus_client.write_register(address, value)
            
            if hasattr(result, 'isError') and result.isError():
                return False
            else:
                return True
                
        except Exception as e:
            print(f"寫入寄存器{address}={value}異常: {e}")
            return False
    
    def test_modbus_communication(self):
        """測試Modbus通訊是否正常"""
        try:
            print("=== Modbus通訊測試 ===")
            
            # 測試讀取PGC狀態寄存器
            print("測試讀取PGC狀態寄存器...")
            for addr in range(500, 506):
                value = self.read_modbus(addr)
                print(f"寄存器 {addr}: {value}")
            
            # 測試寫入指令 (使用Gripper_app的方式)
            print("\n測試寫入指令 (批量方式)...")
            command_base = 520
            test_values = [3, 370, 0, 999, 0, 0, 0, 0, 0, 0]  # 指令3, 位置370, ID=999
            
            try:
                result = self.modbus_client.write_registers(
                    address=command_base,
                    values=test_values,
                    slave=1
                )
                print(f"批量寫入結果: {result}")
                print(f"是否錯誤: {result.isError() if result else 'No result'}")
            except Exception as e:
                print(f"批量寫入異常: {e}")
            
            # 等待一下，檢查指令是否被處理
            time.sleep(1)
            
            # 讀取指令寄存器確認
            print("\n檢查指令寄存器...")
            for addr in range(520, 524):
                value = self.read_modbus(addr)
                print(f"指令寄存器 {addr}: {value}")
                
            return True
            
        except Exception as e:
            print(f"Modbus通訊測試失敗: {e}")
            return False
    
    def check_gripper_module_status(self):
        """檢查Gripper模組是否在TCP Server上運作"""
        try:
            module_status = self.read_modbus(PGC_MODULE_STATUS_ADDR)
            connect_status = self.read_modbus(PGC_CONNECT_STATUS_ADDR)
            
            if module_status == 1 and connect_status == 1:
                print("Gripper模組確認在TCP Server上運作")
                return True
            else:
                print(f"Gripper模組狀態異常: module_status={module_status}, connect_status={connect_status}")
                return False
                
        except Exception as e:
            print(f"檢查Gripper模組狀態失敗: {e}")
            return False
        """檢查Gripper模組是否在TCP Server上運作"""
        try:
            module_status = self.read_modbus(PGC_MODULE_STATUS_ADDR)
            connect_status = self.read_modbus(PGC_CONNECT_STATUS_ADDR)
            
            if module_status == 1 and connect_status == 1:
                print("Gripper模組確認在TCP Server上運作")
                return True
            else:
                print(f"Gripper模組狀態異常: module_status={module_status}, connect_status={connect_status}")
                return False
                
        except Exception as e:
            print(f"檢查Gripper模組狀態失敗: {e}")
            return False
    
    def get_next_command_id(self):
        """獲取下一個指令ID"""
        self.command_id_counter += 1
        return self.command_id_counter
    
    def send_gripper_command(self, command, param1=0, param2=0, timeout=15.0, wait_for_completion=True, target_position=None):
        """發送夾爪指令，可選擇是否等待完成，可指定目標位置進行確認"""
        try:
            cmd_id = self.get_next_command_id()
            start_time = time.time()
            
            # 發送指令
            self.write_modbus(PGC_COMMAND_ADDR, command)
            self.write_modbus(PGC_PARAM1_ADDR, param1)
            self.write_modbus(PGC_PARAM2_ADDR, param2)
            self.write_modbus(PGC_CMD_ID_ADDR, cmd_id)
            
            send_time = time.time()
            print(f"發送PGC指令: cmd={command}, param1={param1}, id={cmd_id} (發送耗時: {(send_time - start_time)*1000:.1f}ms)")
            
            # 如果不需要等待完成，直接返回成功
            if not wait_for_completion:
                print(f"PGC指令發送完成，不等待狀態確認: cmd={command}")
                return True
            
            # 步驟1: 等待指令執行完成 (指令ID清零)
            while time.time() - start_time < timeout:
                current_cmd_id = self.read_modbus(PGC_CMD_ID_ADDR)
                if current_cmd_id == 0:
                    id_clear_time = time.time()
                    print(f"PGC指令ID已清零: cmd={command} (ID清零耗時: {(id_clear_time - start_time)*1000:.1f}ms)")
                    break
                time.sleep(0.05)
            else:
                timeout_time = time.time() - start_time
                print(f"PGC指令ID清零超時: cmd={command} (超時時間: {timeout_time*1000:.1f}ms)")
                return False
            
            # 步驟2: 如果指定目標位置，等待位置到達
            if target_position is not None:
                print(f"等待夾爪到達目標位置: {target_position}")
                position_wait_start = time.time()
                position_reached = False
                
                while time.time() - position_wait_start < 5.0:  # 最多等待5秒
                    current_position = self.read_modbus(PGC_POSITION_ADDR)
                    if current_position is not None:
                        position_diff = abs(current_position - target_position)
                        print(f"當前位置: {current_position}, 目標位置: {target_position}, 差值: {position_diff}")
                        
                        if position_diff <= 5:  # 允許5個單位的誤差
                            position_time = time.time()
                            print(f"位置到達確認: 當前={current_position}, 目標={target_position} (位置確認耗時: {(position_time - position_wait_start)*1000:.1f}ms)")
                            position_reached = True
                            break
                    
                    time.sleep(0.1)
                
                if not position_reached:
                    final_position = self.read_modbus(PGC_POSITION_ADDR)
                    print(f"位置到達超時: 最終位置={final_position}, 目標位置={target_position}")
                    return False
            
            # 步驟3: 等待夾爪狀態穩定
            print("等待夾爪狀態穩定...")
            stable_wait_start = time.time()
            stable_count = 0
            required_stable_readings = 2  # 需要連續2次穩定讀取
            
            while time.time() - stable_wait_start < 3.0:  # 保持3秒超時
                grip_status = self.read_modbus(PGC_GRIP_STATUS_ADDR)
                
                if grip_status in [PGC_STATUS_REACHED, PGC_STATUS_GRIPPED]:  # 到達或夾住狀態
                    stable_count += 1
                    print(f"夾爪狀態穩定讀取 {stable_count}/{required_stable_readings}: {grip_status}")
                    
                    if stable_count >= required_stable_readings:
                        complete_time = time.time()
                        total_time = complete_time - start_time
                        stable_time = complete_time - stable_wait_start
                        print(f"PGC指令執行完成: cmd={command} (總耗時: {total_time*1000:.1f}ms, 穩定確認耗時: {stable_time*1000:.1f}ms)")
                        return True
                else:
                    stable_count = 0  # 重置穩定計數
                    
                time.sleep(0.05)
            
            # 狀態穩定超時但位置正確，仍然認為成功
            final_time = time.time() - start_time
            print(f"PGC指令狀態穩定超時但視為完成: cmd={command} (總耗時: {final_time*1000:.1f}ms)")
            return True
            
        except Exception as e:
            error_time = time.time() - start_time if 'start_time' in locals() else 0
            print(f"發送PGC指令失敗: {e} (錯誤發生時間: {error_time*1000:.1f}ms)")
            return False
    
    def initialize_gripper(self):
        """初始化PGC夾爪並設定最大速度"""
        print("=== 初始化PGC夾爪 ===")
        
        # 檢查當前狀態
        print("檢查夾爪初始化前狀態...")
        for addr in range(500, 506):
            value = self.read_modbus(addr)
            print(f"寄存器 {addr}: {value}")
        
        # 初始化重試機制
        max_init_attempts = 3
        init_success = False
        
        for attempt in range(max_init_attempts):
            print(f"\n--- 初始化嘗試 {attempt + 1}/{max_init_attempts} ---")
            
            # 發送初始化指令
            print("發送初始化指令...")
            result = self.send_gripper_command_batch(PGC_CMD_INIT)
            if not result:
                print("夾爪初始化指令發送失敗")
                continue
            
            # 等待初始化完成（監控寄存器502）
            print("等待夾爪初始化完成...")
            init_start_time = time.time()
            init_timeout = 10.0  # 每次嘗試最多等待10秒
            
            while time.time() - init_start_time < init_timeout:
                device_status = self.read_modbus(PGC_DEVICE_STATUS_ADDR)  # 寄存器502
                
                if device_status == 1:
                    init_time = time.time() - init_start_time
                    print(f"夾爪初始化成功！耗時: {init_time:.2f}秒")
                    init_success = True
                    break
                elif device_status == 2:
                    elapsed = time.time() - init_start_time
                    print(f"初始化進行中... (狀態: {device_status}, 已等待: {elapsed:.1f}秒)")
                elif device_status == 0:
                    elapsed = time.time() - init_start_time
                    print(f"初始化尚未開始 (狀態: {device_status}, 已等待: {elapsed:.1f}秒)")
                else:
                    print(f"初始化狀態異常: {device_status}")
                
                time.sleep(0.2)
            
            if init_success:
                break
            else:
                final_status = self.read_modbus(PGC_DEVICE_STATUS_ADDR)
                print(f"第{attempt + 1}次初始化超時！最終狀態: {final_status}")
                if attempt < max_init_attempts - 1:
                    print("準備重新嘗試初始化...")
                    time.sleep(1)  # 等待1秒後重試
        
        if not init_success:
            print("夾爪初始化失敗，已達到最大重試次數")
            return False
        
        # 步驟3: 設定最大速度
        print("\n設定夾爪最大速度...")
        result2 = self.send_gripper_command_batch(6, 100)  # 指令6=設定速度
        if not result2:
            print("夾爪速度設定失敗")
            return False
        time.sleep(0.5)
        
        # 步驟4: 設定最大力道
        print("設定夾爪最大力道...")
        result3 = self.send_gripper_command_batch(5, 100)  # 指令5=設定力道
        if not result3:
            print("夾爪力道設定失敗")
            return False
        time.sleep(0.5)
        
        # 步驟5: 檢查最終狀態
        print("\n檢查初始化完成後狀態...")
        for addr in range(500, 506):
            value = self.read_modbus(addr)
            print(f"寄存器 {addr}: {value}")
        
        # 確認初始化真正完成
        final_device_status = self.read_modbus(PGC_DEVICE_STATUS_ADDR)
        if final_device_status == 1:
            print("PGC夾爪完整初始化完成並確認")
            return True
        else:
            print(f"初始化完成但狀態異常: {final_device_status}")
            return False
    
    def send_gripper_command_batch(self, command, param1=0, param2=0):
        """使用批量寫入方式發送夾爪指令（模仿Gripper_app方式）"""
        try:
            cmd_id = self.get_next_command_id()
            command_base = 520
            
            # 構建指令數組 (10個寄存器)
            values = [command, param1, param2, cmd_id, 0, 0, 0, 0, 0, 0]
            
            print(f"批量發送PGC指令: cmd={command}, param1={param1}, id={cmd_id}")
            
            result = self.modbus_client.write_registers(
                address=command_base,
                values=values,
                slave=1
            )
            
            if result and not result.isError():
                print(f"批量指令發送成功")
                return True
            else:
                print(f"批量指令發送失敗: {result}")
                return False
                
        except Exception as e:
            print(f"批量發送PGC指令失敗: {e}")
            return False
    
    def gripper_close_fast(self):
        """夾爪快速關閉 (釋放物件，不等待狀態確認)"""
        print("=== 夾爪快速關閉 (釋放物件) ===")
        result = self.send_gripper_command_batch(PGC_CMD_QUICK_CLOSE)
        if result:
            print("夾爪關閉指令發送完成 (釋放物件)")
        else:
            print("夾爪關閉指令發送失敗")
        return result
    
    def gripper_open_to_position_and_check(self, position):
        """夾爪打開到指定位置並檢測是否到達狀態（智能判斷版）"""
        print(f"=== 夾爪撐開到位置 {position} 並檢測到達狀態 ===")
        
        # 記錄初始位置
        initial_position = self.read_modbus(PGC_POSITION_ADDR)
        print(f"初始位置: {initial_position}")
        
        # 使用批量寫入的方式發送位置指令
        result = self.send_gripper_command_batch(PGC_CMD_ABS_POS, position)
        
        if result:
            print(f"位置指令發送成功，等待夾爪移動到 {position}")
            
            # 等待位置變化，最多等待10秒
            start_time = time.time()
            max_position_reached = initial_position if initial_position else 0
            
            while time.time() - start_time < 10.0:
                current_position = self.read_modbus(PGC_POSITION_ADDR)
                grip_status = self.read_modbus(PGC_GRIP_STATUS_ADDR)
                
                if current_position is not None:
                    # 記錄到達的最大位置
                    if current_position > max_position_reached:
                        max_position_reached = current_position
                    
                    position_diff = abs(current_position - position)
                    movement_from_start = abs(current_position - (initial_position if initial_position else 0))
                    
                    print(f"當前位置: {current_position}, 目標位置: {position}, 差值: {position_diff}, 移動距離: {movement_from_start}, 狀態: {grip_status}")
                    
                    # 智能判斷成功條件：
                    # 1. 位置接近目標 (差值 <= 20) 
                    # 2. 或者：有明顯移動 (>100) 且狀態為夾住 (2)
                    # 3. 或者：位置停止變化且狀態穩定
                    
                    if position_diff <= 20:
                        print(f"✓ 夾爪成功到達目標位置範圍: {current_position} (誤差: {position_diff})")
                        return True
                    elif movement_from_start > 100 and grip_status == 2:
                        print(f"✓ 夾爪成功撐開固定物件: 位置 {current_position}, 移動距離 {movement_from_start}")
                        return True
                    elif current_position == max_position_reached and movement_from_start > 50:
                        # 檢查位置是否停止變化（連續3次相同位置）
                        stable_count = 0
                        for _ in range(3):
                            time.sleep(0.1)
                            check_position = self.read_modbus(PGC_POSITION_ADDR)
                            if check_position == current_position:
                                stable_count += 1
                        
                        if stable_count >= 2:
                            print(f"✓ 夾爪位置穩定，撐開物件: 位置 {current_position}, 移動距離 {movement_from_start}")
                            return True
                
                time.sleep(0.2)
            
            # 超時最終檢查
            final_position = self.read_modbus(PGC_POSITION_ADDR)
            final_status = self.read_modbus(PGC_GRIP_STATUS_ADDR)
            final_movement = abs(final_position - (initial_position if initial_position else 0))
            
            print(f"等待超時 - 最終位置: {final_position}, 狀態: {final_status}, 總移動距離: {final_movement}")
            
            # 最後機會：如果有顯著移動就認為成功
            if final_movement > 100:
                print(f"✓ 超時但夾爪有顯著移動，認為撐開成功: {final_position}")
                return True
            else:
                print(f"✗ 夾爪移動不足，可能撞開失敗")
                return False
        else:
            print(f"夾爪位置指令發送失敗")
            return False
    
    def check_gripper_reached(self):
        """檢查夾爪是否到達指定位置或撐開固定"""
        check_start_time = time.time()
        grip_status = self.read_modbus(PGC_GRIP_STATUS_ADDR)
        check_time = (time.time() - check_start_time) * 1000
        
        print(f"夾爪狀態檢查: 數值={grip_status}, 檢查耗時: {check_time:.1f}ms")
        
        if grip_status == PGC_STATUS_REACHED:
            print("✓ 夾爪狀態: 已到達指定位置")
            return True
        elif grip_status == PGC_STATUS_GRIPPED:
            print("✓ 夾爪狀態: 撐開固定物體 (有效狀態)")
            return True
        else:
            status_desc = {
                PGC_STATUS_MOVING: "運動中",
                PGC_STATUS_DROPPED: "物體掉落"
            }.get(grip_status, f"未知狀態({grip_status})")
            print(f"✗ 夾爪狀態: {status_desc}")
            return False
    
    def move_to_point(self, point_name):
        """移動到指定點位"""
        if point_name not in self.robot_points:
            print(f"點位{point_name}不存在")
            return False
            
        point = self.robot_points[point_name]
        joint = point["joint"]
        
        try:
            result = self.move_api.JointMovJ(
                joint["j1"], 
                joint["j2"], 
                joint["j3"], 
                joint["j4"]
            )
            print(f"移動到{point_name}: J1={joint['j1']:.2f}, J2={joint['j2']:.2f}, J3={joint['j3']:.2f}, J4={joint['j4']:.2f}")
            return True
            
        except Exception as e:
            print(f"移動到{point_name}失敗: {e}")
            return False
    
    def execute_enhanced_cycle_motion(self):
        """執行優化版循環運動 (夾爪撐開檢測，關閉快速執行)"""
        print(f"開始執行{CYCLE_COUNT}個完整循環 (優化夾爪控制邏輯)...")
        
        total_start_time = time.time()
        
        for cycle in range(CYCLE_COUNT):
            cycle_start_time = time.time()
            print(f"\n{'='*50}")
            print(f"第{cycle+1}/{CYCLE_COUNT}個循環開始")
            print(f"{'='*50}")
            
            # 1. 夾爪快速關閉 (釋放物件)
            step_start = time.time()
            if not self.gripper_close_fast():
                print("夾爪關閉失敗，停止執行")
                return False
            step1_time = time.time() - step_start
            print(f"步驟1完成耗時: {step1_time*1000:.1f}ms\n")
            
            # 2. 往Rotate_top移動
            step_start = time.time()
            if not self.move_to_point("Rotate_top"):
                print("移動到Rotate_top失敗，停止執行")
                return False
            
            self.move_api.Sync()
            time.sleep(POINT_DELAY)
            step2_time = time.time() - step_start
            print(f"步驟2完成耗時: {step2_time*1000:.1f}ms\n")
            
            # 3. 往Rotate_down移動
            step_start = time.time()
            if not self.move_to_point("Rotate_down"):
                print("移動到Rotate_down失敗，停止執行")
                return False
                
            self.move_api.Sync()
            step3_time = time.time() - step_start
            print(f"步驟3完成耗時: {step3_time*1000:.1f}ms\n")
            
            # 4. 夾爪撐開至370位置並檢測到達狀態
            step_start = time.time()
            reached_ok = self.gripper_open_to_position_and_check(370)
            if not reached_ok:
                print("夾爪撐開到370位置失敗或未到達，停止執行")
                return False
            step4_time = time.time() - step_start
            print(f"步驟4完成耗時: {step4_time*1000:.1f}ms\n")
            
            # 5. 往Rotate_top移動
            step_start = time.time()
            if not self.move_to_point("Rotate_top"):
                print("移動到Rotate_top失敗，停止執行")
                return False
            
            self.move_api.Sync()
            time.sleep(POINT_DELAY)
            step5_time = time.time() - step_start
            print(f"步驟5完成耗時: {step5_time*1000:.1f}ms\n")
            
            # 6. 往Rotate_down移動
            step_start = time.time()
            if not self.move_to_point("Rotate_down"):
                print("移動到Rotate_down失敗，停止執行")
                return False
                
            self.move_api.Sync()
            time.sleep(POINT_DELAY)
            step6_time = time.time() - step_start
            print(f"步驟6完成耗時: {step6_time*1000:.1f}ms\n")
            
            cycle_time = time.time() - cycle_start_time
            gripper_time = step1_time + step4_time
            robot_time = step2_time + step3_time + step5_time + step6_time
            
            print(f"第{cycle+1}個循環完成")
            print(f"循環總耗時: {cycle_time:.2f}秒")
            print(f"夾爪操作耗時: {gripper_time:.2f}秒 ({gripper_time/cycle_time*100:.1f}%)")
            print(f"機械臂操作耗時: {robot_time:.2f}秒 ({robot_time/cycle_time*100:.1f}%)")
        
        total_time = time.time() - total_start_time
        print(f"\n{'='*50}")
        print(f"所有{CYCLE_COUNT}個循環執行完成！")
        print(f"總執行時間: {total_time:.2f}秒")
        print(f"平均每循環: {total_time/CYCLE_COUNT:.2f}秒")
        print(f"{'='*50}")
        return True
    
    def state_machine_loop(self):
        """狀態機主循環"""
        print("狀態機開始運行...")
        
        while self.running:
            try:
                control_value = self.read_modbus(CONTROL_ADDR)
                
                if control_value == 1:
                    print("\n收到控制指令，開始執行動作...")
                    
                    # 設定運行狀態
                    self.write_modbus(RUNNING_ADDR, 1)
                    self.write_modbus(STANDBY_ADDR, 0)
                    
                    # 執行優化版循環運動
                    success = self.execute_enhanced_cycle_motion()
                    
                    # 清除運行狀態
                    self.write_modbus(RUNNING_ADDR, 0)
                    
                    if success:
                        print("動作執行完成，等待控制指令清除...")
                    else:
                        print("動作執行失敗！")
                    
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
        """啟動測試程序"""
        print("=== 機械臂+夾爪整合狀態機程序 (優化版本) ===")
        
        # 載入點位數據
        if not self.load_robot_points():
            return
        
        # 檢查必要點位
        required_points = ["Rotate_top", "Rotate_down"]
        for point_name in required_points:
            if point_name not in self.robot_points:
                print(f"缺少必要點位: {point_name}")
                return
        
        # 連接Modbus
        if not self.connect_modbus():
            return
        
        # 測試Modbus通訊
        print("=== 測試Modbus通訊 ===")
        if not self.test_modbus_communication():
            print("Modbus通訊測試失敗，請檢查連接")
            return
        
        # 檢查Gripper模組
        if not self.check_gripper_module_status():
            print("Gripper模組未運行，請先啟動Gripper.py")
            return
        
        # 初始化PGC夾爪
        if not self.initialize_gripper():
            print("PGC夾爪初始化失敗")
            return
        
        # 連接機械臂
        if not self.connect_robot():
            return
        
        print(f"\n系統配置 (優化版本):")
        print(f"機械臂控制地址: {CONTROL_ADDR}")
        print(f"運行狀態地址: {RUNNING_ADDR}")
        print(f"準備狀態地址: {STANDBY_ADDR}")
        print(f"PGC夾爪基地址: {PGC_COMMAND_BASE}")
        print(f"夾爪控制邏輯: 撐開檢測到達位置，關閉快速執行")
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
    robot_gripper_sm = RobotGripperStateMachine()
    robot_gripper_sm.start()

if __name__ == "__main__":
    main()