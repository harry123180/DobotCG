#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主Modbus Server測試工具
用於測試XC100模組的連接和寄存器讀寫
"""

import time
import sys
from pymodbus.client import ModbusTcpClient

class ModbusServerTester:
    def __init__(self, host="127.0.0.1", port=502, unit_id=1):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.client = None
        self.connected = False
        
    def connect(self):
        """連接到主Modbus Server"""
        try:
            print(f"🔌 正在連接到 {self.host}:{self.port}")
            self.client = ModbusTcpClient(host=self.host, port=self.port, timeout=3)
            
            if self.client.connect():
                self.connected = True
                print("✅ 連接成功")
                return True
            else:
                print("❌ 連接失敗")
                return False
        except Exception as e:
            print(f"❌ 連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.client and self.connected:
            self.client.close()
            self.connected = False
            print("🔌 連接已斷開")
    
    def read_xc_status(self, base_address=1000):
        """讀取XC100模組狀態"""
        if not self.connected:
            print("❌ 未連接到服務器")
            return None
        
        try:
            # 讀取狀態寄存器
            result = self.client.read_holding_registers(
                address=base_address, 
                count=15, 
                slave=self.unit_id
            )
            
            if result.isError():
                print(f"❌ 讀取失敗: {result}")
                return None
            
            registers = result.registers
            
            # 解析狀態
            state_map = {
                0: "離線", 1: "閒置", 2: "移動中", 3: "原點復歸中",
                4: "錯誤", 5: "Servo關閉", 6: "緊急停止"
            }
            
            status = {
                "module_state": state_map.get(registers[0], f"未知({registers[0]})"),
                "xc_connected": "是" if registers[1] == 1 else "否",
                "servo_status": "ON" if registers[2] == 1 else "OFF",
                "error_code": registers[3],
                "current_position": (registers[5] << 16) | registers[4],
                "target_position": (registers[7] << 16) | registers[6],
                "command_executing": "是" if registers[8] == 1 else "否",
                "comm_errors": registers[9],
                "position_A": (registers[11] << 16) | registers[10],
                "position_B": (registers[13] << 16) | registers[12],
                "timestamp": registers[14]
            }
            
            return status
            
        except Exception as e:
            print(f"❌ 讀取狀態異常: {e}")
            return None
    
    def send_command(self, command, param1=0, param2=0, base_address=1000):
        """發送指令到XC100模組"""
        if not self.connected:
            print("❌ 未連接到服務器")
            return False
        
        try:
            command_addr = base_address + 20
            command_id = int(time.time()) % 65536  # 生成唯一ID
            
            # 寫入指令
            values = [command, param1, param2, command_id, 0]
            result = self.client.write_registers(
                address=command_addr,
                values=values,
                slave=self.unit_id
            )
            
            if result.isError():
                print(f"❌ 發送指令失敗: {result}")
                return False
            
            command_names = {
                1: "Servo ON", 2: "Servo OFF", 3: "原點復歸",
                4: "絕對移動", 6: "緊急停止", 7: "錯誤重置"
            }
            
            print(f"✅ 指令發送成功: {command_names.get(command, f'指令{command}')} (ID: {command_id})")
            return True
            
        except Exception as e:
            print(f"❌ 發送指令異常: {e}")
            return False
    
    def init_registers(self, base_address=1000, register_count=50):
        """初始化寄存器區域"""
        if not self.connected:
            print("❌ 未連接到服務器")
            return False
        
        try:
            print(f"🔧 初始化寄存器區域: {base_address} - {base_address + register_count - 1}")
            
            # 初始化狀態寄存器
            status_values = [0] * 15  # 狀態區域清零
            result1 = self.client.write_registers(
                address=base_address,
                values=status_values,
                slave=self.unit_id
            )
            
            # 初始化指令寄存器
            command_values = [0] * 5  # 指令區域清零
            result2 = self.client.write_registers(
                address=base_address + 20,
                values=command_values,
                slave=self.unit_id
            )
            
            if result1.isError() or result2.isError():
                print(f"❌ 初始化失敗: 狀態={result1}, 指令={result2}")
                return False
            
            print("✅ 寄存器初始化成功")
            return True
            
        except Exception as e:
            print(f"❌ 初始化異常: {e}")
            return False
    
    def send_move_to_position(self, position, base_address=1000):
        """發送移動到指定位置的指令"""
        # 將32位位置分解為兩個16位參數
        param1 = position & 0xFFFF          # 低16位
        param2 = (position >> 16) & 0xFFFF  # 高16位
        
        print(f"🎯 發送移動指令: 位置={position}, 參數=({param1}, {param2})")
        return self.send_command(4, param1, param2, base_address)  # 4 = MOVE_ABS
    
    def send_servo_on(self, base_address=1000):
        """發送Servo ON指令"""
        return self.send_command(1, 0, 0, base_address)
    
    def send_servo_off(self, base_address=1000):
        """發送Servo OFF指令"""
        return self.send_command(2, 0, 0, base_address)
    
    def send_home(self, base_address=1000):
        """發送原點復歸指令"""
        return self.send_command(3, 0, 0, base_address)
    
    def send_emergency_stop(self, base_address=1000):
        """發送緊急停止指令"""
        return self.send_command(6, 0, 0, base_address)
    
    def quick_commands_menu(self, base_address=1000):
        """快捷指令選單"""
        print("\n🎮 XC100快捷控制選單")
        print("=" * 30)
        
        while True:
            print("\n可用指令:")
            print("1. Servo ON")
            print("2. Servo OFF") 
            print("3. 原點復歸 (HOME)")
            print("4. 移動到A點 (400)")
            print("5. 移動到B點 (2682)")
            print("6. 自定義位置移動")
            print("7. 緊急停止")
            print("8. 查看當前狀態")
            print("9. 監控模式")
            print("0. 退出")
            
            try:
                choice = input("\n請選擇 (0-9): ").strip()
                
                if choice == '0':
                    print("👋 退出控制選單")
                    break
                elif choice == '1':
                    print("🟢 執行 Servo ON...")
                    self.send_servo_on(base_address)
                elif choice == '2':
                    print("🔴 執行 Servo OFF...")
                    self.send_servo_off(base_address)
                elif choice == '3':
                    print("🏠 執行原點復歸...")
                    self.send_home(base_address)
                elif choice == '4':
                    print("📍 移動到A點 (400)...")
                    self.send_move_to_position(400, base_address)
                elif choice == '5':
                    print("📍 移動到B點 (2682)...")
                    self.send_move_to_position(2682, base_address)
                elif choice == '6':
                    try:
                        position = int(input("請輸入目標位置: "))
                        print(f"📍 移動到自定義位置 ({position})...")
                        self.send_move_to_position(position, base_address)
                    except ValueError:
                        print("❌ 請輸入有效的數字")
                elif choice == '7':
                    print("🛑 執行緊急停止...")
                    self.send_emergency_stop(base_address)
                elif choice == '8':
                    print("📊 讀取當前狀態...")
                    status = self.read_xc_status(base_address)
                    if status:
                        print("\n當前狀態:")
                        print(f"  模組狀態: {status['module_state']}")
                        print(f"  XC100連接: {status['xc_connected']}")
                        print(f"  Servo狀態: {status['servo_status']}")
                        print(f"  當前位置: {status['current_position']}")
                        print(f"  目標位置: {status['target_position']}")
                        print(f"  指令執行中: {status['command_executing']}")
                        print(f"  錯誤代碼: {status['error_code']}")
                elif choice == '9':
                    print("📊 進入監控模式 (按Ctrl+C返回選單)...")
                    try:
                        self.monitor_loop(base_address, 1)
                    except KeyboardInterrupt:
                        print("\n🔙 返回選單")
                else:
                    print("❌ 無效選擇，請重新輸入")
                
                # 執行指令後等待一下再顯示狀態
                if choice in ['1', '2', '3', '4', '5', '6', '7']:
                    time.sleep(0.5)
                    status = self.read_xc_status(base_address)
                    if status:
                        print(f"✅ 指令完成 | 狀態: {status['module_state']} | 執行中: {status['command_executing']}")
                        
            except KeyboardInterrupt:
                print("\n🔙 返回選單")
            except Exception as e:
                print(f"❌ 操作異常: {e}")
    
    def batch_test(self, base_address=1000):
        """批量測試功能"""
        print("\n🧪 執行批量測試...")
        
        tests = [
            ("Servo ON", lambda: self.send_servo_on(base_address)),
            ("移動到A點(400)", lambda: self.send_move_to_position(400, base_address)),
            ("等待移動完成", lambda: time.sleep(3)),
            ("移動到B點(2682)", lambda: self.send_move_to_position(2682, base_address)),
            ("等待移動完成", lambda: time.sleep(3)),
            ("原點復歸", lambda: self.send_home(base_address)),
            ("等待復歸完成", lambda: time.sleep(3)),
            ("Servo OFF", lambda: self.send_servo_off(base_address))
        ]
        
        for i, (desc, action) in enumerate(tests, 1):
            print(f"步驟 {i}/{len(tests)}: {desc}")
            try:
                action()
                if "等待" not in desc:
                    time.sleep(0.5)  # 短暫等待
                    status = self.read_xc_status(base_address)
                    if status:
                        print(f"  ✅ {desc} 完成 | 狀態: {status['module_state']}")
            except Exception as e:
                print(f"  ❌ {desc} 失敗: {e}")
                break
        
        print("🎉 批量測試完成！")
        """監控循環"""
        print(f"📊 開始監控XC100模組 (地址: {base_address}, 間隔: {interval}秒)")
        print("按 Ctrl+C 停止監控")
        
        try:
            while True:
                status = self.read_xc_status(base_address)
                if status:
                    print(f"\r[{time.strftime('%H:%M:%S')}] "
                          f"狀態:{status['module_state']} | "
                          f"XC100:{status['xc_connected']} | "
                          f"Servo:{status['servo_status']} | "
                          f"位置:{status['current_position']} | "
                          f"錯誤:{status['error_code']}", end="")
                else:
                    print(f"\r[{time.strftime('%H:%M:%S')}] ❌ 讀取失敗", end="")
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\n🛑 監控停止")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='主Modbus Server測試工具')
    parser.add_argument('--host', default='127.0.0.1', help='服務器地址')
    parser.add_argument('--port', type=int, default=502, help='服務器端口')
    parser.add_argument('--unit-id', type=int, default=1, help='單元ID')
    parser.add_argument('--base-address', type=int, default=1000, help='XC100模組基地址')
    parser.add_argument('--action', choices=['test', 'monitor', 'init', 'command', 'menu', 'move-a', 'move-b', 'servo-on', 'servo-off', 'home', 'stop', 'batch'], 
                       default='menu', help='操作類型')
    parser.add_argument('--command', type=int, help='發送的指令代碼')
    parser.add_argument('--param1', type=int, default=0, help='指令參數1')
    parser.add_argument('--param2', type=int, default=0, help='指令參數2')
    parser.add_argument('--position', type=int, help='移動目標位置')
    
    args = parser.parse_args()
    
    print("🧪 主Modbus Server測試工具 - 增強版")
    print("=" * 50)
    
    tester = ModbusServerTester(args.host, args.port, args.unit_id)
    
    if not tester.connect():
        sys.exit(1)
    
    try:
        if args.action == 'test':
            # 基本測試
            print("\n📋 執行基本測試...")
            
            # 1. 初始化寄存器
            tester.init_registers(args.base_address)
            
            # 2. 讀取狀態
            status = tester.read_xc_status(args.base_address)
            if status:
                print("\n📊 當前狀態:")
                for key, value in status.items():
                    print(f"  {key}: {value}")
            
        elif args.action == 'monitor':
            # 監控模式
            tester.monitor_loop(args.base_address)
            
        elif args.action == 'init':
            # 初始化寄存器
            tester.init_registers(args.base_address)
            
        elif args.action == 'command':
            # 發送指令
            if args.command is None:
                print("❌ 請指定 --command 參數")
                print("可用指令: 1=Servo ON, 2=Servo OFF, 3=原點復歸, 4=絕對移動, 6=緊急停止")
            else:
                tester.send_command(args.command, args.param1, args.param2, args.base_address)
                
                # 等待一下然後檢查狀態
                time.sleep(1)
                status = tester.read_xc_status(args.base_address)
                if status:
                    print(f"指令後狀態: {status['module_state']}, 執行中: {status['command_executing']}")
        
        elif args.action == 'menu':
            # 交互式選單模式
            tester.quick_commands_menu(args.base_address)
            
        elif args.action == 'move-a':
            # 快捷移動到A點
            print("📍 移動到A點 (400)")
            tester.send_move_to_position(400, args.base_address)
            
        elif args.action == 'move-b':
            # 快捷移動到B點
            print("📍 移動到B點 (2682)")
            tester.send_move_to_position(2682, args.base_address)
            
        elif args.action == 'servo-on':
            # 快捷Servo ON
            print("🟢 Servo ON")
            tester.send_servo_on(args.base_address)
            
        elif args.action == 'servo-off':
            # 快捷Servo OFF
            print("🔴 Servo OFF")
            tester.send_servo_off(args.base_address)
            
        elif args.action == 'home':
            # 快捷原點復歸
            print("🏠 原點復歸")
            tester.send_home(args.base_address)
            
        elif args.action == 'stop':
            # 快捷緊急停止
            print("🛑 緊急停止")
            tester.send_emergency_stop(args.base_address)
            
        elif args.action == 'batch':
            # 批量測試
            tester.batch_test(args.base_address)
        
        # 對於快捷指令，顯示執行後的狀態
        if args.action in ['move-a', 'move-b', 'servo-on', 'servo-off', 'home', 'stop']:
            time.sleep(0.5)
            status = tester.read_xc_status(args.base_address)
            if status:
                print(f"✅ 指令完成 | 狀態: {status['module_state']} | 位置: {status['current_position']}")
        
    finally:
        tester.disconnect()

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()