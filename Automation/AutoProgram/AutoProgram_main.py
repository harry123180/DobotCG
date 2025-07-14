#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_main.py - 機械臂協調控制模組 (簡化監控版)
基地址：1300-1399
專注負責：監控AutoFeeding(940)和Flow5完成狀態，maintain prepare_done=True
AutoFeeding已改為穩定供料模組，持續保持CG_F在保護區域
"""

import time
import os
import json
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


class SystemStatus(Enum):
    """系統狀態"""
    STOPPED = 0
    RUNNING = 1
    FLOW1_TRIGGERED = 2
    FLOW5_COMPLETED = 3
    ERROR = 4


class AutoProgramController:
    """機械臂協調控制模組 (簡化監控版)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址配置
        self.BASE_ADDRESS = 1300
        
        # AutoFeeding模組地址
        self.AF_CG_F_AVAILABLE = 940         # AutoFeeding CG_F可用標誌
        self.AF_TARGET_X_HIGH = 941            # 目標座標X高位
        self.AF_TARGET_X_LOW = 942             # 目標座標X低位
        self.AF_TARGET_Y_HIGH = 943            # 目標座標Y高位
        self.AF_TARGET_Y_LOW = 944             # 目標座標Y低位
        self.AF_COORDS_TAKEN = 945             # 座標已讀取標誌
        
        # Dobot M1Pro地址
        self.DOBOT_FLOW1_CONTROL = 1240        # Flow1控制
        self.DOBOT_FLOW1_COMPLETE = 1204       # Flow1完成狀態
        self.DOBOT_FLOW5_COMPLETE = 1206       # Flow5完成狀態
        
        # 載入配置
        self.config = self.load_config()
        
        # 系統狀態
        self.system_status = SystemStatus.STOPPED
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # 核心狀態變數
        self.prepare_done = False
        self.auto_program_enabled = True  # 自動程序啟用開關
        
        # 統計資訊
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        self.cg_f_taken_count = 0
        
        print("機械臂協調控制模組初始化完成 (簡化監控版)")
        print(f"Modbus服務器: {modbus_host}:{modbus_port}")
        print(f"AutoProgram基地址: {self.BASE_ADDRESS}")
        print(f"監控目標: AutoFeeding(940) + Flow5完成(1206)")
        print(f"目標: 維持prepare_done=True狀態")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autoprogram": {
                "coordination_interval": 0.2,      # 協調週期間隔(更快響應)
                "auto_program_enabled": True,      # 自動程序啟用
                "flow1_trigger_delay": 0.1,        # Flow1觸發延遲
                "coords_confirm_delay": 0.1,       # 座標確認延遲
            },
            "monitoring": {
                "cg_f_check_interval": 0.1,        # CG_F檢查間隔
                "flow5_check_interval": 0.2,       # Flow5檢查間隔
                "status_update_interval": 1.0,     # 狀態更新間隔
            },
            "timing": {
                "register_clear_delay": 0.05,      # 寄存器清除延遲
                "flow1_response_timeout": 10.0,    # Flow1響應超時
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autoprogram_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            print(f"配置檔案處理失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_system_registers()
            else:
                print(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def init_system_registers(self):
        """初始化系統寄存器"""
        try:
            # AutoProgram狀態寄存器 (1300-1319)
            self.write_register(1300, SystemStatus.STOPPED.value)  # 系統狀態
            self.write_register(1301, 0)  # prepare_done狀態
            self.write_register(1302, 1 if self.auto_program_enabled else 0)  # 自動程序啟用狀態
            self.write_register(1303, 0)  # AutoFeeding CG_F狀態
            self.write_register(1304, 0)  # Flow5完成狀態
            self.write_register(1305, 0)  # 協調週期計數
            self.write_register(1306, 0)  # Flow1觸發次數
            self.write_register(1307, 0)  # Flow5完成次數
            self.write_register(1308, 0)  # CG_F取得次數
            self.write_register(1309, 0)  # 錯誤代碼
            
            # AutoProgram控制寄存器 (1320-1339)
            self.write_register(1320, 0)  # 系統控制
            self.write_register(1321, 1 if self.auto_program_enabled else 0)  # 自動程序啟用控制
            self.write_register(1322, 0)  # 錯誤清除
            self.write_register(1323, 0)  # 強制重置
            
            # AutoFeeding狀態寄存器 (1340-1359)
            self.write_register(1340, 0)  # 目標座標X高位
            self.write_register(1341, 0)  # 目標座標X低位
            self.write_register(1342, 0)  # 目標座標Y高位
            self.write_register(1343, 0)  # 目標座標Y低位
            
            print("AutoProgram系統寄存器初始化完成")
        except Exception as e:
            print(f"系統寄存器初始化失敗: {e}")
    
    def read_register(self, address: int) -> Optional[int]:
        """讀取單個寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except Exception:
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            return False
    
    def read_32bit_coordinate(self, high_addr: int, low_addr: int) -> float:
        """讀取32位世界座標並轉換為實際值"""
        high_val = self.read_register(high_addr)
        low_val = self.read_register(low_addr)
        
        if high_val is None or low_val is None:
            return 0.0
        
        # 合併32位值
        combined = (high_val << 16) + low_val
        
        # 處理補碼(負數)
        if combined >= 2147483648:  # 2^31
            combined = combined - 4294967296  # 2^32
        
        # 轉換為毫米(除以100)
        return combined / 100.0
    
    def get_autofeeding_status(self) -> Dict[str, Any]:
        """獲取AutoFeeding模組狀態"""
        cg_f_available = self.read_register(self.AF_CG_F_AVAILABLE) or 0
        coords_taken = self.read_register(self.AF_COORDS_TAKEN) or 0
        
        target_x = 0.0
        target_y = 0.0
        
        if cg_f_available == 1:
            target_x = self.read_32bit_coordinate(self.AF_TARGET_X_HIGH, self.AF_TARGET_X_LOW)
            target_y = self.read_32bit_coordinate(self.AF_TARGET_Y_HIGH, self.AF_TARGET_Y_LOW)
        
        return {
            'cg_f_available': bool(cg_f_available),
            'coords_taken': bool(coords_taken),
            'target_x': target_x,
            'target_y': target_y
        }
    
    def take_cg_f_coordinates(self) -> Optional[tuple]:
        """讀取並確認CG_F座標"""
        af_status = self.get_autofeeding_status()
        
        if not af_status['cg_f_available']:
            return None
        
        # 複製座標到AutoProgram寄存器
        x_int = int(af_status['target_x'] * 100)
        y_int = int(af_status['target_y'] * 100)
        
        # 處理負數(補碼)
        if x_int < 0:
            x_int = x_int + 4294967296  # 2^32
        if y_int < 0:
            y_int = y_int + 4294967296  # 2^32
        
        # 分解為高低位
        x_high = (x_int >> 16) & 0xFFFF
        x_low = x_int & 0xFFFF
        y_high = (y_int >> 16) & 0xFFFF
        y_low = y_int & 0xFFFF
        
        # 寫入AutoProgram座標寄存器
        success = True
        success &= self.write_register(1340, x_high)  # 目標座標X高位
        success &= self.write_register(1341, x_low)   # 目標座標X低位
        success &= self.write_register(1342, y_high)  # 目標座標Y高位
        success &= self.write_register(1343, y_low)   # 目標座標Y低位
        
        if success:
            # 確認已讀取座標
            self.write_register(self.AF_COORDS_TAKEN, 1)
            time.sleep(self.config['autoprogram']['coords_confirm_delay'])
            
            self.cg_f_taken_count += 1
            print(f"[AutoProgram] ✓ 已讀取CG_F座標: ({af_status['target_x']:.2f}, {af_status['target_y']:.2f})")
            
            return (af_status['target_x'], af_status['target_y'])
        else:
            print("[AutoProgram] ✗ 座標複製失敗")
            return None
    
    def trigger_flow1(self) -> bool:
        """觸發Flow1取料作業"""
        print("[AutoProgram] 觸發Flow1取料作業")
        
        # 觸發Flow1控制
        if not self.write_register(self.DOBOT_FLOW1_CONTROL, 1):
            print(f"[AutoProgram] ✗ Flow1觸發失敗 (寫入{self.DOBOT_FLOW1_CONTROL}=1失敗)")
            return False
        
        time.sleep(self.config['autoprogram']['flow1_trigger_delay'])
        
        # 清除Flow1控制狀態
        self.write_register(self.DOBOT_FLOW1_CONTROL, 0)
        
        self.flow1_trigger_count += 1
        self.system_status = SystemStatus.FLOW1_TRIGGERED
        
        print(f"[AutoProgram] ✓ Flow1已觸發 (第{self.flow1_trigger_count}次)")
        return True
    
    def check_flow1_complete(self) -> bool:
        """檢查Flow1是否完成"""
        flow1_complete = self.read_register(self.DOBOT_FLOW1_COMPLETE)
        return flow1_complete == 1
    
    def clear_flow1_complete(self):
        """清除Flow1完成狀態"""
        self.write_register(self.DOBOT_FLOW1_COMPLETE, 0)
        print(f"[AutoProgram] Flow1完成狀態已清除 ({self.DOBOT_FLOW1_COMPLETE}=0)")
    
    def check_flow5_complete(self) -> bool:
        """檢查Flow5是否完成"""
        flow5_complete = self.read_register(self.DOBOT_FLOW5_COMPLETE)
        return flow5_complete == 1
    
    def clear_flow5_complete(self):
        """清除Flow5完成狀態"""
        self.write_register(self.DOBOT_FLOW5_COMPLETE, 0)
        self.flow5_complete_count += 1
        self.system_status = SystemStatus.FLOW5_COMPLETED
        print(f"[AutoProgram] Flow5完成狀態已清除 ({self.DOBOT_FLOW5_COMPLETE}=0，第{self.flow5_complete_count}次)")
    
    def check_control_registers(self):
        """檢查控制寄存器變更"""
        try:
            # 檢查系統控制寄存器 (1320)
            system_control = self.read_register(1320)
            if system_control == 1 and not self.running:
                print("[AutoProgram] 檢測到系統啟動指令 (1320=1)")
                self.start()
            elif system_control == 0 and self.running:
                print("[AutoProgram] 檢測到系統停止指令 (1320=0)")
                self.stop()
            
            # 檢查自動程序控制寄存器 (1321)
            auto_control = self.read_register(1321)
            if auto_control is not None:
                if auto_control != (1 if self.auto_program_enabled else 0):
                    self.auto_program_enabled = (auto_control == 1)
                    print(f"[AutoProgram] 自動程序啟用狀態更新: {self.auto_program_enabled} (1321={auto_control})")
            
        except Exception as e:
            print(f"[AutoProgram] 控制寄存器檢查異常: {e}")
    
    def coordination_cycle(self):
        """機械臂協調控制週期 (簡化版)"""
        try:
            self.coordination_cycle_count += 1
            
            # DEBUG: 每10個週期輸出一次狀態
            if self.coordination_cycle_count % 10 == 0:
                af_status = self.get_autofeeding_status()
                print(f"[AutoProgram] DEBUG - 週期{self.coordination_cycle_count}: "
                    f"prepare_done={self.prepare_done}, "
                    f"CG_F可用={af_status['cg_f_available']}, "
                    f"自動程序啟用={self.auto_program_enabled}")
            
            # 主要協調邏輯
            if not self.prepare_done:
                # prepare_done=False，需要執行Flow1讓機台準備好
                af_status = self.get_autofeeding_status()
                
                if af_status['cg_f_available']:
                    print(f"[AutoProgram] 檢測到CG_F可用，prepare_done=False，觸發Flow1")
                    
                    # 讀取座標
                    coords = self.take_cg_f_coordinates()
                    if coords:
                        # 觸發Flow1
                        if self.trigger_flow1():
                            print(f"[AutoProgram] ✓ Flow1已觸發，等待完成...")
                        else:
                            print(f"[AutoProgram] ✗ Flow1觸發失敗")
                    else:
                        print(f"[AutoProgram] ✗ 座標讀取失敗")
                else:
                    # DEBUG: 每50個週期輸出一次等待訊息
                    if self.coordination_cycle_count % 50 == 0:
                        print(f"[AutoProgram] 等待CG_F可用... (940={af_status.get('cg_f_available', 'N/A')})")
            else:
                # DEBUG: 每50個週期輸出一次prepare_done狀態
                if self.coordination_cycle_count % 50 == 0:
                    print(f"[AutoProgram] prepare_done=True，等待Flow5完成...")
            
            # 檢查Flow1完成狀態
            if self.check_flow1_complete():
                print("[AutoProgram] 檢測到Flow1完成")
                #self.clear_flow1_complete()
                self.prepare_done = True
                print("[AutoProgram] ✓ prepare_done=True，機台準備就緒")
            
            # 檢查Flow5完成狀態
            if self.check_flow5_complete():
                print("[AutoProgram] 檢測到Flow5完成，料件已送至組立區")
                self.clear_flow5_complete()
                self.prepare_done = False
                print("[AutoProgram] ✓ prepare_done=False，準備新週期")
            
        except Exception as e:
            print(f"[AutoProgram] 協調週期異常: {e}")
    
    def update_system_registers(self):
        """更新系統寄存器"""
        try:
            if not self.connected:
                return
            
            # 更新系統狀態
            self.write_register(1300, self.system_status.value)
            self.write_register(1301, 1 if self.prepare_done else 0)
            self.write_register(1302, 1 if self.auto_program_enabled else 0)
            
            # 更新AutoFeeding狀態
            af_status = self.get_autofeeding_status()
            self.write_register(1303, 1 if af_status['cg_f_available'] else 0)
            self.write_register(1304, 1 if self.check_flow5_complete() else 0)
            
            # 更新統計資訊
            self.write_register(1305, self.coordination_cycle_count)
            self.write_register(1306, self.flow1_trigger_count)
            self.write_register(1307, self.flow5_complete_count)
            self.write_register(1308, self.cg_f_taken_count)
            
            # 更新座標
            if af_status['cg_f_available']:
                x_int = int(af_status['target_x'] * 100)
                y_int = int(af_status['target_y'] * 100)
                
                if x_int < 0:
                    x_int = x_int + 4294967296
                if y_int < 0:
                    y_int = y_int + 4294967296
                
                self.write_register(1340, (x_int >> 16) & 0xFFFF)
                self.write_register(1341, x_int & 0xFFFF)
                self.write_register(1342, (y_int >> 16) & 0xFFFF)
                self.write_register(1343, y_int & 0xFFFF)
            
        except Exception as e:
            print(f"系統寄存器更新失敗: {e}")
    
    def start(self):
        """啟動機械臂協調控制系統"""
        if self.running:
            return
        
        print("[AutoProgram] === 啟動機械臂協調控制系統 (簡化監控版) ===")
        self.running = True
        self.system_status = SystemStatus.RUNNING
        
        # 重置狀態
        self.prepare_done = False
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow5_complete_count = 0
        self.cg_f_taken_count = 0
        
        # 立即更新狀態寄存器
        self.write_register(1300, SystemStatus.RUNNING.value)  # 更新系統狀態為運行中
        self.write_register(1301, 0)  # prepare_done=False
        
        self.thread = threading.Thread(target=self._coordination_loop, daemon=True)
        self.thread.start()
        
        print("[AutoProgram] 協調控制系統已啟動")
        print("[AutoProgram] 監控目標:")
        print(f"[AutoProgram]   - AutoFeeding CG_F可用標誌: {self.AF_CG_F_AVAILABLE}")
        print(f"[AutoProgram]   - Flow5完成狀態: {self.DOBOT_FLOW5_COMPLETE}")
        print("[AutoProgram] 目標: 維持prepare_done=True狀態")
    
    def stop(self):
        """停止機械臂協調控制系統"""
        if not self.running:
            return
        
        print("[AutoProgram] === 停止機械臂協調控制系統 ===")
        self.running = False
        self.system_status = SystemStatus.STOPPED
        
        # 立即更新狀態寄存器
        self.write_register(1300, SystemStatus.STOPPED.value)  # 更新系統狀態為停止
        
        # 更新系統寄存器
        self.update_system_registers()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        print("[AutoProgram] 協調控制系統已停止")
        self.print_statistics()
    
    def _coordination_loop(self):
        """協調控制主循環"""
        interval = self.config['autoprogram']['coordination_interval']
        
        print("[AutoProgram] 協調控制主循環已啟動")
        
        loop_count = 0
        while True:  # 持續運行，即使系統停止也要檢查控制寄存器
            try:
                loop_count += 1
                
                # DEBUG: 每100次循環輸出一次心跳
                if loop_count % 100 == 0:
                    print(f"[AutoProgram] 控制循環心跳 - 第{loop_count}次, running={self.running}, auto_enabled={self.auto_program_enabled}")
                
                # 總是檢查控制寄存器變更
                self.check_control_registers()
                
                # 只有在系統運行且自動程序啟用時才執行協調邏輯
                if self.running and self.auto_program_enabled:
                    self.coordination_cycle()
                
                time.sleep(interval)
                
            except Exception as e:
                print(f"[AutoProgram] 協調循環異常: {e}")
                time.sleep(1.0)
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def print_statistics(self):
        """輸出統計資訊"""
        print(f"\n=== AutoProgram統計資訊 ===")
        print(f"協調週期數: {self.coordination_cycle_count}")
        print(f"Flow1觸發次數: {self.flow1_trigger_count}")
        print(f"Flow5完成次數: {self.flow5_complete_count}")
        print(f"CG_F取得次數: {self.cg_f_taken_count}")
        print(f"當前prepare_done狀態: {self.prepare_done}")
        print(f"自動程序啟用: {self.auto_program_enabled}")
    
    def get_status_info(self) -> Dict[str, Any]:
        """獲取狀態資訊"""
        # 讀取AutoFeeding狀態
        af_status = self.get_autofeeding_status()
        
        return {
            "connected": self.connected,
            "system_status": self.system_status.name,
            "running": self.running,
            "auto_program_enabled": self.auto_program_enabled,
            "prepare_done": self.prepare_done,
            "autofeeding_status": af_status,
            "flow1_complete": self.check_flow1_complete(),
            "flow5_complete": self.check_flow5_complete(),
            "statistics": {
                "coordination_cycle_count": self.coordination_cycle_count,
                "flow1_trigger_count": self.flow1_trigger_count,
                "flow5_complete_count": self.flow5_complete_count,
                "cg_f_taken_count": self.cg_f_taken_count
            }
        }


def main():
    """主程序"""
    print("機械臂協調控制模組啟動 (簡化監控版)")
    print("目標: 監控AutoFeeding(940) + Flow5完成(1206)，維持prepare_done=True")
    
    # 創建控制器
    controller = AutoProgramController()
    
    # 連接Modbus
    if not controller.connect():
        print("Modbus連接失敗，程序退出")
        return
    
    try:
        # 啟動控制循環 (不管系統是否啟動，都要檢查控制寄存器)
        print("[AutoProgram] 啟動控制循環，等待指令...")
        controller.thread = threading.Thread(target=controller._coordination_loop, daemon=True)
        controller.thread.start()
        
        # 定期更新系統寄存器
        def update_registers():
            while True:  # 持續更新
                try:
                    controller.update_system_registers()
                    time.sleep(1.0)
                except Exception as e:
                    print(f"寄存器更新異常: {e}")
                    time.sleep(2.0)
        
        update_thread = threading.Thread(target=update_registers, daemon=True)
        update_thread.start()
        
        # 主循環 - 等待用戶操作
        print("\n指令說明:")
        print("  s - 顯示狀態")
        print("  start - 手動啟動系統")
        print("  stop - 手動停止系統")
        print("  enable - 啟用自動程序")
        print("  disable - 停用自動程序")
        print("  flow1 - 手動觸發Flow1")
        print("  clear_f1 - 清除Flow1完成狀態")
        print("  clear_f5 - 清除Flow5完成狀態")
        print("  coords - 手動讀取CG_F座標")
        print("  check_af - 檢查AutoFeeding狀態")
        print("  check_regs - 檢查控制寄存器")
        print("  q - 退出程序")
        
        while True:
            try:
                cmd = input("\n請輸入指令: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    status = controller.get_status_info()
                    print(f"\n系統狀態:")
                    for key, value in status.items():
                        if isinstance(value, dict):
                            print(f"  {key}:")
                            for sub_key, sub_value in value.items():
                                print(f"    {sub_key}: {sub_value}")
                        else:
                            print(f"  {key}: {value}")
                elif cmd == 'start':
                    controller.write_register(1320, 1)
                    print("系統啟動指令已發送 (1320=1)")
                elif cmd == 'stop':
                    controller.write_register(1320, 0)
                    print("系統停止指令已發送 (1320=0)")
                elif cmd == 'enable':
                    controller.auto_program_enabled = True
                    controller.write_register(1321, 1)
                    print("自動程序已啟用")
                elif cmd == 'disable':
                    controller.auto_program_enabled = False
                    controller.write_register(1321, 0)
                    print("自動程序已停用")
                elif cmd == 'flow1':
                    if controller.trigger_flow1():
                        print("Flow1已觸發")
                    else:
                        print("Flow1觸發失敗")
                elif cmd == 'clear_f1':
                    controller.clear_flow1_complete()
                elif cmd == 'clear_f5':
                    controller.clear_flow5_complete()
                elif cmd == 'coords':
                    coords = controller.take_cg_f_coordinates()
                    if coords:
                        print(f"CG_F座標: {coords}")
                    else:
                        print("無可用的CG_F座標")
                elif cmd == 'check_af':
                    af_status = controller.get_autofeeding_status()
                    print(f"AutoFeeding狀態: {af_status}")
                elif cmd == 'check_regs':
                    reg1320 = controller.read_register(1320)
                    reg1321 = controller.read_register(1321)
                    reg940 = controller.read_register(940)
                    print(f"控制寄存器: 1320={reg1320}, 1321={reg1321}")
                    print(f"AutoFeeding: 940={reg940}")
                    print(f"系統狀態: running={controller.running}, auto_enabled={controller.auto_program_enabled}")
                else:
                    print("無效指令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    finally:
        # 清理資源
        if controller.running:
            controller._stop_system()  # 使用內部停止方法避免線程問題
        controller.disconnect()
        print("程序已退出")


if __name__ == "__main__":
    main()