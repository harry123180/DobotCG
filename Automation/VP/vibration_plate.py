# -*- coding: utf-8 -*-
"""
vibration_plate.py - 震動盤控制模組
支援ModbusTCP通訊協定，提供完整的震動盤控制功能
"""

from pymodbus.client import ModbusTcpClient
import threading
import time
import logging
from typing import Dict, Any, Optional, List

class VibrationPlate:
    """
    震動盤控制API類別
    支援ModbusTCP通訊協定
    """
    
    # 動作編碼映射
    ACTION_MAP = {
        'stop': 0,
        'up': 1,
        'down': 2,
        'left': 3,
        'right': 4,
        'upleft': 5,
        'downleft': 6,
        'upright': 7,
        'downright': 8,
        'horizontal': 9,
        'vertical': 10,
        'spread': 11
    }
    
    # 寄存器位址定義
    REGISTERS = {
        'single_action_trigger': 4,      # 單一動作觸發
        'vibration_status': 6,           # 震動盤狀態
        'backlight_test': 58,            # 背光測試開關
        'backlight_brightness': 46,      # 背光亮度
        
        # 強度寄存器 (20-30)
        'strength': {
            'up': 20, 'down': 21, 'left': 22, 'right': 23,
            'upleft': 24, 'downleft': 25, 'upright': 26, 'downright': 27,
            'horizontal': 28, 'vertical': 29, 'spread': 30
        },
        
        # 頻率寄存器 (60-70)
        'frequency': {
            'up': 60, 'down': 61, 'left': 62, 'right': 63,
            'upleft': 64, 'downleft': 65, 'upright': 66, 'downright': 67,
            'horizontal': 68, 'vertical': 69, 'spread': 70
        }
    }

    def __init__(self, ip: str, port: int, slave_id: int, auto_connect: bool = True):
        """
        初始化震動盤控制器
        
        Args:
            ip: ModbusTCP伺服器IP位址
            port: ModbusTCP伺服器埠口
            slave_id: 從機ID
            auto_connect: 是否自動連線
        """
        self.ip = ip
        self.port = port
        self.slave_id = slave_id
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.lock = threading.Lock()
        
        # 狀態快取
        self.current_action = 'stop'
        self.last_brightness = 128
        self.last_backlight_state = True
        self.action_parameters = {}
        
        # 初始化動作參數
        self.init_action_parameters()
        
        # 設定日誌
        self.logger = logging.getLogger(f"VibrationPlate_{ip}")
        
        if auto_connect:
            self.connect()

    def init_action_parameters(self):
        """初始化動作參數"""
        actions = ['up', 'down', 'left', 'right', 'upleft', 'downleft',
                  'upright', 'downright', 'horizontal', 'vertical', 'spread']
        
        for action in actions:
            self.action_parameters[action] = {
                'strength': 100,
                'frequency': 100
            }

    def connect(self) -> bool:
        """連線到ModbusTCP伺服器"""
        try:
            if self.client:
                self.client.close()
            
            self.client = ModbusTcpClient(host=self.ip, port=self.port)
            
            if self.client.connect():
                self.connected = True
                self.logger.info(f"成功連線到震動盤 {self.ip}:{self.port} (從機ID: {self.slave_id})")
                
                # 連線成功後讀取當前狀態
                self.read_current_status()
                
                return True
            else:
                self.connected = False
                self.logger.error(f"無法連線到震動盤 {self.ip}:{self.port}")
                return False
                
        except Exception as e:
            self.connected = False
            self.logger.error(f"連線異常: {e}")
            return False

    def disconnect(self):
        """中斷連線"""
        try:
            if self.client:
                # 停止所有動作
                self.stop()
                self.client.close()
                self.connected = False
                self.logger.info("震動盤連線已中斷")
        except Exception as e:
            self.logger.error(f"中斷連線時發生錯誤: {e}")

    def is_connected(self) -> bool:
        """檢查連線狀態"""
        if not self.client:
            return False
        
        try:
            # 嘗試讀取一個寄存器來檢查連線
            result = self.client.read_holding_registers(
                address=self.REGISTERS['vibration_status'],
                count=1,
                slave=self.slave_id
            )
            
            is_ok = not result.isError()
            self.connected = is_ok
            return is_ok
            
        except:
            self.connected = False
            return False

    def write_register(self, address: int, value: int) -> bool:
        """
        寫入單個寄存器
        
        Args:
            address: 寄存器位址
            value: 寫入值
            
        Returns:
            bool: 操作是否成功
        """
        if not self.is_connected():
            self.logger.warning("設備未連線，嘗試重新連線...")
            if not self.connect():
                return False

        try:
            with self.lock:
                response = self.client.write_register(
                    address=address, 
                    value=value, 
                    slave=self.slave_id
                )
                
                if response.isError():
                    self.logger.error(f"寫入寄存器 {address} 失敗: {response}")
                    return False
                else:
                    self.logger.debug(f"寄存器 {address} 設定為 {value}")
                    return True
                    
        except Exception as e:
            self.logger.error(f"寫入寄存器異常: {e}")
            self.connected = False
            return False

    def read_register(self, address: int) -> Optional[int]:
        """
        讀取單個寄存器
        
        Args:
            address: 寄存器位址
            
        Returns:
            int or None: 寄存器值，失敗返回None
        """
        if not self.is_connected():
            self.logger.warning("設備未連線，嘗試重新連線...")
            if not self.connect():
                return None

        try:
            with self.lock:
                response = self.client.read_holding_registers(
                    address=address, 
                    count=1, 
                    slave=self.slave_id
                )
                
                if response.isError():
                    self.logger.error(f"讀取寄存器 {address} 失敗: {response}")
                    return None
                else:
                    return response.registers[0]
                    
        except Exception as e:
            self.logger.error(f"讀取寄存器異常: {e}")
            self.connected = False
            return None

    def read_multiple_registers(self, start_address: int, count: int) -> Optional[List[int]]:
        """
        批量讀取寄存器
        
        Args:
            start_address: 起始位址
            count: 讀取數量
            
        Returns:
            list or None: 寄存器值列表，失敗返回None
        """
        if not self.is_connected():
            if not self.connect():
                return None

        try:
            with self.lock:
                response = self.client.read_holding_registers(
                    address=start_address, 
                    count=count, 
                    slave=self.slave_id
                )
                
                if response.isError():
                    self.logger.error(f"批量讀取寄存器失敗: {response}")
                    return None
                else:
                    return response.registers
                    
        except Exception as e:
            self.logger.error(f"批量讀取寄存器異常: {e}")
            self.connected = False
            return None

    def read_current_status(self):
        """讀取當前狀態"""
        try:
            # 讀取背光亮度
            brightness = self.read_register(self.REGISTERS['backlight_brightness'])
            if brightness is not None:
                self.last_brightness = brightness
            
            # 讀取震動狀態
            vibration_status = self.read_register(self.REGISTERS['vibration_status'])
            if vibration_status is not None:
                self.current_action = 'stop' if vibration_status == 0 else 'running'
            
            # 讀取動作參數
            self.read_action_parameters()
            
        except Exception as e:
            self.logger.error(f"讀取當前狀態失敗: {e}")

    def read_action_parameters(self):
        """讀取所有動作參數"""
        try:
            # 讀取強度寄存器
            strength_registers = self.read_multiple_registers(20, 11)
            # 讀取頻率寄存器
            frequency_registers = self.read_multiple_registers(60, 11)
            
            if strength_registers and frequency_registers:
                actions = ['up', 'down', 'left', 'right', 'upleft', 'downleft',
                          'upright', 'downright', 'horizontal', 'vertical', 'spread']
                
                for i, action in enumerate(actions):
                    self.action_parameters[action] = {
                        'strength': strength_registers[i],
                        'frequency': frequency_registers[i]
                    }
                    
        except Exception as e:
            self.logger.error(f"讀取動作參數失敗: {e}")

    def set_backlight(self, state: bool) -> bool:
        """
        設定背光開關
        
        Args:
            state: True開啟，False關閉
            
        Returns:
            bool: 操作是否成功
        """
        success = self.write_register(self.REGISTERS['backlight_test'], int(bool(state)))
        if success:
            self.last_backlight_state = state
            self.logger.info(f"背光{'開啟' if state else '關閉'}")
        return success

    def set_backlight_brightness(self, brightness: int) -> bool:
        """
        設定背光亮度
        
        Args:
            brightness: 亮度值 (0-255)
            
        Returns:
            bool: 操作是否成功
        """
        brightness = max(0, min(255, int(brightness)))
        success = self.write_register(self.REGISTERS['backlight_brightness'], brightness)
        if success:
            self.last_brightness = brightness
            self.logger.info(f"背光亮度設定為: {brightness}")
        return success

    def trigger_action(self, action: str) -> bool:
        """
        觸發指定動作
        
        Args:
            action: 動作名稱或編碼
            
        Returns:
            bool: 操作是否成功
        """
        if isinstance(action, str):
            if action not in self.ACTION_MAP:
                self.logger.error(f"未知動作: {action}")
                return False
            action_code = self.ACTION_MAP[action]
        else:
            action_code = int(action)
            
        success = self.write_register(self.REGISTERS['single_action_trigger'], action_code)
        if success:
            self.current_action = action if isinstance(action, str) else str(action_code)
            self.logger.info(f"觸發動作: {action} (代碼: {action_code})")
        return success

    def set_action_parameters(self, action: str, strength: int = None, frequency: int = None) -> bool:
        """
        設定動作參數
        
        Args:
            action: 動作名稱
            strength: 強度值 (0-255)
            frequency: 頻率值 (0-255)
            
        Returns:
            bool: 操作是否成功
        """
        if action not in self.REGISTERS['strength']:
            self.logger.error(f"未知動作: {action}")
            return False
            
        success = True
        
        # 更新本地快取
        if action not in self.action_parameters:
            self.action_parameters[action] = {'strength': 100, 'frequency': 100}
        
        if strength is not None:
            strength = max(0, min(255, int(strength)))
            if self.write_register(self.REGISTERS['strength'][action], strength):
                self.action_parameters[action]['strength'] = strength
                self.logger.debug(f"設定 {action} 強度: {strength}")
            else:
                success = False
            
        if frequency is not None:
            frequency = max(0, min(255, int(frequency)))
            if self.write_register(self.REGISTERS['frequency'][action], frequency):
                self.action_parameters[action]['frequency'] = frequency
                self.logger.debug(f"設定 {action} 頻率: {frequency}")
            else:
                success = False
                
        return success

    def execute_action(self, action: str, strength: int = None, frequency: int = None, duration: float = None) -> bool:
        """
        執行動作（設定參數並觸發）
        
        Args:
            action: 動作名稱
            strength: 強度值 (0-255)
            frequency: 頻率值 (0-255)
            duration: 持續時間(秒)，None表示不自動停止
            
        Returns:
            bool: 操作是否成功
        """
        # 設定參數
        if strength is not None or frequency is not None:
            if not self.set_action_parameters(action, strength, frequency):
                self.logger.warning(f"設定 {action} 參數失敗，但仍嘗試執行動作")
        
        # 觸發動作
        if not self.trigger_action(action):
            return False
        
        # 如果指定了持續時間，則延時後停止
        if duration is not None and duration > 0:
            def stop_after_delay():
                time.sleep(duration)
                self.stop()
                
            threading.Thread(target=stop_after_delay, daemon=True).start()
            self.logger.info(f"動作 {action} 將在 {duration} 秒後停止")
            
        return True

    def stop(self) -> bool:
        """停止所有動作"""
        success = self.trigger_action('stop')
        if success:
            self.current_action = 'stop'
            self.logger.info("所有動作已停止")
        return success

    def get_status(self) -> Dict[str, Any]:
        """
        取得震動盤狀態
        
        Returns:
            dict: 狀態資訊
        """
        status = {
            'connected': self.is_connected(),
            'vibration_active': False,
            'backlight_brightness': self.last_brightness,
            'backlight_on': self.last_backlight_state,
            'current_action': self.current_action,
            'action_parameters': self.action_parameters.copy(),
            'device_info': {
                'ip': self.ip,
                'port': self.port,
                'slave_id': self.slave_id
            }
        }
        
        if not self.is_connected():
            return status
            
        try:
            # 讀取即時震動狀態
            vibration_status = self.read_register(self.REGISTERS['vibration_status'])
            if vibration_status is not None:
                status['vibration_active'] = bool(vibration_status)
                
            # 讀取即時背光亮度
            brightness = self.read_register(self.REGISTERS['backlight_brightness'])
            if brightness is not None:
                status['backlight_brightness'] = brightness
                self.last_brightness = brightness
                
            # 更新動作參數
            self.read_action_parameters()
            status['action_parameters'] = self.action_parameters.copy()
            
        except Exception as e:
            self.logger.error(f"讀取狀態失敗: {e}")
                
        return status

    def get_action_list(self) -> List[str]:
        """取得所有可用動作列表"""
        return list(self.ACTION_MAP.keys())

    def get_register_map(self) -> Dict[str, Any]:
        """取得寄存器映射資訊"""
        return {
            'action_map': self.ACTION_MAP,
            'registers': self.REGISTERS,
            'device_info': {
                'ip': self.ip,
                'port': self.port,
                'slave_id': self.slave_id
            }
        }

    def test_connection(self) -> Dict[str, Any]:
        """測試連線"""
        test_result = {
            'success': False,
            'message': '',
            'details': {}
        }
        
        try:
            # 測試基本連線
            if not self.is_connected():
                if not self.connect():
                    test_result['message'] = f"無法連線到 {self.ip}:{self.port}"
                    return test_result
            
            # 測試讀取寄存器
            vibration_status = self.read_register(self.REGISTERS['vibration_status'])
            brightness = self.read_register(self.REGISTERS['backlight_brightness'])
            
            if vibration_status is not None and brightness is not None:
                test_result['success'] = True
                test_result['message'] = "連線測試成功"
                test_result['details'] = {
                    'vibration_status': vibration_status,
                    'backlight_brightness': brightness,
                    'connected': True
                }
            else:
                test_result['message'] = "無法讀取設備寄存器"
                
        except Exception as e:
            test_result['message'] = f"連線測試失敗: {e}"
            
        return test_result

    def __enter__(self):
        """上下文管理器入口"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()

    def __del__(self):
        """析構函數"""
        try:
            self.disconnect()
        except:
            pass