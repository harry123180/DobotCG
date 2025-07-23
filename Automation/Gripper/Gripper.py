# Gripper_Optimized.py - 高性能夾爪模組
import os
import json
import time
import threading
from datetime import datetime
from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.exceptions import ModbusException
import logging
from logging.handlers import RotatingFileHandler
from collections import deque

class GripperModule:
    def __init__(self, config_file="gripper_config.json"):
        # 設置日誌
        self.logger = self.setup_logging()
        
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # 連接狀態
        self.main_server_client = None
        self.rtu_client = None
        self.is_running = False
        
        # 高性能線程鎖
        self.rtu_lock = threading.RLock()
        self.tcp_lock = threading.RLock()
        self.state_lock = threading.RLock()
        
        # 夾爪狀態 (僅PGC)
        self.gripper_states = {
            'PGC': {
                'connected': False, 
                'last_error': 0, 
                'error_count': 0,
                'last_status_update': 0,
                'status_cache': None
            }
        }
        
        # 高性能指令處理
        self.command_queue = deque(maxlen=100)
        self.last_command_ids = {'PGC': 0}
        
        # 優化的寄存器基地址配置
        self.register_mapping = {
            'PGC': {'status_base': 500, 'command_base': 520, 'unit_id': 6}
        }
        
        # 性能監控
        self.performance_stats = {
            'loop_count': 0,
            'command_count': 0,
            'error_count': 0,
            'avg_loop_time': 0,
            'max_loop_time': 0
        }
        
        # 優化參數
        self.fast_loop_interval = 0.02  # 20ms 高頻循環
        self.status_update_interval = 0.01  # 50ms 狀態更新
        self.connection_test_cycles = 50  # 每20個循環測試連接
        
        self.logger.info(f"夾爪模組啟動 - 優化版本 - 基地址: 500-539")
        self.logger.info(f"性能參數: 主循環={self.fast_loop_interval*1000}ms, 狀態更新={self.status_update_interval*1000}ms")

    def setup_logging(self):
        """設置高性能logging配置"""
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler = RotatingFileHandler(
            os.path.join(log_dir, 'gripper_optimized.log'),
            maxBytes=10*1024*1024,
            backupCount=7,
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        
        logger = logging.getLogger('GripperOptimized')
        logger.setLevel(logging.INFO)  # 優化：減少debug輸出
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger

    def load_config(self):
        default_config = {
            "module_id": "夾爪模組-優化版",
            "rtu_connection": {
                "port": "COM5",
                "baudrate": 115200,
                "parity": "N",
                "stopbits": 1,
                "timeout": 0.5  # 優化：減少超時時間
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 0.5  # 優化：減少超時時間
            },
            "modbus_mapping": {
                "base_address": 500
            },
            "timing": {
                "fast_loop_interval": 0.04,  # 優化：20ms高頻循環
                "status_update_interval": 0.05,  # 優化：50ms狀態更新
                "command_delay": 0.01,  # 優化：減少指令延遲
                "reconnect_interval": 3.0  # 優化：減少重連間隔
            },
            "grippers": {
                "PGC": {"unit_id": 6, "enabled": True}
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    # 合併預設值
                    for key, value in default_config.items():
                        if key not in config:
                            config[key] = value
                    return config
            except Exception as e:
                self.logger.error(f"配置檔案讀取錯誤: {e}", exc_info=True)
                return default_config
        else:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(default_config, f, indent=2, ensure_ascii=False)
            return default_config

    def connect_main_server(self):
        """優化的主服務器連接"""
        with self.tcp_lock:
            try:
                if self.main_server_client and self.main_server_client.connected:
                    return True
                    
                self.main_server_client = ModbusTcpClient(
                    host=self.config["tcp_server"]["host"],
                    port=self.config["tcp_server"]["port"],
                    timeout=self.config["tcp_server"]["timeout"]
                )
                
                if self.main_server_client.connect():
                    self.logger.info(f"主服務器連接成功: {self.config['tcp_server']['host']}:{self.config['tcp_server']['port']}")
                    return True
                else:
                    self.logger.warning("主服務器連接失敗，將在主循環中重試")
                    return False
            except Exception as e:
                self.logger.error(f"主服務器連接異常: {e}", exc_info=True)
                return False

    def connect_rtu_devices(self):
        """優化的RTU設備連接"""
        with self.rtu_lock:
            try:
                if self.rtu_client and self.rtu_client.connected:
                    return True
                    
                self.rtu_client = ModbusSerialClient(
                    port=self.config["rtu_connection"]["port"],
                    baudrate=self.config["rtu_connection"]["baudrate"],
                    parity=self.config["rtu_connection"]["parity"],
                    stopbits=self.config["rtu_connection"]["stopbits"],
                    timeout=self.config["rtu_connection"]["timeout"]
                )
                
                if self.rtu_client.connect():
                    self.logger.info(f"RTU設備連接成功: {self.config['rtu_connection']['port']}")
                    return True
                else:
                    self.logger.warning("RTU設備連接失敗，將在主循環中重試")
                    return False
            except Exception as e:
                self.logger.error(f"RTU設備連接異常: {e}", exc_info=True)
                return False

    def test_gripper_connection(self, gripper_type):
        """優化的夾爪連接測試"""
        if gripper_type != 'PGC':
            return False
            
        with self.rtu_lock:
            try:
                if not self.rtu_client or not self.rtu_client.connected:
                    with self.state_lock:
                        self.gripper_states[gripper_type]['connected'] = False
                    return False
                    
                unit_id = self.register_mapping[gripper_type]['unit_id']
                
                # 快速連接測試
                result = self.rtu_client.read_holding_registers(address=0x0200, count=1, slave=unit_id)
                
                if result and not result.isError():
                    with self.state_lock:
                        self.gripper_states[gripper_type]['connected'] = True
                    return True
                else:
                    with self.state_lock:
                        self.gripper_states[gripper_type]['connected'] = False
                    if self.performance_stats['loop_count'] % 100 == 0:  # 減少日誌頻率
                        self.logger.warning(f"PGC夾爪連接測試失敗 (unit_id={unit_id})")
                    return False
                    
            except Exception as e:
                with self.state_lock:
                    self.gripper_states[gripper_type]['connected'] = False
                    self.gripper_states[gripper_type]['error_count'] += 1
                self.logger.error(f"PGC夾爪連接測試異常: {e}", exc_info=True)
                return False

    def read_gripper_status(self, gripper_type):
        """優化的夾爪狀態讀取 - 帶快取機制"""
        if gripper_type != 'PGC':
            return None
        
        current_time = time.time()
        
        # 檢查快取是否有效
        with self.state_lock:
            last_update = self.gripper_states[gripper_type]['last_status_update']
            cached_status = self.gripper_states[gripper_type]['status_cache']
            
            if cached_status and (current_time - last_update) < self.status_update_interval:
                return cached_status
            
        with self.rtu_lock:
            try:
                if not self.rtu_client or not self.rtu_client.connected:
                    return None
                    
                unit_id = self.register_mapping[gripper_type]['unit_id']
                
                # 批量讀取提升效率
                result = self.rtu_client.read_holding_registers(address=0x0200, count=3, slave=unit_id)
                
                if result and not result.isError():
                    status_data = {
                        'init_status': result.registers[0],
                        'grip_status': result.registers[1], 
                        'position': result.registers[2],
                        'connected': True,
                        'timestamp': current_time
                    }
                    
                    # 更新快取
                    with self.state_lock:
                        self.gripper_states[gripper_type]['status_cache'] = status_data
                        self.gripper_states[gripper_type]['last_status_update'] = current_time
                    
                    return status_data
                else:
                    return None
                    
            except Exception as e:
                with self.state_lock:
                    self.gripper_states[gripper_type]['error_count'] += 1
                self.logger.error(f"PGC狀態讀取異常: {e}", exc_info=True)
                return None

    def execute_gripper_command(self, gripper_type, command, param1=0, param2=0):
        """優化的夾爪指令執行"""
        if gripper_type != 'PGC':
            return False
        
        with self.rtu_lock:
            try:
                if not self.rtu_client or not self.rtu_client.connected:
                    return False
                    
                unit_id = self.register_mapping[gripper_type]['unit_id']
                success = self.execute_pgc_command(unit_id, command, param1, param2)
                
                # 性能統計
                self.performance_stats['command_count'] += 1
                
                if not success:
                    with self.state_lock:
                        self.gripper_states[gripper_type]['error_count'] += 1
                    self.performance_stats['error_count'] += 1
                    
                return success
                
            except Exception as e:
                with self.state_lock:
                    self.gripper_states[gripper_type]['error_count'] += 1
                self.performance_stats['error_count'] += 1
                self.logger.error(f"PGC指令執行異常: {e}", exc_info=True)
                return False

    def execute_pgc_command(self, unit_id, command, param1, param2):
        """優化的PGC夾爪指令執行"""
        try:
            if command == 1:  # 初始化
                result = self.rtu_client.write_register(address=0x0100, value=0x01, slave=unit_id)
            elif command == 2:  # 停止
                result = self.rtu_client.write_register(address=0x0100, value=0, slave=unit_id)
            elif command == 3:  # 設定位置(絕對)
                result = self.rtu_client.write_register(address=0x0103, value=param1, slave=unit_id)
            elif command == 5:  # 設定力道
                result = self.rtu_client.write_register(address=0x0101, value=param1, slave=unit_id)
            elif command == 6:  # 設定速度
                result = self.rtu_client.write_register(address=0x0104, value=param1, slave=unit_id)
            elif command == 7:  # 開啟
                result = self.rtu_client.write_register(address=0x0103, value=1000, slave=unit_id)
            elif command == 8:  # 關閉
                result = self.rtu_client.write_register(address=0x0103, value=0, slave=unit_id)
            else:
                return True  # NOP或未知指令
                
            return not result.isError() if result else False
            
        except Exception as e:
            self.logger.error(f"PGC指令執行失敗: {e}", exc_info=True)
            return False

    def update_status_registers(self):
        """優化的狀態寄存器更新"""
        with self.tcp_lock:
            try:
                if not self.main_server_client or not self.main_server_client.connected:
                    return
                    
                gripper_type = 'PGC'
                mapping = self.register_mapping[gripper_type]
                status_base = mapping['status_base']
                
                # 讀取夾爪狀態
                status_data = self.read_gripper_status(gripper_type)
                
                # 準備寄存器數據
                registers = [0] * 20
                
                if status_data:
                    # 通用狀態
                    registers[0] = 1  # 模組狀態 - 在線
                    registers[1] = 1  # 連接狀態 - 已連接
                    registers[3] = self.gripper_states[gripper_type]['error_count']
                    registers[14] = int(time.time()) & 0xFFFF
                    
                    # PGC特定狀態
                    registers[2] = status_data.get('init_status', 0)
                    registers[4] = status_data.get('grip_status', 0)
                    registers[5] = status_data.get('position', 0)
                    
                    # 性能數據
                    registers[15] = self.performance_stats['command_count'] & 0xFFFF
                    registers[16] = self.performance_stats['loop_count'] & 0xFFFF
                else:
                    # 設備離線狀態
                    registers[0] = 0
                    registers[1] = 0
                    registers[3] = self.gripper_states[gripper_type]['error_count']
                
                # 批量寫入提升效率
                result = self.main_server_client.write_registers(
                    address=status_base,
                    values=registers,
                    slave=self.config["tcp_server"]["unit_id"]
                )
                
                if result and result.isError():
                    self.logger.warning(f"PGC狀態寄存器寫入失敗: {result}")
                        
            except Exception as e:
                self.logger.error(f"狀態寄存器更新異常: {e}", exc_info=True)

    def process_commands(self):
        """優化的指令處理 - 高優先權處理"""
        with self.tcp_lock:
            try:
                if not self.main_server_client or not self.main_server_client.connected:
                    return
                    
                gripper_type = 'PGC'
                mapping = self.register_mapping[gripper_type]
                command_base = mapping['command_base']
                
                # 讀取指令寄存器
                result = self.main_server_client.read_holding_registers(
                    address=command_base,
                    count=4,  # 只讀取需要的4個寄存器
                    slave=self.config["tcp_server"]["unit_id"]
                )
                
                if result and not result.isError():
                    command_id = result.registers[3]
                    
                    # 檢查新指令
                    if command_id != 0 and command_id != self.last_command_ids[gripper_type]:
                        self.last_command_ids[gripper_type] = command_id
                        
                        command = result.registers[0]
                        param1 = result.registers[1]
                        param2 = result.registers[2]
                        
                        self.logger.info(f"收到PGC指令: 代碼={command}, 參數1={param1}, 參數2={param2}, ID={command_id}")
                        
                        # 立即執行指令
                        success = self.execute_gripper_command(gripper_type, command, param1, param2)
                        
                        if success:
                            self.logger.info("PGC指令執行成功")
                        else:
                            self.logger.error("PGC指令執行失敗")
                        
                        # 快速清除指令寄存器
                        clear_values = [0, 0, 0, 0]
                        self.main_server_client.write_registers(
                            address=command_base,
                            values=clear_values,
                            slave=self.config["tcp_server"]["unit_id"]
                        )
                        
                        # 減少延遲
                        time.sleep(self.config["timing"]["command_delay"])
                        
            except Exception as e:
                self.logger.error(f"指令處理異常: {e}", exc_info=True)

    def fast_loop(self):
        """優化的高性能主循環"""
        self.logger.info("夾爪主循環啟動 - 高性能模式")
        
        loop_count = 0
        last_status_update = 0
        last_connection_test = 0
        
        while self.is_running:
            try:
                loop_start = time.time()
                
                # 連接檢查 - 降低頻率
                if not self.connect_main_server():
                    time.sleep(0.5)
                    continue
                    
                if not self.connect_rtu_devices():
                    time.sleep(0.5)
                    continue
                
                # 高優先權 - 指令處理 (每次循環)
                self.process_commands()
                
                # 中優先權 - 狀態更新 (50ms間隔)
                current_time = time.time()
                if current_time - last_status_update >= self.status_update_interval:
                    self.update_status_registers()
                    last_status_update = current_time
                
                # 低優先權 - 連接測試 (每20個循環)
                if loop_count % self.connection_test_cycles == 0:
                    if self.config["grippers"]["PGC"]["enabled"]:
                        self.test_gripper_connection('PGC')
                
                # 性能統計
                loop_count += 1
                self.performance_stats['loop_count'] = loop_count
                
                loop_elapsed = time.time() - loop_start
                self.performance_stats['avg_loop_time'] = (
                    self.performance_stats['avg_loop_time'] * 0.9 + loop_elapsed * 0.1
                )
                if loop_elapsed > self.performance_stats['max_loop_time']:
                    self.performance_stats['max_loop_time'] = loop_elapsed
                
                # 性能監控日誌 (每1000個循環)
                if loop_count % 1000 == 0:
                    self.logger.info(f"性能統計: 循環={loop_count}, 平均耗時={self.performance_stats['avg_loop_time']*1000:.1f}ms, "
                                   f"最大耗時={self.performance_stats['max_loop_time']*1000:.1f}ms, 指令數={self.performance_stats['command_count']}")
                
                # 精確控制循環頻率
                sleep_time = self.fast_loop_interval - loop_elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif loop_elapsed > self.fast_loop_interval * 1.5:
                    self.logger.warning(f"主循環執行時間過長: {loop_elapsed*1000:.1f}ms")
                    
            except KeyboardInterrupt:
                self.logger.info("收到中斷信號，停止主循環")
                break
            except Exception as e:
                self.logger.error(f"主循環異常: {e}", exc_info=True)
                time.sleep(0.1)

    def start(self):
        """啟動優化模組"""
        self.is_running = True
        
        # 啟動主循環線程
        self.main_thread = threading.Thread(target=self.fast_loop, daemon=True)
        self.main_thread.start()
        
        self.logger.info("夾爪模組已啟動 - 高性能模式")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """停止模組"""
        self.logger.info("正在停止夾爪模組...")
        self.is_running = False
        
        if self.main_server_client:
            self.main_server_client.close()
        if self.rtu_client:
            self.rtu_client.close()
            
        # 輸出最終性能統計
        self.logger.info(f"最終性能統計: 總循環={self.performance_stats['loop_count']}, "
                        f"總指令={self.performance_stats['command_count']}, "
                        f"總錯誤={self.performance_stats['error_count']}")
        
        self.logger.info("夾爪模組已停止")

if __name__ == "__main__":
    gripper_module = GripperModule()
    gripper_module.start()