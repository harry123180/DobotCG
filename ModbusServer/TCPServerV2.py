# modbus_tcp_server_optimized.py
# 高性能版本 - 針對10個模組併發連接優化
# 重點：響應速度、穩定性、記憶體效率

import logging
import threading
import time
import json
import os
import sys
import signal
from datetime import datetime
from flask import Flask, jsonify
from pymodbus.server import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
import traceback

def setup_logging():
    """設定高性能日誌配置"""
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    
    # 減少日誌輸出，只記錄重要訊息
    file_handler = logging.FileHandler('modbus_server.log', encoding='utf-8')
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(logging.WARNING)  # 只記錄警告和錯誤
    
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    console_handler.setLevel(logging.INFO)
    
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(file_handler)
    root_logger.addHandler(console_handler)
    
    # 抑制第三方庫日誌
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    logging.getLogger('pymodbus').setLevel(logging.ERROR)

class HighPerformanceDataBlock(ModbusSequentialDataBlock):
    """高性能數據塊 - 移除同步化機制，減少鎖競爭"""
    
    def __init__(self, address, values):
        super().__init__(address, values)
        self._lock = threading.RLock()  # 使用可重入鎖
        self.total_reads = 0
        self.total_writes = 0
    
    def getValues(self, address, count=1):
        """優化讀取操作 - 減少鎖持有時間"""
        with self._lock:
            self.total_reads += 1
            return super().getValues(address, count)
    
    def setValues(self, address, values):
        """優化寫入操作 - 減少鎖持有時間"""
        with self._lock:
            self.total_writes += 1
            return super().setValues(address, values)
    
    def get_stats(self):
        """獲取統計資訊"""
        return {
            'total_reads': self.total_reads,
            'total_writes': self.total_writes
        }

class ModbusTCPServerOptimized:
    def __init__(self):
        self.slave_id = 1
        self.server_host = "0.0.0.0"
        self.server_port = 502
        self.web_port = 8000
        
        # 初始化3000個寄存器
        self.register_count = 3000
        self.registers = [0] * self.register_count
        
        # 高性能相關
        self.server = None
        self.context = None
        self.slave_context = None
        self.data_block = None
        self.server_running = False
        self.shutdown_event = threading.Event()
        self.start_time = time.time()
        
        # 統計資訊
        self.stats = {
            'client_connections': 0,
            'total_operations': 0,
            'start_time': time.time()
        }
        
        # 輕量級Web介面
        self.flask_app = Flask(__name__)
        self.setup_minimal_web()
        
        # 信號處理
        self.setup_signal_handlers()
        
        logging.info("高性能Modbus TCP Server初始化完成")
    
    def setup_signal_handlers(self):
        """設定信號處理器"""
        def signal_handler(signum, frame):
            logging.info(f"收到關閉信號 {signum}")
            self.shutdown()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    
    def create_modbus_context(self):
        """創建高性能Modbus上下文"""
        try:
            # 使用高性能數據塊
            self.data_block = HighPerformanceDataBlock(0, self.registers)
            
            # 創建Slave上下文 - 只配置必要的寄存器類型
            self.slave_context = ModbusSlaveContext(
                hr=self.data_block,  # Holding Registers (主要使用)
                ir=self.data_block,  # Input Registers (共用以節省記憶體)
                # 不初始化DI和CO以節省記憶體
            )
            
            # 創建伺服器上下文
            slaves = {self.slave_id: self.slave_context}
            self.context = ModbusServerContext(slaves=slaves, single=False)
            
            logging.info("高性能Modbus上下文創建成功")
            return self.context
            
        except Exception as e:
            logging.error(f"創建Modbus上下文失敗: {e}")
            return None
    
    def read_register_direct(self, address):
        """直接讀取寄存器 - 繞過Modbus層"""
        try:
            if 0 <= address < self.register_count:
                return self.registers[address]
            return None
        except Exception as e:
            logging.error(f"直接讀取寄存器失敗: {e}")
            return None
    
    def write_register_direct(self, address, value):
        """直接寫入寄存器 - 繞過Modbus層"""
        try:
            if 0 <= address < self.register_count and 0 <= value <= 65535:
                self.registers[address] = value
                # 同步到Modbus上下文 (批量操作更高效)
                if self.data_block:
                    self.data_block.setValues(address, [value])
                return True
            return False
        except Exception as e:
            logging.error(f"直接寫入寄存器失敗: {e}")
            return False
    
    def batch_write_registers(self, start_address, values):
        """批量寫入寄存器 - 高效操作"""
        try:
            if start_address + len(values) <= self.register_count:
                # 批量更新內部陣列
                for i, value in enumerate(values):
                    if 0 <= value <= 65535:
                        self.registers[start_address + i] = value
                
                # 批量同步到Modbus上下文
                if self.data_block:
                    self.data_block.setValues(start_address, values)
                
                return True
            return False
        except Exception as e:
            logging.error(f"批量寫入失敗: {e}")
            return False
    
    def get_server_stats(self):
        """獲取伺服器統計資訊"""
        try:
            uptime = time.time() - self.start_time
            
            # 獲取數據塊統計
            data_stats = {}
            if self.data_block:
                data_stats = self.data_block.get_stats()
            
            # 計算非零寄存器數量 (採樣前100個以提高效率)
            sample_size = min(100, self.register_count)
            non_zero_sample = sum(1 for i in range(sample_size) if self.registers[i] != 0)
            
            return {
                'server_running': self.server_running,
                'uptime_seconds': round(uptime, 2),
                'total_registers': self.register_count,
                'slave_id': self.slave_id,
                'non_zero_sample': non_zero_sample,
                'sample_size': sample_size,
                'client_connections': self.stats['client_connections'],
                'data_block_stats': data_stats,
                'version': '2.0.0-optimized'
            }
        except Exception as e:
            logging.error(f"獲取統計失敗: {e}")
            return {'error': str(e)}
    
    def setup_minimal_web(self):
        """設定精簡Web介面 - 只保留必要功能"""
        
        @self.flask_app.errorhandler(Exception)
        def handle_exception(e):
            logging.error(f"Web錯誤: {e}")
            return jsonify({'error': 'Internal server error'}), 500
        
        @self.flask_app.route('/')
        def index():
            return '''
            <html>
            <head><title>Modbus TCP Server - 高性能版</title></head>
            <body>
                <h1>Modbus TCP Server - 高性能版本</h1>
                <p>版本: 2.0.0-optimized</p>
                <p>設計用途: 10個模組併發連接</p>
                <p><a href="/api/status">查看伺服器狀態</a></p>
                <p><a href="/api/register_sample">查看寄存器採樣</a></p>
            </body>
            </html>
            '''
        
        @self.flask_app.route('/api/status')
        def api_status():
            return jsonify(self.get_server_stats())
        
        @self.flask_app.route('/api/register_sample')
        def api_register_sample():
            """獲取寄存器採樣 - 只顯示前50個非零寄存器"""
            try:
                non_zero = {}
                count = 0
                for i in range(min(500, self.register_count)):  # 只檢查前500個
                    if self.registers[i] != 0:
                        non_zero[i] = self.registers[i]
                        count += 1
                        if count >= 50:  # 最多顯示50個
                            break
                
                return jsonify({
                    'success': True,
                    'non_zero_registers': non_zero,
                    'count': len(non_zero),
                    'scanned_range': min(500, self.register_count)
                })
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.flask_app.route('/api/register/<int:address>')
        def api_get_register(address):
            """讀取單一寄存器"""
            try:
                value = self.read_register_direct(address)
                if value is not None:
                    return jsonify({'address': address, 'value': value})
                else:
                    return jsonify({'error': 'Invalid address'}), 400
            except Exception as e:
                return jsonify({'error': str(e)}), 500
        
        @self.flask_app.route('/api/register/<int:address>/<int:value>', methods=['POST'])
        def api_set_register(address, value):
            """寫入單一寄存器"""
            try:
                if self.write_register_direct(address, value):
                    return jsonify({'success': True, 'address': address, 'value': value})
                else:
                    return jsonify({'success': False, 'error': 'Invalid address or value'}), 400
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
    
    def initialize_test_data(self):
        """初始化測試數據 - 針對已知模組配置"""
        try:
            # 依據專案文檔設定模組基地址的測試數據
            test_modules = {
                # CCD視覺模組 (200-299)
                200: 1,    # 握手狀態
                201: 0,    # 指令寄存器
                
                # VP震動盤模組 (300-349)  
                300: 1,    # 模組狀態
                301: 1,    # 連接狀態
                
                # 機械臂模組 (400-449) 預留
                400: 0,    # 預留狀態
                
                # LED控制器 (600-649)
                600: 1,    # 模組狀態
                601: 0,    # 串口狀態
                
                # Gripper夾爪 (700-799)
                700: 0,    # PGC狀態
                720: 0,    # PGHL狀態  
                740: 0,    # PGE狀態
                
                # CCD3角度檢測 (800-899)
                800: 1,    # 模組狀態
                
                # XC100升降模組 (1000-1049)
                1000: 1,   # 模組狀態
                1001: 0,   # 設備連接狀態
            }
            
            for addr, value in test_modules.items():
                self.write_register_direct(addr, value)
            
            logging.info(f"測試數據初始化完成 - 配置了{len(test_modules)}個模組基地址")
            
        except Exception as e:
            logging.error(f"初始化測試數據失敗: {e}")
    
    def start_modbus_server(self):
        """啟動高性能Modbus TCP伺服器"""
        try:
            # 創建設備識別
            identity = ModbusDeviceIdentification()
            identity.VendorName = 'High Performance Modbus Server'
            identity.ProductCode = 'HPMS'
            identity.ProductName = 'Optimized Modbus TCP Server'
            identity.ModelName = 'v2.0.0'
            identity.MajorMinorRevision = '2.0.0'
            
            # 創建上下文
            context = self.create_modbus_context()
            if context is None:
                raise Exception("無法創建Modbus上下文")
            
            logging.info(f"啟動高性能Modbus TCP Server: {self.server_host}:{self.server_port}")
            logging.info(f"SlaveID: {self.slave_id}, 寄存器數量: {self.register_count}")
            logging.info("針對10個模組併發連接優化")
            
            self.server_running = True
            self.start_time = time.time()
            
            # 啟動伺服器 (阻塞調用)
            StartTcpServer(
                context=context,
                identity=identity,
                address=(self.server_host, self.server_port),
            )
            
        except Exception as e:
            logging.error(f"Modbus伺服器啟動失敗: {e}")
            self.server_running = False
    
    def start_web_server(self):
        """啟動精簡Web介面"""
        try:
            logging.info(f"啟動精簡Web管理介面: http://127.0.0.1:{self.web_port}")
            self.flask_app.run(
                host='127.0.0.1',  # 只綁定本地，減少安全風險
                port=self.web_port,
                debug=False,
                use_reloader=False,
                threaded=True
            )
        except Exception as e:
            logging.error(f"Web伺服器啟動失敗: {e}")
    
    def shutdown(self):
        """優雅關閉伺服器"""
        try:
            logging.info("正在關閉高性能伺服器...")
            self.shutdown_event.set()
            self.server_running = False
            
            # 輸出最終統計
            stats = self.get_server_stats()
            logging.info(f"伺服器統計: {stats}")
            
        except Exception as e:
            logging.error(f"關閉伺服器錯誤: {e}")
    
    def run(self):
        """主運行方法"""
        try:
            logging.info("=== 高性能Modbus TCP Server 啟動 ===")
            logging.info(f"目標: 10個模組併發連接")
            logging.info(f"寄存器數量: {self.register_count}")
            logging.info(f"優化重點: 響應速度、穩定性")
            
            # 初始化測試數據
            self.initialize_test_data()
            
            # 啟動精簡Web伺服器 (背景執行)
            web_thread = threading.Thread(target=self.start_web_server, daemon=True, name="MinimalWebServer")
            web_thread.start()
            
            # 等待Web伺服器啟動
            time.sleep(0.5)
            
            # 啟動Modbus伺服器 (主線程)
            self.start_modbus_server()
            
        except KeyboardInterrupt:
            logging.info("收到中斷信號")
        except Exception as e:
            logging.error(f"伺服器運行錯誤: {e}\n{traceback.format_exc()}")
        finally:
            self.shutdown()

def main():
    """主函數"""
    try:
        setup_logging()
        
        print("=" * 70)
        print("  高性能Modbus TCP Server - 針對10個模組併發優化")
        print("=" * 70)
        print(f"  版本: 2.0.0-optimized")
        print(f"  寄存器數量: 3000")
        print(f"  Modbus TCP 埠: 502")
        print(f"  精簡Web介面: 8000")
        print(f"  設計目標: 10個模組併發連接")
        print(f"  優化重點: 響應速度、穩定性、記憶體效率")
        print("=" * 70)
        print("  按 Ctrl+C 可安全關閉伺服器")
        print("=" * 70)
        
        app = ModbusTCPServerOptimized()
        app.run()
        
    except Exception as e:
        logging.error(f"啟動失敗: {e}")
        print(f"\n啟動失敗: {e}")
        input("按 Enter 鍵退出...")
        sys.exit(1)

if __name__ == "__main__":
    main()