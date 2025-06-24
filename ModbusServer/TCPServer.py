# modbus_tcp_server_production.py
# 生產環境版本 - 支援分離檔案版本 (templates + static)
# 更新：支援無符號 0-65535 範圍，加強錯誤處理和定時日誌記錄

import logging
import threading
import time
import json
import os
import sys
import signal
import traceback
from datetime import datetime
from pathlib import Path
from flask import Flask, render_template, request, jsonify, send_from_directory
from pymodbus.server import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext

# 獲取可執行檔的基礎路徑
def get_base_path():
    """獲取程式基礎路徑，支援 PyInstaller 打包"""
    if getattr(sys, 'frozen', False):
        # PyInstaller 打包後的路徑
        return sys._MEIPASS
    else:
        # 開發環境路徑
        return os.path.dirname(os.path.abspath(__file__))

# 設定日誌 - 生產環境配置（僅錯誤日誌）
def setup_logging():
    """設定生產環境日誌配置 - 僅記錄錯誤"""
    log_formatter = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
    )
    
    # 檔案日誌處理器 - 只記錄ERROR
    file_handler = logging.FileHandler('modbus_server_error.log', encoding='utf-8')
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(logging.ERROR)
    
    # 控制台日誌處理器 - 只顯示重要資訊
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    console_handler.setLevel(logging.WARNING)
    
    # 設定根記錄器
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(file_handler)
    root_logger.addHandler(console_handler)
    
    # 抑制第三方庫的日誌
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    logging.getLogger('pymodbus').setLevel(logging.ERROR)

class TimedLogger:
    """定時日誌管理器 - 每分鐘記錄狀態，一小時後清空"""
    
    def __init__(self, server_app, log_file='modbus_status.log'):
        self.server_app = server_app
        self.log_file = log_file
        self.log_start_time = time.time()
        self.is_running = False
        self.log_thread = None
        
    def start(self):
        """啟動定時日誌"""
        self.is_running = True
        self.log_thread = threading.Thread(target=self._log_worker, daemon=True, name="TimedLogger")
        self.log_thread.start()
        print(f"✓ 定時日誌啟動: 每分鐘記錄到 {self.log_file}")
        
    def stop(self):
        """停止定時日誌"""
        self.is_running = False
        if self.log_thread and self.log_thread.is_alive():
            self.log_thread.join(timeout=1)
        print("✓ 定時日誌已停止")
    
    def _log_worker(self):
        """日誌工作線程"""
        while self.is_running:
            try:
                # 檢查是否需要清空日誌（1小時 = 3600秒）
                current_time = time.time()
                if current_time - self.log_start_time >= 3600:
                    self._clear_log()
                    self.log_start_time = current_time
                
                # 記錄系統狀態
                self._write_status_log()
                
                # 等待60秒（1分鐘）
                for _ in range(60):
                    if not self.is_running:
                        break
                    time.sleep(1)
                    
            except Exception as e:
                logging.error(f"定時日誌錯誤: {e}")
                time.sleep(60)  # 發生錯誤時等待1分鐘再重試
    
    def _clear_log(self):
        """清空日誌檔案"""
        try:
            if os.path.exists(self.log_file):
                with open(self.log_file, 'w', encoding='utf-8') as f:
                    f.write(f"=== 日誌檔案已清空 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
                print(f"✓ 日誌檔案已清空: {self.log_file}")
        except Exception as e:
            logging.error(f"清空日誌失敗: {e}")
    
    def _write_status_log(self):
        """寫入系統狀態日誌"""
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            status = self.server_app.get_register_status()
            
            # 計算額外統計資訊
            non_zero_regs = status.get('non_zero_registers', {})
            changed_registers = len(non_zero_regs)
            
            # 取得前10個非零暫存器作為範例
            sample_regs = dict(list(non_zero_regs.items())[:10]) if non_zero_regs else {}
            
            log_entry = {
                'timestamp': timestamp,
                'server_running': status.get('server_running', False),
                'slave_id': status.get('slave_id', 0),
                'total_registers': status.get('total_registers', 0),
                'changed_registers_count': changed_registers,
                'uptime_seconds': int(status.get('uptime', 0)),
                'sample_registers': sample_regs,
                'memory_usage_mb': self._get_memory_usage()
            }
            
            # 寫入日誌檔案
            with open(self.log_file, 'a', encoding='utf-8') as f:
                f.write(f"{json.dumps(log_entry, ensure_ascii=False, separators=(',', ':'))}\n")
                
        except Exception as e:
            logging.error(f"寫入狀態日誌失敗: {e}")
    
    def _get_memory_usage(self):
        """獲取記憶體使用量（MB）"""
        try:
            import psutil
            process = psutil.Process()
            return round(process.memory_info().rss / 1024 / 1024, 1)
        except:
            return 0.0

class SynchronizedDataBlock(ModbusSequentialDataBlock):
    """同步化的數據塊，當外部修改時會更新主程式的暫存器陣列"""
    
    def __init__(self, address, values, server_app):
        super().__init__(address, values)
        self.server_app = server_app
    
    def setValues(self, address, values):
        """覆寫setValues方法，同步更新主程式的陣列"""
        try:
            result = super().setValues(address, values)
            
            # 同步更新主程式的暫存器陣列
            if self.server_app:
                for i, value in enumerate(values):
                    reg_addr = address + i
                    if 0 <= reg_addr < len(self.server_app.registers):
                        self.server_app.registers[reg_addr] = value
            
            return result
        except Exception as e:
            logging.error(f"同步數據塊設值錯誤: {e}")
            return False

class ModbusTCPServerApp:
    def __init__(self):
        self.base_path = get_base_path()
        self.slave_id = 1  # 預設SlaveID
        self.server_host = "0.0.0.0"
        self.server_port = 502
        self.web_port = 8000
        
        # 初始化暫存器數據 (0-999, 共1000個暫存器)
        self.register_count = 3000
        self.registers = [0] * self.register_count
        
        # 暫存器註解
        self.register_comments = {}
        self.load_comments()
        
        # 創建templates和static目錄
        self.create_directories()
        
        # Modbus相關
        self.server = None
        self.context = None
        self.slave_context = None
        self.modbus_thread = None
        
        # Web應用 - 設定模板和靜態檔案路徑
        template_folder = os.path.join(self.base_path, 'templates')
        static_folder = os.path.join(self.base_path, 'static')
        
        self.flask_app = Flask(__name__, 
                              template_folder=template_folder,
                              static_folder=static_folder)
        self.setup_web_routes()
        
        # 伺服器狀態
        self.server_running = False
        self.shutdown_event = threading.Event()
        
        # 定時日誌器
        self.timed_logger = TimedLogger(self)
        
        # 註冊信號處理器
        self.setup_signal_handlers()
        
        print("✓ Modbus TCP Server 應用程式初始化完成")
    
    def setup_signal_handlers(self):
        """設定信號處理器以優雅關閉"""
        def signal_handler(signum, frame):
            print(f"\n收到信號 {signum}，正在優雅關閉伺服器...")
            self.shutdown()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    
    def create_directories(self):
        """創建必要的目錄結構"""
        directories = [
            os.path.join(self.base_path, 'templates'),
            os.path.join(self.base_path, 'static')
        ]
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
    
    def load_comments(self):
        """載入暫存器註解"""
        try:
            comments_file = 'register_comments.json'
            if os.path.exists(comments_file):
                with open(comments_file, 'r', encoding='utf-8') as f:
                    self.register_comments = json.load(f)
        except Exception as e:
            logging.error(f"載入註解失敗: {e}")
            self.register_comments = {}
    
    def save_comments(self):
        """保存暫存器註解"""
        try:
            with open('register_comments.json', 'w', encoding='utf-8') as f:
                json.dump(self.register_comments, f, ensure_ascii=False, indent=2)
        except Exception as e:
            logging.error(f"保存註解失敗: {e}")
    
    def create_modbus_context(self):
        """創建Modbus數據上下文"""
        try:
            # 使用同步化的數據塊
            holding_registers = SynchronizedDataBlock(0, self.registers, self)
            
            # 創建Slave上下文
            self.slave_context = ModbusSlaveContext(
                di=ModbusSequentialDataBlock(0, [0]*100),  # Discrete Inputs
                co=ModbusSequentialDataBlock(0, [0]*100),  # Coils
                hr=holding_registers,                       # Holding Registers (主要使用)
                ir=holding_registers,                       # Input Registers (共用同樣的數據)
            )
            
            # 創建伺服器上下文，包含多個slave
            slaves = {self.slave_id: self.slave_context}
            self.context = ModbusServerContext(slaves=slaves, single=False)
            
            return self.context
        except Exception as e:
            logging.error(f"創建Modbus上下文失敗: {e}")
            return None
    
    def update_slave_id(self, new_slave_id):
        """更新SlaveID"""
        try:
            if 1 <= new_slave_id <= 247:  # Modbus標準SlaveID範圍
                old_slave_id = self.slave_id
                self.slave_id = new_slave_id
                
                # 如果伺服器正在運行，需要重新創建上下文
                if self.server_running and self.context:
                    # 移除舊的slave context
                    if old_slave_id in self.context:
                        del self.context[old_slave_id]
                    
                    # 添加新的slave context
                    self.context[self.slave_id] = self.slave_context
                
                return True
            else:
                logging.error(f"無效的SlaveID: {new_slave_id}, 必須在1-247範圍內")
                return False
        except Exception as e:
            logging.error(f"更新SlaveID失敗: {e}")
            return False
    
    def read_register(self, address):
        """讀取暫存器值"""
        try:
            if 0 <= address < self.register_count:
                # 從Modbus上下文讀取最新值
                if self.slave_context:
                    try:
                        result = self.slave_context.getValues(3, address, 1)
                        if result:
                            value = result[0]
                            self.registers[address] = value  # 同步到內部陣列
                            return value
                    except Exception:
                        pass
                
                # 如果Modbus上下文不可用，返回內部陣列的值
                value = self.registers[address]
                return value
            else:
                logging.error(f"暫存器地址超出範圍: {address}")
                return None
        except Exception as e:
            logging.error(f"讀取暫存器失敗: {e}")
            return None
    
    def write_register(self, address, value):
        """寫入暫存器值 - 支援無符號 0-65535 範圍"""
        try:
            if 0 <= address < self.register_count:
                # 確保值在無符號16位範圍內 (0 to 65535)
                if 0 <= value <= 65535:
                    self.registers[address] = value
                    
                    # 同步更新到Modbus上下文
                    if self.slave_context:
                        self.slave_context.setValues(3, address, [value])  # Function Code 3 (Holding Registers)
                        self.slave_context.setValues(4, address, [value])  # Function Code 4 (Input Registers)
                    
                    return True
                else:
                    logging.error(f"暫存器值超出無符號16位範圍: {value} (需要 0-65535)")
                    return False
            else:
                logging.error(f"暫存器地址超出範圍: {address}")
                return False
        except Exception as e:
            logging.error(f"寫入暫存器失敗: {e}")
            return False
    
    def write_multiple_registers(self, start_address, values):
        """批量寫入暫存器"""
        try:
            if start_address + len(values) <= self.register_count:
                success_count = 0
                for i, value in enumerate(values):
                    if self.write_register(start_address + i, value):
                        success_count += 1
                
                if success_count == len(values):
                    return True
                else:
                    logging.warning(f"批量寫入部分成功: {success_count}/{len(values)}")
                    return False
            else:
                logging.error(f"批量寫入超出範圍: start={start_address}, count={len(values)}")
                return False
        except Exception as e:
            logging.error(f"批量寫入失敗: {e}")
            return False
    
    def get_register_status(self):
        """獲取暫存器狀態摘要"""
        try:
            # 同步部分暫存器值（避免效能問題）
            if self.slave_context:
                try:
                    for addr in range(min(100, self.register_count)):  # 只同步前100個
                        result = self.slave_context.getValues(3, addr, 1)
                        if result:
                            self.registers[addr] = result[0]
                except Exception:
                    pass
            
            non_zero_registers = {addr: val for addr, val in enumerate(self.registers) if val != 0}
            return {
                'total_registers': self.register_count,
                'non_zero_count': len(non_zero_registers),
                'non_zero_registers': non_zero_registers,
                'slave_id': self.slave_id,
                'server_running': self.server_running,
                'version': '1.0.0',
                'uptime': time.time() - getattr(self, 'start_time', time.time())
            }
        except Exception as e:
            logging.error(f"獲取暫存器狀態失敗: {e}")
            return {
                'total_registers': self.register_count,
                'non_zero_count': 0,
                'non_zero_registers': {},
                'slave_id': self.slave_id,
                'server_running': self.server_running,
                'error': str(e)
            }
    
    def get_register_range(self, start_address, count):
        """獲取指定範圍的暫存器數據"""
        try:
            if start_address < 0 or start_address >= self.register_count:
                return None
            
            end_address = min(start_address + count, self.register_count)
            registers_data = []
            
            for addr in range(start_address, end_address):
                value = self.read_register(addr)  # 使用read_register確保數據同步
                comment = self.register_comments.get(str(addr), '')
                registers_data.append({
                    'address': addr,
                    'value': value if value is not None else 0,
                    'comment': comment
                })
            
            return registers_data
        except Exception as e:
            logging.error(f"獲取暫存器範圍失敗: {e}")
            return None
    
    def update_register_comment(self, address, comment):
        """更新暫存器註解"""
        try:
            if 0 <= address < self.register_count:
                if comment.strip():
                    self.register_comments[str(address)] = comment.strip()
                else:
                    # 如果註解為空，則刪除
                    self.register_comments.pop(str(address), None)
                
                self.save_comments()
                return True
            return False
        except Exception as e:
            logging.error(f"更新註解失敗: {e}")
            return False
    
    def setup_web_routes(self):
        """設定Web介面路由"""
        
        @self.flask_app.errorhandler(Exception)
        def handle_exception(e):
            logging.error(f"Web介面錯誤: {e}\n{traceback.format_exc()}")
            return jsonify({'error': 'Internal server error'}), 500
        
        @self.flask_app.route('/')
        def index():
            return render_template('index.html')
        
        @self.flask_app.route('/static/<path:filename>')
        def static_files(filename):
            return send_from_directory(self.flask_app.static_folder, filename)
        
        @self.flask_app.route('/api/status')
        def api_status():
            return jsonify(self.get_register_status())
        
        @self.flask_app.route('/api/slave_id', methods=['POST'])
        def api_set_slave_id():
            try:
                data = request.get_json()
                new_slave_id = data.get('slave_id')
                if self.update_slave_id(new_slave_id):
                    return jsonify({'success': True, 'slave_id': self.slave_id})
                else:
                    return jsonify({'success': False, 'error': 'Invalid SlaveID'}), 400
            except Exception as e:
                logging.error(f"API設定SlaveID失敗: {e}")
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @self.flask_app.route('/api/register/<int:address>')
        def api_get_register(address):
            try:
                value = self.read_register(address)
                if value is not None:
                    comment = self.register_comments.get(str(address), '')
                    return jsonify({'address': address, 'value': value, 'comment': comment})
                else:
                    return jsonify({'error': 'Invalid address'}), 400
            except Exception as e:
                logging.error(f"API讀取暫存器失敗: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.flask_app.route('/api/register/<int:address>', methods=['POST'])
        def api_set_register(address):
            try:
                data = request.get_json()
                value = data.get('value')
                if self.write_register(address, value):
                    return jsonify({'success': True, 'address': address, 'value': value})
                else:
                    return jsonify({'success': False, 'error': 'Invalid address or value (0-65535)'}), 400
            except Exception as e:
                logging.error(f"API寫入暫存器失敗: {e}")
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @self.flask_app.route('/api/registers', methods=['POST'])
        def api_set_multiple_registers():
            try:
                data = request.get_json()
                start_address = data.get('start_address', 0)
                values = data.get('values', [])
                if self.write_multiple_registers(start_address, values):
                    return jsonify({'success': True})
                else:
                    return jsonify({'success': False, 'error': 'Invalid range or values'}), 400
            except Exception as e:
                logging.error(f"API批量寫入失敗: {e}")
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @self.flask_app.route('/api/register_range')
        def api_get_register_range():
            try:
                start_address = int(request.args.get('start', 0))
                count = int(request.args.get('count', 20))
                count = min(count, 100)  # 限制最大數量
                
                registers_data = self.get_register_range(start_address, count)
                if registers_data is not None:
                    return jsonify({
                        'success': True,
                        'start_address': start_address,
                        'count': len(registers_data),
                        'registers': registers_data
                    })
                else:
                    return jsonify({'success': False, 'error': 'Invalid address range'}), 400
            except Exception as e:
                logging.error(f"API獲取暫存器範圍失敗: {e}")
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @self.flask_app.route('/api/comment/<int:address>', methods=['POST'])
        def api_update_comment(address):
            try:
                data = request.get_json()
                comment = data.get('comment', '')
                if self.update_register_comment(address, comment):
                    return jsonify({'success': True, 'address': address, 'comment': comment})
                else:
                    return jsonify({'success': False, 'error': 'Invalid address'}), 400
            except Exception as e:
                logging.error(f"API更新註解失敗: {e}")
                return jsonify({'success': False, 'error': str(e)}), 500
    
    def start_modbus_server(self):
        """啟動Modbus TCP伺服器"""
        try:
            # 創建設備識別
            identity = ModbusDeviceIdentification()
            identity.VendorName = 'Python Modbus Server'
            identity.ProductCode = 'PMS'
            identity.VendorUrl = 'https://github.com/your-repo'
            identity.ProductName = 'Python Modbus TCP Server'
            identity.ModelName = 'Production Server'
            identity.MajorMinorRevision = '1.0.0'
            
            # 創建上下文
            context = self.create_modbus_context()
            if context is None:
                raise Exception("無法創建Modbus上下文")
            
            # 啟動伺服器 (這會阻塞當前線程)
            print(f"✓ Modbus TCP Server 啟動於 {self.server_host}:{self.server_port}, SlaveID: {self.slave_id}")
            self.server_running = True
            self.start_time = time.time()
            
            StartTcpServer(
                context=context,
                identity=identity,
                address=(self.server_host, self.server_port),
            )
            
        except Exception as e:
            logging.error(f"Modbus伺服器啟動失敗: {e}\n{traceback.format_exc()}")
            self.server_running = False
    
    def start_web_server(self):
        """啟動Web管理介面"""
        try:
            print(f"✓ Web管理介面啟動於 http://127.0.0.1:{self.web_port}")
            self.flask_app.run(
                host='0.0.0.0',
                port=self.web_port,
                debug=False,
                use_reloader=False,
                threaded=True
            )
        except Exception as e:
            logging.error(f"Web伺服器啟動失敗: {e}\n{traceback.format_exc()}")
    
    def initialize_test_data(self):
        """初始化測試數據 - 使用無符號範圍"""
        try:
            # 設定一些測試值 (無符號 0-65535)
            test_data = {
                0: 100,     # 地址0 = 100
                1: 200,     # 地址1 = 200
                10: 1000,   # 地址10 = 1000
                50: 5000,   # 地址50 = 5000
                100: 12345, # 地址100 = 12345
                200: 32768, # 地址200 = 32768 (超過有符號範圍但在無符號範圍內)
                500: 65535, # 地址500 = 65535 (最大無符號值)
                999: 40000  # 地址999 = 40000
            }
            
            for addr, value in test_data.items():
                self.write_register(addr, value)
            
            # 設定一些測試註解
            test_comments = {
                0: "溫度感測器",
                1: "濕度感測器", 
                10: "馬達轉速",
                50: "壓力數值",
                100: "測試數據",
                200: "高數值測試",
                500: "最大值測試",
                999: "邊界測試"
            }
            
            for addr, comment in test_comments.items():
                self.register_comments[str(addr)] = comment
            
            self.save_comments()
            print("✓ 測試數據和註解初始化完成")
        except Exception as e:
            logging.error(f"初始化測試數據失敗: {e}")
    
    def shutdown(self):
        """優雅關閉伺服器"""
        try:
            print("正在關閉伺服器...")
            self.shutdown_event.set()
            self.server_running = False
            
            # 停止定時日誌
            self.timed_logger.stop()
            
            # 保存註解
            self.save_comments()
            
            print("✓ 伺服器已關閉")
        except Exception as e:
            logging.error(f"關閉伺服器時發生錯誤: {e}")
    
    def run(self):
        """主運行方法"""
        try:
            print("=== Modbus TCP Server 啟動 (生產環境 + 定時日誌) ===")
            print(f"Python 版本: {sys.version}")
            print(f"基礎路徑: {self.base_path}")
            
            # 初始化測試數據
            self.initialize_test_data()
            
            # 啟動定時日誌
            self.timed_logger.start()
            
            # 啟動Web伺服器 (在單獨線程中)
            web_thread = threading.Thread(target=self.start_web_server, daemon=True, name="WebServer")
            web_thread.start()
            
            # 等待一秒確保Web伺服器啟動
            time.sleep(1)
            
            # 啟動Modbus伺服器 (主線程，會阻塞)
            self.start_modbus_server()
            
        except KeyboardInterrupt:
            print("收到中斷信號，正在關閉...")
        except Exception as e:
            logging.error(f"伺服器運行錯誤: {e}\n{traceback.format_exc()}")
        finally:
            self.shutdown()

def main():
    """主函數 - 生產環境入口點"""
    try:
        # 設定日誌
        setup_logging()
        
        # 打印啟動資訊
        print("=" * 60)
        print("  Modbus TCP Server - 生產環境版本 + 定時日誌")
        print("=" * 60)
        print(f"  版本: 1.0.0")
        print(f"  數值範圍: 0-65535 (無符號 16 位)")
        print(f"  Modbus TCP 埠: 502")
        print(f"  Web 管理埠: 8000")
        print(f"  Web 管理網址: http://localhost:8000")
        print(f"  狀態日誌: 每分鐘記錄，1小時清空")
        print("=" * 60)
        print("  按 Ctrl+C 可安全關閉伺服器")
        print("=" * 60)
        
        # 創建並運行伺服器
        app = ModbusTCPServerApp()
        app.run()
        
    except Exception as e:
        logging.error(f"應用程式啟動失敗: {e}\n{traceback.format_exc()}")
        print(f"\n❌ 啟動失敗: {e}")
        print("請檢查 modbus_server_error.log 檔案獲取詳細錯誤資訊")
        input("按 Enter 鍵退出...")
        sys.exit(1)

if __name__ == "__main__":
    main()