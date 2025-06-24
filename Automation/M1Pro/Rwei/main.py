import sys
import time
from datetime import datetime
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# 簡化的機械臂控制器（用於測試）
class SimpleRobotController(QObject):
    log_update = pyqtSignal(str)
    error_update = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)
    enable_changed = pyqtSignal(bool)
    feedback_update = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.is_connected = False
        self.is_enabled = False
        self.client_dash = None
        
    def connect_robot(self, ip, dash_port, move_port, feed_port):
        try:
            # 這裡導入實際的robot控制邏輯
            from dobot_api import DobotApiDashboard
            self.client_dash = DobotApiDashboard(ip, dash_port)
            
            # 測試連接
            result = self.client_dash.RobotMode()
            self.log_update.emit(f"連接成功，機械臂狀態: {result}")
            
            self.is_connected = True
            self.connection_changed.emit(True)
            return True
        except Exception as e:
            self.error_update.emit(f"連接失敗: {str(e)}")
            self.is_connected = False
            self.connection_changed.emit(False)
            return False
    
    def disconnect_robot(self):
        try:
            if self.client_dash:
                self.client_dash.close()
            self.is_connected = False
            self.is_enabled = False
            self.connection_changed.emit(False)
            self.enable_changed.emit(False)
            self.log_update.emit("機械臂已斷開連接")
            return True
        except Exception as e:
            self.error_update.emit(f"斷開連接失敗: {str(e)}")
            return False
    
    def toggle_enable(self):
        if not self.is_connected:
            self.error_update.emit("機械臂未連接")
            return False
        
        try:
            if self.is_enabled:
                result = self.client_dash.DisableRobot()
                self.is_enabled = False
                self.log_update.emit(f"機械臂下使能: {result}")
            else:
                result = self.client_dash.EnableRobot()
                self.is_enabled = True
                self.log_update.emit(f"機械臂使能: {result}")
            
            self.enable_changed.emit(self.is_enabled)
            return True
        except Exception as e:
            self.error_update.emit(f"使能切換失敗: {str(e)}")
            return False
    
    def set_do_execute(self, index, status):
        """立即執行DO控制"""
        if not self.is_connected:
            self.error_update.emit("機械臂未連接")
            return False
        
        if not self.is_enabled:
            self.error_update.emit("機械臂未使能")
            return False
        
        try:
            start_time = time.time()
            
            # 嘗試DOExecute方法
            if hasattr(self.client_dash, 'DOExecute'):
                result = self.client_dash.DOExecute(index, status)
                end_time = time.time()
                delay = (end_time - start_time) * 1000
                
                self.log_update.emit(f"DOExecute({index},{status}) 回應: {result} (延遲: {delay:.1f}ms)")
                
                if result and str(result).strip():
                    if "0," in str(result):
                        self.log_update.emit(f"✓ DO{index} 立即執行成功")
                        return True
                    elif "-1," in str(result):
                        self.error_update.emit(f"✗ DO{index} 立即執行被拒絕 (錯誤代碼:-1)")
                        # 嘗試隊列指令
                        return self.set_do_queue(index, status)
                    else:
                        self.error_update.emit(f"✗ DO{index} 未知回應: {result}")
                        return False
                else:
                    self.error_update.emit(f"✗ DO{index} 無回應")
                    return False
            else:
                self.error_update.emit("DOExecute方法不存在")
                return self.set_do_queue(index, status)
                
        except Exception as e:
            self.error_update.emit(f"DO{index} 執行異常: {str(e)}")
            return False
    
    def set_do_queue(self, index, status):
        """隊列DO控制"""
        try:
            result = self.client_dash.DO(index, status)
            self.log_update.emit(f"DO({index},{status}) 隊列回應: {result}")
            
            if result and "0," in str(result):
                # 嘗試執行隊列
                try:
                    continue_result = self.client_dash.Continue()
                    self.log_update.emit(f"Continue() 回應: {continue_result}")
                except:
                    pass
                
                self.log_update.emit(f"✓ DO{index} 隊列執行成功")
                return True
            else:
                self.error_update.emit(f"✗ DO{index} 隊列執行失敗: {result}")
                return False
                
        except Exception as e:
            self.error_update.emit(f"DO{index} 隊列異常: {str(e)}")
            return False
    
    def get_robot_status(self):
        """獲取機械臂狀態"""
        if not self.is_connected:
            return "未連接"
        
        try:
            result = self.client_dash.RobotMode()
            return result
        except Exception as e:
            self.error_update.emit(f"獲取狀態失敗: {str(e)}")
            return "獲取失敗"
    
    def get_error_status(self):
        """獲取錯誤狀態"""
        if not self.is_connected:
            return "未連接"
        
        try:
            result = self.client_dash.GetErrorID()
            return result
        except Exception as e:
            self.error_update.emit(f"獲取錯誤狀態失敗: {str(e)}")
            return "獲取失敗"

class CircularDOButton(QPushButton):
    """圓形DO控制按鈕"""
    
    def __init__(self, do_index, parent=None):
        super().__init__(parent)
        self.do_index = do_index
        self.is_on = False
        self.setFixedSize(80, 80)
        self.setText(f"DO{do_index}")
        self.setFont(QFont("Arial", 10, QFont.Bold))
        self.update_style()
        
    def update_style(self):
        """更新按鈕樣式"""
        if self.is_on:
            # ON狀態 - 綠色發光效果
            style = """
                QPushButton {
                    background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4,
                        radius: 1.35, stop: 0 #00ff00, stop: 1 #00aa00);
                    border: 4px solid #00ff00;
                    border-radius: 40px;
                    color: white;
                    font-weight: bold;
                    text-align: center;
                    box-shadow: 0 0 20px #00ff00;
                }
                QPushButton:hover {
                    background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4,
                        radius: 1.35, stop: 0 #22ff22, stop: 1 #00cc00);
                    box-shadow: 0 0 25px #00ff00;
                }
                QPushButton:pressed {
                    background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4,
                        radius: 1.35, stop: 0 #00dd00, stop: 1 #008800);
                }
                QPushButton:disabled {
                    background-color: #666666;
                    border: 3px solid #444444;
                    color: #999999;
                    box-shadow: none;
                }
            """
        else:
            # OFF狀態 - 紅色
            style = """
                QPushButton {
                    background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4,
                        radius: 1.35, stop: 0 #ff0000, stop: 1 #aa0000);
                    border: 4px solid #ff0000;
                    border-radius: 40px;
                    color: white;
                    font-weight: bold;
                    text-align: center;
                    box-shadow: 0 0 20px #ff0000;
                }
                QPushButton:hover {
                    background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4,
                        radius: 1.35, stop: 0 #ff2222, stop: 1 #cc0000);
                    box-shadow: 0 0 25px #ff0000;
                }
                QPushButton:pressed {
                    background: qradialgradient(cx: 0.3, cy: -0.4, fx: 0.3, fy: -0.4,
                        radius: 1.35, stop: 0 #dd0000, stop: 1 #880000);
                }
                QPushButton:disabled {
                    background-color: #666666;
                    border: 3px solid #444444;
                    color: #999999;
                    box-shadow: none;
                }
            """
        self.setStyleSheet(style)
    
    def set_state(self, is_on, animate=True):
        """設置DO狀態"""
        if self.is_on != is_on:
            self.is_on = is_on
            self.update_style()
            if animate:
                self.animate_state_change()
    
    def animate_state_change(self):
        """狀態變化動畫"""
        self.animation = QPropertyAnimation(self, b"geometry")
        self.animation.setDuration(300)
        
        current_rect = self.geometry()
        expanded_rect = QRect(current_rect.x() - 8, current_rect.y() - 8, 
                            current_rect.width() + 16, current_rect.height() + 16)
        
        self.animation.setStartValue(current_rect)
        self.animation.setKeyValueAt(0.5, expanded_rect)
        self.animation.setEndValue(current_rect)
        self.animation.start()

class DOTestMainWindow(QMainWindow):
    """DO測試主窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DO控制診斷測試工具 v2.0")
        self.setFixedSize(1200, 800)
        
        # 機械臂控制器
        self.robot_controller = SimpleRobotController()
        self.connect_signals()
        
        # 控制狀態
        self.is_connected = False
        self.is_enabled = False
        
        # DO按鈕字典
        self.do_buttons = {}
        
        # 統計變量
        self.success_count = 0
        self.failure_count = 0
        self.test_count = 0
        self.delay_sum = 0
        
        # 測試狀態
        self.batch_testing = False
        self.sequence_testing = False
        
        self.setupUI()
        self.init_do_buttons()
        
    def connect_signals(self):
        """連接信號槽"""
        self.robot_controller.log_update.connect(self.append_log, Qt.QueuedConnection)
        self.robot_controller.error_update.connect(self.append_error, Qt.QueuedConnection)
        self.robot_controller.connection_changed.connect(self.on_connection_changed, Qt.QueuedConnection)
        self.robot_controller.enable_changed.connect(self.on_enable_changed, Qt.QueuedConnection)
        
    def setupUI(self):
        """建立UI界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 設置漸層背景
        central_widget.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                    stop: 0 #f0f8ff, stop: 1 #e6f3ff);
            }
        """)
        
        # 主布局
        main_layout = QHBoxLayout()
        
        # 左側 - 控制區域
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.create_connection_group())
        left_layout.addWidget(self.create_do_control_group())
        left_layout.addWidget(self.create_batch_control_group())
        
        # 右側 - 信息區域
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.create_status_group())
        right_layout.addWidget(self.create_statistics_group())
        right_layout.addWidget(self.create_log_group())
        right_layout.addWidget(self.create_error_group())
        
        main_layout.addLayout(left_layout, 3)
        main_layout.addLayout(right_layout, 2)
        
        central_widget.setLayout(main_layout)
        
    def create_connection_group(self):
        """建立連接控制群組"""
        group = QGroupBox("🔗 機械臂連接")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #4CAF50;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: rgba(255, 255, 255, 0.9);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        layout = QGridLayout()
        
        # IP輸入
        layout.addWidget(QLabel("IP地址:"), 0, 0)
        self.ip_edit = QLineEdit("192.168.1.6")
        self.ip_edit.setStyleSheet("""
            QLineEdit {
                border: 2px solid #ddd;
                border-radius: 5px;
                padding: 5px;
                font-size: 12pt;
            }
            QLineEdit:focus {
                border: 2px solid #4CAF50;
            }
        """)
        layout.addWidget(self.ip_edit, 0, 1)
        
        # 連接按鈕
        self.connect_btn = QPushButton("🔌 連接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 10px 20px;
                font-size: 12pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        layout.addWidget(self.connect_btn, 0, 2)
        
        # 使能按鈕
        self.enable_btn = QPushButton("⚡ 使能")
        self.enable_btn.clicked.connect(self.toggle_enable)
        self.enable_btn.setEnabled(False)
        self.enable_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 10px 20px;
                font-size: 12pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        layout.addWidget(self.enable_btn, 0, 3)
        
        # 狀態顯示
        status_layout = QHBoxLayout()
        self.status_indicator = QLabel("●")
        self.status_indicator.setFont(QFont("Arial", 20))
        self.status_indicator.setStyleSheet("color: red;")
        status_layout.addWidget(self.status_indicator)
        
        self.status_label = QLabel("未連接")
        self.status_label.setStyleSheet("font-weight: bold; font-size: 12pt;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        
        layout.addLayout(status_layout, 1, 0, 1, 4)
        
        group.setLayout(layout)
        return group
        
    def create_do_control_group(self):
        """建立DO控制群組"""
        group = QGroupBox("🎛️ DO控制面板")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #FF9800;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 15px;
                background-color: rgba(255, 255, 255, 0.9);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        layout = QVBoxLayout()
        
        # 說明文字
        info_label = QLabel("點擊圓形按鈕切換DO狀態 (綠色=ON, 紅色=OFF)")
        info_label.setStyleSheet("color: #666; font-style: italic; margin: 5px;")
        info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(info_label)
        
        # 第一排 DO1-DO8
        row1_widget = QWidget()
        row1_layout = QHBoxLayout(row1_widget)
        row1_layout.addStretch()
        
        for i in range(1, 9):
            do_btn = CircularDOButton(i)
            do_btn.clicked.connect(lambda checked, idx=i: self.toggle_do(idx))
            do_btn.setEnabled(False)
            self.do_buttons[i] = do_btn
            row1_layout.addWidget(do_btn)
            if i < 8:
                row1_layout.addSpacing(15)
        
        row1_layout.addStretch()
        layout.addWidget(row1_widget)
        
        layout.addSpacing(20)
        
        # 第二排 DO9-DO16
        row2_widget = QWidget()
        row2_layout = QHBoxLayout(row2_widget)
        row2_layout.addStretch()
        
        for i in range(9, 17):
            do_btn = CircularDOButton(i)
            do_btn.clicked.connect(lambda checked, idx=i: self.toggle_do(idx))
            do_btn.setEnabled(False)
            self.do_buttons[i] = do_btn
            row2_layout.addWidget(do_btn)
            if i < 16:
                row2_layout.addSpacing(15)
        
        row2_layout.addStretch()
        layout.addWidget(row2_widget)
        
        group.setLayout(layout)
        return group
        
    def create_batch_control_group(self):
        """建立批量控制群組"""
        group = QGroupBox("🔧 批量操作與診斷")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #9C27B0;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: rgba(255, 255, 255, 0.9);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        layout = QGridLayout()
        
        # 批量操作
        self.all_on_btn = QPushButton("🟢 全部ON")
        self.all_on_btn.clicked.connect(self.set_all_on)
        self.all_on_btn.setEnabled(False)
        layout.addWidget(self.all_on_btn, 0, 0)
        
        self.all_off_btn = QPushButton("🔴 全部OFF")
        self.all_off_btn.clicked.connect(self.set_all_off)
        self.all_off_btn.setEnabled(False)
        layout.addWidget(self.all_off_btn, 0, 1)
        
        self.sequence_btn = QPushButton("🌊 流水燈測試")
        self.sequence_btn.clicked.connect(self.start_sequence_test)
        self.sequence_btn.setEnabled(False)
        layout.addWidget(self.sequence_btn, 0, 2)
        
        self.cycle_btn = QPushButton("🔄 循環測試")
        self.cycle_btn.clicked.connect(self.start_cycle_test)
        self.cycle_btn.setEnabled(False)
        layout.addWidget(self.cycle_btn, 0, 3)
        
        # 診斷功能
        self.diagnose_btn = QPushButton("🔍 DO診斷")
        self.diagnose_btn.clicked.connect(self.diagnose_do_control)
        self.diagnose_btn.setEnabled(False)
        layout.addWidget(self.diagnose_btn, 1, 0)
        
        self.status_check_btn = QPushButton("📊 狀態檢查")
        self.status_check_btn.clicked.connect(self.check_robot_status)
        self.status_check_btn.setEnabled(False)
        layout.addWidget(self.status_check_btn, 1, 1)
        
        self.error_check_btn = QPushButton("⚠️ 錯誤檢查")
        self.error_check_btn.clicked.connect(self.check_error_status)
        self.error_check_btn.setEnabled(False)
        layout.addWidget(self.error_check_btn, 1, 2)
        
        self.clear_stats_btn = QPushButton("🗑️ 清除統計")
        self.clear_stats_btn.clicked.connect(self.clear_statistics)
        layout.addWidget(self.clear_stats_btn, 1, 3)
        
        # 按鈕樣式
        button_style = """
            QPushButton {
                background-color: #607D8B;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px 12px;
                font-size: 10pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #546E7A;
            }
            QPushButton:pressed {
                background-color: #455A64;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """
        
        for i in range(layout.count()):
            widget = layout.itemAt(i).widget()
            if isinstance(widget, QPushButton):
                widget.setStyleSheet(button_style)
        
        # 保存控制按鈕引用
        self.control_buttons = [
            self.all_on_btn, self.all_off_btn, self.sequence_btn, self.cycle_btn,
            self.diagnose_btn, self.status_check_btn, self.error_check_btn
        ]
        
        group.setLayout(layout)
        return group
        
    def create_status_group(self):
        """建立狀態群組"""
        group = QGroupBox("📈 實時狀態")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #2196F3;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: rgba(255, 255, 255, 0.9);
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        layout = QGridLayout()
        
        # 機械臂狀態
        layout.addWidget(QLabel("機械臂模式:"), 0, 0)
        self.robot_mode_label = QLabel("未知")
        self.robot_mode_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        layout.addWidget(self.robot_mode_label, 0, 1)
        
        layout.addWidget(QLabel("連接狀態:"), 1, 0)
        self.conn_status_label = QLabel("未連接")
        layout.addWidget(self.conn_status_label, 1, 1)
        
        layout.addWidget(QLabel("使能狀態:"), 2, 0)
        self.enable_status_label = QLabel("未使能")
        layout.addWidget(self.enable_status_label, 2, 1)
        
        # DO統計
        layout.addWidget(QLabel("DO ON數量:"), 0, 2)
        self.do_on_count = QLabel("0")
        self.do_on_count.setStyleSheet("font-weight: bold; color: green; font-size: 14pt;")
        layout.addWidget(self.do_on_count, 0, 3)
        
        layout.addWidget(QLabel("DO OFF數量:"), 1, 2)
        self.do_off_count = QLabel("16")
        self.do_off_count.setStyleSheet("font-weight: bold; color: red; font-size: 14pt;")
        layout.addWidget(self.do_off_count, 1, 3)
        
        group.setLayout(layout)
        return group
        
    def create_statistics_group(self):
        """建立統計群組"""
        group = QGroupBox("📊 測試統計")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #4CAF50;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: rgba(255, 255, 255, 0.9);
            }
        """)
        layout = QGridLayout()
        
        layout.addWidget(QLabel("成功次數:"), 0, 0)
        self.success_label = QLabel("0")
        self.success_label.setStyleSheet("font-weight: bold; color: green; font-size: 12pt;")
        layout.addWidget(self.success_label, 0, 1)
        
        layout.addWidget(QLabel("失敗次數:"), 0, 2)
        self.failure_label = QLabel("0")
        self.failure_label.setStyleSheet("font-weight: bold; color: red; font-size: 12pt;")
        layout.addWidget(self.failure_label, 0, 3)
        
        layout.addWidget(QLabel("成功率:"), 1, 0)
        self.success_rate_label = QLabel("0%")
        self.success_rate_label.setStyleSheet("font-weight: bold; color: blue; font-size: 12pt;")
        layout.addWidget(self.success_rate_label, 1, 1)
        
        layout.addWidget(QLabel("平均延遲:"), 1, 2)
        self.avg_delay_label = QLabel("0ms")
        self.avg_delay_label.setStyleSheet("font-weight: bold; color: orange; font-size: 12pt;")
        layout.addWidget(self.avg_delay_label, 1, 3)
        
        group.setLayout(layout)
        return group
        
    def create_log_group(self):
        """建立日誌群組"""
        group = QGroupBox("📝 系統日誌")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #795548;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: rgba(255, 255, 255, 0.9);
            }
        """)
        layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(120)
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                border: 1px solid #dee2e6;
                border-radius: 5px;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 9pt;
                padding: 5px;
            }
        """)
        layout.addWidget(self.log_text)
        
        # 日誌控制
        log_controls = QHBoxLayout()
        clear_log_btn = QPushButton("清除日誌")
        clear_log_btn.clicked.connect(self.clear_logs)
        log_controls.addWidget(clear_log_btn)
        
        self.auto_scroll_check = QCheckBox("自動滾動")
        self.auto_scroll_check.setChecked(True)
        log_controls.addWidget(self.auto_scroll_check)
        log_controls.addStretch()
        
        layout.addLayout(log_controls)
        group.setLayout(layout)
        return group
        
    def create_error_group(self):
        """建立錯誤群組"""
        group = QGroupBox("⚠️ 錯誤信息")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #f44336;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: rgba(255, 255, 255, 0.9);
            }
        """)
        layout = QVBoxLayout()
        
        self.error_text = QTextEdit()
        self.error_text.setMaximumHeight(80)
        self.error_text.setReadOnly(True)
        self.error_text.setStyleSheet("""
            QTextEdit {
                background-color: #fff5f5;
                border: 1px solid #ffcccb;
                border-radius: 5px;
                color: #d32f2f;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 9pt;
                padding: 5px;
            }
        """)
        layout.addWidget(self.error_text)
        
        clear_error_btn = QPushButton("清除錯誤")
        clear_error_btn.clicked.connect(self.clear_errors)
        layout.addWidget(clear_error_btn)
        
        group.setLayout(layout)
        return group

    def init_do_buttons(self):
        """初始化DO按鈕狀態"""
        for i in range(1, 17):
            self.do_buttons[i].set_state(False, animate=False)

    # ==================== 連接控制 ====================
    
    def toggle_connection(self):
        """切換連接狀態"""
        if self.is_connected:
            self.robot_controller.disconnect_robot()
        else:
            ip = self.ip_edit.text().strip()
            if not ip:
                QMessageBox.warning(self, "警告", "請輸入有效的IP地址")
                return
            
            success = self.robot_controller.connect_robot(ip, 29999, 30003, 30004)
            if not success:
                QMessageBox.critical(self, "連接錯誤", "機械臂連接失敗，請檢查IP地址和網路連接")
    
    def toggle_enable(self):
        """切換使能狀態"""
        self.robot_controller.toggle_enable()
    
    @pyqtSlot(bool)
    def on_connection_changed(self, connected):
        """連接狀態變化處理"""
        self.is_connected = connected
        
        if connected:
            self.connect_btn.setText("🔌 斷開")
            self.status_indicator.setStyleSheet("color: green;")
            self.status_label.setText("已連接")
            self.conn_status_label.setText("已連接")
        else:
            self.connect_btn.setText("🔌 連接")
            self.status_indicator.setStyleSheet("color: red;")
            self.status_label.setText("未連接")
            self.conn_status_label.setText("未連接")
            
        # 啟用/禁用控制按鈕
        self.enable_btn.setEnabled(connected)
        for btn in self.control_buttons:
            btn.setEnabled(connected)
        for do_btn in self.do_buttons.values():
            do_btn.setEnabled(connected)
    
    @pyqtSlot(bool)
    def on_enable_changed(self, enabled):
        """使能狀態變化處理"""
        self.is_enabled = enabled
        
        if enabled:
            self.enable_btn.setText("⚡ 下使能")
            self.status_indicator.setStyleSheet("color: blue;")
            self.status_label.setText("已使能")
            self.enable_status_label.setText("已使能")
        else:
            self.enable_btn.setText("⚡ 使能")
            if self.is_connected:
                self.status_indicator.setStyleSheet("color: green;")
                self.status_label.setText("已連接")
            self.enable_status_label.setText("未使能")

    # ==================== DO控制功能 ====================
    
    def toggle_do(self, do_index):
        """切換DO狀態"""
        try:
            current_state = self.do_buttons[do_index].is_on
            new_state = not current_state
            
            self.append_log(f"🎯 用戶點擊DO{do_index} -> {('ON' if new_state else 'OFF')}")
            
            if not self.is_connected:
                self.append_error("❌ 機械臂未連接")
                return
            
            if not self.is_enabled:
                self.append_error("❌ 機械臂未使能")
                return
            
            start_time = time.time()
            success = self.robot_controller.set_do_execute(do_index, int(new_state))
            end_time = time.time()
            
            delay = (end_time - start_time) * 1000
            self.delay_sum += delay
            self.test_count += 1
            
            if success:
                self.do_buttons[do_index].set_state(new_state)
                self.append_log(f"✅ DO{do_index} 控制成功 (延遲: {delay:.1f}ms)")
                self.success_count += 1
            else:
                self.append_error(f"❌ DO{do_index} 控制失敗")
                self.failure_count += 1
            
            self.update_statistics()
            self.update_do_count()
            
        except Exception as e:
            self.append_error(f"❌ DO{do_index} 控制異常: {str(e)}")
            self.failure_count += 1
            self.update_statistics()
    
    def set_all_on(self):
        """設置所有DO為ON"""
        try:
            self.append_log("🟢 開始設置所有DO為ON...")
            if not self.is_connected or not self.is_enabled:
                self.append_error("❌ 機械臂未連接或未使能")
                return
                
            for i in range(1, 17):
                success = self.robot_controller.set_do_execute(i, 1)
                if success:
                    self.do_buttons[i].set_state(True)
                    self.success_count += 1
                else:
                    self.failure_count += 1
                self.test_count += 1
                time.sleep(0.1)  # 避免指令過快
                
            self.update_statistics()
            self.update_do_count()
            self.append_log("✅ 全部ON操作完成")
            
        except Exception as e:
            self.append_error(f"❌ 全部ON操作異常: {str(e)}")
    
    def set_all_off(self):
        """設置所有DO為OFF"""
        try:
            self.append_log("🔴 開始設置所有DO為OFF...")
            if not self.is_connected or not self.is_enabled:
                self.append_error("❌ 機械臂未連接或未使能")
                return
                
            for i in range(1, 17):
                success = self.robot_controller.set_do_execute(i, 0)
                if success:
                    self.do_buttons[i].set_state(False)
                    self.success_count += 1
                else:
                    self.failure_count += 1
                self.test_count += 1
                time.sleep(0.1)
                
            self.update_statistics()
            self.update_do_count()
            self.append_log("✅ 全部OFF操作完成")
            
        except Exception as e:
            self.append_error(f"❌ 全部OFF操作異常: {str(e)}")
    
    def start_sequence_test(self):
        """開始流水燈測試"""
        if self.sequence_testing:
            self.stop_sequence_test()
            return
            
        self.sequence_testing = True
        self.sequence_btn.setText("⏹️ 停止流水燈")
        self.append_log("🌊 開始流水燈測試...")
        
        # 先全部關閉
        for i in range(1, 17):
            self.robot_controller.set_do_execute(i, 0)
            self.do_buttons[i].set_state(False, animate=False)
        
        self.sequence_timer = QTimer()
        self.sequence_timer.timeout.connect(self.sequence_step)
        self.sequence_index = 0
        self.sequence_timer.start(300)  # 每300ms執行一次
    
    def sequence_step(self):
        """流水燈步驟"""
        if not self.sequence_testing:
            return
            
        # 關閉上一個
        if self.sequence_index > 0:
            prev_index = self.sequence_index
            self.robot_controller.set_do_execute(prev_index, 0)
            self.do_buttons[prev_index].set_state(False)
        
        # 開啟當前
        self.sequence_index += 1
        if self.sequence_index <= 16:
            self.robot_controller.set_do_execute(self.sequence_index, 1)
            self.do_buttons[self.sequence_index].set_state(True)
        else:
            # 關閉最後一個，重新開始
            self.robot_controller.set_do_execute(16, 0)
            self.do_buttons[16].set_state(False)
            self.sequence_index = 0
        
        self.update_do_count()
    
    def stop_sequence_test(self):
        """停止流水燈測試"""
        self.sequence_testing = False
        if hasattr(self, 'sequence_timer'):
            self.sequence_timer.stop()
        self.sequence_btn.setText("🌊 流水燈測試")
        self.append_log("⏹️ 流水燈測試已停止")
    
    def start_cycle_test(self):
        """開始循環測試"""
        if self.batch_testing:
            self.stop_cycle_test()
            return
            
        self.batch_testing = True
        self.cycle_btn.setText("⏹️ 停止循環")
        self.cycle_count = 0
        self.max_cycles = 20
        
        self.append_log(f"🔄 開始循環測試 (共{self.max_cycles}次)")
        
        self.cycle_timer = QTimer()
        self.cycle_timer.timeout.connect(self.cycle_step)
        self.cycle_timer.start(2000)  # 每2秒執行一次
    
    def cycle_step(self):
        """循環測試步驟"""
        if not self.batch_testing or self.cycle_count >= self.max_cycles:
            self.stop_cycle_test()
            return
            
        self.cycle_count += 1
        
        # 交替設置DO1-DO8
        for i in range(1, 9):
            state = (self.cycle_count + i) % 2
            success = self.robot_controller.set_do_execute(i, state)
            if success:
                self.do_buttons[i].set_state(bool(state))
                self.success_count += 1
            else:
                self.failure_count += 1
            self.test_count += 1
        
        self.append_log(f"🔄 循環測試 {self.cycle_count}/{self.max_cycles}")
        self.update_statistics()
        self.update_do_count()
    
    def stop_cycle_test(self):
        """停止循環測試"""
        self.batch_testing = False
        if hasattr(self, 'cycle_timer'):
            self.cycle_timer.stop()
        self.cycle_btn.setText("🔄 循環測試")
        self.append_log("⏹️ 循環測試已停止")

    # ==================== 診斷功能 ====================
    
    def diagnose_do_control(self):
        """診斷DO控制"""
        self.append_log("🔍 === 開始DO控制診斷 ===")
        
        if not self.is_connected:
            self.append_error("❌ 診斷失敗：機械臂未連接")
            return
        
        try:
            # 1. 檢查API方法
            if hasattr(self.robot_controller.client_dash, 'DOExecute'):
                self.append_log("✅ DOExecute方法存在")
            else:
                self.append_error("❌ DOExecute方法不存在")
            
            if hasattr(self.robot_controller.client_dash, 'DO'):
                self.append_log("✅ DO方法存在")
            else:
                self.append_error("❌ DO方法不存在")
            
            # 2. 檢查機械臂狀態
            status = self.robot_controller.get_robot_status()
            self.append_log(f"🤖 機械臂狀態: {status}")
            self.robot_mode_label.setText(str(status))
            
            # 3. 檢查使能狀態
            if self.is_enabled:
                self.append_log("✅ 機械臂已使能")
            else:
                self.append_error("❌ 機械臂未使能")
            
            # 4. 測試DO1控制
            self.append_log("🧪 測試DO1控制...")
            
            # 測試隊列指令
            start_time = time.time()
            success1 = self.robot_controller.set_do_queue(1, 1)
            delay1 = (time.time() - start_time) * 1000
            
            if success1:
                self.append_log(f"✅ DO1隊列指令成功 (延遲: {delay1:.1f}ms)")
            else:
                self.append_error("❌ DO1隊列指令失敗")
            
            time.sleep(0.5)
            
            # 測試立即指令
            start_time = time.time()
            success2 = self.robot_controller.set_do_execute(1, 0)
            delay2 = (time.time() - start_time) * 1000
            
            if success2:
                self.append_log(f"✅ DO1立即指令成功 (延遲: {delay2:.1f}ms)")
                self.do_buttons[1].set_state(False)
            else:
                self.append_error("❌ DO1立即指令失敗")
            
            # 5. 檢查錯誤狀態
            error_status = self.robot_controller.get_error_status()
            if "[]" in str(error_status) or "0," in str(error_status):
                self.append_log("✅ 無機械臂錯誤")
            else:
                self.append_error(f"⚠️ 機械臂錯誤: {error_status}")
            
            self.append_log("🔍 === DO控制診斷完成 ===")
            
        except Exception as e:
            self.append_error(f"❌ 診斷過程發生錯誤: {str(e)}")
    
    def check_robot_status(self):
        """檢查機械臂狀態"""
        self.append_log("📊 === 檢查機械臂狀態 ===")
        
        if not self.is_connected:
            self.append_error("❌ 機械臂未連接")
            return
            
        try:
            # 機械臂模式
            status = self.robot_controller.get_robot_status()
            self.append_log(f"🤖 機械臂模式: {status}")
            self.robot_mode_label.setText(str(status))
            
            # 連接狀態
            self.append_log(f"🔗 連接狀態: {'已連接' if self.is_connected else '未連接'}")
            self.append_log(f"⚡ 使能狀態: {'已使能' if self.is_enabled else '未使能'}")
            
            # 控制器信息
            if hasattr(self.robot_controller, 'client_dash') and self.robot_controller.client_dash:
                self.append_log("✅ Dashboard客戶端: 正常")
            else:
                self.append_error("❌ Dashboard客戶端: 異常")
                
        except Exception as e:
            self.append_error(f"❌ 狀態檢查失敗: {str(e)}")
    
    def check_error_status(self):
        """檢查錯誤狀態"""
        self.append_log("⚠️ === 檢查錯誤狀態 ===")
        
        if not self.is_connected:
            self.append_error("❌ 機械臂未連接")
            return
            
        try:
            error_status = self.robot_controller.get_error_status()
            self.append_log(f"⚠️ 錯誤狀態: {error_status}")
            
            if "[]" in str(error_status) or "0," in str(error_status):
                self.append_log("✅ 無機械臂錯誤")
            else:
                self.append_error(f"⚠️ 發現錯誤: {error_status}")
                
        except Exception as e:
            self.append_error(f"❌ 錯誤狀態檢查失敗: {str(e)}")

    # ==================== 統計和顯示更新 ====================
    
    def update_statistics(self):
        """更新統計信息"""
        try:
            self.success_label.setText(str(self.success_count))
            self.failure_label.setText(str(self.failure_count))
            
            # 計算成功率
            total = self.success_count + self.failure_count
            if total > 0:
                success_rate = (self.success_count / total) * 100
                self.success_rate_label.setText(f"{success_rate:.1f}%")
            else:
                self.success_rate_label.setText("0%")
            
            # 計算平均延遲
            if self.test_count > 0:
                avg_delay = self.delay_sum / self.test_count
                self.avg_delay_label.setText(f"{avg_delay:.1f}ms")
            else:
                self.avg_delay_label.setText("0ms")
                
        except Exception as e:
            print(f"統計更新錯誤: {e}")
    
    def update_do_count(self):
        """更新DO計數"""
        try:
            on_count = sum(1 for btn in self.do_buttons.values() if btn.is_on)
            off_count = 16 - on_count
            
            self.do_on_count.setText(str(on_count))
            self.do_off_count.setText(str(off_count))
            
        except Exception as e:
            print(f"DO計數更新錯誤: {e}")
    
    def clear_statistics(self):
        """清除統計"""
        self.success_count = 0
        self.failure_count = 0
        self.test_count = 0
        self.delay_sum = 0
        self.update_statistics()
        self.append_log("🗑️ 統計數據已清除")

    # ==================== 日誌處理 ====================
    
    @pyqtSlot(str)
    def append_log(self, message):
        """添加日誌"""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            formatted_message = f"[{timestamp}] {message}"
            
            if hasattr(self, 'log_text'):
                self.log_text.append(formatted_message)
                
                if hasattr(self, 'auto_scroll_check') and self.auto_scroll_check.isChecked():
                    scrollbar = self.log_text.verticalScrollBar()
                    scrollbar.setValue(scrollbar.maximum())
                
                # 限制日誌行數
                if self.log_text.document().blockCount() > 1000:
                    cursor = self.log_text.textCursor()
                    cursor.movePosition(cursor.Start)
                    cursor.movePosition(cursor.Down, cursor.KeepAnchor, 200)
                    cursor.removeSelectedText()
            else:
                print(formatted_message)
                
        except Exception as e:
            print(f"日誌錯誤: {e}")
    
    @pyqtSlot(str)
    def append_error(self, message):
        """添加錯誤信息"""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            formatted_message = f"[{timestamp}] {message}"
            
            if hasattr(self, 'error_text'):
                self.error_text.append(formatted_message)
                
                # 自動滾動到底部
                scrollbar = self.error_text.verticalScrollBar()
                scrollbar.setValue(scrollbar.maximum())
            else:
                print(f"ERROR: {formatted_message}")
                
        except Exception as e:
            print(f"錯誤日誌錯誤: {e}")
    
    def clear_logs(self):
        """清除日誌"""
        self.log_text.clear()
        self.append_log("📝 日誌已清除")
    
    def clear_errors(self):
        """清除錯誤"""
        self.error_text.clear()

    # ==================== 關閉事件 ====================
    
    def closeEvent(self, event):
        """關閉事件"""
        try:
            # 停止所有測試
            self.sequence_testing = False
            self.batch_testing = False
            
            # 停止定時器
            if hasattr(self, 'sequence_timer'):
                self.sequence_timer.stop()
            if hasattr(self, 'cycle_timer'):
                self.cycle_timer.stop()
            
            # 斷開機械臂連接
            if self.is_connected:
                self.robot_controller.disconnect_robot()
                
            event.accept()
        except Exception as e:
            print(f"關閉錯誤: {e}")
            event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # 設置應用程式樣式
    app.setStyle('Fusion')
    
    window = DOTestMainWindow()
    window.show()
    
    sys.exit(app.exec_())