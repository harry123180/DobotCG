#main.py
import sys
from datetime import datetime
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from logic import RobotController, PointDataValidator, LABEL_ROBOT_MODE

class PointEditDialog(QDialog):
    """點位編輯對話框 - 修正版本，完全移除上下限拘束"""
    def __init__(self, point_data=None, parent=None):
        super().__init__(parent)
        self.point_data = point_data or {}
        self.setupUI()
        self.load_data()
        
    def setupUI(self):
        self.setWindowTitle("編輯點位")
        self.setFixedSize(400, 350)
        layout = QVBoxLayout()
        
        # 名稱輸入
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("點位名稱:"))
        self.name_edit = QLineEdit()
        name_layout.addWidget(self.name_edit)
        layout.addLayout(name_layout)
        
        # 笛卡爾座標 - 完全移除範圍限制
        cart_group = QGroupBox("笛卡爾座標 (mm/度)")
        cart_layout = QGridLayout()
        
        # X軸 - 無限制範圍
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-999999.99, 999999.99)
        self.x_spin.setDecimals(2)
        
        # Y軸 - 無限制範圍
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-999999.99, 999999.99)
        self.y_spin.setDecimals(2)
        
        # Z軸 - 無限制範圍
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-999999.99, 999999.99)
        self.z_spin.setDecimals(2)
        
        # R軸 - 無限制範圍
        self.r_spin = QDoubleSpinBox()
        self.r_spin.setRange(-999999.99, 999999.99)
        self.r_spin.setDecimals(2)
        
        cart_layout.addWidget(QLabel("X:"), 0, 0)
        cart_layout.addWidget(self.x_spin, 0, 1)
        cart_layout.addWidget(QLabel("Y:"), 0, 2)
        cart_layout.addWidget(self.y_spin, 0, 3)
        cart_layout.addWidget(QLabel("Z:"), 1, 0)
        cart_layout.addWidget(self.z_spin, 1, 1)
        cart_layout.addWidget(QLabel("R:"), 1, 2)
        cart_layout.addWidget(self.r_spin, 1, 3)
        cart_group.setLayout(cart_layout)
        layout.addWidget(cart_group)
        
        # 關節座標 - 完全移除範圍限制
        joint_group = QGroupBox("關節座標 (度)")
        joint_layout = QGridLayout()
        
        # J1軸 - 無限制範圍
        self.j1_spin = QDoubleSpinBox()
        self.j1_spin.setRange(-999999.99, 999999.99)
        self.j1_spin.setDecimals(2)
        
        # J2軸 - 無限制範圍
        self.j2_spin = QDoubleSpinBox()
        self.j2_spin.setRange(-999999.99, 999999.99)
        self.j2_spin.setDecimals(2)
        
        # J3軸 - 無限制範圍
        self.j3_spin = QDoubleSpinBox()
        self.j3_spin.setRange(-999999.99, 999999.99)
        self.j3_spin.setDecimals(2)
        
        # J4軸 - 無限制範圍
        self.j4_spin = QDoubleSpinBox()
        self.j4_spin.setRange(-999999.99, 999999.99)
        self.j4_spin.setDecimals(2)
        
        joint_layout.addWidget(QLabel("J1:"), 0, 0)
        joint_layout.addWidget(self.j1_spin, 0, 1)
        joint_layout.addWidget(QLabel("J2:"), 0, 2)
        joint_layout.addWidget(self.j2_spin, 0, 3)
        joint_layout.addWidget(QLabel("J3:"), 1, 0)
        joint_layout.addWidget(self.j3_spin, 1, 1)
        joint_layout.addWidget(QLabel("J4:"), 1, 2)
        joint_layout.addWidget(self.j4_spin, 1, 3)
        joint_group.setLayout(joint_layout)
        layout.addWidget(joint_group)
        
        # 按鈕
        button_layout = QHBoxLayout()
        self.ok_btn = QPushButton("確定")
        self.cancel_btn = QPushButton("取消")
        self.ok_btn.clicked.connect(self.accept)
        self.cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(self.ok_btn)
        button_layout.addWidget(self.cancel_btn)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
        
    def load_data(self):
        """載入點位數據"""
        if self.point_data:
            self.name_edit.setText(self.point_data.get('name', ''))
            cartesian = self.point_data.get('cartesian', {})
            self.x_spin.setValue(cartesian.get('x', 0))
            self.y_spin.setValue(cartesian.get('y', 0))
            self.z_spin.setValue(cartesian.get('z', 0))
            self.r_spin.setValue(cartesian.get('r', 0))
            
            joint = self.point_data.get('joint', {})
            self.j1_spin.setValue(joint.get('j1', 0))
            self.j2_spin.setValue(joint.get('j2', 0))
            self.j3_spin.setValue(joint.get('j3', 0))
            self.j4_spin.setValue(joint.get('j4', 0))
    
    def get_data(self):
        """獲取編輯後的數據"""
        return {
            'name': self.name_edit.text(),
            'cartesian': {
                'x': self.x_spin.value(),
                'y': self.y_spin.value(),
                'z': self.z_spin.value(),
                'r': self.r_spin.value()
            },
            'joint': {
                'j1': self.j1_spin.value(),
                'j2': self.j2_spin.value(),
                'j3': self.j3_spin.value(),
                'j4': self.j4_spin.value()
            },
            'created_time': self.point_data.get('created_time', datetime.now().isoformat()),
            'modified_time': datetime.now().isoformat()
        }

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MG400/M1Pro Python Demo - PyQt版")
        self.setFixedSize(1400, 1000)
        
        # 機械臂控制器
        self.robot_controller = RobotController()
        self.connect_signals()
        
        # 控制狀態
        self.is_connected = False
        self.is_enabled = False
        
        self.setupUI()
        self.refresh_points_list()
        
    def connect_signals(self):
        """連接信號槽"""
        self.robot_controller.feedback_update.connect(self.update_feedback_display, Qt.QueuedConnection)
        self.robot_controller.log_update.connect(self.append_log, Qt.QueuedConnection)
        self.robot_controller.error_update.connect(self.append_error, Qt.QueuedConnection)
        self.robot_controller.connection_changed.connect(self.on_connection_changed, Qt.QueuedConnection)
        self.robot_controller.enable_changed.connect(self.on_enable_changed, Qt.QueuedConnection)
        
    def setupUI(self):
        """建立UI界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 設定窗口標題
        self.setWindowTitle("MG400/M1Pro Python Demo - PyQt版 (ESC=緊急停止) - 高頻反饋模式")
        
        # 主布局
        main_layout = QVBoxLayout()
        
        # 連接設定區域
        main_layout.addWidget(self.create_connection_group())
        
        # 中間區域 - 左右分割
        middle_layout = QHBoxLayout()
        
        # 左側區域
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.create_dashboard_group())
        left_layout.addWidget(self.create_move_group())
        left_layout.addWidget(self.create_gripper_group())
        
        # 右側區域 - 點位管理
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.create_points_group())
        
        # 設定左右比例
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        
        middle_layout.addWidget(left_widget, 2)
        middle_layout.addWidget(right_widget, 1)
        
        main_layout.addLayout(middle_layout)
        
        # 底部狀態顯示區域
        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(self.create_feedback_group(), 2)
        bottom_layout.addWidget(self.create_log_group(), 1)
        
        main_layout.addLayout(bottom_layout)
        
        central_widget.setLayout(main_layout)
        
        # 設定焦點策略，確保能接收鍵盤事件
        self.setFocusPolicy(Qt.StrongFocus)
        
        # 啟動UI更新定時器
        self.start_ui_update_timer()
        
    def create_connection_group(self):
        """建立連接設定群組"""
        group = QGroupBox("機械臂連接設定")
        layout = QHBoxLayout()
        
        layout.addWidget(QLabel("IP:"))
        self.ip_edit = QLineEdit("192.168.1.6")
        self.ip_edit.setFixedWidth(120)
        layout.addWidget(self.ip_edit)
        
        layout.addWidget(QLabel("Dashboard:"))
        self.dash_edit = QLineEdit("29999")
        self.dash_edit.setFixedWidth(60)
        layout.addWidget(self.dash_edit)
        
        layout.addWidget(QLabel("Move:"))
        self.move_edit = QLineEdit("30003")
        self.move_edit.setFixedWidth(60)
        layout.addWidget(self.move_edit)
        
        layout.addWidget(QLabel("Feed:"))
        self.feed_edit = QLineEdit("30004")
        self.feed_edit.setFixedWidth(60)
        layout.addWidget(self.feed_edit)
        
        self.connect_btn = QPushButton("連接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
        
    def create_dashboard_group(self):
        """建立控制面板群組 - 增加速度控制"""
        group = QGroupBox("機械臂控制")
        layout = QGridLayout()
        
        # 緊急停止按鈕
        self.emergency_stop_btn = QPushButton("緊急停止")
        self.emergency_stop_btn.setFixedSize(120, 60)
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF4444;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border: 3px solid #CC0000;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #FF6666;
            }
            QPushButton:pressed {
                background-color: #CC0000;
            }
        """)
        self.emergency_stop_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_stop_btn, 0, 0, 2, 1)
        
        # 使能/下使能
        self.enable_btn = QPushButton("使能")
        self.enable_btn.clicked.connect(self.toggle_enable)
        self.enable_btn.setEnabled(False)
        layout.addWidget(self.enable_btn, 0, 1)
        
        # 重置機械臂
        self.reset_btn = QPushButton("重置機械臂")
        self.reset_btn.clicked.connect(self.reset_robot)
        self.reset_btn.setEnabled(False)
        layout.addWidget(self.reset_btn, 0, 2)
        
        # 清除錯誤
        self.clear_btn = QPushButton("清除錯誤")
        self.clear_btn.clicked.connect(self.clear_error)
        self.clear_btn.setEnabled(False)
        layout.addWidget(self.clear_btn, 0, 3)
        
        # 速度控制區域
        speed_group = QGroupBox("速度控制")
        speed_layout = QGridLayout()
        
        # 全局速度比例
        speed_layout.addWidget(QLabel("全局速度:"), 0, 0)
        self.global_speed_spin = QSpinBox()
        self.global_speed_spin.setRange(1, 100)
        self.global_speed_spin.setValue(50)
        self.global_speed_spin.setSuffix("%")
        speed_layout.addWidget(self.global_speed_spin, 0, 1)
        
        self.global_speed_btn = QPushButton("設定")
        self.global_speed_btn.clicked.connect(self.set_global_speed)
        self.global_speed_btn.setEnabled(False)
        speed_layout.addWidget(self.global_speed_btn, 0, 2)
        
        # 關節運動速度
        speed_layout.addWidget(QLabel("關節速度:"), 1, 0)
        self.joint_speed_spin = QSpinBox()
        self.joint_speed_spin.setRange(1, 100)
        self.joint_speed_spin.setValue(50)
        self.joint_speed_spin.setSuffix("%")
        speed_layout.addWidget(self.joint_speed_spin, 1, 1)
        
        self.joint_speed_btn = QPushButton("設定")
        self.joint_speed_btn.clicked.connect(self.set_joint_speed)
        self.joint_speed_btn.setEnabled(False)
        speed_layout.addWidget(self.joint_speed_btn, 1, 2)
        
        # 直線運動速度
        speed_layout.addWidget(QLabel("直線速度:"), 2, 0)
        self.linear_speed_spin = QSpinBox()
        self.linear_speed_spin.setRange(1, 100)
        self.linear_speed_spin.setValue(50)
        self.linear_speed_spin.setSuffix("%")
        speed_layout.addWidget(self.linear_speed_spin, 2, 1)
        
        self.linear_speed_btn = QPushButton("設定")
        self.linear_speed_btn.clicked.connect(self.set_linear_speed)
        self.linear_speed_btn.setEnabled(False)
        speed_layout.addWidget(self.linear_speed_btn, 2, 2)
        
        # 加速度控制
        speed_layout.addWidget(QLabel("關節加速度:"), 0, 3)
        self.joint_acc_spin = QSpinBox()
        self.joint_acc_spin.setRange(1, 100)
        self.joint_acc_spin.setValue(50)
        self.joint_acc_spin.setSuffix("%")
        speed_layout.addWidget(self.joint_acc_spin, 0, 4)
        
        self.joint_acc_btn = QPushButton("設定")
        self.joint_acc_btn.clicked.connect(self.set_joint_acc)
        self.joint_acc_btn.setEnabled(False)
        speed_layout.addWidget(self.joint_acc_btn, 0, 5)
        
        speed_layout.addWidget(QLabel("直線加速度:"), 1, 3)
        self.linear_acc_spin = QSpinBox()
        self.linear_acc_spin.setRange(1, 100)
        self.linear_acc_spin.setValue(50)
        self.linear_acc_spin.setSuffix("%")
        speed_layout.addWidget(self.linear_acc_spin, 1, 4)
        
        self.linear_acc_btn = QPushButton("設定")
        self.linear_acc_btn.clicked.connect(self.set_linear_acc)
        self.linear_acc_btn.setEnabled(False)
        speed_layout.addWidget(self.linear_acc_btn, 1, 5)
        
        # 快速設定按鈕
        speed_layout.addWidget(QLabel("快速設定:"), 2, 3)
        self.speed_preset_combo = QComboBox()
        self.speed_preset_combo.addItems(["低速(20%)", "中速(50%)", "高速(80%)", "最高速(100%)"])
        self.speed_preset_combo.setCurrentIndex(1)
        speed_layout.addWidget(self.speed_preset_combo, 2, 4)
        
        self.apply_preset_btn = QPushButton("套用")
        self.apply_preset_btn.clicked.connect(self.apply_speed_preset)
        self.apply_preset_btn.setEnabled(False)
        speed_layout.addWidget(self.apply_preset_btn, 2, 5)
        
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group, 1, 1, 1, 3)
        
        # DO控制
        layout.addWidget(QLabel("DO索引:"), 2, 0)
        self.do_index_spin = QSpinBox()
        self.do_index_spin.setRange(1, 24)
        layout.addWidget(self.do_index_spin, 2, 1)
        
        self.do_status_combo = QComboBox()
        self.do_status_combo.addItems(["高電平", "低電平"])
        layout.addWidget(self.do_status_combo, 2, 2)
        
        self.do_confirm_btn = QPushButton("設定DO(隊列)")
        self.do_confirm_btn.clicked.connect(self.confirm_do)
        self.do_confirm_btn.setEnabled(False)
        layout.addWidget(self.do_confirm_btn, 2, 3)
        
        # 快速DO控制區域
        quick_do_group = QGroupBox("快速DO控制 (立即執行)")
        quick_do_layout = QGridLayout()
        
        # 建立快速DO按鈕 (DO1-DO8)
        self.quick_do_buttons = {}
        for i in range(8):
            do_index = i + 1
            
            # DO狀態標籤
            status_label = QLabel(f"DO{do_index}: OFF")
            status_label.setStyleSheet("color: gray; font-weight: bold;")
            quick_do_layout.addWidget(status_label, i // 4, (i % 4) * 3)
            
            # ON按鈕
            on_btn = QPushButton("ON")
            on_btn.setFixedSize(40, 30)
            on_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-weight: bold;
                    border: 1px solid #45a049;
                    border-radius: 3px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
                QPushButton:disabled {
                    background-color: #cccccc;
                    color: #666666;
                }
            """)
            on_btn.clicked.connect(lambda checked, idx=do_index: self.quick_do_on(idx))
            on_btn.setEnabled(False)
            quick_do_layout.addWidget(on_btn, i // 4, (i % 4) * 3 + 1)
            
            # OFF按鈕
            off_btn = QPushButton("OFF")
            off_btn.setFixedSize(40, 30)
            off_btn.setStyleSheet("""
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    font-weight: bold;
                    border: 1px solid #da190b;
                    border-radius: 3px;
                }
                QPushButton:hover {
                    background-color: #da190b;
                }
                QPushButton:disabled {
                    background-color: #cccccc;
                    color: #666666;
                }
            """)
            off_btn.clicked.connect(lambda checked, idx=do_index: self.quick_do_off(idx))
            off_btn.setEnabled(False)
            quick_do_layout.addWidget(off_btn, i // 4, (i % 4) * 3 + 2)
            
            # 保存按鈕引用
            self.quick_do_buttons[do_index] = {
                'status_label': status_label,
                'on_btn': on_btn,
                'off_btn': off_btn
            }
        
        quick_do_group.setLayout(quick_do_layout)
        layout.addWidget(quick_do_group, 3, 0, 1, 4)
        
        # 保存按鈕引用 (增加快速DO按鈕)
        quick_do_btn_list = []
        for buttons in self.quick_do_buttons.values():
            quick_do_btn_list.extend([buttons['on_btn'], buttons['off_btn']])
        
        self.control_buttons = [
            self.reset_btn, self.clear_btn, self.do_confirm_btn,
            self.global_speed_btn, self.joint_speed_btn, self.linear_speed_btn,
            self.joint_acc_btn, self.linear_acc_btn, self.apply_preset_btn
        ] + quick_do_btn_list
        
        group.setLayout(layout)
        return group
    
    def start_ui_update_timer(self):
        """啟動UI更新定時器 - 控制UI刷新頻率"""
        self.ui_update_timer = QTimer()
        self.ui_update_timer.timeout.connect(self.update_ui_info)
        self.ui_update_timer.start(50)  # 50ms = 20Hz UI更新頻率
        
        # 緩存最新的反饋數據
        self.latest_feedback_data = None
        
    def update_ui_info(self):
        """定時更新UI信息 - 降低UI更新頻率"""
        if self.latest_feedback_data and self.is_connected:
            # 只更新關鍵信息，避免過於頻繁的UI刷新
            try:
                data = self.latest_feedback_data
                
                # 更新數值顯示
                speed = data.get('speed_scaling', 0)
                self.current_speed_label.setText(f"{speed:.1f}%")
                
                mode = data.get('robot_mode', 0)
                mode_text = LABEL_ROBOT_MODE.get(mode, f"未知模式({mode})")
                self.robot_mode_label.setText(mode_text)
                
                # 更新位置信息
                pos = data.get('tool_vector_actual', [0, 0, 0, 0])
                if pos and len(pos) >= 4:
                    pos_text = f"{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}"
                    self.position_label.setText(pos_text)
                
                joints = data.get('q_actual', [0, 0, 0, 0])
                if joints and len(joints) >= 4:
                    joint_text = f"{joints[0]:.2f}, {joints[1]:.2f}, {joints[2]:.2f}, {joints[3]:.2f}"
                    self.joint_label.setText(joint_text)
                
                # 更新IO狀態 (降低頻率)
                if hasattr(self, 'io_update_counter'):
                    self.io_update_counter += 1
                else:
                    self.io_update_counter = 0
                    
                if self.io_update_counter % 4 == 0:  # 每4次更新一次IO (200ms)
                    di_bits = data.get('digital_input_bits', 0)
                    do_bits = data.get('digital_outputs', 0)
                    
                    di_str = format(di_bits & 0xFFFFFF, '024b')
                    do_str = format(do_bits & 0xFFFFFF, '024b')
                    
                    self.di_label.setText(di_str)
                    self.do_label.setText(do_str)
                    
                    # 更新快速DO按鈕狀態顯示
                    self.update_all_do_status_from_feedback(do_bits)
                
            except Exception as e:
                pass  # 靜默處理UI更新錯誤
        
    def create_move_group(self):
        """建立運動控制群組 - 修正版本，完全移除上下限拘束"""
        group = QGroupBox("運動控制")
        layout = QGridLayout()
        
        # 笛卡爾座標輸入 - 完全移除範圍限制
        layout.addWidget(QLabel("X:"), 0, 0)
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-999999.99, 999999.99)
        self.x_spin.setValue(600)
        self.x_spin.setDecimals(2)
        layout.addWidget(self.x_spin, 0, 1)
        
        layout.addWidget(QLabel("Y:"), 0, 2)
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-999999.99, 999999.99)
        self.y_spin.setValue(-260)
        self.y_spin.setDecimals(2)
        layout.addWidget(self.y_spin, 0, 3)
        
        layout.addWidget(QLabel("Z:"), 0, 4)
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-999999.99, 999999.99)
        self.z_spin.setValue(380)
        self.z_spin.setDecimals(2)
        layout.addWidget(self.z_spin, 0, 5)
        
        layout.addWidget(QLabel("R:"), 0, 6)
        self.r_spin = QDoubleSpinBox()
        self.r_spin.setRange(-999999.99, 999999.99)
        self.r_spin.setValue(170)
        self.r_spin.setDecimals(2)
        layout.addWidget(self.r_spin, 0, 7)
        
        # 運動按鈕
        self.movj_btn = QPushButton("MovJ")
        self.movj_btn.clicked.connect(self.movj)
        self.movj_btn.setEnabled(False)
        layout.addWidget(self.movj_btn, 1, 0)
        
        self.movl_btn = QPushButton("MovL")
        self.movl_btn.clicked.connect(self.movl)
        self.movl_btn.setEnabled(False)
        layout.addWidget(self.movl_btn, 1, 1)
        
        # 速度控制
        layout.addWidget(QLabel("MovL速度:"), 1, 2)
        self.movl_speed_slider = QSlider(Qt.Horizontal)
        self.movl_speed_slider.setRange(1, 100)
        self.movl_speed_slider.setValue(50)
        layout.addWidget(self.movl_speed_slider, 1, 3, 1, 2)
        
        self.speed_label = QLabel("50%")
        self.movl_speed_slider.valueChanged.connect(
            lambda v: self.speed_label.setText(f"{v}%"))
        layout.addWidget(self.speed_label, 1, 5)
        
        # 關節座標輸入 - 完全移除範圍限制
        layout.addWidget(QLabel("J1:"), 2, 0)
        self.j1_spin = QDoubleSpinBox()
        self.j1_spin.setRange(-999999.99, 999999.99)
        self.j1_spin.setValue(0)
        self.j1_spin.setDecimals(2)
        layout.addWidget(self.j1_spin, 2, 1)
        
        layout.addWidget(QLabel("J2:"), 2, 2)
        self.j2_spin = QDoubleSpinBox()
        self.j2_spin.setRange(-999999.99, 999999.99)
        self.j2_spin.setValue(-20)
        self.j2_spin.setDecimals(2)
        layout.addWidget(self.j2_spin, 2, 3)
        
        layout.addWidget(QLabel("J3:"), 2, 4)
        self.j3_spin = QDoubleSpinBox()
        self.j3_spin.setRange(-999999.99, 999999.99)
        self.j3_spin.setValue(-80)
        self.j3_spin.setDecimals(2)
        layout.addWidget(self.j3_spin, 2, 5)
        
        layout.addWidget(QLabel("J4:"), 2, 6)
        self.j4_spin = QDoubleSpinBox()
        self.j4_spin.setRange(-999999.99, 999999.99)
        self.j4_spin.setValue(30)
        self.j4_spin.setDecimals(2)
        layout.addWidget(self.j4_spin, 2, 7)
        
        self.joint_movj_btn = QPushButton("JointMovJ")
        self.joint_movj_btn.clicked.connect(self.joint_movj)
        self.joint_movj_btn.setEnabled(False)
        layout.addWidget(self.joint_movj_btn, 3, 0)
        
        # 點動控制按鈕群組
        jog_group = QGroupBox("點動控制")
        jog_layout = QGridLayout()
        jog_layout.setSpacing(8)
        
        # 關節點動
        self.create_jog_buttons(jog_layout, 0, ["J1-", "J1+"], ["j1-", "j1+"])
        self.create_jog_buttons(jog_layout, 1, ["J2-", "J2+"], ["j2-", "j2+"])
        self.create_jog_buttons(jog_layout, 2, ["J3-", "J3+"], ["j3-", "j3+"])
        self.create_jog_buttons(jog_layout, 3, ["J4-", "J4+"], ["j4-", "j4+"])
        
        # 笛卡爾點動
        self.create_jog_buttons(jog_layout, 0, ["X-", "X+"], ["x-", "x+"], col_offset=3)
        self.create_jog_buttons(jog_layout, 1, ["Y-", "Y+"], ["y-", "y+"], col_offset=3)
        self.create_jog_buttons(jog_layout, 2, ["Z-", "Z+"], ["z-", "z+"], col_offset=3)
        self.create_jog_buttons(jog_layout, 3, ["R-", "R+"], ["r-", "r+"], col_offset=3)
        
        jog_group.setLayout(jog_layout)
        layout.addWidget(jog_group, 4, 0, 1, 8)
        
        # 保存運動按鈕引用
        self.move_buttons = [self.movj_btn, self.movl_btn, self.joint_movj_btn]
        
        group.setLayout(layout)
        return group
        
    def create_jog_buttons(self, layout, row, labels, commands, col_offset=0):
        """建立點動按鈕"""
        for i, (label, cmd) in enumerate(zip(labels, commands)):
            btn = QPushButton(label)
            btn.setMinimumSize(60, 40)
            btn.setEnabled(False)
            btn.pressed.connect(lambda c=cmd: self.start_jog(c))
            btn.released.connect(self.stop_jog)
            layout.addWidget(btn, row, i + col_offset)
            if not hasattr(self, 'jog_buttons'):
                self.jog_buttons = []
            self.jog_buttons.append(btn)
    
    def create_gripper_group(self):
        """建立PGC夾爪控制群組"""
        group = QGroupBox("PGC夾爪控制")
        layout = QGridLayout()
        
        # 連接狀態顯示
        layout.addWidget(QLabel("連接狀態:"), 0, 0)
        self.gripper_status_label = QLabel("未連接")
        self.gripper_status_label.setStyleSheet("color: red")
        layout.addWidget(self.gripper_status_label, 0, 1)
        
        # 夾爪狀態顯示
        layout.addWidget(QLabel("夾持狀態:"), 0, 2)
        self.gripper_hold_label = QLabel("未知")
        layout.addWidget(self.gripper_hold_label, 0, 3)
        
        # 位置顯示
        layout.addWidget(QLabel("當前位置:"), 0, 4)
        self.gripper_pos_label = QLabel("0")
        layout.addWidget(self.gripper_pos_label, 0, 5)
        
        # 控制按鈕
        self.gripper_init_btn = QPushButton("初始化")
        self.gripper_init_btn.clicked.connect(lambda: self.robot_controller.send_gripper_command(1))
        self.gripper_init_btn.setEnabled(False)
        layout.addWidget(self.gripper_init_btn, 1, 0)
        
        self.gripper_open_btn = QPushButton("快速開啟")
        self.gripper_open_btn.clicked.connect(lambda: self.robot_controller.send_gripper_command(7))
        self.gripper_open_btn.setEnabled(False)
        layout.addWidget(self.gripper_open_btn, 1, 1)
        
        self.gripper_close_btn = QPushButton("快速關閉")
        self.gripper_close_btn.clicked.connect(lambda: self.robot_controller.send_gripper_command(8))
        self.gripper_close_btn.setEnabled(False)
        layout.addWidget(self.gripper_close_btn, 1, 2)
        
        self.gripper_stop_btn = QPushButton("停止")
        self.gripper_stop_btn.clicked.connect(lambda: self.robot_controller.send_gripper_command(2))
        self.gripper_stop_btn.setEnabled(False)
        layout.addWidget(self.gripper_stop_btn, 1, 3)
        
        # 位置控制
        layout.addWidget(QLabel("目標位置:"), 2, 0)
        self.gripper_pos_spin = QSpinBox()
        self.gripper_pos_spin.setRange(0, 1000)
        self.gripper_pos_spin.setValue(500)
        layout.addWidget(self.gripper_pos_spin, 2, 1)
        
        self.gripper_pos_btn = QPushButton("移動到位置")
        self.gripper_pos_btn.clicked.connect(self.gripper_move_to_pos)
        self.gripper_pos_btn.setEnabled(False)
        layout.addWidget(self.gripper_pos_btn, 2, 2)
        
        # 力道控制
        layout.addWidget(QLabel("夾持力道:"), 2, 3)
        self.gripper_force_spin = QSpinBox()
        self.gripper_force_spin.setRange(20, 100)
        self.gripper_force_spin.setValue(50)
        layout.addWidget(self.gripper_force_spin, 2, 4)
        
        self.gripper_force_btn = QPushButton("設定力道")
        self.gripper_force_btn.clicked.connect(self.gripper_set_force)
        self.gripper_force_btn.setEnabled(False)
        layout.addWidget(self.gripper_force_btn, 2, 5)
        
        # 保存夾爪按鈕引用
        self.gripper_buttons = [self.gripper_init_btn, self.gripper_open_btn, self.gripper_close_btn, 
                                self.gripper_stop_btn, self.gripper_pos_btn, self.gripper_force_btn]
        
        group.setLayout(layout)
        return group
    
    def create_points_group(self):
        """建立點位管理群組"""
        group = QGroupBox("點位管理")
        layout = QVBoxLayout()
        
        # 操作按鈕
        btn_layout = QHBoxLayout()
        
        self.save_point_btn = QPushButton("保存當前點位")
        self.save_point_btn.clicked.connect(self.save_current_point)
        self.save_point_btn.setEnabled(False)
        self.save_point_btn.setToolTip("保存機械臂當前實際位置為點位")
        btn_layout.addWidget(self.save_point_btn)
        
        self.sync_btn = QPushButton("同步到輸入框")
        self.sync_btn.clicked.connect(self.sync_current_position)
        self.sync_btn.setEnabled(False)
        self.sync_btn.setToolTip("將機械臂當前位置同步到移動控制輸入框")
        btn_layout.addWidget(self.sync_btn)
        
        edit_btn = QPushButton("編輯點位")
        edit_btn.clicked.connect(self.edit_selected_point)
        btn_layout.addWidget(edit_btn)
        
        delete_btn = QPushButton("刪除點位")
        delete_btn.clicked.connect(self.delete_selected_point)
        btn_layout.addWidget(delete_btn)
        
        layout.addLayout(btn_layout)
        
        # 點位列表
        self.points_list = QListWidget()
        self.points_list.itemDoubleClicked.connect(self.move_to_selected_point_with_dialog)
        layout.addWidget(self.points_list)
        
        # 移動控制區域
        move_control_group = QGroupBox("移動控制")
        move_control_layout = QVBoxLayout()
        
        # 運動類型選擇
        motion_type_layout = QHBoxLayout()
        motion_type_layout.addWidget(QLabel("運動類型:"))
        
        self.motion_type_combo = QComboBox()
        self.motion_type_combo.addItems([
            "直線運動(MovL) - 可能遇到手勢切換問題",
            "關節運動(MovJ) - 避免手勢切換問題", 
            "關節座標運動(JointMovJ) - 使用儲存關節角度"
        ])
        self.motion_type_combo.setCurrentIndex(1)
        motion_type_layout.addWidget(self.motion_type_combo)
        
        move_control_layout.addLayout(motion_type_layout)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("移動速度:"))
        
        self.point_speed_slider = QSlider(Qt.Horizontal)
        self.point_speed_slider.setRange(1, 100)
        self.point_speed_slider.setValue(30)
        speed_layout.addWidget(self.point_speed_slider)
        
        self.point_speed_label = QLabel("30%")
        self.point_speed_slider.valueChanged.connect(
            lambda v: self.point_speed_label.setText(f"{v}%"))
        speed_layout.addWidget(self.point_speed_label)
        
        move_control_layout.addLayout(speed_layout)
        
        # 移動按鈕
        move_btn_layout = QHBoxLayout()
        
        self.move_to_point_btn = QPushButton("移動到選中點位")
        self.move_to_point_btn.clicked.connect(self.move_to_selected_point)
        self.move_to_point_btn.setEnabled(False)
        move_btn_layout.addWidget(self.move_to_point_btn)
        
        self.quick_move_btn = QPushButton("快速移動(使用預設類型)")
        self.quick_move_btn.clicked.connect(lambda: self.move_to_selected_point(use_preset=True))
        self.quick_move_btn.setEnabled(False)
        move_btn_layout.addWidget(self.quick_move_btn)
        
        move_control_layout.addLayout(move_btn_layout)
        
        move_control_group.setLayout(move_control_layout)
        layout.addWidget(move_control_group)
        
        # 保存點位按鈕引用
        self.point_buttons = [self.save_point_btn, self.sync_btn, self.move_to_point_btn, self.quick_move_btn]
        
        group.setLayout(layout)
        return group
        
    def create_feedback_group(self):
        """建立狀態反饋群組"""
        group = QGroupBox("狀態反饋")
        layout = QGridLayout()
        
        # 速度比例
        layout.addWidget(QLabel("當前速度比例:"), 0, 0)
        self.current_speed_label = QLabel("0%")
        layout.addWidget(self.current_speed_label, 0, 1)
        
        # 機械臂模式
        layout.addWidget(QLabel("機械臂模式:"), 0, 2)
        self.robot_mode_label = QLabel("未知")
        layout.addWidget(self.robot_mode_label, 0, 3)
        
        # 位置反饋
        layout.addWidget(QLabel("當前位置(X,Y,Z,R):"), 1, 0)
        self.position_label = QLabel("0.00, 0.00, 0.00, 0.00")
        layout.addWidget(self.position_label, 1, 1, 1, 3)
        
        # 關節角度反饋
        layout.addWidget(QLabel("關節角度(J1,J2,J3,J4):"), 2, 0)
        self.joint_label = QLabel("0.00, 0.00, 0.00, 0.00")
        layout.addWidget(self.joint_label, 2, 1, 1, 3)
        
        # 數位IO
        layout.addWidget(QLabel("數位輸入:"), 3, 0)
        self.di_label = QLabel("")
        layout.addWidget(self.di_label, 3, 1, 1, 3)
        
        layout.addWidget(QLabel("數位輸出:"), 4, 0)
        self.do_label = QLabel("")
        layout.addWidget(self.do_label, 4, 1, 1, 3)
        
        group.setLayout(layout)
        return group
        
    def create_log_group(self):
        """建立日誌群組"""
        group = QGroupBox("系統日誌")
        layout = QVBoxLayout()
        
        # 日誌顯示區域
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(200)
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        
        # 錯誤顯示區域
        error_group = QGroupBox("錯誤信息")
        error_layout = QVBoxLayout()
        self.error_text = QTextEdit()
        self.error_text.setMaximumHeight(100)
        self.error_text.setReadOnly(True)
        error_layout.addWidget(self.error_text)
        
        clear_error_btn = QPushButton("清除錯誤信息")
        clear_error_btn.clicked.connect(self.clear_error_info)
        error_layout.addWidget(clear_error_btn)
        
        error_group.setLayout(error_layout)
        layout.addWidget(error_group)
        
        group.setLayout(layout)
        return group

    # ==================== 事件處理 ====================
    
    def keyPressEvent(self, event):
        """鍵盤快捷鍵處理"""
        # 按 ESC 鍵觸發緊急停止
        if event.key() == Qt.Key_Escape:
            self.emergency_stop()
        # 按 Space 鍵停止所有點動
        elif event.key() == Qt.Key_Space:
            self.robot_controller.stop_jog()
            self.append_log("空白鍵停止點動")
        else:
            super().keyPressEvent(event)
    
    def closeEvent(self, event):
        """關閉事件"""
        if hasattr(self, 'ui_update_timer'):
            self.ui_update_timer.stop()
        if self.is_connected:
            self.robot_controller.disconnect_robot()
        event.accept()

    # ==================== 信號槽處理 ====================
    
    @pyqtSlot(bool)
    def on_connection_changed(self, connected):
        """連接狀態變化處理 - 更新版本"""
        self.is_connected = connected
        if connected:
            self.connect_btn.setText("斷開")
        else:
            self.connect_btn.setText("連接")
            
        # 啟用/禁用控制按鈕 (包含新的速度控制按鈕)
        self.enable_btn.setEnabled(connected)
        for btn in self.control_buttons + self.move_buttons + self.jog_buttons + self.gripper_buttons + self.point_buttons:
            btn.setEnabled(connected)
    
    @pyqtSlot(bool)
    def on_enable_changed(self, enabled):
        """使能狀態變化處理"""
        self.is_enabled = enabled
        if enabled:
            self.enable_btn.setText("下使能")
        else:
            self.enable_btn.setText("使能")
    
    @pyqtSlot(dict)
    def update_feedback_display(self, data):
        """更新反饋顯示 - 高頻版本，只緩存數據"""
        try:
            # 只緩存最新數據，不直接更新UI
            self.latest_feedback_data = data
            
            # 更新控制器內部位置數據 (這個需要高頻更新)
            # 其他UI更新由定時器處理
            
            # 夾爪狀態更新 (低頻)
            if hasattr(self, 'gripper_update_counter'):
                self.gripper_update_counter += 1
            else:
                self.gripper_update_counter = 0
                
            if self.gripper_update_counter % 20 == 0:  # 每20次更新一次夾爪狀態
                self.update_gripper_display()
            
        except Exception as e:
            pass  # 靜默處理錯誤，避免日誌洪水
    
    @pyqtSlot(str)
    def append_log(self, message):
        """添加日誌 - 過濾高頻訊息"""
        # 過濾掉高頻率的反饋相關日誌
        filter_keywords = [
            "數據解析錯誤", "接收數據錯誤", "socket接收超時", 
            "測試值錯誤", "數據長度錯誤", "反饋循環運行正常"
        ]
        
        # 檢查是否為需要過濾的訊息
        should_filter = any(keyword in message for keyword in filter_keywords)
        
        # 只顯示重要訊息和低頻訊息
        if not should_filter or "錯誤次數超過" in message or "線程已" in message:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_text.append(f"[{timestamp}] {message}")
            
            # 限制日誌行數，避免記憶體問題
            if self.log_text.document().blockCount() > 1000:
                cursor = self.log_text.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.movePosition(cursor.Down, cursor.KeepAnchor, 100)
                cursor.removeSelectedText()
        
    @pyqtSlot(str)
    def append_error(self, message):
        """添加錯誤信息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.error_text.append(f"[{timestamp}] {message}")

    # ==================== 控制操作 ====================
    
    def toggle_connection(self):
        """切換連接狀態"""
        if self.is_connected:
            self.robot_controller.disconnect_robot()
        else:
            ip = self.ip_edit.text()
            dash_port = int(self.dash_edit.text())
            move_port = int(self.move_edit.text())
            feed_port = int(self.feed_edit.text())
            
            success = self.robot_controller.connect_robot(ip, dash_port, move_port, feed_port)
            if not success:
                QMessageBox.critical(self, "連接錯誤", "機械臂連接失敗，請檢查網路設定")
    
    def toggle_enable(self):
        """切換使能狀態"""
        self.robot_controller.toggle_enable()
    
    def emergency_stop(self):
        """緊急停止功能"""
        success = self.robot_controller.emergency_stop()
        if success:
            QMessageBox.warning(self, "緊急停止", 
                "緊急停止已觸發！\n" +
                "機械臂已停止所有運動。\n" +
                "請檢查安全狀況後重新使能機械臂。")
        else:
            QMessageBox.information(self, "緊急停止", "緊急停止按鈕已被按下。")
    
    def reset_robot(self):
        """重置機械臂"""
        self.robot_controller.reset_robot()
    
    def clear_error(self):
        """清除錯誤"""
        self.robot_controller.clear_error()
    
    def confirm_do(self):
        """確認DO設定 - 隊列指令"""
        index = self.do_index_spin.value()
        status = 1 if self.do_status_combo.currentText() == "高電平" else 0
        self.robot_controller.set_do(index, status)

    # ==================== 快速DO控制 ====================
    
    def quick_do_on(self, index):
        """快速DO開啟 - 立即執行"""
        success = self.robot_controller.set_do_execute(index, 1)
        if success:
            self.update_do_status_display(index, True)
    
    def quick_do_off(self, index):
        """快速DO關閉 - 立即執行"""
        success = self.robot_controller.set_do_execute(index, 0)
        if success:
            self.update_do_status_display(index, False)
    
    def update_do_status_display(self, index, is_on):
        """更新DO狀態顯示"""
        if index in self.quick_do_buttons:
            status_label = self.quick_do_buttons[index]['status_label']
            if is_on:
                status_label.setText(f"DO{index}: ON")
                status_label.setStyleSheet("color: green; font-weight: bold; font-size: 12px;")
            else:
                status_label.setText(f"DO{index}: OFF")
                status_label.setStyleSheet("color: red; font-weight: bold; font-size: 12px;")
    
    def update_all_do_status_from_feedback(self, do_bits):
        """從反饋數據更新所有DO狀態顯示"""
        for index in range(8):  # DO0-DO7
            is_on = bool((do_bits >> index) & 1)
            self.update_do_status_display(index, is_on)

    # ==================== 速度控制操作 ====================
    
    def set_global_speed(self):
        """設定全局速度"""
        speed = self.global_speed_spin.value()
        success = self.robot_controller.set_speed_factor(speed)
        if success:
            QMessageBox.information(self, "設定成功", f"全局速度已設定為 {speed}%")
    
    def set_joint_speed(self):
        """設定關節運動速度"""
        speed = self.joint_speed_spin.value()
        success = self.robot_controller.set_speed_j(speed)
        if success:
            QMessageBox.information(self, "設定成功", f"關節運動速度已設定為 {speed}%")
    
    def set_linear_speed(self):
        """設定直線運動速度"""
        speed = self.linear_speed_spin.value()
        success = self.robot_controller.set_speed_l(speed)
        if success:
            QMessageBox.information(self, "設定成功", f"直線運動速度已設定為 {speed}%")
    
    def set_joint_acc(self):
        """設定關節運動加速度"""
        acc = self.joint_acc_spin.value()
        success = self.robot_controller.set_acc_j(acc)
        if success:
            QMessageBox.information(self, "設定成功", f"關節運動加速度已設定為 {acc}%")
    
    def set_linear_acc(self):
        """設定直線運動加速度"""
        acc = self.linear_acc_spin.value()
        success = self.robot_controller.set_acc_l(acc)
        if success:
            QMessageBox.information(self, "設定成功", f"直線運動加速度已設定為 {acc}%")
    
    def apply_speed_preset(self):
        """套用速度預設值"""
        preset_text = self.speed_preset_combo.currentText()
        
        if "20%" in preset_text:
            speed = 20
        elif "50%" in preset_text:
            speed = 50
        elif "80%" in preset_text:
            speed = 80
        elif "100%" in preset_text:
            speed = 100
        else:
            speed = 50
        
        # 設定所有速度參數
        self.global_speed_spin.setValue(speed)
        self.joint_speed_spin.setValue(speed)
        self.linear_speed_spin.setValue(speed)
        self.joint_acc_spin.setValue(speed)
        self.linear_acc_spin.setValue(speed)
        
        # 套用到機械臂
        success = True
        success &= self.robot_controller.set_speed_factor(speed)
        success &= self.robot_controller.set_speed_j(speed)
        success &= self.robot_controller.set_speed_l(speed)
        success &= self.robot_controller.set_acc_j(speed)
        success &= self.robot_controller.set_acc_l(speed)
        
        if success:
            QMessageBox.information(self, "設定成功", f"已套用 {preset_text} 到所有運動參數")
        else:
            QMessageBox.warning(self, "設定失敗", "部分速度參數設定失敗，請檢查連接狀態")

    # ==================== 運動控制 ====================
    
    def get_cartesian_values(self):
        """獲取笛卡爾座標值"""
        return (self.x_spin.value(), self.y_spin.value(), 
                self.z_spin.value(), self.r_spin.value())
                
    def get_joint_values(self):
        """獲取關節角度值"""
        return (self.j1_spin.value(), self.j2_spin.value(),
                self.j3_spin.value(), self.j4_spin.value())
    
    def movj(self):
        """關節運動"""
        x, y, z, r = self.get_cartesian_values()
        self.robot_controller.movj(x, y, z, r)
    
    def movl(self):
        """直線運動"""
        x, y, z, r = self.get_cartesian_values()
        speed = self.movl_speed_slider.value()
        self.robot_controller.movl(x, y, z, r, speed)
    
    def joint_movj(self):
        """關節座標運動"""
        j1, j2, j3, j4 = self.get_joint_values()
        self.robot_controller.joint_movj(j1, j2, j3, j4)
    
    def start_jog(self, command):
        """開始點動"""
        self.robot_controller.start_jog(command)
    
    def stop_jog(self):
        """停止點動"""
        self.robot_controller.stop_jog()

    # ==================== 夾爪控制 ====================
    
    def gripper_move_to_pos(self):
        """PGC夾爪移動到指定位置"""
        pos = self.gripper_pos_spin.value()
        self.robot_controller.send_gripper_command(3, pos)
    
    def gripper_set_force(self):
        """PGC夾爪設定力道"""
        force = self.gripper_force_spin.value()
        self.robot_controller.send_gripper_command(5, force)
    
    def update_gripper_display(self):
        """更新PGC夾爪狀態顯示"""
        status = self.robot_controller.get_gripper_status()
        if status:
            # 更新連接狀態
            if status['connection_status'] == 1:
                self.gripper_status_label.setText("已連接")
                self.gripper_status_label.setStyleSheet("color: green")
            else:
                self.gripper_status_label.setText("未連接")
                self.gripper_status_label.setStyleSheet("color: red")
                
            # 夾持狀態映射
            hold_status_map = {0: "運動中", 1: "到達", 2: "夾住", 3: "掉落"}
            self.gripper_hold_label.setText(hold_status_map.get(status['hold_status'], "未知"))
            
            self.gripper_pos_label.setText(str(status['current_pos']))
        else:
            self.gripper_status_label.setText("通訊錯誤")
            self.gripper_status_label.setStyleSheet("color: red")

    # ==================== 點位管理 ====================
    
    def sync_current_position(self):
        """將機械臂當前位置同步到輸入框"""
        if not self.is_connected:
            QMessageBox.warning(self, "警告", "請先連接機械臂")
            return
            
        current_pos = self.robot_controller.current_position
        
        # 檢查是否有有效的位置數據
        if (current_pos['cartesian']['x'] == 0 and 
            current_pos['cartesian']['y'] == 0 and 
            current_pos['cartesian']['z'] == 0):
            QMessageBox.warning(self, "警告", "尚未獲取到機械臂位置反饋，請稍後再試")
            return
            
        # 同步笛卡爾座標到輸入框
        cart = current_pos['cartesian']
        self.x_spin.setValue(cart['x'])
        self.y_spin.setValue(cart['y'])
        self.z_spin.setValue(cart['z'])
        self.r_spin.setValue(cart['r'])
        
        # 同步關節角度到輸入框
        joint = current_pos['joint']
        self.j1_spin.setValue(joint['j1'])
        self.j2_spin.setValue(joint['j2'])
        self.j3_spin.setValue(joint['j3'])
        self.j4_spin.setValue(joint['j4'])
        
        self.append_log("當前位置已同步到輸入框")
    
    def save_current_point(self):
        """保存當前點位"""
        if not self.is_connected:
            QMessageBox.warning(self, "警告", "請先連接機械臂")
            return
            
        name, ok = QInputDialog.getText(self, "保存點位", "輸入點位名稱:")
        if not ok or not name.strip():
            return
            
        success = self.robot_controller.save_current_point(name.strip())
        if success:
            self.refresh_points_list()
        else:
            QMessageBox.warning(self, "警告", "保存點位失敗")
    
    def edit_selected_point(self):
        """編輯選中的點位 - 修正版本"""
        current_row = self.points_list.currentRow()
        if current_row < 0 or current_row >= len(self.robot_controller.saved_points):
            QMessageBox.warning(self, "警告", "請先選擇要編輯的點位")
            return
            
        point = self.robot_controller.saved_points[current_row]
        
        # 確保點位數據完整性
        if 'id' not in point:
            point['id'] = current_row
        if 'name' not in point:
            point['name'] = f"Point_{current_row}"
        if 'cartesian' not in point:
            point['cartesian'] = {'x': 0, 'y': 0, 'z': 0, 'r': 0}
        if 'joint' not in point:
            point['joint'] = {'j1': 0, 'j2': 0, 'j3': 0, 'j4': 0}
        
        dialog = PointEditDialog(point, self)
        
        if dialog.exec_() == QDialog.Accepted:
            new_data = dialog.get_data()
            
            # 確保新數據包含ID
            new_data['id'] = point['id']
            
            # 驗證位置變化
            if not PointDataValidator.validate_position_change(point, new_data):
                reply = QMessageBox.question(self, "安全警告", 
                    "檢測到大幅度位置變化或正負號改變，這可能導致機械臂碰撞。\n是否繼續保存？",
                    QMessageBox.Yes | QMessageBox.No)
                if reply == QMessageBox.No:
                    return
                    
            # 更新點位
            success = self.robot_controller.update_point(current_row, new_data)
            if success:
                self.refresh_points_list()
            else:
                QMessageBox.warning(self, "錯誤", "點位更新失敗")
    
    def delete_selected_point(self):
        """刪除選中的點位"""
        current_row = self.points_list.currentRow()
        if current_row < 0 or current_row >= len(self.robot_controller.saved_points):
            QMessageBox.warning(self, "警告", "請先選擇要刪除的點位")
            return
            
        point = self.robot_controller.saved_points[current_row]
        reply = QMessageBox.question(self, "確認刪除", 
            f"確定要刪除點位 '{point['name']}' 嗎？",
            QMessageBox.Yes | QMessageBox.No)
            
        if reply == QMessageBox.Yes:
            self.robot_controller.delete_point(current_row)
            self.refresh_points_list()
    
    def move_to_selected_point_with_dialog(self):
        """雙擊點位列表時彈出運動類型選擇對話框"""
        current_row = self.points_list.currentRow()
        if current_row < 0 or current_row >= len(self.robot_controller.saved_points):
            return
        
        motion_type, ok = QInputDialog.getItem(self, "選擇運動類型", 
            "請選擇運動方式:", 
            [
                "直線運動(MovL) - 可能遇到手勢切換問題",
                "關節運動(MovJ) - 避免手勢切換問題", 
                "關節座標運動(JointMovJ) - 使用儲存關節角度"
            ], 
            1, False)
        
        if ok:
            speed = self.point_speed_slider.value()
            self.robot_controller.execute_point_movement(current_row, motion_type, speed)
    
    def move_to_selected_point(self, use_preset=False):
        """移動到選中的點位 - 增強診斷版本"""
        current_row = self.points_list.currentRow()
        if current_row < 0 or current_row >= len(self.robot_controller.saved_points):
            QMessageBox.warning(self, "警告", "請先選擇要移動的點位")
            return
            
        if not self.is_connected:
            QMessageBox.warning(self, "警告", "請先連接機械臂")
            return
            
        if not self.is_enabled:
            QMessageBox.warning(self, "警告", "請先使能機械臂")
            return
        
        # 檢查機械臂是否準備好運動
        if not self.robot_controller.check_robot_ready_for_movement():
            reply = QMessageBox.question(self, "機械臂狀態檢查", 
                "機械臂狀態檢查發現問題。是否要進行詳細診斷？",
                QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.robot_controller.diagnose_movement_issue()
            return
        
        if use_preset:
            motion_type = self.motion_type_combo.currentText()
        else:
            motion_type, ok = QInputDialog.getItem(self, "選擇運動類型", 
                "請選擇運動方式:", 
                [
                    "直線運動(MovL) - 可能遇到手勢切換問題",
                    "關節運動(MovJ) - 避免手勢切換問題", 
                    "關節座標運動(JointMovJ) - 使用儲存關節角度"
                ], 
                1, False)
            
            if not ok:
                return
        
        speed = self.point_speed_slider.value()
        self.append_log(f"開始執行點位移動 - 速度: {speed}%, 類型: {motion_type}")
        
        success = self.robot_controller.execute_point_movement(current_row, motion_type, speed)
        if not success:
            reply = QMessageBox.question(self, "移動失敗", 
                "點位移動失敗。是否要進行運動診斷？",
                QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.robot_controller.diagnose_movement_issue()
        else:
            self.append_log("點位移動指令執行完成")
    
    def refresh_points_list(self):
        """刷新點位列表顯示 - 修正版本"""
        self.points_list.clear()
        
        # 修正所有點位數據的完整性
        for i, point in enumerate(self.robot_controller.saved_points):
            # 檢查並修正缺少的id欄位
            if 'id' not in point:
                point['id'] = i
            
            # 檢查並修正缺少的name欄位
            if 'name' not in point:
                point['name'] = f"Point_{i}"
            
            # 檢查cartesian數據
            if 'cartesian' not in point:
                point['cartesian'] = {'x': 0, 'y': 0, 'z': 0, 'r': 0}
            
            # 確保cartesian字典包含所有必要的鍵
            cartesian = point['cartesian']
            required_keys = ['x', 'y', 'z', 'r']
            for key in required_keys:
                if key not in cartesian:
                    cartesian[key] = 0.0
            
            # 檢查joint數據
            if 'joint' not in point:
                point['joint'] = {'j1': 0, 'j2': 0, 'j3': 0, 'j4': 0}
            
            # 確保joint字典包含所有必要的鍵
            joint = point['joint']
            required_joint_keys = ['j1', 'j2', 'j3', 'j4']
            for key in required_joint_keys:
                if key not in joint:
                    joint[key] = 0.0
        
        # 保存修正後的數據
        self.robot_controller.save_points()
        
        # 重新生成列表項目
        for i, point in enumerate(self.robot_controller.saved_points):
            cartesian = point['cartesian']
            item_text = f"[{point['id']}] {point['name']} - " \
                       f"X:{cartesian.get('x', 0):.2f} " \
                       f"Y:{cartesian.get('y', 0):.2f} " \
                       f"Z:{cartesian.get('z', 0):.2f}"
            self.points_list.addItem(item_text)

    # ==================== 工具函數 ====================
    
    def clear_error_info(self):
        """清除錯誤信息"""
        self.error_text.clear()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotUI()
    window.show()
    sys.exit(app.exec_())