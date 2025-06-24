# DobotMotion技術文檔

## 第一章 開發架構與規格訂定

### 1.1 專案概述

DobotMotion是基於Dobot M1Pro四軸工業機械臂的運動控制模組，採用模組化架構設計，支援狀態機交握協議與外部設備整合。系統將實現機械臂運動控制、外部模組通訊、流程管理等核心功能，為自動化生產線提供完整的運動控制解決方案。

### 1.2 系統架構設計

#### 1.2.1 模組分層架構

```
┌─────────────────────────────────────────────────────────────┐
│                    Web控制介面                               │
│                   Dobot_app.py                             │
│                 (Flask + SocketIO)                         │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                    流程控制層                               │
│  Dobot_flow1.py  │  Dobot_flow2.py  │  Dobot_flow3.py     │
│      (流程1)     │      (流程2)     │      (流程3)       │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                    核心控制層                               │
│                   Dobot_main.py                            │
│         (狀態機管理 + 外部模組整合 + 點位管理)              │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                    硬體抽象層                               │
│                   dobot_api.py                             │
│               (TCP/IP通訊 + 指令封裝)                      │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────┐
│                    硬體設備層                               │
│  Dobot M1Pro  │   CCD1視覺   │   CCD3角度   │   PGC夾爪   │
│  (192.168.1.6)│ (192.168.1.8)│(192.168.1.10)│   (COM5)   │
└─────────────────────────────────────────────────────────────┘
```

#### 1.2.2 檔案結構規範

```
M1Pro/
├── dobot_api.py              # 硬體抽象層 - Dobot API封裝
├── Dobot_main.py             # 核心控制層 - 狀態機+模組整合
├── Dobot_flow1.py            # 流程控制層 - 動作流程1
├── Dobot_flow2.py            # 流程控制層 - 動作流程2  
├── Dobot_flow3.py            # 流程控制層 - 動作流程3
├── Dobot_app.py              # Web控制介面
├── templates/
│   └── index.html            # Web UI界面
├── saved_points/
│   └── robot_points.json     # 點位數據檔案
└── config/
    └── dobot_config.json     # 系統配置檔案
```

### 1.3 核心類別設計

#### 1.3.1 DobotM1Pro核心控制類

```python
class DobotM1Pro:
    """Dobot M1Pro機械臂核心控制類"""
    
    def __init__(self, ip="192.168.1.6"):
        self.dashboard_api = DobotApiDashboard(ip, 29999)
        self.move_api = DobotApiMove(ip, 30003)
        self.points_manager = PointsManager()
        self.status_monitor = StatusMonitor()
        self.is_connected = False
        self.global_speed = 50
        
    # 基礎控制方法
    def initialize(self) -> bool
    def disconnect(self) -> bool
    def emergency_stop(self) -> bool
    def clear_error(self) -> bool
    
    # 速度控制
    def set_global_speed(self, speed: int) -> bool
    def set_joint_speed(self, speed: int) -> bool
    def set_linear_speed(self, speed: int) -> bool
    
    # 運動控制 (點位名稱版本)
    def MovJ(self, point_name: str, **kwargs) -> bool
    def MovL(self, point_name: str, **kwargs) -> bool
    def MovJ_coord(self, x: float, y: float, z: float, r: float, **kwargs) -> bool
    def MovL_coord(self, x: float, y: float, z: float, r: float, **kwargs) -> bool
    
    # 狀態查詢
    def get_robot_mode(self) -> int
    def get_current_pose(self) -> dict
    def get_current_joints(self) -> dict
    def is_ready(self) -> bool
    def is_running(self) -> bool
    
    # IO控制
    def set_do(self, index: int, status: int) -> bool
    def get_di(self, index: int) -> int
    def set_tool_do(self, index: int, status: int) -> bool
    def get_tool_di(self, index: int) -> int
    
    # 同步控制
    def sync(self) -> bool
    def wait(self, milliseconds: int) -> bool
```

#### 1.3.2 外部模組整合類

```python
class PGCGripperController:
    """PGC夾爪控制器"""
    
    def __init__(self, modbus_ip="127.0.0.1", modbus_port=502):
        self.modbus_client = ModbusTcpClient(modbus_ip, modbus_port)
        self.base_address = 520  # PGC指令寄存器基地址
        self.status_address = 500  # PGC狀態寄存器基地址
        
    def initialize(self) -> bool
    def open_to_position(self, position: int) -> bool
    def close_fast(self) -> bool
    def check_reached(self) -> bool
    def get_current_position(self) -> int
    def get_grip_status(self) -> int

class CCD1VisionController:
    """CCD1視覺檢測控制器"""
    
    def __init__(self, modbus_ip="127.0.0.1", modbus_port=502):
        self.modbus_client = ModbusTcpClient(modbus_ip, modbus_port)
        self.base_address = 200  # CCD1控制寄存器基地址
        
    def initialize(self) -> bool
    def capture_and_detect(self) -> bool
    def get_object_center_world(self, object_index: int) -> list
    def get_object_center_pixel(self, object_index: int) -> list
    def get_detection_count(self) -> int
    def is_ready(self) -> bool

class CCD3AngleController:
    """CCD3角度檢測控制器"""
    
    def __init__(self, modbus_ip="127.0.0.1", modbus_port=502):
        self.modbus_client = ModbusTcpClient(modbus_ip, modbus_port)
        self.base_address = 800  # CCD3控制寄存器基地址
        
    def initialize(self) -> bool
    def detect_angle(self) -> bool
    def get_detected_angle(self) -> float
    def get_object_center(self) -> tuple
    def is_ready(self) -> bool
```

#### 1.3.3 點位管理類

```python
class PointsManager:
    """點位管理類"""
    
    def __init__(self, points_file="saved_points/robot_points.json"):
        self.points_file = points_file
        self.points_data = {}
        
    def load_points(self) -> bool
    def get_point(self, point_name: str) -> dict
    def add_point(self, point_name: str, point_data: dict) -> bool
    def update_point(self, point_name: str, point_data: dict) -> bool
    def delete_point(self, point_name: str) -> bool
    def save_points(self) -> bool
    def list_points(self) -> list
```

### 1.4 狀態機交握協議設計

#### 1.4.1 狀態機架構

```python
class DobotStateMachine:
    """Dobot狀態機管理"""
    
    # 狀態定義
    STATE_IDLE = 0          # 空閒狀態
    STATE_RUNNING = 1       # 運行狀態  
    STATE_PAUSED = 2        # 暫停狀態
    STATE_ERROR = 3         # 錯誤狀態
    STATE_EMERGENCY = 4     # 緊急停止狀態
    
    # 流程狀態
    FLOW_NONE = 0           # 無流程
    FLOW_1 = 1              # 流程1
    FLOW_2 = 2              # 流程2
    FLOW_3 = 3              # 流程3
    
    def __init__(self, modbus_base_address=400):
        self.base_address = modbus_base_address
        self.current_state = self.STATE_IDLE
        self.current_flow = self.FLOW_NONE
        self.modbus_client = None
        
    def set_modbus_connection(self, client)
    def update_status_to_plc(self)
    def read_control_from_plc(self) -> dict
    def set_state(self, new_state: int)
    def set_flow(self, flow_id: int)
    def is_ready_for_command(self) -> bool
```

#### 1.4.2 Modbus寄存器映射 (基地址400)

| 地址 | 功能 | 數值定義 | 讀寫 |
|------|------|----------|------|
| 400 | 控制指令 | 0=清空, 1=流程1, 2=流程2, 3=流程3, 99=緊急停止 | R |
| 401 | 機械臂狀態 | 0=空閒, 1=運行, 2=暫停, 3=錯誤, 4=緊急停止 | W |
| 402 | 當前流程ID | 0=無, 1=流程1, 2=流程2, 3=流程3 | W |
| 403 | 流程執行進度 | 0-100百分比 | W |
| 404 | 錯誤代碼 | 具體錯誤編號 | W |
| 405 | 機械臂模式 | RobotMode()返回值 | W |
| 406-409 | 當前位置 | X, Y, Z, R座標 | W |
| 410-413 | 當前關節角度 | J1, J2, J3, J4角度 | W |
| 414 | DI狀態 | 數位輸入狀態 | W |
| 415 | DO狀態 | 數位輸出狀態 | W |
| 416 | 操作計數器 | 累積操作次數 | W |
| 417 | 錯誤計數器 | 累積錯誤次數 | W |
| 418 | 運行時間 | 累積運行時間(分鐘) | W |
| 419 | 保留 | 未來擴展 | - |

### 1.5 流程控制層設計

#### 1.5.1 基礎流程類

```python
class BaseFlow:
    """基礎流程類"""
    
    def __init__(self, robot: DobotM1Pro, gripper: PGCGripperController, 
                 ccd1: CCD1VisionController, ccd3: CCD3AngleController):
        self.robot = robot
        self.gripper = gripper
        self.ccd1 = ccd1
        self.ccd3 = ccd3
        self.flow_id = 0
        self.current_step = 0
        self.total_steps = 0
        self.is_running = False
        self.last_error = ""
        
    def start(self) -> bool
    def pause(self) -> bool
    def resume(self) -> bool
    def stop(self) -> bool
    def reset(self) -> bool
    def get_progress(self) -> int
    def get_status(self) -> dict
    
    # 抽象方法 - 子類實現
    def execute(self) -> bool
    def handle_error(self, error_msg: str) -> bool
```

#### 1.5.2 流程實現範例 (Dobot_flow1.py)

```python
class Flow1(BaseFlow):
    """流程1實現 - VP震動盤視覺抓取流程"""
    
    def __init__(self, *args):
        super().__init__(*args)
        self.flow_id = 1
        self.total_steps = 12
        
    def execute(self) -> bool:
        try:
            # 步驟1: 初始化檢查
            self.current_step = 1
            if not self._initialize_check():
                return False
                
            # 步驟2: 機械臂回到待機點
            self.current_step = 2
            self.robot.set_global_speed(50)
            if not self.robot.MovL("standby"):
                return self._handle_move_error("移動到待機點失敗")
                
            # 步驟3: 夾爪關閉
            self.current_step = 3
            if not self.gripper.close_fast():
                return self._handle_gripper_error("夾爪關閉失敗")
                
            # 步驟4-6: 測試動作序列
            self.current_step = 4
            test_points = ["Rotate_V2", "Rotate_top", "Rotate_down"]
            for point in test_points:
                if not self.robot.MovL(point):
                    return self._handle_move_error(f"移動到{point}失敗")
                self.current_step += 1
                
            # 步驟7: 夾爪撐開測試
            self.current_step = 7
            if not self._gripper_open_and_check(0):
                return self._handle_gripper_error("夾爪撐開測試失敗")
                
            # 步驟8-9: 回到待機點再到VP上方
            self.current_step = 8
            if not self.robot.MovL("Rotate_top"):
                return self._handle_move_error("移動失敗")
            self.current_step = 9
            if not self.robot.MovL("standby"):
                return self._handle_move_error("回到待機點失敗")
            if not self.robot.MovL("VP_TOPSIDE"):
                return self._handle_move_error("移動到VP上方失敗")
                
            # 步驟10: CCD1視覺檢測
            self.current_step = 10
            if not self._vision_detection_and_pickup():
                return self._handle_vision_error("視覺檢測失敗")
                
            # 步驟11-12: 完成動作
            self.current_step = 11
            if not self.robot.MovL("VP_TOPSIDE"):
                return self._handle_move_error("移動失敗")
            self.current_step = 12
            if not self.robot.MovL("standby"):
                return self._handle_move_error("回到待機點失敗")
                
            return True
            
        except Exception as e:
            return self.handle_error(f"流程執行異常: {str(e)}")
    
    def _gripper_open_and_check(self, position: int) -> bool:
        """夾爪撐開並檢測"""
        if not self.gripper.open_to_position(position):
            return False
            
        # 等待並檢測
        timeout = 10.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.gripper.check_reached():
                return True
            time.sleep(0.1)
        return False
    
    def _vision_detection_and_pickup(self) -> bool:
        """視覺檢測並抓取"""
        # 觸發CCD1檢測
        if not self.ccd1.capture_and_detect():
            return False
            
        # 獲取檢測結果
        if self.ccd1.get_detection_count() == 0:
            self.last_error = "未檢測到物體"
            return False
            
        # 獲取第一個物體的世界座標
        point = self.ccd1.get_object_center_world(1)
        if not point or len(point) < 2:
            self.last_error = "獲取物體座標失敗"
            return False
            
        # 設定Z軸高度
        point[2] = 238.86  # CCD1檢測高度
        
        # 移動到物體上方
        if not self.robot.MovL_coord(point[0], point[1], point[2], 0):
            return False
            
        # 下降到抓取高度
        point[2] = 137.52  # 插入高度
        if not self.robot.MovL_coord(point[0], point[1], point[2], 0):
            return False
            
        # 夾爪撐開抓取
        return self._gripper_open_and_check(370)
    
    def _handle_move_error(self, error_msg: str) -> bool:
        """處理移動錯誤"""
        self.last_error = error_msg
        self.robot.emergency_stop()
        return False
        
    def _handle_gripper_error(self, error_msg: str) -> bool:
        """處理夾爪錯誤"""
        self.last_error = error_msg
        # 可以添加夾爪重試邏輯
        return False
        
    def _handle_vision_error(self, error_msg: str) -> bool:
        """處理視覺錯誤"""
        self.last_error = error_msg
        # 可以添加視覺重檢邏輯
        return False
```

### 1.6 系統整合設計

#### 1.6.1 Dobot_main.py架構

```python
class DobotMotionController:
    """Dobot運動控制主控制器"""
    
    def __init__(self):
        # 初始化各子系統
        self.robot = DobotM1Pro()
        self.gripper = PGCGripperController()
        self.ccd1 = CCD1VisionController()
        self.ccd3 = CCD3AngleController()
        self.state_machine = DobotStateMachine()
        
        # 初始化流程控制器
        self.flows = {
            1: Flow1(self.robot, self.gripper, self.ccd1, self.ccd3),
            2: Flow2(self.robot, self.gripper, self.ccd1, self.ccd3),
            3: Flow3(self.robot, self.gripper, self.ccd1, self.ccd3)
        }
        
        self.current_flow = None
        self.is_running = False
        
    def initialize_system(self) -> bool
    def connect_all_devices(self) -> bool
    def start_handshake_sync(self)
    def stop_handshake_sync(self)
    def execute_flow(self, flow_id: int) -> bool
    def emergency_stop_all(self) -> bool
    def get_system_status(self) -> dict
    
    def _handshake_loop(self):
        """狀態機交握主循環"""
        while self.is_running:
            try:
                # 讀取PLC控制指令
                control_data = self.state_machine.read_control_from_plc()
                
                # 處理控制指令
                if control_data['command'] != 0:
                    self._handle_plc_command(control_data)
                
                # 更新狀態到PLC
                self.state_machine.update_status_to_plc()
                
                time.sleep(0.05)  # 50ms循環
                
            except Exception as e:
                print(f"狀態機循環錯誤: {e}")
                time.sleep(1)
```

### 1.7 Web介面設計

#### 1.7.1 API設計規範

```python
# Dobot_app.py API路由設計
@app.route('/api/system/status', methods=['GET'])
def get_system_status()

@app.route('/api/system/initialize', methods=['POST'])  
def initialize_system()

@app.route('/api/robot/connect', methods=['POST'])
def connect_robot()

@app.route('/api/robot/emergency_stop', methods=['POST'])
def emergency_stop()

@app.route('/api/flow/start/<int:flow_id>', methods=['POST'])
def start_flow(flow_id)

@app.route('/api/flow/stop', methods=['POST'])
def stop_flow()

@app.route('/api/points/list', methods=['GET'])
def list_points()

@app.route('/api/points/add', methods=['POST'])
def add_point()

@app.route('/api/manual/move_to_point', methods=['POST'])
def manual_move_to_point()

@app.route('/api/manual/jog', methods=['POST'])
def manual_jog()
```

### 1.8 配置管理規範

#### 1.8.1 配置檔案格式 (dobot_config.json)

```json
{
    "robot": {
        "ip": "192.168.1.6",
        "dashboard_port": 29999,
        "move_port": 30003,
        "default_speed": 50,
        "default_acceleration": 50,
        "enable_collision_detection": true,
        "collision_level": 3
    },
    "modbus": {
        "server_ip": "127.0.0.1",
        "server_port": 502,
        "robot_base_address": 400,
        "timeout": 3.0
    },
    "gripper": {
        "type": "PGC",
        "base_address": 520,
        "status_address": 500,
        "default_force": 50,
        "default_speed": 80
    },
    "vision": {
        "ccd1_base_address": 200,
        "ccd3_base_address": 800,
        "detection_timeout": 10.0
    },
    "flows": {
        "flow1_enabled": true,
        "flow2_enabled": true,
        "flow3_enabled": false
    },
    "safety": {
        "enable_emergency_stop": true,
        "max_error_count": 5,
        "auto_recovery": false
    }
}
```

### 1.9 開發標準與規範

#### 1.9.1 程式碼規範
- 使用Type Hints進行類型標註
- 所有公開方法必須有docstring文檔
- 錯誤處理必須回傳明確的狀態碼和錯誤訊息
- 關鍵操作必須有日誌記錄
- 資源使用完畢後必須正確釋放

#### 1.9.2 測試規範
- 每個流程必須有單元測試
- 外部設備連接必須有模擬測試
- 狀態機轉換必須有完整測試覆蓋
- 異常情況必須有對應測試案例

#### 1.9.3 部署規範
- 所有配置參數必須可外部配置
- 系統啟動順序必須明確定義
- 異常情況下必須能安全停止
- 系統狀態必須可透過Web介面監控

### 1.10 擴展性設計

#### 1.10.1 模組擴展
- 支援新增更多流程模組 (Flow4, Flow5...)
- 支援新增更多外部設備控制器
- 支援自定義點位操作
- 支援參數化流程配置

#### 1.10.2 功能擴展
- 軌跡記錄與回放功能
- 多機械臂協調控制
- 視覺引導高精度定位
- 力控制與感測器整合

### 1.11 開發里程碑

#### 階段1: 基礎架構 (Week 1-2)
- 完成dobot_api.py基礎封裝
- 實現DobotM1Pro核心控制類
- 完成點位管理系統
- 建立基本Web介面

#### 階段2: 狀態機整合 (Week 3-4)  
- 實現狀態機交握協議
- 完成外部模組整合類
- 實現Modbus通訊功能
- 完成系統初始化流程

#### 階段3: 流程實現 (Week 5-6)
- 完成Flow1基礎流程
- 實現錯誤處理機制
- 完成Web控制介面
- 進行整合測試

#### 階段4: 優化完善 (Week 7-8)
- 效能優化與穩定性提升
- 完善錯誤處理與異常恢復
- 補齊文檔與測試案例
- 部署與驗收測試

---

## 總結

本開發架構採用分層模組化設計，確保系統的可維護性、可擴展性與可靠性。透過標準化的介面設計和嚴格的開發規範，能夠快速實現複雜的機械臂自動化流程，並為未來的功能擴展奠定堅實基礎。