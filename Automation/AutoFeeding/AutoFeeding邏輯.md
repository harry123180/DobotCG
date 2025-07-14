# AutoFeeding邏輯 - CG版本

## 架構概述

CG版本AutoFeeding獨立模組實現自動入料檢測功能，確保保護區域內始終有CG_F物件可用。採用獨立模組設計，避免執行緒管理複雜性，提供穩定的長時間運行能力。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> AutoFeeding_main.py (TCP Client)
                    |
                    |-- 檢測 --> CG視覺模組 (基地址200)
                    |-- 控制 --> VP震動盤 (基地址300)
                    |-- 送料 --> Flow4直振 (地址448)
                    |-- 監控 --> 當前執行Flow (地址1201)
```

## 實現組件

### AutoFeeding_main.py - 主模組
- TCP Client連接主服務器 (127.0.0.1:502)
- 寄存器映射基地址: 900-999
- CG_F物件檢測與保護區域判斷
- VP震動盤控制與Flow4直振送料
- Flow1執行狀態監控與自動暫停機制
- 獨立模組設計，無執行緒依賴

### ProtectionZone - CG保護區域判斷
- CG專案保護區域座標: (-86,-369.51) 到 (8.07,-244.64)
- 射線法四邊形內點判斷算法
- 保證只有保護區域內的CG_F才會被使用

### 配置檔案系統
- autofeeding_config.json自動生成
- 支援檢測週期、超時時間、VP參數等配置
- 執行檔同層目錄配置檔案管理

## 寄存器映射 (基地址900)

### 狀態寄存器 (900-919)
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 900 | 模組狀態 | 0=停止, 1=運行, 2=Flow1暫停, 3=檢測中, 4=VP震動, 5=VP清空, 6=錯誤 | AutoFeedingStatus |
| 901 | 週期計數 | 累積檢測週期數 | 週期統計 |
| 902 | CG_F找到次數 | 累積找到CG_F次數 | 成功統計 |
| 903 | Flow4觸發次數 | 累積Flow4送料次數 | 送料統計 |
| 904 | VP震動次數 | 累積VP震動次數 | 震動統計 |
| 905-906 | 保留 | - | 未來擴展 |
| 907 | 錯誤代碼 | 錯誤類型編碼 | 故障診斷 |
| 908 | 操作狀態 | 0=空閒, 1=CG檢測, 2=VP控制, 3=Flow4觸發 | OperationStatus |
| 909 | Flow1監控狀態 | 0=未執行, 1=執行中 | Flow1狀態監控 |

### CG_F狀態寄存器 (940-959) - 簡化交握
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 940 | CG_F可用標誌 | 0=無, 1=有 | 保護區內CG_F可用性 |
| 941 | CG_F座標X高位 | 32位X座標高16位 | 世界座標X (×100精度) |
| 942 | CG_F座標X低位 | 32位X座標低16位 | 世界座標X (×100精度) |
| 943 | CG_F座標Y高位 | 32位Y座標高16位 | 世界座標Y (×100精度) |
| 944 | CG_F座標Y低位 | 32位Y座標低16位 | 世界座標Y (×100精度) |
| 945 | 座標已讀取標誌 | 0=未讀, 1=已讀 | Flow1設置此標誌表示已讀取 |
| 946-947 | 保留 | - | 未來擴展 |

### 配置參數寄存器 (960-979)
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 960 | 檢測週期間隔 | 毫秒單位 | cycle_interval×1000 |
| 961 | CG檢測超時 | 毫秒單位 | cg_timeout×1000 |
| 962 | VP震動強度 | 0-100 | spread_strength |
| 963 | VP震動頻率 | Hz | spread_frequency |
| 964 | VP震動持續時間 | 毫秒單位 | spread_duration×1000 |
| 965-979 | 保留參數 | - | 未來擴展 |

## 核心邏輯流程

### 主循環邏輯
```python
def main_loop(self):
    """主循環 - 獨立模組設計"""
    while True:
        # 1. 檢查Modbus連接狀態
        if not self.connected:
            self.connect()  # 自動重連
            continue
        
        # 2. 監控當前執行Flow狀態 (1201地址)
        self.check_flow1_status()
        
        # 3. 檢查座標是否被Flow1讀取
        if self.cg_f_available:
            self.check_coords_taken()
        
        # 4. 更新狀態寄存器
        self.update_status_registers()
        
        # 5. 執行入料檢測週期 (條件執行)
        if self.running and not self.flow1_active:
            self.feeding_cycle()
        
        # 6. 適應性休眠控制
        time.sleep(self.cycle_interval)
```

### Flow1狀態監控機制
```python
def check_flow1_status(self) -> bool:
    """監控當前執行Flow狀態 - 修改為監控1201"""
    current_motion_flow = self.read_register(1201)
    flow1_now_active = (current_motion_flow == 1)
    
    if flow1_now_active != self.flow1_active:
        self.flow1_active = flow1_now_active
        if self.flow1_active:
            print("[AutoFeeding] 檢測到Flow1正在執行，暫停檢測")
            self.status = AutoFeedingStatus.FLOW1_PAUSED
        else:
            print("[AutoFeeding] Flow1執行完成，恢復檢測")
            self.status = AutoFeedingStatus.RUNNING
            self.check_coords_taken()  # 檢查座標是否被讀取
```

### 入料檢測週期邏輯
```python
def feeding_cycle(self) -> bool:
    """執行一次入料檢測週期"""
    # 1. 模組狀態檢查
    if not self.check_modules_status():
        return False
    
    # 2. 觸發CG檢測
    detection_result = self.trigger_cg_detection()
    if not detection_result.operation_success:
        return False
    
    # 3. 尋找保護區域內的CG_F
    target_coords = self.find_cg_f_in_protection_zone(detection_result)
    
    # 4. 根據檢測結果執行對應動作
    if target_coords:
        # 找到CG_F：設置可用狀態
        self.set_cg_f_available(target_coords)
    elif detection_result.total_detections < 4:
        # 料件不足：觸發Flow4送料
        self.trigger_flow4_feeding()
        self.flow4_consecutive_count += 1
    else:
        # 料件充足但無正面：VP震動重檢
        self.trigger_vp_vibration_and_redetect()
    
    return True
```

## CG檢測與保護區域判斷

### CG檢測結果數據結構
```python
@dataclass
class CGDetectionResult:
    """CG檢測結果"""
    cg_f_count: int = 0              # CG_F物件數量
    total_detections: int = 0        # 總檢測物件數
    cg_f_world_coords: List[Tuple[float, float]] = None  # CG_F世界座標列表
    operation_success: bool = False  # 檢測操作是否成功
```

### CG保護區域判斷算法
```python
class ProtectionZone:
    """CG專案保護區域判斷"""
    
    @staticmethod
    def is_point_in_quad(x_a: float, y_a: float) -> bool:
        """判斷點是否在CG保護區域四邊形內"""
        # CG專案保護區域座標
        points = [
            (-86, -369.51),       # x1, y1
            (-112.82, -244.63),   # x2, y2
            (8.07, -244.64),      # x3, y3
            (8.06, -369.52)       # x4, y4
        ]
        
        # 射線法四邊形內點判斷
        return point_in_polygon(x_a, y_a, sorted_points)
```

### CG檢測觸發流程
```python
def trigger_cg_detection(self) -> CGDetectionResult:
    """觸發CG檢測"""
    # 1. 發送拍照+檢測指令 (200地址寫入16)
    self.write_register(200, 16)
    
    # 2. 等待檢測完成 (203,204,205寄存器)
    while timeout_not_reached:
        capture_complete = self.read_register(203)
        detect_complete = self.read_register(204)
        operation_success = self.read_register(205)
        
        if all([capture_complete, detect_complete, operation_success]):
            break
    
    # 3. 讀取檢測結果
    result.cg_f_count = self.read_register(240)
    result.total_detections = self.read_register(243)
    
    # 4. 提取CG_F世界座標 (261-276寄存器)
    for i in range(result.cg_f_count):
        base_addr = 261 + (i * 4)
        world_x = self.read_32bit_register(base_addr, base_addr + 1)
        world_y = self.read_32bit_register(base_addr + 2, base_addr + 3)
        result.cg_f_world_coords.append((world_x, world_y))
    
    return result
```

## VP震動盤控制

### VP震動參數配置
```python
"vp_params": {
    "spread_action_code": 11,    # 散開動作代碼
    "spread_strength": 60,       # 震動強度 (0-100)
    "spread_frequency": 50,      # 震動頻率 (Hz)
    "spread_duration": 0.3,      # 持續時間 (秒)
    "stop_command_code": 3,      # 停止指令代碼
    "stop_delay": 0.1           # 停止後延遲 (秒)
}
```

### VP震動控制流程
```python
def trigger_vp_vibration(self) -> bool:
    """觸發VP震動"""
    # 1. 啟動震動 (320-324寄存器)
    self.write_register(320, 5)  # execute_action
    self.write_register(321, self.config['vp_params']['spread_action_code'])
    self.write_register(322, self.config['vp_params']['spread_strength'])
    self.write_register(323, self.config['vp_params']['spread_frequency'])
    self.write_register(324, int(time.time()) % 65535)  # command_id
    
    # 2. 等待震動持續時間
    time.sleep(self.config['vp_params']['spread_duration'])
    
    # 3. 停止震動
    return self.stop_vp_vibration()

def stop_vp_vibration(self) -> bool:
    """停止VP震動"""
    self.write_register(320, self.config['vp_params']['stop_command_code'])
    self.write_register(321, 0)
    self.write_register(322, 0)
    self.write_register(323, 0)
    self.write_register(324, 99)  # emergency stop id
    
    time.sleep(self.config['vp_params']['stop_delay'])
    return True
```

## Flow4直振送料控制

### Flow4送料參數配置
```python
"flow4_params": {
    "pulse_duration": 0.1,       # 脈衝持續時間 (秒)
    "pulse_interval": 0.05       # 脈衝間隔 (秒)
}
```

### Flow4觸發流程
```python
def trigger_flow4_feeding(self) -> bool:
    """觸發Flow4送料"""
    # 1. 觸發脈衝 (448地址)
    if not self.write_register(448, 1):
        return False
    
    # 2. 脈衝持續時間
    time.sleep(self.config['flow4_params']['pulse_duration'])
    
    # 3. 結束脈衝
    if not self.write_register(448, 0):
        return False
    
    return True
```

### 連續直振監控
```python
# 連續直振限制檢查
if self.flow4_consecutive_count >= self.config['autofeeding']['flow4_consecutive_limit']:
    print("[AutoFeeding] 達到連續直振限制，需要VP清空")
    # 可擴展VP清空流程
```

## 座標交握機制

### CG_F座標設置
```python
def set_cg_f_available(self, coords: Tuple[float, float]):
    """設置CG_F可用狀態"""
    self.cg_f_available = True
    self.cg_f_coords = coords
    self.cg_f_taken = False
    
    # 立即更新寄存器讓Flow1可以讀取
    self.write_register(940, 1)  # CG_F可用標誌
    self.write_32bit_register(941, 942, coords[0])  # X座標
    self.write_32bit_register(943, 944, coords[1])  # Y座標
```

### 座標讀取檢查
```python
def check_coords_taken(self):
    """檢查座標是否被Flow1讀取"""
    coords_taken = self.read_register(945)  # Flow1設置此標誌
    if coords_taken == 1:
        # 清除CG_F狀態，繼續檢測新的
        self.cg_f_available = False
        self.cg_f_coords = (0.0, 0.0)
        
        # 清除相關寄存器
        self.write_register(940, 0)  # CG_F可用標誌
        self.write_register(945, 0)  # 座標已讀取標誌
        for addr in [941, 942, 943, 944]:
            self.write_register(addr, 0)
```

## 32位座標處理

### 座標編碼算法
```python
def write_32bit_register(self, high_addr: int, low_addr: int, value: float) -> bool:
    """寫入32位世界座標"""
    # 1. 轉換為整數形式(×100)
    value_int = int(value * 100)
    
    # 2. 處理負數(補碼)
    if value_int < 0:
        value_int = value_int + 4294967296  # 2^32
    
    # 3. 分解為高低位
    high_val = (value_int >> 16) & 0xFFFF
    low_val = value_int & 0xFFFF
    
    # 4. 寫入寄存器
    success = True
    success &= self.write_register(high_addr, high_val)
    success &= self.write_register(low_addr, low_val)
    
    return success
```

### 座標解碼算法
```python
def read_32bit_register(self, high_addr: int, low_addr: int) -> float:
    """讀取32位世界座標並轉換為實際值"""
    high_val = self.read_register(high_addr)
    low_val = self.read_register(low_addr)
    
    # 1. 合併32位值
    combined = (high_val << 16) + low_val
    
    # 2. 處理補碼(負數)
    if combined >= 2147483648:  # 2^31
        combined = combined - 4294967296  # 2^32
    
    # 3. 轉換為毫米(除以100)
    return combined / 100.0
```

## 模組狀態檢查

### CG模組狀態檢查
```python
def check_modules_status(self) -> bool:
    """檢查CG、VP模組狀態"""
    # 檢查CG狀態 (201寄存器)
    cg_status = self.read_register(201)
    cg_ready = bool(cg_status & 0x01)      # bit0=Ready
    cg_alarm = bool(cg_status & 0x04)      # bit2=Alarm
    cg_initialized = bool(cg_status & 0x08) # bit3=Initialized
    
    if cg_alarm or not cg_initialized or not cg_ready:
        return False
    
    # 檢查VP狀態 (300-301寄存器)
    vp_status = self.read_register(300)     # module_status
    vp_connected = self.read_register(301)  # device_connection
    
    if vp_status != 1 or vp_connected != 1:
        return False
    
    return True
```

### 錯誤代碼定義
| 錯誤代碼 | 說明 | 原因 |
|----------|------|------|
| 101 | CG模組無回應 | 網路連接或模組故障 |
| 102 | CG模組未Ready | 模組初始化中或有警報 |
| 103 | VP模組狀態異常 | VP模組連接或狀態問題 |
| 201 | CG指令寫入失敗 | Modbus通訊錯誤 |
| 202 | CG檢測超時失敗 | 檢測過程超時 |
| 301 | VP震動啟動失敗 | VP控制指令錯誤 |
| 401 | Flow4觸發失敗 | Flow4控制失敗 |
| 402 | Flow4停止失敗 | Flow4停止指令失敗 |
| 999 | 入料週期異常 | 未知週期執行錯誤 |

## 配置系統

### 預設配置結構
```python
default_config = {
    "autofeeding": {
        "cycle_interval": 1.0,              # 檢測週期間隔 (秒)
        "cg_timeout": 5.0,                  # CG檢測超時 (秒)
        "flow4_consecutive_limit": 5,       # 連續直振限制
        "vp_empty_check_count": 3,          # VP空檢測計數
        "auto_start": True                  # 自動啟動檢測
    },
    "vp_params": {
        "spread_action_code": 11,           # VP散開動作代碼
        "spread_strength": 60,              # VP震動強度
        "spread_frequency": 50,             # VP震動頻率
        "spread_duration": 0.3,             # VP震動持續時間
        "stop_command_code": 3,             # VP停止指令代碼
        "stop_delay": 0.1                   # VP停止延遲
    },
    "flow4_params": {
        "pulse_duration": 0.1,              # Flow4脈衝持續時間
        "pulse_interval": 0.05              # Flow4脈衝間隔
    },
    "timing": {
        "command_delay": 0.05,              # 指令延遲
        "status_check_interval": 0.05,      # 狀態檢查間隔
        "register_clear_delay": 0.02,       # 寄存器清零延遲
        "vp_stabilize_delay": 0.15,         # VP穩定延遲
        "flow1_check_interval": 0.1         # Flow1監控間隔
    },
    "coordination": {
        "coords_taken_timeout": 10.0        # 座標被讀取超時
    }
}
```

### 配置檔案管理
```python
def load_config(self) -> Dict[str, Any]:
    """載入配置檔案"""
    config_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 
        'autofeeding_config.json'
    )
    
    if os.path.exists(config_path):
        # 載入現有配置
        with open(config_path, 'r', encoding='utf-8') as f:
            loaded_config = json.load(f)
            default_config.update(loaded_config)
    else:
        # 創建預設配置檔案
        with open(config_path, 'w', encoding='utf-8') as f:
            json.dump(default_config, f, indent=2, ensure_ascii=False)
    
    return default_config
```

## 部署配置

### 運行順序
1. 啟動主Modbus TCP Server (端口502)
2. 確保CG視覺模組正常運行 (基地址200)
3. 確保VP震動盤模組連接 (基地址300)
4. 啟動AutoFeeding_main.py
5. 系統自動開始檢測 (auto_start=True)

### 檔案結構
```
Automation/
├── AutoFeeding_main.py           # CG版本主程序
├── autofeeding_config.json       # 配置檔案 (自動生成)
└── logs/                         # 日誌目錄 (可選)
    └── autofeeding.log
```

### 系統整合
- **與CG視覺模組整合**: 通過基地址200進行通訊
- **與VP震動盤整合**: 通過基地址300控制震動
- **與Flow4整合**: 通過地址448控制直振送料
- **與Flow1整合**: 監控地址1201，提供座標940-944

## 測試驗證

### 連接測試
1. Modbus TCP連接狀態檢查 (127.0.0.1:502)
2. CG視覺模組通訊測試 (基地址200)
3. VP震動盤模組通訊測試 (基地址300)
4. Flow4控制通訊測試 (地址448)
5. Flow1狀態監控測試 (地址1201)

### 功能測試
1. CG檢測功能驗證
2. 保護區域判斷準確性
3. VP震動控制功能
4. Flow4送料控制功能
5. Flow1狀態監控與暫停機制
6. 座標交握機制驗證

### 性能監控
1. 檢測週期時間統計
2. CG檢測成功率
3. VP震動響應時間
4. Flow4送料成功率
5. 系統長期穩定性驗證

## 已知限制

### 硬體依賴
- 需要CG視覺模組正常運行
- VP震動盤必須正確連接
- Flow4機構必須可控
- Modbus TCP網路連接穩定

### 軟體限制
- 最多檢測5個CG_F物件
- 保護區域固定四邊形
- 單一CG視覺模組支援
- 同步執行模式

### 系統限制
- 獨立模組設計，無負載分散
- 記憶體中狀態管理，重啟會重置
- 配置變更需重啟生效

## 擴展規劃

### 功能擴展
- VP清空完整流程實現
- 多保護區域支援
- CG_B/STACK物件檢測整合
- 檢測歷史記錄功能

### 性能優化
- 適應性檢測週期調整
- 智能VP震動參數優化
- 預測性Flow4送料策略
- 多執行緒安全模式

### 監控增強
- Web介面狀態監控
- 遠端配置修改介面
- 即時性能圖表顯示
- 警報通知機制

## 故障排除

### 常見問題
1. **CG檢測失敗**: 檢查CG視覺模組狀態和網路連接
2. **VP震動無效**: 確認VP模組連接和參數設定
3. **Flow4送料失敗**: 檢查Flow4機構和控制信號
4. **Flow1監控異常**: 確認地址1201寄存器功能
5. **座標交握失敗**: 檢查940-945寄存器讀寫權限

### 除錯方法
1. 檢查系統日誌輸出
2. 監控寄存器數值變化
3. 驗證模組狀態寄存器
4. 測試單一功能模組
5. 分析錯誤代碼含義

### 效能優化建議
1. 調整檢測週期間隔
2. 優化VP震動參數
3. 減少不必要的寄存器讀寫
4. 使用適當的超時設定
5. 定期清理系統資源

## 與CASE版本差異

### 核心差異對比
| 特性 | CASE版本 | CG版本 |
|------|----------|--------|
| 檢測對象 | CASE_F/CASE_B/STACK | CG_F/CG_B/STACK |
| 保護區域 | (10.71,-246.12)到(-111.25,-364.22) | (-86,-369.51)到(8.07,-244.64) |
| 視覺模組基地址 | 200 | 200 |
| AutoFeeding基地址 | 900 | 900 |
| 架構設計 | 執行緒版本 | 獨立模組版本 |
| 穩定性策略 | 執行緒管理 | 無執行緒依賴 |

### 共同特性
- Modbus TCP通訊協議一致
- 32位座標處理算法相同
- VP震動盤控制邏輯相同
- Flow4送料機制相同
- Flow1監控機制相同
- 寄存器映射規範相同

### CG版本優勢
- **無執行緒依賴**: 避免長時間運行的執行緒穩定性問題
- **獨立模組設計**: 更好的可維護性和除錯能力
- **簡化架構**: 減少複雜的執行緒同步邏輯
- **資源管理**: 更精確的記憶體和連接管理
- **錯誤恢復**: 更直接的錯誤處理和恢復機制

## API介面文檔

### AutoFeedingModule核心方法
```python
class AutoFeedingModule:
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502)
    def connect(self) -> bool                              # 連接Modbus服務器
    def disconnect(self)                                   # 斷開連接
    def start_feeding(self)                               # 啟動入料檢測
    def stop_feeding(self)                                # 停止入料檢測
    def main_loop(self)                                   # 主循環執行
    def feeding_cycle(self) -> bool                       # 執行檢測週期
    def trigger_cg_detection(self) -> CGDetectionResult   # 觸發CG檢測
    def trigger_vp_vibration(self) -> bool               # 觸發VP震動
    def trigger_flow4_feeding(self) -> bool              # 觸發Flow4送料
    def set_cg_f_available(self, coords: Tuple[float, float])  # 設置CG_F可用
    def check_flow1_status(self) -> bool                 # 檢查Flow1狀態
    def check_coords_taken(self)                         # 檢查座標讀取
```

### ProtectionZone保護區域類
```python
class ProtectionZone:
    @staticmethod
    def is_point_in_quad(x_a: float, y_a: float) -> bool  # 點在區域內判斷
```

### AutoFeedingStatus狀態枚舉
```python
class AutoFeedingStatus(Enum):
    STOPPED = 0        # 停止
    RUNNING = 1        # 運行
    FLOW1_PAUSED = 2   # Flow1暫停
    DETECTING = 3      # 檢測中
    VP_VIBRATING = 4   # VP震動
    VP_CLEARING = 5    # VP清空
    ERROR = 6          # 錯誤
```

### OperationStatus操作狀態枚舉
```python
class OperationStatus(Enum):
    IDLE = 0              # 空閒
    CG_DETECTING = 1      # CG檢測
    VP_CONTROLLING = 2    # VP控制
    FLOW4_TRIGGERING = 3  # Flow4觸發
```