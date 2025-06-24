# VP震動盤模組技術文檔

## 架構概述

VP震動盤模組實現TCP到TCP橋接，非RTU轉TCP。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> VP_main.py (TCP Client)
                    |
                    |-- TCP --> 震動盤設備 (192.168.1.7:1000)
```

## 實現組件

### VP_main.py - 主模組
- TCP Client連接主服務器 (127.0.0.1:502)
- TCP Client連接震動盤設備 (192.168.1.7:1000)
- 寄存器映射基地址: 300
- 狀態機交握處理

### VP_app.py - Web控制應用
- 獨立的Modbus TCP Client
- 連接主服務器讀寫VP模組寄存器
- Flask Web介面 (0.0.0.0:5053)

### vibration_plate.py - 設備控制API
- ModbusTCP通訊封裝
- 震動盤硬體控制接口

## 寄存器映射 (基地址300)

### 狀態寄存器 (只讀 300-314)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 300 | 模組狀態 | 0=離線, 1=閒置, 2=執行中, 3=初始化, 4=錯誤 |
| 301 | 設備連接狀態 | 0=斷開, 1=已連接 |
| 302 | 設備狀態 | 0=OFF, 1=ON |
| 303 | 錯誤代碼 | 錯誤編號，0=無錯誤 |
| 304 | 當前動作低位 | 當前動作編碼低16位 |
| 305 | 當前動作高位 | 當前動作編碼高16位 |
| 306 | 目標動作低位 | 目標動作編碼低16位 |
| 307 | 目標動作高位 | 目標動作編碼高16位 |
| 308 | 指令執行狀態 | 0=空閒, 1=執行中 |
| 309 | 通訊錯誤計數 | 累積錯誤次數 |
| 310 | 背光亮度狀態 | 當前背光亮度 (0-255) |
| 311 | 背光開關狀態 | 0=關閉, 1=開啟 |
| 312 | 震動狀態 | 0=停止, 1=運行 |
| 313 | 保留 | 保留欄位 |
| 314 | 時間戳 | 最後更新時間戳 |

### 指令寄存器 (讀寫 320-324)
| 地址 | 功能 | 說明 |
|------|------|------|
| 320 | 指令代碼 | 執行的指令類型 |
| 321 | 參數1 | 強度/亮度/動作碼 |
| 322 | 參數2 | 頻率 |
| 323 | 指令ID | 唯一識別碼，防重複執行 |
| 324 | 保留 | 保留欄位 |

## 指令映射

### 基本指令 (0-7)
| 代碼 | 指令 | 參數1 | 參數2 | 功能 |
|------|------|-------|-------|------|
| 0 | NOP | - | - | 無操作 |
| 1 | 設備啟用 | - | - | 背光開啟 |
| 2 | 設備停用 | - | - | 背光關閉 |
| 3 | 停止所有動作 | - | - | 停止震動 |
| 4 | 設定背光亮度 | 亮度值(0-255) | - | 設定背光亮度 |
| 5 | 執行動作 | 動作碼 | 強度 | 執行震動動作 |
| 6 | 緊急停止 | - | - | 立即停止 |
| 7 | 錯誤重置 | - | - | 清除錯誤狀態 |

### VP專用指令 (11-13)
| 代碼 | 指令 | 參數1 | 參數2 | 功能 |
|------|------|-------|-------|------|
| 11 | 設定動作參數 | 動作碼 | 強度 | 設定動作參數 |
| 12 | 背光切換 | 狀態(0/1) | - | 背光開關切換 |
| 13 | 執行特定動作 | 動作碼 | 強度 | 執行動作並設定參數 |

## 動作映射

```python
ACTION_MAP = {
    'stop': 0, 'up': 1, 'down': 2, 'left': 3, 'right': 4,
    'upleft': 5, 'downleft': 6, 'upright': 7, 'downright': 8,
    'horizontal': 9, 'vertical': 10, 'spread': 11
}
```

## 震動盤設備寄存器

### 關鍵寄存器
| 寄存器 | 功能 |
|--------|------|
| 4 | 單一動作觸發 |
| 6 | 震動盤狀態 |
| 46 | 背光亮度 |
| 58 | 背光測試開關 |
| 20-30 | 強度寄存器 (各動作) |
| 60-70 | 頻率寄存器 (各動作) |

## 狀態機交握流程

### VP_main.py指令處理
1. 讀取新指令ID (323)
2. 檢測到新指令，讀取指令代碼 (320)、參數1 (321)、參數2 (322)
3. 設置執行狀態 (308=1)
4. 調用execute_command()執行指令
5. 更新錯誤代碼 (303)
6. 清除執行狀態 (308=0)
7. 清除指令寄存器 (320-324=0)

### 指令執行映射
```python
def execute_command(self, command: int, param1: int, param2: int) -> bool:
    if command == 1:  # 背光開啟
        success = self.vibration_plate.set_backlight(True)
    elif command == 4:  # 設定背光亮度
        success = self.vibration_plate.set_backlight_brightness(param1)
    elif command == 5:  # 執行動作
        actions = ['stop', 'up', 'down', 'left', 'right', 'upleft', 
                  'downleft', 'upright', 'downright', 'horizontal', 'vertical', 'spread']
        if 0 <= param1 < len(actions):
            action = actions[param1]
            success = self.vibration_plate.execute_action(action, param2, param2)
```

## 配置檔案

### vp_config.json (VP_main.py)
```json
{
  "module_id": "震動盤模組",
  "device_connection": {
    "ip": "192.168.1.7",
    "port": 1000,
    "slave_id": 10,
    "timeout": 0.2
  },
  "tcp_server": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1,
    "timeout": 1.0
  },
  "modbus_mapping": {
    "base_address": 300
  },
  "timing": {
    "fast_loop_interval": 0.02,
    "movement_delay": 0.1,
    "command_delay": 0.02
  }
}
```

### vp_app_config.json (VP_app.py)
```json
{
  "module_id": "震動盤Web UI",
  "tcp_server": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1,
    "timeout": 1.0
  },
  "modbus_mapping": {
    "base_address": 300
  },
  "web_server": {
    "host": "0.0.0.0",
    "port": 5053,
    "debug": false
  }
}
```

## 實現細節

### 線程安全
- VP_main.py使用self.loop_lock保護主循環
- vibration_plate.py使用self.lock保護Modbus操作

### 連接管理
- VP_main.py實現雙TCP連接自動重連
- 主服務器斷線重連 (connect_main_server)
- 震動盤設備斷線重連 (connect_device)
- 連接狀態檢測 (is_connected)

### 錯誤處理
- 操作計數統計 (operation_count)
- 錯誤計數統計 (error_count)
- 連接計數統計 (connection_count)
- 錯誤代碼映射到寄存器303

### 狀態更新
- 20ms主循環 (fast_loop_interval)
- 設備狀態同步到寄存器301-302
- 震動狀態同步到寄存器312
- 背光狀態同步到寄存器310-311

## 已知問題與修正

### 1. 通訊協議理解錯誤 (已修正)
**錯誤**: 誤認為RTU轉TCP橋接
**修正**: VP是TCP到TCP橋接，震動盤設備本身就是ModbusTCP設備

### 2. 配置檔案路徑 (已修正)
**錯誤**: 配置檔案生成在專案根目錄
**修正**: 使用os.path.dirname(os.path.abspath(__file__))生成在執行檔同層目錄

### 3. 寄存器同步問題 (已實現)
**解決**: 實現完整的寄存器映射同步
- VP_main.py定期更新狀態寄存器到主服務器
- VP_app.py讀取主服務器寄存器獲取VP狀態

### 4. 指令ID機制 (已實現)
**解決**: 實現指令ID防重複執行
- last_command_id追蹤
- 指令完成後清除寄存器

### 5. 記憶體管理 (已實現)
**解決**: 
- 線程daemon模式
- 連接對象正確關閉
- 異常處理和資源回收

## API介面

### VibrationPlate類關鍵方法
```python
def set_backlight(self, state: bool) -> bool
def set_backlight_brightness(self, brightness: int) -> bool  
def trigger_action(self, action: str) -> bool
def set_action_parameters(self, action: str, strength: int, frequency: int) -> bool
def execute_action(self, action: str, strength: int, frequency: int, duration: float) -> bool
def stop(self) -> bool
def get_status(self) -> Dict[str, Any]
```

### VP_main.py關鍵方法
```python
def connect_main_server(self) -> bool
def connect_device(self) -> bool
def execute_command(self, command: int, param1: int, param2: int) -> bool
def update_status_registers(self)
def process_commands(self)
```

### VP_app.py Web API路由
- GET / - Web介面
- GET /api/status - 獲取狀態
- POST /api/connect - 連接服務器
- POST /api/execute_action - 執行動作
- POST /api/set_brightness - 設定亮度
- POST /api/emergency_stop - 緊急停止

## 檔案結構
```
VP/
├── VP_main.py          # 主模組程序
├── VP_app.py           # Web控制應用
├── vibration_plate.py  # 設備控制API
├── vp_config.json      # 主模組配置
├── vp_app_config.json  # Web應用配置
└── templates/
    └── index.html      # Web介面
```

## 運行狀態

### VP_main.py輸出
```
震動盤模組運行中 - 基地址: 300
寄存器映射:
  狀態寄存器: 300 ~ 314
  指令寄存器: 320 ~ 324
```

### VP_app.py輸出
```
震動盤Web控制應用啟動中...
Web服務器啟動 - http://0.0.0.0:5053
Modbus服務器地址: 127.0.0.1:502
震動盤模組基地址: 300
```

## 測試驗證

### 連接測試
1. VP_main.py能否連接主服務器和震動盤設備
2. VP_app.py能否連接主服務器讀寫寄存器
3. 寄存器數值同步是否正確

### 功能測試
1. 震動動作執行 (11種模式)
2. 背光控制 (開關/亮度)
3. 緊急停止功能
4. 指令ID防重複機制

### Web介面測試
1. 狀態監控顯示
2. 動作控制按鈕
3. 參數設定功能
4. SocketIO即時更新