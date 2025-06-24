# LED控制器模組技術文檔

## 架構概述

LED控制器模組實現RS232轉ModbusTCP橋接，採用分離式架構設計。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> LED_main.py (TCP Client)
                    |
                    |-- RS232 --> LED控制器設備 (COM6, 9600bps)
    |
    |-- TCP --> LED_app.py (獨立TCP Client)
```

## 實現組件

### LED_main.py - 主模組
- TCP Client連接主服務器 (127.0.0.1:502)
- RS232連接LED控制器設備 (COM6, 9600, N, 8, 1)
- 寄存器映射基地址: 600
- 狀態機交握處理

### LED_app.py - Web控制應用
- 獨立TCP Client連接主服務器 (127.0.0.1:502)
- Flask Web介面 (0.0.0.0:5008)
- SocketIO即時通訊支援
- 純ModbusTCP Client實現

## 寄存器映射 (基地址600)

### 狀態寄存器 (只讀 600-615)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 600 | 模組狀態 | 0=離線, 1=閒置, 2=執行中, 3=初始化, 4=錯誤 |
| 601 | 設備連接狀態 | 0=斷開, 1=已連接 |
| 602 | 開啟通道數量 | 0-4 |
| 603 | 錯誤代碼 | 錯誤編號，0=無錯誤 |
| 604 | L1狀態 | 0=OFF, 1=ON |
| 605 | L2狀態 | 0=OFF, 1=ON |
| 606 | L3狀態 | 0=OFF, 1=ON |
| 607 | L4狀態 | 0=OFF, 1=ON |
| 608 | L1亮度 | 0-511 |
| 609 | L2亮度 | 0-511 |
| 610 | L3亮度 | 0-511 |
| 611 | L4亮度 | 0-511 |
| 612 | 操作計數 | 累積操作次數 |
| 613 | 錯誤計數 | 累積錯誤次數 |
| 614 | 保留 | 保留欄位 |
| 615 | 時間戳 | 最後更新時間戳 |

### 指令寄存器 (讀寫 620-624)
| 地址 | 功能 | 說明 |
|------|------|------|
| 620 | 指令代碼 | 執行的指令類型 |
| 621 | 參數1 | 通道號/亮度值 |
| 622 | 參數2 | 亮度值 |
| 623 | 指令ID | 唯一識別碼，防重複執行 |
| 624 | 保留 | 保留欄位 |

## 指令映射

| 代碼 | 指令 | 參數1 | 參數2 | 功能 |
|------|------|-------|-------|------|
| 0 | NOP | - | - | 無操作 |
| 1 | 全部開啟 | - | - | 所有通道設為255亮度 |
| 2 | 全部關閉 | - | - | 所有通道設為0亮度 |
| 3 | 重置設備 | - | - | 發送RESET指令 |
| 4 | 設定通道亮度 | 通道1-4 | 亮度0-511 | 設定單一通道亮度 |
| 5 | 開啟通道 | 通道1-4 | - | 開啟單一通道 |
| 6 | 關閉通道 | 通道1-4 | - | 關閉單一通道 |
| 7 | 錯誤重置 | - | - | 清除錯誤狀態 |

## RS232通訊協議

### 基本設定
- 端口: COM6
- 波特率: 9600
- 資料位: 8
- 停止位: 1
- 校驗位: None

### 指令格式
- 設定亮度: `CH{通道}:{亮度}\r\n`
- 重置設備: `RESET\r\n`
- 查詢版本: `VERSION?\r\n`

### 亮度範圍
- 支援範圍: 0-511
- 0為關閉，511為最大亮度

## 狀態機交握流程

### LED_main.py指令處理
1. 讀取新指令ID (623)
2. 檢測到新指令，讀取指令代碼 (620)、參數1 (621)、參數2 (622)
3. 設置執行狀態
4. 執行RS232指令發送
5. 更新LED狀態陣列
6. 更新錯誤代碼 (603)
7. 清除執行狀態
8. 清除指令寄存器 (620-624=0)

### 指令執行映射
- 指令1: 發送CH1:255, CH2:255, CH3:255, CH4:255
- 指令2: 發送CH1:0, CH2:0, CH3:0, CH4:0
- 指令3: 發送RESET
- 指令4: 發送CH{param1}:{param2}
- 指令5: 發送CH{param1}:{當前亮度或255}
- 指令6: 發送CH{param1}:0

## 配置檔案

### led_config.json (LED_main.py)
```json
{
  "module_id": "LED控制器模組",
  "serial_connection": {
    "port": "COM6",
    "baudrate": 9600,
    "parity": "N",
    "stopbits": 1,
    "bytesize": 8,
    "timeout": 1.0
  },
  "tcp_server": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1,
    "timeout": 3.0
  },
  "modbus_mapping": {
    "base_address": 600
  },
  "timing": {
    "fast_loop_interval": 0.05,
    "command_delay": 0.1,
    "serial_delay": 0.05
  }
}
```

### led_app_config.json (LED_app.py)
```json
{
  "module_id": "LED控制器Web UI",
  "tcp_server": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1,
    "timeout": 3.0
  },
  "modbus_mapping": {
    "base_address": 600
  },
  "web_server": {
    "host": "0.0.0.0",
    "port": 5008,
    "debug": false
  },
  "ui_settings": {
    "refresh_interval": 2.0,
    "auto_refresh": true
  }
}
```

## 開發錯誤復盤

### 1. 基地址衝突問題 (已修正)
**錯誤**: LED控制器基地址500與其他模組衝突
**修正**: 基地址調整到600，避開其他模組範圍

### 2. PySerial參數錯誤 (已修正)
**錯誤**: 配置檔案參數名稱錯誤導致串口連接失敗
**修正**: 確認PySerial參數名稱正確性，統一使用bytesize

### 3. Flask模板路徑問題 (已修正)
**錯誤**: Flask找不到led_index.html模板檔案
**修正**: LED_app.py中明確指定模板路徑

### 4. 架構理解錯誤 (已修正)
**錯誤**: LED_app.py初期包含串口操作代碼
**修正**: 明確分離職責，LED_app.py純粹作為ModbusTCP Client + Web UI

## API介面

### LED_app.py Web API路由
- GET / - Web介面首頁
- GET /api/status - 獲取LED狀態
- POST /api/connect - 連接Modbus服務器
- POST /api/channel/brightness - 設定通道亮度
- POST /api/channel/on - 開啟通道
- POST /api/channel/off - 關閉通道
- POST /api/all_on - 全部開啟
- POST /api/all_off - 全部關閉
- POST /api/reset - 重置設備
- POST /api/error_reset - 錯誤重置

### SocketIO事件
- connect/disconnect - 連接管理
- get_status - 狀態查詢
- set_brightness - 亮度設定
- channel_control - 通道控制
- global_control - 全域控制

## 檔案結構
```
LED控制器模組/
├── LED_main.py               # 主模組程序
├── LED_app.py                # Web控制應用
├── templates/                # Web模板目錄
│   └── led_index.html        # Web介面
├── led_config.json           # 主模組配置 (自動生成)
├── led_app_config.json       # Web應用配置 (自動生成)
└── tools.py                  # 測試工具 (可選)
```

## 運行狀態

### LED_main.py輸出
```
LED控制器模組運行中 - 基地址: 600
寄存器映射:
  狀態寄存器: 600 ~ 615
  指令寄存器: 620 ~ 624
```

### LED_app.py輸出
```
LED控制器Web應用啟動中...
Web服務器: http://0.0.0.0:5008
Modbus服務器地址: 127.0.0.1:502
LED控制器基地址: 600
```

## 部署說明

### 運行順序
1. 啟動主Modbus TCP Server (端口502)
2. 啟動LED_main.py (RS232橋接)
3. 啟動LED_app.py (Web介面)
4. 訪問http://localhost:5008進行控制

### 端口配置
- Modbus TCP: 502 (主服務器)
- Web介面: 5008 (LED_app.py)
- 串口: COM6 (LED控制器硬體)

### 相依性需求
- pymodbus>=3.0.0
- flask
- flask-socketio
- pyserial

## 測試驗證

### 功能測試結果
1. LED_main.py與主服務器連接: 正常
2. COM6串口連接: 需確認硬體連接
3. 寄存器讀寫: 地址映射正確
4. Web介面操作: http://localhost:5008正常運行
5. 亮度控制: 0-511範圍正常
6. 指令執行: 狀態機交握正常

### 已知限制
- LED控制器需要正確的COM6連接
- Web介面需要主服務器運行
- 亮度調整有200ms防抖動延遲

## 與專案其他模組的關係

### 基地址分配
- XC100升降模組: 1000-1049
- VP震動盤模組: 300-349
- CCD視覺模組: 200-299
- LED控制器: 600-649
- 機械臂模組: 400-449 (規劃中)

### 主服務器依賴
- 需要統一的ModbusTCP Server在502端口
- 與其他模組共享同一主服務器