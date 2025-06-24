# XC100升降模組技術文檔

## 架構概述

XC100升降模組實現RTU轉TCP橋接，採用分離式架構設計。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> XCModule.py (TCP Client)
                    |
                    |-- RTU --> XC100設備 (COM5)
    |
    |-- TCP --> XCApp.py (獨立TCP Client)
```

## 實現組件

### XCModule.py - 主模組
- TCP Client連接主服務器 (127.0.0.1:502)
- RTU Serial連接XC100設備 (COM5, 115200)
- 寄存器映射基地址: 1000
- 超高速循環: 20ms間隔

### XCApp.py - Web控制應用
- 獨立TCP Client連接主服務器 (127.0.0.1:502)
- Flask Web介面 (localhost:5007)
- SocketIO即時通訊支援
- 手動刷新模式控制

## 寄存器映射 (基地址1000)

### 狀態寄存器 (只讀 1000-1014)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 1000 | 模組狀態 | 0=離線, 1=閒置, 2=移動中, 3=原點復歸中, 4=錯誤, 5=Servo關閉, 6=緊急停止 |
| 1001 | XC設備連接狀態 | 0=斷開, 1=已連接 |
| 1002 | Servo狀態 | 0=OFF, 1=ON |
| 1003 | 錯誤代碼 | 錯誤編號，0=無錯誤 |
| 1004 | 當前位置低位 | 32位位置低16位 |
| 1005 | 當前位置高位 | 32位位置高16位 |
| 1006 | 目標位置低位 | 32位位置低16位 |
| 1007 | 目標位置高位 | 32位位置高16位 |
| 1008 | 指令執行狀態 | 0=空閒, 1=執行中 |
| 1009 | 通訊錯誤計數 | 累積錯誤次數 |
| 1010 | 位置A低位 | 預設位置A低16位 |
| 1011 | 位置A高位 | 預設位置A高16位 |
| 1012 | 位置B低位 | 預設位置B低16位 |
| 1013 | 位置B高位 | 預設位置B高16位 |
| 1014 | 時間戳 | 最後更新時間戳 |

### 指令寄存器 (讀寫 1020-1024)
| 地址 | 功能 | 說明 |
|------|------|------|
| 1020 | 指令代碼 | 執行的指令類型 |
| 1021 | 參數1 | 第一個參數 (位置低位) |
| 1022 | 參數2 | 第二個參數 (位置高位) |
| 1023 | 指令ID | 唯一識別碼，防重複執行 |
| 1024 | 保留 | 保留欄位 |

## 指令映射

| 代碼 | 指令 | 參數1 | 參數2 | 功能 |
|------|------|-------|-------|------|
| 0 | NOP | - | - | 無操作 |
| 1 | SERVO_ON | - | - | 伺服馬達啟動 |
| 2 | SERVO_OFF | - | - | 伺服馬達停止 |
| 3 | HOME | - | - | 原點復歸 |
| 4 | MOVE_ABS | 位置低位 | 位置高位 | 絕對位置移動 |
| 5 | MOVE_REL | 距離低位 | 距離高位 | 相對位置移動 |
| 6 | EMERGENCY_STOP | - | - | 緊急停止 |
| 7 | RESET_ERROR | - | - | 錯誤重置 |

## XC100設備RTU寄存器

### 關鍵控制寄存器
| 寄存器 | 功能 | 說明 |
|--------|------|------|
| 0x2011 | 伺服控制 | 0=ON, 1=OFF |
| 0x201E | 動作指令 | 1=絕對移動, 3=原點復歸 |
| 0x2002 | 目標位置 | 32位位置值 (高位在前) |
| 0x2020 | 緊急停止 | 1=停止 |

## 狀態機交握流程

### XCModule.py超高速循環
1. 讀取指令寄存器 (1020-1024)
2. 檢測新指令ID避免重複執行
3. 執行XC100 RTU指令
4. 更新狀態寄存器 (1000-1014)
5. 清除指令寄存器

### 指令執行實現
- SERVO_ON: 寫入0x2011=0
- MOVE_ABS: 先寫入0x2002位置，再寫入0x201E=1
- HOME: 寫入0x201E=3
- EMERGENCY_STOP: 寫入0x2020=1

## 32位數值處理

### 位置數據分解與合併
```python
# 32位轉16位 (發送到XC100)
pos_high = (position >> 16) & 0xFFFF
pos_low = position & 0xFFFF

# 16位合併32位 (從XC100讀取)
position = (high_16bit << 16) | low_16bit
```

## 配置檔案

### xc_module_config.json
```json
{
  "module_id": "XC100_01",
  "xc_connection": {
    "port": "COM5",
    "baudrate": 115200,
    "unit_id": 2,
    "timeout": 2.0
  },
  "tcp_server": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1
  },
  "modbus_mapping": {
    "base_address": 1000
  },
  "timing": {
    "fast_loop_interval": 0.02
  }
}
```

### xc_app_config.json
```json
{
  "modbus_tcp": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1
  },
  "web_server": {
    "host": "0.0.0.0",
    "port": 5007
  },
  "xc_module": {
    "base_address": 1000
  },
  "ui_settings": {
    "refresh_interval": 3.0,
    "manual_mode": false
  }
}
```

## 開發錯誤復盤

### 1. 寄存器地址映射錯誤 (已修正)
**錯誤**: App.py使用相對地址讀寫寄存器
```python
# 錯誤實現
result = self.modbus_client.read_holding_registers(address=0, count=16)
result = self.modbus_client.write_registers(address=7, values=values)
```

**修正**: XCApp.py使用絕對地址與XCModule一致
```python
# 修正實現
self.base_address = 1000
result = self.modbus_client.read_holding_registers(address=self.base_address, count=15)
command_address = self.base_address + 20
result = self.modbus_client.write_registers(address=command_address, values=values)
```

### 2. Web介面刷新頻率過高 (已修正)
**問題**: 原始更新間隔過短導致位置輸入被打斷
**修正**: 
- 刷新間隔增加到3秒
- 新增手動刷新模式
- SocketIO控制自動更新開關

### 3. 32位位置處理錯誤 (已修正)
**錯誤**: 位置合併順序錯誤
```python
# 錯誤順序
position = (registers[4] << 16) | registers[3]  # 低位在前
```

**修正**: 依據XC100文檔修正位置合併
```python
# 修正順序
position = (registers[5] << 16) | registers[4]  # 高位在前
```

### 4. 配置檔案路徑問題 (已修正)
**錯誤**: 配置檔案可能生成在專案根目錄
**修正**: 使用執行檔案目錄路徑
```python
self.current_dir = os.path.dirname(os.path.abspath(__file__))
self.config_file = os.path.join(self.current_dir, config_file)
```

### 5. 指令ID重複執行問題 (已修正)
**解決**: 實現指令ID檢查機制
- last_command_id追蹤已執行指令
- 避免重複執行相同指令
- 指令完成後清除寄存器

## 線程安全實現

### XCModule.py
- 使用_state_lock保護狀態變量
- ultra_fast_loop獨立執行緒
- 原子化操作更新狀態

### XCApp.py
- monitor_thread處理狀態監控
- SocketIO異步通訊
- 手動/自動模式切換控制

## 通訊時序

### 循環頻率
- XCModule主循環: 20ms
- XCApp狀態監控: 3秒
- Web介面更新: 手動觸發或3秒自動

### 超時設定
- RTU通訊: 2秒
- TCP連接: 3秒
- Web介面: 按需刷新

## 錯誤處理機制

### 連接管理
- TCP斷線自動重連 (最大5次)
- RTU斷線重試機制
- 錯誤計數統計

### 狀態同步
- 指令執行狀態追蹤
- 通訊健康度計算
- 錯誤代碼映射

## 測試驗證

### 功能測試結果
1. XCModule與主服務器連接: 正常
2. XC100設備連接: COM5權限問題需處理
3. 寄存器讀寫: 地址映射已修正
4. Web介面操作: localhost:5007正常運行
5. 位置控制: 輸入體驗已改善
6. 指令執行: 狀態機交握正常

### 已知限制
- XC100設備需要正確的COM端口權限
- 超高速模式下通訊錯誤處理需持續監控
- Web介面手動模式下需用戶主動刷新狀態

## 部署配置

### 運行順序
1. 啟動主Modbus Server (端口502)
2. 啟動XCModule.py (RTU橋接)
3. 啟動XCApp.py (Web介面)
4. 訪問localhost:5007進行控制

### 文件結構
```
XC100/
├── XCModule.py              # 主模組
├── XCApp.py                 # Web應用  
├── xc_module_config.json    # 主模組配置
├── xc_app_config.json       # Web應用配置
└── templates/
    └── index.html           # Web介面
```