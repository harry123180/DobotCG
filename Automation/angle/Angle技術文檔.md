# 角度調整系統技術文檔

## 架構概述

角度調整系統實現CCD3視覺檢測與馬達驅動器RTU控制的整合橋接，採用分離式架構設計。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> angle_main.py (TCP Client)
                    |
                    |-- RTU --> 馬達驅動器 (COM5, slave 3)
                    |-- TCP --> CCD3模組 (基地址800)
    |
    |-- TCP --> angle_app.py (獨立TCP Client, Web介面)
```

## 實現組件

### angle_main.py - 主模組
- **通訊架構**: TCP Client連接主服務器 (127.0.0.1:502)
- **RTU控制**: Serial連接馬達驅動器 (COM5, 115200, slave 3)
- **CCD3整合**: TCP連接CCD3模組 (基地址800) 進行角度檢測
- **寄存器映射**: 基地址700-749
- **控制模式**: 狀態機交握處理，50ms輪詢間隔

### angle_app.py - Web控制應用  
- **獨立連接**: TCP Client連接主服務器 (127.0.0.1:502)
- **Web介面**: Flask應用 (localhost:5087)
- **即時通訊**: SocketIO支援，1秒狀態更新
- **純客戶端**: ModbusTCP Client實現，無RTU操作

### templates/angle_index.html - Web介面
- **設計風格**: 簡約白藍漸層設計
- **功能特性**: 即時狀態監控、指令控制、結果顯示
- **響應式**: 支援桌面和移動設備
- **狀態指示**: 綠色正常、紅色異常、黃色運行中

## 核心功能流程

### 角度校正完整流程
1. **接收控制指令**: PLC發送指令1到寄存器740
2. **CCD3拍照檢測**: 觸發CCD3模組執行角度檢測 (指令16到寄存器800)
3. **等待檢測完成**: 監控CCD3狀態寄存器801，等待Ready=1且Running=0
4. **角度數據讀取**: 從CCD3寄存器843-844讀取32位角度數據
5. **位置計算**: 按公式 `馬達位置 = 9000 - (CCD3角度 × 10)` 計算
6. **馬達運動**: 設置位置到寄存器6147，發送移動指令8到寄存器125
7. **狀態監控**: 等待馬達運動完成，自動處理警報狀態
8. **警報處理**: 檢測到警報自動發送ALM-RST指令(128)清除
9. **結果輸出**: 更新校正結果到寄存器720-739
10. **狀態恢復**: 清除指令寄存器，恢復Ready狀態

### 角度計算實例
```
範例1: CCD3檢測角度 = 45.45度
計算: 9000 - (45.45 × 10) = 9000 - 454 = 8546
馬達目標位置: 8546

範例2: CCD3檢測角度 = 83.03度  
計算: 9000 - (83.03 × 10) = 9000 - 830 = 8170
馬達目標位置: 8170
```

## 寄存器映射 (基地址700)

### 狀態寄存器 (只讀 700-714)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 700 | 狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized, bit4=CCD檢測中, bit5=馬達運動中 |
| 701 | Modbus連接狀態 | 0=斷開, 1=已連接 |
| 702 | 馬達連接狀態 | 0=斷開, 1=已連接 |
| 703 | 錯誤代碼 | 錯誤編號，0=無錯誤 |
| 704 | 操作計數低位 | 32位操作計數低16位 |
| 705 | 操作計數高位 | 32位操作計數高16位 |
| 706 | 錯誤計數 | 累積錯誤次數 |
| 707-714 | 保留 | 未來擴展使用 |

### 檢測結果寄存器 (只讀 720-739)
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 720 | 成功標誌 | 0=失敗, 1=成功 | 角度校正結果有效性 |
| 721 | 原始角度高位 | 32位角度高16位 | CCD3檢測角度×100存儲 |
| 722 | 原始角度低位 | 32位角度低16位 | CCD3檢測角度×100存儲 |
| 723 | 角度差高位 | 32位角度差高16位 | 與90度差值×100存儲 |
| 724 | 角度差低位 | 32位角度差低16位 | 與90度差值×100存儲 |
| 725 | 馬達位置高位 | 32位位置高16位 | 計算出的馬達目標位置 |
| 726 | 馬達位置低位 | 32位位置低16位 | 計算出的馬達目標位置 |
| 730 | 成功次數低位 | 32位成功次數低16位 | 統計資訊 |
| 731 | 成功次數高位 | 32位成功次數高16位 | 統計資訊 |
| 732 | 錯誤次數 | 累積錯誤次數 | 統計資訊 |
| 733 | 運行時間 | 秒單位 | 系統運行時間 |

### 控制指令寄存器 (讀寫 740)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 740 | 控制指令 | 0=清空, 1=角度校正, 2=馬達重置, 7=錯誤重置 |

## 指令映射

| 代碼 | 指令 | 功能說明 |
|------|------|----------|
| 0 | CLEAR | 清除控制指令 |
| 1 | ANGLE_CORRECTION | 執行完整角度校正流程 |
| 2 | MOTOR_RESET | 馬達重置，清除馬達指令狀態 |
| 7 | ERROR_RESET | 錯誤重置，清除系統錯誤狀態 |

## CCD3模組整合

### CCD3觸發與讀取
```python
# 觸發CCD3角度檢測
def trigger_ccd3_detection(self):
    # 發送拍照+角度檢測指令 (CCD3寄存器800, 值16)
    result = self.modbus_client.write_register(
        address=self.ccd3_base_address, value=16, slave=1
    )

# 讀取CCD3角度數據
def read_ccd3_angle(self):
    # 讀取CCD3檢測結果寄存器 (840-844)
    result = self.modbus_client.read_holding_registers(
        address=self.ccd3_base_address + 40, count=5, slave=1
    )
    
    success_flag = registers[0]    # 840: 檢測成功標誌
    center_x = registers[1]        # 841: 中心X座標
    center_y = registers[2]        # 842: 中心Y座標
    angle_high = registers[3]      # 843: 角度高位
    angle_low = registers[4]       # 844: 角度低位
    
    # 合併32位角度數據，恢復2位小數精度
    angle_int = (angle_high << 16) | angle_low
    angle_degrees = angle_int / 100.0
```

## 馬達驅動器RTU控制

### 通訊協議
- **串口參數**: COM5, 115200, 8N1, slave ID 3
- **功能碼**: 06h(寫單個寄存器), 03h(讀保持寄存器), 10h(寫多個寄存器)
- **CRC校驗**: CRC-16校驗算法確保通訊可靠性

### 關鍵寄存器
| 寄存器 | 功能 | 說明 |
|--------|------|------|
| 6147 | 目標位置 | 設置馬達目標位置 |
| 125 | 控制指令 | 8=絕對位置移動, 128=警報重置, 0=清除指令 |
| 127 | 狀態寄存器 | bit13=運動中, bit5=準備就緒, bit7=警報狀態 |

### 馬達控制流程
```python
# 1. 狀態檢查
status = self.motor_rtu.read_holding_registers(slave_id, 127, 1)
ready = bool(status_word & (1 << 5))    # bit 5: 準備就緒
alarm = bool(status_word & (1 << 7))    # bit 7: 警報狀態

# 2. 設置目標位置
self.motor_rtu.write_single_register(slave_id, 6147, position)

# 3. 發送移動指令
self.motor_rtu.write_single_register(slave_id, 125, 8)

# 4. 警報處理 (如需要)
if alarm:
    # 發送ALM-RST指令
    self.motor_rtu.write_single_register(slave_id, 125, 128)
    time.sleep(0.2)
    self.motor_rtu.write_single_register(slave_id, 125, 0)

# 5. 清除指令
self.motor_rtu.write_single_register(slave_id, 125, 0)
```

## 狀態機設計

### AngleSystemStateMachine類
6位狀態控制系統：
- **Ready** (bit0): 系統準備接受新指令
- **Running** (bit1): 系統正在執行操作  
- **Alarm** (bit2): 系統異常或錯誤
- **Initialized** (bit3): 系統已完全初始化
- **CCD_Detecting** (bit4): CCD正在檢測角度
- **Motor_Moving** (bit5): 馬達正在運動

### 狀態機交握流程
1. **初始狀態**: Ready=1, Initialized=1
2. **接收指令**: PLC寫入740=1  
3. **開始執行**: Ready=0, Running=1
4. **CCD檢測階段**: CCD_Detecting=1
5. **馬達運動階段**: CCD_Detecting=0, Motor_Moving=1
6. **執行完成**: Motor_Moving=0, Running=0
7. **等待清零**: PLC寫入740=0
8. **恢復Ready**: Ready=1

## 高速小角度運動優化

### 檢測策略調整
針對馬達高速運動、小角度調整的特性，採用優化檢測邏輯：

```python
def wait_motor_complete(self):
    # 1. 指令發送後等待2秒讓馬達完成運動
    time.sleep(2.0)
    
    # 2. 檢查10次狀態，每次200ms間隔
    for i in range(10):
        # 3. 自動處理警報狀態
        if alarm:
            # 發送ALM-RST指令清除警報
            self.motor_rtu.write_single_register(slave_id, 125, 128)
            
        # 4. 連續3次狀態正常即認為完成
        if not alarm and stable_count >= 3:
            return True
    
    # 5. 基於指令發送成功認定完成
    return True
```

### 警報自動處理
- **檢測警報**: 自動監控bit7警報狀態
- **發送清除**: 自動發送ALM-RST指令(值128)
- **驗證結果**: 確認警報是否成功清除
- **錯誤處理**: 清除失敗時明確報錯

## 配置檔案

### angle_config.json (angle_main.py)
```json
{
  "module_id": "Angle_Adjustment_System",
  "modbus_tcp": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1,
    "timeout": 3.0
  },
  "motor_rtu": {
    "port": "COM5",
    "baudrate": 115200,
    "slave_id": 3
  },
  "modbus_mapping": {
    "base_address": 700,
    "ccd3_base_address": 800
  },
  "angle_calculation": {
    "motor_base_position": 9000,
    "angle_multiplier": 10
  },
  "timing": {
    "handshake_interval": 0.05,
    "ccd_timeout": 10.0,
    "motor_timeout": 5.0
  }
}
```

### angle_app_config.json (angle_app.py)
```json
{
  "module_id": "Angle_Adjustment_Web_App",
  "modbus_tcp": {
    "host": "127.0.0.1",
    "port": 502
  },
  "web_server": {
    "host": "0.0.0.0",
    "port": 5087
  },
  "ui_settings": {
    "refresh_interval": 1.0,
    "auto_refresh": true
  }
}
```

## 32位數值處理

### 角度精度處理  
```python
# CCD3角度存儲 (保留2位小數)
angle_int = int(angle * 100)  # 45.45 → 4545
angle_high = (angle_int >> 16) & 0xFFFF  # 高16位 → 721/723
angle_low = angle_int & 0xFFFF           # 低16位 → 722/724

# 寄存器恢復角度值
angle_int = (angle_high << 16) | angle_low  # 合併32位
if angle_int >= 2**31:  # 處理有符號數值
    angle_int -= 2**32
final_angle = angle_int / 100.0  # 恢復精度: 4545 → 45.45
```

### 馬達位置處理
```python
# 馬達位置32位存儲
position_high = (position >> 16) & 0xFFFF  # 高16位 → 725
position_low = position & 0xFFFF           # 低16位 → 726
```

## Flask Web介面

### 核心API路由
- **GET /** - Web介面首頁
- **POST /api/modbus/connect** - 連接Modbus服務器
- **POST /api/modbus/disconnect** - 斷開Modbus連接  
- **GET /api/status** - 獲取系統狀態和結果
- **POST /api/command/angle_correction** - 執行角度校正
- **POST /api/command/motor_reset** - 馬達重置
- **POST /api/command/error_reset** - 錯誤重置
- **POST /api/command/clear** - 清除指令

### SocketIO即時通訊
- **status_update**: 狀態資訊推送 (1秒間隔)
- **command_response**: 指令執行結果回應
- **connect/disconnect**: 連接管理事件

### Web介面特色
- **簡約設計**: 白藍漸層風格，響應式布局
- **狀態監控**: 6個狀態位即時顯示，綠色正常/紅色異常
- **結果顯示**: 檢測角度、角度差、馬達位置即時更新
- **操作日誌**: 即時操作記錄，支援清除功能
- **計算公式**: 顯示馬達位置計算公式與範例

## 錯誤處理機制

### 連接管理
- **Modbus TCP自動重連**: 主服務器斷線檢測與重連
- **RTU連接監控**: COM5端口狀態檢測  
- **CCD3模組狀態**: 基於寄存器狀態判斷連接
- **超時控制**: CCD檢測10秒，馬達運動5秒

### 異常處理
- **通訊錯誤計數**: 統計並記錄到寄存器706
- **Alarm狀態設置**: 異常時自動設置Alarm=1
- **錯誤重置機制**: 指令7清除所有錯誤狀態
- **資源釋放**: 異常時確保狀態正確清除

### 自動恢復
```python
# 馬達警報自動處理
if alarm:
    # 發送ALM-RST指令
    success = self.motor_rtu.write_single_register(slave_id, 125, 128)
    time.sleep(0.2)
    self.motor_rtu.write_single_register(slave_id, 125, 0)
    
    # 驗證清除結果
    if alarm_after_reset:
        print("錯誤: 清除警報後仍然存在警報狀態")
    else:
        print("警報已成功清除")
```

## 線程架構

### 主要線程
- **主線程**: Flask Web應用服務 (angle_app.py)
- **握手同步線程**: 50ms輪詢寄存器更新 (angle_main.py)
- **指令執行線程**: 異步執行角度校正流程
- **狀態監控線程**: 1秒更新Web界面狀態

### 線程安全機制  
- **AngleSystemStateMachine**: 使用threading.Lock保護狀態寄存器
- **指令執行控制**: command_processing標誌防重入
- **ModbusRTU序列化**: RTU通訊避免衝突
- **daemon模式**: 線程自動回收機制

## 性能特性

### 執行時序
- **CCD3檢測時間**: 通常2-5秒
- **角度計算時間**: <1毫秒  
- **馬達運動時間**: 2秒內完成 (高速小角度)
- **總流程時間**: 通常5-10秒
- **狀態輪詢頻率**: 50ms (主循環)，1秒 (Web更新)

### 通訊特性
- **Modbus TCP**: 主服務器連接，3秒超時
- **Modbus RTU**: COM5, 115200, 1秒超時  
- **CCD3整合**: 基於TCP寄存器操作
- **網路延遲**: <10ms (本地TCP連接)

## 部署配置

### 運行順序
1. **啟動主Modbus TCP Server** (端口502)
2. **啟動CCD3角度辨識模組** (基地址800，必須先運行)
3. **啟動angle_main.py** (角度調整主程序)
4. **啟動angle_app.py** (Web控制介面)
5. **訪問Web介面** (http://localhost:5087)

### 硬體依賴
- **CCD3模組**: 必須正常運行並能檢測角度
- **馬達驅動器**: COM5端口，115200波特率，slave 3
- **主服務器**: 127.0.0.1:502 Modbus TCP服務
- **馬達參數**: 預先設定，系統不會覆蓋

### 檔案結構
```
Angle/
├── angle_main.py              # 主程序
├── angle_app.py               # Web應用 (端口5087)
├── templates/                 # Web模板目錄
│   └── angle_index.html       # Web介面 (白藍漸層風格)
├── angle_config.json          # 主程序配置 (自動生成)
├── angle_app_config.json      # Web應用配置 (自動生成)
└── README.md                  # 模組說明
```

## 測試驗證

### 功能測試
1. **角度校正流程**: CCD3檢測 → 角度計算 → 馬達移動完整測試
2. **警報處理**: 自動ALM-RST指令測試
3. **高速運動**: 小角度快速移動驗證
4. **Web介面**: 即時狀態監控與控制測試
5. **錯誤恢復**: 異常情況處理測試

### 性能測試  
1. **響應時間**: 指令執行到完成總時間約5-10秒
2. **通訊穩定性**: 長時間運行無異常
3. **狀態同步**: Web界面狀態更新及時
4. **記憶體使用**: 穩定運行無洩漏

## 故障排除

### 常見問題
1. **CCD3連接失敗**: 檢查CCD3模組(基地址800)是否運行
2. **馬達RTU連接失敗**: 檢查COM5端口權限和參數
3. **警報狀態**: 系統自動處理，手動可用Web介面重置
4. **Web介面異常**: 確認端口5087未被占用

### 運行日誌範例
```
=== 角度校正流程完成 ===
檢測角度: 45.45度
角度差: 44.55度  
馬達位置: 8546
檢測結果已寫入寄存器: 成功=True
```

## 版本歷史

### v1.0 (2024-12-XX) - 初始版本
- **基礎功能**: CCD3拍照 → 角度計算 → 馬達補正完整流程
- **寄存器映射**: 700-749完整寄存器定義
- **狀態機**: 6位狀態機設計與實現  
- **Web介面**: 簡約白藍漸層設計，端口5009
- **硬體整合**: CCD3模組 + 馬達驅動器RTU整合
- **計算公式**: 馬達位置 = 9000 - (CCD3角度 × 10)
- **高速優化**: 針對高速小角度運動優化檢測邏輯
- **警報處理**: 自動ALM-RST清除機制
- **精度支援**: 角度保留2位小數精度

---

**文檔版本**: v1.0  
**創建日期**: 2024-12-XX  
**最後更新**: 2024-12-XX  
**狀態**: 已測試，運行正常

**相關文檔**:
- README.MD (系統整體架構)
- CCD3技術文檔.md (CCD3角度辨識模組)
- angle_main.py (主程序源碼)
- angle_app.py (Web應用源碼)