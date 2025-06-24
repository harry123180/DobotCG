# CCD3角度辨識模組技術文檔

## 架構概述

CCD3角度辨識模組實現Ring物件角度檢測，採用Modbus TCP Client架構，整合既有opencv_detect_module.py核心算法。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> CCD3AngleDetection.py (TCP Client)
                    |
                    |-- 直連 --> 相機設備 (192.168.1.10)
                    |-- 算法 --> opencv_detect_module.py核心算法
```

## 實現組件

### CCD3AngleDetection.py - 主模組
- TCP Client連接主服務器 (127.0.0.1:502)
- 相機直連控制 (192.168.1.10)
- 寄存器映射基地址: 800
- 握手式狀態機交握
- 50ms輪詢間隔
- 整合opencv_detect_module.py角度檢測算法

### angle_detector.py - 角度檢測算法封裝
- 基於opencv_detect_module.py核心算法
- 支援橢圓擬合和最小外接矩形兩種模式
- 可調整檢測參數
- 增強版影像前處理

### templates/ccd3_angle_detection.html - Web介面
- 簡約白藍漸層設計風格
- 即時狀態監控
- 參數調整介面
- 寄存器監控功能

## 寄存器映射 (基地址800)

### 核心控制握手寄存器 (800-801)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 800 | 控制指令 | 0=清空, 8=拍照, 16=拍照+角度檢測, 32=重新初始化 |
| 801 | 狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized |

### 檢測參數寄存器 (810-819)
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 810 | 檢測模式 | 0=橢圓擬合, 1=最小外接矩形 | 算法模式選擇 |
| 811 | 最小面積比例 | ×1000存儲 | 0.05 → 50 |
| 812 | 序列模式 | 0=最大輪廓, 1=序列輪廓 | 輪廓選擇方式 |
| 813 | 高斯模糊核大小 | 奇數值 | 3, 5, 7, 9等 |
| 814 | 閾值處理模式 | 0=OTSU自動, 1=手動 | 二值化方式 |
| 815 | 手動閾值 | 0-255 | 手動模式閾值 |
| 816-819 | 保留參數 | - | 未來擴展 |

### 角度檢測結果寄存器 (840-859)
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 840 | 檢測成功標誌 | 0=失敗, 1=成功 | 結果有效性 |
| 841 | 物體中心X座標 | 像素座標 | 檢測物體中心 |
| 842 | 物體中心Y座標 | 像素座標 | 檢測物體中心 |
| 843 | 角度高位 | 32位角度高16位 | 角度×100存儲 |
| 844 | 角度低位 | 32位角度低16位 | 角度×100存儲 |
| 845 | 長軸長度 | 像素單位 | 橢圓模式有效 |
| 846 | 短軸長度 | 像素單位 | 橢圓模式有效 |
| 847 | 矩形寬度 | 像素單位 | 矩形模式有效 |
| 848 | 矩形高度 | 像素單位 | 矩形模式有效 |
| 849 | 檢測輪廓面積 | 像素平方 | 物體面積 |
| 850 | 內徑高位 | 32位內徑高16位 | 預留擴展 |
| 851 | 內徑低位 | 32位內徑低16位 | 預留擴展 |
| 852 | 外徑高位 | 32位外徑高16位 | 預留擴展 |
| 853 | 外徑低位 | 32位外徑低16位 | 預留擴展 |
| 854-859 | 保留結果 | - | 未來擴展 |

### 統計資訊寄存器 (880-899)
| 地址 | 功能 | 說明 |
|------|------|------|
| 880 | 最後拍照耗時 | 毫秒單位 |
| 881 | 最後處理耗時 | 毫秒單位 |
| 882 | 最後總耗時 | 毫秒單位 |
| 883 | 操作計數器 | 累積成功次數 |
| 884 | 錯誤計數器 | 累積錯誤次數 |
| 885 | 連接計數器 | 累積連接次數 |
| 890 | 軟體版本主號 | 版本3 |
| 891 | 軟體版本次號 | 版本0 |
| 892 | 運行時間小時 | 系統運行時間 |
| 893 | 運行時間分鐘 | 系統運行時間 |
| 894-899 | 保留統計 | 未來擴展 |

## 握手協議實現

### 狀態機定義
SystemStateMachine類實現4位狀態控制:
- Ready (bit0): 系統準備接受新指令
- Running (bit1): 系統正在執行操作
- Alarm (bit2): 系統異常或錯誤
- Initialized (bit3): 系統已完全初始化

### 握手交握流程
1. 系統初始化完成 → 狀態寄存器固定值1 (Ready=1)
2. PLC下達控制指令 → 檢查Ready=1
3. 開始執行 → Ready=0, Running=1
4. 執行完成 → Running=0
5. PLC清零指令 → Ready=1 (準備下次)
6. 異常發生 → Alarm=1, Initialized=0

### 控制指令映射
| 指令碼 | 功能 | 執行內容 |
|--------|------|----------|
| 0 | 清空控制 | 狀態機恢復Ready |
| 8 | 拍照 | 單純圖像捕獲 |
| 16 | 拍照+角度檢測 | 圖像捕獲與角度檢測 |
| 32 | 重新初始化 | 相機重新初始化 |

## 角度檢測算法

### EnhancedAngleDetector類實現
基於opencv_detect_module.py核心算法，提供以下增強功能:
- 可調整高斯濾波參數
- 可選OTSU或手動閾值
- 輪廓選擇模式控制
- 面積比例參數調整

### 檢測模式說明

#### 橢圓擬合模式 (mode=0)
- 基於opencv_detect_module.py的get_obj_angle(mode=0)
- 適用於圓形或橢圓形Ring物件
- 輸出長軸、短軸資訊
- 提供更精確的角度檢測

#### 最小外接矩形模式 (mode=1)
- 基於opencv_detect_module.py的get_obj_angle(mode=1)
- 適用於矩形或不規則Ring物件
- 輸出矩形寬度、高度資訊
- 處理速度較快

### 檢測參數配置
```python
class AngleDetectionParams:
    detection_mode: int = 0          # 檢測模式
    min_area_rate: float = 0.05      # 最小面積比例
    sequence_mode: int = 0           # 輪廓選擇模式
    gaussian_kernel: int = 3         # 高斯模糊核大小
    threshold_mode: int = 0          # 閾值處理模式
    manual_threshold: int = 127      # 手動閾值
```

### 結果數據結構
```python
class AngleDetectionResult:
    success: bool                    # 檢測成功標誌
    center: Tuple[int, int]         # 物體中心座標
    angle: float                     # 檢測角度(-180~180度)
    major_axis: float               # 長軸長度(橢圓模式)
    minor_axis: float               # 短軸長度(橢圓模式)
    rect_width: float               # 矩形寬度(矩形模式)
    rect_height: float              # 矩形高度(矩形模式)
    contour_area: float             # 輪廓面積
    processing_time: float          # 處理耗時
    inner_diameter_high: int        # 內徑高位(預留)
    inner_diameter_low: int         # 內徑低位(預留)
    outer_diameter_high: int        # 外徑高位(預留)
    outer_diameter_low: int         # 外徑低位(預留)
```

## 相機管理實現

### OptimizedCamera類整合
- 使用Automation/API/camera_manager.py
- 海康威視SDK封裝
- 單相機完整生命週期管理
- 幀緩存機制
- 性能監控統計

### CameraConfig配置
```python
class CameraConfig:
    name: str = "ccd3_camera"
    ip: str = "192.168.1.10"
    exposure_time: float = 20000.0
    gain: float = 200.0
    frame_rate: float = 30.0
    width: int = 2592
    height: int = 1944
```

## 線程架構

### 主要線程
- **主線程**: Flask Web應用
- **握手同步線程**: _handshake_sync_loop (50ms輪詢)
- **指令執行線程**: _execute_command_async (異步執行)

### 線程安全機制
- SystemStateMachine使用self.lock保護狀態
- 指令執行狀態command_processing防重入
- 相機操作使用threading.RLock

## Flask Web介面

### 核心API路由
- POST /api/modbus/set_server - 設置Modbus服務器地址
- POST /api/modbus/connect - 連接Modbus服務器
- GET /api/modbus/registers - 讀取所有寄存器即時數值
- POST /api/initialize - 初始化相機連接
- POST /api/capture_and_detect - 執行拍照+角度檢測
- GET /api/status - 獲取系統狀態

### Web介面功能
- 運行在localhost:5052
- SocketIO即時通訊
- 狀態監控顯示
- 檢測參數調整介面
- 手動控制功能
- 寄存器監控顯示

## 依賴模組版本

### 核心依賴
- PyModbus 3.9.2 (Modbus TCP Client)
- OpenCV (圖像處理)
- NumPy (數值計算)
- Flask + SocketIO (Web介面)

### 相機管理
- camera_manager.py (來自Automation/API/)
- 海康威視SDK支援

### 算法依賴
- opencv_detect_module.py (核心角度檢測算法)

## 32位角度精度處理

### 角度存儲機制
```python
# 角度轉換為32位整數存儲 (保留2位小數)
angle_int = int(angle_degrees * 100)
angle_high = (angle_int >> 16) & 0xFFFF  # 高16位 → 843寄存器
angle_low = angle_int & 0xFFFF           # 低16位 → 844寄存器

# 寄存器恢復為角度值
angle_int = (angle_high << 16) | angle_low
final_angle = angle_int / 100.0  # 恢復2位小數精度
```

### 角度範圍支援
- 32位有符號整數: ±2,147,483,647
- 角度範圍: ±21,474,836.47度
- 實際使用: ±180度足夠使用
- 精度: 0.01度

## 錯誤處理機制

### 連接管理
- Modbus TCP自動重連 (reconnect_delay: 5秒)
- 相機連接異常處理
- 超時控制 (read_timeout: 3秒)

### 狀態異常處理
- 通訊錯誤計數統計
- Alarm狀態自動設置
- 錯誤資訊記錄與回報

### 異常恢復
- 重新初始化指令 (32)
- 狀態機重置機制
- 資源釋放與重建

## 配置管理

### ccd3_config.json配置檔案
```json
{
  "module_id": "CCD3_Angle_Detection",
  "camera_config": {
    "name": "ccd3_camera",
    "ip": "192.168.1.10",
    "exposure_time": 20000.0,
    "gain": 200.0,
    "frame_rate": 30.0,
    "width": 2592,
    "height": 1944
  },
  "tcp_server": {
    "host": "127.0.0.1",
    "port": 502,
    "unit_id": 1
  },
  "modbus_mapping": {
    "base_address": 800
  },
  "detection_params": {
    "min_area_rate": 50,
    "sequence_mode": 0,
    "gaussian_kernel": 3,
    "threshold_mode": 0,
    "manual_threshold": 127
  }
}
```

### 配置檔案生成
- 執行檔同層目錄自動生成
- 首次運行創建預設配置
- 參數可通過Web介面動態調整

## 部署配置

### 運行順序
1. 啟動主Modbus TCP Server (端口502)
2. 啟動CCD3AngleDetection.py
3. 訪問Web介面 (localhost:5052)
4. 設置Modbus服務器地址並連接
5. 初始化相機連接 (192.168.1.10)
6. 系統進入握手模式等待PLC指令

### 檔案結構
```
CCD3/
├── CCD3AngleDetection.py          # 主程序
├── angle_detector.py              # 角度檢測算法封裝
├── ccd3_config.json               # 配置檔案 (自動生成)
└── templates/
    └── ccd3_angle_detection.html  # Web介面模板
```

## 測試驗證

### 連接測試
1. Modbus TCP連接狀態檢查
2. 相機設備連接驗證 (192.168.1.10)
3. 寄存器讀寫功能測試

### 功能測試
1. 握手協議狀態機驗證
2. 控制指令執行確認
3. 角度檢測演算法準確性
4. 雙模式檢測對比
5. 錯誤處理機制測試

### 性能監控
1. 拍照耗時統計
2. 角度檢測處理時間
3. 握手響應延遲
4. 系統穩定性驗證

## 開發實現特點

### 算法整合
- 完全保留opencv_detect_module.py核心算法邏輯
- 不修改原始算法，僅進行封裝整合
- 增加參數化控制接口
- 提供結構化結果輸出

### 架構設計
- 遵循既有模組設計模式
- 統一的握手協議實現
- 標準化寄存器映射
- 模組化組件設計

### 記憶體管理
- daemon模式線程自動回收
- 握手同步線程正確退出機制
- 相機資源釋放管理
- numpy數組記憶體回收

## 已知限制

### 硬體依賴
- 需要海康威視SDK完整安裝
- 相機必須支援TCP/IP連接 (192.168.1.10)
- 網路延遲影響響應時間

### 軟體限制
- 基於opencv_detect_module.py算法能力
- 單相機支援
- 同步執行模式

### 系統限制
- 50ms握手輪詢頻率
- 內外徑檢測功能待整合
- 無持久化配置機制

## 擴展規劃

### 內外徑檢測整合
- 寄存器850-853已預留
- 等待內外徑核心算法開發完成
- 整合到angle_detector.py中

### 功能擴展
- 多Ring物件同時檢測
- 檢測結果歷史記錄
- 檢測精度統計分析

## 運行狀態輸出

### 系統啟動輸出
```
CCD3角度辨識系統啟動中...
系統架構: Modbus TCP Client - 運動控制握手模式
基地址: 800
相機IP: 192.168.1.10
Web介面啟動中... http://localhost:5052
```

### 握手協議運行日誌
```
收到新控制指令: 16 (上次: 0)
開始處理控制指令: 16
執行拍照+角度檢測指令
角度檢測完成: 中心(1296, 972), 角度45.67度
控制指令 16 執行完成
恢復Ready狀態
```

### Web介面狀態監控
- 即時連接狀態顯示
- 檢測參數即時調整
- 檢測結果視覺化顯示
- 寄存器數值即時監控

## API介面文檔

### CCD3AngleDetectionService關鍵方法
```python
def connect_modbus(self) -> bool
def initialize_camera(self, ip_address: str) -> bool
def capture_and_detect_angle(self, mode: int) -> AngleResult
def read_detection_parameters(self) -> Dict[str, Any]
def write_detection_result(self, result: AngleResult)
def start_handshake_service(self)
def disconnect(self)
```

### SystemStateMachine狀態控制
```python
def is_ready(self) -> bool
def is_running(self) -> bool
def is_alarm(self) -> bool
def is_initialized(self) -> bool
def set_ready(self, ready: bool)
def set_running(self, running: bool)
def reset_to_idle(self)
```

### EnhancedAngleDetector檢測方法
```python
def detect_angle(self, image) -> AngleDetectionResult
def update_params(self, **kwargs)
def get_params_dict(self) -> Dict[str, Any]
def set_params_from_registers(self, registers: list)
```

## 與其他模組差異

### 與CCD1圓形檢測對比
| 特性 | CCD1 | CCD3 |
|------|------|------|
| 檢測對象 | 圓形物件 | Ring物件角度 |
| 基地址 | 200 | 800 |
| 輸出結果 | 座標+半徑 | 座標+角度 |
| 檢測模式 | 圓形檢測 | 雙模式角度檢測 |
| 相機IP | 192.168.1.8 | 192.168.1.10 |
| Web端口 | 5051 | 5052 |

### 寄存器映射對比
- CCD1: 200-299 (圓形檢測)
- CCD3: 800-899 (角度檢測)
- 握手協議: 完全一致
- 狀態機邏輯: 相同實現
- 精度處理: 統一32位標準

## 故障排除

### 常見問題
1. **相機連接失敗**: 檢查192.168.1.10網路連通性
2. **Modbus連接失敗**: 確認主服務器127.0.0.1:502運行
3. **角度檢測失敗**: 檢查Ring物件是否在視野內
4. **握手異常**: 檢查Ready狀態和指令序列

### 除錯方法
1. Web介面狀態監控
2. 寄存器數值檢查
3. 系統日誌分析
4. 統計資訊追蹤

### 效能優化
1. 調整檢測參數減少處理時間
2. 優化高斯模糊核大小
3. 使用適當的閾值處理模式
4. 監控記憶體使用情況

---

**文檔版本**: v1.0  
**創建日期**: 2024-12-XX  
**最後更新**: 2024-12-XX  
**負責人**: 開發團隊

**相關文檔**:
- CCD3開發需求.md
- opencv_detect_module.py
- README.MD (系統整體架構)
- camera_manager.py (API文檔)