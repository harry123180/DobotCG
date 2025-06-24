# CCD1視覺檢測模組世界座標功能開發規格書 v4.0

## 專案概述
基於現有CCD1VisionCode_Enhanced.py v3.0握手版本，整合NPY格式相機內外參載入功能，實現像素座標到世界座標轉換，並透過Modbus寄存器輸出結果。

## 核心功能需求

### 1. 內外參檔案管理規格

#### 檔案命名規範 (已確認)
```
內參檔案 (嚴格命名):
- camera_matrix_YYYYMMDD_HHMMSS.npy
- dist_coeffs_YYYYMMDD_HHMMSS.npy

外參檔案 (寬鬆命名):
- *extrinsic*.npy  (包含extrinsic字串即可)
```

#### 檔案內容格式
```python
# 內參矩陣: Shape (3,3)
camera_matrix = np.array([
    [fx,  0, cx],
    [ 0, fy, cy], 
    [ 0,  0,  1]
], dtype=np.float64)

# 畸變係數: Shape (5,) 或 (1,5) 或 (5,1)
dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

# 外參檔案: 字典格式
extrinsic_data = {
    'rvec': np.array([[rx], [ry], [rz]]),  # 3×1旋轉向量
    'tvec': np.array([[tx], [ty], [tz]])   # 3×1平移向量
}
```

#### 檔案管理功能
- **掃描位置**: 程式執行檔同層目錄
- **自動掃描**: 系統啟動時自動掃描
- **檔案驗證**: 載入前驗證格式和內容正確性
- **整合方式**: 使用CalibrationManager架構

### 2. 座標轉換規格 (已確認)

#### 轉換範圍
- **轉換類型**: 像素座標 → 世界座標
- **投影平面**: Z=0平面投影（固定）
- **轉換演算法**: OpenCV標準PnP逆投影

#### 精度處理
- **精度要求**: 保留小數點後2位
- **存儲方式**: 世界座標×100轉換為32位有符號整數
- **數值範圍**: ±21474.83mm
- **寄存器分配**: 每個座標使用2個16位寄存器（高位+低位）

#### 轉換實現
```python
# 精度轉換邏輯
world_x_mm = 123.45
world_x_int = int(world_x_mm * 100)  # 12345
world_x_high = (world_x_int >> 16) & 0xFFFF
world_x_low = world_x_int & 0xFFFF

# 恢復邏輯
world_x_int = (world_x_high << 16) | world_x_low
world_x_mm = world_x_int / 100.0  # 123.45
```

### 3. Modbus寄存器擴展規格 (已確認)

#### 新增寄存器映射 (256-275)
```
世界座標檢測結果寄存器 (256-275)
```

| 地址 | 功能 | 說明 |
|------|------|------|
| 256-257 | 圓形1世界X座標 | 高位/低位 (×100) |
| 258-259 | 圓形1世界Y座標 | 高位/低位 (×100) |
| 260-261 | 圓形2世界X座標 | 高位/低位 (×100) |
| 262-263 | 圓形2世界Y座標 | 高位/低位 (×100) |
| 264-265 | 圓形3世界X座標 | 高位/低位 (×100) |
| 266-267 | 圓形3世界Y座標 | 高位/低位 (×100) |
| 268-269 | 圓形4世界X座標 | 高位/低位 (×100) |
| 270-271 | 圓形4世界Y座標 | 高位/低位 (×100) |
| 272-273 | 圓形5世界X座標 | 高位/低位 (×100) |
| 274-275 | 圓形5世界Y座標 | 高位/低位 (×100) |

#### 現有寄存器保持不變
- 像素座標結果寄存器 (240-255) 維持原有功能
- 控制握手寄存器 (200-201) 不變
- 統計資訊寄存器 (280-299) 保持，版本號更新

### 4. 向下兼容處理規格 (已確認)

#### 降級運作策略
- **無標定檔案**: 系統正常運作，只提供像素座標
- **檔案載入失敗**: 不設置Alarm狀態，繼續提供像素座標
- **轉換失敗**: 世界座標寄存器清零，像素座標正常

#### 用戶介面適應
- **有標定**: UI顯示像素座標+世界座標
- **無標定**: UI僅顯示像素座標
- **標定狀態**: 標定管理面板顯示檔案狀態

## 實現架構設計

### CalibrationManager類 (新增)
```python
class CalibrationManager:
    def __init__(self, working_dir):
        self.working_dir = working_dir
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvec = None
        self.tvec = None
        self.transformer = None
        
    def scan_calibration_files(self) -> Dict[str, Any]
    def load_calibration_data(self) -> Dict[str, Any]
    def get_status(self) -> Dict[str, Any]
    def is_calibration_loaded(self) -> bool
```

### CameraCoordinateTransformer類 (新增)
```python
class CameraCoordinateTransformer:
    def __init__(self, camera_matrix, dist_coeffs, rvec, tvec):
        self.K = camera_matrix
        self.D = dist_coeffs
        self.rvec = rvec
        self.tvec = tvec
        
    def pixel_to_world(self, pixel_coords) -> Optional[np.ndarray]
    def is_valid(self) -> bool
```

### VisionResult數據結構擴展
```python
@dataclass
class VisionResult:
    circle_count: int
    circles: List[Dict[str, Any]]  # 包含world_coords欄位
    processing_time: float
    capture_time: float
    total_time: float
    timestamp: str
    success: bool
    has_world_coords: bool = False  # 新增: 世界座標有效性
    error_message: Optional[str] = None
```

## Web界面更新規格

### 標定管理面板 (新增)
```html
<div class="calibration-panel">
    <h3>📐 相機標定管理</h3>
    <div class="calibration-status">
        <div class="status-item">
            <span class="label">內參狀態:</span>
            <span class="value" id="intrinsic-status">未載入</span>
        </div>
        <div class="status-item">
            <span class="label">外參狀態:</span>
            <span class="value" id="extrinsic-status">未載入</span>
        </div>
        <div class="status-item">
            <span class="label">轉換器:</span>
            <span class="value" id="transformer-status">未啟用</span>
        </div>
    </div>
    <div class="calibration-actions">
        <button onclick="scanCalibrationFiles()">🔍 掃描檔案</button>
        <button onclick="loadCalibrationData()">✅ 載入標定</button>
    </div>
</div>
```

### 檢測結果顯示更新
```html
<div class="detection-result">
    <div class="circle-item">
        <div class="circle-header">圓形 1</div>
        <div class="coordinate-group">
            <div class="pixel-coords">
                <span class="coord-label">像素:</span>
                <span class="coord-value">X: 320 px, Y: 240 px</span>
            </div>
            <div class="world-coords" style="display: none;">
                <span class="coord-label">世界:</span>
                <span class="coord-value">X: 123.45 mm, Y: 678.90 mm</span>
            </div>
        </div>
    </div>
</div>
```

## API介面擴展

### 新增REST API端點
```python
# 標定管理API
GET  /api/calibration/scan    # 掃描標定檔案
POST /api/calibration/load    # 載入標定數據
GET  /api/calibration/status  # 獲取標定狀態

# 座標轉換API
POST /api/transform/pixel_to_world  # 手動座標轉換測試
```

### 現有API更新
- `/api/capture_and_detect`: 回應包含世界座標資訊
- `/api/status`: 增加標定狀態資訊
- `/api/modbus/registers`: 增加世界座標寄存器讀取

## 版本升級規格

### 版本資訊更新
- **軟體版本**: v4.0 (寄存器290=4, 291=0)
- **功能標識**: 世界座標轉換功能
- **向下兼容**: 完全兼容v3.0握手協議

### 系統啟動輸出更新
```
CCD1 視覺控制系統啟動中 (運動控制握手版本 v4.0 + 世界座標轉換)
新功能: NPY內外參管理 + 像素座標到世界座標轉換
標定檔案掃描: [工作目錄路徑]
```

## 錯誤處理策略

### 檔案處理錯誤
- 檔案不存在: 記錄警告，繼續運作
- 格式錯誤: 記錄錯誤，跳過該檔案
- 載入失敗: 提供詳細錯誤訊息

### 座標轉換錯誤
- 轉換失敗: 世界座標清零，保留像素座標
- 數值溢出: 限制在±21474.83mm範圍內
- 標定無效: 不執行轉換，正常檢測

## 測試驗證要求

### 功能測試
1. 檔案掃描功能: 正確識別內外參檔案
2. 標定載入功能: 驗證格式和內容正確性
3. 座標轉換功能: 像素座標到世界座標準確性
4. 寄存器映射: 世界座標正確寫入256-275寄存器
5. 向下兼容: 無標定時系統正常運作

### 精度測試
1. 轉換精度: 驗證×100存儲的精度保持
2. 數值範圍: 測試±21474.83mm邊界值
3. 重投影誤差: 驗證轉換演算法準確性

## 部署更新

### 檔案結構
```
Vision/
├── CCD1VisionCode_Enhanced.py     # 主程序 (v4.0)
├── camera_manager.py              # 相機管理API
├── templates/
│   └── ccd_vision_enhanced.html   # Web介面 (更新)
├── camera_matrix_20241210_143022.npy  # 內參矩陣 (範例)
├── dist_coeffs_20241210_143022.npy    # 畸變係數 (範例)
└── extrinsic_20241210_143530.npy      # 外參數據 (範例)
```

### 依賴需求
- 現有依賴保持不變
- OpenCV用於座標轉換計算
- NumPy用於NPY檔案載入和數值處理

## Web用戶界面（WebUI）規格設計

### 界面架構概述
基於現有ccd_vision_enhanced_world_coord.html模板，採用響應式網格布局設計，支持世界座標功能展示。

### 核心UI組件規格

#### 1. 標定管理面板 (calibration-panel)
**位置**: 頁面頂部，全寬度跨欄
**功能**: 內外參檔案管理與世界座標轉換控制

**組件元素**:
```html
<!-- 標定狀態顯示區 -->
<div class="calibration-status">
    <!-- 內參狀態卡片 -->
    <div class="calibration-item" id="intrinsicStatus">
        - 狀態指示: 有效/無效 (綠色/紅色邊框)
        - 檔案資訊: 檔案名稱和時間戳
        - 載入狀態: 已載入/未載入
    </div>
    
    <!-- 外參狀態卡片 -->
    <div class="calibration-item" id="extrinsicStatus">
        - 狀態指示: 有效/無效
        - 檔案資訊: 檔案名稱
        - 格式驗證: 通過/失敗
    </div>
    
    <!-- 轉換器狀態卡片 -->
    <div class="calibration-item" id="transformerStatus">
        - 啟用狀態: 已啟用/未啟用
        - 載入時間: 標定數據載入時間
        - 轉換準備: 就緒/未就緒
    </div>
    
    <!-- 工作目錄資訊 -->
    <div class="calibration-item" id="workingDirStatus">
        - 目錄路徑: 程式執行目錄
        - 檔案掃描: 已掃描檔案數量
    </div>
</div>

<!-- 操作按鈕區 -->
<div class="calibration-actions">
    - 🔍 掃描檔案: 自動掃描同層目錄NPY檔案
    - ✅ 確認導入: 載入已掃描的標定數據
    - 🔄 刷新狀態: 更新標定狀態顯示
</div>

<!-- 說明資訊區 -->
<div class="handshake-info">
    - 內外參檔案命名要求說明
    - 操作步驟指引
    - 格式要求說明
</div>
```

**狀態指示規則**:
- `.calibration-item.valid`: 綠色邊框，淺綠背景
- `.calibration-item.invalid`: 紅色邊框，淺紅背景
- 動態更新檔案名稱和載入狀態

#### 2. 世界座標狀態指示器 (world-coord-indicator)
**位置**: 多個面板標題旁
**功能**: 實時顯示世界座標轉換是否啟用

**顯示狀態**:
```html
<!-- 啟用狀態 -->
<span class="world-coord-indicator enabled">
    🌍 世界座標: 已啟用
</span>

<!-- 未啟用狀態 -->
<span class="world-coord-indicator disabled">
    🌍 世界座標: 未啟用
</span>
```

**樣式規範**:
- 啟用: 青綠色背景，深綠文字，綠色邊框
- 未啟用: 淺紅色背景，深紅文字，紅色邊框

#### 3. 檢測結果雙座標顯示 (circle-coords)
**位置**: 檢測結果面板內
**功能**: 同時顯示像素座標和世界座標

**組件結構**:
```html
<div class="circle-coords">
    <!-- 像素座標組 -->
    <div class="coord-group pixel-coord">
        <div class="coord-label">像素座標</div>
        <div class="coord-value">X: 320 px<br>Y: 240 px</div>
    </div>
    
    <!-- 世界座標組 -->
    <div class="coord-group world-coord">
        <div class="coord-label">世界座標</div>
        <div class="coord-value">
            <!-- 條件顯示 -->
            有標定: X: 123.45 mm<br>Y: 678.90 mm
            無標定: 未啟用世界座標轉換
        </div>
    </div>
</div>
```

**樣式差異**:
- 像素座標: 藍色主題 (.pixel-coord)
- 世界座標: 青色主題 (.world-coord)

#### 4. Modbus寄存器監控擴展
**位置**: 底部寄存器監控面板
**功能**: 新增世界座標寄存器組顯示

**新增寄存器組**:
```html
<div class="register-group">
    <h4>檢測結果-世界座標 (256-276)</h4>
    <!-- 世界座標寄存器項目 -->
    <div class="register-item">
        <span class="register-name">圓形1世界X座標高位</span>
        <span class="register-value">12345</span>
    </div>
    <!-- 更多世界座標寄存器... -->
</div>
```

**高亮規則擴展**:
- 非零的世界座標值: 紅色高亮
- 世界座標有效標誌: 當值為1時高亮

#### 5. 系統狀態面板擴展
**新增狀態項目**:
```html
<div class="info-item">
    <div class="label">世界座標</div>
    <div class="value" id="worldCoordStatus">已啟用/未啟用</div>
</div>

<div class="info-item">
    <div class="label">標定狀態</div>
    <div class="value" id="calibrationStatusText">已載入/未載入</div>
</div>
```

### 交互行為規格

#### 1. 標定檔案管理流程
```javascript
// 操作序列
1. 用戶點擊 "🔍 掃描檔案"
   → 調用 scanCalibrationFiles()
   → 更新檔案狀態顯示
   → 啟用/禁用 "確認導入" 按鈕

2. 用戶點擊 "✅ 確認導入"
   → 調用 loadCalibrationData()
   → 顯示載入進度 (loading spinner)
   → 更新轉換器狀態
   → 更新世界座標指示器

3. 自動刷新機制
   → 頁面載入1秒後自動掃描
   → 每3秒檢查標定狀態
```

#### 2. 世界座標指示器聯動
```javascript
// 全域更新函數
function updateWorldCoordIndicators(enabled) {
    // 更新所有 .world-coord-indicator 元素
    // 更新檢測結果中的世界座標顯示
    // 更新系統狀態中的世界座標狀態
}

// 觸發時機
- 標定數據載入成功/失敗
- 檢測結果返回時
- 系統狀態刷新時
```

#### 3. 檢測結果條件顯示
```javascript
// 世界座標顯示邏輯
if (hasWorldCoords && data.has_world_coords) {
    // 顯示實際世界座標值
    coordinateHTML = `X: ${coords[0].toFixed(2)} mm<br>Y: ${coords[1].toFixed(2)} mm`;
} else {
    // 顯示未啟用提示
    coordinateHTML = '未啟用世界座標轉換';
}
```

### 響應式設計規格

#### 桌面版布局 (>768px)
- 網格布局: 2欄式主要內容
- 標定面板: 全寬跨欄
- 檢測結果: 圖像+結果雙欄顯示
- 寄存器監控: 多欄網格顯示

#### 移動版布局 (≤768px)
- 網格布局: 單欄堆疊
- 所有面板: 全寬顯示
- 座標顯示: 垂直堆疊
- 按鈕組: 縱向排列

### 視覺設計規範

#### 色彩系統
```css
/* 標定相關 */
--calibration-valid: #48bb78 (綠色)
--calibration-invalid: #f56565 (紅色)
--calibration-bg-valid: #f0fff4 (淺綠)
--calibration-bg-invalid: #fef2f2 (淺紅)

/* 世界座標 */
--world-coord-enabled: #4fd1c7 (青色)
--world-coord-disabled: #f56565 (紅色)
--world-coord-bg: #e6fffa (淺青)

/* 座標類型區分 */
--pixel-coord: #4299e1 (藍色)
--world-coord: #4fd1c7 (青色)
```

#### 狀態指示圖標
- 🌍: 世界座標功能
- 📐: 標定管理
- 🔍: 掃描功能
- ✅: 載入成功
- ❌: 載入失敗

### API端點對應

#### 標定管理相關
```javascript
GET  /api/calibration/scan    → scanCalibrationFiles()
POST /api/calibration/load    → loadCalibrationData()
GET  /api/calibration/status  → refreshCalibrationStatus()
```

#### 擴展檢測API
```javascript
POST /api/capture_and_detect  → 返回包含 has_world_coords 欄位
GET  /api/status             → 包含 calibration_status 資訊
GET  /api/modbus/registers   → 包含世界座標寄存器 (256-276)
```

### 錯誤處理UI規範

#### Toast訊息規範
```javascript
// 成功訊息 (綠色邊框)
showToast("標定數據載入成功，世界座標轉換已啟用", true);

// 錯誤訊息 (紅色邊框)
showToast("檔案格式錯誤，請確認NPY檔案格式", false);

// 警告訊息 (橙色邊框)
showToast("未找到外參檔案，僅載入內參數據", false);
```

#### 載入狀態指示
```html
<!-- 載入中按鈕狀態 -->
<button class="btn btn-success" disabled>
    <span class="loading"></span> 載入中...
</button>

<!-- 禁用狀態原因提示 -->
<button disabled title="請先掃描並找到有效的內外參檔案">
    ✅ 確認導入
</button>
```

### 開發實現檢查清單

#### UI組件實現
- [ ] 標定管理面板布局和樣式
- [ ] 世界座標狀態指示器全域更新
- [ ] 檢測結果雙座標顯示邏輯
- [ ] Modbus寄存器監控擴展
- [ ] 系統狀態面板新增項目

#### 交互功能實現
- [ ] 標定檔案掃描和載入流程
- [ ] 世界座標指示器聯動機制
- [ ] 檢測結果條件顯示
- [ ] 錯誤處理和用戶反饋
- [ ] 響應式布局適配

#### 樣式和視覺實現
- [ ] 色彩系統定義和應用
- [ ] 狀態指示樣式規範
- [ ] 圖標和視覺元素
- [ ] 動畫和過渡效果
- [ ] 移動端適配優化

## 開發檢查清單

### 必須實現功能
- [ ] CalibrationManager類實現
- [ ] CameraCoordinateTransformer類實現
- [ ] 檔案掃描和載入功能
- [ ] 座標轉換邏輯
- [ ] 世界座標寄存器映射 (256-275)
- [ ] Web界面標定管理面板
- [ ] API端點擴展
- [ ] 向下兼容處理

### 品質保證
- [ ] 錯誤處理完善
- [ ] 日誌記錄適當
- [ ] 記憶體管理優化
- [ ] 線程安全確保
- [ ] 測試用例覆蓋

此規格書作為CCD1模組v4.0世界座標功能開發的技術基準，所有實現必須嚴格遵循此規格要求。