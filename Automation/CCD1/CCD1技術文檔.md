# CCD視覺檢測模組技術文檔 v4.1

## 架構概述

CCD視覺檢測模組實現握手式狀態機控制，採用Modbus TCP Client架構，v4.1版本在v4.0世界座標轉換功能基礎上新增保護範圍過濾功能，並提供CCD1HighLevel.py高層API簡化使用。

```
主服務器 (ModbusTCP:502)
    |
    |-- TCP --> CCD1VisionCode_Enhanced.py (TCP Client)
    |            |
    |            |-- 直連 --> 相機設備 (192.168.1.8)
    |            |-- 檔案 --> 內外參NPY檔案 (同層目錄)
    |            |-- 過濾 --> 保護範圍過濾器 (v4.1新增)
    |
    |-- TCP --> CCD1HighLevel.py (高層API Client) ⭐新增⭐
                 |
                 |-- FIFO佇列 --> 圓心座標管理
                 |-- 自動觸發 --> 拍照+檢測指令
                 |-- 簡化介面 --> 其他模組調用
```

## 實現組件

### CCD1VisionCode_Enhanced.py - 主模組 (v4.1)
- TCP Client連接主服務器 (默認127.0.0.1:502)
- 相機直連控制 (192.168.1.8)
- 寄存器映射基地址: 200
- 握手式狀態機交握
- 50ms輪詢間隔
- **v4.0**: 內外參檔案管理
- **v4.0**: 世界座標轉換功能
- **v4.1新增**: 保護範圍過濾功能

### CCD1HighLevel.py - 高層API模組 ⭐新增⭐
- 提供簡化的CCD1功能介面
- 處理複雜的ModbusTCP握手協議
- FIFO佇列管理圓心座標
- 適用於其他模組import使用
- 自動觸發拍照+檢測功能
- 世界座標數據結構化管理

### camera_manager.py - 相機管理API
- 海康威視SDK封裝
- 優化相機管理器
- 幀數據處理
- 性能監控

### CalibrationManager - 標定管理器 (v4.0)
- 內外參NPY檔案掃描
- 檔案格式驗證
- 標定數據載入
- 座標轉換器管理

### CameraCoordinateTransformer - 座標轉換器 (v4.0)
- 像素座標到世界座標轉換
- 基於OpenCV實現
- Z=0平面投影
- 畸變校正處理

### ProtectionZoneConfig - 保護範圍配置 (v4.1新增)
- 保護範圍啟用/關閉控制
- X軸範圍設定 (x_min, x_max)
- Y軸範圍設定 (y_min, y_max)
- 預設範圍: X(-122.0~-4.0mm), Y(243.0~341.0mm)

## CCD1高層API使用說明 ⭐新增⭐

### 概述
CCD1HighLevel.py提供簡化的CCD1功能介面，封裝複雜的ModbusTCP握手協議和FIFO佇列管理，適用於其他模組快速整合CCD1視覺檢測功能。

### 主要功能
1. **自動拍照+檢測**: 透過握手協議自動觸發檢測
2. **FIFO佇列管理**: 圓心座標先進先出管理
3. **世界座標支援**: 支援像素座標與世界座標
4. **簡化介面**: 一行代碼獲取下個物件座標
5. **自動重連**: 連接異常自動恢復

### 核心類別

#### CCD1HighLevelAPI
```python
class CCD1HighLevelAPI:
    """
    CCD1高層API - 簡化CCD1功能使用
    
    主要功能:
    1. 拍照+檢測指令 (自動處理握手協議)
    2. 獲取物件圓心世界座標 (FIFO佇列管理)
    """
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502)
    def get_next_circle_world_coord(self) -> Optional[CircleWorldCoord]
    def capture_and_detect(self) -> bool
    def get_queue_status(self) -> Dict[str, Any]
    def clear_queue(self)
    def is_ready(self) -> bool
```

#### CircleWorldCoord數據結構
```python
@dataclass
class CircleWorldCoord:
    """圓心世界座標數據"""
    id: int                    # 圓形ID
    world_x: float            # 世界座標X (mm)
    world_y: float            # 世界座標Y (mm)
    pixel_x: int              # 像素座標X
    pixel_y: int              # 像素座標Y
    radius: int               # 半徑 (像素)
    timestamp: str            # 檢測時間戳
```

### API方法說明

#### 1. get_next_circle_world_coord()
**功能**: 獲取下一個物件圓心世界座標
**邏輯**: 
- 如果佇列為空，自動觸發拍照+檢測
- 從佇列前端取出一個座標 (FIFO)
- 返回座標，佇列中移除該座標

```python
# 使用範例
ccd1 = CCD1HighLevelAPI()
coord = ccd1.get_next_circle_world_coord()

if coord:
    print(f"圓心{coord.id}: 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm")
    print(f"像素座標=({coord.pixel_x}, {coord.pixel_y}), 半徑={coord.radius}")
else:
    print("無可用座標")
```

#### 2. capture_and_detect()
**功能**: 手動執行拍照+檢測指令
**協議**: 處理完整的握手協議
- 檢查Ready狀態
- 發送拍照+檢測指令 (16)
- 等待執行完成
- 讀取檢測結果並更新FIFO佇列

```python
# 使用範例
success = ccd1.capture_and_detect()
if success:
    print("檢測完成，結果已加入佇列")
else:
    print("檢測失敗")
```

#### 3. get_queue_status()
**功能**: 獲取佇列狀態資訊
**返回**: 佇列長度、最後檢測數量、佇列預覽

```python
# 使用範例
status = ccd1.get_queue_status()
print(f"佇列長度: {status['queue_length']}")
print(f"最後檢測數量: {status['last_detection_count']}")
print(f"連接狀態: {status['connected']}")
```

#### 4. get_system_status()
**功能**: 獲取CCD1系統狀態
**返回**: Ready、Running、Alarm、世界座標有效性等狀態

```python
# 使用範例
status = ccd1.get_system_status()
print(f"Ready: {status['ready']}")
print(f"世界座標有效: {status['world_coord_valid']}")
```

### 握手協議處理

#### 自動握手流程
CCD1HighLevel.py內部自動處理握手協議，開發者無需關心細節：

1. **等待Ready狀態**: `_wait_for_ready()`
2. **發送檢測指令**: 控制指令16 (拍照+檢測)
3. **等待執行完成**: `_wait_for_command_complete()`
4. **讀取檢測結果**: `_read_world_coordinates()`
5. **更新FIFO佇列**: 自動管理佇列
6. **清空控制指令**: 完成握手循環

#### 寄存器映射 (由高層API自動處理)
```python
self.REGISTERS = {
    'CONTROL_COMMAND': 200,        # 控制指令
    'STATUS_REGISTER': 201,        # 狀態寄存器
    'CIRCLE_COUNT': 240,           # 檢測圓形數量
    'WORLD_COORD_VALID': 256,      # 世界座標有效標誌
}
```

### 使用範例

#### 基本使用範例
```python
from CCD1HighLevel import CCD1HighLevelAPI

# 創建CCD1高層API實例
ccd1 = CCD1HighLevelAPI()

try:
    # 檢查系統狀態
    status = ccd1.get_system_status()
    print(f"系統狀態: {status}")
    
    # 逐一獲取圓心座標
    for i in range(5):  # 嘗試獲取5個座標
        coord = ccd1.get_next_circle_world_coord()
        if coord:
            print(f"圓心{coord.id}: 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm")
        else:
            print(f"第{i+1}次獲取座標失敗")
            break
    
finally:
    # 清理資源
    ccd1.disconnect()
```

#### 整合到其他模組範例
```python
# 機械臂模組整合範例
class RobotArmController:
    def __init__(self):
        self.ccd1 = CCD1HighLevelAPI()
    
    def pick_next_object(self):
        """抓取下一個物件"""
        # 獲取物件座標
        coord = self.ccd1.get_next_circle_world_coord()
        
        if coord:
            # 移動到物件位置
            self.move_to_position(coord.world_x, coord.world_y)
            # 執行抓取
            self.grab_object()
            return True
        else:
            print("未找到可抓取的物件")
            return False
    
    def batch_pick_objects(self, count: int):
        """批量抓取物件"""
        for i in range(count):
            if not self.pick_next_object():
                break
            print(f"成功抓取第{i+1}個物件")
```

### 錯誤處理

#### 連接管理
```python
# 自動重連機制
def connect(self) -> bool:
    try:
        self.modbus_client = ModbusTcpClient(
            host=self.modbus_host,
            port=self.modbus_port,
            timeout=3.0
        )
        
        if self.modbus_client.connect():
            self.connected = True
            return True
    except Exception as e:
        self.logger.error(f"連接異常: {e}")
        return False
```

#### 超時處理
```python
# 操作超時控制
def _wait_for_ready(self, timeout: float = 10.0) -> bool:
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        status = self._read_register('STATUS_REGISTER')
        if status is not None:
            ready = bool(status & (1 << CCD1StatusBits.READY))
            if ready:
                return True
        time.sleep(0.1)
    
    return False
```

#### 異常狀態檢查
```python
# Alarm狀態檢查
if alarm:
    self.logger.warning("CCD1系統處於Alarm狀態")
    return False
```

### 佇列管理機制

#### FIFO佇列實現
```python
# 線程安全的佇列操作
with self.queue_lock:
    for coord in coordinates:
        self.coord_queue.append(coord)  # 加入佇列尾端

    coord = self.coord_queue.popleft()  # 從佇列前端取出
```

#### 佇列狀態監控
```python
def get_queue_status(self) -> Dict[str, Any]:
    with self.queue_lock:
        return {
            'queue_length': len(self.coord_queue),
            'last_detection_count': self.last_detection_count,
            'queue_preview': self._get_queue_preview()
        }
```

### 部署配置

#### 檔案結構
```
Vision/
├── CCD1VisionCode_Enhanced.py    # 主程序 (v4.1)
├── CCD1HighLevel.py              # 高層API模組 ⭐新增⭐
├── camera_manager.py             # 相機管理API
├── templates/
│   └── ccd_vision_enhanced_world_coord.html  # Web介面 (v4.1)
├── camera_matrix_20241210_143022.npy         # 內參矩陣 (範例)
├── dist_coeffs_20241210_143022.npy           # 畸變係數 (範例)
└── extrinsic_20241210_143530.npy             # 外參數據 (範例)
```

#### 運行順序 (使用高層API)
1. 啟動主Modbus TCP Server (端口502)
2. 準備內外參NPY檔案 (放入程式同層目錄)
3. 啟動CCD1VisionCode_Enhanced.py (主模組)
4. 在其他模組中import CCD1HighLevel.py
5. 創建CCD1HighLevelAPI實例
6. 直接調用get_next_circle_world_coord()

#### 依賴需求
```python
# CCD1HighLevel.py依賴
from pymodbus.client import ModbusTcpClient  # PyModbus 3.9.2
import time
import threading
from typing import Optional, Tuple, List, Dict, Any
from collections import deque
from enum import IntEnum
import logging
from dataclasses import dataclass
```

## 寄存器映射 (基地址200) - v4.1擴展版本

### 核心控制握手寄存器 (200-201)
| 地址 | 功能 | 數值定義 |
|------|------|----------|
| 200 | 控制指令 | 0=清空, 8=拍照, 16=拍照+檢測+保護範圍過濾, 32=重新初始化 |
| 201 | 狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized |

### 檢測參數寄存器 (210-219)
| 地址 | 功能 | 說明 |
|------|------|------|
| 210 | 最小面積高位 | 32位面積設定高16位 |
| 211 | 最小面積低位 | 32位面積設定低16位 |
| 212 | 最小圓度 | 圓度設定值乘以1000 |
| 213 | 高斯核大小 | 圖像處理參數 |
| 214 | Canny低閾值 | 邊緣檢測參數 |
| 215 | Canny高閾值 | 邊緣檢測參數 |

### 像素座標檢測結果寄存器 (240-255)
| 地址 | 功能 | 說明 |
|------|------|------|
| 240 | 檢測圓形數量 | 最多5個 (已過濾的數量) |
| 241-243 | 圓形1像素座標半徑 | X, Y, Radius |
| 244-246 | 圓形2像素座標半徑 | X, Y, Radius |
| 247-249 | 圓形3像素座標半徑 | X, Y, Radius |
| 250-252 | 圓形4像素座標半徑 | X, Y, Radius |
| 253-255 | 圓形5像素座標半徑 | X, Y, Radius |

### 世界座標檢測結果寄存器 (256-276) - v4.0
| 地址 | 功能 | 說明 |
|------|------|------|
| 256 | 世界座標有效標誌 | 0=無效, 1=有效 |
| 257 | 圓形1世界X座標高位 | 32位世界X座標高16位 (×100) |
| 258 | 圓形1世界X座標低位 | 32位世界X座標低16位 (×100) |
| 259 | 圓形1世界Y座標高位 | 32位世界Y座標高16位 (×100) |
| 260 | 圓形1世界Y座標低位 | 32位世界Y座標低16位 (×100) |
| 261-262 | 圓形2世界X座標 | 高位/低位 (×100) |
| 263-264 | 圓形2世界Y座標 | 高位/低位 (×100) |
| 265-266 | 圓形3世界X座標 | 高位/低位 (×100) |
| 267-268 | 圓形3世界Y座標 | 高位/低位 (×100) |
| 269-270 | 圓形4世界X座標 | 高位/低位 (×100) |
| 271-272 | 圓形4世界Y座標 | 高位/低位 (×100) |
| 273-274 | 圓形5世界X座標 | 高位/低位 (×100) |
| 275-276 | 圓形5世界Y座標 | 高位/低位 (×100) |

### 保護範圍寄存器 (294-299, 277-279) - v4.1新增
| 地址 | 功能 | 說明 |
|------|------|------|
| 294 | 保護範圍啟用標誌 | 0=關閉, 1=啟用 |
| 295 | X最小值高位 | 32位X最小值高16位 (×100精度) |
| 296 | X最小值低位 | 32位X最小值低16位 (×100精度) |
| 297 | X最大值高位 | 32位X最大值高16位 (×100精度) |
| 298 | X最大值低位 | 32位X最大值低16位 (×100精度) |
| 299 | Y最小值高位 | 32位Y最小值高16位 (×100精度) |
| 277 | Y最小值低位 | 32位Y最小值低16位 (×100精度) |
| 278 | Y最大值高位 | 32位Y最大值高16位 (×100精度) |
| 279 | Y最大值低位 | 32位Y最大值低16位 (×100精度) |

### 統計資訊寄存器 (280-299) - v4.1更新
| 地址 | 功能 | 說明 |
|------|------|------|
| 280 | 最後拍照耗時 | 毫秒單位 |
| 281 | 最後處理耗時 | 毫秒單位 |
| 282 | 最後總耗時 | 毫秒單位 |
| 283 | 操作計數器 | 累積操作次數 |
| 284 | 有效物件數量 | 有效範圍內物件數量 (v4.1更新) |
| 285 | 過濾物件數量 | 被過濾掉的物件數量 (v4.1更新) |
| 290 | 軟體版本主號 | 版本4 (v4.1保護範圍功能) |
| 291 | 軟體版本次號 | 版本1 |
| 292 | 運行時間小時 | 系統運行時間 |
| 293 | 運行時間分鐘 | 系統運行時間 |

## 保護範圍過濾功能 (v4.1新增)

### 功能概述
保護範圍過濾功能基於世界座標系統，可以設定有效檢測區域，自動過濾範圍外的物件，確保只輸出有效範圍內的檢測結果。

### 保護範圍配置
```python
@dataclass
class ProtectionZoneConfig:
    enabled: bool = False          # 啟用/關閉保護範圍
    x_min: float = -122.0         # X軸最小值 (mm)
    x_max: float = -4.0           # X軸最大值 (mm)
    y_min: float = 243.0          # Y軸最小值 (mm)
    y_max: float = 341.0          # Y軸最大值 (mm)
```

### 過濾邏輯實現
```python
def _is_in_protection_zone(self, world_x: float, world_y: float) -> bool:
    """檢查座標是否在保護範圍內"""
    if not self.protection_zone.enabled:
        return True  # 保護範圍關閉時，所有物件都有效
    
    x_in_range = self.protection_zone.x_min <= world_x <= self.protection_zone.x_max
    y_in_range = self.protection_zone.y_min <= world_y <= self.protection_zone.y_max
    
    return x_in_range and y_in_range

def _filter_circles_by_protection_zone(self, circles: List[Dict], has_world_coords: bool) -> Tuple[List[Dict], int, int]:
    """根據保護範圍過濾圓形"""
    if not self.protection_zone.enabled or not has_world_coords:
        return circles, len(circles), 0
    
    valid_circles = []
    filtered_count = 0
    
    for circle in circles:
        if 'world_coords' in circle:
            world_x, world_y = circle['world_coords']
            if self._is_in_protection_zone(world_x, world_y):
                circle['in_protection_zone'] = True
                valid_circles.append(circle)
            else:
                circle['in_protection_zone'] = False
                filtered_count += 1
        else:
            # 沒有世界座標的情況，保持原邏輯
            valid_circles.append(circle)
    
    return valid_circles, len(valid_circles), filtered_count
```

### 過濾統計輸出
- **original_count**: 原始檢測到的物件數量
- **valid_count**: 有效範圍內物件數量
- **filtered_count**: 被過濾掉的物件數量

## 世界座標轉換功能 (v4.0維持)

### 內外參檔案管理

#### 檔案格式要求
```python
# 內參檔案 (規範命名)
camera_matrix_YYYYMMDD_HHMMSS.npy    # 3×3內參矩陣
dist_coeffs_YYYYMMDD_HHMMSS.npy      # 畸變係數陣列

# 外參檔案 (較寬鬆命名)
extrinsic_*.npy                      # 包含rvec和tvec的字典格式
*extrinsic*.npy                      # 其他包含extrinsic的檔案
```

#### 檔案格式驗證
```python
# 內參驗證
camera_matrix.shape == (3, 3)        # 內參矩陣必須為3×3
len(dist_coeffs) >= 4                # 畸變係數至少4個參數

# 外參驗證
extrinsic_data = {
    'rvec': np.array([[rx], [ry], [rz]]),    # 3×1旋轉向量
    'tvec': np.array([[tx], [ty], [tz]])     # 3×1平移向量
}
```

### 座標轉換實現

#### CameraCoordinateTransformer類
```python
class CameraCoordinateTransformer:
    def __init__(self, camera_matrix, dist_coeffs, rvec, tvec):
        self.K = camera_matrix
        self.D = dist_coeffs
        self.rvec = rvec.reshape(3, 1)
        self.tvec = tvec.reshape(3, 1)
        self.R, _ = cv2.Rodrigues(self.rvec)
```

#### 轉換流程
1. **去畸變處理**: `cv2.undistortPoints()`
2. **歸一化座標**: `K^(-1) * [u, v, 1]`
3. **深度計算**: `s = -t_z / (R3 · normalized_coords)`
4. **世界座標**: `R^(-1) * (s * normalized_coords - t)`

#### 轉換數學模型
```python
def pixel_to_world(self, pixel_coords):
    # 步驟1: 去畸變
    undistorted_uv = cv2.undistortPoints(
        pixel_coords.reshape(1, 1, 2), self.K, self.D, P=self.K
    ).reshape(-1)
    
    # 步驟2: 歸一化座標
    normalized_coords = np.linalg.inv(self.K) @ [u, v, 1]
    
    # 步驟3: 計算深度係數 (Z=0平面)
    s = (0 - self.tvec[2, 0]) / (self.R[2] @ normalized_coords)
    
    # 步驟4: 計算世界座標
    camera_point = s * normalized_coords
    world_point = np.linalg.inv(self.R) @ (camera_point - self.tvec.ravel())
    
    return world_point[:2]  # 返回X,Y座標
```

### 精度處理

#### 世界座標精度設計
- **存儲格式**: 32位有符號整數
- **精度係數**: ×100 (保留2位小數)
- **數值範圍**: ±21474.83mm
- **寄存器分配**: 每個座標使用2個16位寄存器 (高位+低位)

#### 數值轉換實現
```python
# 世界座標轉寄存器值
world_x_int = int(world_x * 100)        # 123.45mm → 12345
world_x_high = (world_x_int >> 16) & 0xFFFF
world_x_low = world_x_int & 0xFFFF

# 寄存器值恢復世界座標
world_x_int = (world_x_high << 16) | world_x_low
world_x = world_x_int / 100.0           # 12345 → 123.45mm
```

## 握手協議實現 (v4.1更新)

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
| 16 | 拍照+檢測 | 圖像捕獲與圓形檢測 (含世界座標+保護範圍過濾) |
| 32 | 重新初始化 | 相機重新初始化 |

### 保護範圍檢測流程 (v4.1新增)
```python
def capture_and_detect(self):
    # 1. 圓形檢測 (像素座標)
    circles, annotated_image = self.detector.detect_circles(image)
    
    # 2. 世界座標轉換 (如果標定數據有效)
    if self.calibration_manager.transformer.is_valid():
        for circle in circles:
            pixel_coords = [circle['center']]
            world_coords = self.transformer.pixel_to_world(pixel_coords)
            circle['world_coords'] = (world_coords[0], world_coords[1])
    
    # 3. v4.1新增: 保護範圍過濾
    original_count = len(circles)
    circles, valid_count, filtered_count = self._filter_circles_by_protection_zone(circles, has_world_coords)
    
    # 4. 更新Modbus寄存器 (含過濾統計)
    result.has_world_coords = can_transform
    result.original_count = original_count
    result.valid_count = valid_count
    result.filtered_count = filtered_count
    self.modbus_client.update_detection_results(result)
```

## 圓形檢測演算法 (維持原有)

### CircleDetector類實現
- 高斯濾波預處理
- Canny邊緣檢測
- 輪廓分析與圓度計算
- 最多輸出5個檢測結果

### 檢測參數配置
```python
class DetectionParams:
    min_area: float = 50000.0
    min_roundness: float = 0.8
    gaussian_kernel_size: int = 9
    gaussian_sigma: float = 2.0
    canny_low: int = 20
    canny_high: int = 60
```

### 結果數據結構 (v4.1擴展)
```python
class VisionResult:
    circle_count: int                    # 過濾後的圓形數量
    circles: List[Dict[str, Any]]        # 包含world_coords和in_protection_zone
    processing_time: float
    capture_time: float
    total_time: float
    timestamp: str
    success: bool
    has_world_coords: bool = False       # v4.0: 世界座標有效性
    error_message: Optional[str] = None
    # v4.1新增: 保護範圍過濾統計
    original_count: int = 0              # 原始檢測到的物件數量
    valid_count: int = 0                 # 有效範圍內物件數量
    filtered_count: int = 0              # 被過濾掉的物件數量
```

## 相機管理實現 (維持原有)

### OptimizedCamera類
- 海康威視SDK封裝
- 單相機完整生命週期管理
- 幀緩存機制
- 性能監控統計

### CameraConfig配置
```python
class CameraConfig:
    name: str = "cam_1"
    ip: str = "192.168.1.8"
    exposure_time: float = 20000.0
    gain: float = 200.0
    frame_rate: float = 30.0
    pixel_format: PixelFormat = BAYER_GR8
    width: int = 2592
    height: int = 1944
```

## 線程架構 (v4.1更新)

### 主要線程
- **主線程**: Flask Web應用
- **握手同步線程**: _handshake_sync_loop (50ms輪詢)
- **指令執行線程**: _execute_command_async (異步執行)

### 線程安全機制
- SystemStateMachine使用self.lock保護狀態
- 指令執行狀態command_processing防重入
- 相機操作使用threading.RLock

## Flask Web介面 (v4.1更新)

### 核心API路由
- POST /api/modbus/set_server - 設置Modbus服務器地址
- POST /api/modbus/connect - 連接Modbus服務器
- GET /api/modbus/registers - 讀取所有寄存器即時數值 (含世界座標+保護範圍)
- POST /api/modbus/manual_command - 手動發送控制指令
- POST /api/initialize - 初始化相機連接
- POST /api/capture_and_detect - 執行拍照檢測 (含世界座標+保護範圍)
- **v4.0**: GET /api/calibration/scan - 掃描標定檔案
- **v4.0**: POST /api/calibration/load - 載入標定數據
- **v4.0**: GET /api/calibration/status - 獲取標定狀態
- **v4.1新增**: POST /api/protection_zone/set - 設置保護範圍
- **v4.1新增**: GET /api/protection_zone/get - 獲取保護範圍配置

### Web介面功能
- 運行在localhost:5051
- SocketIO即時通訊
- 狀態監控顯示
- 參數調整介面
- 手動控制功能
- **v4.0**: 標定檔案管理介面
- **v4.0**: 世界座標狀態顯示
- **v4.0**: 世界座標結果顯示
- **v4.1新增**: 保護範圍設定介面
- **v4.1新增**: 過濾統計顯示

### UI界面更新 (v4.1)
```html
<!-- 標定管理面板 (v4.0維持) -->
<div class="calibration-panel">
    <h2>🌍 相機標定與世界座標轉換</h2>
    <div class="calibration-status">
        <!-- 內參狀態、外參狀態、轉換器狀態、工作目錄 -->
    </div>
    <button onclick="scanCalibrationFiles()">🔍 掃描檔案</button>
    <button onclick="loadCalibrationData()">✅ 確認導入</button>
</div>

<!-- v4.1新增: 保護範圍設定面板 -->
<div class="protection-zone-panel">
    <h2>🛡️ 保護範圍設定</h2>
    <div class="protection-zone-config">
        <label>
            <input type="checkbox" id="protectionEnabled"> 啟用保護範圍
        </label>
        <div class="range-inputs">
            <div>X範圍: <input type="number" id="xMin" value="-122.0"> ~ <input type="number" id="xMax" value="-4.0"> mm</div>
            <div>Y範圍: <input type="number" id="yMin" value="243.0"> ~ <input type="number" id="yMax" value="341.0"> mm</div>
        </div>
        <button onclick="setProtectionZone()">🔧 設置保護範圍</button>
    </div>
</div>

<!-- 檢測結果顯示 (v4.1擴展) -->
<div class="circle-coords">
    <div class="coord-group pixel-coord">
        <div class="coord-label">像素座標</div>
        <div class="coord-value">X: 320 px, Y: 240 px</div>
    </div>
    <div class="coord-group world-coord">
        <div class="coord-label">世界座標</div>
        <div class="coord-value">X: 123.45 mm, Y: 678.90 mm</div>
        <div class="protection-status">🛡️ 範圍內</div>
    </div>
    <!-- v4.1新增: 過濾統計 -->
    <div class="filter-stats">
        <span>原始: 5個</span> | 
        <span>有效: 3個</span> | 
        <span>過濾: 2個</span>
    </div>
</div>
```

## 依賴模組版本 (維持原有)

### 核心依賴
- PyModbus 3.9.2 (Modbus TCP Client)
- OpenCV (圖像處理 + 座標轉換)
- NumPy (數值計算)
- Flask + SocketIO (Web介面)

### 海康威視SDK
- MvCameraControl_class
- CameraParams_const
- CameraParams_header
- MvErrorDefine_const
- PixelType_header

## 錯誤處理機制 (v4.1擴展)

### 連接管理
- Modbus TCP自動重連 (reconnect_delay: 5秒)
- 相機連接異常處理
- 超時控制 (read_timeout: 3秒)

### 標定檔案錯誤處理 (v4.0維持)
- 檔案格式驗證
- 檔案缺失提示
- 轉換失敗處理
- 詳細錯誤信息顯示

### 保護範圍錯誤處理 (v4.1新增)
- 範圍參數驗證
- 世界座標依賴檢查
- 過濾失敗處理
- 統計數據一致性檢查

### 狀態異常處理
- 通訊錯誤計數統計
- Alarm狀態自動設置
- 錯誤資訊記錄與回報

### 異常恢復
- 重新初始化指令 (32)
- 狀態機重置機制
- 資源釋放與重建

## 配置管理 (v4.1更新)

### 模組配置
- 無獨立配置檔案
- 硬編碼預設參數
- 運行時動態調整

### 預設配置值
```python
# Modbus連接
server_ip = "192.168.1.100"
server_port = 502
base_address = 200

# 相機配置  
camera_ip = "192.168.1.8"
exposure_time = 20000.0
gain = 200.0

# 檢測參數
min_area = 50000.0
min_roundness = 0.8

# 標定檔案 (v4.0)
working_dir = os.path.dirname(os.path.abspath(__file__))

# 保護範圍 (v4.1新增)
protection_zone = ProtectionZoneConfig(
    enabled=False,
    x_min=-122.0, x_max=-4.0,
    y_min=243.0, y_max=341.0
)
```

## 部署配置 (v4.1更新)

### 運行順序 (完整版本)
1. 啟動主Modbus TCP Server (端口502)
2. 準備內外參NPY檔案 (放入程式同層目錄)
3. 啟動CCD1VisionCode_Enhanced.py
4. 訪問Web介面 (localhost:5051)
5. 掃描並載入標定檔案
6. 設定保護範圍參數 (v4.1新增)
7. 設置Modbus服務器地址並連接
8. 初始化相機連接
9. 系統進入握手模式等待PLC指令

### 運行順序 (使用高層API)
1. 啟動主Modbus TCP Server (端口502)
2. 準備內外參NPY檔案 (放入程式同層目錄)
3. 啟動CCD1VisionCode_Enhanced.py (主模組)
4. 在其他模組中import CCD1HighLevel.py
5. 創建CCD1HighLevelAPI實例並直接使用

### 檔案結構 (v4.1 + 高層API)
```
Vision/
├── CCD1VisionCode_Enhanced.py    # 主程序 (v4.1)
├── CCD1HighLevel.py              # 高層API模組 ⭐新增⭐
├── camera_manager.py             # 相機管理API
├── templates/
│   └── ccd_vision_enhanced_world_coord.html  # Web介面 (v4.1)
├── camera_matrix_20241210_143022.npy         # 內參矩陣 (範例)
├── dist_coeffs_20241210_143022.npy           # 畸變係數 (範例)
└── extrinsic_20241210_143530.npy             # 外參數據 (範例)
```

## 測試驗證 (v4.1擴展 + 高層API)

### 連接測試
1. Modbus TCP連接狀態檢查
2. 相機設備連接驗證
3. 寄存器讀寫功能測試 (含世界座標+保護範圍寄存器)
4. **新增**: CCD1HighLevel.py連接測試

### 標定功能測試 (v4.0維持)
1. NPY檔案掃描功能
2. 檔案格式驗證
3. 標定數據載入測試
4. 座標轉換精度驗證

### 保護範圍功能測試 (v4.1新增)
1. 保護範圍設定功能
2. 範圍參數驗證
3. 過濾邏輯測試
4. 統計計數器準確性
5. 邊界條件測試

### 高層API功能測試 ⭐新增⭐
1. `get_next_circle_world_coord()`功能驗證
2. FIFO佇列管理測試
3. 自動觸發檢測機制
4. 握手協議處理驗證
5. 異常處理和超時控制
6. 多模組整合測試

### 功能測試
1. 握手協議狀態機驗證
2. 控制指令執行確認
3. 圓形檢測演算法準確性
4. 世界座標轉換準確性 (v4.0)
5. 保護範圍過濾準確性 (v4.1新增)
6. 錯誤處理機制測試

### 性能監控
1. 拍照耗時統計
2. 檢測處理時間
3. 座標轉換耗時 (v4.0)
4. 保護範圍過濾耗時 (v4.1新增)
5. 握手響應延遲
6. 系統穩定性驗證
7. **新增**: 高層API響應時間
8. **新增**: FIFO佇列性能

## 已知限制 (v4.1更新 + 高層API)

### 硬體依賴
- 需要海康威視SDK完整安裝
- 相機必須支援TCP/IP連接
- 網路延遲影響響應時間

### 世界座標限制 (v4.0維持)
- 僅支援Z=0平面投影
- 精度受標定品質影響
- 需要高品質內外參數據
- 座標範圍限制: ±21474.83mm

### 保護範圍限制 (v4.1新增)
- 依賴世界座標系統，需要有效標定數據
- 僅支援矩形範圍過濾
- 過濾精度受座標轉換精度影響
- 不支援複雜幾何形狀的保護範圍

### 高層API限制 ⭐新增⭐
- 依賴CCD1VisionCode_Enhanced.py主模組運行
- FIFO佇列基於記憶體，系統重啟會清空
- 不支援多實例同時連接同一主服務器
- 超時參數固定，不支援動態調整

### 軟體限制
- 最多檢測5個圓形
- 固定圓形檢測演算法
- 無持久化配置機制
- NPY檔案格式限制

### 系統限制
- 單相機支援
- 同步執行模式
- 無負載平衡機制

## 開發問題修正記錄

### v4.1保護範圍功能開發記錄

#### 1. 寄存器地址重分配 (新增)
**設計**: 保護範圍寄存器地址分配
**實現**: 重用統計寄存器區域，替換錯誤/連接計數器
```python
# 寄存器重新分配
284: 錯誤計數器 → 有效物件數量
285: 連接計數器 → 過濾物件數量
294-299: 保護範圍參數 (新增)
277-279: Y範圍參數 (使用空閒地址)
```

#### 2. 過濾邏輯實現 (新增)
**需求**: 基於世界座標的範圍過濾
**實現**: 條件式過濾器
```python
def _filter_circles_by_protection_zone(self, circles, has_world_coords):
    # 1. 檢查保護範圍啟用狀態
    # 2. 檢查世界座標有效性
    # 3. 逐一檢查圓形世界座標
    # 4. 過濾範圍外的物件
    # 5. 統計有效/過濾數量
```

#### 3. 統計計數器更新 (修改)
**變更**: 錯誤/連接計數器改為有效/過濾計數器
**實現**: 寄存器功能轉換
- 284寄存器: error_count → valid_count
- 285寄存器: connection_count → filtered_count

#### 4. UI界面擴展 (新增)
**功能**: 保護範圍設定與過濾統計顯示
**實現**: Web界面新增
- 保護範圍設定面板
- 範圍啟用開關
- X/Y範圍輸入框
- 過濾統計顯示

#### 5. 向下兼容保證 (重要)
**需求**: 保護範圍關閉時系統正常工作
**實現**: 條件式過濾邏輯
```python
if not self.protection_zone.enabled or not has_world_coords:
    return circles, len(circles), 0  # 不過濾，維持原有行為
```

### 高層API開發記錄 ⭐新增⭐

#### 1. API架構設計 (新增)
**設計**: 簡化使用介面，封裝複雜握手協議
**實現**: CCD1HighLevelAPI類
- ModbusTCP Client封裝
- FIFO佇列管理
- 自動重連機制
- 超時控制

#### 2. 數據結構設計 (新增)
**需求**: 結構化圓心座標數據
**實現**: CircleWorldCoord dataclass
```python
@dataclass
class CircleWorldCoord:
    id: int, world_x: float, world_y: float
    pixel_x: int, pixel_y: int, radius: int
    timestamp: str
```

#### 3. FIFO佇列實現 (新增)
**功能**: 圓心座標先進先出管理
**實現**: deque + threading.Lock
- 線程安全操作
- 自動觸發檢測機制
- 佇列狀態監控

#### 4. 握手協議封裝 (新增)
**需求**: 自動處理握手細節
**實現**: 私有方法封裝
- `_wait_for_ready()`: 等待Ready狀態
- `_wait_for_command_complete()`: 等待執行完成
- `_read_world_coordinates()`: 讀取結果

#### 5. 錯誤處理機制 (新增)
**功能**: 完善的異常處理
**實現**: 多層級錯誤處理
- 連接異常處理
- 超時控制
- Alarm狀態檢查
- 日誌記錄

### v4.0世界座標功能開發記錄 (維持)

#### 1. 寄存器地址分配 (v4.0)
**設計**: 世界座標寄存器區域規劃
**實現**: 採用方案A，256-276地址區間
```python
# 寄存器分配
像素座標: 241-255 (現有)
世界座標: 256-276 (v4.0新增，20個寄存器)
統計資訊: 280-299 (v4.1部分更新)
```

#### 2. 精度處理實現 (v4.0)
**需求**: 世界座標保留2位小數精度
**實現**: ×100整數存儲方案
```python
world_x_int = int(world_x * 100)  # 123.45 → 12345
world_x = world_x_int / 100.0     # 12345 → 123.45
```

#### 3. 檔案管理功能 (v4.0)
**功能**: 自動掃描內外參NPY檔案
**實現**: CalibrationManager類
- 規範內參檔案命名檢查
- 寬鬆外參檔案命名支援
- 檔案格式自動驗證

#### 4. UI界面整合 (v4.0)
**功能**: 標定管理與世界座標顯示
**實現**: Web界面擴展
- 標定狀態面板
- 世界座標指示器
- 檔案掃描與導入按鈕
- 檢測結果雙座標顯示

#### 5. 向下兼容處理 (v4.0)
**需求**: 無標定數據時系統正常工作
**實現**: 條件式世界座標轉換
```python
has_world_coords = (標定數據有效 and 轉換成功)
result.has_world_coords = has_world_coords
```

### 原有問題修正記錄 (維持)

#### 1. 狀態機初始值問題 (已修正)
**錯誤**: 狀態寄存器初始值不固定
**修正**: 強制設置初始值為1 (Ready=1, 其他位=0)
```python
# 修正實現
initial_status = 0b0001
self.state_machine.status_register = initial_status
```

#### 2. 握手協議時序錯誤 (已修正)
**錯誤**: Running狀態未正確清除
**修正**: 指令執行完成後確保Running=0，等待PLC清零指令恢復Ready

#### 3. 相機連接異常處理 (已修正)
**錯誤**: 相機斷線未設置Alarm狀態
**修正**: 相機異常時自動設置Alarm=1, Initialized=0

#### 4. 線程同步問題 (已修正)
**錯誤**: 多線程訪問狀態機競爭條件
**修正**: 使用threading.Lock保護狀態機操作
```python
def set_bit(self, bit_pos: StatusBits, value: bool):
    with self.lock:
        # 原子化操作
```

#### 5. 指令重複執行問題 (已修正)
**錯誤**: 同一指令被重複執行
**修正**: 實現command_processing標誌防重入
- last_control_command追蹤已處理指令
- command_processing防止重複執行

## 狀態機交握實現細節 (v4.1更新)

### EnhancedModbusTcpClientService核心方法
```python
def _handshake_sync_loop(self):
    # 50ms高頻輪詢
    # 1. 更新狀態寄存器到PLC
    # 2. 讀取控制指令並處理握手邏輯
    # 3. 定期更新統計資訊
    # 4. 更新世界座標有效性標誌 (v4.0)
    # 5. 更新保護範圍狀態到PLC (v4.1新增)
```

### 指令執行流程
```python
def _handle_action_command(self, command: ControlCommand):
    # 1. 檢查Ready狀態
    # 2. 設置Running狀態，清除Ready
    # 3. 異步執行指令 (含世界座標轉換+保護範圍過濾)
    # 4. 完成後清除Running狀態
```

### 異常處理機制
```python
def _update_initialization_status(self):
    # 檢查Modbus和相機連接狀態
    # 檢查標定數據有效性 (v4.0)
    # 檢查保護範圍配置狀態 (v4.1新增)
    # 設置Initialized和Alarm位
    # 確保狀態一致性
```

## API介面文檔 (v4.1擴展 + 高層API)

### CCD1VisionController關鍵方法
```python
def set_modbus_server(self, ip: str, port: int) -> Dict[str, Any]
def connect_modbus(self) -> Dict[str, Any]  
def initialize_camera(self, ip_address: str) -> Dict[str, Any]
def capture_and_detect(self) -> VisionResult  # 含世界座標+保護範圍過濾
def update_detection_params(self, **kwargs)
def get_status(self) -> Dict[str, Any]       # 含標定狀態+保護範圍狀態

# v4.0標定相關方法
def scan_calibration_files(self) -> Dict[str, Any]
def load_calibration_data(self, intrinsic_file=None, extrinsic_file=None) -> Dict[str, Any]
def get_calibration_status(self) -> Dict[str, Any]

# v4.1新增保護範圍相關方法
def set_protection_zone(self, enabled: bool, x_min: float, x_max: float, y_min: float, y_max: float) -> Dict[str, Any]
def _is_in_protection_zone(self, world_x: float, world_y: float) -> bool
def _filter_circles_by_protection_zone(self, circles: List[Dict], has_world_coords: bool) -> Tuple[List[Dict], int, int]
```

### CCD1HighLevelAPI關鍵方法 ⭐新增⭐
```python
def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502)
def connect(self) -> bool
def disconnect(self)
def get_next_circle_world_coord(self) -> Optional[CircleWorldCoord]  # 主要API
def capture_and_detect(self) -> bool
def get_queue_status(self) -> Dict[str, Any]
def clear_queue(self)
def is_ready(self) -> bool
def get_system_status(self) -> Dict[str, Any]

# 私有方法 (內部使用)
def _wait_for_ready(self, timeout: float = 10.0) -> bool
def _wait_for_command_complete(self, timeout: float = 10.0) -> bool
def _read_world_coordinates(self) -> List[CircleWorldCoord]
def _read_register(self, register_name: str) -> Optional[int]
def _write_register(self, register_name: str, value: int) -> bool
def _read_multiple_registers(self, start_address: int, count: int) -> Optional[List[int]]
```

### SystemStateMachine狀態控制 (維持)
```python
def is_ready(self) -> bool
def is_running(self) -> bool
def is_alarm(self) -> bool
def is_initialized(self) -> bool
def set_ready(self, ready: bool)
def set_running(self, running: bool)
def reset_to_idle(self)
```

### CalibrationManager標定管理 (v4.0維持)
```python
def scan_calibration_files(self) -> Dict[str, Any]
def load_calibration_data(self, intrinsic_file=None, extrinsic_file=None) -> Dict[str, Any]
def get_status(self) -> Dict[str, Any]
```

### CameraCoordinateTransformer座標轉換 (v4.0維持)
```python
def pixel_to_world(self, pixel_coords) -> Optional[np.ndarray]
def is_valid(self) -> bool
```

### ProtectionZoneConfig保護範圍配置 (v4.1新增)
```python
@dataclass
class ProtectionZoneConfig:
    enabled: bool = False
    x_min: float = -122.0
    x_max: float = -4.0
    y_min: float = 243.0
    y_max: float = 341.0
```

## 運行狀態輸出 (v4.1更新 + 高層API)

### 系統啟動輸出
```
CCD1 視覺控制系統啟動中 (運動控制握手版本 v4.1 + 世界座標轉換 + 保護範圍)
系統架構: Modbus TCP Client - 運動控制握手模式 v4.1
連接模式: 主動連接外部PLC/HMI設備
握手協議: 指令/狀態模式，50ms高頻輪詢
v4.0功能: NPY內外參管理 + 像素座標到世界座標轉換
v4.1新增: 保護範圍過濾功能 + 世界座標範圍檢查
⭐ 高層API: CCD1HighLevel.py 簡化使用介面
Web介面啟動中... http://localhost:5051
```

### 握手協議運行日誌
```
收到新控制指令: 16 (上次: 0)
開始處理控制指令: 16
執行拍照+檢測指令 (含世界座標轉換和保護範圍過濾)
檢測成功，找到 5 個圓形 (包含世界座標)
保護範圍過濾: 原始5個 → 有效3個 (過濾2個)
控制指令 16 執行完成
恢復Ready狀態
```

### 世界座標轉換日誌 (v4.0維持)
```
掃描結果: 找到1組內參檔案, 1個外參檔案
標定數據載入成功
轉換器已啟用，載入時間: 2024-12-10 14:30:22
像素座標 [320, 240] 對應世界座標 [123.45, 678.90] mm
世界座標寄存器更新: 256=1, 257-260=[1, 23400, 6, 78900]
```

### 保護範圍過濾日誌 (v4.1新增)
```
保護範圍設定: 啟用=True, X(-122.0~-4.0), Y(243.0~341.0)
保護範圍寄存器更新: 294=1, 295-299=[...], 277-279=[...]
圓形1 世界座標(-50.5, 300.2) → 範圍內 ✓
圓形2 世界座標(10.0, 250.0) → 範圍外 ✗
圓形3 世界座標(-80.3, 280.1) → 範圍內 ✓
過濾結果: 原始3個 → 有效2個 (過濾1個)
統計寄存器更新: 284=2(有效), 285=1(過濾)
```

### 高層API運行日誌 ⭐新增⭐
```
CCD1HighLevel API初始化完成
正在連接Modbus TCP服務器: 127.0.0.1:502
Modbus TCP連接成功: 127.0.0.1:502
發送拍照+檢測指令...
等待指令執行完成...
檢測完成，新增 3 個圓心座標到佇列
返回圓心座標: ID=1, 世界座標=(-50.5, 300.2)mm
佇列狀態: 長度=2, 最後檢測=3個
```

## 記憶體管理 (v4.1維持 + 高層API)

### 線程生命週期
- daemon模式線程自動回收
- 握手同步線程正確退出機制
- 相機資源釋放管理

### 幀數據管理
- 固定大小幀緩存隊列
- 舊幀自動丟棄機制
- numpy數組記憶體回收

### 標定數據管理 (v4.0維持)
- NPY檔案按需載入
- 標定數據快取管理
- 座標轉換器資源釋放

### 保護範圍數據管理 (v4.1新增)
- 保護範圍配置輕量化存儲
- 過濾邏輯計算優化
- 統計計數器記憶體控制

### 高層API記憶體管理 ⭐新增⭐
- FIFO佇列大小控制 (deque)
- 座標對象自動回收
- ModbusTCP Client連接池管理
- 線程安全鎖資源釋放

### 資源釋放
```python
def disconnect(self):
    # 停止握手同步線程
    # 斷開相機連接
    # 關閉Modbus連接
    # 釋放標定數據 (v4.0)
    # 清空保護範圍配置 (v4.1新增)
    # 清空FIFO佇列 (高層API新增)
    # 釋放所有資源
```

## 版本歷史

### v4.1 (2024-12-XX) - 保護範圍過濾功能 + 高層API
- **新增**: 保護範圍過濾功能
- **新增**: 世界座標範圍檢查邏輯
- **新增**: 保護範圍寄存器映射 (294-299, 277-279)
- **新增**: 過濾統計計數器 (284: 有效數量, 285: 過濾數量)
- **新增**: 保護範圍設定API (/api/protection_zone/set, /api/protection_zone/get)
- **新增**: Web界面保護範圍管理功能
- **新增**: 過濾統計結果顯示 (original_count, valid_count, filtered_count)
- **⭐新增**: CCD1HighLevel.py高層API模組
- **⭐新增**: CircleWorldCoord數據結構
- **⭐新增**: FIFO佇列管理機制
- **⭐新增**: 簡化使用介面 get_next_circle_world_coord()
- **⭐新增**: 自動握手協議處理
- **更新**: 軟體版本號升級到4.1
- **重構**: 錯誤/連接計數器轉為有效/過濾計數器
- **擴展**: VisionResult數據結構新增過濾統計欄位

### v4.0 (2024-12-XX) - 世界座標轉換功能
- **新增**: 內外參NPY檔案管理
- **新增**: 像素座標到世界座標轉換 (Z=0平面)
- **新增**: 世界座標寄存器映射 (256-276)
- **新增**: 標定檔案掃描與載入API
- **新增**: Web界面標定管理功能
- **新增**: 世界座標結果顯示 (保留2位小數)
- **更新**: 軟體版本號升級到4.0
- **擴展**: Modbus寄存器總數增加到21個

### v3.0 (原版本) - 運動控制握手
- 握手式狀態機控制
- 50ms高頻輪詢
- 圓形檢測演算法
- Web界面控制
- Modbus TCP Client架構

## 開發建議與最佳實踐 ⭐新增⭐

### 使用CCD1高層API的建議

#### 1. 選擇合適的使用方式
```python
# 簡單使用場景 - 推薦使用高層API
from CCD1HighLevel import CCD1HighLevelAPI

ccd1 = CCD1HighLevelAPI()
coord = ccd1.get_next_circle_world_coord()  # 一行代碼獲取座標

# 複雜應用場景 - 直接使用主模組
# 需要自定義檢測參數、Web界面等功能時
```

#### 2. 錯誤處理建議
```python
# 推薦的錯誤處理模式
try:
    coord = ccd1.get_next_circle_world_coord()
    if coord:
        # 處理有效座標
        process_coordinate(coord)
    else:
        # 處理無座標情況
        handle_no_coordinate()
except Exception as e:
    # 處理異常
    logger.error(f"CCD1檢測異常: {e}")
```

#### 3. 資源管理建議
```python
# 推薦的資源管理模式
class MyApplication:
    def __init__(self):
        self.ccd1 = CCD1HighLevelAPI()
    
    def __del__(self):
        if hasattr(self, 'ccd1'):
            self.ccd1.disconnect()
    
    def cleanup(self):
        """主動清理資源"""
        self.ccd1.disconnect()
```

#### 4. 性能優化建議
```python
# 批量處理建議
def process_batch_objects(count: int):
    results = []
    for i in range(count):
        coord = ccd1.get_next_circle_world_coord()
        if coord:
            results.append(coord)
        else:
            break  # 無更多對象時退出
    return results

# 佇列狀態監控
status = ccd1.get_queue_status()
if status['queue_length'] < 2:
    # 佇列不足時主動觸發檢測
    ccd1.capture_and_detect()
```

### 系統整合建議

#### 1. 模組依賴關係
```
應用層模組 (機械臂、輸送帶等)
    ↓ import
CCD1HighLevel.py (高層API)
    ↓ ModbusTCP
CCD1VisionCode_Enhanced.py (主模組)
    ↓ 直連
相機硬體
```

#### 2. 部署檢查清單
- [ ] 主Modbus TCP Server已啟動 (端口502)
- [ ] 相機網路連接正常 (192.168.1.8)
- [ ] 內外參NPY檔案已準備 (程式同層目錄)
- [ ] CCD1VisionCode_Enhanced.py主模組已運行
- [ ] Web界面可正常訪問 (localhost:5051)
- [ ] 標定檔案已載入，世界座標轉換有效
- [ ] 保護範圍參數已設定 (若需要)
- [ ] CCD1HighLevel.py可正常import

#### 3. 故障排除指南
```python
# 連接問題排除
def diagnose_connection():
    ccd1 = CCD1HighLevelAPI()
    
    # 檢查基本連接
    if not ccd1.connected:
        print("❌ Modbus連接失敗")
        return False
    
    # 檢查系統狀態
    status = ccd1.get_system_status()
    if status['alarm']:
        print("❌ 系統處於Alarm狀態")
        return False
    
    if not status['ready']:
        print("⚠️ 系統未Ready")
        return False
    
    # 檢查世界座標
    if not status['world_coord_valid']:
        print("⚠️ 世界座標無效，可能缺少標定數據")
    
    print("✅ 系統狀態正常")
    return True
```

#### 4. 開發測試建議
```python
# 開發階段測試代碼
def development_test():
    ccd1 = CCD1HighLevelAPI()
    
    # 測試基本功能
    print("=== CCD1高層API測試 ===")
    
    # 測試連接
    print(f"連接狀態: {ccd1.connected}")
    
    # 測試系統狀態
    status = ccd1.get_system_status()
    print(f"系統狀態: {status}")
    
    # 測試佇列狀態
    queue_status = ccd1.get_queue_status()
    print(f"佇列狀態: {queue_status}")
    
    # 測試檢測功能
    success = ccd1.capture_and_detect()
    print(f"檢測結果: {'成功' if success else '失敗'}")
    
    # 測試座標獲取
    for i in range(3):
        coord = ccd1.get_next_circle_world_coord()
        if coord:
            print(f"座標{i+1}: ({coord.world_x:.2f}, {coord.world_y:.2f})mm")
        else:
            print(f"座標{i+1}: 無可用座標")
    
    # 清理
    ccd1.disconnect()
```

### 技術支援與維護

#### 版本兼容性
- CCD1HighLevel.py需要PyModbus 3.9.2或更高版本
- 相容CCD1VisionCode_Enhanced.py v4.0及以上版本
- 需要主Modbus TCP Server運行在502端口
- 支援世界座標轉換功能 (v4.0+)
- 支援保護範圍過濾功能 (v4.1+)

#### 升級指南
```python
# 從直接寄存器操作升級到高層API
# 舊方式 (不推薦)
# modbus_client.write_register(200, 16)  # 手動發送檢測指令
# time.sleep(5)  # 等待完成
# result = modbus_client.read_registers(241, 15)  # 手動讀取結果

# 新方式 (推薦)
ccd1 = CCD1HighLevelAPI()
coord = ccd1.get_next_circle_world_coord()  # 一行代碼完成所有操作
```

#### 常見問題FAQ

**Q: 高層API和主模組有什麼區別？**
A: 主模組(CCD1VisionCode_Enhanced.py)提供完整功能和Web界面，高層API(CCD1HighLevel.py)提供簡化的程式介面，兩者配合使用。

**Q: 可以同時運行多個高層API實例嗎？**
A: 不建議，因為它們會競爭同一個主服務器資源。如需多實例，請使用不同的服務器地址。

**Q: 高層API支援自定義檢測參數嗎？**
A: 高層API使用主模組的檢測參數，如需自定義請通過Web界面(localhost:5051)調整。

**Q: FIFO佇列會持久化嗎？**
A: 不會，佇列基於記憶體，系統重啟會清空。如需持久化請自行實現。

**Q: 高層API的超時時間可以調整嗎？**
A: 可以在創建實例後修改operation_timeout屬性：
```python
ccd1 = CCD1HighLevelAPI()
ccd1.operation_timeout = 15.0  # 設定為15秒
```

## 結論

CCD視覺檢測模組v4.1版本在v4.0世界座標轉換功能基礎上，新增了保護範圍過濾功能和CCD1HighLevel.py高層API，提供更完整和易用的視覺檢測解決方案。

### 主要特色
1. **完整的握手式狀態機控制** - 可靠的PLC通訊協議
2. **世界座標轉換功能** - 支援NPY格式標定檔案，提供mm級精度
3. **保護範圍過濾功能** - 基於世界座標的智能範圍過濾
4. **⭐高層API介面** - 一行代碼獲取物件座標，大幅簡化使用
5. **FIFO佇列管理** - 自動管理檢測結果，支援批量處理
6. **Web管理界面** - 完整的標定管理和參數調整功能
7. **向下兼容性** - 無標定數據時仍可正常提供像素座標

### 適用場景
- **自動化產線** - 物件位置檢測與抓取
- **品質檢測** - 圓形零件尺寸與位置檢測
- **機械手臂導引** - 提供精確的物件世界座標
- **輸送帶分揀** - 基於位置範圍的物件過濾
- **系統整合** - 透過高層API快速整合到其他模組

### 技術優勢
- **高精度** - 世界座標精度達0.01mm
- **高可靠** - 完整的錯誤處理和異常恢復機制
- **高效率** - 50ms輪詢間隔，快速響應
- **易整合** - 高層API大幅降低整合複雜度
- **易維護** - 清晰的模組架構和完整的技術文檔

CCD視覺檢測模組已成為工業自動化中可靠且高效的視覺解決方案，透過持續的功能增強和API優化，為各類自動化應用提供強大的技術支撐。