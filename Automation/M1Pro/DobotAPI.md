# Dobot TCP/IP遠程控制API技術文檔

## 概述

Dobot四軸工業機械臂支援TCP/IP協議控制，控制器版本需V1.5.5.0及以上。

### 端口說明
- **29999端口**: Dashboard控制指令（設置、狀態查詢）
- **30003端口**: 運動指令
- **30004端口**: 實時反饋（8ms週期，1440字節）
- **30005端口**: 狀態反饋（200ms週期）
- **30006端口**: 可配置反饋（預設50ms週期）

### 消息格式
```
指令格式: 指令名稱(參數1,參數2,...,參數N)
回應格式: ErrorID,{返回值},指令名稱(參數);
```

### 指令類型
- **立即指令**: 立即執行並返回結果
- **隊列指令**: 進入後台隊列排隊執行

## dobot_api.py實現架構

### 類別結構
```python
class DobotApi                  # 基礎連接類別
class DobotApiDashboard(DobotApi)   # Dashboard指令實現（29999端口）
class DobotApiMove(DobotApi)        # 運動指令實現（30003端口）
```

### 連接管理
- 支援端口：29999（Dashboard）、30003（運動）、30004（反饋）
- 線程安全：使用threading.Lock保護發送接收
- 錯誤處理：自動拋出連接異常

## Dashboard指令 (29999端口)

### 控制相關指令

#### EnableRobot - 使能機械臂
```python
EnableRobot(*dynParams)
```
**實現參數**:
- dynParams: 動態參數(負載重量, 偏心X, 偏心Y, 偏心Z)

**使用範例**:
```python
dashboard.EnableRobot()           # 使能，不設置負載
dashboard.EnableRobot(0.5)        # 設置負載0.5kg
dashboard.EnableRobot(0.5,0,0,5.5) # 設置負載和偏心
```

#### DisableRobot - 下使能機械臂
```python
DisableRobot()
```

#### ClearError - 清除告警
```python
ClearError()
```

#### ResetRobot - 停止機械臂
```python
ResetRobot()
```

#### 工程控制
```python
RunScript(project_name)     # 執行工程
StopScript()               # 停止工程
PauseScript()              # 暫停工程
ContinueScript()           # 繼續工程
```

#### 運動控制
```python
pause()                    # 暫停運動隊列（小寫實現）
Continue()                 # 繼續運動隊列（實現為continue()）
wait(t)                    # 隊列延時（小寫實現）
```

#### 拖拽模式
```python
StartDrag()                # 開始拖拽模式
StopDrag()                 # 停止拖拽模式
```

#### EmergencyStop - 緊急停止
```python
EmergencyStop()
```

### 設置相關指令

#### SpeedFactor - 全局速度比例
```python
SpeedFactor(speed)         # 設置全局速度比例1~100%
```

#### 座標系設置
```python
User(index)                # 設置全局使用者座標系
Tool(index)                # 設置全局工具座標系
```

#### 負載設置
```python
PayLoad(weight, inertia)   # 舊版本負載設置
SetPayload(weight, *dynParams)  # 新版本負載設置，支援偏心參數
```

#### 速度加速度設置
```python
AccJ(speed)                # 關節運動加速度比例1~100
AccL(speed)                # 直線運動加速度比例1~100
SpeedJ(speed)              # 關節運動速度比例1~100
SpeedL(speed)              # 直線運動速度比例1~100
```

#### 其他設置
```python
Arch(index)                # 設置Jump運動門型參數索引
CP(ratio)                  # 設置平滑過渡比例0~100
LimZ(value)                # 設置門型參數最大抬升高度
SetArmOrientation(LorR)    # 設置手系（M1 Pro）
SetCollisionLevel(level)   # 設置碰撞檢測等級0~5
```

### 計算和獲取相關指令

#### RobotMode - 機械臂狀態
```python
RobotMode()
```
**返回值**:
- 1: 初始化
- 2: 有任意關節抱閘松開
- 3: 本體未上電
- 4: 未使能
- 5: 使能且空閒
- 6: 拖拽模式
- 7: 運行狀態
- 8: 軌跡錄製模式
- 9: 有未清除報警
- 10: 暫停狀態
- 11: 點動中

#### 位置獲取
```python
GetAngle()                 # 獲取關節角度
GetPose()                  # 獲取笛卡爾座標
GetErrorID()               # 獲取錯誤碼
```

#### 正逆解運算
```python
PositiveSolution(j1,j2,j3,j4,user,tool)    # 正解運算
InverseSolution(x,y,z,r,user,tool,*dynParams)  # 逆解運算
```

### IO相關指令

#### 數位輸出控制
```python
DO(index, status)                    # 隊列指令
DOExecute(index, status)             # 立即指令
DOGroup(*dynParams)                  # 批量設置
ToolDO(index, status)                # 末端數位輸出（隊列）
ToolDOExecute(index, status)         # 末端數位輸出（立即）
```

#### 數位輸入讀取
```python
DI(index)                           # 讀取DI狀態
ToolDI(index)                       # 讀取末端DI狀態
```

### 特殊控制指令

#### 抱閘控制
```python
BrakeControl(joint_index, status)   # 抱閘控制（0：鎖定，1：釋放）
```

#### 負載開關
```python
LoadSwitch(status)                  # 負載開關
```

### Modbus相關指令

#### 連接管理
```python
ModbusCreate(ip, port, slave_id, isRTU)  # 建立Modbus連接
ModbusClose(index)                       # 關閉Modbus連接
```

#### 寄存器操作
```python
GetInBits(index, addr, count)            # 讀取離散輸入
GetInRegs(index, addr, count, *dynParams) # 讀取輸入寄存器
GetCoils(index, addr, count)             # 讀取線圈寄存器
SetCoils(index, addr, count, valTab)     # 寫入線圈寄存器
GetHoldRegs(index, addr, count, type)    # 讀取保持寄存器
SetHoldRegs(index, addr, count, valTab, type) # 寫入保持寄存器
```

**數據類型支援**:
- U16: 16位無符號整數
- U32: 32位無符號整數
- F32: 32位浮點數
- F64: 64位雙精度浮點數

## 運動指令 (30003端口)

所有運動指令均為隊列指令。

### 基本運動指令

#### MovJ - 關節運動
```python
MovJ(x, y, z, r, *dynParams)
```
**參數說明**:
- x, y, z, r: 目標笛卡爾座標
- dynParams: 動態參數（User, Tool, SpeedJ, AccJ, CP等）

#### MovL - 直線運動
```python
MovL(x, y, z, r, *dynParams)
```
**參數說明**:
- x, y, z, r: 目標笛卡爾座標
- dynParams: 動態參數（User, Tool, SpeedL, AccL, CP等）

#### JointMovJ - 關節座標運動
```python
JointMovJ(j1, j2, j3, j4, *dynParams)
```
**參數說明**:
- j1~j4: 目標關節角度
- dynParams: 動態參數（SpeedJ, AccJ, CP等）

### 相對運動指令

#### RelMovJ - 相對關節運動
```python
RelMovJ(x, y, z, r, *dynParams)
```

#### RelMovL - 相對直線運動
```python
RelMovL(offsetX, offsetY, offsetZ, offsetR, *dynParams)
```

#### RelMovJUser - 使用者座標系相對關節運動
```python
RelMovJUser(offset_x, offset_y, offset_z, offset_r, user, *dynParams)
```

#### RelMovLUser - 使用者座標系相對直線運動
```python
RelMovLUser(offset_x, offset_y, offset_z, offset_r, user, *dynParams)
```

#### RelJointMovJ - 關節座標系相對運動
```python
RelJointMovJ(offset1, offset2, offset3, offset4, *dynParams)
```

### 帶IO的運動指令

#### MovLIO - 直線運動帶IO
```python
MovLIO(x, y, z, r, *dynParams)
```
**IO參數格式**: (Mode, Distance, Index, Status)
- Mode: 0=距離百分比，1=距離數值
- Distance: 觸發距離或百分比
- Index: DO端子編號
- Status: DO狀態(0/1)

#### MovJIO - 關節運動帶IO
```python
MovJIO(x, y, z, r, *dynParams)
```
**參數說明**: 同MovLIO

### 圓弧運動指令

#### Arc - 圓弧插補
```python
Arc(x1, y1, z1, r1, x2, y2, z2, r2, *dynParams)
```
**參數說明**:
- x1, y1, z1, r1: 中間點座標
- x2, y2, z2, r2: 終點座標

#### Circle - 整圓插補
```python
Circle(x1, y1, z1, r1, x2, y2, z2, r2, count, *dynParams)
```
**參數說明**:
- count: 運行圈數
- 其他參數同Arc

### 點動指令

#### MoveJog - 點動控制
```python
MoveJog(axis_id=None, *dynParams)
MoveJog()                   # 停止點動
```

**軸ID參數**:
- J1+/J1-: 關節1正/負方向
- J2+/J2-: 關節2正/負方向
- J3+/J3-: 關節3正/負方向
- J4+/J4-: 關節4正/負方向
- X+/X-: X軸正/負方向
- Y+/Y-: Y軸正/負方向
- Z+/Z-: Z軸正/負方向
- R+/R-: R軸正/負方向

**dynParams參數**:
- coord_type: 1=使用者座標，2=工具座標
- user_index: 使用者索引0~9
- tool_index: 工具索引0~9

### 擴展軸指令

#### MovJExt - 滑軌運動
```python
MovJExt(offset1, *dynParams)
```
**參數說明**:
- offset1: 位置或角度
- dynParams: SpeedE, AccE, Sync等參數

### 同步指令

#### Sync - 等待隊列完成
```python
Sync()                      # 等待最後指令完成
```

#### SyncAll - 等待所有指令完成
```python
SyncAll()                   # 等待所有指令（含擴展軸）完成
```

## 實時反饋信息 (30004端口)

### 反饋數據結構
使用numpy結構化陣列MyType定義，包含1440字節機械臂狀態信息：

#### 主要數據欄位
- 數位輸入/輸出狀態 (8字節each)
- 機械臂模式和時間戳
- 關節目標/實際位置、速度、電流
- TCP座標、速度、力值
- 溫度、電壓等監控數據
- 座標系和控制狀態

#### 狀態標誌
- BrakeStatus: 抱閘狀態
- EnableStatus: 使能狀態
- DragStatus: 拖拽狀態
- RunningStatus: 運行狀態
- ErrorStatus: 錯誤狀態
- JogStatus: 點動狀態

## dobot_api.py實現特點

### 1. 線程安全設計
```python
self.__globalLock = threading.Lock()
```
使用全局鎖保護發送接收操作。

### 2. 動態參數處理
多數函數支援*dynParams動態參數，靈活擴展功能。

### 3. 日誌記錄功能
支援tkinter.Text組件或console輸出，記錄通信過程。

### 4. 指令名稱差異
- MD規範：Wait(), Pause(), Continue()
- 實現版本：wait(), pause(), Continue()

### 5. 錯誤處理
自動處理socket連接異常，提供清晰錯誤訊息。

## 使用範例

### 基本控制流程
```python
# 建立連接
dashboard = DobotApiDashboard("192.168.1.6", 29999)
move = DobotApiMove("192.168.1.6", 30003)

# 使能機械臂
dashboard.EnableRobot(0.5)

# 檢查狀態
status = dashboard.RobotMode()

# 運動控制
move.MovJ(300, 200, 200, 0)
move.MovL(400, 200, 200, 0)

# 等待完成
move.Sync()

# 下使能
dashboard.DisableRobot()

# 關閉連接
dashboard.close()
move.close()
```

### 帶IO的運動控制
```python
# 運動過程中控制IO
move.MovLIO(300, 200, 200, 0, (0, 50, 1, 1))  # 50%距離時DO1置高
```

### Modbus通訊
```python
# 建立連接
index = dashboard.ModbusCreate("192.168.1.100", 502, 1, 0)

# 讀取保持寄存器
values = dashboard.GetHoldRegs(index, 1000, 5, "U16")

# 寫入保持寄存器
dashboard.SetHoldRegs(index, 1000, 2, [100, 200], "U16")

# 關閉連接
dashboard.ModbusClose(index)
```

## 注意事項

1. 控制器版本需V1.5.5.0及以上
2. TCP連接建立後需先使能機械臂
3. 清除報警後需呼叫Continue()重新啟動
4. 隊列指令和立即指令的執行順序差異
5. 座標系參數的正確設置
6. 實時反饋數據採用小端格式
7. Modbus最多同時連接5個設備
8. 指令名稱大小寫需注意（wait vs Wait等）