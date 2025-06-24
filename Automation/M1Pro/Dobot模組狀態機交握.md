# Dobot機械臂狀態機交握設計 (基地址400-449) - 含速度控制

## 寄存器映射規劃

### 狀態寄存器 (400-419) - 只讀
| 地址 | 功能 | 數值定義 | 二進制位定義 |
|------|------|----------|--------------|
| 400 | 主狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized | 狀態機核心控制位 |
| 401 | 機械臂狀態 | 0=離線, 1=閒置, 2=執行中, 3=錯誤, 4=緊急停止 | 機械臂運行狀態 |
| 402 | 當前流程ID | 0=無, 1=Flow1(取料), 2=Flow2(出料), 3=Flow3(完整) | 正在執行的流程 |
| 403 | 流程執行進度 | 0-100百分比 | 當前流程完成度 |
| 404 | 錯誤代碼 | 0=無錯誤, >0=錯誤類型 | 具體錯誤資訊 |
| 405 | 機械臂模式 | 機械臂實際模式值 | 來自GetRobotMode() |
| 406 | 當前X座標 | 位置座標(mm) | 實時位置資訊 |
| 407 | 當前Y座標 | 位置座標(mm) | 實時位置資訊 |
| 408 | 當前Z座標 | 位置座標(mm) | 實時位置資訊 |
| 409 | 當前R座標 | 旋轉角度 | 實時姿態資訊 |
| 410 | 關節J1角度 | 角度值×100 | 關節1實時角度 |
| 411 | 關節J2角度 | 角度值×100 | 關節2實時角度 |
| 412 | 關節J3角度 | 角度值×100 | 關節3實時角度 |
| 413 | 關節J4角度 | 角度值×100 | 關節4實時角度 |
| 414 | 數位輸入狀態 | DI狀態位 | 輸入IO狀態 |
| 415 | 數位輸出狀態 | DO狀態位 | 輸出IO狀態 |
| 416 | 操作計數器 | 累積成功次數 | 統計資訊 |
| 417 | 錯誤計數器 | 累積錯誤次數 | 統計資訊 |
| 418 | 運行時間 | 分鐘數 | 系統運行時間 |
| 419 | **全局速度設定值** | **1-100百分比** | **當前機械臂全局速度** |
| 420 | Flow1完成狀態0=未完成, 1=完成且角度校正成功Flow1真正完成標誌| | |

### 控制寄存器 (440-449) - 讀寫
| 地址 | 功能 | 數值定義 | 說明 |
|------|------|----------|------|
| 440 | VP視覺取料控制 | 0=清空, 1=啟動Flow1 | VP取料流程觸發 |
| 441 | 出料控制 | 0=清空, 1=啟動Flow2 | 出料流程觸發 |
| 442 | 清除警報控制 | 0=無動作, 1=清除Alarm | 錯誤重置功能 |
| 443 | 緊急停止控制 | 0=正常, 1=緊急停止 | 安全停止功能 |
| 444 | 手動指令 | Flow ID或特殊指令 | Web端手動控制 |
| **445** | **速度控制指令** | **0=無動作, 1=設定速度** | **速度設定觸發** |
| **446** | **速度數值** | **1-100百分比** | **要設定的速度值** |
| **447** | **速度指令ID** | **1-65535唯一識別** | **防重複執行機制** |
| 448 | 保留控制1 | - | 未來擴展使用 |
| 449 | 保留控制2 | - | 未來擴展使用 |

## 狀態機核心邏輯 (地址400二進制位)

### 狀態位定義
```
地址400的二進制位元控制:
bit0 = Ready      (系統準備就緒，可接受新指令)
bit1 = Running    (系統執行中，拒絕新指令)
bit2 = Alarm      (系統警報狀態，需要重置)
bit3 = Initialized (系統已完成初始化)
bit4-15 = 保留   (未來擴展使用)
```

### 狀態組合解讀
| 二進制 | 十進制 | Ready | Running | Alarm | 狀態說明 |
|--------|--------|-------|---------|-------|----------|
| 00001001 | 9 | 1 | 0 | 0 | 系統Ready，可接受指令 |
| 00001010 | 10 | 0 | 1 | 0 | 系統執行中，拒絕新指令 |
| 00001100 | 12 | 0 | 0 | 1 | 系統警報，需要重置 |
| 00001000 | 8 | 0 | 0 | 0 | 系統空閒，等待狀態更新 |

## 交握協議流程設計

### Flow1 (VP視覺取料) 完整流程
```
1. PLC檢查: 讀取400寄存器 = 9 (Ready=1)
2. PLC觸發: 寫入440=1 (VP視覺取料控制)
3. 系統響應: 400寄存器 = 10 (Ready=0, Running=1)
4. 系統執行: 402寄存器 = 1 (Flow1執行中)
5. Flow1完成: 
   - 成功: 400寄存器 = 8 (Ready=0, Running=0, 等待清零)
   - 失敗: 400寄存器 = 12 (Alarm=1)
6. PLC清零: 寫入440=0
7. 系統恢復: 400寄存器 = 9 (Ready=1)
```

### Flow2 (出料流程) 聯動控制
```
前提條件: 400寄存器bit0=1 (Ready=1) 且 440=0 (取料控制已清零)

1. PLC檢查: 讀取400寄存器 = 9 (Ready=1)
2. PLC觸發: 寫入441=1 (出料控制)
3. 系統響應: 400寄存器 = 10 (Ready=0, Running=1)
4. 系統執行: 402寄存器 = 2 (Flow2執行中)
5. Flow2完成:
   - 成功: 400寄存器 = 8 (Ready=0, Running=0)
   - 失敗: 400寄存器 = 12 (Alarm=1)
6. PLC清零: 寫入441=0
7. 系統恢復: 400寄存器 = 9 (Ready=1)
```

### **速度控制流程 (新增)**
```
全局速度設定流程:

1. 準備階段: 檢查400寄存器Ready=1 (非必須，速度可在任何時候設定)
2. 參數設定: 寫入446=速度值 (1-100百分比)
3. ID設定: 寫入447=唯一指令ID (1-65535)
4. 觸發執行: 寫入445=1 (觸發速度設定)
5. 系統處理: 
   - 讀取446、447寄存器
   - 驗證速度範圍 (1-100)
   - 檢查指令ID防重複
   - 調用機械臂API設定速度
   - 更新419寄存器為新速度值
6. 完成清零: 系統自動清零445=0
7. 狀態更新: 419寄存器顯示當前生效速度
```

### 警報處理流程
```
當400寄存器bit2=1 (Alarm=1)時:
1. PLC檢測: 400寄存器 = 12 (Alarm=1)
2. PLC重置: 寫入442=1 (清除警報控制)
3. 系統清除: 400寄存器bit2=0 (Alarm清除)
4. PLC確認: 寫入442=0
5. 系統恢復: 400寄存器 = 9 (Ready=1)
```

## 錯誤觸發條件 (Alarm=1)

### Flow執行錯誤
- 機械臂連接失敗
- 夾爪操作失敗
- 運動控制異常
- 點位不存在

### CCD檢測錯誤
- CCD1視覺檢測無物體
- CCD3角度檢測失敗
- 通訊超時

### **速度控制錯誤 (新增)**
- 速度值超出範圍 (< 1 或 > 100)
- 機械臂API連接失敗
- 機械臂未初始化
- 速度設定API調用失敗

### 系統錯誤
- Modbus通訊異常
- 設備初始化失敗
- 緊急停止觸發

## 實現代碼修改重點

### 1. DobotStateMachine類擴展 - 速度控制
```python
class DobotStateMachine:
    def __init__(self, modbus_client: ModbusTcpClient):
        self.modbus_client = modbus_client
        self.status_register = 0b1001  # 初始: Ready=1, Initialized=1
        
        # 速度控制狀態機 (新增)
        self.current_global_speed = 50  # 當前全局速度
        self.last_speed_cmd_id = 0      # 最後處理的速度指令ID
        
    def process_speed_command(self, robot: DobotM1Pro) -> bool:
        """處理速度控制指令 - 狀態機交握"""
        try:
            # 讀取速度控制指令
            speed_command = self.read_control_register(5)      # 445
            speed_value = self.read_control_register(6)        # 446
            speed_cmd_id = self.read_control_register(7)       # 447
            
            # 檢查是否有新的速度指令
            if (speed_command == 1 and 
                speed_cmd_id != 0 and 
                speed_cmd_id != self.last_speed_cmd_id):
                
                # 驗證速度範圍
                if speed_value < 1 or speed_value > 100:
                    self.safe_write_register(DobotRegisters.SPEED_COMMAND, 0)
                    return False
                
                # 設定機械臂速度
                success = robot.set_global_speed(speed_value)
                
                if success:
                    self.update_global_speed_register(speed_value)
                    
                # 更新最後處理的指令ID
                self.last_speed_cmd_id = speed_cmd_id
                
                # 清除速度控制指令
                self.safe_write_register(DobotRegisters.SPEED_COMMAND, 0)
                
                return success
                
            return True
            
        except Exception as e:
            print(f"處理速度控制指令異常: {e}")
            return False
    
    def update_global_speed_register(self, speed: int):
        """更新全局速度寄存器"""
        try:
            self.current_global_speed = speed
            self.safe_write_register(DobotRegisters.GLOBAL_SPEED, speed)
        except Exception as e:
            print(f"更新全局速度寄存器失敗: {e}")
```

### 2. DobotM1Pro類擴展 - 速度控制API
```python
class DobotM1Pro:
    def __init__(self, ip: str = "192.168.1.6"):
        self.ip = ip
        self.dashboard_api = None
        self.move_api = None
        self.is_connected = False
        self.global_speed = 50  # 預設全局速度
        self.last_set_speed = 50  # 追蹤最後設定的速度
        
    def set_global_speed(self, speed: int) -> bool:
        """設置全局速度 - 增強版"""
        try:
            if speed < 1 or speed > 100:
                return False
                
            if self.dashboard_api:
                # 同時設定所有速度相關參數
                self.dashboard_api.SpeedFactor(speed)     # 全局速度比例
                self.dashboard_api.SpeedJ(speed)          # 關節運動速度
                self.dashboard_api.SpeedL(speed)          # 直線運動速度
                self.dashboard_api.AccJ(speed)            # 關節運動加速度
                self.dashboard_api.AccL(speed)            # 直線運動加速度
                
                self.global_speed = speed
                self.last_set_speed = speed
                return True
            return False
                
        except Exception as e:
            print(f"設置全局速度失敗: {e}")
            return False
    
    def get_global_speed(self) -> int:
        """獲取當前全局速度"""
        return self.global_speed
```

### 3. 握手循環修改 - 新增速度處理
```python
def _handshake_loop(self):
    """狀態機交握主循環 - 新增速度控制"""
    while self.is_running:
        try:
            # 讀取控制寄存器
            vp_control = self.state_machine.read_control_register(0)      # 440
            unload_control = self.state_machine.read_control_register(1)  # 441
            clear_alarm = self.state_machine.read_control_register(2)     # 442
            emergency_stop = self.state_machine.read_control_register(3)  # 443
            manual_command = self.state_machine.read_control_register(4)  # 444
            
            # === 新增: 處理速度控制指令 ===
            if self.robot.is_connected:
                self.state_machine.process_speed_command(self.robot)
            
            # ... 其他握手邏輯保持不變 ...
            
            # 更新狀態到PLC (含速度)
            self.state_machine.update_status_to_plc()
            
            time.sleep(0.05)  # 50ms循環
            
        except Exception as e:
            print(f"狀態機循環錯誤: {e}")
            time.sleep(1)
```

### 4. 狀態更新修改 - 含速度資訊
```python
def update_status_to_plc(self):
    """更新狀態到PLC - 增強版含速度"""
    try:
        # 更新主狀態寄存器 (400)
        self.modbus_client.write_register(
            address=DobotRegisters.STATUS_REGISTER, 
            value=self.status_register
        )
        
        # ... 其他狀態更新 ...
        
        # 更新全局速度 (419) - 新增
        self.modbus_client.write_register(
            address=DobotRegisters.GLOBAL_SPEED, 
            value=self.current_global_speed
        )
        
    except Exception as e:
        print(f"更新狀態到PLC異常: {e}")
```

## Web端相容性保證

### 手動控制寄存器 (444) - 擴展速度功能
```python
def handle_manual_command(self, command: int):
    """處理Web端手動指令 - 增強版含速度"""
    try:
        if command == 1:  # 手動Flow1
            if self.state_machine.is_ready_for_command():
                self.execute_flow1_async()
        elif command == 2:  # 手動Flow2
            if self.state_machine.is_ready_for_command():
                self.execute_flow2_async()
        elif command >= 10 and command <= 100:  # 速度設定指令 (新增)
            if self.robot.is_connected:
                success = self.robot.set_global_speed(command)
                if success:
                    self.state_machine.update_global_speed_register(command)
        elif command == 99:  # 緊急停止
            self.emergency_stop_all()
            
    except Exception as e:
        print(f"處理Web端手動指令{command}失敗: {e}")
```

## PLC操作指南

### **速度控制操作 (新增)**
```
設定全局速度流程:
1. 準備速度值: 確定要設定的速度 (1-100%)
2. 寫入速度值: 寫入446 = 速度值
3. 生成指令ID: 寫入447 = 唯一ID (1-65535)
4. 觸發設定: 寫入445 = 1
5. 等待處理: 監控445寄存器自動清零
6. 確認生效: 讀取419寄存器確認新速度值
7. 完成操作: 速度設定完成

範例操作序列:
- 設定速度為75%: 寫入446=75, 447=12345, 445=1
- 等待完成: 445自動變為0
- 確認結果: 讀取419=75
```

### VP視覺取料操作 (Flow1)
```
1. 檢查準備: 讀取400 = 9 (00001001)
2. 啟動取料: 寫入440 = 1
3. 監控執行: 讀取400 = 10 (00001010), 402 = 1
4. 等待完成: 讀取400 = 8 (00001000), 402 = 0
5. 清零指令: 寫入440 = 0
6. 確認恢復: 讀取400 = 9 (00001001)
```

### 出料流程操作 (Flow2)
```
前提: 400 = 9 且 440 = 0 (取料已完成且已清零)
1. 檢查準備: 讀取400 = 9, 440 = 0
2. 啟動出料: 寫入441 = 1
3. 監控執行: 讀取400 = 10, 402 = 2
4. 等待完成: 讀取400 = 8, 402 = 0
5. 清零指令: 寫入441 = 0
6. 確認恢復: 讀取400 = 9
```

### 警報處理操作
```
1. 檢測警報: 讀取400 = 12 (00001100)
2. 清除警報: 寫入442 = 1
3. 確認清除: 讀取400 = 9 (Alarm位清除)
4. 清零指令: 寫入442 = 0
```

## 關鍵特點

1. **聯動控制**: Flow2只有在Flow1完成且控制指令清零後才能啟動
2. **狀態隔離**: Ready位控制整體可用性，Running位防止並發執行
3. **錯誤保護**: 任何執行失敗都會觸發Alarm，需要手動重置
4. **Web相容**: 通過444寄存器實現Web端手動控制，不影響自動化流程
5. **完整握手**: 確保PLC與機械臂系統的可靠通訊和狀態同步
6. **速度控制**: 透過445/446/447寄存器實現全局速度的狀態機交握設定 **(新增)**
7. **即時生效**: 速度設定立即生效於所有後續運動指令 **(新增)**
8. **防重複機制**: 指令ID確保同一速度指令不會重複執行 **(新增)**
9. **範圍保護**: 自動驗證速度範圍1-100%，超出範圍自動拒絕 **(新增)**
10. **狀態同步**: 419寄存器即時反映當前生效的全局速度值 **(新增)**

## 速度控制技術規範 **(新增章節)**

### 速度API映射
| 機械臂API | 功能 | 參數範圍 |
|-----------|------|----------|
| SpeedFactor(speed) | 全局速度比例 | 1-100% |
| SpeedJ(speed) | 關節運動速度 | 1-100% |
| SpeedL(speed) | 直線運動速度 | 1-100% |
| AccJ(speed) | 關節運動加速度 | 1-100% |
| AccL(speed) | 直線運動加速度 | 1-100% |

### 速度設定時機
- **初始化時**: 系統啟動時設定預設速度 (50%)
- **運行時**: 可隨時透過狀態機交握修改速度
- **流程中**: 速度修改不影響正在執行的運動，影響後續運動
- **Web端**: 支援滑桿即時調整，範圍1-100%

### 錯誤處理機制
- **範圍檢查**: 自動驗證1-100%範圍
- **連接檢查**: 確認機械臂API連接正常
- **重複檢查**: 指令ID防止重複執行相同速度設定
- **異常捕獲**: 完整的try-catch錯誤處理
- **狀態恢復**: 設定失敗不影響系統其他功能

### 性能特點
- **即時響應**: 50ms握手循環，快速響應速度設定
- **線程安全**: 使用鎖保護狀態機操作
- **記憶體效率**: 最小記憶體占用，無無限堆疊
- **可靠傳輸**: Modbus TCP保證指令可靠傳輸
- **狀態持久**: 速度設定在系統重啟前保持有效