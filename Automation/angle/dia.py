import time
import json
import os
from typing import Dict, Any, Optional
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

class CCD3DiagnosisService:
    """CCD3角度檢測系統診斷服務"""
    
    def __init__(self):
        self.angle_base_address = 700      # angle_main.py 基地址
        self.ccd3_base_address = 800       # CCD3模組基地址
        self.modbus_client = None
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        
    def connect_modbus(self) -> bool:
        """連接Modbus TCP服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3
            )
            
            return self.modbus_client.connect()
        except Exception as e:
            print(f"Modbus連接錯誤: {e}")
            return False
    
    def read_registers_safe(self, address: int, count: int) -> Optional[list]:
        """安全讀取寄存器"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                return None
            
            result = self.modbus_client.read_holding_registers(
                address=address, count=count, slave=1
            )
            
            if result.isError():
                return None
                
            return result.registers
        except Exception as e:
            print(f"讀取寄存器 {address} 錯誤: {e}")
            return None
    
    def write_register_safe(self, address: int, value: int) -> bool:
        """安全寫入寄存器"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                return False
            
            result = self.modbus_client.write_register(
                address=address, value=value, slave=1
            )
            
            return not result.isError()
        except Exception as e:
            print(f"寫入寄存器 {address} 錯誤: {e}")
            return False
    
    def diagnose_full_system(self):
        """完整系統診斷"""
        print("=" * 80)
        print("CCD3角度檢測系統完整診斷開始")
        print("=" * 80)
        
        # 1. 基礎連線診斷
        self.diagnose_basic_connectivity()
        
        # 2. Angle主程序診斷
        self.diagnose_angle_main_module()
        
        # 3. CCD3模組診斷
        self.diagnose_ccd3_module()
        
        # 4. 通訊協議診斷
        self.diagnose_communication_protocol()
        
        # 5. 指令流程診斷
        self.diagnose_command_flow()
        
        # 6. 實際測試診斷
        self.diagnose_actual_test()
        
        print("=" * 80)
        print("診斷完成，請檢查上述問題點")
        print("=" * 80)
    
    def diagnose_basic_connectivity(self):
        """基礎連線診斷"""
        print("\n【1. 基礎連線診斷】")
        print("-" * 40)
        
        # Modbus TCP連接測試
        modbus_ok = self.connect_modbus()
        print(f"✓ Modbus TCP連接: {'成功' if modbus_ok else '失敗'} ({self.server_ip}:{self.server_port})")
        
        if not modbus_ok:
            print("  ❌ 問題1: 主Modbus TCP Server (端口502) 可能未啟動")
            print("     解決方案: 確認主服務器程序正在運行")
            return
        
        # 測試寄存器讀寫能力
        test_result = self.read_registers_safe(self.angle_base_address, 1)
        if test_result is not None:
            print(f"✓ Angle模組寄存器讀取: 成功 (地址{self.angle_base_address})")
        else:
            print(f"❌ Angle模組寄存器讀取: 失敗 (地址{self.angle_base_address})")
            print("  問題2: angle_main.py可能未正常運行或未註冊寄存器")
        
        test_result = self.read_registers_safe(self.ccd3_base_address, 1)
        if test_result is not None:
            print(f"✓ CCD3模組寄存器讀取: 成功 (地址{self.ccd3_base_address})")
        else:
            print(f"❌ CCD3模組寄存器讀取: 失敗 (地址{self.ccd3_base_address})")
            print("  問題3: CCD3模組可能未正常運行或未註冊寄存器")
    
    def diagnose_angle_main_module(self):
        """Angle主程序診斷"""
        print("\n【2. Angle主程序診斷】")
        print("-" * 40)
        
        # 讀取Angle狀態寄存器 (700-714)
        status_regs = self.read_registers_safe(self.angle_base_address, 15)
        if status_regs is None:
            print("❌ 無法讀取Angle狀態寄存器")
            print("  問題4: angle_main.py可能崩潰或寄存器映射錯誤")
            return
        
        # 解析狀態
        status_register = status_regs[0]
        ready = bool(status_register & (1 << 0))
        running = bool(status_register & (1 << 1))
        alarm = bool(status_register & (1 << 2))
        initialized = bool(status_register & (1 << 3))
        ccd_detecting = bool(status_register & (1 << 4))
        motor_moving = bool(status_register & (1 << 5))
        
        modbus_connected = bool(status_regs[1])
        motor_connected = bool(status_regs[2])
        error_code = status_regs[3]
        operation_count = (status_regs[5] << 16) | status_regs[4]
        error_count = status_regs[6]
        
        print(f"✓ Angle狀態寄存器 (700): {status_register} (二進制: {bin(status_register)})")
        print(f"  Ready: {ready}, Running: {running}, Alarm: {alarm}")
        print(f"  Initialized: {initialized}, CCD_Detecting: {ccd_detecting}, Motor_Moving: {motor_moving}")
        print(f"✓ Modbus連接狀態 (701): {modbus_connected}")
        print(f"✓ 馬達連接狀態 (702): {motor_connected}")
        print(f"✓ 錯誤代碼 (703): {error_code}")
        print(f"✓ 操作計數 (704-705): {operation_count}")
        print(f"✓ 錯誤計數 (706): {error_count}")
        
        # 問題判斷
        if not ready:
            print("❌ 問題5: Angle系統未Ready，無法執行指令")
            if alarm:
                print("  原因: 系統處於Alarm狀態")
            if not initialized:
                print("  原因: 系統未完全初始化")
        
        if not modbus_connected:
            print("❌ 問題6: Angle模組報告Modbus未連接")
        
        if not motor_connected:
            print("❌ 問題7: 馬達驅動器未連接 (COM5可能有問題)")
        
        if error_code != 0:
            print(f"❌ 問題8: 系統錯誤代碼 {error_code}")
    
    def diagnose_ccd3_module(self):
        """CCD3模組診斷"""
        print("\n【3. CCD3模組診斷】")
        print("-" * 40)
        
        # 讀取CCD3握手寄存器 (800-801)
        handshake_regs = self.read_registers_safe(self.ccd3_base_address, 2)
        if handshake_regs is None:
            print("❌ 無法讀取CCD3握手寄存器")
            print("  問題9: CCD3模組可能未運行")
            print("  解決方案: 確認CCD3AngleDetection.py正在運行且端口5052可訪問")
            return
        
        control_command = handshake_regs[0]
        status_register = handshake_regs[1]
        
        ready = bool(status_register & (1 << 0))
        running = bool(status_register & (1 << 1))
        alarm = bool(status_register & (1 << 2))
        initialized = bool(status_register & (1 << 3))
        
        print(f"✓ CCD3控制指令 (800): {control_command}")
        print(f"✓ CCD3狀態寄存器 (801): {status_register} (二進制: {bin(status_register)})")
        print(f"  Ready: {ready}, Running: {running}, Alarm: {alarm}, Initialized: {initialized}")
        
        # 讀取CCD3檢測參數 (810-819)
        param_regs = self.read_registers_safe(self.ccd3_base_address + 10, 10)
        if param_regs:
            detection_mode = param_regs[0]
            min_area_rate = param_regs[1]
            sequence_mode = param_regs[2]
            gaussian_kernel = param_regs[3]
            threshold_mode = param_regs[4]
            manual_threshold = param_regs[5]
            
            print(f"✓ CCD3檢測參數:")
            print(f"  檢測模式 (810): {detection_mode} ({'橢圓擬合' if detection_mode == 0 else '最小外接矩形'})")
            print(f"  最小面積比例 (811): {min_area_rate} (實際比例: {min_area_rate/1000.0})")
            print(f"  序列模式 (812): {sequence_mode}")
            print(f"  高斯核大小 (813): {gaussian_kernel}")
            print(f"  閾值模式 (814): {threshold_mode} ({'OTSU自動' if threshold_mode == 0 else '手動'})")
            print(f"  手動閾值 (815): {manual_threshold}")
        
        # 讀取CCD3檢測結果 (840-859)
        result_regs = self.read_registers_safe(self.ccd3_base_address + 40, 20)
        if result_regs:
            success_flag = result_regs[0]
            center_x = result_regs[1]
            center_y = result_regs[2]
            angle_high = result_regs[3]
            angle_low = result_regs[4]
            
            print(f"✓ CCD3最後檢測結果:")
            print(f"  成功標誌 (840): {success_flag}")
            print(f"  中心座標 (841-842): ({center_x}, {center_y})")
            
            if success_flag == 1:
                # 重建角度
                angle_int = (angle_high << 16) | angle_low
                if angle_int >= 2**31:
                    angle_int -= 2**32
                angle_degrees = angle_int / 100.0
                print(f"  檢測角度 (843-844): {angle_degrees:.2f}度")
            else:
                print(f"  檢測角度: 無效 (高位={angle_high}, 低位={angle_low})")
        
        # 問題判斷
        if not initialized:
            print("❌ 問題10: CCD3模組未初始化")
            print("  可能原因: 相機連接失敗 (192.168.1.10)")
        
        if alarm:
            print("❌ 問題11: CCD3模組處於Alarm狀態")
            print("  可能原因: 相機故障或檢測參數錯誤")
        
        if not ready:
            print("❌ 問題12: CCD3模組未Ready")
    
    def diagnose_communication_protocol(self):
        """通訊協議診斷"""
        print("\n【4. 通訊協議診斷】")
        print("-" * 40)
        
        print("✓ 檢查Angle → CCD3指令發送邏輯:")
        
        # 檢查angle_main.py中的CCD3觸發邏輯
        print("  angle_main.py觸發CCD3步驟:")
        print("  1. trigger_ccd3_detection(): 寫入800=16")
        print("  2. wait_ccd3_complete(): 輪詢801狀態直到Ready=1,Running=0")
        print("  3. read_ccd3_angle(): 讀取843-844角度數據")
        
        # 實際測試寫入CCD3指令
        print("✓ 測試寫入CCD3指令 (800=16):")
        write_success = self.write_register_safe(self.ccd3_base_address, 16)
        print(f"  寫入結果: {'成功' if write_success else '失敗'}")
        
        if write_success:
            time.sleep(0.5)  # 等待一下
            # 檢查CCD3是否收到指令
            control_reg = self.read_registers_safe(self.ccd3_base_address, 1)
            if control_reg:
                print(f"  CCD3收到指令: {control_reg[0]} ({'已收到' if control_reg[0] == 16 else '未收到或已清除'})")
        
        # 檢查CCD3是否正確處理指令
        print("✓ CCD3指令處理邏輯診斷:")
        print("  CCD3AngleDetection.py應該:")
        print("  1. _process_control_commands()檢測到新指令16")
        print("  2. _handle_control_command()設置Running=1")
        print("  3. _execute_command_async()執行拍照+檢測")
        print("  4. capture_and_detect_angle()返回結果")
        print("  5. write_detection_result()寫入840-859")
        print("  6. 完成後設置Running=0")
    
    def diagnose_command_flow(self):
        """指令流程診斷"""
        print("\n【5. 指令流程診斷】")
        print("-" * 40)
        
        # 檢查完整的角度校正流程
        print("✓ 完整角度校正流程分析:")
        print("  PLC發送指令1 → angle_main.py收到")
        print("  → trigger_ccd3_detection()")
        print("  → 寫入CCD3寄存器800=16")
        print("  → CCD3執行拍照+檢測")
        print("  → angle_main.py輪詢等待CCD3完成")
        print("  → read_ccd3_angle()讀取843-844")
        print("  → calculate_motor_position()計算馬達位置")
        print("  → send_motor_command()發送馬達指令")
        
        # 檢查可能的失敗點
        print("\n✓ 可能的失敗點分析:")
        
        # 失敗點1: CCD3檢測未成功
        result_regs = self.read_registers_safe(self.ccd3_base_address + 40, 5)
        if result_regs:
            success_flag = result_regs[0]
            if success_flag != 1:
                print("  ❌ 失敗點1: CCD3檢測未成功 (840寄存器=0)")
                print("    可能原因:")
                print("    - 相機未捕獲到有效圖像")
                print("    - 檢測參數設置不當")
                print("    - 物體不在視野內或光照條件不佳")
                print("    - 檢測算法未找到符合條件的輪廓")
        
        # 失敗點2: Angle模組超時
        print("  ✓ angle_main.py等待超時設置:")
        print("    CCD檢測超時: 10秒")
        print("    馬達運動超時: 30秒")
        
        # 失敗點3: 寄存器數據格式
        print("  ✓ 32位角度數據格式檢查:")
        if result_regs and len(result_regs) >= 5:
            angle_high = result_regs[3]
            angle_low = result_regs[4]
            print(f"    角度高位 (843): {angle_high}")
            print(f"    角度低位 (844): {angle_low}")
            
            if angle_high == 0 and angle_low == 0:
                print("    ❌ 問題13: 角度數據為0，可能是檢測失敗或未寫入")
    
    def diagnose_actual_test(self):
        """實際測試診斷"""
        print("\n【6. 實際測試診斷】")
        print("-" * 40)
        
        print("✓ 執行實際CCD3檢測測試...")
        
        # 1. 檢查CCD3初始狀態
        status_regs = self.read_registers_safe(self.ccd3_base_address + 1, 1)
        if not status_regs:
            print("❌ 無法讀取CCD3狀態")
            return
        
        status_register = status_regs[0]
        ready = bool(status_register & (1 << 0))
        running = bool(status_register & (1 << 1))
        
        print(f"  CCD3當前狀態: Ready={ready}, Running={running}")
        
        if not ready:
            print("❌ CCD3未Ready，跳過實際測試")
            return
        
        # 2. 清除之前的結果
        print("  清除CCD3之前的檢測結果...")
        clear_success = self.write_register_safe(self.ccd3_base_address, 0)
        time.sleep(0.5)
        
        # 3. 發送檢測指令
        print("  發送CCD3檢測指令 (800=16)...")
        cmd_success = self.write_register_safe(self.ccd3_base_address, 16)
        
        if not cmd_success:
            print("❌ 發送指令失敗")
            return
        
        # 4. 監控執行過程
        print("  監控CCD3執行過程...")
        max_wait_time = 15  # 15秒超時
        start_time = time.time()
        
        while time.time() - start_time < max_wait_time:
            status_regs = self.read_registers_safe(self.ccd3_base_address + 1, 1)
            if status_regs:
                status = status_regs[0]
                ready = bool(status & (1 << 0))
                running = bool(status & (1 << 1))
                
                print(f"    時間 {time.time() - start_time:.1f}s: Ready={ready}, Running={running}")
                
                if ready and not running:
                    print("  ✓ CCD3執行完成")
                    break
            
            time.sleep(1)
        else:
            print("  ❌ CCD3執行超時")
            return
        
        # 5. 檢查結果
        print("  檢查CCD3檢測結果...")
        result_regs = self.read_registers_safe(self.ccd3_base_address + 40, 10)
        if result_regs:
            success_flag = result_regs[0]
            center_x = result_regs[1]
            center_y = result_regs[2]
            angle_high = result_regs[3]
            angle_low = result_regs[4]
            
            print(f"    成功標誌 (840): {success_flag}")
            print(f"    中心座標: ({center_x}, {center_y})")
            
            if success_flag == 1:
                angle_int = (angle_high << 16) | angle_low
                if angle_int >= 2**31:
                    angle_int -= 2**32
                angle_degrees = angle_int / 100.0
                print(f"    檢測角度: {angle_degrees:.2f}度")
                print("  ✓ CCD3檢測成功")
            else:
                print("  ❌ CCD3檢測失敗")
                print("    可能原因:")
                print("    - 沒有在視野內找到物體")
                print("    - 檢測參數不適合當前物體")
                print("    - 光照條件不佳")
                print("    - 相機故障")
        
        # 6. 清除指令
        print("  清除CCD3指令...")
        self.write_register_safe(self.ccd3_base_address, 0)
    
    def get_diagnostic_summary(self):
        """獲取診斷總結"""
        print("\n【診斷總結和建議】")
        print("-" * 40)
        
        print("1. 檢查CCD3模組是否正常運行:")
        print("   - 確認CCD3AngleDetection.py程序正在運行")
        print("   - 訪問 http://localhost:5052 檢查Web介面")
        print("   - 檢查相機連接 (192.168.1.10)")
        
        print("\n2. 檢查angle_main.py的CCD3通訊邏輯:")
        print("   - 確認trigger_ccd3_detection()正確寫入800=16")
        print("   - 確認wait_ccd3_complete()正確輪詢801狀態")
        print("   - 確認read_ccd3_angle()正確讀取843-844")
        
        print("\n3. 檢查CCD3檢測參數:")
        print("   - 檢測模式是否適合物體 (橢圓擬合vs最小外接矩形)")
        print("   - 最小面積比例是否合理")
        print("   - 高斯模糊和閾值參數是否合適")
        
        print("\n4. 物理環境檢查:")
        print("   - 物體是否在相機視野內")
        print("   - 光照條件是否充足")
        print("   - 物體對比度是否足夠")
        
        print("\n5. 如果CCD3檢測成功但angle讀取失敗:")
        print("   - 檢查32位角度數據的合併邏輯")
        print("   - 檢查寄存器地址映射是否正確")
        print("   - 檢查CCD3的write_detection_result()方法")

if __name__ == '__main__':
    print("CCD3角度檢測系統診斷程序")
    print("此程序將全面診斷angle_main.py和CCD3模組的通訊問題")
    
    diagnosis = CCD3DiagnosisService()
    diagnosis.diagnose_full_system()
    diagnosis.get_diagnostic_summary()
    
    if diagnosis.modbus_client:
        diagnosis.modbus_client.close()
    
    print("\n診斷程序結束")