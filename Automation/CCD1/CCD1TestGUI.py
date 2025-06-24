# -*- coding: utf-8 -*-
"""
CCD1TestGUI.py - CCD1高層API測試工具
基於CustomTkinter開發的GUI測試工具，用於測試CCD1HighLevel.py的功能
"""

import customtkinter as ctk
import threading
import time
from typing import Optional
from datetime import datetime
import tkinter.messagebox as messagebox

# 導入CCD1高層API
try:
    from CCD1HighLevel import CCD1HighLevelAPI, CircleWorldCoord
    CCD1_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ 無法導入CCD1HighLevel: {e}")
    CCD1_AVAILABLE = False


class CCD1TestGUI:
    """CCD1高層API測試GUI"""
    
    def __init__(self):
        # 設置主題
        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")
        
        # 創建主視窗
        self.root = ctk.CTk()
        self.root.title("CCD1高層API測試工具")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        # CCD1 API實例
        self.ccd1: Optional[CCD1HighLevelAPI] = None
        self.connected = False
        
        # 狀態更新線程控制
        self.status_thread_running = False
        self.status_thread = None
        
        # 創建UI
        self.setup_ui()
        
        # 自動嘗試連接
        self.connect_ccd1()
    
    def setup_ui(self):
        """創建用戶介面"""
        # 主框架
        main_frame = ctk.CTkFrame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # 標題
        title_label = ctk.CTkLabel(
            main_frame, 
            text="CCD1高層API測試工具", 
            font=ctk.CTkFont(size=20, weight="bold")
        )
        title_label.pack(pady=10)
        
        # 連接狀態框架
        self.create_connection_frame(main_frame)
        
        # 控制按鈕框架
        self.create_control_frame(main_frame)
        
        # FIFO佇列顯示框架
        self.create_fifo_frame(main_frame)
        
        # 系統狀態框架
        self.create_status_frame(main_frame)
    
    def create_connection_frame(self, parent):
        """創建連接狀態框架"""
        conn_frame = ctk.CTkFrame(parent)
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        # 連接狀態標籤
        self.conn_status_label = ctk.CTkLabel(
            conn_frame, 
            text="連接狀態: 未連接", 
            font=ctk.CTkFont(size=14)
        )
        self.conn_status_label.pack(side="left", padx=10, pady=10)
        
        # 連接按鈕
        self.connect_btn = ctk.CTkButton(
            conn_frame,
            text="連接CCD1",
            command=self.connect_ccd1,
            width=100
        )
        self.connect_btn.pack(side="right", padx=10, pady=10)
        
        # 斷開按鈕
        self.disconnect_btn = ctk.CTkButton(
            conn_frame,
            text="斷開連接",
            command=self.disconnect_ccd1,
            width=100,
            state="disabled"
        )
        self.disconnect_btn.pack(side="right", padx=5, pady=10)
    
    def create_control_frame(self, parent):
        """創建控制按鈕框架"""
        control_frame = ctk.CTkFrame(parent)
        control_frame.pack(fill="x", padx=10, pady=5)
        
        control_title = ctk.CTkLabel(
            control_frame, 
            text="控制操作", 
            font=ctk.CTkFont(size=16, weight="bold")
        )
        control_title.pack(pady=5)
        
        # 按鈕容器
        btn_container = ctk.CTkFrame(control_frame)
        btn_container.pack(fill="x", padx=10, pady=5)
        
        # 檢測按鈕
        self.detect_btn = ctk.CTkButton(
            btn_container,
            text="執行拍照+檢測",
            command=self.execute_detection,
            width=150,
            height=40,
            font=ctk.CTkFont(size=14, weight="bold")
        )
        self.detect_btn.pack(side="left", padx=10, pady=10)
        
        # 獲取座標按鈕
        self.get_coord_btn = ctk.CTkButton(
            btn_container,
            text="獲取下一個圓心座標",
            command=self.get_next_coordinate,
            width=150,
            height=40,
            font=ctk.CTkFont(size=14, weight="bold")
        )
        self.get_coord_btn.pack(side="left", padx=10, pady=10)
        
        # 清空佇列按鈕
        self.clear_queue_btn = ctk.CTkButton(
            btn_container,
            text="清空FIFO佇列",
            command=self.clear_queue,
            width=120,
            height=40,
            fg_color="orange",
            hover_color="darkorange"
        )
        self.clear_queue_btn.pack(side="left", padx=10, pady=10)
        
        # 刷新狀態按鈕
        self.refresh_btn = ctk.CTkButton(
            btn_container,
            text="刷新狀態",
            command=self.refresh_status,
            width=100,
            height=40,
            fg_color="green",
            hover_color="darkgreen"
        )
        self.refresh_btn.pack(side="right", padx=10, pady=10)
    
    def create_fifo_frame(self, parent):
        """創建FIFO佇列顯示框架"""
        fifo_frame = ctk.CTkFrame(parent)
        fifo_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        fifo_title = ctk.CTkLabel(
            fifo_frame, 
            text="FIFO佇列檢測結果", 
            font=ctk.CTkFont(size=16, weight="bold")
        )
        fifo_title.pack(pady=5)
        
        # 佇列狀態標籤
        self.queue_status_label = ctk.CTkLabel(
            fifo_frame, 
            text="佇列長度: 0 | 最後檢測: 0個", 
            font=ctk.CTkFont(size=12)
        )
        self.queue_status_label.pack(pady=5)
        
        # 檢測結果文本框
        self.result_text = ctk.CTkTextbox(
            fifo_frame,
            width=750,
            height=250,
            font=ctk.CTkFont(family="Consolas", size=12)
        )
        self.result_text.pack(fill="both", expand=True, padx=10, pady=10)
        
        # 初始提示
        self.result_text.insert("1.0", "=== CCD1高層API測試工具 ===\n")
        self.result_text.insert("end", "點擊上方按鈕開始測試...\n\n")
        self.result_text.insert("end", "操作說明:\n")
        self.result_text.insert("end", "1. 執行拍照+檢測: 觸發CCD1檢測，結果自動加入FIFO佇列\n")
        self.result_text.insert("end", "2. 獲取下一個圓心座標: 從FIFO佇列取出一個座標（佇列空時自動檢測）\n")
        self.result_text.insert("end", "3. 清空FIFO佇列: 清空所有待處理的圓心座標\n")
        self.result_text.insert("end", "4. 刷新狀態: 更新連接和佇列狀態資訊\n\n")
    
    def create_status_frame(self, parent):
        """創建系統狀態框架"""
        status_frame = ctk.CTkFrame(parent)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        status_title = ctk.CTkLabel(
            status_frame, 
            text="系統狀態", 
            font=ctk.CTkFont(size=16, weight="bold")
        )
        status_title.pack(pady=5)
        
        # 狀態指示器容器
        status_container = ctk.CTkFrame(status_frame)
        status_container.pack(fill="x", padx=10, pady=5)
        
        # Ready狀態
        self.ready_label = ctk.CTkLabel(
            status_container, 
            text="Ready: ❌", 
            font=ctk.CTkFont(size=12)
        )
        self.ready_label.pack(side="left", padx=10, pady=5)
        
        # Running狀態
        self.running_label = ctk.CTkLabel(
            status_container, 
            text="Running: ❌", 
            font=ctk.CTkFont(size=12)
        )
        self.running_label.pack(side="left", padx=10, pady=5)
        
        # Alarm狀態
        self.alarm_label = ctk.CTkLabel(
            status_container, 
            text="Alarm: ❌", 
            font=ctk.CTkFont(size=12)
        )
        self.alarm_label.pack(side="left", padx=10, pady=5)
        
        # 世界座標狀態
        self.world_coord_label = ctk.CTkLabel(
            status_container, 
            text="世界座標: ❌", 
            font=ctk.CTkFont(size=12)
        )
        self.world_coord_label.pack(side="left", padx=10, pady=5)
    
    def connect_ccd1(self):
        """連接CCD1"""
        if not CCD1_AVAILABLE:
            self.log_message("❌ CCD1HighLevel模組不可用")
            messagebox.showerror("錯誤", "CCD1HighLevel模組不可用，請檢查導入")
            return
        
        self.log_message("🔗 正在連接CCD1...")
        
        def connect_thread():
            try:
                self.ccd1 = CCD1HighLevelAPI()
                if self.ccd1.connected:
                    self.connected = True
                    self.log_message("✅ CCD1連接成功")
                    
                    # 更新UI狀態
                    self.root.after(0, self.update_connection_ui, True)
                    
                    # 啟動狀態監控線程
                    self.start_status_thread()
                else:
                    self.log_message("❌ CCD1連接失敗")
                    self.root.after(0, self.update_connection_ui, False)
                    
            except Exception as e:
                self.log_message(f"❌ CCD1連接異常: {e}")
                self.root.after(0, self.update_connection_ui, False)
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def disconnect_ccd1(self):
        """斷開CCD1連接"""
        if self.ccd1:
            self.stop_status_thread()
            self.ccd1.disconnect()
            self.ccd1 = None
        
        self.connected = False
        self.update_connection_ui(False)
        self.log_message("🔌 CCD1連接已斷開")
    
    def update_connection_ui(self, connected: bool):
        """更新連接狀態UI"""
        if connected:
            self.conn_status_label.configure(text="連接狀態: 已連接", text_color="green")
            self.connect_btn.configure(state="disabled")
            self.disconnect_btn.configure(state="normal")
            
            # 啟用控制按鈕
            self.detect_btn.configure(state="normal")
            self.get_coord_btn.configure(state="normal")
            self.clear_queue_btn.configure(state="normal")
        else:
            self.conn_status_label.configure(text="連接狀態: 未連接", text_color="red")
            self.connect_btn.configure(state="normal")
            self.disconnect_btn.configure(state="disabled")
            
            # 禁用控制按鈕
            self.detect_btn.configure(state="disabled")
            self.get_coord_btn.configure(state="disabled")
            self.clear_queue_btn.configure(state="disabled")
    
    def execute_detection(self):
        """執行拍照+檢測"""
        if not self.connected or not self.ccd1:
            self.log_message("❌ CCD1未連接")
            return
        
        self.log_message("📸 執行拍照+檢測...")
        
        def detection_thread():
            try:
                success = self.ccd1.capture_and_detect()
                if success:
                    queue_status = self.ccd1.get_queue_status()
                    self.root.after(0, self.log_message, 
                                  f"✅ 檢測完成! 新增 {queue_status['last_detection_count']} 個圓心到佇列")
                    self.root.after(0, self.update_queue_status)
                else:
                    self.root.after(0, self.log_message, "❌ 檢測失敗")
            except Exception as e:
                self.root.after(0, self.log_message, f"❌ 檢測異常: {e}")
        
        threading.Thread(target=detection_thread, daemon=True).start()
    
    def get_next_coordinate(self):
        """獲取下一個圓心座標"""
        if not self.connected or not self.ccd1:
            self.log_message("❌ CCD1未連接")
            return
        
        self.log_message("🎯 獲取下一個圓心座標...")
        
        def get_coord_thread():
            try:
                coord = self.ccd1.get_next_circle_world_coord()
                if coord:
                    self.root.after(0, self.display_coordinate, coord)
                    self.root.after(0, self.update_queue_status)
                else:
                    self.root.after(0, self.log_message, "⚠️ 佇列為空，無可用座標")
            except Exception as e:
                self.root.after(0, self.log_message, f"❌ 獲取座標異常: {e}")
        
        threading.Thread(target=get_coord_thread, daemon=True).start()
    
    def display_coordinate(self, coord: CircleWorldCoord):
        """顯示圓心座標"""
        coord_info = (
            f"🔵 圓心座標 #{coord.id} | {coord.timestamp}\n"
            f"   世界座標: ({coord.world_x:.2f}, {coord.world_y:.2f}) mm\n"
            f"   像素座標: ({coord.pixel_x}, {coord.pixel_y}) px\n"
            f"   半徑: {coord.radius} px\n"
        )
        self.log_message(coord_info)
    
    def clear_queue(self):
        """清空FIFO佇列"""
        if not self.connected or not self.ccd1:
            self.log_message("❌ CCD1未連接")
            return
        
        try:
            self.ccd1.clear_queue()
            self.log_message("🗑️ FIFO佇列已清空")
            self.update_queue_status()
        except Exception as e:
            self.log_message(f"❌ 清空佇列異常: {e}")
    
    def refresh_status(self):
        """刷新狀態"""
        if not self.connected or not self.ccd1:
            self.log_message("❌ CCD1未連接")
            return
        
        try:
            system_status = self.ccd1.get_system_status()
            queue_status = self.ccd1.get_queue_status()
            
            self.log_message("🔄 狀態已刷新")
            self.update_queue_status()
            self.update_system_status(system_status)
            
        except Exception as e:
            self.log_message(f"❌ 刷新狀態異常: {e}")
    
    def update_queue_status(self):
        """更新佇列狀態"""
        if not self.connected or not self.ccd1:
            return
        
        try:
            queue_status = self.ccd1.get_queue_status()
            self.queue_status_label.configure(
                text=f"佇列長度: {queue_status['queue_length']} | "
                     f"最後檢測: {queue_status['last_detection_count']}個"
            )
        except Exception as e:
            print(f"更新佇列狀態異常: {e}")
    
    def update_system_status(self, status: dict = None):
        """更新系統狀態"""
        if not self.connected or not self.ccd1:
            # 重置狀態顯示
            self.ready_label.configure(text="Ready: ❌", text_color="red")
            self.running_label.configure(text="Running: ❌", text_color="red")
            self.alarm_label.configure(text="Alarm: ❌", text_color="red")
            self.world_coord_label.configure(text="世界座標: ❌", text_color="red")
            return
        
        try:
            if status is None:
                status = self.ccd1.get_system_status()
            
            # 更新Ready狀態
            if status.get('ready', False):
                self.ready_label.configure(text="Ready: ✅", text_color="green")
            else:
                self.ready_label.configure(text="Ready: ❌", text_color="red")
            
            # 更新Running狀態
            if status.get('running', False):
                self.running_label.configure(text="Running: ✅", text_color="orange")
            else:
                self.running_label.configure(text="Running: ❌", text_color="gray")
            
            # 更新Alarm狀態
            if status.get('alarm', False):
                self.alarm_label.configure(text="Alarm: ⚠️", text_color="red")
            else:
                self.alarm_label.configure(text="Alarm: ✅", text_color="green")
            
            # 更新世界座標狀態
            if status.get('world_coord_valid', False):
                self.world_coord_label.configure(text="世界座標: ✅", text_color="green")
            else:
                self.world_coord_label.configure(text="世界座標: ❌", text_color="orange")
                
        except Exception as e:
            print(f"更新系統狀態異常: {e}")
    
    def start_status_thread(self):
        """啟動狀態監控線程"""
        if self.status_thread_running:
            return
        
        self.status_thread_running = True
        
        def status_monitor():
            while self.status_thread_running and self.connected:
                try:
                    if self.ccd1:
                        queue_status = self.ccd1.get_queue_status()
                        system_status = self.ccd1.get_system_status()
                        
                        self.root.after(0, self.update_queue_status)
                        self.root.after(0, self.update_system_status, system_status)
                    
                    time.sleep(2.0)  # 2秒更新一次
                except Exception as e:
                    print(f"狀態監控線程異常: {e}")
                    break
            
            self.status_thread_running = False
        
        self.status_thread = threading.Thread(target=status_monitor, daemon=True)
        self.status_thread.start()
    
    def stop_status_thread(self):
        """停止狀態監控線程"""
        self.status_thread_running = False
        if self.status_thread and self.status_thread.is_alive():
            self.status_thread.join(timeout=1.0)
    
    def log_message(self, message: str):
        """記錄訊息到文本框"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.result_text.insert("end", log_entry)
        self.result_text.see("end")  # 自動滾動到底部
    
    def on_closing(self):
        """視窗關閉事件"""
        self.stop_status_thread()
        if self.ccd1:
            self.ccd1.disconnect()
        self.root.destroy()
    
    def run(self):
        """運行GUI"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()


def main():
    """主函數"""
    print("🚀 啟動CCD1高層API測試工具...")
    
    # 檢查CustomTkinter
    try:
        import customtkinter
        print("✅ CustomTkinter已安裝")
    except ImportError:
        print("❌ CustomTkinter未安裝，請執行: pip install customtkinter")
        return
    
    # 檢查CCD1模組
    if not CCD1_AVAILABLE:
        print("⚠️ CCD1HighLevel模組不可用，GUI將在受限模式下運行")
    
    # 啟動GUI
    app = CCD1TestGUI()
    app.run()


if __name__ == "__main__":
    main()