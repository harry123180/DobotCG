#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time
from tkinter import messagebox

class LEDTestTool:
    def __init__(self):
        # 設置CustomTkinter主題
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        
        # 主視窗
        self.root = ctk.CTk()
        self.root.title("LED控制器測試工具 (COM6)")
        self.root.geometry("800x600")
        
        # 串口連接
        self.serial_connection = None
        self.connected = False
        
        # LED狀態
        self.led_states = [False, False, False, False]  # L1-L4
        self.led_brightness = [0, 0, 0, 0]  # L1-L4亮度 (0-511)
        
        self.setup_ui()
        
    def setup_ui(self):
        """設置用戶界面"""
        # 主框架
        main_frame = ctk.CTkFrame(self.root)
        main_frame.pack(fill="both", expand=True, padx=20, pady=20)
        
        # 標題
        title_label = ctk.CTkLabel(
            main_frame, 
            text="LED控制器測試工具", 
            font=ctk.CTkFont(size=24, weight="bold")
        )
        title_label.pack(pady=(20, 30))
        
        # 連接區域
        self.setup_connection_frame(main_frame)
        
        # LED控制區域
        self.setup_led_control_frame(main_frame)
        
        # 日誌區域
        self.setup_log_frame(main_frame)
        
    def setup_connection_frame(self, parent):
        """設置連接控制區域"""
        conn_frame = ctk.CTkFrame(parent)
        conn_frame.pack(fill="x", padx=20, pady=(0, 20))
        
        # 標題
        conn_title = ctk.CTkLabel(conn_frame, text="串口連接", font=ctk.CTkFont(size=16, weight="bold"))
        conn_title.pack(pady=(15, 10))
        
        # 連接控制
        controls_frame = ctk.CTkFrame(conn_frame)
        controls_frame.pack(fill="x", padx=20, pady=(0, 15))
        
        # COM端口選擇
        port_frame = ctk.CTkFrame(controls_frame)
        port_frame.pack(side="left", padx=(10, 20), pady=10)
        
        ctk.CTkLabel(port_frame, text="COM端口:").pack(side="left", padx=(10, 5))
        self.port_var = ctk.StringVar(value="COM6")
        self.port_combo = ctk.CTkComboBox(
            port_frame, 
            variable=self.port_var,
            values=self.get_com_ports(),
            width=100
        )
        self.port_combo.pack(side="left", padx=(0, 10))
        
        # 刷新端口按鈕
        refresh_btn = ctk.CTkButton(
            port_frame, 
            text="刷新", 
            command=self.refresh_ports,
            width=60
        )
        refresh_btn.pack(side="left", padx=(0, 10))
        
        # 連接按鈕
        self.connect_btn = ctk.CTkButton(
            controls_frame, 
            text="連接", 
            command=self.toggle_connection,
            width=100
        )
        self.connect_btn.pack(side="right", padx=(20, 10), pady=10)
        
        # 狀態指示
        self.status_label = ctk.CTkLabel(
            controls_frame, 
            text="未連接", 
            text_color="red"
        )
        self.status_label.pack(side="right", padx=(20, 10), pady=10)
        
    def setup_led_control_frame(self, parent):
        """設置LED控制區域"""
        led_frame = ctk.CTkFrame(parent)
        led_frame.pack(fill="both", expand=True, padx=20, pady=(0, 20))
        
        # 標題
        led_title = ctk.CTkLabel(led_frame, text="LED控制", font=ctk.CTkFont(size=16, weight="bold"))
        led_title.pack(pady=(15, 20))
        
        # LED控制網格
        grid_frame = ctk.CTkFrame(led_frame)
        grid_frame.pack(fill="both", expand=True, padx=20, pady=(0, 15))
        
        # 配置網格權重
        for i in range(4):
            grid_frame.grid_columnconfigure(i, weight=1)
        for i in range(4):
            grid_frame.grid_rowconfigure(i, weight=1)
        
        # 創建LED控制元件
        self.led_controls = []
        for i in range(4):
            led_control = self.create_led_control(grid_frame, i)
            led_control.grid(row=0, column=i, padx=10, pady=10, sticky="nsew")
            self.led_controls.append(led_control)
        
        # 全域控制按鈕
        global_frame = ctk.CTkFrame(led_frame)
        global_frame.pack(fill="x", padx=20, pady=(10, 15))
        
        all_on_btn = ctk.CTkButton(
            global_frame, 
            text="全部開啟", 
            command=self.all_on,
            height=40
        )
        all_on_btn.pack(side="left", padx=(20, 10), pady=15)
        
        all_off_btn = ctk.CTkButton(
            global_frame, 
            text="全部關閉", 
            command=self.all_off,
            height=40
        )
        all_off_btn.pack(side="left", padx=(10, 10), pady=15)
        
        reset_btn = ctk.CTkButton(
            global_frame, 
            text="重置設備", 
            command=self.reset_device,
            height=40
        )
        reset_btn.pack(side="right", padx=(10, 20), pady=15)
        
    def create_led_control(self, parent, channel):
        """創建單個LED控制組件"""
        led_frame = ctk.CTkFrame(parent)
        
        # 通道標題
        title = ctk.CTkLabel(
            led_frame, 
            text=f"L{channel+1} 通道", 
            font=ctk.CTkFont(size=14, weight="bold")
        )
        title.pack(pady=(15, 10))
        
        # 開關按鈕
        toggle_btn = ctk.CTkButton(
            led_frame,
            text="關閉",
            command=lambda: self.toggle_led(channel),
            height=40,
            fg_color="gray",
            text_color="white"
        )
        toggle_btn.pack(pady=(0, 15))
        
        # 亮度標籤
        brightness_label = ctk.CTkLabel(led_frame, text="亮度: 0")
        brightness_label.pack(pady=(0, 5))
        
        # 亮度滑桿
        brightness_slider = ctk.CTkSlider(
            led_frame,
            from_=0,
            to=511,
            number_of_steps=511,
            command=lambda value, ch=channel: self.set_brightness(ch, int(value)),
            height=20
        )
        brightness_slider.set(0)
        brightness_slider.pack(pady=(0, 15), padx=15, fill="x")
        
        # 存儲控制元件引用
        led_frame.toggle_btn = toggle_btn
        led_frame.brightness_label = brightness_label
        led_frame.brightness_slider = brightness_slider
        
        return led_frame
        
    def setup_log_frame(self, parent):
        """設置日誌區域"""
        log_frame = ctk.CTkFrame(parent)
        log_frame.pack(fill="x", padx=20, pady=(0, 20))
        
        log_title = ctk.CTkLabel(log_frame, text="通訊日誌", font=ctk.CTkFont(size=14, weight="bold"))
        log_title.pack(pady=(10, 5))
        
        self.log_text = ctk.CTkTextbox(log_frame, height=120)
        self.log_text.pack(fill="x", padx=15, pady=(0, 15))
        
    def get_com_ports(self):
        """獲取可用COM端口"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports] if ports else ["COM6"]
        
    def refresh_ports(self):
        """刷新COM端口列表"""
        ports = self.get_com_ports()
        self.port_combo.configure(values=ports)
        self.log_message("已刷新COM端口列表")
        
    def toggle_connection(self):
        """切換連接狀態"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        """連接串口"""
        try:
            port = self.port_var.get()
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=9600,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=1.0
            )
            
            self.connected = True
            self.connect_btn.configure(text="斷開")
            self.status_label.configure(text="已連接", text_color="green")
            self.log_message(f"成功連接到 {port}")
            
        except Exception as e:
            messagebox.showerror("連接錯誤", f"無法連接到 {self.port_var.get()}:\n{str(e)}")
            self.log_message(f"連接失敗: {str(e)}")
            
    def disconnect(self):
        """斷開串口"""
        try:
            if self.serial_connection:
                self.serial_connection.close()
                self.serial_connection = None
                
            self.connected = False
            self.connect_btn.configure(text="連接")
            self.status_label.configure(text="未連接", text_color="red")
            self.log_message("已斷開連接")
            
        except Exception as e:
            self.log_message(f"斷開連接錯誤: {str(e)}")
            
    def send_rs232_command(self, command):
        """發送RS232指令"""
        if not self.connected or not self.serial_connection:
            self.log_message("錯誤: 串口未連接")
            return False
            
        try:
            # 添加換行符 (根據手冊要求)
            full_command = command + "\r\n"
            self.serial_connection.write(full_command.encode('ascii'))
            
            # 讀取回應
            time.sleep(0.1)
            if self.serial_connection.in_waiting > 0:
                response = self.serial_connection.read(self.serial_connection.in_waiting).decode('ascii', errors='ignore')
                self.log_message(f"發送: {command} | 回應: {response.strip()}")
            else:
                self.log_message(f"發送: {command}")
                
            return True
            
        except Exception as e:
            self.log_message(f"發送指令失敗: {str(e)}")
            return False
            
    def toggle_led(self, channel):
        """切換LED開關"""
        current_state = self.led_states[channel]
        new_state = not current_state
        
        if new_state:
            # 開啟 - 設置為當前滑桿亮度，如果為0則設為255
            brightness = self.led_brightness[channel] if self.led_brightness[channel] > 0 else 255
            command = f"CH{channel+1}:{brightness}"
        else:
            # 關閉 - 設置亮度為0
            brightness = 0
            command = f"CH{channel+1}:0"
            
        if self.send_rs232_command(command):
            self.led_states[channel] = new_state
            self.led_brightness[channel] = brightness
            self.update_led_ui(channel)
            
    def set_brightness(self, channel, brightness):
        """設置LED亮度"""
        command = f"CH{channel+1}:{brightness}"
        
        if self.send_rs232_command(command):
            self.led_brightness[channel] = brightness
            self.led_states[channel] = brightness > 0
            self.update_led_ui(channel)
            
    def update_led_ui(self, channel):
        """更新LED UI狀態"""
        control = self.led_controls[channel]
        state = self.led_states[channel]
        brightness = self.led_brightness[channel]
        
        # 更新按鈕
        if state:
            control.toggle_btn.configure(
                text="開啟", 
                fg_color=["#3B8ED0", "#1F6AA5"],
                text_color="white"
            )
        else:
            control.toggle_btn.configure(
                text="關閉", 
                fg_color="gray",
                text_color="white"
            )
            
        # 更新亮度標籤和滑桿
        control.brightness_label.configure(text=f"亮度: {brightness}")
        control.brightness_slider.set(brightness)
        
    def all_on(self):
        """全部開啟"""
        for i in range(4):
            brightness = 255  # 設為最大亮度
            command = f"CH{i+1}:{brightness}"
            if self.send_rs232_command(command):
                self.led_states[i] = True
                self.led_brightness[i] = brightness
                self.update_led_ui(i)
            time.sleep(0.05)  # 小延遲避免指令衝突
            
    def all_off(self):
        """全部關閉"""
        for i in range(4):
            command = f"CH{i+1}:0"
            if self.send_rs232_command(command):
                self.led_states[i] = False
                self.led_brightness[i] = 0
                self.update_led_ui(i)
            time.sleep(0.05)  # 小延遲避免指令衝突
            
    def reset_device(self):
        """重置設備"""
        if self.send_rs232_command("RESET"):
            self.log_message("已發送重置指令")
            # 重置本地狀態
            for i in range(4):
                self.led_states[i] = False
                self.led_brightness[i] = 0
                self.update_led_ui(i)
                
    def log_message(self, message):
        """添加日誌消息"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert("end", log_entry)
        self.log_text.see("end")
        
        # 限制日誌行數
        lines = self.log_text.get("1.0", "end").split('\n')
        if len(lines) > 100:
            self.log_text.delete("1.0", "2.0")
            
    def run(self):
        """運行應用程序"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()
        
    def on_closing(self):
        """關閉應用程序"""
        if self.connected:
            self.disconnect()
        self.root.destroy()

def main():
    """主函數"""
    print("LED控制器測試工具啟動中...")
    print("支援指令格式:")
    print("  CH1:255  - 設定L1亮度為255")
    print("  CH2:0    - 關閉L2")
    print("  RESET    - 重置設備")
    print("支援亮度範圍: 0-511")
    
    app = LEDTestTool()
    app.run()

if __name__ == "__main__":
    main()