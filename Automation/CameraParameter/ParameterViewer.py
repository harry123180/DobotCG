import customtkinter as ctk
import numpy as np
import cv2
import tkinter as tk
from tkinter import filedialog, messagebox
import os

class CoordinateConverterTool:
    def __init__(self):
        # 設置 CustomTkinter 主題
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        
        # 初始化主窗口
        self.root = ctk.CTk()
        self.root.title("座標轉換工具")
        self.root.geometry("700x800")
        
        # 設置中文字體
        try:
            import platform
            system = platform.system()
            if system == "Darwin":  # macOS
                self.font_family = "PingFang SC"
            elif system == "Windows":
                self.font_family = "Microsoft YaHei"
            else:  # Linux
                self.font_family = "DejaVu Sans"
        except:
            self.font_family = "Arial"
        
        # 初始化變數
        self.K = None  # 內參矩陣
        self.D = None  # 畸變係數
        self.rvec = None  # 旋轉向量
        self.tvec = None  # 平移向量
        self.R = None  # 旋轉矩陣
        
        # 創建界面
        self.create_widgets()
        
    def create_widgets(self):
        """創建主界面"""
        # 主框架
        main_frame = ctk.CTkScrollableFrame(self.root)
        main_frame.pack(fill="both", expand=True, padx=20, pady=20)
        
        # 標題
        title = ctk.CTkLabel(main_frame, text="相機座標轉換工具", 
                           font=ctk.CTkFont(family=self.font_family, size=24, weight="bold"))
        title.pack(pady=(20, 30))
        
        # 1. 檔案導入區域
        self.create_file_import_section(main_frame)
        
        # 2. 標定狀態顯示
        self.create_status_section(main_frame)
        
        # 3. 座標輸入區域
        self.create_coordinate_input_section(main_frame)
        
        # 4. 結果顯示區域
        self.create_result_section(main_frame)
        
        # 5. 批量轉換區域
        self.create_batch_section(main_frame)
        
    def create_file_import_section(self, parent):
        """創建檔案導入區域"""
        # 檔案導入框架
        file_frame = ctk.CTkFrame(parent)
        file_frame.pack(fill="x", pady=(0, 20))
        
        ctk.CTkLabel(file_frame, text="標定檔案導入:", 
                    font=ctk.CTkFont(family=self.font_family, size=16, weight="bold")).pack(anchor="w", padx=20, pady=(15, 10))
        
        # 內參導入
        intrinsic_frame = ctk.CTkFrame(file_frame)
        intrinsic_frame.pack(fill="x", padx=20, pady=5)
        
        ctk.CTkLabel(intrinsic_frame, text="內參矩陣:", width=100,
                    font=ctk.CTkFont(family=self.font_family)).pack(side="left", padx=10, pady=10)
        ctk.CTkButton(intrinsic_frame, text="導入 camera_matrix.npy", 
                     font=ctk.CTkFont(family=self.font_family),
                     command=self.import_camera_matrix).pack(side="left", padx=5, pady=10)
        self.intrinsic_status = ctk.CTkLabel(intrinsic_frame, text="未載入", 
                                           font=ctk.CTkFont(family=self.font_family))
        self.intrinsic_status.pack(side="left", padx=10, pady=10)
        
        # 畸變係數導入
        distortion_frame = ctk.CTkFrame(file_frame)
        distortion_frame.pack(fill="x", padx=20, pady=5)
        
        ctk.CTkLabel(distortion_frame, text="畸變係數:", width=100,
                    font=ctk.CTkFont(family=self.font_family)).pack(side="left", padx=10, pady=10)
        ctk.CTkButton(distortion_frame, text="導入 dist_coeffs.npy", 
                     font=ctk.CTkFont(family=self.font_family),
                     command=self.import_dist_coeffs).pack(side="left", padx=5, pady=10)
        self.distortion_status = ctk.CTkLabel(distortion_frame, text="未載入", 
                                            font=ctk.CTkFont(family=self.font_family))
        self.distortion_status.pack(side="left", padx=10, pady=10)
        
        # 外參導入
        extrinsic_frame = ctk.CTkFrame(file_frame)
        extrinsic_frame.pack(fill="x", padx=20, pady=(5, 15))
        
        ctk.CTkLabel(extrinsic_frame, text="外參數據:", width=100,
                    font=ctk.CTkFont(family=self.font_family)).pack(side="left", padx=10, pady=10)
        ctk.CTkButton(extrinsic_frame, text="導入 extrinsic.npy", 
                     font=ctk.CTkFont(family=self.font_family),
                     command=self.import_extrinsic).pack(side="left", padx=5, pady=10)
        self.extrinsic_status = ctk.CTkLabel(extrinsic_frame, text="未載入", 
                                           font=ctk.CTkFont(family=self.font_family))
        self.extrinsic_status.pack(side="left", padx=10, pady=10)
        
    def create_status_section(self, parent):
        """創建標定狀態顯示區域"""
        status_frame = ctk.CTkFrame(parent)
        status_frame.pack(fill="x", pady=(0, 20))
        
        ctk.CTkLabel(status_frame, text="轉換器狀態:", 
                    font=ctk.CTkFont(family=self.font_family, size=16, weight="bold")).pack(anchor="w", padx=20, pady=(15, 10))
        
        self.status_label = ctk.CTkLabel(status_frame, text="請導入所有標定檔案", 
                                       font=ctk.CTkFont(family=self.font_family, size=14),
                                       text_color="orange")
        self.status_label.pack(anchor="w", padx=20, pady=(0, 15))
        
    def create_coordinate_input_section(self, parent):
        """創建座標輸入區域"""
        input_frame = ctk.CTkFrame(parent)
        input_frame.pack(fill="x", pady=(0, 20))
        
        ctk.CTkLabel(input_frame, text="圖像座標輸入:", 
                    font=ctk.CTkFont(family=self.font_family, size=16, weight="bold")).pack(anchor="w", padx=20, pady=(15, 10))
        
        # 座標輸入框
        coord_input_frame = ctk.CTkFrame(input_frame)
        coord_input_frame.pack(fill="x", padx=20, pady=5)
        
        ctk.CTkLabel(coord_input_frame, text="X (像素):", width=80,
                    font=ctk.CTkFont(family=self.font_family)).pack(side="left", padx=5, pady=10)
        self.x_entry = ctk.CTkEntry(coord_input_frame, width=120, placeholder_text="如: 320.5",
                                   font=ctk.CTkFont(family=self.font_family))
        self.x_entry.pack(side="left", padx=5, pady=10)
        
        ctk.CTkLabel(coord_input_frame, text="Y (像素):", width=80,
                    font=ctk.CTkFont(family=self.font_family)).pack(side="left", padx=5, pady=10)
        self.y_entry = ctk.CTkEntry(coord_input_frame, width=120, placeholder_text="如: 240.8",
                                   font=ctk.CTkFont(family=self.font_family))
        self.y_entry.pack(side="left", padx=5, pady=10)
        
        # 轉換按鈕
        convert_btn_frame = ctk.CTkFrame(input_frame)
        convert_btn_frame.pack(fill="x", padx=20, pady=(10, 15))
        
        ctk.CTkButton(convert_btn_frame, text="轉換座標", 
                     font=ctk.CTkFont(family=self.font_family, size=14),
                     command=self.convert_single_point).pack(pady=10)
        
    def create_result_section(self, parent):
        """創建結果顯示區域"""
        result_frame = ctk.CTkFrame(parent)
        result_frame.pack(fill="x", pady=(0, 20))
        
        ctk.CTkLabel(result_frame, text="轉換結果:", 
                    font=ctk.CTkFont(family=self.font_family, size=16, weight="bold")).pack(anchor="w", padx=20, pady=(15, 10))
        
        # 結果顯示
        self.result_text = ctk.CTkTextbox(result_frame, height=150, wrap="word",
                                         font=ctk.CTkFont(family=self.font_family, size=12))
        self.result_text.pack(fill="x", padx=20, pady=(0, 15))
        
    def create_batch_section(self, parent):
        """創建批量轉換區域"""
        batch_frame = ctk.CTkFrame(parent)
        batch_frame.pack(fill="x", pady=(0, 20))
        
        ctk.CTkLabel(batch_frame, text="批量轉換:", 
                    font=ctk.CTkFont(family=self.font_family, size=16, weight="bold")).pack(anchor="w", padx=20, pady=(15, 10))
        
        # 批量輸入說明
        ctk.CTkLabel(batch_frame, text="格式: 每行一個點，用逗號分隔 x,y", 
                    font=ctk.CTkFont(family=self.font_family, size=11)).pack(anchor="w", padx=20, pady=5)
        
        # 批量輸入框
        self.batch_input = ctk.CTkTextbox(batch_frame, height=100, wrap="word",
                                         font=ctk.CTkFont(family=self.font_family))
        self.batch_input.pack(fill="x", padx=20, pady=5)
        self.batch_input.insert("1.0", "320.5,240.8\n640.2,480.1\n160.7,120.3")
        
        # 批量轉換按鈕
        batch_btn_frame = ctk.CTkFrame(batch_frame)
        batch_btn_frame.pack(fill="x", padx=20, pady=(10, 15))
        
        ctk.CTkButton(batch_btn_frame, text="批量轉換", 
                     font=ctk.CTkFont(family=self.font_family),
                     command=self.convert_batch_points).pack(side="left", padx=5, pady=10)
        ctk.CTkButton(batch_btn_frame, text="清空", 
                     font=ctk.CTkFont(family=self.font_family),
                     command=self.clear_batch).pack(side="left", padx=5, pady=10)
        
    def import_camera_matrix(self):
        """導入內參矩陣"""
        file_path = filedialog.askopenfilename(
            title="選擇內參矩陣檔案",
            filetypes=[("NPY files", "*.npy"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                K = np.load(file_path)
                if K.shape == (3, 3):
                    self.K = K
                    self.intrinsic_status.configure(text="✓ 已載入", text_color="green")
                    self.update_converter_status()
                    
                    # 顯示內參信息
                    info = f"內參矩陣載入成功:\n"
                    info += f"fx: {K[0,0]:.2f}, fy: {K[1,1]:.2f}\n"
                    info += f"cx: {K[0,2]:.2f}, cy: {K[1,2]:.2f}"
                    messagebox.showinfo("成功", info)
                else:
                    messagebox.showerror("錯誤", "內參矩陣格式不正確，應為3x3矩陣！")
            except Exception as e:
                messagebox.showerror("錯誤", f"載入失敗: {str(e)}")
                
    def import_dist_coeffs(self):
        """導入畸變係數"""
        file_path = filedialog.askopenfilename(
            title="選擇畸變係數檔案",
            filetypes=[("NPY files", "*.npy"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                D = np.load(file_path)
                # 處理不同的畸變係數格式
                if D.shape == (1, 5):
                    D = D.ravel()
                elif D.shape == (5,):
                    pass
                elif D.shape == (5, 1):
                    D = D.ravel()
                elif len(D) >= 4:  # 至少4個係數
                    D = D[:5] if len(D) >= 5 else np.append(D, [0.0] * (5 - len(D)))
                else:
                    raise ValueError("畸變係數數量不足")
                
                self.D = D
                self.distortion_status.configure(text="✓ 已載入", text_color="green")
                self.update_converter_status()
                
                # 顯示畸變係數信息
                info = f"畸變係數載入成功:\n"
                info += f"k1: {D[0]:.6f}, k2: {D[1]:.6f}\n"
                info += f"p1: {D[2]:.6f}, p2: {D[3]:.6f}"
                if len(D) > 4:
                    info += f"\nk3: {D[4]:.6f}"
                messagebox.showinfo("成功", info)
            except Exception as e:
                messagebox.showerror("錯誤", f"載入失敗: {str(e)}")
                
    def import_extrinsic(self):
        """導入外參數據"""
        file_path = filedialog.askopenfilename(
            title="選擇外參檔案",
            filetypes=[("NPY files", "*.npy"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                data = np.load(file_path, allow_pickle=True)
                
                if isinstance(data, np.ndarray) and data.shape == ():
                    # 字典格式
                    data = data.item()
                    rvec = data['rvec']
                    tvec = data['tvec']
                elif isinstance(data, dict):
                    rvec = data['rvec']
                    tvec = data['tvec']
                else:
                    raise ValueError("外參檔案格式不正確")
                
                # 確保形狀正確
                if rvec.shape == (3,):
                    rvec = rvec.reshape(3, 1)
                if tvec.shape == (3,):
                    tvec = tvec.reshape(3, 1)
                
                self.rvec = rvec
                self.tvec = tvec
                self.R, _ = cv2.Rodrigues(self.rvec)  # 計算旋轉矩陣
                
                self.extrinsic_status.configure(text="✓ 已載入", text_color="green")
                self.update_converter_status()
                
                # 顯示外參信息
                info = f"外參數據載入成功:\n"
                info += f"旋轉向量: [{rvec[0,0]:.4f}, {rvec[1,0]:.4f}, {rvec[2,0]:.4f}]\n"
                info += f"平移向量: [{tvec[0,0]:.2f}, {tvec[1,0]:.2f}, {tvec[2,0]:.2f}]"
                messagebox.showinfo("成功", info)
                
            except Exception as e:
                messagebox.showerror("錯誤", f"載入失敗: {str(e)}")
                
    def update_converter_status(self):
        """更新轉換器狀態"""
        if self.K is not None and self.D is not None and self.rvec is not None and self.tvec is not None:
            self.status_label.configure(text="✓ 轉換器就緒，可以進行座標轉換", text_color="green")
        else:
            missing = []
            if self.K is None:
                missing.append("內參矩陣")
            if self.D is None:
                missing.append("畸變係數")
            if self.rvec is None or self.tvec is None:
                missing.append("外參數據")
            self.status_label.configure(text=f"缺少: {', '.join(missing)}", text_color="orange")
            
    def pixel_to_world(self, pixel_x, pixel_y):
        """像素座標轉世界座標"""
        if not self.is_ready():
            raise ValueError("轉換器未就緒，請先載入所有標定檔案")
        
        try:
            # 步驟1: 去畸變處理
            pixel_coords = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
            undistorted_uv = cv2.undistortPoints(pixel_coords, self.K, self.D, P=self.K).reshape(-1)
            
            # 步驟2: 歸一化座標 (轉換到相機座標系)
            normalized_coords = np.array([
                (undistorted_uv[0] - self.K[0, 2]) / self.K[0, 0],  # (u - cx) / fx
                (undistorted_uv[1] - self.K[1, 2]) / self.K[1, 1],  # (v - cy) / fy
                1.0
            ])
            
            # 步驟3: 計算深度係數 (假設Z=0平面)
            denominator = self.R[2] @ normalized_coords
            if abs(denominator) < 1e-10:
                raise ValueError("無法計算深度，點可能在無限遠處")
                
            s = (0 - self.tvec[2, 0]) / denominator
            
            # 步驟4: 計算相機座標系中的3D點
            camera_point = s * normalized_coords
            
            # 步驟5: 轉換到世界座標系
            world_point = np.linalg.inv(self.R) @ (camera_point - self.tvec.ravel())
            
            return world_point[0], world_point[1]  # 返回X,Y座標
            
        except Exception as e:
            raise ValueError(f"座標轉換失敗: {str(e)}")
            
    def is_ready(self):
        """檢查轉換器是否就緒"""
        return (self.K is not None and self.D is not None and 
                self.rvec is not None and self.tvec is not None and self.R is not None)
                
    def convert_single_point(self):
        """轉換單個點"""
        if not self.is_ready():
            messagebox.showwarning("警告", "請先載入所有標定檔案！")
            return
            
        try:
            # 獲取輸入座標
            pixel_x = float(self.x_entry.get())
            pixel_y = float(self.y_entry.get())
            
            # 執行轉換
            world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
            
            # 顯示結果
            result_text = f"=== 座標轉換結果 ===\n\n"
            result_text += f"圖像座標: ({pixel_x:.2f}, {pixel_y:.2f}) 像素\n"
            result_text += f"世界座標: ({world_x:.2f}, {world_y:.2f}) mm\n\n"
            result_text += f"轉換詳情:\n"
            result_text += f"• 已去畸變處理\n"
            result_text += f"• 假設Z=0平面投影\n"
            result_text += f"• 使用載入的內外參數據"
            
            self.result_text.delete("1.0", "end")
            self.result_text.insert("1.0", result_text)
            
        except ValueError as e:
            if "轉換器未就緒" in str(e):
                messagebox.showwarning("警告", str(e))
            else:
                messagebox.showerror("錯誤", f"請輸入有效的數值！\n詳情: {str(e)}")
        except Exception as e:
            messagebox.showerror("錯誤", f"轉換失敗: {str(e)}")
            
    def convert_batch_points(self):
        """批量轉換點"""
        if not self.is_ready():
            messagebox.showwarning("警告", "請先載入所有標定檔案！")
            return
            
        try:
            # 獲取批量輸入
            text_content = self.batch_input.get("1.0", "end-1c")
            lines = text_content.strip().split('\n')
            
            results = []
            results.append("=== 批量座標轉換結果 ===\n")
            
            for i, line in enumerate(lines, 1):
                if line.strip():
                    try:
                        # 解析座標
                        coords = line.strip().split(',')
                        if len(coords) != 2:
                            results.append(f"第{i}行格式錯誤: {line}")
                            continue
                            
                        pixel_x = float(coords[0])
                        pixel_y = float(coords[1])
                        
                        # 執行轉換
                        world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
                        
                        results.append(f"點{i}: ({pixel_x:.2f}, {pixel_y:.2f}) → ({world_x:.2f}, {world_y:.2f}) mm")
                        
                    except Exception as e:
                        results.append(f"點{i}轉換失敗: {line} - {str(e)}")
            
            # 顯示結果
            self.result_text.delete("1.0", "end")
            self.result_text.insert("1.0", '\n'.join(results))
            
            messagebox.showinfo("完成", f"批量轉換完成！處理了{len(lines)}個點。")
            
        except Exception as e:
            messagebox.showerror("錯誤", f"批量轉換失敗: {str(e)}")
            
    def clear_batch(self):
        """清空批量輸入"""
        self.batch_input.delete("1.0", "end")
        
    def run(self):
        """運行應用"""
        self.root.mainloop()

# 使用範例
if __name__ == "__main__":
    app = CoordinateConverterTool()
    app.run()
        
    def import_camera_matrix(self):
        """導入內參矩陣"""
        file_path = filedialog.askopenfilename(
            title="選擇內參矩陣檔案",
            filetypes=[("NPY files", "*.npy"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                K = np.load(file_path)
                if K.shape == (3, 3):
                    self.K = K
                    self.intrinsic_status.configure(text="✓ 已載入", text_color="green")
                    self.update_converter_status()
                    
                    # 顯示內參信息
                    info = f"內參矩陣載入成功:\\n"
                    info += f"fx: {K[0,0]:.2f}, fy: {K[1,1]:.2f}\\n"
                    info += f"cx: {K[0,2]:.2f}, cy: {K[1,2]:.2f}"
                    messagebox.showinfo("成功", info)
                else:
                    messagebox.showerror("錯誤", "內參矩陣格式不正確，應為3x3矩陣！")
            except Exception as e:
                messagebox.showerror("錯誤", f"載入失敗: {str(e)}")
                
    def import_dist_coeffs(self):
        """導入畸變係數"""
        file_path = filedialog.askopenfilename(
            title="選擇畸變係數檔案",
            filetypes=[("NPY files", "*.npy"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                D = np.load(file_path)
                # 處理不同的畸變係數格式
                if D.shape == (1, 5):
                    D = D.ravel()
                elif D.shape == (5,):
                    pass
                elif D.shape == (5, 1):
                    D = D.ravel()
                elif len(D) >= 4:  # 至少4個係數
                    D = D[:5] if len(D) >= 5 else np.append(D, [0.0] * (5 - len(D)))
                else:
                    raise ValueError("畸變係數數量不足")
                
                self.D = D
                self.distortion_status.configure(text="✓ 已載入", text_color="green")
                self.update_converter_status()
                
                # 顯示畸變係數信息
                info = f"畸變係數載入成功:\\n"
                info += f"k1: {D[0]:.6f}, k2: {D[1]:.6f}\\n"
                info += f"p1: {D[2]:.6f}, p2: {D[3]:.6f}"
                if len(D) > 4:
                    info += f"\\nk3: {D[4]:.6f}"
                messagebox.showinfo("成功", info)
            except Exception as e:
                messagebox.showerror("錯誤", f"載入失敗: {str(e)}")
                
    def import_extrinsic(self):
        """導入外參數據"""
        file_path = filedialog.askopenfilename(
            title="選擇外參檔案",
            filetypes=[("NPY files", "*.npy"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                data = np.load(file_path, allow_pickle=True)
                
                if isinstance(data, np.ndarray) and data.shape == ():
                    # 字典格式
                    data = data.item()
                    rvec = data['rvec']
                    tvec = data['tvec']
                elif isinstance(data, dict):
                    rvec = data['rvec']
                    tvec = data['tvec']
                else:
                    raise ValueError("外參檔案格式不正確")
                
                # 確保形狀正確
                if rvec.shape == (3,):
                    rvec = rvec.reshape(3, 1)
                if tvec.shape == (3,):
                    tvec = tvec.reshape(3, 1)
                
                self.rvec = rvec
                self.tvec = tvec
                self.R, _ = cv2.Rodrigues(self.rvec)  # 計算旋轉矩陣
                
                self.extrinsic_status.configure(text="✓ 已載入", text_color="green")
                self.update_converter_status()
                
                # 顯示外參信息
                info = f"外參數據載入成功:\\n"
                info += f"旋轉向量: [{rvec[0,0]:.4f}, {rvec[1,0]:.4f}, {rvec[2,0]:.4f}]\\n"
                info += f"平移向量: [{tvec[0,0]:.2f}, {tvec[1,0]:.2f}, {tvec[2,0]:.2f}]"
                messagebox.showinfo("成功", info)
                
            except Exception as e:
                messagebox.showerror("錯誤", f"載入失敗: {str(e)}")
                
    def update_converter_status(self):
        """更新轉換器狀態"""
        if self.K is not None and self.D is not None and self.rvec is not None and self.tvec is not None:
            self.status_label.configure(text="✓ 轉換器就緒，可以進行座標轉換", text_color="green")
        else:
            missing = []
            if self.K is None:
                missing.append("內參矩陣")
            if self.D is None:
                missing.append("畸變係數")
            if self.rvec is None or self.tvec is None:
                missing.append("外參數據")
            self.status_label.configure(text=f"缺少: {', '.join(missing)}", text_color="orange")
            
    def pixel_to_world(self, pixel_x, pixel_y):
        """像素座標轉世界座標"""
        if not self.is_ready():
            raise ValueError("轉換器未就緒，請先載入所有標定檔案")
        
        try:
            # 步驟1: 去畸變處理
            pixel_coords = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
            undistorted_uv = cv2.undistortPoints(pixel_coords, self.K, self.D, P=self.K).reshape(-1)
            
            # 步驟2: 歸一化座標 (轉換到相機座標系)
            normalized_coords = np.array([
                (undistorted_uv[0] - self.K[0, 2]) / self.K[0, 0],  # (u - cx) / fx
                (undistorted_uv[1] - self.K[1, 2]) / self.K[1, 1],  # (v - cy) / fy
                1.0
            ])
            
            # 步驟3: 計算深度係數 (假設Z=0平面)
            denominator = self.R[2] @ normalized_coords
            if abs(denominator) < 1e-10:
                raise ValueError("無法計算深度，點可能在無限遠處")
                
            s = (0 - self.tvec[2, 0]) / denominator
            
            # 步驟4: 計算相機座標系中的3D點
            camera_point = s * normalized_coords
            
            # 步驟5: 轉換到世界座標系
            world_point = np.linalg.inv(self.R) @ (camera_point - self.tvec.ravel())
            
            return world_point[0], world_point[1]  # 返回X,Y座標
            
        except Exception as e:
            raise ValueError(f"座標轉換失敗: {str(e)}")
            
    def is_ready(self):
        """檢查轉換器是否就緒"""
        return (self.K is not None and self.D is not None and 
                self.rvec is not None and self.tvec is not None and self.R is not None)
                
    def convert_single_point(self):
        """轉換單個點"""
        if not self.is_ready():
            messagebox.showwarning("警告", "請先載入所有標定檔案！")
            return
            
        try:
            # 獲取輸入座標
            pixel_x = float(self.x_entry.get())
            pixel_y = float(self.y_entry.get())
            
            # 執行轉換
            world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
            
            # 顯示結果
            result_text = f"=== 座標轉換結果 ===\\n\\n"
            result_text += f"圖像座標: ({pixel_x:.2f}, {pixel_y:.2f}) 像素\\n"
            result_text += f"世界座標: ({world_x:.2f}, {world_y:.2f}) mm\\n\\n"
            result_text += f"轉換詳情:\\n"
            result_text += f"• 已去畸變處理\\n"
            result_text += f"• 假設Z=0平面投影\\n"
            result_text += f"• 使用載入的內外參數據"
            
            self.result_text.delete("1.0", "end")
            self.result_text.insert("1.0", result_text)
            
        except ValueError as e:
            if "轉換器未就緒" in str(e):
                messagebox.showwarning("警告", str(e))
            else:
                messagebox.showerror("錯誤", f"請輸入有效的數值！\\n詳情: {str(e)}")
        except Exception as e:
            messagebox.showerror("錯誤", f"轉換失敗: {str(e)}")
            
    def convert_batch_points(self):
        """批量轉換點"""
        if not self.is_ready():
            messagebox.showwarning("警告", "請先載入所有標定檔案！")
            return
            
        try:
            # 獲取批量輸入
            text_content = self.batch_input.get("1.0", "end-1c")
            lines = text_content.strip().split('\\n')
            
            results = []
            results.append("=== 批量座標轉換結果 ===\\n")
            
            for i, line in enumerate(lines, 1):
                if line.strip():
                    try:
                        # 解析座標
                        coords = line.strip().split(',')
                        if len(coords) != 2:
                            results.append(f"第{i}行格式錯誤: {line}")
                            continue
                            
                        pixel_x = float(coords[0])
                        pixel_y = float(coords[1])
                        
                        # 執行轉換
                        world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
                        
                        results.append(f"點{i}: ({pixel_x:.2f}, {pixel_y:.2f}) → ({world_x:.2f}, {world_y:.2f}) mm")
                        
                    except Exception as e:
                        results.append(f"點{i}轉換失敗: {line} - {str(e)}")
            
            # 顯示結果
            self.result_text.delete("1.0", "end")
            self.result_text.insert("1.0", '\\n'.join(results))
            
            messagebox.showinfo("完成", f"批量轉換完成！處理了{len(lines)}個點。")
            
        except Exception as e:
            messagebox.showerror("錯誤", f"批量轉換失敗: {str(e)}")
            
    def clear_batch(self):
        """清空批量輸入"""
        self.batch_input.delete("1.0", "end")
        
    def run(self):
        """運行應用"""
        self.root.mainloop()

# 使用範例
if __name__ == "__main__":
    app = CoordinateConverterTool()
    app.run()