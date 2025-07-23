# -*- coding: utf-8 -*-
"""
LogViewer.py - 進程監控日誌分析和可視化工具
分析ProcessMonitor生成的JSON報告，生成記憶體、執行緒、CPU使用率趨勢圖
支援模組切換、圖表縮放、互動式查看
"""

import json
import os
import glob
from datetime import datetime
import pandas as pd
import numpy as np

# 設置matplotlib後端和中文字體
import matplotlib
matplotlib.use('TkAgg')  # 強制使用TkAgg後端
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.widgets import CheckButtons, Button

from typing import Dict, List, Optional
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading

# 設置matplotlib後端和中文字體
import matplotlib
matplotlib.use('TkAgg')  # 強制使用TkAgg後端
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.widgets import CheckButtons, Button

# 設置matplotlib中文字體
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

class ProcessLogAnalyzer:
    """進程監控日誌分析器"""
    
    def __init__(self, logs_dir: str = "logs"):
        self.logs_dir = logs_dir
        self.data = {}
        self.module_colors = {}
        # 使用matplotlib內建顏色調色盤代替seaborn
        self.color_palette = plt.cm.tab10(np.linspace(0, 1, 10))  # 10種顏色
        # 擴充更多顏色
        additional_colors = plt.cm.Set3(np.linspace(0, 1, 12))  # 12種額外顏色
        self.color_palette = np.vstack([self.color_palette, additional_colors])
        
    def load_json_reports(self) -> bool:
        """載入所有JSON報告"""
        try:
            # 查找所有JSON報告檔案
            json_files = glob.glob(os.path.join(self.logs_dir, "monitor_report_*.json"))
            
            if not json_files:
                print(f"❌ 在 {self.logs_dir} 目錄中未找到任何監控報告")
                return False
            
            print(f"📁 找到 {len(json_files)} 個監控報告檔案")
            
            all_data = []
            
            for json_file in sorted(json_files):
                try:
                    with open(json_file, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                        data['file_path'] = json_file
                        all_data.append(data)
                except Exception as e:
                    print(f"⚠️ 載入檔案失敗 {json_file}: {e}")
                    continue
            
            if not all_data:
                print("❌ 沒有成功載入任何監控報告")
                return False
            
            # 轉換為DataFrame格式
            self._process_data(all_data)
            print(f"✅ 成功載入並處理 {len(all_data)} 個監控報告")
            return True
            
        except Exception as e:
            print(f"❌ 載入JSON報告失敗: {e}")
            return False
    
    def _process_data(self, raw_data: List[Dict]):
        """處理原始數據轉換為分析格式"""
        processed_data = []
        
        for report in raw_data:
            timestamp_str = report.get('timestamp', '')
            try:
                timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S')
            except:
                continue
            
            # 系統整體資訊
            system_info = report.get('system_info', {})
            
            # 處理各模組數據
            modules = report.get('modules', {})
            
            for module_name, module_info in modules.items():
                if module_info.get('process_count', 0) > 0:
                    for process in module_info.get('processes', []):
                        record = {
                            'timestamp': timestamp,
                            'module_name': module_name,
                            'pid': process.get('pid'),
                            'memory_mb': process.get('memory_rss_mb', 0),
                            'memory_percent': process.get('memory_percent', 0),
                            'cpu_percent': process.get('cpu_percent', 0),
                            'thread_count': process.get('thread_count', 0),
                            'running_hours': process.get('running_time_hours', 0),
                            'open_files': process.get('open_files_count', 0),
                            'connections': process.get('connections_count', 0),
                            'status': process.get('status', 'unknown'),
                            'system_cpu': system_info.get('cpu_percent', 0),
                            'system_memory': system_info.get('memory_percent', 0),
                        }
                        processed_data.append(record)
        
        # 轉換為DataFrame
        if processed_data:
            self.df = pd.DataFrame(processed_data)
            self.df = self.df.sort_values(['module_name', 'pid', 'timestamp'])
            
            # 分配顏色
            unique_modules = self.df['module_name'].unique()
            for i, module in enumerate(unique_modules):
                self.module_colors[module] = self.color_palette[i % len(self.color_palette)]
            
            print(f"📊 處理完成: {len(processed_data)} 條記錄，{len(unique_modules)} 個模組")
        else:
            self.df = pd.DataFrame()
    
    def get_module_summary(self) -> Dict:
        """獲取模組摘要統計"""
        if self.df.empty:
            return {}
        
        summary = {}
        
        for module in self.df['module_name'].unique():
            module_data = self.df[self.df['module_name'] == module]
            
            # 計算統計資訊
            latest_data = module_data.groupby('pid').last()
            
            summary[module] = {
                'process_count': len(latest_data),
                'total_memory_mb': latest_data['memory_mb'].sum(),
                'avg_cpu_percent': latest_data['cpu_percent'].mean(),
                'total_threads': latest_data['thread_count'].sum(),
                'max_memory_mb': module_data['memory_mb'].max(),
                'memory_growth': self._calculate_growth_rate(module_data, 'memory_mb'),
                'thread_growth': self._calculate_growth_rate(module_data, 'thread_count'),
                'time_span_hours': (module_data['timestamp'].max() - module_data['timestamp'].min()).total_seconds() / 3600,
                'record_count': len(module_data)
            }
        
        return summary
    
    def _calculate_growth_rate(self, data: pd.DataFrame, column: str) -> float:
        """計算增長率 (每小時)"""
        if len(data) < 2:
            return 0.0
        
        try:
            # 線性回歸計算增長率
            x = np.arange(len(data))
            y = data[column].values
            
            # 移除NaN值
            valid_mask = ~np.isnan(y)
            if valid_mask.sum() < 2:
                return 0.0
            
            x_valid = x[valid_mask]
            y_valid = y[valid_mask]
            
            slope = np.polyfit(x_valid, y_valid, 1)[0]
            
            # 轉換為每小時增長率
            time_span = (data['timestamp'].max() - data['timestamp'].min()).total_seconds() / 3600
            if time_span > 0:
                hourly_growth = slope * (len(data) / time_span)
            else:
                hourly_growth = 0.0
            
            return hourly_growth
            
        except:
            return 0.0


class InteractiveLogViewer:
    """互動式日誌查看器"""
    
    def __init__(self, analyzer: ProcessLogAnalyzer):
        self.analyzer = analyzer
        self.fig = None
        self.axes = None
        self.current_modules = set()
        self.check_buttons = None
        
    def create_dashboard(self):
        """創建互動式儀表板"""
        if self.analyzer.df.empty:
            print("❌ 沒有數據可以顯示")
            return
        
        # 創建圖表
        self.fig, self.axes = plt.subplots(2, 2, figsize=(16, 12))
        self.fig.suptitle('🖥️ 進程監控分析儀表板', fontsize=16, fontweight='bold')
        
        # 調整佈局
        plt.subplots_adjust(left=0.15, right=0.85, top=0.93, bottom=0.1, hspace=0.3, wspace=0.3)
        
        # 初始化所有模組顯示
        self.current_modules = set(self.analyzer.df['module_name'].unique())
        
        # 繪製初始圖表
        self._plot_memory_trend()
        self._plot_cpu_trend()
        self._plot_thread_trend()
        self._plot_system_overview()
        
        # 創建模組選擇器
        self._create_module_selector()
        
        # 創建控制按鈕
        self._create_control_buttons()
        
        plt.show()
    
    def _plot_memory_trend(self):
        """繪製記憶體趨勢圖"""
        ax = self.axes[0, 0]
        ax.clear()
        ax.set_title('📊 記憶體使用趨勢', fontweight='bold')
        
        for module in self.current_modules:
            module_data = self.analyzer.df[self.analyzer.df['module_name'] == module]
            if not module_data.empty:
                # 按PID分組繪製
                for pid in module_data['pid'].unique():
                    pid_data = module_data[module_data['pid'] == pid]
                    color = self.analyzer.module_colors[module]
                    
                    ax.plot(pid_data['timestamp'], pid_data['memory_mb'], 
                           color=color, alpha=0.7, linewidth=2,
                           label=f'{module} (PID {pid})' if len(module_data['pid'].unique()) > 1 else module)
        
        ax.set_xlabel('時間')
        ax.set_ylabel('記憶體使用量 (MB)')
        ax.grid(True, alpha=0.3)
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # 格式化時間軸
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=30))
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45)
    
    def _plot_cpu_trend(self):
        """繪製CPU使用率趨勢圖"""
        ax = self.axes[0, 1]
        ax.clear()
        ax.set_title('💻 CPU使用率趨勢', fontweight='bold')
        
        for module in self.current_modules:
            module_data = self.analyzer.df[self.analyzer.df['module_name'] == module]
            if not module_data.empty:
                # 按時間聚合CPU使用率
                cpu_trend = module_data.groupby('timestamp')['cpu_percent'].sum().reset_index()
                color = self.analyzer.module_colors[module]
                
                ax.plot(cpu_trend['timestamp'], cpu_trend['cpu_percent'], 
                       color=color, linewidth=2, label=module, marker='o', markersize=3)
        
        ax.set_xlabel('時間')
        ax.set_ylabel('CPU使用率 (%)')
        ax.grid(True, alpha=0.3)
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # 格式化時間軸
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=30))
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45)
    
    def _plot_thread_trend(self):
        """繪製執行緒數量趨勢圖"""
        ax = self.axes[1, 0]
        ax.clear()
        ax.set_title('🧵 執行緒數量趨勢', fontweight='bold')
        
        for module in self.current_modules:
            module_data = self.analyzer.df[self.analyzer.df['module_name'] == module]
            if not module_data.empty:
                # 按時間聚合執行緒數量
                thread_trend = module_data.groupby('timestamp')['thread_count'].sum().reset_index()
                color = self.analyzer.module_colors[module]
                
                ax.plot(thread_trend['timestamp'], thread_trend['thread_count'], 
                       color=color, linewidth=2, label=module, marker='s', markersize=3)
        
        ax.set_xlabel('時間')
        ax.set_ylabel('執行緒數量')
        ax.grid(True, alpha=0.3)
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # 格式化時間軸
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=30))
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45)
    
    def _plot_system_overview(self):
        """繪製系統整體概覽"""
        ax = self.axes[1, 1]
        ax.clear()
        ax.set_title('🖥️ 系統整體概覽', fontweight='bold')
        
        # 系統CPU和記憶體使用率
        system_data = self.analyzer.df.groupby('timestamp').first().reset_index()
        
        ax2 = ax.twinx()
        
        line1 = ax.plot(system_data['timestamp'], system_data['system_cpu'], 
                       color='red', linewidth=2, label='系統CPU (%)', alpha=0.8)
        line2 = ax2.plot(system_data['timestamp'], system_data['system_memory'], 
                        color='blue', linewidth=2, label='系統記憶體 (%)', alpha=0.8)
        
        ax.set_xlabel('時間')
        ax.set_ylabel('CPU使用率 (%)', color='red')
        ax2.set_ylabel('記憶體使用率 (%)', color='blue')
        
        ax.grid(True, alpha=0.3)
        
        # 合併圖例
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax.legend(lines, labels, loc='upper left')
        
        # 格式化時間軸
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=30))
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45)
    
    def _create_module_selector(self):
        """創建模組選擇器"""
        # 在右側創建CheckButtons
        ax_check = plt.axes([0.86, 0.4, 0.12, 0.4])
        
        modules = list(self.analyzer.df['module_name'].unique())
        module_status = [True] * len(modules)  # 初始全部選中
        
        self.check_buttons = CheckButtons(ax_check, modules, module_status)
        
        # 設置顏色 - 兼容不同matplotlib版本
        try:
            # 新版本matplotlib
            if hasattr(self.check_buttons, 'rectangles'):
                for i, (module, rect) in enumerate(zip(modules, self.check_buttons.rectangles)):
                    rect.set_facecolor(self.analyzer.module_colors[module])
                    rect.set_alpha(0.3)
            # 舊版本matplotlib
            elif hasattr(self.check_buttons, 'rectangles'):
                for i, (module, rect) in enumerate(zip(modules, self.check_buttons.rectangles)):
                    rect.set_facecolor(self.analyzer.module_colors[module])
                    rect.set_alpha(0.3)
            else:
                # 如果都沒有，嘗試其他屬性
                print("⚠️ CheckButtons顏色設置跳過 (版本兼容性問題)")
        except Exception as e:
            print(f"⚠️ CheckButtons顏色設置失敗: {e}")
        
        # 設置回調函數
        self.check_buttons.on_clicked(self._on_module_toggle)
    
    def _create_control_buttons(self):
        """創建控制按鈕"""
        # 重新整理按鈕
        ax_refresh = plt.axes([0.86, 0.85, 0.12, 0.04])
        self.btn_refresh = Button(ax_refresh, '🔄 重新整理')
        self.btn_refresh.on_clicked(self._on_refresh)
        
        # 全選按鈕
        ax_select_all = plt.axes([0.86, 0.35, 0.05, 0.04])
        self.btn_select_all = Button(ax_select_all, '全選')
        self.btn_select_all.on_clicked(self._on_select_all)
        
        # 全不選按鈕
        ax_select_none = plt.axes([0.93, 0.35, 0.05, 0.04])
        self.btn_select_none = Button(ax_select_none, '全不選')
        self.btn_select_none.on_clicked(self._on_select_none)
        
        # 匯出數據按鈕
        ax_export = plt.axes([0.86, 0.30, 0.12, 0.04])
        self.btn_export = Button(ax_export, '📊 匯出分析')
        self.btn_export.on_clicked(self._on_export)
    
    def _on_module_toggle(self, label):
        """模組切換回調"""
        if label in self.current_modules:
            self.current_modules.remove(label)
        else:
            self.current_modules.add(label)
        
        self._refresh_plots()
    
    def _on_refresh(self, event):
        """重新整理按鈕回調"""
        # 重新載入數據
        self.analyzer.load_json_reports()
        self._refresh_plots()
    
    def _on_select_all(self, event):
        """全選按鈕回調"""
        self.current_modules = set(self.analyzer.df['module_name'].unique())
        
        # 更新CheckButtons狀態 - 兼容不同版本
        try:
            modules_list = list(self.analyzer.df['module_name'].unique())
            for i, module in enumerate(modules_list):
                if i < len(self.check_buttons.labels):
                    if not self.check_buttons.get_status()[i]:
                        self.check_buttons.set_active(i)
        except Exception as e:
            print(f"⚠️ 全選操作失敗: {e}")
        
        self._refresh_plots()
    
    def _on_select_none(self, event):
        """全不選按鈕回調"""
        self.current_modules.clear()
        
        # 更新CheckButtons狀態 - 兼容不同版本
        try:
            modules_list = list(self.analyzer.df['module_name'].unique())
            for i in range(min(len(modules_list), len(self.check_buttons.labels))):
                if self.check_buttons.get_status()[i]:
                    self.check_buttons.set_active(i)
        except Exception as e:
            print(f"⚠️ 全不選操作失敗: {e}")
        
        self._refresh_plots()
    
    def _on_export(self, event):
        """匯出分析按鈕回調"""
        self._export_analysis_report()
    
    def _refresh_plots(self):
        """重新整理所有圖表"""
        self._plot_memory_trend()
        self._plot_cpu_trend()
        self._plot_thread_trend()
        self._plot_system_overview()
        self.fig.canvas.draw()
    
    def _export_analysis_report(self):
        """匯出分析報告"""
        try:
            summary = self.analyzer.get_module_summary()
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            report_file = f"analysis_report_{timestamp}.txt"
            
            with open(report_file, 'w', encoding='utf-8') as f:
                f.write("📊 進程監控分析報告\n")
                f.write("=" * 50 + "\n\n")
                f.write(f"生成時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                
                for module, stats in summary.items():
                    f.write(f"🔧 {module}\n")
                    f.write(f"   進程數量: {stats['process_count']}\n")
                    f.write(f"   總記憶體: {stats['total_memory_mb']:.1f} MB\n")
                    f.write(f"   平均CPU: {stats['avg_cpu_percent']:.1f}%\n")
                    f.write(f"   總執行緒: {stats['total_threads']}\n")
                    f.write(f"   記憶體峰值: {stats['max_memory_mb']:.1f} MB\n")
                    f.write(f"   記憶體增長率: {stats['memory_growth']:.2f} MB/h\n")
                    f.write(f"   執行緒增長率: {stats['thread_growth']:.2f} /h\n")
                    f.write(f"   監控時長: {stats['time_span_hours']:.1f} 小時\n")
                    f.write(f"   記錄數量: {stats['record_count']}\n\n")
            
            print(f"✅ 分析報告已匯出: {report_file}")
            
        except Exception as e:
            print(f"❌ 匯出分析報告失敗: {e}")


class LogViewerGUI:
    """圖形界面日誌查看器"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🖥️ 進程監控日誌查看器")
        self.root.geometry("800x600")
        
        self.analyzer = None
        self.viewer = None
        
        self.setup_gui()
    
    def setup_gui(self):
        """設置GUI界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 標題
        title_label = ttk.Label(main_frame, text="🖥️ 進程監控日誌查看器", font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # 日誌目錄選擇
        ttk.Label(main_frame, text="日誌目錄:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.logs_dir_var = tk.StringVar(value="logs")
        logs_dir_entry = ttk.Entry(main_frame, textvariable=self.logs_dir_var, width=50)
        logs_dir_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=5, padx=5)
        
        browse_btn = ttk.Button(main_frame, text="瀏覽", command=self.browse_logs_dir)
        browse_btn.grid(row=1, column=2, pady=5)
        
        # 載入按鈕
        load_btn = ttk.Button(main_frame, text="📁 載入監控日誌", command=self.load_logs)
        load_btn.grid(row=2, column=0, columnspan=3, pady=20)
        
        # 狀態顯示
        self.status_text = tk.Text(main_frame, height=20, width=80)
        self.status_text.grid(row=3, column=0, columnspan=3, pady=10)
        
        # 滾動條
        scrollbar = ttk.Scrollbar(main_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        scrollbar.grid(row=3, column=3, sticky=(tk.N, tk.S))
        self.status_text.configure(yscrollcommand=scrollbar.set)
        
        # 操作按鈕框架
        btn_frame = ttk.Frame(main_frame)
        btn_frame.grid(row=4, column=0, columnspan=3, pady=20)
        
        # 分析按鈕
        analyze_btn = ttk.Button(btn_frame, text="📊 開啟互動式分析", 
                               command=self.open_interactive_viewer, state=tk.DISABLED)
        analyze_btn.grid(row=0, column=0, padx=10)
        self.analyze_btn = analyze_btn
        
        # 摘要按鈕
        summary_btn = ttk.Button(btn_frame, text="📋 生成摘要報告", 
                               command=self.generate_summary, state=tk.DISABLED)
        summary_btn.grid(row=0, column=1, padx=10)
        self.summary_btn = summary_btn
        
        # 配置列權重
        main_frame.columnconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
    
    def browse_logs_dir(self):
        """瀏覽日誌目錄"""
        directory = filedialog.askdirectory(title="選擇日誌目錄")
        if directory:
            self.logs_dir_var.set(directory)
    
    def log_message(self, message):
        """顯示日誌訊息"""
        self.status_text.insert(tk.END, f"{datetime.now().strftime('%H:%M:%S')} {message}\n")
        self.status_text.see(tk.END)
        self.root.update()
    
    def load_logs(self):
        """載入監控日誌"""
        logs_dir = self.logs_dir_var.get()
        
        if not os.path.exists(logs_dir):
            messagebox.showerror("錯誤", f"目錄不存在: {logs_dir}")
            return
        
        self.log_message("🔄 開始載入監控日誌...")
        
        # 在後台執行緒中載入數據
        def load_thread():
            try:
                self.analyzer = ProcessLogAnalyzer(logs_dir)
                success = self.analyzer.load_json_reports()
                
                if success:
                    self.log_message("✅ 日誌載入成功!")
                    
                    # 顯示摘要
                    summary = self.analyzer.get_module_summary()
                    self.log_message(f"📊 找到 {len(summary)} 個模組:")
                    
                    for module, stats in summary.items():
                        self.log_message(f"   • {module}: {stats['total_memory_mb']:.1f}MB, "
                                       f"{stats['total_threads']}執行緒, "
                                       f"{stats['time_span_hours']:.1f}h監控")
                    
                    # 啟用按鈕
                    self.analyze_btn.config(state=tk.NORMAL)
                    self.summary_btn.config(state=tk.NORMAL)
                    
                else:
                    self.log_message("❌ 日誌載入失敗")
                    
            except Exception as e:
                self.log_message(f"❌ 載入異常: {e}")
        
        threading.Thread(target=load_thread, daemon=True).start()
    
    def open_interactive_viewer(self):
        """開啟互動式查看器"""
        if self.analyzer and not self.analyzer.df.empty:
            self.log_message("🚀 啟動互動式分析界面...")
            
            try:
                # 直接在主執行緒中啟動matplotlib界面
                self.viewer = InteractiveLogViewer(self.analyzer)
                self.viewer.create_dashboard()
            except Exception as e:
                self.log_message(f"❌ 互動式查看器啟動失敗: {e}")
        else:
            messagebox.showwarning("警告", "請先載入監控日誌")
    
    def generate_summary(self):
        """生成摘要報告"""
        if self.analyzer and not self.analyzer.df.empty:
            try:
                summary = self.analyzer.get_module_summary()
                
                self.log_message("📋 生成詳細摘要報告:")
                self.log_message("=" * 50)
                
                for module, stats in summary.items():
                    self.log_message(f"\n🔧 {module}")
                    self.log_message(f"   進程數量: {stats['process_count']}")
                    self.log_message(f"   總記憶體: {stats['total_memory_mb']:.1f} MB")
                    self.log_message(f"   平均CPU: {stats['avg_cpu_percent']:.1f}%")
                    self.log_message(f"   總執行緒: {stats['total_threads']}")
                    self.log_message(f"   記憶體峰值: {stats['max_memory_mb']:.1f} MB")
                    
                    if stats['memory_growth'] > 5:
                        self.log_message(f"   ⚠️ 記憶體增長率: {stats['memory_growth']:.2f} MB/h (疑似洩漏)")
                    else:
                        self.log_message(f"   ✅ 記憶體增長率: {stats['memory_growth']:.2f} MB/h (正常)")
                    
                    self.log_message(f"   執行緒增長率: {stats['thread_growth']:.2f} /h")
                    self.log_message(f"   監控時長: {stats['time_span_hours']:.1f} 小時")
                    self.log_message(f"   記錄數量: {stats['record_count']}")
                
                self.log_message("\n✅ 摘要報告生成完成")
                
            except Exception as e:
                self.log_message(f"❌ 生成摘要報告失敗: {e}")
        else:
            messagebox.showwarning("警告", "請先載入監控日誌")
    
    def run(self):
        """運行GUI"""
        self.root.mainloop()


def main():
    """主函數"""
    import argparse
    
    parser = argparse.ArgumentParser(description='進程監控日誌分析和可視化工具')
    parser.add_argument('--logs-dir', default='logs', help='日誌目錄路徑 (預設: logs)')
    parser.add_argument('--gui', action='store_true', help='啟動圖形界面')
    parser.add_argument('--interactive', action='store_true', help='直接啟動互動式分析')
    
    args = parser.parse_args()
    
    if args.gui:
        # 啟動GUI
        app = LogViewerGUI()
        app.run()
    else:
        # 命令行模式
        print("🚀 進程監控日誌分析工具")
        print(f"📁 日誌目錄: {args.logs_dir}")
        
        analyzer = ProcessLogAnalyzer(args.logs_dir)
        
        if not analyzer.load_json_reports():
            print("❌ 無法載入監控日誌")
            return
        
        if args.interactive:
            # 互動式分析
            viewer = InteractiveLogViewer(analyzer)
            viewer.create_dashboard()
        else:
            # 生成摘要報告
            summary = analyzer.get_module_summary()
            
            print("\n📊 模組摘要報告:")
            print("=" * 60)
            
            for module, stats in summary.items():
                print(f"\n🔧 {module}")
                print(f"   進程數量: {stats['process_count']}")
                print(f"   總記憶體: {stats['total_memory_mb']:.1f} MB")
                print(f"   平均CPU: {stats['avg_cpu_percent']:.1f}%")
                print(f"   總執行緒: {stats['total_threads']}")
                print(f"   記憶體峰值: {stats['max_memory_mb']:.1f} MB")
                
                if stats['memory_growth'] > 10:
                    print(f"   ⚠️ 記憶體增長率: {stats['memory_growth']:.2f} MB/h (疑似洩漏)")
                else:
                    print(f"   ✅ 記憶體增長率: {stats['memory_growth']:.2f} MB/h")
                
                print(f"   執行緒增長率: {stats['thread_growth']:.2f} /h")
                print(f"   監控時長: {stats['time_span_hours']:.1f} 小時")


if __name__ == "__main__":
    main()