# -*- coding: utf-8 -*-
"""
ProcessMonitor_CG.py - DobotCG多模組記憶體和執行緒監控工具
監控DobotCG自動化專案中各個模組的資源使用情況
基於DobotCG專案路徑結構
"""

import psutil
import time
import os
import json
import threading
from datetime import datetime
from typing import Dict, List, Optional
import logging
from logging.handlers import RotatingFileHandler

class ProcessMonitorCG:
    """DobotCG多模組進程和執行緒監控器"""
    
    def __init__(self, monitor_interval: int = 10):
        """
        初始化監控器
        
        Args:
            monitor_interval: 監控間隔(秒)，預設10秒
        """
        self.monitor_interval = monitor_interval
        self.monitoring = False
        self.monitor_thread = None
        
        # 工作目錄：執行檔同層目錄
        self.working_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_dir = os.path.join(self.working_dir, 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # 設置日誌
        self.setup_logging()
        
        # 監控目標模組列表 - 基於DobotCG實際啟動腳本
        self.target_processes = {
            # 基礎服務
            'TCP-Server': ['TCPServer.py', 'DobotCG\\ModbusServer'],
            
            # VP震動盤模組
            'VP-Main': ['VP_main.py', 'DobotCG\\Automation\\VP'],
            'VP-App': ['VP_app.py', 'DobotCG\\Automation\\VP'],
            
            # LED燈光模組
            'LED-Main': ['LED_main.py', 'DobotCG\\Automation\\light'],
            'LED-App': ['LED_app.py', 'DobotCG\\Automation\\light'],
            
            # 夾爪模組
            'Gripper-Main': ['Gripper.py', 'DobotCG\\Automation\\Gripper'],
            'Gripper-App': ['Gripper_app.py', 'DobotCG\\Automation\\Gripper'],
            
            # 視覺模組
            'CCD1-Vision': ['CCD1VisionCodeYOLO.py', 'DobotCG\\Automation\\CCD1'],
            'CCD3-Main': ['CCD3_main_app.py', 'DobotCG\\Automation\\CCD3'],
            
            # 機械臂模組
            'Dobot-Main': ['Dobot_main.py', 'DobotCG\\Automation\\M1Pro\\new_architecture'],
            
            # 自動化流程
            'AutoFeeding': ['AutoFeeding_main.py', 'DobotCG\\Automation\\AutoFeeding'],
            'AutoProgram-Main': ['AutoProgram_main.py', 'DobotCG\\Automation\\AutoProgram'],
            'AutoProgram-App': ['AutoProgram_app.py', 'DobotCG\\Automation\\AutoProgram'],
        }
        
        # 記憶體歷史記錄
        self.memory_history = {}
        self.thread_history = {}
        self.cpu_history = {}
        
        # 警告閾值
        self.memory_warning_mb = 1000  # 1GB
        self.memory_critical_mb = 2000  # 2GB
        self.cpu_warning_percent = 80
        self.thread_warning_count = 50
        
        self.logger.info("DobotCG ProcessMonitor初始化完成")
        
    def setup_logging(self):
        """設置日誌系統"""
        # 創建日誌格式
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(name)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        # 文件處理器
        log_file = os.path.join(self.log_dir, 'process_monitor_cg.log')
        file_handler = RotatingFileHandler(
            log_file,
            maxBytes=50*1024*1024,  # 50MB
            backupCount=7,
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        
        # 控制台處理器
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        
        # 配置logger
        self.logger = logging.getLogger('ProcessMonitorCG')
        self.logger.setLevel(logging.INFO)
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)
        
    def find_target_processes(self) -> Dict[str, List[psutil.Process]]:
        """找到目標進程 - 基於DobotCG實際啟動腳本精確匹配"""
        found_processes = {}
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'create_time']):
            try:
                proc_info = proc.info
                proc_name = proc_info['name']
                cmdline = ' '.join(proc_info['cmdline']) if proc_info['cmdline'] else ''
                
                # 檢查每個目標模組
                for module_name, search_terms in self.target_processes.items():
                    found = False
                    
                    # 檢查命令行中是否包含目標檔案和DobotCG路徑
                    for search_term in search_terms:
                        if search_term in cmdline:
                            found = True
                            break
                    
                    # 特殊處理：檢查conda activate ROBOT環境和DobotCG專案路徑
                    try:
                        if hasattr(proc, 'name') and 'cmd.exe' in proc_name.lower():
                            if ('conda activate ROBOT' in cmdline and 
                                'DobotCG' in cmdline):
                                for search_term in search_terms:
                                    if search_term.split('\\')[-1] in cmdline:  # 檢查檔案名
                                        found = True
                                        break
                    except:
                        pass
                    
                    if found:
                        if module_name not in found_processes:
                            found_processes[module_name] = []
                        found_processes[module_name].append(proc)
                        
                        # 詳細記錄找到的進程
                        self.logger.debug(f"找到DobotCG模組 {module_name}: PID {proc_info['pid']}, CMD: {cmdline[:100]}...")
                        break  # 避免重複匹配
                        
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
                
        return found_processes
    
    def get_process_info(self, process: psutil.Process) -> Optional[Dict]:
        """獲取進程詳細資訊"""
        try:
            with process.oneshot():
                # 基本資訊
                info = {
                    'pid': process.pid,
                    'name': process.name(),
                    'status': process.status(),
                    'create_time': datetime.fromtimestamp(process.create_time()).strftime('%Y-%m-%d %H:%M:%S'),
                    'running_time_hours': (time.time() - process.create_time()) / 3600,
                }
                
                # 記憶體資訊
                memory_info = process.memory_info()
                memory_percent = process.memory_percent()
                info.update({
                    'memory_rss_mb': memory_info.rss / 1024 / 1024,
                    'memory_vms_mb': memory_info.vms / 1024 / 1024,
                    'memory_percent': memory_percent,
                })
                
                # CPU資訊
                try:
                    cpu_percent = process.cpu_percent()
                    info['cpu_percent'] = cpu_percent
                except:
                    info['cpu_percent'] = 0.0
                
                # 執行緒資訊
                try:
                    threads = process.threads()
                    info['thread_count'] = len(threads)
                    info['threads'] = [{'id': t.id, 'user_time': t.user_time, 'system_time': t.system_time} 
                                     for t in threads]
                except:
                    info['thread_count'] = 0
                    info['threads'] = []
                
                # 檔案描述符
                try:
                    info['open_files_count'] = len(process.open_files())
                except:
                    info['open_files_count'] = 0
                
                # 網路連接
                try:
                    info['connections_count'] = len(process.connections())
                except:
                    info['connections_count'] = 0
                
                return info
                
        except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
            self.logger.warning(f"無法獲取進程資訊 PID {process.pid}: {e}")
            return None
    
    def check_memory_leak(self, module_name: str, current_memory: float):
        """檢查記憶體洩漏"""
        if module_name not in self.memory_history:
            self.memory_history[module_name] = []
        
        self.memory_history[module_name].append({
            'timestamp': time.time(),
            'memory_mb': current_memory
        })
        
        # 保留最近100個記錄
        if len(self.memory_history[module_name]) > 100:
            self.memory_history[module_name] = self.memory_history[module_name][-100:]
        
        # 分析記憶體趨勢
        if len(self.memory_history[module_name]) >= 10:
            recent_10 = self.memory_history[module_name][-10:]
            first_memory = recent_10[0]['memory_mb']
            last_memory = recent_10[-1]['memory_mb']
            growth_rate = (last_memory - first_memory) / len(recent_10)  # MB per check
            
            if growth_rate > 5.0:  # 每次檢查增長超過5MB
                time_span = recent_10[-1]['timestamp'] - recent_10[0]['timestamp']
                hourly_growth = growth_rate * 360  # 每小時增長(假設10秒檢查一次)
                
                self.logger.warning(
                    f"DobotCG記憶體洩漏警告 [{module_name}]: "
                    f"當前{current_memory:.1f}MB, "
                    f"增長率{growth_rate:.2f}MB/次, "
                    f"預估每小時增長{hourly_growth:.1f}MB"
                )
                
                return True
        
        return False
    
    def analyze_threads(self, module_name: str, threads: List[Dict]):
        """分析執行緒狀況"""
        thread_count = len(threads)
        
        if module_name not in self.thread_history:
            self.thread_history[module_name] = []
        
        self.thread_history[module_name].append({
            'timestamp': time.time(),
            'thread_count': thread_count,
            'threads': threads
        })
        
        # 保留最近50個記錄
        if len(self.thread_history[module_name]) > 50:
            self.thread_history[module_name] = self.thread_history[module_name][-50:]
        
        # 檢查執行緒數量異常增長
        if len(self.thread_history[module_name]) >= 5:
            recent_counts = [h['thread_count'] for h in self.thread_history[module_name][-5:]]
            max_threads = max(recent_counts)
            min_threads = min(recent_counts)
            
            if max_threads - min_threads > 10:
                self.logger.warning(
                    f"DobotCG執行緒數量波動 [{module_name}]: "
                    f"當前{thread_count}個, "
                    f"最近5次檢查範圍{min_threads}-{max_threads}"
                )
        
        # 檢查執行緒過多
        if thread_count > self.thread_warning_count:
            self.logger.warning(
                f"DobotCG執行緒數量過多 [{module_name}]: {thread_count}個執行緒"
            )
    
    def generate_report(self) -> Dict:
        """生成監控報告"""
        processes = self.find_target_processes()
        report = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'system_info': {
                'cpu_percent': psutil.cpu_percent(interval=1),
                'memory_percent': psutil.virtual_memory().percent,
                'disk_percent': psutil.disk_usage('/').percent if os.name != 'nt' else psutil.disk_usage('C:\\').percent,
                'boot_time': datetime.fromtimestamp(psutil.boot_time()).strftime('%Y-%m-%d %H:%M:%S'),
            },
            'modules': {}
        }
        
        total_memory = 0
        total_cpu = 0
        total_threads = 0
        
        for module_name, proc_list in processes.items():
            module_info = {
                'process_count': len(proc_list),
                'processes': []
            }
            
            module_memory = 0
            module_cpu = 0
            module_threads = 0
            
            for proc in proc_list:
                proc_info = self.get_process_info(proc)
                if proc_info:
                    module_info['processes'].append(proc_info)
                    module_memory += proc_info['memory_rss_mb']
                    module_cpu += proc_info['cpu_percent']
                    module_threads += proc_info['thread_count']
                    
                    # 檢查警告
                    if proc_info['memory_rss_mb'] > self.memory_warning_mb:
                        self.logger.warning(
                            f"DobotCG記憶體警告 [{module_name} PID:{proc_info['pid']}]: "
                            f"{proc_info['memory_rss_mb']:.1f}MB"
                        )
                    
                    if proc_info['cpu_percent'] > self.cpu_warning_percent:
                        self.logger.warning(
                            f"DobotCG CPU使用率警告 [{module_name} PID:{proc_info['pid']}]: "
                            f"{proc_info['cpu_percent']:.1f}%"
                        )
                    
                    # 檢查記憶體洩漏
                    self.check_memory_leak(f"{module_name}_{proc_info['pid']}", proc_info['memory_rss_mb'])
                    
                    # 分析執行緒
                    self.analyze_threads(f"{module_name}_{proc_info['pid']}", proc_info['threads'])
            
            module_info.update({
                'total_memory_mb': module_memory,
                'total_cpu_percent': module_cpu,
                'total_threads': module_threads
            })
            
            report['modules'][module_name] = module_info
            
            total_memory += module_memory
            total_cpu += module_cpu
            total_threads += module_threads
        
        report['totals'] = {
            'total_memory_mb': total_memory,
            'total_cpu_percent': total_cpu,
            'total_threads': total_threads
        }
        
        return report
    
    def save_report(self, report: Dict):
        """保存報告到檔案"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = os.path.join(self.log_dir, f'monitor_report_cg_{timestamp}.json')
        
        try:
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.logger.error(f"保存DobotCG報告失敗: {e}")
    
    def print_summary(self, report: Dict):
        """打印監控摘要 - 按照DobotCG實際模組分組顯示"""
        print("\n" + "="*80)
        print(f"📊 DobotCG多模組監控報告 - {report['timestamp']}")
        print("="*80)
        
        # 系統資訊
        sys_info = report['system_info']
        print(f"🖥️  系統狀態:")
        print(f"   CPU使用率: {sys_info['cpu_percent']:.1f}%")
        print(f"   記憶體使用率: {sys_info['memory_percent']:.1f}%")
        print(f"   磁碟使用率: {sys_info['disk_percent']:.1f}%")
        print(f"   系統啟動時間: {sys_info['boot_time']}")
        
        # 總計
        totals = report['totals']
        print(f"\n📈 總計使用情況:")
        print(f"   總記憶體: {totals['total_memory_mb']:.1f} MB")
        print(f"   總CPU: {totals['total_cpu_percent']:.1f}%")
        print(f"   總執行緒: {totals['total_threads']} 個")
        
        # 按模組類型分組顯示
        module_groups = {
            '🔧 基礎服務': ['TCP-Server'],
            '📳 VP震動盤模組': ['VP-Main', 'VP-App'],
            '💡 LED燈光模組': ['LED-Main', 'LED-App'],
            '🤖 夾爪模組': ['Gripper-Main', 'Gripper-App'],
            '👁️  視覺模組': ['CCD1-Vision', 'CCD3-Main'],
            '🦾 機械臂模組': ['Dobot-Main'],
            '⚙️  自動化流程': ['AutoFeeding', 'AutoProgram-Main', 'AutoProgram-App'],
        }
        
        print(f"\n📋 DobotCG模組詳細狀況:")
        
        for group_name, module_list in module_groups.items():
            group_found = False
            group_memory = 0
            group_cpu = 0
            group_threads = 0
            
            for module_name in module_list:
                if module_name in report['modules'] and report['modules'][module_name]['process_count'] > 0:
                    if not group_found:
                        print(f"\n{group_name}")
                        group_found = True
                    
                    module_info = report['modules'][module_name]
                    group_memory += module_info['total_memory_mb']
                    group_cpu += module_info['total_cpu_percent']
                    group_threads += module_info['total_threads']
                    
                    print(f"   ✅ {module_name}:")
                    print(f"      進程數: {module_info['process_count']}")
                    print(f"      記憶體: {module_info['total_memory_mb']:.1f} MB")
                    print(f"      CPU: {module_info['total_cpu_percent']:.1f}%")
                    print(f"      執行緒: {module_info['total_threads']} 個")
                    
                    # 顯示每個進程詳情
                    for proc in module_info['processes']:
                        runtime = proc['running_time_hours']
                        status_icon = "🟢" if proc['status'] == 'running' else "🟡"
                        
                        # CCD1特別標示
                        if module_name == 'CCD1-Vision':
                            memory_status = ""
                            if proc['memory_rss_mb'] > 1500:
                                memory_status = " ⚠️ 記憶體過高!"
                            elif proc['memory_rss_mb'] > 1000:
                                memory_status = " ⚡ 記憶體偏高"
                            
                            print(f"         {status_icon} PID {proc['pid']}: {proc['memory_rss_mb']:.1f}MB{memory_status}, "
                                  f"{proc['cpu_percent']:.1f}%CPU, {proc['thread_count']}執行緒, "
                                  f"運行{runtime:.1f}h")
                        else:
                            print(f"         {status_icon} PID {proc['pid']}: {proc['memory_rss_mb']:.1f}MB, "
                                  f"{proc['cpu_percent']:.1f}%CPU, {proc['thread_count']}執行緒, "
                                  f"運行{runtime:.1f}h")
                else:
                    if not group_found:
                        print(f"\n{group_name}")
                        group_found = True
                    print(f"   ❌ {module_name}: 未運行")
            
            # 顯示群組總計
            if group_found and (group_memory > 0 or group_cpu > 0):
                print(f"      📊 群組總計: {group_memory:.1f}MB, {group_cpu:.1f}%CPU, {group_threads}執行緒")
        
        print("="*80)
    
    def monitor_loop(self):
        """監控主循環"""
        self.logger.info("開始DobotCG監控循環")
        
        while self.monitoring:
            try:
                start_time = time.time()
                
                # 生成報告
                report = self.generate_report()
                
                # 打印摘要
                self.print_summary(report)
                
                # 每10次循環保存一次詳細報告
                if hasattr(self, '_loop_count'):
                    self._loop_count += 1
                else:
                    self._loop_count = 1
                
                if self._loop_count % 10 == 0:
                    self.save_report(report)
                    self.logger.info(f"已保存第 {self._loop_count} 次DobotCG監控報告")
                
                # 等待下一次檢查
                elapsed = time.time() - start_time
                sleep_time = max(0, self.monitor_interval - elapsed)
                time.sleep(sleep_time)
                
            except KeyboardInterrupt:
                self.logger.info("收到中斷信號，停止DobotCG監控")
                break
            except Exception as e:
                self.logger.error(f"DobotCG監控循環異常: {e}")
                time.sleep(5)  # 異常後等待5秒再繼續
        
        self.monitoring = False
        self.logger.info("DobotCG監控循環已停止")
    
    def start_monitoring(self):
        """啟動監控"""
        if self.monitoring:
            self.logger.warning("DobotCG監控已在運行中")
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=False)
        self.monitor_thread.start()
        self.logger.info(f"DobotCG監控已啟動，間隔 {self.monitor_interval} 秒")
    
    def stop_monitoring(self):
        """停止監控"""
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5)
        self.logger.info("DobotCG監控已停止")
    
    def get_memory_trend_analysis(self) -> Dict:
        """獲取記憶體趨勢分析"""
        analysis = {}
        
        for module_proc, history in self.memory_history.items():
            if len(history) < 5:
                continue
            
            # 計算趨勢
            memories = [h['memory_mb'] for h in history]
            timestamps = [h['timestamp'] for h in history]
            
            # 線性回歸計算增長率
            n = len(memories)
            sum_x = sum(range(n))
            sum_y = sum(memories)
            sum_xy = sum(i * memories[i] for i in range(n))
            sum_x2 = sum(i*i for i in range(n))
            
            if n * sum_x2 - sum_x * sum_x != 0:  # 避免除零
                slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
                intercept = (sum_y - slope * sum_x) / n
                
                # 計算時間跨度(小時)
                time_span_hours = (timestamps[-1] - timestamps[0]) / 3600
                hourly_growth = slope * (3600 / self.monitor_interval)  # 每小時增長
                
                analysis[module_proc] = {
                    'current_memory': memories[-1],
                    'initial_memory': memories[0],
                    'total_growth': memories[-1] - memories[0],
                    'hourly_growth_rate': hourly_growth,
                    'time_span_hours': time_span_hours,
                    'samples': len(history)
                }
        
        return analysis


def main():
    """主函數"""
    import argparse
    
    parser = argparse.ArgumentParser(description='DobotCG多模組進程監控工具')
    parser.add_argument('--interval', '-i', type=int, default=10, help='監控間隔(秒)，預設10秒')
    parser.add_argument('--memory-warning', type=int, default=1000, help='記憶體警告閾值(MB)，預設1000MB')
    parser.add_argument('--cpu-warning', type=int, default=80, help='CPU警告閾值(%)，預設80%')
    parser.add_argument('--debug', action='store_true', help='啟用詳細除錯資訊')
    
    args = parser.parse_args()
    
    print("🚀 DobotCG多模組進程監控器啟動")
    print(f"📊 監控間隔: {args.interval} 秒")
    print(f"⚠️  記憶體警告閾值: {args.memory_warning} MB")
    print(f"⚠️  CPU警告閾值: {args.cpu_warning}%")
    print("🎯 DobotCG監控目標模組:")
    print("   • TCP-Server (Modbus TCP伺服器)")
    print("   • VP-Main/VP-App (震動盤模組)")
    print("   • LED-Main/LED-App (LED燈光模組)")
    print("   • Gripper-Main/Gripper-App (夾爪模組)")
    print("   • CCD1-Vision/CCD3-Main (視覺模組)")
    print("   • Dobot-Main (機械臂模組)")
    print("   • AutoFeeding/AutoProgram-Main/AutoProgram-App (自動化流程)")
    print("📁 日誌目錄: ./logs/")
    print("💡 按 Ctrl+C 停止監控")
    print("="*80)
    
    monitor = ProcessMonitorCG(monitor_interval=args.interval)
    monitor.memory_warning_mb = args.memory_warning
    monitor.cpu_warning_percent = args.cpu_warning
    
    if args.debug:
        monitor.logger.setLevel(logging.DEBUG)
    
    try:
        monitor.start_monitoring()
        
        # 保持主執行緒運行
        while monitor.monitoring:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n🛑 收到停止信號")
    finally:
        monitor.stop_monitoring()
        
        # 最終記憶體趨勢分析
        print("\n📈 DobotCG記憶體趨勢分析:")
        trend_analysis = monitor.get_memory_trend_analysis()
        
        critical_modules = []
        for module_proc, analysis in trend_analysis.items():
            if analysis['hourly_growth_rate'] > 10:  # 每小時增長超過10MB
                critical_modules.append((module_proc, analysis))
        
        if critical_modules:
            print("⚠️  發現DobotCG記憶體洩漏嫌疑模組:")
            for module_proc, analysis in critical_modules:
                print(f"   🔴 {module_proc}:")
                print(f"      當前記憶體: {analysis['current_memory']:.1f} MB")
                print(f"      每小時增長: {analysis['hourly_growth_rate']:.2f} MB/h")
                print(f"      監控時長: {analysis['time_span_hours']:.1f} 小時")
                print(f"      總增長: {analysis['total_growth']:.1f} MB")
        else:
            print("✅ 未發現明顯的記憶體洩漏問題")
        
        print("✅ DobotCG監控器已安全停止")


if __name__ == "__main__":
    main()