#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import time
from .config_manager import ConfigManager

class ModuleController:
    """單一模組控制器"""
    
    def __init__(self, name, script_path, config_path=None, needs_com=False):
        self.name = name
        self.script_path = script_path
        self.config_path = config_path
        self.needs_com = needs_com
        self.process = None
        
    def start(self, com_port=None):
        """啟動模組"""
        try:
            if self.is_running():
                return False, "模組已運行中"
            
            # 如果需要COM口，先更新配置
            if self.needs_com and com_port and self.config_path:
                module_type = self.name.replace("_app", "")
                success = ConfigManager.update_com_port(self.config_path, module_type, com_port)
                if not success:
                    return False, f"COM口配置更新失敗: {com_port}"
            
            # 檢查腳本檔案存在
            if not os.path.exists(self.script_path):
                return False, f"腳本檔案不存在: {self.script_path}"
            
            # 啟動程序
            self.process = subprocess.Popen(
                [sys.executable, self.script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=os.path.dirname(self.script_path)
            )
            
            # 等待一小段時間確認啟動成功
            time.sleep(2)
            if self.process.poll() is not None:
                stdout, stderr = self.process.communicate()
                error_msg = stderr.decode('utf-8', errors='ignore')
                return False, f"程序啟動失敗: {error_msg}"
            
            return True, "模組啟動成功"
            
        except Exception as e:
            return False, f"啟動異常: {str(e)}"
    
    def stop(self):
        """停止模組"""
        try:
            if not self.is_running():
                return True, "模組未運行"
            
            self.process.terminate()
            
            # 等待程序結束
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            
            self.process = None
            return True, "模組停止成功"
            
        except Exception as e:
            return False, f"停止異常: {str(e)}"
    
    def is_running(self):
        """檢查模組是否運行中"""
        if self.process is None:
            return False
        
        return self.process.poll() is None
    
    def get_status(self):
        """獲取模組狀態"""
        if self.is_running():
            return "運行中"
        else:
            return "停止"
