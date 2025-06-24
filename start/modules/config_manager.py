#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json

class ConfigManager:
    """配置檔案管理器"""
    
    @staticmethod
    def read_com_port(config_path, module_type):
        """讀取config檔案中的COM口設定"""
        try:
            if not os.path.exists(config_path):
                return None
                
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            com_port_mappings = {
                "Gripper": ['rtu_connection', 'port'],
                "LED": ['serial_connection', 'port'],
                "XC100": ['xc_connection', 'port']
            }
            
            if module_type in com_port_mappings:
                keys = com_port_mappings[module_type]
                data = config
                for key in keys:
                    if key in data:
                        data = data[key]
                    else:
                        return None
                return data
            return None
            
        except Exception as e:
            print(f"讀取配置檔案錯誤: {e}")
            return None
    
    @staticmethod
    def update_com_port(config_path, module_type, new_com):
        """更新config檔案中的COM口設定"""
        try:
            if not os.path.exists(config_path):
                return False
                
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            com_port_mappings = {
                "Gripper": ['rtu_connection', 'port'],
                "LED": ['serial_connection', 'port'],
                "XC100": ['xc_connection', 'port']
            }
            
            if module_type in com_port_mappings:
                keys = com_port_mappings[module_type]
                data = config
                for key in keys[:-1]:
                    if key not in data:
                        data[key] = {}
                    data = data[key]
                data[keys[-1]] = new_com
                
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                return True
            return False
            
        except Exception as e:
            print(f"更新配置檔案錯誤: {e}")
            return False
    
    @staticmethod
    def validate_config(config_path):
        """驗證config檔案格式正確性"""
        try:
            if not os.path.exists(config_path):
                return False, "配置檔案不存在"
            
            with open(config_path, 'r', encoding='utf-8') as f:
                json.load(f)
            
            return True, "配置檔案格式正確"
            
        except json.JSONDecodeError as e:
            return False, f"JSON格式錯誤: {e}"
        except Exception as e:
            return False, f"驗證錯誤: {e}"
