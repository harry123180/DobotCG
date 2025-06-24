#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
啟動工具除錯腳本
專門診斷啟動工具無法啟動ModbusTCP服務器的問題
"""

import os
import sys
import subprocess
import socket
import time
import json

def find_project_root():
    """尋找專案根目錄"""
    current_dir = os.getcwd()
    
    # 檢查當前目錄
    if 'DobotM1Project' in current_dir:
        if current_dir.endswith('DobotM1Project'):
            return current_dir
        else:
            # 找到DobotM1Project在路徑中的位置
            parts = current_dir.split(os.sep)
            try:
                index = parts.index('DobotM1Project')
                return os.sep.join(parts[:index + 1])
            except ValueError:
                pass
    
    # 向上查找
    for i in range(3):
        if os.path.exists(os.path.join(current_dir, 'DobotM1Project')):
            return os.path.join(current_dir, 'DobotM1Project')
        current_dir = os.path.dirname(current_dir)
    
    return None

def check_tcpserver_file(project_root):
    """檢查TCPServer.py檔案"""
    print("檢查TCPServer.py檔案...")
    
    tcpserver_path = os.path.join(project_root, 'ModbusServer', 'TCPServer.py')
    
    if not os.path.exists(tcpserver_path):
        print(f"❌ TCPServer.py不存在: {tcpserver_path}")
        return False, None
    
    print(f"✅ TCPServer.py存在: {tcpserver_path}")
    
    # 檢查檔案大小
    file_size = os.path.getsize(tcpserver_path)
    print(f"   檔案大小: {file_size} bytes")
    
    if file_size < 1000:
        print("⚠️  檔案太小，可能不完整")
        return False, tcpserver_path
    
    return True, tcpserver_path

def test_tcpserver_import(tcpserver_path):
    """測試TCPServer.py是否能正確導入"""
    print("\n測試TCPServer.py導入...")
    
    try:
        # 改變工作目錄到ModbusServer
        original_cwd = os.getcwd()
        modbus_dir = os.path.dirname(tcpserver_path)
        os.chdir(modbus_dir)
        
        # 嘗試導入
        sys.path.insert(0, modbus_dir)
        
        # 測試基本導入
        import importlib.util
        spec = importlib.util.spec_from_file_location("TCPServer", tcpserver_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        print("✅ TCPServer.py導入成功")
        
        # 檢查是否有main函數
        if hasattr(module, 'main'):
            print("✅ 找到main函數")
        else:
            print("❌ 未找到main函數")
            return False
        
        return True
        
    except Exception as e:
        print(f"❌ TCPServer.py導入失敗: {e}")
        return False
    finally:
        os.chdir(original_cwd)
        if modbus_dir in sys.path:
            sys.path.remove(modbus_dir)

def test_subprocess_start(tcpserver_path):
    """測試使用subprocess啟動TCPServer"""
    print("\n測試subprocess啟動TCPServer...")
    
    try:
        # 改變工作目錄
        modbus_dir = os.path.dirname(tcpserver_path)
        
        # 啟動程序
        process = subprocess.Popen(
            [sys.executable, tcpserver_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=modbus_dir
        )
        
        print(f"✅ 程序已啟動，PID: {process.pid}")
        
        # 等待一小段時間
        time.sleep(3)
        
        # 檢查程序狀態
        if process.poll() is None:
            print("✅ 程序仍在運行")
            
            # 檢查端口502
            if check_port_502():
                print("✅ 端口502已被監聽")
                result = True
            else:
                print("❌ 端口502未被監聽")
                result = False
            
            # 終止程序
            process.terminate()
            try:
                process.wait(timeout=5)
                print("✅ 程序已正常終止")
            except subprocess.TimeoutExpired:
                process.kill()
                print("⚠️  程序被強制終止")
                
            return result
        else:
            # 程序已退出，獲取錯誤訊息
            stdout, stderr = process.communicate()
            print(f"❌ 程序啟動後立即退出")
            print(f"   返回碼: {process.returncode}")
            if stderr:
                print(f"   錯誤訊息: {stderr.decode('utf-8', errors='ignore')}")
            if stdout:
                print(f"   輸出訊息: {stdout.decode('utf-8', errors='ignore')}")
            return False
            
    except Exception as e:
        print(f"❌ subprocess啟動失敗: {e}")
        return False

def check_port_502():
    """檢查端口502狀態"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1)
            result = s.connect_ex(('127.0.0.1', 502))
            return result == 0
    except Exception:
        return False

def check_startup_tool_config(project_root):
    """檢查啟動工具配置"""
    print("\n檢查啟動工具配置...")
    
    start_dir = os.path.join(project_root, 'start')
    start_py = os.path.join(start_dir, 'start.py')
    
    if not os.path.exists(start_py):
        print(f"❌ start.py不存在: {start_py}")
        return False
    
    print(f"✅ start.py存在: {start_py}")
    
    # 檢查start.py中的路徑配置
    try:
        with open(start_py, 'r', encoding='utf-8') as f:
            content = f.read()
        
        if 'ModbusServer/TCPServer.py' in content or 'ModbusServer\\TCPServer.py' in content:
            print("✅ start.py中包含正確的TCPServer路徑")
        else:
            print("⚠️  start.py中可能缺少正確的TCPServer路徑")
        
        if 'PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)' in content:
            print("✅ start.py中包含正確的專案根目錄設定")
        else:
            print("⚠️  start.py中可能缺少正確的專案根目錄設定")
        
        return True
        
    except Exception as e:
        print(f"❌ 檢查start.py內容失敗: {e}")
        return False

def generate_fix_suggestions(project_root, tcpserver_path):
    """生成修復建議"""
    print("\n" + "=" * 60)
    print("修復建議:")
    print("=" * 60)
    
    print("1. 確認TCPServer.py路徑:")
    print(f"   預期路徑: {tcpserver_path}")
    print(f"   實際存在: {'是' if os.path.exists(tcpserver_path) else '否'}")
    
    print("\n2. 測試單獨啟動TCPServer:")
    print(f"   cd {os.path.dirname(tcpserver_path)}")
    print(f"   python TCPServer.py")
    
    print("\n3. 檢查啟動工具設定:")
    start_py = os.path.join(project_root, 'start', 'start.py')
    print(f"   編輯: {start_py}")
    print("   確認PROJECT_ROOT設定正確")
    
    print("\n4. 檢查端口佔用:")
    print("   netstat -ano | findstr :502")
    
    print("\n5. 重新安裝依賴:")
    print("   pip install --upgrade pymodbus Flask Flask-SocketIO")

def main():
    """主診斷流程"""
    print("=" * 60)
    print("啟動工具ModbusTCP服務器問題診斷")
    print("=" * 60)
    
    # 1. 尋找專案根目錄
    project_root = find_project_root()
    if not project_root:
        print("❌ 找不到DobotM1Project專案目錄")
        print("   請在專案目錄中執行此腳本")
        return
    
    print(f"✅ 專案根目錄: {project_root}")
    
    # 2. 檢查TCPServer.py檔案
    tcpserver_ok, tcpserver_path = check_tcpserver_file(project_root)
    if not tcpserver_ok:
        print("請先確保TCPServer.py檔案存在且完整")
        return
    
    # 3. 測試TCPServer導入
    import_ok = test_tcpserver_import(tcpserver_path)
    
    # 4. 測試subprocess啟動
    subprocess_ok = test_subprocess_start(tcpserver_path)
    
    # 5. 檢查啟動工具配置
    config_ok = check_startup_tool_config(project_root)
    
    # 6. 生成修復建議
    generate_fix_suggestions(project_root, tcpserver_path)
    
    # 7. 總結
    print("\n" + "=" * 60)
    print("診斷結果總結:")
    print("=" * 60)
    print(f"TCPServer.py檔案: {'✅' if tcpserver_ok else '❌'}")
    print(f"TCPServer.py導入: {'✅' if import_ok else '❌'}")
    print(f"subprocess啟動: {'✅' if subprocess_ok else '❌'}")
    print(f"啟動工具配置: {'✅' if config_ok else '❌'}")
    
    if all([tcpserver_ok, import_ok, subprocess_ok, config_ok]):
        print("\n🎉 所有測試通過！啟動工具應該能正常工作")
    else:
        print("\n⚠️  發現問題，請按照修復建議進行處理")

if __name__ == "__main__":
    main()