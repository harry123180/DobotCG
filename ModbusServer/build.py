#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自動化建置腳本
用於將 Modbus TCP Server 打包成可執行檔案
"""

import os
import sys
import shutil
import subprocess
import zipfile
from pathlib import Path

def check_requirements():
    """檢查建置需求"""
    print("🔍 檢查建置環境...")
    
    # 檢查 Python 版本
    if sys.version_info < (3, 7):
        print("❌ 需要 Python 3.7 或更高版本")
        return False
    
    # 檢查必要檔案
    required_files = [
        'TCPServer.py',
        'templates/index.html',
        'static/style.css',
        'static/script.js'
    ]
    
    for file in required_files:
        if not os.path.exists(file):
            print(f"❌ 缺少必要檔案: {file}")
            return False
    
    print("✅ 建置環境檢查通過")
    return True

def install_dependencies():
    """安裝建置依賴"""
    print("📦 安裝建置依賴...")
    
    dependencies = [
        'pyinstaller>=5.0',
        'pymodbus>=3.0.0',
        'flask>=2.0.0',
        'werkzeug>=2.0.0'
    ]
    
    for dep in dependencies:
        print(f"安裝 {dep}...")
        result = subprocess.run([sys.executable, '-m', 'pip', 'install', dep], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            print(f"❌ 安裝 {dep} 失敗")
            print(result.stderr)
            return False
    
    print("✅ 依賴安裝完成")
    return True

def create_directories():
    """創建必要的目錄結構"""
    print("📁 創建目錄結構...")
    
    directories = [
        'templates',
        'static',
        'build',
        'dist'
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
    
    print("✅ 目錄結構創建完成")

def create_html_template():
    """創建 HTML 模板檔案"""
    print("📄 創建 HTML 模板...")
    
    html_content = '''<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Modbus TCP Server 管理介面</title>
    <link rel="stylesheet" href="/static/style.css">
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🔧 Modbus TCP Server 管理介面</h1>
            <p>伺服器地址: 0.0.0.0:502 | Web管理: 0.0.0.0:8000 | 數值範圍: 0-65535 (無符號)</p>
        </div>
        
        <!-- 伺服器狀態區塊 -->
        <div class="section status">
            <h2>📊 伺服器狀態</h2>
            <div id="status-info">載入中...</div>
            <button onclick="refreshStatus()">🔄 刷新狀態</button>
            <button onclick="toggleAutoRefresh()" id="auto-refresh-btn">🔄 開啟自動刷新</button>
        </div>
        
        <!-- 控制面板 -->
        <div class="section controls">
            <h2>⚙️ 控制面板</h2>
            <div class="control-row">
                <label>SlaveID (1-247):</label>
                <input type="number" id="slave-id" min="1" max="247" value="1">
                <button onclick="updateSlaveId()">更新 SlaveID</button>
            </div>
            <div class="control-row">
                <label>暫存器地址 (0-999):</label>
                <input type="number" id="reg-address" min="0" max="999" value="0">
                <label>值 (0 ~ 65535):</label>
                <input type="number" id="reg-value" min="0" max="65535" value="0">
                <button onclick="writeRegister()">寫入暫存器</button>
                <button onclick="readRegister()">讀取暫存器</button>
            </div>
            <div id="result-message"></div>
        </div>
        
        <!-- 暫存器範圍顯示設定 -->
        <div class="section display-settings">
            <h2>🖥️ 顯示設定</h2>
            <div class="control-row">
                <label>起始地址:</label>
                <input type="number" id="display-start" min="0" max="999" value="0">
                <label>顯示數量:</label>
                <input type="number" id="display-count" min="1" max="100" value="20">
                <label>顯示格式:</label>
                <select id="display-format">
                    <option value="decimal">無符號十進制</option>
                    <option value="hex">十六進制</option>
                    <option value="binary">二進制</option>
                    <option value="signed">有符號十進制</option>
                </select>
                <button onclick="updateDisplay()">更新顯示</button>
            </div>
        </div>
        
        <!-- 暫存器顯示區域 -->
        <div class="section registers">
            <h2>📋 暫存器狀態</h2>
            <div class="registers-header">
                <span>地址範圍: <span id="address-range">0-19</span></span>
                <span>格式: <span id="current-format">無符號十進制</span></span>
            </div>
            <div id="registers-grid" class="register-grid">載入中...</div>
        </div>
        
        <!-- 快速操作區 -->
        <div class="section quick-actions">
            <h2>⚡ 快速操作</h2>
            <div class="control-row">
                <button onclick="clearAllRegisters()">清除所有暫存器</button>
                <button onclick="setTestData()">設定測試數據</button>
                <button onclick="exportRegisters()">匯出暫存器數據</button>
                <input type="file" id="import-file" accept=".json" style="display:none" onchange="importRegisters()">
                <button onclick="document.getElementById('import-file').click()">匯入暫存器數據</button>
            </div>
        </div>
        
        <!-- 使用說明 -->
        <div class="section">
            <h2>📖 使用說明</h2>
            <ul>
                <li><strong>Modbus Poll 連接設定:</strong> 連接到目標電腦IP:502</li>
                <li><strong>Function Code:</strong> 使用 0x03 (Read Holding Registers) 讀取暫存器</li>
                <li><strong>SlaveID:</strong> 預設為 1，可透過控制面板修改</li>
                <li><strong>暫存器範圍:</strong> 0-999 (共1000個暫存器)</li>
                <li><strong>數據類型:</strong> 16位無符號整數 (0 到 65535)</li>
                <li><strong>有符號顯示:</strong> 選擇「有符號十進制」格式時，大於32767的值會顯示為負數</li>
                <li><strong>即時同步:</strong> 支援外部Modbus設備讀寫時的即時更新</li>
                <li><strong>註解功能:</strong> 點擊暫存器格子可添加用途註解</li>
                <li><strong>格式轉換:</strong> 支援十進制、十六進制、二進制和有符號十進制顯示</li>
            </ul>
        </div>
    </div>

    <script src="/static/script.js"></script>
</body>
</html>'''
    
    with open('templates/index.html', 'w', encoding='utf-8') as f:
        f.write(html_content)
    
    print("✅ HTML 模板創建完成")

def build_executable():
    """使用 PyInstaller 建置可執行檔"""
    print("🔨 開始建置可執行檔...")
    
    # 清理舊的建置檔案
    if os.path.exists('build'):
        shutil.rmtree('build')
    if os.path.exists('dist'):
        shutil.rmtree('dist')
    
    # 執行 PyInstaller
    cmd = [
        sys.executable, '-m', 'PyInstaller',
        '--clean',
        '--noconfirm',
        'modbus_server.spec'
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        print("❌ 建置失敗")
        print(result.stderr)
        return False
    
    print("✅ 可執行檔建置完成")
    return True

def create_installer_config():
    """創建安裝程式配置檔"""
    print("📝 創建部署配置...")
    
    # 創建 README
    readme_content = '''# Modbus TCP Server

## 系統需求
- Windows 10/11 (64-bit)
- 可用記憶體: 最少 512MB
- 網路連接埠: 502 (Modbus TCP), 8000 (Web管理介面)

## 安裝步驟
1. 解壓縮檔案到目標目錄
2. 以管理員權限執行 ModbusTCPServer.exe
3. 在瀏覽器中開啟 http://localhost:8000

## 防火牆設定
請確保 Windows 防火牆允許程式使用連接埠 502 和 8000

## 網路連接
- Modbus TCP 連接埠: 502
- Web 管理介面: 8000
- 支援的 Modbus 功能碼: 0x03 (Read Holding Registers), 0x06 (Write Single Register)

## 技術支援
如有問題請檢查 modbus_server.log 日誌檔案
'''
    
    with open('dist/ModbusTCPServer/README.txt', 'w', encoding='utf-8') as f:
        f.write(readme_content)
    
    # 創建啟動腳本
    start_script = '''@echo off
title Modbus TCP Server
echo 正在啟動 Modbus TCP Server...
echo.
echo Modbus TCP 連接埠: 502
echo Web 管理介面: http://localhost:8000
echo.
echo 請保持此視窗開啟，關閉視窗將停止伺服器
echo 按 Ctrl+C 可安全關閉伺服器
echo.
ModbusTCPServer.exe
pause
'''
    
    with open('dist/ModbusTCPServer/start_server.bat', 'w', encoding='utf-8') as f:
        f.write(start_script)
    
    print("✅ 部署配置創建完成")

def create_zip_package():
    """創建部署壓縮包"""
    print("📦 創建部署壓縮包...")
    
    zip_filename = 'ModbusTCPServer_Production.zip'
    
    with zipfile.ZipFile(zip_filename, 'w', zipfile.ZIP_DEFLATED) as zipf:
        # 遍歷 dist 目錄並加入所有檔案
        for root, dirs, files in os.walk('dist/ModbusTCPServer'):
            for file in files:
                file_path = os.path.join(root, file)
                arcname = os.path.relpath(file_path, 'dist')
                zipf.write(file_path, arcname)
    
    file_size = os.path.getsize(zip_filename) / (1024 * 1024)  # MB
    print(f"✅ 部署包創建完成: {zip_filename} ({file_size:.1f} MB)")

def main():
    """主建置流程"""
    print("🚀 Modbus TCP Server 建置工具")
    print("=" * 50)
    
    # 檢查環境
    if not check_requirements():
        return False
    
    # 安裝依賴
    if not install_dependencies():
        return False
    
    # 創建目錄
    create_directories()
    
    # 創建檔案 (如果不存在)
    if not os.path.exists('templates/index.html'):
        create_html_template()
    
    # 建置可執行檔
    if not build_executable():
        return False
    
    # 創建部署配置
    create_installer_config()
    
    # 創建壓縮包
    create_zip_package()
    
    print("\n🎉 建置完成！")
    print(f"📁 可執行檔位置: dist/ModbusTCPServer/")
    print(f"📦 部署包: ModbusTCPServer_Production.zip")
    print(f"🌐 使用方式: 解壓縮後執行 start_server.bat")
    
    return True

if __name__ == "__main__":
    success = main()
    if not success:
        print("\n❌ 建置失敗，請檢查錯誤訊息")
        sys.exit(1)
    else:
        print("\n✅ 建置成功！")
        sys.exit(0)