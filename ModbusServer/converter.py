#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
圖示轉換工具
將 PNG 圖示轉換為 ICO 格式，用於 PyInstaller 建置
"""

import os
import sys
from PIL import Image

def convert_png_to_ico(png_path, ico_path):
    """將 PNG 圖示轉換為 ICO 格式"""
    try:
        print(f"🔄 轉換圖示: {png_path} -> {ico_path}")
        
        # 開啟 PNG 圖片
        with Image.open(png_path) as img:
            # 確保圖片為 RGBA 模式
            if img.mode != 'RGBA':
                img = img.convert('RGBA')
            
            # 創建多種尺寸的圖示 (Windows 標準尺寸)
            sizes = [(16, 16), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)]
            
            # 調整圖片尺寸並保存為 ICO
            resized_images = []
            for size in sizes:
                resized_img = img.resize(size, Image.Resampling.LANCZOS)
                resized_images.append(resized_img)
            
            # 保存為 ICO 檔案
            resized_images[0].save(
                ico_path, 
                format='ICO', 
                sizes=sizes
            )
        
        print(f"✅ 圖示轉換完成: {ico_path}")
        return True
        
    except ImportError:
        print("❌ 缺少 Pillow 庫，正在安裝...")
        import subprocess
        result = subprocess.run([sys.executable, '-m', 'pip', 'install', 'Pillow'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ Pillow 安裝完成，請重新運行此腳本")
        else:
            print("❌ Pillow 安裝失敗")
        return False
        
    except Exception as e:
        print(f"❌ 圖示轉換失敗: {e}")
        return False

def create_alternative_spec():
    """創建使用 PNG 圖示的替代 spec 檔案"""
    spec_content = '''# -*- mode: python ; coding: utf-8 -*-
# modbus_server_png.spec
# PyInstaller 規格檔案用於打包 Modbus TCP Server (使用 PNG 圖示)

block_cipher = None

# 收集所有需要的數據檔案
datas = [
    ('templates', 'templates'),
    ('static', 'static'),
    ('640.png', '.'),  # 包含圖示檔案
]

# 隱藏導入的模組
hiddenimports = [
    'pymodbus',
    'pymodbus.server',
    'pymodbus.device',
    'pymodbus.datastore',
    'pymodbus.client',
    'flask',
    'werkzeug',
    'jinja2',
    'markupsafe',
    'click',
    'itsdangerous',
    'cryptography',
    'pyserial',
    'twisted',
    'pymodbus.framer.socket_framer',
    'pymodbus.framer.rtu_framer',
    'pymodbus.framer.ascii_framer',
    'pymodbus.transaction',
    'pymodbus.constants',
    'pymodbus.utilities',
    'logging.handlers',
]

a = Analysis(
    ['TCPServer.py'],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'matplotlib',
        'numpy',
        'pandas',
        'scipy',
        'tkinter',
        'PIL',
        'cv2',
        'torch',
        'tensorflow',
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='ModbusTCPServer',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    # 注意: PNG 圖示需要先轉換為 ICO 格式
    # icon='640.ico'  # 如果有轉換的 ICO 檔案
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='ModbusTCPServer'
)'''
    
    with open('modbus_server_png.spec', 'w', encoding='utf-8') as f:
        f.write(spec_content)
    
    print("✅ 已創建 PNG 圖示專用的 spec 檔案: modbus_server_png.spec")

def main():
    """主函數"""
    print("🎨 Modbus TCP Server 圖示轉換工具")
    print("=" * 50)
    
    png_file = '640.png'
    ico_file = '640.ico'
    
    # 檢查 PNG 檔案是否存在
    if not os.path.exists(png_file):
        print(f"❌ 找不到圖示檔案: {png_file}")
        return False
    
    print(f"📁 找到圖示檔案: {png_file}")
    
    # 嘗試轉換為 ICO 格式
    if convert_png_to_ico(png_file, ico_file):
        print(f"\n✅ 轉換成功！現在可以在 PyInstaller 中使用 {ico_file}")
        print("💡 建議在 modbus_server.spec 中將 icon 設定為 '640.ico'")
    else:
        print(f"\n⚠️  無法轉換為 ICO 格式，創建替代方案...")
        create_alternative_spec()
        print("💡 請使用 modbus_server_png.spec 進行建置")
    
    return True

if __name__ == "__main__":
    success = main()
    if not success:
        print("\n❌ 圖示處理失敗")
        sys.exit(1)
    else:
        print("\n✅ 圖示處理完成！")
        sys.exit(0)