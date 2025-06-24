# -*- mode: python ; coding: utf-8 -*-
# modbus_server.spec
# PyInstaller 規格檔案用於打包 Modbus TCP Server

block_cipher = None

# 收集所有需要的數據檔案
datas = [
    ('templates', 'templates'),
    ('static', 'static'),
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
    icon='icon.ico'  # 如果有圖示檔案的話
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
)