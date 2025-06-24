# Modbus TCP Server 生產環境部署指南

## 📋 目錄結構

```
ModbusTCPServer/
├── TCPServer.py                    # 主程式碼
├── TCPServer_Production.py         # 生產環境版本
├── build.py                       # 自動化建置腳本
├── modbus_server.spec             # PyInstaller 規格檔
├── requirements.txt               # 依賴清單
├── templates/
│   └── index.html                 # Web 介面模板
├── static/
│   ├── style.css                  # 樣式檔案
│   └── script.js                  # JavaScript 檔案
└── README_DEPLOYMENT.md           # 本檔案
```

## 🚀 快速部署

### 方法一：使用自動化建置腳本 (推薦)

1. **準備環境**
   ```bash
   # 確保 Python 3.7+ 已安裝
   python --version
   
   # 安裝 pip (如果尚未安裝)
   python -m ensurepip --upgrade
   ```

2. **運行建置腳本**
   ```bash
   python build.py
   ```

3. **部署結果**
   - 可執行檔位於: `dist/ModbusTCPServer/`
   - 部署包: `ModbusTCPServer_Production.zip`

### 方法二：手動建置

1. **安裝依賴**
   ```bash
   pip install -r requirements.txt
   ```

2. **建置可執行檔**
   ```bash
   pyinstaller --clean --noconfirm modbus_server.spec
   ```

3. **複製必要檔案**
   ```bash
   copy templates dist/ModbusTCPServer/
   copy static dist/ModbusTCPServer/
   ```

## 🖥️ 生產環境安裝

### 系統需求

- **作業系統**: Windows 10/11 (64-bit)
- **記憶體**: 最少 512MB 可用記憶體
- **磁碟空間**: 100MB 可用空間
- **網路**: 開放 TCP 埠 502 和 8000

### 安裝步驟

1. **解壓縮部署包**
   ```
   解壓 ModbusTCPServer_Production.zip 到目標目錄
   例如: C:\ModbusServer\
   ```

2. **設定防火牆**
   - 允許程式使用 TCP 埠 502 (Modbus TCP)
   - 允許程式使用 TCP 埠 8000 (Web 管理介面)

3. **啟動伺服器**
   - 方法一: 執行 `start_server.bat`
   - 方法二: 直接執行 `ModbusTCPServer.exe`

4. **驗證安裝**
   - 開啟瀏覽器，訪問: http://localhost:8000
   - 確認 Web 管理介面正常顯示

## 🔧 配置說明

### 網路設定

| 服務 | 埠號 | 用途 |
|------|------|------|
| Modbus TCP | 502 | Modbus 協議通訊 |
| Web 管理 | 8000 | 網頁管理介面 |

### Modbus 設定

- **協議**: Modbus TCP/IP
- **SlaveID**: 1 (可透過 Web 介面修改)
- **功能碼**: 
  - 0x03 (Read Holding Registers)
  - 0x06 (Write Single Register)
  - 0x10 (Write Multiple Registers)
- **暫存器範圍**: 0-999 (共 1000 個)
- **數據類型**: 16位無符號整數 (0-65535)

### 檔案說明

- `ModbusTCPServer.exe`: 主程式
- `start_server.bat`: 啟動腳本
- `README.txt`: 快速使用說明
- `modbus_server.log`: 運行日誌
- `register_comments.json`: 暫存器註解檔案

## 📊 使用方式

### Web 管理介面

1. **訪問介面**
   ```
   http://localhost:8000
   ```

2. **主要功能**
   - 查看伺服器狀態
   - 讀寫暫存器數值
   - 設定 SlaveID
   - 匯出/匯入數據
   - 暫存器註解管理

### Modbus 客戶端連接

1. **連接設定**
   - IP 地址: 伺服器 IP 地址
   - 埠號: 502
   - SlaveID: 1 (或透過 Web 介面設定的值)

2. **測試工具推薦**
   - Modbus Poll (商業軟體)
   - QModMaster (開源工具)
   - pymodbus-repl (命令列工具)

## 🛠️ 故障排除

### 常見問題

1. **埠號被占用**
   ```
   錯誤: Address already in use
   解決: 
   - 檢查是否有其他程式使用 502 或 8000 埠
   - 使用 netstat -an | findstr :502 檢查埠號狀態
   ```

2. **防火牆阻擋**
   ```
   錯誤: 無法從外部連接
   解決:
   - 在 Windows 防火牆中新增例外
   - 允許程式通過防火牆
   ```

3. **權限不足**
   ```
   錯誤: Permission denied
   解決:
   - 以管理員權限執行程式
   - 確認程式有網路監聽權限
   ```

### 日誌檢查

檢查 `modbus_server.log` 檔案:
```
錯誤類型           日誌位置
--------          --------
啟動錯誤          開頭部分
連接問題          TCP Server 相關日誌
數據錯誤          暫存器讀寫日誌
Web錯誤           Flask 相關日誌
```

### 效能調優

1. **記憶體使用**
   - 正常: 50-100MB
   - 高負載: 100-200MB

2. **連接數限制**
   - 建議同時連接數 < 10
   - 高頻率請求可能影響效能

## 🔒 安全考量

### 網路安全

1. **存取控制**
   - 僅在受信任網路中運行
   - 考慮使用 VPN 或防火牆限制存取

2. **埠號管理**
   - 不要將 502 埠暴露到公網
   - Web 管理介面可考慮更改埠號

### 數據安全

1. **備份**
   - 定期備份 `register_comments.json`
   - 匯出重要的暫存器數據

2. **監控**
   - 監控日誌檔案大小
   - 定期檢查異常連接

## 📞 技術支援

### 聯絡資訊

- **文件**: 本 README 檔案
- **日誌**: 檢查 `modbus_server.log`
- **測試**: 使用內建的測試數據功能

### 版本資訊

- **版本**: 1.0.0
- **建置日期**: 2025-06-04
- **Python 版本**: 3.7+
- **依賴版本**: 見 requirements.txt

---

© 2025 Modbus TCP Server. 保留所有權利。