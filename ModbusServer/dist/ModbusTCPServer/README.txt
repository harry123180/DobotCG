# Modbus TCP Server

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
