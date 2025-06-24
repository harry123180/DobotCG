@echo off
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
