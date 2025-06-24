// DobotM1專案啟動工具前端邏輯

class StartupTool {
    constructor() {
        this.socket = io();
        this.currentStatus = {
            modbus_server: false,
            modules: {},
            web_apps: {},
            com_ports: []
        };
        
        this.init();
    }

    init() {
        this.initializeDOM();
        this.initializeEventListeners();
        this.initializeSocketEvents();
        this.loadComPorts();
        this.startStatusMonitoring();
        this.addLog('系統', '啟動工具已初始化');
    }

    initializeDOM() {
        this.elements = {
            modbusStartBtn: document.getElementById('modbus-start'),
            modbusStopBtn: document.getElementById('modbus-stop'),
            modbusStatusDot: document.getElementById('modbus-status-dot'),
            modbusStatusText: document.getElementById('modbus-status-text'),
            logsContainer: document.getElementById('logs'),
            toggleSwitches: document.querySelectorAll('.toggle-switch'),
            comSelects: document.querySelectorAll('.com-select')
        };
    }

    initializeEventListeners() {
        // ModbusTCP服務器控制
        this.elements.modbusStartBtn.addEventListener('click', () => this.controlModbusServer('start'));
        this.elements.modbusStopBtn.addEventListener('click', () => this.controlModbusServer('stop'));

        // Toggle開關事件
        this.elements.toggleSwitches.forEach(toggle => {
            toggle.addEventListener('click', (e) => this.handleToggleClick(e));
        });

        // COM口選擇事件
        this.elements.comSelects.forEach(select => {
            select.addEventListener('change', (e) => this.handleComSelectChange(e));
        });
    }

    initializeSocketEvents() {
        this.socket.on('connect', () => {
            this.addLog('系統', '已連接到啟動工具服務器');
        });

        this.socket.on('status_update', (data) => {
            this.updateAllStatus(data);
        });

        this.socket.on('connected', (data) => {
            this.addLog('系統', data.message);
        });

        this.socket.on('monitoring_started', (data) => {
            this.addLog('系統', data.message);
        });

        this.socket.on('disconnect', () => {
            this.addLog('系統', '與服務器連接中斷');
        });
    }

    handleToggleClick(event) {
        const toggle = event.currentTarget;
        const module = toggle.dataset.module;
        const isActive = toggle.classList.contains('active');
        
        if (isActive) {
            this.stopModule(module);
        } else {
            this.startModule(module);
        }
    }

    handleComSelectChange(event) {
        const select = event.currentTarget;
        const module = select.id.replace('-com', '');
        const comPort = select.value;
        
        if (comPort) {
            this.updateComConfig(module, comPort);
        }
    }

    async controlModbusServer(action) {
        try {
            const response = await fetch(`/api/modbus_server/${action}`, {
                method: 'POST'
            });
            const result = await response.json();
            
            if (result.success) {
                this.addLog('ModbusTCP', `服務器${action === 'start' ? '啟動' : '停止'}成功`);
            } else {
                this.addLog('錯誤', result.message);
            }
        } catch (error) {
            this.addLog('錯誤', `ModbusTCP服務器${action}失敗: ${error.message}`);
        }
    }

    async startModule(moduleName) {
        try {
            const comPort = this.getComPortForModule(moduleName);
            const response = await fetch(`/api/modules/${moduleName}/start`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ com_port: comPort })
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.addLog(moduleName, '啟動成功');
            } else {
                this.addLog('錯誤', `${moduleName} ${result.message}`);
            }
        } catch (error) {
            this.addLog('錯誤', `${moduleName}啟動失敗: ${error.message}`);
        }
    }

    async stopModule(moduleName) {
        try {
            const response = await fetch(`/api/modules/${moduleName}/stop`, {
                method: 'POST'
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.addLog(moduleName, '停止成功');
            } else {
                this.addLog('錯誤', `${moduleName} ${result.message}`);
            }
        } catch (error) {
            this.addLog('錯誤', `${moduleName}停止失敗: ${error.message}`);
        }
    }

    async updateComConfig(moduleName, comPort) {
        try {
            const response = await fetch(`/api/config/${moduleName}/com`, {
                method: 'PUT',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ com_port: comPort })
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.addLog(moduleName, `COM口配置更新為 ${comPort}`);
            } else {
                this.addLog('錯誤', result.message);
            }
        } catch (error) {
            this.addLog('錯誤', `更新COM口配置失敗: ${error.message}`);
        }
    }

    async loadComPorts() {
        try {
            const response = await fetch('/api/com_ports');
            const result = await response.json();
            
            this.updateComPortSelects(result.ports);
        } catch (error) {
            this.addLog('錯誤', `載入COM口列表失敗: ${error.message}`);
        }
    }

    updateComPortSelects(ports) {
        this.elements.comSelects.forEach(select => {
            const currentValue = select.value;
            select.innerHTML = '<option value="">選擇COM口</option>';
            
            ports.forEach(port => {
                const option = document.createElement('option');
                option.value = port;
                option.textContent = port;
                if (port === currentValue) {
                    option.selected = true;
                }
                select.appendChild(option);
            });
        });
    }

    getComPortForModule(moduleName) {
        const modulesNeedingCom = ['Gripper', 'LED', 'XC100'];
        
        if (modulesNeedingCom.includes(moduleName)) {
            const select = document.getElementById(`${moduleName}-com`);
            return select ? select.value : null;
        }
        
        return null;
    }

    updateAllStatus(data) {
        this.currentStatus = data;
        
        // 更新ModbusTCP服務器狀態
        if (data.modbus_server) {
            const isRunning = data.modbus_server.running || data.modbus_server.port_active;
            this.elements.modbusStatusDot.classList.toggle('active', isRunning);
            this.elements.modbusStatusText.textContent = `ModbusTCP服務器 (端口502) - ${isRunning ? '運行中' : '停止'}`;
        }
        
        // 更新主模組狀態
        if (data.modules) {
            Object.entries(data.modules).forEach(([moduleName, status]) => {
                this.updateModuleStatus(moduleName, status.running);
            });
        }
        
        // 更新WebUI狀態
        if (data.web_apps) {
            Object.entries(data.web_apps).forEach(([appName, status]) => {
                this.updateWebAppStatus(appName, status.running, status.port_active);
            });
        }
        
        // 更新COM口列表
        if (data.com_ports) {
            this.updateComPortSelects(data.com_ports);
        }
    }

    updateModuleStatus(moduleName, isRunning) {
        const statusDot = document.getElementById(`${moduleName}-status-dot`);
        const statusText = document.getElementById(`${moduleName}-status`);
        const toggle = document.querySelector(`[data-module="${moduleName}"]`);
        
        if (statusDot) {
            statusDot.classList.toggle('active', isRunning);
        }
        
        if (statusText) {
            statusText.textContent = isRunning ? '運行中' : '停止';
        }
        
        if (toggle) {
            toggle.classList.toggle('active', isRunning);
        }
    }

    updateWebAppStatus(appName, isRunning, portActive) {
        const toggle = document.querySelector(`[data-module="${appName}"]`);
        const moduleName = appName.replace('_app', '');
        const webNav = document.getElementById(`${moduleName}-web-nav`);
        
        if (toggle) {
            toggle.classList.toggle('active', isRunning);
        }
        
        if (webNav) {
            if (portActive) {
                webNav.classList.remove('disabled');
            } else {
                webNav.classList.add('disabled');
            }
        }
    }

    startStatusMonitoring() {
        this.socket.emit('start_monitoring');
    }

    addLog(source, message) {
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = 'log-entry';
        logEntry.innerHTML = `
            <span class="log-timestamp">[${timestamp}]</span>
            <span>[${source}]</span> ${message}
        `;
        
        this.elements.logsContainer.appendChild(logEntry);
        this.elements.logsContainer.scrollTop = this.elements.logsContainer.scrollHeight;
        
        // 限制日誌數量
        while (this.elements.logsContainer.children.length > 100) {
            this.elements.logsContainer.removeChild(this.elements.logsContainer.firstChild);
        }
    }
}

// 初始化應用
document.addEventListener('DOMContentLoaded', function() {
    window.startupTool = new StartupTool();
    
    // 定期重新載入COM口列表
    setInterval(() => {
        window.startupTool.loadComPorts();
    }, 5000);
});