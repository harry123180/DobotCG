// script.js
// 更新：支援無符號 0-65535 範圍

// 全域變數
let autoRefreshInterval = null;
let currentDisplayStart = 0;
let currentDisplayCount = 20;
let currentDisplayFormat = 'decimal';

// 格式化數值顯示
function formatValue(value, format) {
    const num = parseInt(value);
    switch (format) {
        case 'hex':
            return '0x' + num.toString(16).toUpperCase().padStart(4, '0');
        case 'binary':
            return '0b' + num.toString(2).padStart(16, '0');
        case 'signed':
            // 將無符號16位數轉換為有符號 (-32768 to 32767)
            return num > 32767 ? num - 65536 : num;
        case 'decimal':
        default:
            return num.toString();
    }
}

// 格式名稱對應
function getFormatName(format) {
    const formatNames = {
        'decimal': '無符號十進制',
        'hex': '十六進制',
        'binary': '二進制',
        'signed': '有符號十進制'
    };
    return formatNames[format] || '無符號十進制';
}

// 刷新伺服器狀態
function refreshStatus() {
    const statusInfo = document.getElementById('status-info');
    statusInfo.innerHTML = '<div class="loading"></div>載入中...';
    
    fetch('/api/status')
        .then(response => response.json())
        .then(data => {
            const statusIndicator = data.server_running ? 
                '<span class="status-indicator status-running"></span>' : 
                '<span class="status-indicator status-stopped"></span>';
            
            statusInfo.innerHTML = `
                <p><strong>伺服器狀態:</strong> ${statusIndicator}${data.server_running ? '運行中' : '停止'}</p>
                <p><strong>當前 SlaveID:</strong> ${data.slave_id}</p>
                <p><strong>總暫存器數:</strong> ${data.total_registers}</p>
                <p><strong>非零暫存器數:</strong> ${data.non_zero_count}</p>
                <p><strong>數值範圍:</strong> 0 ~ 65535 (無符號16位)</p>
                <p><strong>最後更新:</strong> ${new Date().toLocaleString()}</p>
            `;
            
            // 更新SlaveID輸入框
            document.getElementById('slave-id').value = data.slave_id;
        })
        .catch(error => {
            statusInfo.innerHTML = `<p class="error">❌ 無法獲取狀態: ${error}</p>`;
        });
}

// 更新暫存器顯示
function updateDisplay() {
    currentDisplayStart = parseInt(document.getElementById('display-start').value) || 0;
    currentDisplayCount = parseInt(document.getElementById('display-count').value) || 20;
    currentDisplayFormat = document.getElementById('display-format').value || 'decimal';
    
    // 限制範圍
    currentDisplayStart = Math.max(0, Math.min(999, currentDisplayStart));
    currentDisplayCount = Math.max(1, Math.min(100, currentDisplayCount));
    
    // 更新顯示設定的輸入框
    document.getElementById('display-start').value = currentDisplayStart;
    document.getElementById('display-count').value = currentDisplayCount;
    
    loadRegistersRange();
}

// 載入指定範圍的暫存器
function loadRegistersRange() {
    const grid = document.getElementById('registers-grid');
    grid.innerHTML = '<div class="loading"></div>載入中...';
    
    const url = `/api/register_range?start=${currentDisplayStart}&count=${currentDisplayCount}`;
    
    fetch(url)
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                updateRegistersDisplay(data.registers);
                updateDisplayInfo(data.start_address, data.count);
            } else {
                grid.innerHTML = `<p class="error">載入失敗: ${data.error}</p>`;
            }
        })
        .catch(error => {
            grid.innerHTML = `<p class="error">載入錯誤: ${error}</p>`;
        });
}

// 更新暫存器顯示區域
function updateRegistersDisplay(registers) {
    const grid = document.getElementById('registers-grid');
    
    if (!registers || registers.length === 0) {
        grid.innerHTML = '<p>沒有暫存器數據</p>';
        return;
    }
    
    let html = '';
    registers.forEach(reg => {
        const isNonZero = reg.value !== 0;
        const formattedValue = formatValue(reg.value, currentDisplayFormat);
        
        html += `
            <div class="register-item ${isNonZero ? 'non-zero' : ''}" 
                 data-address="${reg.address}" 
                 onclick="editRegisterValue(${reg.address})">
                <div class="register-address">地址 ${reg.address}</div>
                <div class="register-value" id="value-${reg.address}">${formattedValue}</div>
                <textarea class="register-comment" 
                         placeholder="點擊添加註解..." 
                         data-address="${reg.address}"
                         onclick="event.stopPropagation()"
                         onblur="saveComment(${reg.address})"
                         onkeydown="handleCommentKeydown(event, ${reg.address})">${reg.comment || ''}</textarea>
            </div>
        `;
    });
    
    grid.innerHTML = html;
}

// 更新顯示資訊
function updateDisplayInfo(startAddress, count) {
    const endAddress = startAddress + count - 1;
    document.getElementById('address-range').textContent = `${startAddress}-${endAddress}`;
    document.getElementById('current-format').textContent = getFormatName(currentDisplayFormat);
}

// 編輯暫存器值
function editRegisterValue(address) {
    const valueElement = document.getElementById(`value-${address}`);
    const registerItem = valueElement.closest('.register-item');
    
    // 創建輸入框 - 更新為無符號範圍
    const input = document.createElement('input');
    input.type = 'number';
    input.className = 'quick-edit-input';
    input.min = '0';
    input.max = '65535';
    input.value = getCurrentRegisterValue(address);
    
    // 添加輸入框
    registerItem.appendChild(input);
    registerItem.classList.add('editing');
    input.focus();
    input.select();
    
    // 處理按鍵事件
    input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
            saveRegisterValue(address, input.value);
            removeEditInput(registerItem, input);
        } else if (e.key === 'Escape') {
            removeEditInput(registerItem, input);
        }
    });
    
    // 處理失去焦點
    input.addEventListener('blur', () => {
        saveRegisterValue(address, input.value);
        removeEditInput(registerItem, input);
    });
}

// 移除編輯輸入框
function removeEditInput(registerItem, input) {
    registerItem.classList.remove('editing');
    if (input.parentNode) {
        input.parentNode.removeChild(input);
    }
}

// 獲取當前暫存器值 (從格式化顯示中反推原始值)
function getCurrentRegisterValue(address) {
    const valueElement = document.getElementById(`value-${address}`);
    const displayValue = valueElement.textContent;
    
    // 根據當前格式解析值
    if (currentDisplayFormat === 'hex') {
        return parseInt(displayValue.replace('0x', ''), 16);
    } else if (currentDisplayFormat === 'binary') {
        return parseInt(displayValue.replace('0b', ''), 2);
    } else if (currentDisplayFormat === 'signed') {
        // 從有符號轉回無符號
        const signedValue = parseInt(displayValue);
        return signedValue < 0 ? signedValue + 65536 : signedValue;
    } else {
        return parseInt(displayValue);
    }
}

// 保存暫存器值 - 更新為無符號範圍
function saveRegisterValue(address, value) {
    const numValue = parseInt(value);
    if (isNaN(numValue) || numValue < 0 || numValue > 65535) {
        showMessage('❌ 無效的數值範圍 (0 ~ 65535)', 'error');
        return;
    }
    
    fetch(`/api/register/${address}`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({value: numValue})
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 更新顯示
            const valueElement = document.getElementById(`value-${address}`);
            valueElement.textContent = formatValue(numValue, currentDisplayFormat);
            
            // 更新樣式
            const registerItem = valueElement.closest('.register-item');
            if (numValue === 0) {
                registerItem.classList.remove('non-zero');
            } else {
                registerItem.classList.add('non-zero');
            }
            
            showMessage(`✅ 暫存器 ${address} 已更新為: ${numValue}`, 'success');
        } else {
            showMessage(`❌ 更新失敗: ${data.error}`, 'error');
        }
    })
    .catch(error => {
        showMessage(`❌ 請求失敗: ${error}`, 'error');
    });
}

// 保存註解
function saveComment(address) {
    const commentElement = document.querySelector(`textarea[data-address="${address}"]`);
    const comment = commentElement.value.trim();
    
    fetch(`/api/comment/${address}`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({comment: comment})
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            if (comment) {
                showMessage(`✅ 暫存器 ${address} 註解已保存`, 'success');
            }
        } else {
            showMessage(`❌ 註解保存失敗: ${data.error}`, 'error');
        }
    })
    .catch(error => {
        showMessage(`❌ 註解保存錯誤: ${error}`, 'error');
    });
}

// 處理註解輸入框按鍵
function handleCommentKeydown(event, address) {
    if (event.key === 'Enter' && !event.shiftKey) {
        event.preventDefault();
        event.target.blur(); // 觸發保存
    }
}

// 更新SlaveID
function updateSlaveId() {
    const slaveId = parseInt(document.getElementById('slave-id').value);
    
    fetch('/api/slave_id', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({slave_id: slaveId})
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            showMessage(`✅ SlaveID 已更新為: ${data.slave_id}`, 'success');
            refreshStatus();
        } else {
            showMessage(`❌ 更新失敗: ${data.error}`, 'error');
        }
    })
    .catch(error => showMessage(`❌ 請求失敗: ${error}`, 'error'));
}

// 寫入暫存器 (控制面板) - 更新為無符號範圍
function writeRegister() {
    const address = parseInt(document.getElementById('reg-address').value);
    const value = parseInt(document.getElementById('reg-value').value);
    
    // 檢查無符號範圍
    if (isNaN(value) || value < 0 || value > 65535) {
        showMessage('❌ 數值必須在 0-65535 範圍內', 'error');
        return;
    }
    
    fetch(`/api/register/${address}`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({value: value})
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            showMessage(`✅ 暫存器 ${address} 已設為: ${value}`, 'success');
            // 如果在當前顯示範圍內，更新顯示
            if (address >= currentDisplayStart && address < currentDisplayStart + currentDisplayCount) {
                loadRegistersRange();
            }
        } else {
            showMessage(`❌ 寫入失敗: ${data.error}`, 'error');
        }
    })
    .catch(error => showMessage(`❌ 請求失敗: ${error}`, 'error'));
}

// 讀取暫存器 (控制面板)
function readRegister() {
    const address = parseInt(document.getElementById('reg-address').value);
    
    fetch(`/api/register/${address}`)
        .then(response => response.json())
        .then(data => {
            if (data.address !== undefined) {
                showMessage(`📖 暫存器 ${data.address} 的值: ${data.value}`, 'success');
                document.getElementById('reg-value').value = data.value;
            } else {
                showMessage(`❌ 讀取失敗: ${data.error}`, 'error');
            }
        })
        .catch(error => showMessage(`❌ 請求失敗: ${error}`, 'error'));
}

// 切換自動刷新
function toggleAutoRefresh() {
    const button = document.getElementById('auto-refresh-btn');
    
    if (autoRefreshInterval) {
        // 停止自動刷新
        clearInterval(autoRefreshInterval);
        autoRefreshInterval = null;
        button.textContent = '🔄 開啟自動刷新';
        button.classList.remove('auto-refresh-active');
    } else {
        // 開始自動刷新
        autoRefreshInterval = setInterval(() => {
            loadRegistersRange();
            refreshStatus();
        }, 3000); // 每3秒刷新一次
        
        button.textContent = '⏸️ 停止自動刷新';
        button.classList.add('auto-refresh-active');
    }
}

// 清除所有暫存器
function clearAllRegisters() {
    if (!confirm('確定要清除所有暫存器的值嗎？此操作無法復原。')) {
        return;
    }
    
    const values = new Array(1000).fill(0);
    
    fetch('/api/registers', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({start_address: 0, values: values})
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            showMessage('✅ 所有暫存器已清除', 'success');
            loadRegistersRange();
        } else {
            showMessage(`❌ 清除失敗: ${data.error}`, 'error');
        }
    })
    .catch(error => showMessage(`❌ 請求失敗: ${error}`, 'error'));
}

// 設定測試數據 - 更新為無符號範圍
function setTestData() {
    const testData = [
        {address: 0, value: 100},
        {address: 1, value: 200},
        {address: 10, value: 1000},
        {address: 50, value: 5000},
        {address: 100, value: 12345},
        {address: 200, value: 32768},   // 超過有符號範圍但在無符號範圍內
        {address: 500, value: 65535},   // 最大無符號值
        {address: 999, value: 40000}    // 無符號範圍內的高值
    ];
    
    Promise.all(testData.map(item => 
        fetch(`/api/register/${item.address}`, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({value: item.value})
        })
    ))
    .then(() => {
        showMessage('✅ 測試數據已設定 (無符號 0-65535)', 'success');
        loadRegistersRange();
    })
    .catch(error => {
        showMessage(`❌ 設定測試數據失敗: ${error}`, 'error');
    });
}

// 匯出暫存器數據
function exportRegisters() {
    fetch('/api/status')
        .then(response => response.json())
        .then(data => {
            const exportData = {
                timestamp: new Date().toISOString(),
                slave_id: data.slave_id,
                value_range: "0-65535 (unsigned 16-bit)",
                registers: data.non_zero_registers,
                comments: {} // 需要從當前頁面收集註解
            };
            
            // 收集當前顯示的註解
            document.querySelectorAll('.register-comment').forEach(textarea => {
                const address = textarea.dataset.address;
                const comment = textarea.value.trim();
                if (comment) {
                    exportData.comments[address] = comment;
                }
            });
            
            const blob = new Blob([JSON.stringify(exportData, null, 2)], {type: 'application/json'});
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `modbus_registers_unsigned_${new Date().toISOString().slice(0,19).replace(/:/g, '-')}.json`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
            
            showMessage('✅ 暫存器數據已匯出 (無符號格式)', 'success');
        })
        .catch(error => {
            showMessage(`❌ 匯出失敗: ${error}`, 'error');
        });
}

// 匯入暫存器數據
function importRegisters() {
    const fileInput = document.getElementById('import-file');
    const file = fileInput.files[0];
    
    if (!file) return;
    
    const reader = new FileReader();
    reader.onload = function(e) {
        try {
            const data = JSON.parse(e.target.result);
            
            if (!data.registers) {
                showMessage('❌ 無效的匯入檔案格式', 'error');
                return;
            }
            
            // 匯入暫存器值 - 檢查無符號範圍
            const promises = [];
            for (const [address, value] of Object.entries(data.registers)) {
                const numValue = parseInt(value);
                if (numValue >= 0 && numValue <= 65535) {
                    promises.push(
                        fetch(`/api/register/${address}`, {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({value: numValue})
                        })
                    );
                } else {
                    showMessage(`⚠️ 跳過超出範圍的值: 地址${address} = ${value}`, 'warning');
                }
            }
            
            // 匯入註解
            if (data.comments) {
                for (const [address, comment] of Object.entries(data.comments)) {
                    promises.push(
                        fetch(`/api/comment/${address}`, {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({comment: comment})
                        })
                    );
                }
            }
            
            Promise.all(promises)
                .then(() => {
                    showMessage('✅ 暫存器數據已匯入 (無符號格式)', 'success');
                    loadRegistersRange();
                })
                .catch(error => {
                    showMessage(`❌ 匯入過程中發生錯誤: ${error}`, 'error');
                });
                
        } catch (error) {
            showMessage(`❌ 檔案解析失敗: ${error}`, 'error');
        }
    };
    
    reader.readAsText(file);
    fileInput.value = ''; // 清空檔案選擇
}

// 顯示訊息
function showMessage(message, type) {
    const msgDiv = document.getElementById('result-message');
    msgDiv.innerHTML = `<p class="${type}">${message}</p>`;
    setTimeout(() => msgDiv.innerHTML = '', 5000);
}

// 頁面載入時的初始化
window.onload = function() {
    // 初始化顯示設定
    updateDisplay();
    refreshStatus();
    
    // 設定事件監聽器
    document.getElementById('display-start').addEventListener('change', updateDisplay);
    document.getElementById('display-count').addEventListener('change', updateDisplay);
    document.getElementById('display-format').addEventListener('change', updateDisplay);
    
    // 每30秒自動刷新狀態 (不包括暫存器數據)
    setInterval(refreshStatus, 30000);
};