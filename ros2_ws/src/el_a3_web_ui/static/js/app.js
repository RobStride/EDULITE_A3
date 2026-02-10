/**
 * EL-A3 Web UI - Main Application
 * Handles WebSocket communication and global state management
 */

// Global state
const AppState = {
    connected: false,
    robotState: null,
    jointLimits: {},
    lastUpdateTime: 0,
    updateCount: 0,
    updateRateHz: 0,
    motorsEnabled: false,
    teleopStatus: 'stopped',
    canInterface: 'can0'
};

// Socket.IO connection
let socket = null;

/**
 * Initialize the application
 */
function initApp() {
    console.log('EL-A3 Web UI initializing...');
    
    // Initialize WebSocket connection
    initWebSocket();
    
    // Initialize UI components
    initTabs();
    initEmergencyStop();
    initMotorEnableButton();
    
    // Load joint limits
    loadJointLimits();
    
    // Update rate calculation
    setInterval(calculateUpdateRate, 1000);
}

/**
 * Initialize WebSocket connection
 */
function initWebSocket() {
    socket = io({
        transports: ['websocket', 'polling'],
        reconnection: true,
        reconnectionAttempts: Infinity,
        reconnectionDelay: 1000
    });
    
    socket.on('connect', () => {
        console.log('WebSocket connected');
        AppState.connected = true;
        updateConnectionStatus(true);
        addLog('已连接到服务器', 'info');
    });
    
    socket.on('disconnect', () => {
        console.log('WebSocket disconnected');
        AppState.connected = false;
        updateConnectionStatus(false);
        addLog('与服务器断开连接', 'warn');
    });
    
    socket.on('state', (data) => {
        AppState.robotState = data;
        AppState.lastUpdateTime = Date.now();
        AppState.updateCount++;
        
        // Update connection status based on robot status
        updateConnectionStatus(data.status === 'connected');
        
        // Update motor enable state
        if (data.motors_enabled !== undefined) {
            AppState.motorsEnabled = data.motors_enabled;
            updateMotorEnableButton(data.motors_enabled);
        }
        
        // Update teleop status
        if (data.teleop_status !== undefined) {
            AppState.teleopStatus = data.teleop_status;
            updateTeleopStatus(data.teleop_status);
        }
        
        // Update CAN interface
        if (data.can_interface !== undefined) {
            AppState.canInterface = data.can_interface;
        }
        
        // Dispatch state update event
        window.dispatchEvent(new CustomEvent('robotStateUpdate', { detail: data }));
    });
    
    socket.on('log', (data) => {
        addLog(data.message, data.level);
    });
    
    socket.on('command_result', (data) => {
        if (data.success) {
            console.log(`Command ${data.type} succeeded`);
            
            // Handle specific command results
            if (data.type === 'enable_motors' || data.type === 'disable_motors') {
                if (data.motors_enabled !== undefined) {
                    AppState.motorsEnabled = data.motors_enabled;
                    updateMotorEnableButton(data.motors_enabled);
                }
            }
            
            if (data.type === 'start_teleop' || data.type === 'stop_teleop') {
                if (data.teleop_status !== undefined) {
                    AppState.teleopStatus = data.teleop_status;
                    updateTeleopStatus(data.teleop_status);
                    // Update toggle checkbox
                    const toggle = document.getElementById('teleopToggle');
                    if (toggle) {
                        toggle.checked = data.teleop_status === 'running';
                    }
                }
            }
            
            if (data.type === 'set_can_interface') {
                if (data.can_interface !== undefined) {
                    AppState.canInterface = data.can_interface;
                    addLog(`CAN接口已设置为 ${data.can_interface}`, 'info');
                }
            }
        } else {
            console.error(`Command ${data.type} failed`);
            addLog(`命令执行失败: ${data.type}`, 'error');
        }
    });
    
    socket.on('error', (data) => {
        console.error('Socket error:', data);
        addLog(`错误: ${data.message}`, 'error');
    });
}

/**
 * Send command to robot
 */
function sendCommand(type, params = {}) {
    if (!socket || !AppState.connected) {
        addLog('未连接到服务器', 'error');
        return false;
    }
    
    socket.emit('command', { type, ...params });
    return true;
}

/**
 * Update connection status display
 */
function updateConnectionStatus(connected) {
    const statusEl = document.getElementById('connectionStatus');
    const dot = statusEl.querySelector('.status-dot');
    const text = statusEl.querySelector('.status-text');
    
    if (connected) {
        dot.classList.remove('disconnected');
        dot.classList.add('connected');
        text.textContent = '已连接';
    } else {
        dot.classList.remove('connected');
        dot.classList.add('disconnected');
        text.textContent = '未连接';
    }
}

/**
 * Calculate and display update rate
 */
function calculateUpdateRate() {
    AppState.updateRateHz = AppState.updateCount;
    AppState.updateCount = 0;
    
    const rateEl = document.getElementById('updateRate');
    rateEl.textContent = `更新频率: ${AppState.updateRateHz} Hz`;
}

/**
 * Load joint limits from server
 */
async function loadJointLimits() {
    try {
        const response = await fetch('/api/joint_limits');
        AppState.jointLimits = await response.json();
        console.log('Joint limits loaded:', AppState.jointLimits);
    } catch (error) {
        console.error('Failed to load joint limits:', error);
    }
}

/**
 * Initialize tab navigation
 */
function initTabs() {
    const tabBtns = document.querySelectorAll('.tab-btn');
    const tabContents = document.querySelectorAll('.tab-content');
    
    tabBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            const targetTab = btn.dataset.tab;
            
            // Update buttons
            tabBtns.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            
            // Update content
            tabContents.forEach(content => {
                content.classList.remove('active');
                if (content.id === `tab-${targetTab}`) {
                    content.classList.add('active');
                }
            });
        });
    });
}

/**
 * Initialize emergency stop button
 */
function initEmergencyStop() {
    const btn = document.getElementById('emergencyStopBtn');
    btn.addEventListener('click', () => {
        sendCommand('emergency_stop');
        addLog('紧急停止已触发', 'warn');
    });
}

/**
 * Initialize motor enable button
 */
function initMotorEnableButton() {
    const btn = document.getElementById('enableMotorsBtn');
    if (!btn) return;
    
    btn.addEventListener('click', () => {
        if (AppState.motorsEnabled) {
            // Disable motors
            sendCommand('disable_motors');
            addLog('正在失能电机...', 'info');
        } else {
            // Enable motors
            sendCommand('enable_motors');
            addLog('正在使能电机...', 'info');
        }
    });
}

/**
 * Update motor enable button display
 */
function updateMotorEnableButton(enabled) {
    const btn = document.getElementById('enableMotorsBtn');
    if (!btn) return;
    
    const textSpan = btn.querySelector('.btn-text');
    
    if (enabled) {
        btn.classList.remove('btn-success');
        btn.classList.add('btn-warning');
        if (textSpan) textSpan.textContent = '已使能';
        btn.title = '点击失能电机';
    } else {
        btn.classList.remove('btn-warning');
        btn.classList.add('btn-success');
        if (textSpan) textSpan.textContent = '使能电机';
        btn.title = '点击使能电机';
    }
}

/**
 * Update teleop status display
 */
function updateTeleopStatus(status) {
    const statusEl = document.getElementById('teleopStatus');
    if (!statusEl) return;
    
    const indicator = statusEl.querySelector('.status-indicator');
    const textSpan = statusEl.querySelector('span:last-child');
    
    if (indicator) {
        indicator.classList.remove('running', 'stopped', 'error');
        indicator.classList.add(status);
    }
    
    let statusText = '未启动';
    if (status === 'running') {
        statusText = '运行中';
    } else if (status === 'error') {
        statusText = '错误';
    }
    
    if (textSpan) {
        textSpan.textContent = `状态: ${statusText}`;
    }
}

/**
 * Add log entry to the log container
 */
function addLog(message, level = 'info') {
    const container = document.getElementById('logContainer');
    if (!container) return;
    
    const timestamp = new Date().toLocaleTimeString('zh-CN');
    const entry = document.createElement('div');
    entry.className = `log-entry log-${level}`;
    entry.innerHTML = `<span class="log-time">[${timestamp}]</span> ${message}`;
    
    container.appendChild(entry);
    
    // Auto scroll to bottom
    container.scrollTop = container.scrollHeight;
    
    // Limit log entries
    while (container.children.length > 100) {
        container.removeChild(container.firstChild);
    }
}

/**
 * Convert radians to degrees
 */
function radToDeg(rad) {
    return rad * 180 / Math.PI;
}

/**
 * Convert degrees to radians
 */
function degToRad(deg) {
    return deg * Math.PI / 180;
}

/**
 * Format number for display
 */
function formatNumber(num, decimals = 2) {
    return Number(num).toFixed(decimals);
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', initApp);

// Export for other modules
window.AppState = AppState;
window.sendCommand = sendCommand;
window.addLog = addLog;
window.radToDeg = radToDeg;
window.degToRad = degToRad;
window.formatNumber = formatNumber;
