/**
 * EL-A3 Web UI - Control Panel
 * Joint control sliders and parameter adjustment
 */

class ControlPanel {
    constructor() {
        this.jointNames = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint'];
        this.jointLabels = ['L1', 'L2', 'L3', 'L4', 'L5', 'L6'];
        
        // Default limits (will be updated from server)
        this.jointLimits = {
            'L1_joint': { lower: -160, upper: 160 },
            'L2_joint': { lower: -180, upper: 180 },
            'L3_joint': { lower: -180, upper: 180 },
            'L4_joint': { lower: -180, upper: 180 },
            'L5_joint': { lower: -180, upper: 180 },
            'L6_joint': { lower: -180, upper: 180 }
        };
        
        // Current target positions (degrees)
        this.targetPositions = [0, 0, 0, 0, 0, 0];
        
        // Current actual positions (from robot)
        this.actualPositions = [0, 0, 0, 0, 0, 0];
        
        // Preset positions (degrees)
        this.presets = {
            home: [0, 0, 0, 0, 0, 0],
            preset1: [0, -45, 90, 0, 45, 0],
            preset2: [45, 0, 45, 0, 0, 90]
        };
        
        this.init();
    }
    
    init() {
        // Wait for joint limits from server
        this.loadJointLimits();
        
        // Create sliders
        this.createJointSliders();
        
        // Setup preset buttons
        this.setupPresetButtons();
        
        // Setup zero torque toggle
        this.setupZeroTorqueToggle();
        
        // Setup teleop control
        this.setupTeleopControl();
        
        // Setup CAN interface config
        this.setupCANConfig();
        
        // Setup log controls
        this.setupLogControls();
        
        // Listen for state updates
        window.addEventListener('robotStateUpdate', (e) => this.updateFromState(e.detail));
    }
    
    async loadJointLimits() {
        try {
            const response = await fetch('/api/joint_limits');
            const limits = await response.json();
            
            // Convert radians to degrees
            for (const [joint, lim] of Object.entries(limits)) {
                this.jointLimits[joint] = {
                    lower: radToDeg(lim.lower),
                    upper: radToDeg(lim.upper)
                };
            }
            
            // Recreate sliders with new limits
            this.createJointSliders();
        } catch (error) {
            console.error('Failed to load joint limits:', error);
        }
    }
    
    createJointSliders() {
        const container = document.getElementById('jointSliders');
        if (!container) return;
        
        container.innerHTML = '';
        
        this.jointNames.forEach((name, i) => {
            const limits = this.jointLimits[name];
            const label = this.jointLabels[i];
            
            const sliderDiv = document.createElement('div');
            sliderDiv.className = 'joint-slider';
            sliderDiv.innerHTML = `
                <div class="slider-header">
                    <span class="joint-label">${label}</span>
                    <span class="joint-value" id="value-${name}">0.0°</span>
                </div>
                <div class="slider-row">
                    <span class="limit-label">${limits.lower.toFixed(0)}°</span>
                    <input type="range" 
                           id="slider-${name}" 
                           min="${limits.lower}" 
                           max="${limits.upper}" 
                           value="0" 
                           step="0.5"
                           class="joint-range">
                    <span class="limit-label">${limits.upper.toFixed(0)}°</span>
                </div>
                <div class="slider-input-row">
                    <input type="number" 
                           id="input-${name}" 
                           value="0" 
                           min="${limits.lower}" 
                           max="${limits.upper}" 
                           step="0.5"
                           class="joint-input">
                    <button class="btn btn-small btn-send" data-joint="${i}">发送</button>
                </div>
            `;
            
            container.appendChild(sliderDiv);
            
            // Setup event listeners
            const slider = sliderDiv.querySelector(`#slider-${name}`);
            const input = sliderDiv.querySelector(`#input-${name}`);
            const valueDisplay = sliderDiv.querySelector(`#value-${name}`);
            const sendBtn = sliderDiv.querySelector('.btn-send');
            
            // Slider change
            slider.addEventListener('input', () => {
                const value = parseFloat(slider.value);
                this.targetPositions[i] = value;
                input.value = value.toFixed(1);
                valueDisplay.textContent = `${value.toFixed(1)}°`;
            });
            
            // Slider release - send command
            slider.addEventListener('change', () => {
                this.sendSingleJointCommand(i);
            });
            
            // Input change
            input.addEventListener('change', () => {
                let value = parseFloat(input.value);
                value = Math.max(limits.lower, Math.min(limits.upper, value));
                input.value = value.toFixed(1);
                slider.value = value;
                this.targetPositions[i] = value;
                valueDisplay.textContent = `${value.toFixed(1)}°`;
            });
            
            // Send button
            sendBtn.addEventListener('click', () => {
                this.sendSingleJointCommand(i);
            });
        });
    }
    
    sendSingleJointCommand(jointIndex) {
        const positionRad = degToRad(this.targetPositions[jointIndex]);
        const duration = parseFloat(document.getElementById('motionDuration')?.value || 2.0);
        
        sendCommand('set_single_joint', {
            joint_index: jointIndex,
            position: positionRad,
            duration: duration
        });
        
        addLog(`发送关节 ${this.jointLabels[jointIndex]} 命令: ${this.targetPositions[jointIndex].toFixed(1)}°`, 'info');
    }
    
    sendAllJointsCommand() {
        const positionsRad = this.targetPositions.map(deg => degToRad(deg));
        const duration = parseFloat(document.getElementById('motionDuration')?.value || 2.0);
        
        sendCommand('set_joints', {
            positions: positionsRad,
            duration: duration
        });
        
        addLog(`发送全部关节命令`, 'info');
    }
    
    setupPresetButtons() {
        // Go home button
        const homeBtn = document.getElementById('goHomeBtn');
        if (homeBtn) {
            homeBtn.addEventListener('click', () => {
                const duration = parseFloat(document.getElementById('motionDuration')?.value || 3.0);
                sendCommand('go_home', { duration: duration });
                
                // Update sliders to home position
                this.targetPositions = [...this.presets.home];
                this.updateSliders();
                
                addLog('执行回原点', 'info');
            });
        }
        
        // Preset 1 button
        const preset1Btn = document.getElementById('preset1Btn');
        if (preset1Btn) {
            preset1Btn.addEventListener('click', () => {
                this.targetPositions = [...this.presets.preset1];
                this.updateSliders();
                this.sendAllJointsCommand();
                addLog('执行预设位姿1', 'info');
            });
        }
        
        // Preset 2 button
        const preset2Btn = document.getElementById('preset2Btn');
        if (preset2Btn) {
            preset2Btn.addEventListener('click', () => {
                this.targetPositions = [...this.presets.preset2];
                this.updateSliders();
                this.sendAllJointsCommand();
                addLog('执行预设位姿2', 'info');
            });
        }
    }
    
    setupZeroTorqueToggle() {
        const toggle = document.getElementById('zeroTorqueToggle');
        if (toggle) {
            toggle.addEventListener('change', () => {
                const enabled = toggle.checked;
                sendCommand('set_zero_torque', { enable: enabled });
                
                if (enabled) {
                    addLog('启用零力矩模式 - 可手动拖动机械臂', 'warn');
                } else {
                    addLog('禁用零力矩模式 - 恢复位置控制', 'info');
                }
            });
        }
    }
    
    setupTeleopControl() {
        const toggle = document.getElementById('teleopToggle');
        if (toggle) {
            toggle.addEventListener('change', () => {
                const enabled = toggle.checked;
                
                if (enabled) {
                    sendCommand('start_teleop');
                    addLog('正在启动手柄控制...', 'info');
                } else {
                    sendCommand('stop_teleop');
                    addLog('正在停止手柄控制...', 'info');
                }
            });
        }
    }
    
    setupCANConfig() {
        // Load available CAN interfaces
        this.loadCANInterfaces();
        
        // Apply button
        const applyBtn = document.getElementById('applyCanBtn');
        if (applyBtn) {
            applyBtn.addEventListener('click', () => {
                const select = document.getElementById('canInterfaceSelect');
                if (select) {
                    const iface = select.value;
                    
                    // Confirm before applying
                    if (confirm(`确定要将CAN接口切换为 ${iface} 吗？\n这将重启硬件驱动，机械臂会短暂失控。`)) {
                        sendCommand('set_can_interface', { interface: iface });
                        addLog(`正在切换CAN接口到 ${iface}...`, 'warn');
                    }
                }
            });
        }
    }
    
    async loadCANInterfaces() {
        try {
            const response = await fetch('/api/can_interfaces');
            const data = await response.json();
            
            const select = document.getElementById('canInterfaceSelect');
            if (select && data.interfaces) {
                select.innerHTML = '';
                
                data.interfaces.forEach(iface => {
                    const option = document.createElement('option');
                    option.value = iface;
                    option.textContent = iface;
                    if (iface === data.current) {
                        option.selected = true;
                    }
                    select.appendChild(option);
                });
            }
        } catch (error) {
            console.error('Failed to load CAN interfaces:', error);
        }
    }
    
    setupLogControls() {
        // Clear logs button
        const clearBtn = document.getElementById('clearLogsBtn');
        if (clearBtn) {
            clearBtn.addEventListener('click', () => {
                const container = document.getElementById('logContainer');
                if (container) {
                    container.innerHTML = '';
                    addLog('日志已清除', 'info');
                }
            });
        }
        
        // Export logs button
        const exportBtn = document.getElementById('exportLogsBtn');
        if (exportBtn) {
            exportBtn.addEventListener('click', () => {
                this.exportLogs();
            });
        }
    }
    
    exportLogs() {
        const container = document.getElementById('logContainer');
        if (!container) return;
        
        const entries = container.querySelectorAll('.log-entry');
        let logText = 'EL-A3 操作日志\n';
        logText += '导出时间: ' + new Date().toLocaleString('zh-CN') + '\n';
        logText += '=' .repeat(50) + '\n\n';
        
        entries.forEach(entry => {
            logText += entry.textContent + '\n';
        });
        
        // Create download
        const blob = new Blob([logText], { type: 'text/plain;charset=utf-8' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `el_a3_log_${new Date().toISOString().slice(0,10)}.txt`;
        a.click();
        URL.revokeObjectURL(url);
        
        addLog('日志已导出', 'info');
    }
    
    updateFromState(state) {
        if (!state || !state.joints) return;
        
        state.joints.forEach((joint, i) => {
            this.actualPositions[i] = radToDeg(joint.position);
        });
        
        // Optionally sync sliders to actual position
        // (commented out to allow independent target setting)
        // this.targetPositions = [...this.actualPositions];
        // this.updateSliders();
    }
    
    updateSliders() {
        this.jointNames.forEach((name, i) => {
            const slider = document.getElementById(`slider-${name}`);
            const input = document.getElementById(`input-${name}`);
            const valueDisplay = document.getElementById(`value-${name}`);
            
            if (slider && input && valueDisplay) {
                const value = this.targetPositions[i];
                slider.value = value;
                input.value = value.toFixed(1);
                valueDisplay.textContent = `${value.toFixed(1)}°`;
            }
        });
    }
    
    syncToActual() {
        // Sync target positions to actual robot positions
        this.targetPositions = [...this.actualPositions];
        this.updateSliders();
        addLog('已同步到当前位置', 'info');
    }
}

// Initialize control panel
let controlPanel = null;

document.addEventListener('DOMContentLoaded', () => {
    controlPanel = new ControlPanel();
});

window.controlPanel = controlPanel;
