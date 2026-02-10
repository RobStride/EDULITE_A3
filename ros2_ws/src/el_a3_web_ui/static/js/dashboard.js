/**
 * EL-A3 Web UI - Dashboard
 * Status monitoring and real-time charts
 */

class Dashboard {
    constructor() {
        this.temperatureChart = null;
        this.effortChart = null;
        this.historyData = {
            timestamps: [],
            temperatures: [[], [], [], [], [], []],
            efforts: [[], [], [], [], [], []]
        };
        this.maxDataPoints = 100;
        
        this.jointNames = ['L1', 'L2', 'L3', 'L4', 'L5', 'L6'];
        this.jointColors = [
            'rgba(255, 99, 132, 1)',   // L1 - Red
            'rgba(54, 162, 235, 1)',   // L2 - Blue
            'rgba(75, 192, 192, 1)',   // L3 - Teal
            'rgba(255, 206, 86, 1)',   // L4 - Yellow
            'rgba(153, 102, 255, 1)',  // L5 - Purple
            'rgba(255, 159, 64, 1)'    // L6 - Orange
        ];
        
        this.init();
    }
    
    init() {
        this.initCharts();
        
        // Listen for state updates
        window.addEventListener('robotStateUpdate', (e) => this.updateDashboard(e.detail));
    }
    
    initCharts() {
        // Temperature chart
        const tempCtx = document.getElementById('temperatureChart');
        if (tempCtx) {
            this.temperatureChart = new Chart(tempCtx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: this.jointNames.map((name, i) => ({
                        label: name,
                        data: [],
                        borderColor: this.jointColors[i],
                        backgroundColor: this.jointColors[i].replace('1)', '0.1)'),
                        borderWidth: 2,
                        pointRadius: 0,
                        tension: 0.3,
                        fill: false
                    }))
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    animation: false,
                    scales: {
                        x: {
                            display: true,
                            title: {
                                display: false
                            },
                            ticks: {
                                color: '#888',
                                maxTicksLimit: 5
                            },
                            grid: {
                                color: 'rgba(255,255,255,0.1)'
                            }
                        },
                        y: {
                            display: true,
                            title: {
                                display: true,
                                text: '温度 (°C)',
                                color: '#888'
                            },
                            ticks: {
                                color: '#888'
                            },
                            grid: {
                                color: 'rgba(255,255,255,0.1)'
                            },
                            suggestedMin: 20,
                            suggestedMax: 80
                        }
                    },
                    plugins: {
                        legend: {
                            display: true,
                            position: 'top',
                            labels: {
                                color: '#ccc',
                                usePointStyle: true,
                                pointStyle: 'circle',
                                boxWidth: 6
                            }
                        }
                    }
                }
            });
        }
        
        // Effort chart
        const effCtx = document.getElementById('effortChart');
        if (effCtx) {
            this.effortChart = new Chart(effCtx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: this.jointNames.map((name, i) => ({
                        label: name,
                        data: [],
                        borderColor: this.jointColors[i],
                        backgroundColor: this.jointColors[i].replace('1)', '0.1)'),
                        borderWidth: 2,
                        pointRadius: 0,
                        tension: 0.3,
                        fill: false
                    }))
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    animation: false,
                    scales: {
                        x: {
                            display: true,
                            ticks: {
                                color: '#888',
                                maxTicksLimit: 5
                            },
                            grid: {
                                color: 'rgba(255,255,255,0.1)'
                            }
                        },
                        y: {
                            display: true,
                            title: {
                                display: true,
                                text: '力矩 (Nm)',
                                color: '#888'
                            },
                            ticks: {
                                color: '#888'
                            },
                            grid: {
                                color: 'rgba(255,255,255,0.1)'
                            }
                        }
                    },
                    plugins: {
                        legend: {
                            display: true,
                            position: 'top',
                            labels: {
                                color: '#ccc',
                                usePointStyle: true,
                                pointStyle: 'circle',
                                boxWidth: 6
                            }
                        }
                    }
                }
            });
        }
    }
    
    updateDashboard(state) {
        if (!state || !state.joints) return;
        
        // Update joint state table
        this.updateStateTable(state.joints);
        
        // Update history data
        this.updateHistoryData(state);
        
        // Update charts
        this.updateCharts();
    }
    
    updateStateTable(joints) {
        const table = document.getElementById('jointStateTable');
        if (!table) return;
        
        const tbody = table.querySelector('tbody');
        if (!tbody) return;
        
        joints.forEach((joint, i) => {
            const row = tbody.rows[i];
            if (row) {
                row.cells[0].textContent = this.jointNames[i];
                row.cells[1].textContent = formatNumber(radToDeg(joint.position), 1);
                row.cells[2].textContent = formatNumber(joint.velocity, 3);
                row.cells[3].textContent = formatNumber(joint.effort, 2);
                
                // Temperature display with color coding
                const temp = joint.temperature || 0;
                row.cells[4].textContent = formatNumber(temp, 1);
                
                // Temperature warning colors
                if (temp > 60) {
                    row.cells[4].classList.add('temp-danger');
                    row.cells[4].classList.remove('temp-warning');
                } else if (temp > 45) {
                    row.cells[4].classList.add('temp-warning');
                    row.cells[4].classList.remove('temp-danger');
                } else {
                    row.cells[4].classList.remove('temp-warning', 'temp-danger');
                }
                
                // Color code based on limit proximity
                const pos = joint.position;
                const limits = joint.limits;
                const range = limits.upper - limits.lower;
                const margin = range * 0.1;
                
                if (pos < limits.lower + margin || pos > limits.upper - margin) {
                    row.classList.add('warning');
                } else {
                    row.classList.remove('warning');
                }
            }
        });
    }
    
    updateHistoryData(state) {
        // Add timestamp
        const now = new Date();
        const timeStr = now.toLocaleTimeString('zh-CN', { 
            hour12: false, 
            hour: '2-digit', 
            minute: '2-digit', 
            second: '2-digit' 
        });
        
        this.historyData.timestamps.push(timeStr);
        
        // Add joint data
        state.joints.forEach((joint, i) => {
            this.historyData.temperatures[i].push(joint.temperature || 0);
            this.historyData.efforts[i].push(joint.effort);
        });
        
        // Trim to max length
        if (this.historyData.timestamps.length > this.maxDataPoints) {
            this.historyData.timestamps.shift();
            this.historyData.temperatures.forEach(arr => arr.shift());
            this.historyData.efforts.forEach(arr => arr.shift());
        }
    }
    
    updateCharts() {
        // Update temperature chart
        if (this.temperatureChart) {
            this.temperatureChart.data.labels = this.historyData.timestamps;
            this.temperatureChart.data.datasets.forEach((dataset, i) => {
                dataset.data = this.historyData.temperatures[i];
            });
            this.temperatureChart.update('none');
        }
        
        // Update effort chart
        if (this.effortChart) {
            this.effortChart.data.labels = this.historyData.timestamps;
            this.effortChart.data.datasets.forEach((dataset, i) => {
                dataset.data = this.historyData.efforts[i];
            });
            this.effortChart.update('none');
        }
    }
}

// Initialize dashboard
let dashboard = null;

document.addEventListener('DOMContentLoaded', () => {
    dashboard = new Dashboard();
});

window.dashboard = dashboard;
