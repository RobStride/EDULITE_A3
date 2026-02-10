# EL-A3 Web UI

基于 Flask + Three.js 的 EL-A3 机械臂 Web 控制界面。

## 功能特性

### 核心功能
- **3D 模型可视化**：实时显示机械臂姿态，支持STL网格加载
- **关节状态监控**：位置、速度、力矩、温度实时显示
- **关节控制**：滑块和数值输入控制各关节
- **预设位姿**：快速切换到预设位置

### 控制功能
- **电机使能**：一键使能/失能所有电机
- **紧急停止**：立即停止机械臂运动
- **零力矩模式**：启用后可手动拖动机械臂
- **Xbox手柄遥操作**：支持手柄控制机械臂末端运动

### 监控功能
- **温度监控**：6轴电机温度实时显示和曲线图
- **力矩曲线**：关节力矩实时曲线
- **日志记录**：操作日志和状态记录

### 配置功能
- **CAN接口选择**：动态切换CAN通信接口
- **PID参数调节**：位置环和速度环参数
- **重力补偿**：补偿比例调节

## 快速启动

### 方法 1：直接运行

```bash
# 安装依赖
pip3 install flask flask-socketio eventlet python-socketio

# 进入工作空间
cd /home/wy/RS/A3/ros2_ws

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 运行服务器
python3 src/el_a3_web_ui/el_a3_web_ui/web_server.py --port 5000
```

### 方法 2：使用 ROS2 Launch

```bash
# 先构建包
cd /home/wy/RS/A3
colcon build --packages-select el_a3_web_ui

# 启动
source install/setup.bash
ros2 launch el_a3_web_ui web_ui.launch.py
```

## 访问界面

启动后，在浏览器中打开：

- 本机访问: `http://localhost:5000`
- 局域网访问: `http://<服务器IP>:5000`

## 界面说明

### 顶部栏

| 组件 | 功能 |
|------|------|
| 连接状态 | 显示与机械臂的连接状态 |
| 使能电机 | 点击使能/失能所有电机 |
| 紧急停止 | 立即停止机械臂运动 |

### 左侧面板 - 3D视图

- 实时显示机械臂3D模型
- 鼠标左键拖拽旋转视角
- 滚轮缩放
- 重置视角/显示网格按钮

### 右侧面板 - 控制标签页

#### 1. 状态监控

| 功能 | 说明 |
|------|------|
| 关节状态表 | 显示位置(°)、速度、力矩、温度(°C) |
| 温度曲线 | 6轴电机温度实时曲线 (20-80°C) |
| 力矩曲线 | 关节力矩实时曲线 |

温度颜色指示：
- 白色: 正常 (< 45°C)
- 橙色: 警告 (45-60°C)
- 红色闪烁: 危险 (> 60°C)

#### 2. 关节控制

- 6个关节滑块控制
- 数值输入框精确控制
- 预设位姿按钮（回原点、预设1、预设2）
- 运动时间设置

#### 3. 参数设置

| 功能 | 说明 |
|------|------|
| 零力矩模式 | 启用后机械臂可自由拖动 |
| 手柄控制 | 启动/停止Xbox手柄遥操作 |
| CAN接口配置 | 选择并切换CAN通信接口 |
| PID参数 | 位置环Kp、速度环Kd |
| 重力补偿 | 补偿比例(0-100%) |

**手柄控制映射：**
- 左摇杆: X/Y平移
- LT/RT: Z平移
- 右摇杆: Yaw/Roll
- LB/RB: Pitch
- A键: 切换速度档位
- B键: 回Home位置

#### 4. 日志

- 实时操作日志显示
- 清除日志/导出日志功能

## 技术架构

```
┌─────────────────────────────────────────────────────────┐
│                      Frontend                            │
│  ┌───────────┐ ┌───────────┐ ┌───────────────────────┐ │
│  │ Three.js  │ │ Chart.js  │ │ Socket.IO Client      │ │
│  │ 3D Model  │ │ Charts    │ │ Real-time Comm        │ │
│  └───────────┘ └───────────┘ └───────────────────────┘ │
└───────────────────────┬─────────────────────────────────┘
                        │ WebSocket
┌───────────────────────┴─────────────────────────────────┐
│                      Backend                             │
│  ┌───────────────────┐ ┌─────────────────────────────┐  │
│  │  Flask Server     │ │  ROS2 Bridge                │  │
│  │  + SocketIO       │ │  - 话题订阅/发布            │  │
│  │  + REST API       │ │  - 服务调用                 │  │
│  └───────────────────┘ │  - 进程管理(teleop)         │  │
│                        └─────────────────────────────┘  │
└───────────────────────┬─────────────────────────────────┘
                        │ ROS2 Topics/Services
┌───────────────────────┴─────────────────────────────────┐
│                    ROS2 System                           │
│  Topics:                                                 │
│    /joint_states, /debug/motor_temperature              │
│    /debug/gravity_torque, /arm_controller/trajectory    │
│  Services:                                               │
│    /el_a3/set_zero_torque_mode                          │
└─────────────────────────────────────────────────────────┘
```

## ROS2 接口

### 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节状态(位置/速度/力矩) |
| `/debug/gravity_torque` | `sensor_msgs/JointState` | 重力补偿力矩 |
| `/debug/motor_temperature` | `sensor_msgs/JointState` | 电机温度(°C) |

### 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 轨迹命令 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/el_a3/set_zero_torque_mode` | `std_srvs/SetBool` | 零力矩模式/电机使能 |

### REST API

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/state` | GET | 获取机械臂状态 |
| `/api/history` | GET | 获取状态历史 |
| `/api/logs` | GET | 获取日志 |
| `/api/joint_limits` | GET | 获取关节限位 |
| `/api/motor_status` | GET | 获取电机使能状态 |
| `/api/teleop_status` | GET | 获取手柄控制状态 |
| `/api/can_interfaces` | GET | 获取可用CAN接口 |

### WebSocket 命令

| 命令类型 | 参数 | 说明 |
|---------|------|------|
| `set_joints` | positions, duration | 设置所有关节位置 |
| `set_single_joint` | joint_index, position, duration | 设置单个关节 |
| `go_home` | duration | 回原点 |
| `emergency_stop` | - | 紧急停止 |
| `enable_motors` | - | 使能电机 |
| `disable_motors` | - | 失能电机 |
| `set_zero_torque` | enable | 设置零力矩模式 |
| `start_teleop` | - | 启动手柄控制 |
| `stop_teleop` | - | 停止手柄控制 |
| `set_can_interface` | interface | 设置CAN接口 |

## 文件结构

```
el_a3_web_ui/
├── el_a3_web_ui/
│   ├── __init__.py
│   ├── web_server.py      # Flask服务器主程序
│   └── ros2_bridge.py     # ROS2通信桥接
├── templates/
│   └── index.html         # 主页面模板
├── static/
│   ├── css/
│   │   └── style.css      # 样式表
│   ├── js/
│   │   ├── app.js         # 主应用逻辑
│   │   ├── robot_viewer.js # 3D模型渲染
│   │   ├── dashboard.js   # 状态监控和图表
│   │   └── control_panel.js # 控制面板逻辑
│   └── urdf/
│       └── meshes/        # STL网格文件
├── launch/
│   └── web_ui.launch.py   # ROS2 Launch文件
├── package.xml
├── setup.py
└── README.md
```

## 依赖项

### Python 依赖

```
flask >= 2.0
flask-socketio >= 5.0
python-socketio >= 5.0
rclpy
sensor_msgs
trajectory_msgs
std_srvs
```

### 前端依赖 (CDN)

- Three.js v0.158.0
- Chart.js v4.4.1
- Socket.IO Client v4.7.2

## 注意事项

1. **启动顺序**：先启动机械臂控制系统 (`el_a3_control.launch.py`)，再启动Web UI
2. **CAN接口**：切换CAN接口前确保机械臂处于安全状态
3. **温度监控**：温度超过60°C时会显示红色警告
4. **紧急停止**：紧急情况下点击紧急停止按钮
5. **零力矩模式**：启用后机械臂可自由移动，注意安全

## 故障排除

### Web UI无法连接

1. 检查ROS2控制系统是否正常运行
2. 确认`/joint_states`话题有数据发布
3. 检查端口5000是否被占用

### 使能按钮不工作

1. 检查`/el_a3/set_zero_torque_mode`服务是否可用
2. 重启机械臂控制系统
3. 清理ROS2共享内存: `sudo rm -rf /dev/shm/fastrtps*`

### 3D模型不显示

1. 检查浏览器控制台是否有错误
2. 确认STL文件路径正确
3. 尝试刷新页面或清除缓存

## 更新日志

### v1.1.0 (2026-01-26)
- 新增电机使能/失能功能
- 新增Xbox手柄遥操作支持
- 新增CAN接口动态配置
- 新增电机温度监控和曲线显示
- 优化3D模型加载和显示

### v1.0.0 (初始版本)
- 基础3D可视化
- 关节状态监控
- 关节控制功能
- 零力矩模式
