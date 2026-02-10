# Joy-Con IMU 遥控使用指南

本指南介绍如何使用 Nintendo Switch Joy-Con 手柄的 IMU 传感器控制 EL-A3 机械臂。

## 目录

- [系统要求](#系统要求)
- [安装驱动](#安装驱动)
- [配对 Joy-Con](#配对-joy-con)
- [快速开始](#快速开始)
- [控制说明](#控制说明)
- [参数调优](#参数调优)
- [故障排除](#故障排除)

---

## 系统要求

- Ubuntu 22.04 LTS
- Linux 内核 >= 5.16（内置 hid-nintendo 模块）
- 蓝牙适配器
- Nintendo Switch Joy-Con（左手或右手均可）

## 安装驱动

### 一键安装

```bash
cd /home/wy/A3/scripts
sudo ./install_joycon_driver.sh
```

### 手动安装

1. **加载 hid-nintendo 模块**

```bash
sudo modprobe hid-nintendo

# 设置开机自动加载
echo "hid-nintendo" | sudo tee /etc/modules-load.d/hid-nintendo.conf
```

2. **安装 joycond（可选，推荐）**

joycond 是一个守护进程，用于管理 Joy-Con 的配对和组合。

```bash
sudo apt install joycond
sudo systemctl enable joycond
sudo systemctl start joycond
```

3. **安装 Python 依赖**

```bash
pip3 install evdev
```

4. **配置 udev 规则**

```bash
sudo tee /etc/udev/rules.d/50-nintendo.rules << 'EOF'
KERNEL=="hidraw*", ATTRS{idVendor}=="057e", MODE="0666"
KERNEL=="event*", ATTRS{name}=="*Joy-Con*", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 配对 Joy-Con

### 使用 bluetoothctl

1. **进入配对模式**
   
   长按 Joy-Con 侧边的同步按钮（小圆按钮），直到 LED 灯快速闪烁。

2. **蓝牙配对**

```bash
bluetoothctl
```

在 bluetoothctl 中：

```
[bluetooth]# scan on
# 等待发现 "Joy-Con (L)" 或 "Joy-Con (R)"
[bluetooth]# pair XX:XX:XX:XX:XX:XX
[bluetooth]# trust XX:XX:XX:XX:XX:XX
[bluetooth]# connect XX:XX:XX:XX:XX:XX
[bluetooth]# quit
```

3. **验证连接**

```bash
# 检查手柄设备
ls /dev/input/js*

# 查看设备信息
cat /proc/bus/input/devices | grep -A 10 "Joy-Con"

# 测试输入
jstest /dev/input/js0
```

---

## 快速开始

### 仿真模式（无需真实硬件）

```bash
cd /home/wy/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch el_a3_teleop joycon_imu_teleop.launch.py
```

### 真实硬件模式

```bash
# 1. 设置 CAN 接口
sudo /home/wy/A3/scripts/setup_can.sh can0

# 2. 启动遥控
ros2 launch el_a3_teleop joycon_imu_teleop.launch.py \
    use_mock_hardware:=false \
    can_interface:=can0
```

---

## 控制说明

### IMU 姿态控制

| 动作 | 机械臂响应 |
|------|-----------|
| **Pitch (前后倾斜)** | X 方向平移 |
| **Roll (左右倾斜)** | Y 方向平移 |
| **Yaw (水平旋转)** | 末端 Z 轴旋转 |

### 按键控制

| 按键 | 功能 |
|------|------|
| **ZL** | Z 轴向下移动 |
| **ZR** | Z 轴向上移动 |
| **A** | 切换速度档位 |
| **B** | 回到 home 位置 |
| **X / -** | 重新标定 IMU |
| **Y** | 设置当前姿态为零点 |

### 速度档位

| 档位 | 名称 | 平移速度 | 旋转速度 |
|------|------|----------|----------|
| 1 | 精细 | ~25 mm/s | ~0.25 rad/s |
| 2 | 标准 | ~100 mm/s | ~1.0 rad/s |
| 3 | 快速 | ~250 mm/s | ~2.5 rad/s |

---

## 参数调优

配置文件位置：`config/joycon_imu_teleop.yaml`

### IMU 参数

```yaml
# 标定采样数（越多越准确，但标定时间越长）
calibration_samples: 200

# Madgwick 滤波器增益（0.01-0.5）
# 较大: 响应快，噪声大
# 较小: 平滑，响应慢
madgwick_beta: 0.1
```

### 控制参数

```yaml
# 平移灵敏度（米/弧度/更新）
translation_scale: 0.002

# 旋转灵敏度（弧度/弧度/更新）
rotation_scale: 0.02

# 死区（弧度，约 3°）
deadzone: 0.05
```

### 使用自定义配置

```bash
ros2 launch el_a3_teleop joycon_imu_teleop.launch.py \
    --ros-args -p translation_scale:=0.003
```

---

## 故障排除

### Joy-Con 无法连接

1. 检查蓝牙服务

```bash
sudo systemctl status bluetooth
```

2. 重新配对

```bash
bluetoothctl
[bluetooth]# remove XX:XX:XX:XX:XX:XX
[bluetooth]# scan on
# 重新配对...
```

### IMU 数据异常

1. 重新标定（按 X 或 - 键）
2. 确保标定时 Joy-Con 静止在平稳表面
3. 检查 IMU 话题

```bash
ros2 topic echo /imu
```

### 机械臂不响应

1. 检查控制器状态

```bash
ros2 control list_controllers
```

2. 检查关节状态

```bash
ros2 topic echo /joint_states
```

3. 查看节点日志

```bash
ros2 launch el_a3_teleop joycon_imu_teleop.launch.py debug:=true
```

### hid-nintendo 模块问题

```bash
# 检查模块
lsmod | grep hid_nintendo

# 手动加载
sudo modprobe hid-nintendo

# 检查 dmesg 日志
dmesg | grep -i nintendo
```

---

## 技术细节

### IMU 数据处理流程

```
Joy-Con IMU 原始数据
        ↓
    零飘标定
        ↓
    低通滤波
        ↓
 Madgwick 姿态估计
        ↓
   笛卡尔增量映射
        ↓
     IK 逆解
        ↓
   关节轨迹发布
```

### ROS2 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/imu` | sensor_msgs/Imu | IMU 原始数据 |
| `/joy` | sensor_msgs/Joy | 按键数据 |
| `/joycon/imu_euler` | geometry_msgs/Vector3 | 姿态欧拉角 |
| `/joycon/calibration_status` | std_msgs/String | 标定状态 |
| `/target_pose` | geometry_msgs/PoseStamped | 目标末端位姿 |

---

## 已知限制

1. **Yaw 漂移**: 由于没有磁力计，Yaw 角会随时间漂移。定期按 Y 键重置零点可缓解。

2. **延迟**: IMU 数据经蓝牙传输有约 10-20ms 延迟。

3. **单手柄限制**: 当前仅支持单个 Joy-Con 控制。

---

**最后更新**: 2026-01-11
