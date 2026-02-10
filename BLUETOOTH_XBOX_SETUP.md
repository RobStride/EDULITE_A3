# 蓝牙Xbox手柄设置指南

## 当前状态

您的Xbox手柄通过蓝牙连接，需要确保正确配置才能使用。

## 🔧 设置步骤

### 1. 加载必要的内核模块

```bash
# 加载joystick设备模块
sudo modprobe joydev

# 设置为开机自动加载
echo "joydev" | sudo tee -a /etc/modules
```

### 2. 检查蓝牙手柄连接

```bash
# 检查蓝牙设备
bluetoothctl devices

# 应该能看到类似这样的输出：
# Device XX:XX:XX:XX:XX:XX Xbox Wireless Controller
```

### 3. 检查输入设备

```bash
# 查看所有输入设备
ls -la /dev/input/

# 应该能看到：
# - eventX 设备（事件接口）
# - jsX 设备（joystick接口）← 这个是我们需要的！
```

### 4. 查找手柄设备

```bash
# 列出所有输入事件
cat /proc/bus/input/devices | grep -A 5 "Xbox"

# 或者查看哪些设备是joystick
ls -l /dev/input/by-id/ | grep -i "xbox\|joystick"
```

### 5. 测试手柄输入

```bash
# 方法1: 使用jstest（需要安装）
sudo apt install joystick
jstest /dev/input/js0

# 方法2: 使用evtest
sudo apt install evtest
sudo evtest

# 然后选择Xbox手柄对应的设备号
```

### 6. 设置权限

如果手柄设备存在但无法访问：

```bash
# 临时设置权限
sudo chmod 666 /dev/input/js0

# 永久设置：添加用户到input组
sudo usermod -a -G input $USER

# 注销并重新登录后生效
```

## 🎮 使用joy_node测试

一旦手柄设备可用（`/dev/input/js0`），在新终端运行：

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动joy节点
ros2 run joy joy_node --ros-args -p device_name:=/dev/input/js0

# 在另一个终端查看手柄数据
ros2 topic echo /joy
```

移动摇杆和按下按钮，应该能看到数据变化。

## 📱 蓝牙Xbox手柄配对步骤

如果手柄还未配对：

### 方法1：使用图形界面（推荐）

1. 打开系统设置 → 蓝牙
2. 打开Xbox手柄：按住配对按钮（顶部圆形按钮，Xbox logo旁边）
3. 手柄上的Xbox logo开始快速闪烁
4. 在蓝牙设置中找到"Xbox Wireless Controller"
5. 点击连接

### 方法2：使用命令行

```bash
# 启动bluetoothctl
bluetoothctl

# 在bluetoothctl中执行：
power on
agent on
default-agent
scan on

# 按住手柄的配对按钮，等待出现：
# [NEW] Device XX:XX:XX:XX:XX:XX Xbox Wireless Controller

# 复制设备MAC地址，然后执行：
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX

# 连接成功后：
scan off
exit
```

## 🔍 故障排除

### 问题1：没有/dev/input/js0设备

**原因**：joydev模块未加载

**解决**：
```bash
sudo modprobe joydev
ls /dev/input/js*  # 应该能看到设备了
```

### 问题2：手柄连接但无响应

**原因**：xpad驱动问题

**解决**：安装xpadneo驱动（Xbox手柄的高级Linux驱动）
```bash
# 克隆xpadneo仓库
cd ~/Downloads
git clone https://github.com/atar-axis/xpadneo.git
cd xpadneo

# 安装
sudo ./install.sh

# 重启蓝牙
sudo systemctl restart bluetooth
```

### 问题3：权限被拒绝

**解决**：
```bash
# 查看当前用户的组
groups

# 如果不在input组中，添加：
sudo usermod -a -G input $USER

# 注销并重新登录
```

### 问题4：手柄频繁断连

**原因**：电池电量低或蓝牙干扰

**解决**：
- 更换手柄电池
- 靠近蓝牙接收器
- 关闭其他2.4GHz设备（如某些WiFi路由器）

## ✅ 验证设置成功

运行以下命令检查：

```bash
# 1. 检查joydev模块
lsmod | grep joydev

# 2. 检查js设备
ls /dev/input/js*

# 3. 检查蓝牙连接
bluetoothctl info | grep -i xbox

# 4. 测试joy_node
ros2 run joy joy_node
```

如果所有步骤都成功，继续阅读下一节。

## 🚀 重新启动控制系统

设置完成后，重新启动仿真系统：

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch el_a3_teleop simple_teleop.launch.py
```

## 📝 检查列表

- [ ] 加载joydev模块：`sudo modprobe joydev`
- [ ] 手柄已通过蓝牙配对和连接
- [ ] 存在/dev/input/js0设备
- [ ] 用户在input组中：`groups | grep input`
- [ ] joy_node能读取手柄数据：`ros2 topic echo /joy`
- [ ] RViz中机器人响应手柄输入

## 🎯 快速测试流程

```bash
# 终端1: 测试手柄
jstest /dev/input/js0
# 移动摇杆看是否有反应

# 终端2: 启动控制系统
cd /home/wy/RS/A3/ros2_ws
source install/setup.bash
ros2 launch el_a3_teleop simple_teleop.launch.py

# 终端3: 监控手柄数据
ros2 topic echo /joy
```

## 💡 有用的命令

```bash
# 查看手柄电池电量（需要xpadneo驱动）
cat /sys/class/power_supply/xbox_battery_*/capacity

# 重新连接手柄
bluetoothctl connect XX:XX:XX:XX:XX:XX

# 查看实时输入
evtest /dev/input/event<N>  # N是手柄对应的事件号

# 调试joy_node
ros2 run joy joy_node --ros-args --log-level debug
```

## 📚 相关资源

- [xpadneo GitHub](https://github.com/atar-axis/xpadneo)
- [ROS2 joy包文档](https://index.ros.org/p/joy/)
- [Linux输入子系统文档](https://www.kernel.org/doc/html/latest/input/)

---

**下一步**：完成设置后，查看 `/home/wy/RS/A3/XBOX_HOW_TO_USE.md` 了解如何控制机器人。





