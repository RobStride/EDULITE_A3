# Xbox手柄控制 - 快速开始指南

## 快速启动（推荐）

### 一键启动（仿真模式）

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动完整系统（包含MoveIt和手柄控制）
ros2 launch el_a3_teleop complete_teleop.launch.py use_mock_hardware:=true
```

### 真实硬件模式

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 首先设置CAN接口
sudo ./scripts/setup_can.sh can0

# 启动完整系统
ros2 launch el_a3_teleop complete_teleop.launch.py use_mock_hardware:=false
```

## 分步启动（高级用户）

如果需要更多控制，可以分别启动各个组件：

### 步骤1：启动机器人和MoveIt

```bash
# 终端1
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_moveit_config robot.launch.py
```

### 步骤2：启动MoveIt Servo

```bash
# 终端2
cd /home/wy/RS/A3/ros2_ws
source install/setup.bash
ros2 run moveit_servo servo_node --ros-args --params-file src/el_a3_teleop/config/moveit_servo_config.yaml
```

### 步骤3：启动Xbox手柄控制

```bash
# 终端3
cd /home/wy/RS/A3/ros2_ws
source install/setup.bash
ros2 launch el_a3_teleop xbox_servo_teleop.launch.py
```

## 控制说明

连接Xbox手柄后：

- **左摇杆**: 控制X/Y平移
- **LT/RT**: 控制Z轴上下
- **右摇杆**: 控制Yaw/Pitch旋转
- **LB/RB**: 控制Roll旋转

## 常见问题

### 1. 手柄未检测到

```bash
# 检查设备
ls /dev/input/js*

# 测试手柄
sudo apt install joystick
jstest /dev/input/js0
```

### 2. 权限问题

```bash
# 添加用户到input组
sudo usermod -a -G input $USER
# 重新登录后生效

# 或临时修改权限
sudo chmod 666 /dev/input/js0
```

### 3. MoveIt Servo未安装

```bash
sudo apt install ros-humble-moveit-servo
```

### 4. 控制不响应

检查话题是否正常：

```bash
# 检查手柄输入
ros2 topic echo /joy

# 检查Twist命令
ros2 topic echo /servo_node/delta_twist_cmds

# 检查MoveIt Servo状态
ros2 topic echo /servo_node/status
```

## 调整控制参数

编辑配置文件以调整控制灵敏度：

```bash
nano /home/wy/RS/A3/ros2_ws/src/el_a3_teleop/config/xbox_servo_teleop.yaml
```

主要参数：
- `linear_scale`: 线性速度 (默认0.1 m/s)
- `angular_scale`: 角速度 (默认0.5 rad/s)
- `deadzone`: 摇杆死区 (默认0.1)

修改后重新启动节点。

## 安全提示

⚠️ **首次使用请务必注意**：

1. 从较低的速度开始（`linear_scale: 0.05`，`angular_scale: 0.2`）
2. 确保工作空间内无障碍物
3. 准备好急停装置
4. 先在仿真中熟悉控制

## 获取帮助

如遇问题，请查看完整README：

```bash
cat /home/wy/RS/A3/ros2_ws/src/el_a3_teleop/README.md
```





