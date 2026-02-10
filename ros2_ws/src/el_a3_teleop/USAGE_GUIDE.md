# Xbox手柄控制 - 完整使用指南

## 目录

- [系统架构](#系统架构)
- [快速开始](#快速开始)
- [控制映射详解](#控制映射详解)
- [配置调优](#配置调优)
- [高级功能](#高级功能)
- [故障排除](#故障排除)

## 系统架构

本系统采用以下架构实现Xbox手柄的实时控制：

```
Xbox手柄 → joy_node → xbox_servo_teleop_node → MoveIt Servo → ros2_control → 机械臂
         (发布/joy)  (发布TwistStamped)      (计算关节速度)  (执行命令)
```

### 组件说明

1. **joy_node**: ROS2标准手柄驱动节点
   - 读取`/dev/input/js0`设备
   - 发布`sensor_msgs/Joy`消息到`/joy`话题

2. **xbox_servo_teleop_node**: 自定义手柄解释节点
   - 订阅`/joy`话题
   - 将手柄输入转换为笛卡尔速度命令
   - 发布`TwistStamped`消息到`/servo_node/delta_twist_cmds`

3. **MoveIt Servo**: 实时伺服控制节点
   - 接收笛卡尔速度命令
   - 通过逆运动学计算关节速度
   - 发送关节命令到ros2_control

4. **ros2_control**: 硬件抽象层
   - 接收关节命令
   - 通过CAN总线控制实际电机（或仿真）

## 快速开始

### 最简单的启动方式

```bash
cd ~/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动完整系统（仿真）
ros2 launch el_a3_teleop complete_teleop.launch.py
```

这个命令会启动所有必要的组件。

### 检查手柄连接

```bash
# 查看手柄设备
ls -l /dev/input/js*

# 应该看到类似：
# /dev/input/js0

# 测试手柄输入
ros2 topic echo /joy
# 按下手柄按钮，应该能看到数据变化
```

## 控制映射详解

### 基本控制

#### 平移控制（位置）

| 输入 | 轴索引 | 控制方向 | 默认速度 |
|------|--------|----------|----------|
| 左摇杆 → | axes[0] | +X (右) | 0.1 m/s |
| 左摇杆 ← | axes[0] | -X (左) | 0.1 m/s |
| 左摇杆 ↑ | axes[1] | +Y (前) | 0.1 m/s |
| 左摇杆 ↓ | axes[1] | -Y (后) | 0.1 m/s |
| RT扳机 | axes[5] | +Z (上) | 0.1 m/s |
| LT扳机 | axes[2] | -Z (下) | 0.1 m/s |

#### 旋转控制（姿态）

| 输入 | 轴/按钮 | 控制方向 | 默认速度 |
|------|---------|----------|----------|
| 右摇杆 → | axes[3] | +Yaw (逆时针看下) | 0.5 rad/s |
| 右摇杆 ← | axes[3] | -Yaw (顺时针看下) | 0.5 rad/s |
| 右摇杆 ↑ | axes[4] | +Pitch (抬头) | 0.5 rad/s |
| 右摇杆 ↓ | axes[4] | -Pitch (低头) | 0.5 rad/s |
| RB肩键 | buttons[5] | +Roll (右倾) | 0.5 rad/s |
| LB肩键 | buttons[4] | -Roll (左倾) | 0.5 rad/s |

### 坐标系说明

所有运动都是相对于机器人的`base_link`坐标系：
- **X轴**: 机器人前方
- **Y轴**: 机器人左侧
- **Z轴**: 机器人上方

旋转采用RPY（Roll-Pitch-Yaw）欧拉角：
- **Roll**: 绕X轴旋转
- **Pitch**: 绕Y轴旋转
- **Yaw**: 绕Z轴旋转

## 配置调优

### 速度调整

编辑配置文件：
```bash
nano ~/RS/A3/ros2_ws/src/el_a3_teleop/config/xbox_servo_teleop.yaml
```

#### 线性速度 (linear_scale)

```yaml
linear_scale: 0.1  # 米/秒
```

**推荐值**：
- 新手/精密操作: `0.05` (50mm/s)
- 正常使用: `0.1` (100mm/s)
- 快速移动: `0.2` (200mm/s)
- 最大（谨慎）: `0.5` (500mm/s)

#### 角速度 (angular_scale)

```yaml
angular_scale: 0.5  # 弧度/秒
```

**推荐值**：
- 新手/精密操作: `0.2` (~11°/s)
- 正常使用: `0.5` (~29°/s)
- 快速旋转: `1.0` (~57°/s)
- 最大（谨慎）: `2.0` (~115°/s)

#### 死区调整 (deadzone)

```yaml
deadzone: 0.1  # 0-1之间
```

死区消除摇杆漂移：
- 太小 (0.05): 可能有漂移
- 适中 (0.1): 平衡
- 太大 (0.3): 响应迟钝

### MoveIt Servo调优

编辑Servo配置：
```bash
nano ~/RS/A3/ros2_ws/src/el_a3_teleop/config/moveit_servo_config.yaml
```

#### 关键参数

```yaml
publish_period: 0.034  # 约30Hz，越小越流畅但计算量越大
low_pass_filter_coeff: 2.0  # 滤波系数，越小越平滑但延迟越大
check_collisions: false  # 碰撞检查（实时控制时建议关闭）
```

**调优建议**：

1. **减少延迟**（牺牲平滑度）：
   ```yaml
   publish_period: 0.02  # 50Hz
   low_pass_filter_coeff: 5.0
   ```

2. **增加平滑度**（增加延迟）：
   ```yaml
   publish_period: 0.05  # 20Hz
   low_pass_filter_coeff: 1.0
   ```

3. **启用安全检查**（降低性能）：
   ```yaml
   check_collisions: true
   collision_check_rate: 10.0
   ```

## 高级功能

### 多手柄支持

如果有多个手柄：

```bash
# 查看所有手柄
ls /dev/input/js*

# 使用第二个手柄
ros2 launch el_a3_teleop xbox_servo_teleop.launch.py device:=/dev/input/js1
```

### 自定义按键映射

如果你的手柄按键映射不同，可以修改节点代码：

1. 首先确定你的手柄映射：
```bash
ros2 topic echo /joy
# 移动摇杆和按下按钮，观察哪个索引变化
```

2. 修改映射：
```bash
nano ~/RS/A3/ros2_ws/src/el_a3_teleop/el_a3_teleop/xbox_servo_teleop.py
```

找到这些行并修改索引：
```python
# 修改前
twist_msg.twist.linear.x = self.apply_deadzone(msg.axes[0]) * self.linear_scale

# 修改为你的手柄对应的轴
twist_msg.twist.linear.x = self.apply_deadzone(msg.axes[YOUR_AXIS]) * self.linear_scale
```

### 录制和回放

可以录制手柄操作序列：

```bash
# 录制
ros2 bag record /servo_node/delta_twist_cmds -o my_motion

# 回放
ros2 bag play my_motion
```

### 与其他控制方式结合

可以同时运行多个控制界面：

```bash
# 终端1: 启动系统
ros2 launch el_a3_teleop complete_teleop.launch.py

# 终端2: 启动键盘控制（需要另外实现）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/servo_node/delta_twist_cmds
```

## 故障排除

### 手柄问题

#### 手柄未检测到

```bash
# 检查USB连接
lsusb | grep -i xbox

# 检查设备节点
ls -l /dev/input/js*

# 如果没有，尝试重新插拔或重启
```

#### 权限不足

```bash
# 临时解决
sudo chmod 666 /dev/input/js0

# 永久解决
sudo usermod -a -G input $USER
# 注销并重新登录
```

#### 手柄输入但机器人不动

1. 检查话题连接：
```bash
# 应该有数据
ros2 topic echo /joy
ros2 topic echo /servo_node/delta_twist_cmds
```

2. 检查MoveIt Servo状态：
```bash
ros2 topic echo /servo_node/status
```

3. 查看日志：
```bash
ros2 node list
ros2 node info /servo_node
```

### 性能问题

#### 控制延迟高

1. 降低发布频率：
```yaml
# moveit_servo_config.yaml
publish_period: 0.05  # 从0.034增加到0.05
```

2. 减少滤波：
```yaml
low_pass_filter_coeff: 5.0  # 从2.0增加到5.0
```

3. 关闭碰撞检查：
```yaml
check_collisions: false
```

#### CPU占用高

```bash
# 查看节点CPU占用
top -p $(pgrep -d',' -f servo_node)

# 降低更新率
# 修改 moveit_servo_config.yaml
publish_period: 0.05  # 20Hz instead of 30Hz
```

### 运动问题

#### 机器人运动不平滑

1. 增加滤波：
```yaml
low_pass_filter_coeff: 1.0  # 从2.0降低到1.0
```

2. 检查死区设置：
```yaml
deadzone: 0.15  # 增加死区
```

#### 机器人移动太快/太慢

修改速度缩放：
```yaml
# xbox_servo_teleop.yaml
linear_scale: 0.05   # 降低线性速度
angular_scale: 0.3   # 降低角速度
```

#### 运动突然停止

可能触发了奇异点保护或关节限制：

1. 查看Servo状态：
```bash
ros2 topic echo /servo_node/status
```

2. 调整限制：
```yaml
# moveit_servo_config.yaml
joint_limit_margin: 0.2  # 增加安全余量
lower_singularity_threshold: 20.0  # 调整奇异性阈值
```

### 调试工具

#### 可视化工具

```bash
# RQT图形界面
rqt

# 话题监控
rqt_plot /servo_node/delta_twist_cmds/twist/linear/x

# TF树查看
ros2 run tf2_tools view_frames
evince frames.pdf
```

#### 日志调试

```bash
# 设置日志级别
ros2 run el_a3_teleop xbox_servo_teleop_node --ros-args --log-level debug

# 查看所有话题
ros2 topic list -v

# 查看话题频率
ros2 topic hz /servo_node/delta_twist_cmds
```

## 最佳实践

### 安全操作

1. ✅ **首次使用前**：
   - 在仿真中充分测试
   - 从最低速度开始
   - 确保急停装置就绪

2. ✅ **日常使用**：
   - 清空工作空间
   - 保持注意力集中
   - 避免接近关节限制
   - 定期检查手柄电量

3. ❌ **避免**：
   - 在不熟悉的环境中使用高速
   - 长时间连续操作（电机过热）
   - 在接近奇异点时大幅移动

### 性能优化

1. 根据任务调整参数：
   - **精密装配**: 低速 + 高频率
   - **快速移动**: 高速 + 低频率
   - **演示**: 中速 + 平滑滤波

2. 监控系统负载：
```bash
# 查看CPU占用
htop

# 查看话题延迟
ros2 topic delay /servo_node/delta_twist_cmds
```

## 扩展开发

### 添加新功能

想要添加更多按钮功能？编辑节点代码：

```python
# el_a3_teleop/xbox_servo_teleop.py

def joy_callback(self, msg):
    # 现有代码...
    
    # 添加新功能：按A键切换速度模式
    if len(msg.buttons) > 0 and msg.buttons[0]:  # A键
        self.toggle_speed_mode()
```

### 集成其他传感器

可以订阅力/力矩传感器实现力控制：

```python
self.ft_sub = self.create_subscription(
    WrenchStamped,
    '/force_torque',
    self.ft_callback,
    10
)
```

## 参考资源

- [MoveIt Servo文档](https://moveit.picknik.ai/humble/doc/realtime_servo/realtime_servo_tutorial.html)
- [ROS2 Joy包](https://index.ros.org/p/joy/)
- [Xbox手柄Linux驱动](https://github.com/xpadneo/xpadneo)

## 反馈和贡献

如有问题或建议，请联系维护者或提交Issue。





