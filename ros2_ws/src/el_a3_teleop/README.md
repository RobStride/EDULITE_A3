# EL-A3 Xbox手柄实时控制

使用Xbox手柄进行EL-A3机械臂的笛卡尔空间实时控制。

## 功能特性

- **实时控制**: 使用MoveIt Servo进行低延迟的实时控制
- **笛卡尔空间控制**: 直接控制末端执行器的位置和姿态
- **直观的手柄映射**: 符合人体工学的控制布局

## 控制映射

### Xbox手柄按键映射

| 输入 | 控制 | 说明 |
|------|------|------|
| 左摇杆 左右 | X轴平移 | 左右移动 |
| 左摇杆 上下 | Y轴平移 | 前后移动 |
| LT (左扳机) | Z轴向下 | 按下越深，下降越快 |
| RT (右扳机) | Z轴向上 | 按下越深，上升越快 |
| 右摇杆 左右 | Yaw旋转 | 绕Z轴旋转 |
| 右摇杆 上下 | Pitch旋转 | 绕Y轴旋转 |
| LB (左肩键) | Roll向左 | 绕X轴逆时针旋转 |
| RB (右肩键) | Roll向右 | 绕X轴顺时针旋转 |

## 依赖项

### 系统依赖

```bash
# ROS2 Humble
sudo apt install ros-humble-joy
sudo apt install ros-humble-moveit-servo

# 或者如果使用完整的MoveIt安装
sudo apt install ros-humble-moveit
```

### 手柄驱动

Xbox手柄通常可以直接使用，无需额外驱动。对于其他手柄，可能需要安装`xboxdrv`：

```bash
sudo apt install xboxdrv
```

## 编译

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select el_a3_teleop
source install/setup.bash
```

## 使用方法

### 方法1：使用MoveIt Servo (推荐用于实时控制)

MoveIt Servo提供低延迟的实时伺服控制，最适合手柄操作。

```bash
# 终端1: 启动机器人和MoveIt
ros2 launch el_a3_moveit_config robot.launch.py

# 终端2: 启动MoveIt Servo
ros2 launch moveit_servo servo.launch.py

# 终端3: 启动Xbox手柄控制
ros2 launch el_a3_teleop xbox_servo_teleop.launch.py
```

### 方法2：使用MoveGroup规划

这个方法会进行轨迹规划，更安全但延迟较高。

```bash
# 终端1: 启动机器人和MoveIt
ros2 launch el_a3_moveit_config robot.launch.py

# 终端2: 启动Xbox手柄控制
ros2 launch el_a3_teleop xbox_teleop.launch.py
```

## 配置参数

### xbox_servo_teleop.yaml (伺服控制)

```yaml
xbox_servo_teleop_node:
  ros__parameters:
    linear_scale: 0.1        # 线性速度缩放 (m/s)
    angular_scale: 0.5       # 角速度缩放 (rad/s)
    base_frame: "base_link"  # 基坐标系
    deadzone: 0.1            # 摇杆死区 (0-1)
```

### xbox_teleop.yaml (规划控制)

```yaml
xbox_teleop_node:
  ros__parameters:
    update_rate: 20.0              # 更新频率 (Hz)
    translation_scale: 0.001       # 平移增量 (m/update)
    rotation_scale: 0.01           # 旋转增量 (rad/update)
    planning_group: "arm"          # MoveIt规划组
    base_frame: "base_link"        # 基坐标系
    end_effector_frame: "L6"       # 末端执行器坐标系
    deadzone: 0.1                  # 摇杆死区
```

## 启动参数

```bash
# 指定手柄设备
ros2 launch el_a3_teleop xbox_servo_teleop.launch.py device:=/dev/input/js0

# 使用自定义配置文件
ros2 launch el_a3_teleop xbox_servo_teleop.launch.py config_file:=/path/to/config.yaml
```

## 调试

### 检查手柄连接

```bash
# 查看手柄设备
ls /dev/input/js*

# 测试手柄输入
ros2 run joy joy_node
ros2 topic echo /joy
```

### 监控Twist命令

```bash
# 查看发送到MoveIt Servo的命令
ros2 topic echo /servo_node/delta_twist_cmds
```

### 查看目标位姿

```bash
# 查看目标位姿 (仅用于MoveGroup版本)
ros2 topic echo /target_pose
```

## 故障排除

### 手柄未检测到

1. 检查手柄连接：`ls /dev/input/js*`
2. 检查手柄权限：`sudo chmod 666 /dev/input/js0`
3. 测试手柄：`jstest /dev/input/js0` (需要安装`joystick`包)

### 机器人不响应

1. 确保MoveIt Servo或MoveGroup正在运行
2. 检查话题连接：`ros2 topic list`
3. 查看节点日志输出

### 控制延迟高

- 使用MoveIt Servo版本而不是MoveGroup版本
- 降低`update_rate`参数
- 增加`translation_scale`和`rotation_scale`以获得更快的响应

### 控制灵敏度问题

- 调整`linear_scale`和`angular_scale`参数
- 调整`deadzone`参数以改善摇杆响应

## 安全提示

⚠️ **重要安全提示**：

1. 首次使用时，设置较低的速度缩放因子
2. 确保机械臂工作空间内没有障碍物
3. 始终准备好急停按钮
4. 在实际硬件上使用前，先在仿真中测试

## 高级配置

### 自定义按键映射

如果你的手柄按键映射不同，可以修改节点代码中的轴和按钮索引。常见的索引可以通过运行`ros2 topic echo /joy`并按下相应按钮来确认。

### 调整速度限制

在配置文件中调整`linear_scale`和`angular_scale`：

- 增大值：更快的响应，但可能不太精确
- 减小值：更精确的控制，但响应较慢

## 技术细节

### 节点架构

- **joy_node**: 读取手柄输入并发布到`/joy`话题
- **xbox_servo_teleop_node**: 订阅`/joy`，将输入转换为`TwistStamped`消息并发布到`/servo_node/delta_twist_cmds`
- **MoveIt Servo**: 接收Twist命令并计算关节速度，实时控制机器人

### 坐标系

所有运动都是相对于`base_frame`（默认为`base_link`）的。

## 许可证

Apache-2.0

## 维护者

wy <wy@todo.todo>





