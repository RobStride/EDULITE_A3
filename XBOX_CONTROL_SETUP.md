# Xbox手柄实时控制 - 安装和使用指南

## 📦 已完成的工作

✅ 创建了`el_a3_teleop`功能包  
✅ 实现了两种控制节点（MoveGroup和Servo）  
✅ 配置了完整的启动文件  
✅ 编译并测试通过  

## 🎮 功能特性

- **实时笛卡尔空间控制**: 直接控制末端执行器的位置和姿态
- **直观的按键映射**: 左摇杆控制XY，扳机控制Z，右摇杆控制姿态
- **可配置参数**: 速度、死区、滤波等参数可调
- **安全保护**: 关节限制、奇异点检测
- **低延迟**: 使用MoveIt Servo实现30Hz+的实时控制

## 📋 控制映射

| 输入 | 功能 | 说明 |
|------|------|------|
| 左摇杆 左右 | X轴平移 | 机器人左右移动 |
| 左摇杆 上下 | Y轴平移 | 机器人前后移动 |
| LT (左扳机) | Z轴向下 | 按下越深，下降越快 |
| RT (右扳机) | Z轴向上 | 按下越深，上升越快 |
| 右摇杆 左右 | Yaw旋转 | 绕Z轴旋转（偏航） |
| 右摇杆 上下 | Pitch旋转 | 绕Y轴旋转（俯仰） |
| LB (左肩键) | Roll向左 | 绕X轴逆时针旋转 |
| RB (右肩键) | Roll向右 | 绕X轴顺时针旋转 |

## ⚙️ 安装步骤

### 1. 安装依赖

```bash
# 安装joy和moveit_servo包
sudo apt update
sudo apt install ros-humble-joy ros-humble-moveit-servo

# 如果需要，安装手柄测试工具
sudo apt install joystick
```

### 2. 编译（已完成）

功能包已经编译完成。如果需要重新编译：

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select el_a3_teleop
source install/setup.bash
```

### 3. 测试安装

```bash
# 运行测试脚本
cd /home/wy/RS/A3
bash scripts/test_xbox_control.sh
```

## 🚀 使用方法

### 方法1: 一键启动（推荐）

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 仿真模式
ros2 launch el_a3_teleop complete_teleop.launch.py use_mock_hardware:=true

# 真实硬件模式
sudo ./scripts/setup_can.sh can0  # 先设置CAN接口
ros2 launch el_a3_teleop complete_teleop.launch.py use_mock_hardware:=false
```

这个命令会自动启动：
- 机器人模型和ros2_control
- MoveIt2运动规划
- MoveIt Servo实时控制
- Joy手柄驱动
- Xbox控制节点
- RViz可视化

### 方法2: 分步启动

如果需要更多控制或调试：

```bash
# 终端1: 启动机器人和MoveIt
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_moveit_config robot.launch.py

# 终端2: 启动MoveIt Servo
ros2 run moveit_servo servo_node --ros-args \
  --params-file /home/wy/RS/A3/ros2_ws/src/el_a3_teleop/config/moveit_servo_config.yaml

# 终端3: 启动手柄控制
ros2 launch el_a3_teleop xbox_servo_teleop.launch.py
```

## 🔧 配置调整

### 调整控制速度

编辑配置文件：

```bash
nano /home/wy/RS/A3/ros2_ws/src/el_a3_teleop/config/xbox_servo_teleop.yaml
```

修改这些参数：

```yaml
xbox_servo_teleop_node:
  ros__parameters:
    linear_scale: 0.1    # 线性速度 (m/s)，增大=更快
    angular_scale: 0.5   # 角速度 (rad/s)，增大=更快
    deadzone: 0.1        # 摇杆死区 (0-1)，减小=更灵敏
```

推荐设置：
- **新手**: linear_scale=0.05, angular_scale=0.2
- **正常**: linear_scale=0.1, angular_scale=0.5 (默认)
- **快速**: linear_scale=0.2, angular_scale=1.0

### 调整Servo性能

编辑Servo配置：

```bash
nano /home/wy/RS/A3/ros2_ws/src/el_a3_teleop/config/moveit_servo_config.yaml
```

## 🔍 调试和故障排除

### 检查手柄连接

```bash
# 查看手柄设备
ls /dev/input/js*

# 测试手柄输入
ros2 run joy joy_node
# 在另一个终端
ros2 topic echo /joy
```

### 检查权限

```bash
# 如果没有权限读取手柄
sudo chmod 666 /dev/input/js0

# 或永久添加到input组
sudo usermod -a -G input $USER
# 注销并重新登录
```

### 监控控制命令

```bash
# 查看发送到Servo的命令
ros2 topic echo /servo_node/delta_twist_cmds

# 查看Servo状态
ros2 topic echo /servo_node/status
```

### 常见问题

**问题1: 手柄有输入但机器人不动**

1. 检查MoveIt Servo是否运行：
   ```bash
   ros2 node list | grep servo
   ```

2. 检查话题连接：
   ```bash
   ros2 topic list | grep twist
   ```

**问题2: 控制延迟高**

- 降低发布频率（moveit_servo_config.yaml中的publish_period）
- 关闭碰撞检查（check_collisions: false）

**问题3: 运动不平滑**

- 增加滤波系数（moveit_servo_config.yaml中的low_pass_filter_coeff，减小值）

## 📚 文档位置

详细文档已创建在以下位置：

```bash
# 快速开始指南
/home/wy/RS/A3/ros2_ws/src/el_a3_teleop/QUICK_START.md

# 完整README
/home/wy/RS/A3/ros2_ws/src/el_a3_teleop/README.md

# 详细使用指南
/home/wy/RS/A3/ros2_ws/src/el_a3_teleop/USAGE_GUIDE.md

# 测试脚本
/home/wy/RS/A3/scripts/test_xbox_control.sh
```

查看文档：

```bash
# 查看快速开始
cat /home/wy/RS/A3/ros2_ws/src/el_a3_teleop/QUICK_START.md

# 查看完整指南
less /home/wy/RS/A3/ros2_ws/src/el_a3_teleop/USAGE_GUIDE.md
```

## 🎯 下一步

1. **安装MoveIt Servo**（如果还没安装）：
   ```bash
   sudo apt install ros-humble-moveit-servo
   ```

2. **连接Xbox手柄**：
   - 有线：直接插入USB
   - 无线：使用Xbox无线适配器

3. **首次测试**（建议在仿真中）：
   ```bash
   ros2 launch el_a3_teleop complete_teleop.launch.py use_mock_hardware:=true
   ```

4. **熟悉控制**：
   - 先尝试单轴移动
   - 逐渐增加复杂度
   - 调整速度参数到舒适的程度

5. **实际硬件测试**：
   - 确保CAN接口正确设置
   - 从低速开始
   - 准备好急停

## ⚠️ 安全提示

**重要**：
1. ✅ 首次使用必须在仿真中测试
2. ✅ 从最低速度开始（linear_scale=0.05）
3. ✅ 确保工作空间内无障碍物和人员
4. ✅ 准备好急停装置
5. ✅ 避免接近关节限制和奇异点
6. ❌ 不要在不熟悉的环境中使用高速
7. ❌ 不要长时间连续操作（防止电机过热）

## 📞 支持

如遇问题，可以：
1. 查看详细使用指南
2. 运行测试脚本诊断
3. 检查ROS2日志
4. 联系维护者

## 🎉 完成

Xbox手柄实时控制功能已经完全集成到EL-A3系统中。祝使用愉快！

---

**功能包版本**: 0.0.1  
**创建日期**: 2026-01-05  
**ROS2版本**: Humble  
**许可证**: Apache-2.0





