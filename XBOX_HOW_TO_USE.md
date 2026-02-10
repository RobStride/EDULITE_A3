# Xbox手柄控制 - 实机使用说明

## ✅ 系统状态

系统已经成功启动！以下组件正在运行：

- ✅ **robot_state_publisher**: 机器人模型发布
- ✅ **ros2_control_node**: 硬件控制管理（连接CAN总线）
- ✅ **move_group**: MoveIt运动规划
- ✅ **rviz2**: 3D可视化界面
- ✅ **joy_node**: 手柄驱动
- ✅ **xbox_teleop_node**: 手柄控制节点

## 🚀 启动初始化流程

程序启动时会自动执行以下步骤：
1. **读取电机位置** - 等待关节状态，确保读取到所有电机的当前位置
2. **显示当前位置** - 打印各关节的初始位置（弧度）
3. **规划回Home** - 自动规划并执行运动到Home位置
4. **Home位置定义**: L1=0°, L2=45°, L3=-45°, L4=0°, L5=0°, L6=0°

## 🎮 如何使用

### 1. 检查RViz窗口

RViz应该已经打开，显示机械臂模型。如果没有看到：
- 检查屏幕上是否有RViz窗口
- 机器人应该显示在3D视图中

### 2. 连接Xbox手柄

**有线连接**：
```bash
# 在新终端中检查手柄是否连接
ls /dev/input/js0
```

如果没有`/dev/input/js0`设备：
1. 插入Xbox手柄USB线
2. 或使用Xbox无线适配器
3. 再次检查：`ls /dev/input/js0`

### 3. 测试手柄输入

打开新终端：
```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 查看手柄输入
ros2 topic echo /joy
```

移动摇杆和按下按钮，应该能看到数值变化。

### 4. 控制机械臂

一旦手柄连接并工作：

#### 平移控制（移动位置）
- **左摇杆 左右** → 机器人左右移动 (X轴)
- **左摇杆 上下** → 机器人前后移动 (Y轴)
- **LT扳机** → 机器人向下移动 (Z轴)
- **RT扳机** → 机器人向上移动 (Z轴)

#### 旋转控制（改变姿态）
- **右摇杆 左右** → 绕Z轴旋转 (Yaw)
- **右摇杆 上下** → 绕Y轴旋转 (Pitch)
- **LB肩键** → 绕X轴逆时针旋转 (Roll)
- **RB肩键** → 绕X轴顺时针旋转 (Roll)

### 5. 观察机器人运动

- 在RViz中观察机器人运动
- 橙色/蓝色的交互标记显示目标位置
- 机器人会规划路径并移动到目标位置

## ⚙️ 当前配置

控制参数（位于`/home/wy/RS/A3/ros2_ws/src/el_a3_teleop/config/xbox_teleop.yaml`）：

- **更新频率**: 20 Hz
- **平移速度**: 0.001 m/更新 (慢速，适合初学)
- **旋转速度**: 0.01 rad/更新
- **摇杆死区**: 0.1

## 🔧 如果没有手柄怎么办？

系统已经在仿真模式下运行。即使没有Xbox手柄，你也可以：

1. **使用RViz交互控制**：
   - 在RViz的MotionPlanning面板中
   - 拖动交互标记来移动机器人
   - 点击"Plan & Execute"执行运动

2. **使用MoveIt界面**：
   - 设置目标位姿
   - 规划轨迹
   - 执行运动

## 📊 监控系统

### 查看目标位姿
```bash
# 新终端
cd /home/wy/RS/A3/ros2_ws
source install/setup.bash
ros2 topic echo /target_pose
```

### 查看关节状态
```bash
ros2 topic echo /joint_states
```

### 查看可用话题
```bash
ros2 topic list
```

## ⚠️ 注意事项

### 当前是仿真模式
- ✅ 可以安全测试
- ✅ 没有真实硬件风险
- ✅ 可以尝试各种运动

### 控制特点
- 📝 使用MoveGroup规划器（不是实时Servo）
- 📝 每次输入会规划并执行完整轨迹
- 📝 有轻微延迟是正常的
- 📝 适合精确定位，不适合连续快速运动

### 如果要更流畅的控制
需要安装MoveIt Servo：
```bash
sudo apt install ros-humble-moveit-servo
```

然后使用：
```bash
ros2 launch el_a3_teleop complete_teleop.launch.py
```

## 🛑 停止系统

在运行launch的终端中按 `Ctrl+C`

## 📚 更多信息

- 快速入门指南: `/home/wy/RS/A3/ros2_ws/src/el_a3_teleop/QUICK_START.md`
- 完整使用指南: `/home/wy/RS/A3/ros2_ws/src/el_a3_teleop/USAGE_GUIDE.md`
- 安装指南: `/home/wy/RS/A3/XBOX_CONTROL_SETUP.md`

## 🎯 快速测试步骤

1. ✅ **RViz已打开** - 你应该能看到机器人模型
2. 📱 **连接Xbox手柄** - 插入USB或使用无线适配器
3. 🎮 **测试输入** - `ros2 topic echo /joy`
4. 🤖 **移动摇杆** - 观察RViz中的机器人运动
5. 🎉 **开始控制** - 享受！

---

## 🔧 启动命令

### 实机控制（连接真实机械臂）
```bash
# 1. 重新加载CAN驱动并配置
sudo modprobe -r gs_usb && sleep 1 && sudo modprobe gs_usb && sleep 2
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 2. 启动控制系统
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/wy/RS/A3/install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0
```

### 仿真控制（无硬件）
```bash
ros2 launch el_a3_teleop simple_teleop.launch.py
```

### 速度控制

- **A键**: 切换速度档位（5档：超慢/慢速/中速/快速/极速）
- **B键**: 回到Home位置

祝使用愉快！🚀





