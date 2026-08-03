# el_a3_hardware — 硬件接口参数说明

> `el_a3_hardware` 包含 ROS2 Control 硬件接口插件 (`RsA3HardwareInterface`) 和零力矩控制器插件 (`ZeroTorqueController`)。

---

## 1. RsA3HardwareInterface

ros2_control SystemInterface 插件，通过 SocketCAN 与 Robstride 电机通信。

### 1.1 全局硬件参数

在 `el_a3_ros2_control.xacro` 的 `<hardware>` 节内配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | (必填) | CAN 接口名（如 `can0`） |
| `host_can_id` | (必填) | 主机 CAN ID（通常 `253` / 0xFD） |
| `position_kp` | 100.0 | 全局默认位置增益 Kp |
| `position_kd` | 4.0 | 全局默认速度增益 Kd |
| `velocity_limit` | 10.0 | 速度限制 (rad/s) |

### 1.2 平滑滤波参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `smoothing_alpha` | 0.8 | EMA 低通滤波系数 (0=全保持, 1=直通) |

### 1.3 指令平滑限制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_velocity` | 3.0 | 最大速度限制 (rad/s) |
| `max_acceleration` | 15.0 | 最大加速度限制 (rad/s^2) |

### 1.4 重力补偿参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `urdf_path` | (自动) | URDF 路径（Pinocchio 模型加载） |
| `use_pinocchio_gravity` | true | 启用 Pinocchio RNEA 全动力学重力补偿 |
| `inertia_config_path` | (自动) | 标定惯量参数 YAML（覆盖 URDF 中的质量/质心） |
| `gravity_feedforward_ratio` | 0.0 | 位置控制模式下的重力前馈比例 (0=禁用, 1=全补偿) |

#### 遗留正弦重力补偿参数

每个关节 L1-L6 各有 3 个参数（Pinocchio 启用后仅作为备用）：

| 参数模式 | 说明 |
|---------|------|
| `gravity_comp_Lx_sin` | sin(theta) 系数 |
| `gravity_comp_Lx_cos` | cos(theta) 系数 |
| `gravity_comp_Lx_offset` | 偏移力矩 |

#### 惯量缩放因子（遗留）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `inertia_scale_L1_mass` | 1.0 | L1 质量缩放 |
| `inertia_scale_L2_mass` | 2.0 | L2 质量缩放 |
| `inertia_scale_L3_mass` | 0.6839 | L3 质量缩放 |
| `inertia_scale_L4_mass` | 0.35 | L4 质量缩放 |
| `inertia_scale_L5_mass` | 1.0 | L5 质量缩放 |
| `inertia_scale_L6_mass` | 1.0 | L6 质量缩放 |

### 1.5 零力矩模式参数（硬件层）

这些参数在硬件接口层内实现零力矩模式的自适应阻尼：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `zero_torque_kd` | 0.3 | 固定 Kd 回退值（adaptive_kd 禁用时使用） |
| `adaptive_kd_enabled` | true | 启用速度自适应 Kd（Lorentzian 衰减 + EMA 平滑） |
| `zero_torque_kd_min` | 0.001 | 高速时的 Kd（手感丝滑） |
| `zero_torque_kd_max` | 0.15 | 静止时的 Kd（保证稳定） |
| `kd_velocity_ref` | 1.0 | Kd 衰减到中间值时的参考速度 (rad/s) |
| `kd_smoothing_alpha` | 0.15 | EMA 平滑因子（越小过渡越平滑） |

自适应 Kd 公式：`Kd(v) = kd_min + (kd_max - kd_min) / (1 + (v/v_ref)^2)`

### 1.6 关节级参数

每个关节 `<joint>` 内可独立配置：

| 参数 | 说明 |
|------|------|
| `motor_id` | 电机 CAN ID (1-7) |
| `motor_type` | 电机型号 (`RS00`, `EL05`, `RS05`) |
| `position_offset` | 零位偏移 (rad) |
| `direction` | 方向系数 (1.0 或 -1.0) |
| `kp` | 关节独立 Kp（覆盖全局默认值） |
| `kd` | 关节独立 Kd（覆盖全局默认值） |
| `zero_torque_kd_min` | 关节独立零力矩 Kd 下限 |
| `zero_torque_kd_max` | 关节独立零力矩 Kd 上限 |

---

## 2. 调试 Topics

硬件接口在 `on_activate` 时创建一个内部节点 `el_a3_hw_debug`，发布以下调试数据：

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/debug/hw_command` | `sensor_msgs/JointState` | 200 Hz | `write()` 时接收到的原始位置指令 |
| `/debug/smoothed_command` | `sensor_msgs/JointState` | 200 Hz | EMA 平滑后的位置指令 |
| `/debug/gravity_torque` | `sensor_msgs/JointState` | 200 Hz | Pinocchio RNEA 计算的重力力矩 |
| `/debug/velocity_feedforward` | `sensor_msgs/JointState` | 200 Hz | 速度前馈量 |
| `/debug/motor_temperature` | `sensor_msgs/JointState` | 200 Hz | 电机绕组温度 (effort 字段, 单位 °C) |
| `/debug/torque_feedback` | `sensor_msgs/JointState` | 200 Hz | 电机实际力矩反馈 |
| `/debug/adaptive_kd` | `sensor_msgs/JointState` | 200 Hz | 自适应 Kd 当前值（零力矩模式下） |

使用 `ros2 topic echo /debug/motor_temperature` 进行实时监控。

---

## 3. ZeroTorqueController

ros2_control ControllerInterface 插件，实现 Pinocchio RNEA 重力补偿 + 纯阻尼控制。

### 3.1 参数

在 `el_a3_controllers.yaml` 中配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `joints` | (必填) | 参与控制的关节名列表 |
| `kd` | 1.0 | 阻尼增益（所有关节共用） |
| `urdf_path` | `""` | URDF 路径（由 launch 文件动态解析） |
| `inertia_config_path` | `""` | 标定惯量参数 YAML（由 launch 文件动态解析） |

### 3.2 Topics

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `~/gravity_torque` | `sensor_msgs/JointState` | ~20 Hz | 重力补偿力矩（每 10 个控制周期发布一次） |

### 3.3 与 arm_controller 互斥

`zero_torque_controller` 和 `arm_controller` 共享 L1-L6 关节的 command_interface（position, velocity, effort），不能同时激活。切换方法：

```bash
# 激活零力矩模式
ros2 control switch_controllers --deactivate arm_controller --activate zero_torque_controller

# 恢复轨迹控制
ros2 control switch_controllers --deactivate zero_torque_controller --activate arm_controller
```

---

## 4. 插件注册

| 插件 XML | 基类 | 插件名 |
|----------|------|--------|
| `el_a3_hardware_plugin.xml` | `hardware_interface::SystemInterface` | `el_a3_hardware/RsA3HardwareInterface` |
| `el_a3_controller_plugin.xml` | `controller_interface::ControllerInterface` | `el_a3_hardware/ZeroTorqueController` |
