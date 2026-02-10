# EL-A3 重力补偿标定程序使用说明

## 概述

本程序用于标定 EL-A3 机械臂的重力补偿参数。通过在多个关节角度采集力矩数据，拟合出每个关节的重力补偿模型：

```
τ = sin_coeff × sin(θ) + cos_coeff × cos(θ) + offset
```

## 文件说明

| 文件 | 功能 |
|------|------|
| `gravity_calibration.py` | 主标定程序 |
| `gravity_calibration_analyzer.py` | 结果分析和可视化工具 |

## 使用方法

### 1. 启动机器人控制器

```bash
# 终端 1: 启动控制器
ros2 launch el_a3_description el_a3_control.launch.py
```

### 2. 运行标定程序

```bash
# 终端 2: 运行标定
cd ~/RS/A3/scripts
python3 gravity_calibration.py
```

### 3. 标定模式

#### 交互模式（默认）
```bash
python3 gravity_calibration.py
```
显示菜单，可以选择：
- `[1]` 全自动标定 - 标定所有 6 个关节
- `[2]` 标定单个关节 - 选择特定关节标定
- `[3]` 快速标定 - 仅标定 L2, L3, L5（主要受重力影响）
- `[4]` 查看当前关节状态
- `[5]` 移动到 Home 位置

#### 命令行模式
```bash
# 全自动标定
python3 gravity_calibration.py --auto

# 快速标定 (L2, L3, L5)
python3 gravity_calibration.py --quick

# 标定指定关节 (0=L1, 1=L2, ..., 5=L6)
python3 gravity_calibration.py --joint 1   # 标定 L2
```

### 4. 分析结果

```bash
# 分析最新的标定结果
python3 gravity_calibration_analyzer.py

# 分析指定文件
python3 gravity_calibration_analyzer.py gravity_calibration_20240122_143000.json
```

## 标定流程

```
┌─────────────────────────────────────────────────────────┐
│  1. 机械臂回到 Home 点 (0, 0, 0, 0, 0, 0)              │
├─────────────────────────────────────────────────────────┤
│  2. 对每个关节 (按 L6→L5→L4→L3→L2→L1 顺序):           │
│     a. 固定其他关节在零位                               │
│     b. 在限位范围内生成 15 个均匀分布的测试角度          │
│     c. 对每个角度:                                      │
│        - 移动到目标角度                                 │
│        - 等待 2 秒稳定                                  │
│        - 采集 100 个力矩样本                            │
│        - 计算平均值和标准差                             │
│     d. 使用最小二乘法拟合 sin/cos/offset 参数          │
├─────────────────────────────────────────────────────────┤
│  3. 输出结果和 XACRO 参数                              │
│  4. 保存 JSON 文件                                     │
└─────────────────────────────────────────────────────────┘
```

## 参数配置

在 `gravity_calibration.py` 中可以修改以下参数：

```python
config = CalibrationConfig(
    num_angles=15,              # 每关节采样角度数
    samples_per_angle=100,      # 每角度采样次数
    settle_time=2.0,            # 稳定等待时间 (秒)
    motion_duration=4.0,        # 单次运动时间 (秒)
    margin_ratio=0.15,          # 限位边界余量 (15%)
)
```

## 输出结果

### 控制台输出
```
================================================================================
  EL-A3 重力补偿标定结果汇总
================================================================================

关节        sin_coeff    cos_coeff       offset       RMSE       R²
--------------------------------------------------------------------------------
L1_joint       0.0012      -0.0008       0.0231     0.0120   0.9856
L2_joint       3.4821       0.1234       0.0892     0.0450   0.9921
L3_joint       2.1456      -0.0567       0.0345     0.0380   0.9889
...
```

### JSON 文件
保存在 `scripts/gravity_calibration_YYYYMMDD_HHMMSS.json`

### XACRO 参数
复制到 `el_a3_description/urdf/el_a3_ros2_control.xacro`:

```xml
<param name="gravity_comp_L2_sin">3.4821</param>
<param name="gravity_comp_L2_cos">0.1234</param>
<param name="gravity_comp_L2_offset">0.0892</param>
```

## 标定建议

1. **环境准备**
   - 确保机械臂安装牢固
   - 避免外力干扰
   - 室温稳定

2. **预热**
   - 建议先运动 5-10 分钟让电机预热
   - 避免冷态标定

3. **顺序**
   - 从末端关节开始标定（L6→L1）
   - 减少耦合影响

4. **验证**
   - 标定完成后启用零力矩模式测试
   - 手动移动机械臂检查是否平滑

## 零力矩模式测试

标定完成后，可以使用零力矩模式验证：

```bash
# 启用零力矩模式（可手动拖动机械臂）
ros2 service call /el_a3/set_zero_torque_mode std_srvs/srv/SetBool "{data: true}"

# 禁用零力矩模式
ros2 service call /el_a3/set_zero_torque_mode std_srvs/srv/SetBool "{data: false}"
```

## 故障排除

| 问题 | 解决方案 |
|------|---------|
| 无法获取关节状态 | 检查控制器是否启动 |
| Action Server 不可用 | 检查 `/arm_controller/follow_joint_trajectory` |
| 力矩数据异常 | 检查电机是否正常，增加 settle_time |
| 拟合 R² 较低 | 增加采样点数，检查是否有外力干扰 |

## 预期结果

| 关节 | 预期 sin_coeff 范围 | 说明 |
|------|---------------------|------|
| L1 | ≈ 0 | 绕 Z 轴旋转，无重力影响 |
| L2 | 2.0 ~ 5.0 | 大臂俯仰，主要负载 |
| L3 | 1.0 ~ 3.0 | 小臂俯仰 |
| L4 | ≈ 0 | 腕部 Roll |
| L5 | 0.1 ~ 0.5 | 腕部 Pitch |
| L6 | ≈ 0 | 腕部 Yaw |
