# EL-A3 Robotic Arm ROS2 Control System

> **EL-A3** is a 6-DOF + Gripper desktop robotic arm built on ROS2 Control, driven by Robstride motors over CAN bus. It features S-curve trajectory planning, Pinocchio dynamics-based gravity compensation with automatic inertia calibration, and supports Xbox gamepad Cartesian teleoperation, master-slave teleoperation, gravity-compensated teach mode, MoveIt2 motion planning, and vision-based grasping — balancing motion smoothness with safety protection.

## Table of Contents

- [Introduction](#el-a3-robotic-arm-ros2-control-system)
- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Environment](#software-environment)
- [Packages](#packages)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Control Parameters](#control-parameters)
- [ROS2 Interfaces](#ros2-interfaces)
- [Motor Communication Protocol](#motor-communication-protocol)
- [Troubleshooting](#troubleshooting)
- [Directory Structure](#directory-structure-1)
- [中文版 (Chinese Version)](#el-a3-机械臂-ros2-控制系统)

---

## System Overview

### Key Features

- **S-Curve Trajectory Planning**: Standard 7-segment S-curve velocity profiling for smooth, jerk-free motion
- **Pinocchio Dynamics Gravity Compensation**: Full dynamics model considering all joint cascading effects
- **Automatic Inertia Calibration**: Multi-point sampling to fit link inertia parameters for precise teach mode
- **Velocity Feedforward**: Position-differential velocity feedforward with low-pass filtering to reduce vibration
- **Joint Limit Protection**: Soft-limit deceleration + hard-limit stop for safety
- **Real-Time Cartesian Control**: 50 Hz Xbox gamepad Cartesian-space teleoperation
- **MoveIt2 Integration**: Motion planning and obstacle avoidance
- **Cartesian Velocity Mapping**: Joystick input directly mapped to end-effector velocity
- **Multi-Level Smoothing**: Input smoothing + joint output smoothing + acceleration limiting to eliminate jitter
- **Singularity Protection**: Automatic detection and rejection of IK solutions causing large jumps; auto-recovery after 50 consecutive rejections
- **Collision Detection**: Self-collision and environment collision detection enabled on startup
- **Home / Zero Return**: Precise control resumption from preset positions

### Arm Specifications

| Property | Specification |
|----------|---------------|
| DOF | 6 + Gripper |
| End Effector | Customizable |
| Protocol | Proprietary CAN 2.0 (29-bit extended ID) |
| Baud Rate | 1 Mbps |
| Control Mode | MIT-like PD Control |

### Motor Configuration

| Joint | Motor ID | Model | Torque Limit | Velocity Limit | Position Limit | Direction |
|-------|----------|-------|-------------|----------------|----------------|-----------|
| L1_joint | 1 | RS00 | ±14 Nm | ±33 rad/s | ±2.79 rad (±160°) | -1 |
| L2_joint | 2 | RS00 | ±14 Nm | ±33 rad/s | -0.17\~3.14 rad (-10°\~180°) | +1 |
| L3_joint | 3 | RS00 | ±14 Nm | ±33 rad/s | -2.96\~0.17 rad (-170°\~10°) | -1 |
| L4_joint | 4 | EL05 | ±6 Nm | ±50 rad/s | ±1.75 rad (±100°) | +1 |
| L5_joint | 5 | EL05 | ±6 Nm | ±50 rad/s | ±1.75 rad (±100°) | -1 |
| L6_joint | 6 | EL05 | ±6 Nm | ±50 rad/s | ±3.14 rad (±180°) | +1 |
| Gripper | 7 | EL05 | ±0.4 Nm* | - | - | +1 |

> *Gripper uses torque control mode, ±0.4 Nm controlled via D-pad up/down

---

## Hardware Requirements

### Required

- **EL-A3 Robotic Arm** (with 6 Robstride motors)
- **CAN Adapter**: CANdle / gs_usb compatible device
- **Power Supply**: 24V/48V DC (per motor specs)
- **PC**: Ubuntu 22.04 x86_64

### Optional

- **Xbox Controller**: Wired or Bluetooth
  - Xbox One Controller
  - Xbox Series X|S Controller
  - Other XInput-compatible gamepads

---

## Software Environment

### System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **Kernel Module**: `gs_usb` (for CANdle adapter)

### Dependency Installation

```bash
# One-click install
cd scripts
sudo ./install_deps.sh
```

Or install manually:

```bash
# ROS2 Control packages
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-hardware-interface \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-joint-trajectory-controller

# MoveIt2 packages
sudo apt install ros-humble-moveit \
                 ros-humble-moveit-ros-move-group \
                 ros-humble-moveit-ros-planning-interface \
                 ros-humble-moveit-ros-visualization \
                 ros-humble-moveit-planners-ompl \
                 ros-humble-moveit-kinematics

# Tools and other dependencies
sudo apt install ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2 \
                 ros-humble-joy \
                 can-utils

# Python dependencies
pip3 install python-can scipy
```

### Xbox Bluetooth Driver (Optional)

```bash
cd scripts
./install_xpadneo.sh
```

---

## Packages

| Package | Description |
|---------|-------------|
| `el_a3_hardware` | ROS2 Control hardware interface with CAN communication driver |
| `el_a3_description` | URDF robot description, ros2_control config, controller parameters |
| `el_a3_moveit_config` | MoveIt2 motion planning configuration |
| `el_a3_teleop` | Xbox gamepad real-time Cartesian-space control |

---

## Installation

### 1. Clone / Copy Project

```bash
cd /path/to/EL-A3
```

### 2. Build Workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Set Up CAN Interface

```bash
# After connecting the CAN adapter
sudo ./scripts/setup_can.sh can0 1000000

# Verify interface status
ip link show can0
candump can0
```

### 4. Configure Gamepad (Optional)

**Wired**:
```bash
ls /dev/input/js*
```

**Bluetooth**:
```bash
./scripts/setup_bluetooth_xbox.sh
```

---

## Quick Start

### Xbox Gamepad Real-Time Control (Recommended)

**Real Hardware**:
```bash
# Terminal 1: Set up CAN
sudo ./scripts/setup_can.sh can0

# Terminal 2: Launch control system
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0
```

Or use the one-click script:
```bash
./scripts/start_real_xbox_control.sh can0
```

**Gamepad Control Mapping**:

| Button / Stick | Function |
|---------------|----------|
| Left Stick Y | X-axis translation |
| Left Stick X | Y-axis translation |
| LT / RT | Z-axis up/down |
| Right Stick X | Yaw rotation |
| Right Stick Y | Pitch rotation |
| LB / RB | Roll rotation |
| A Button | Cycle speed gear (5 gears) |
| B Button | Return to home position |
| X Button | Return to zero position |
| Y Button | Toggle gravity compensation mode (teach mode) |
| Menu Button | Toggle master-slave teleop (can0 master / can1 slave) |
| D-pad Up | Gripper close (+0.4 Nm) |
| D-pad Down | Gripper open (-0.4 Nm) |

**Speed Gears**:

| Gear | Name | Linear Speed | Angular Speed |
|------|------|-------------|---------------|
| 1 | Ultra Slow | 25 mm/s | 0.25 rad/s |
| 2 | Slow | 60 mm/s | 0.6 rad/s |
| 3 | Medium | 120 mm/s | 1.2 rad/s |
| 4 | Fast | 240 mm/s | 2.4 rad/s |
| 5 | Ultra Fast | 600 mm/s | 6 rad/s |

### Master-Slave Teleoperation Mode

Press the Xbox Menu button to toggle between Cartesian control and master-slave teleoperation. In master-slave mode, the master arm on can0 enters pure zero-torque mode (Kp=0, Kd=0) for free-hand dragging, while the slave arm on can1 follows the master's joint positions in real-time via direct CAN MIT control, including Motor 7 (gripper).

**Prerequisites**:
- Both can0 and can1 interfaces are up
- Master arm (can0) `ros2_control_node` is running
- **Do NOT** run `ros2_control_node` for the slave arm (can1) — the slave is controlled directly via CAN

**Launch**:
```bash
# 1. Set up CAN interfaces
sudo ./scripts/setup_can.sh can0 1000000
sudo ./scripts/setup_can.sh can1 1000000

# 2. Launch control system (can0 only)
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0

# 3. Press Menu button to toggle master-slave mode
```

**Workflow**:

1. **Press Menu to enable**:
   - Master arm (can0) slowly returns to zero via MoveIt
   - Slave arm (can1) returns to zero via CAN S-curve trajectory
   - After zeroing, master enters pure zero-torque mode (free dragging)
   - All 7 slave motors begin MIT position following

2. **During master-slave following**:
   - Master L1-L6: reads ROS2 `/joint_states` joint positions
   - Slave L1-L6: follows via direct CAN MIT commands (with automatic direction mapping)
   - Master Motor 7 (gripper): pure zero-torque mode (Kp=0, Kd=0)
   - Slave Motor 7 (gripper): reads master CAN feedback position and follows

3. **Press Menu again to disable**:
   - Both arms slowly return to zero simultaneously
   - After zeroing, slave motors are released (zero torque)
   - Master arm resumes normal position control

**Slave Arm Direct CAN Control Parameters**:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Kp | 80.0 | MIT position following stiffness |
| Kd | 2.0 | MIT velocity damping |
| Torque feedforward | 0.0 | No additional torque |
| Control rate | 50 Hz | Synced with gamepad control loop |

**Motor Direction Mapping** (joint frame → motor frame):

| Motor | Joint | Direction |
|-------|-------|-----------|
| Motor 1 | L1_joint | -1.0 |
| Motor 2 | L2_joint | +1.0 |
| Motor 3 | L3_joint | -1.0 |
| Motor 4 | L4_joint | +1.0 |
| Motor 5 | L5_joint | -1.0 |
| Motor 6 | L6_joint | +1.0 |
| Motor 7 | Gripper | +1.0 |

### Simulation Mode (MoveIt Demo)

No real hardware needed — uses mock hardware:

```bash
ros2 launch el_a3_moveit_config demo.launch.py
```

### Simulation + Xbox Gamepad

Test with simulated hardware and Xbox gamepad (no real arm required):

```bash
ros2 launch el_a3_teleop sim_teleop.launch.py
```

### Real Hardware + MoveIt

```bash
sudo ./scripts/setup_can.sh can0
ros2 launch el_a3_moveit_config robot.launch.py can_interface:=can0
```

### ros2_control Only (Without MoveIt)

```bash
ros2 launch el_a3_description el_a3_control.launch.py use_mock_hardware:=false can_interface:=can0
```

---

## Control Parameters

### Hardware Interface Parameters (`el_a3_ros2_control.xacro`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | can0 | CAN interface name |
| `host_can_id` | 253 (0xFD) | Host CAN ID |
| `position_kp` | 80.0 | Position PD control Kp gain |
| `position_kd` | 4.0 | Position PD control Kd gain |
| `velocity_limit` | 10.0 | Velocity limit (rad/s) |
| `velocity_filter_alpha` | 0.1 | Velocity feedforward filter coefficient (0-1) |
| `smoothing_alpha` | 0.08 | Low-pass filter coefficient (0-1) |
| `max_velocity` | 2.0 | Maximum velocity limit (rad/s) |
| `max_acceleration` | 8.0 | Maximum acceleration limit (rad/s²) |
| `max_jerk` | 50.0 | Maximum jerk limit (rad/s³) — S-curve planning |
| `s_curve_enabled` | true | Enable S-curve trajectory planning |
| `gravity_feedforward_ratio` | 0.5 | Gravity compensation feedforward ratio (0-1) |
| `limit_margin` | 0.15 | Joint limit deceleration zone (rad) |
| `limit_stop_margin` | 0.02 | Joint limit hard-stop zone (rad) |
| `can_frame_delay_us` | 50 | CAN frame send delay (μs) |

**Default Gravity Compensation Coefficients**:

| Joint | sin_coeff | Description |
|-------|-----------|-------------|
| L1 | 0.0 | Base rotation about Z-axis, no gravity effect |
| L2 | 3.5 | Upper arm pitch, primary gravity load |
| L3 | 2.0 | Forearm pitch |
| L4 | 0.0 | Wrist Roll |
| L5 | 0.3 | Wrist Pitch |
| L6 | 0.0 | Wrist Yaw |

Simplified gravity compensation formula: `τ_ff = (sin_coeff × sin(θ) + cos_coeff × cos(θ) + offset) × gravity_feedforward_ratio`

### Gravity Compensation System

The system uses a **dual-model architecture**: a simplified trigonometric model and a full Pinocchio RNEA dynamics model.

#### Architecture Overview

```
                     ┌──────────────────────────────┐
                     │    el_a3_hardware.cpp         │
                     │    write() @ 200Hz            │
                     └──────────┬───────────────────┘
                                │
                 ┌──────────────┼──────────────────┐
                 │              │                   │
          ┌──────▼──────┐  ┌───▼────────────┐  ┌──▼───────────────┐
          │ Simplified   │  │ Pinocchio RNEA │  │  Zero-Torque     │
          │ τ=A·sin+B·cos│  │ τ=RNEA(q,0,0) │  │  Kp=0, 100% comp│
          │ +C (per joint)│  │ (cascading)   │  │  Teach/Teleop    │
          └──────────────┘  └────────────────┘  └──────────────────┘
```

#### Model 1: Simplified Trigonometric Model

Each joint computed independently, ignoring other joints' effects:

```
τ_ff = (sin_coeff × sin(θ) + cos_coeff × cos(θ) + offset) × gravity_feedforward_ratio
```

#### Model 2: Pinocchio RNEA Full Dynamics Model (Recommended)

Uses Recursive Newton-Euler Algorithm (RNEA), considering all joint cascading effects:

```
τ_gravity = RNEA(model, data, q, v=0, a=0)
```

**Principle**: When velocity and acceleration are both zero, RNEA inverse dynamics output equals the gravity compensation torque for each joint. Pinocchio uses the URDF model for kinematic chain (DH parameters, link geometry), combined with calibrated inertia parameters (mass `m` and center of mass `c`), recursively computing gravity load from end-effector to base.

**Parameters to Calibrate (12 total)**:

| Link | Parameters | Description |
|------|------------|-------------|
| L2 | mass, com_x | Upper arm (2 params), primary load bearing |
| L3 | mass, com_x, com_y | Forearm (3 params) |
| L4 | mass, com_x, com_y | Wrist Roll (3 params) |
| L5 | mass, com_z | Wrist Pitch (2 params) |
| L6 | mass, com_z | End Yaw (2 params) |

> L1 rotates about Z-axis, unaffected by gravity — no calibration needed.

#### Control Mode Comparison

| Mode | Kp | Kd | Velocity | Gravity Comp. | Use Case |
|------|----|----|----------|---------------|----------|
| Normal Position | 80.0 | 4.0 | Differential feedforward | 50% | Precise position tracking |
| Zero-Torque (Gravity Comp.) | 0 | Per-joint | 0 | 100% | Gravity-compensated teaching |
| Pure Zero-Torque | 0 | 0 | 0 | 0 | Master-slave teleop (master arm) |

### Zero-Torque Mode (Teach Mode)

In zero-torque mode, position control Kp=0, retaining only damping and gravity compensation, allowing manual guidance of the arm.

#### ROS2 Service Interface

| Service | Type | Description |
|---------|------|-------------|
| `/{ns}/set_zero_torque_mode` | `std_srvs/SetBool` | Gravity-compensated zero-torque mode (teaching) |
| `/{ns}/set_pure_zero_torque_mode` | `std_srvs/SetBool` | Pure zero-torque mode (master-slave teleop) |

```bash
# Enable zero-torque mode
ros2 service call /arm1/set_zero_torque_mode std_srvs/srv/SetBool "{data: true}"

# Enable pure zero-torque mode (for teleop master arm)
ros2 service call /arm1/set_pure_zero_torque_mode std_srvs/srv/SetBool "{data: true}"

# Disable zero-torque mode
ros2 service call /arm1/set_zero_torque_mode std_srvs/srv/SetBool "{data: false}"
```

#### Per-Joint Kp/Kd Configuration

Each joint can have independently configured damping in zero-torque mode via `el_a3_ros2_control.xacro`:

```xml
<param name="zero_torque_kp_L1">0.0</param>
<param name="zero_torque_kd_L1">0.05</param>
<param name="zero_torque_kd_L2">0.125</param>  <!-- Upper arm, higher damping -->
<param name="zero_torque_kd_L3">0.15</param>
<param name="zero_torque_kd_L4">0.15</param>
<param name="zero_torque_kd_L5">0.02</param>
<param name="zero_torque_kd_L6">0.02</param>
```

| Kd Range | Feel | Use Case |
|----------|------|----------|
| 0.02\~0.05 | Light, almost no resistance | Fine teaching, small joints |
| 0.1\~0.15 | Moderate damping | General teaching |
| 0.2\~0.5 | Noticeable resistance | Safety-first, large joints |

### Inertia Parameter Calibration

Automatic inertia calibration (`scripts/inertia_calibration.py`) uses the Pinocchio dynamics model to collect torque data at multiple joint configurations and fits link mass and center-of-mass parameters via least-squares optimization.

#### Calibration Modes

```bash
# Full calibration L2-L6 (~46 points, ~10 min)
python3 scripts/inertia_calibration.py

# Quick calibration (~20 points, ~3 min)
python3 scripts/inertia_calibration.py --quick

# High-precision (~80 points, ~20 min)
python3 scripts/inertia_calibration.py --high

# Ultra-high-precision (~120 points, ~35 min)
python3 scripts/inertia_calibration.py --ultra --samples 60

# Wrist-only L4-L6 (preserves L2/L3)
python3 scripts/inertia_calibration.py --wrist

# L2-L5 combined (fixes L6 to URDF values)
python3 scripts/inertia_calibration.py --combo --samples 50
```

#### Calibration Workflow

1. **Start controller**: `ros2 launch el_a3_description el_a3_control.launch.py`
2. **Run calibration**: The program automatically moves the arm to test points for data collection
3. **Wait for completion**: Parameters are saved and the arm returns to home position
4. **Restart controller**: New parameters are loaded on restart

#### Output File

Results are saved in `el_a3_description/config/inertia_params.yaml`:

```yaml
inertia_params:
  L2:
    mass: 0.9106         # Mass (kg)
    com: [0.087, 0.0, 0.0]  # Center of mass (m)
  L3:
    mass: 0.3747
    com: [-0.080, 0.033, 0.003]
  # ... L4, L5, L6

calibration_info:
  date: "2026-02-04 18:59:19"
  num_samples: 115
  rmse: 0.0793            # Fitting error (Nm)
  r_squared: 0.9952       # Goodness of fit
```

### Controller Parameters (`el_a3_controllers.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate` | 200 Hz | Controller update rate |
| `state_publish_rate` | 200 Hz | State publish rate |
| `interpolation_method` | splines | Trajectory interpolation method |
| `goal` | 0.03 rad | Goal position tolerance |

### Xbox Teleop Parameters (`xbox_teleop.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate` | 50.0 Hz | Control loop frequency |
| `use_fast_ik_mode` | true | Use fast IK mode |
| `max_linear_velocity` | 0.15 | Maximum linear velocity (m/s) |
| `max_angular_velocity` | 1.5 | Maximum angular velocity (rad/s) |
| `joint_smoothing_alpha` | 0.15 | Joint output smoothing coefficient (0-1) |
| `max_joint_velocity` | 1.5 | Maximum single-joint velocity (rad/s) |
| `max_joint_acceleration` | 5.0 | Maximum single-joint acceleration (rad/s²) |
| `input_smoothing_factor` | 0.3 | Input smoothing filter coefficient |
| `deadzone` | 0.15 | Joystick deadzone threshold |
| `max_ik_jump_threshold` | 0.5 | Maximum allowed single-joint IK jump (rad) |
| `enable_collision_check` | true | Enable collision detection |

---

## ROS2 Interfaces

### Topics

#### Published

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | Joint states (position/velocity/effort) |
| `/robot_description` | `std_msgs/String` | latched | URDF description |
| `/target_pose` | `geometry_msgs/PoseStamped` | 50 Hz | Target end-effector pose |
| `/debug/ik_solution` | `sensor_msgs/JointState` | 50 Hz | IK solution debug info |
| `/debug/hw_command` | `sensor_msgs/JointState` | 20 Hz | Command positions from controller |
| `/debug/smoothed_command` | `sensor_msgs/JointState` | 20 Hz | Smoothed commands sent to motors |
| `/debug/gravity_torque` | `sensor_msgs/JointState` | 20 Hz | Gravity compensation torques |
| `/debug/motor_temperature` | `sensor_msgs/JointState` | 4 Hz | Motor temperature (°C) |

#### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Xbox gamepad input |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Joint trajectory command |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/el_a3/set_zero_torque_mode` | `std_srvs/SetBool` | Enable/disable zero-torque mode (teaching) |
| `/el_a3/set_pure_zero_torque_mode` | `std_srvs/SetBool` | Enable/disable pure zero-torque mode (master-slave teleop master arm) |
| `/compute_ik` | `moveit_msgs/GetPositionIK` | Inverse kinematics solver |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` | Cartesian path planning |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Joint trajectory execution |
| `/move_action` | `moveit_msgs/MoveGroup` | MoveIt motion planning |

### TF Frames

```
base_link
├── L1_joint → l1_link_urdf_asm
│   └── L2_joint → l2_l3_urdf_asm
│       └── L3_joint → l3_lnik_urdf_asm
│           └── L4_joint → l4_l5_urdf_asm
│               └── L5_joint → part_9
│                   └── L6_joint → l5_l6_urdf_asm
│                       └── end_effector
```

---

## Motor Communication Protocol

The system communicates with motors via a proprietary protocol using **MIT-like control mode** for real-time control.

### MIT-like Control Principle

Each control cycle sends the following parameters:

```
τ = Kp × (θ_target - θ_actual) + Kd × (ω_target - ω_actual) + τ_ff
```

| Parameter | Range | Description |
|-----------|-------|-------------|
| θ_target | ±12.57 rad | Target position |
| ω_target | Per motor spec | Target velocity (feedforward, position differential) |
| Kp | 0\~500 (RS00/EL05) | Position stiffness |
| Kd | 0\~5 (RS00/EL05) | Damping coefficient |
| τ_ff | Per motor spec | Feedforward torque (gravity compensation) |

### Communication Types

| Type | Function | Description |
|------|----------|-------------|
| 1 | Control command | Send position/velocity/Kp/Kd/torque |
| 2 | Motor feedback | Receive position/velocity/torque/temperature |
| 3 | Motor enable | Start motor |
| 4 | Motor stop | Stop motor |
| 6 | Set zero position | Set current position as zero |
| 18 | Parameter write | Write operating mode/position parameters |

---

## Troubleshooting

### CAN Interface Issues

```bash
# Check USB devices
lsusb | grep -i can

# Load kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe gs_usb

# Check interface
ip link show type can

# Restart interface
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Monitor CAN data
candump can0
```

### Motor Not Responding

1. Check CAN wiring and termination resistors
2. Confirm motor ID configuration (1-6)
3. Check power supply
4. Monitor with `candump`
5. Verify host CAN ID (default 253 / 0xFD)

### Xbox Controller Issues

```bash
# Check gamepad device
ls -la /dev/input/js*
jstest /dev/input/js0

# Check joy node
ros2 topic echo /joy
```

### Build Errors

```bash
# Clean and rebuild
cd ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### End-Effector Vibration

1. Adjust `position_kp` and `position_kd` parameters
2. Increase `smoothing_alpha` (smoother but slower response)
3. Check joints for mechanical backlash
4. Lower control frequency

### Vibration Suppression

The system employs a multi-level smoothing strategy:

**1. Input Smoothing**
- Parameter: `input_smoothing_factor` (default 0.3)
- First-order low-pass filter on raw joystick input

**2. Joint Output Smoothing**
- Parameter: `joint_smoothing_alpha` (default 0.15)
- Low-pass filter on IK solutions: `filtered = α × target + (1-α) × previous`

**3. Velocity Limiting**
- Parameter: `max_joint_velocity` (default 1.5 rad/s)
- Limits maximum single-joint displacement per cycle

**4. Acceleration Limiting**
- Parameter: `max_joint_acceleration` (default 5.0 rad/s²)
- Limits rate of velocity change for smooth acceleration/deceleration

**5. Singularity Protection**
- Parameter: `max_ik_jump_threshold` (default 0.5 rad)
- Detects and rejects IK solutions causing large joint jumps

---

## Directory Structure

```
EL-A3/
├── README.md                              # This document
├── el_a3_description/                     # Robot description (URDF, config, launch)
│   ├── urdf/
│   │   ├── el_a3.urdf.xacro              # URDF main file (xacro macros)
│   │   ├── el_a3.urdf                     # Compiled URDF
│   │   └── el_a3_ros2_control.xacro       # ros2_control hardware interface config
│   ├── config/
│   │   ├── el_a3_controllers.yaml         # Single-arm controller parameters
│   │   ├── multi_arm_controllers.yaml     # Multi-arm controller parameters
│   │   ├── multi_arm_config.yaml          # Multi-arm CAN and namespace config
│   │   ├── master_slave_config.yaml       # Master-slave teleop mapping config
│   │   └── inertia_params.yaml            # Calibrated inertia parameters
│   ├── launch/
│   │   ├── el_a3_control.launch.py        # Single-arm control launch
│   │   └── multi_arm_control.launch.py    # Multi-arm control launch
│   └── meshes/                            # 3D model files (STL)
│
├── el_a3_hardware/                        # ROS2 Control hardware interface
│   ├── include/el_a3_hardware/
│   │   ├── el_a3_hardware.hpp             # Hardware interface header
│   │   ├── robstride_can_driver.hpp       # CAN driver header
│   │   └── s_curve_generator.hpp          # S-curve trajectory generator
│   └── src/
│       ├── el_a3_hardware.cpp             # Hardware interface (Pinocchio gravity comp, zero-torque)
│       ├── robstride_can_driver.cpp       # CAN communication driver
│       └── s_curve_generator.cpp          # S-curve trajectory generator
│
├── el_a3_moveit_config/                   # MoveIt2 motion planning config
│   ├── config/                            # SRDF, kinematics, OMPL, joint limits
│   └── launch/
│       ├── demo.launch.py                 # Simulation demo
│       └── robot.launch.py                # Real hardware + MoveIt
│
├── ros2_ws/src/                           # ROS2 workspace (Python packages)
│   ├── el_a3_teleop/                      # Teleoperation package
│   │   ├── config/                        # Xbox / Servo parameters
│   │   ├── launch/                        # Real / Sim / Master-slave launch files
│   │   └── el_a3_teleop/                  # Node implementations
│   ├── el_a3_vision/                      # Vision grasping package
│   └── el_a3_web_ui/                      # Web visualization UI
│
├── hardware/                              # Hardware design files
│   ├── mechanical/                        # Mechanical structure models
│   │   ├── step/                          # STEP format 3D models
│   │   ├── stl/                           # STL mesh files
│   │   └── drawings/                      # Engineering drawings (DXF/DWG/PDF)
│   └── electronics/                       # PCB & circuit board files
│       ├── schematic/                     # Schematics
│       ├── pcb/                           # PCB layouts
│       ├── gerber/                        # Gerber manufacturing files
│       ├── bom/                           # Bill of Materials
│       └── datasheet/                     # Component datasheets
│
├── scripts/                               # Utility scripts and calibration tools
│   ├── inertia_calibration.py             # Pinocchio inertia calibration (recommended)
│   ├── setup_can.sh                       # CAN interface setup
│   ├── install_deps.sh                    # ROS2 dependency installation
│   ├── start_real_xbox_control.sh         # One-click Xbox real hardware launch
│   └── ...                                # More test and setup scripts
│
├── EDULITE-A3/                            # Original URDF export models (PART/STL)
└── EL_A3_urdf/                            # Early URDF version
```

---

## Script Reference

| Script | Function | Usage |
|--------|----------|-------|
| `setup_can.sh` | Set up single CAN interface | `sudo ./setup_can.sh can0 1000000` |
| `setup_multi_can.sh` | Batch set up multiple CAN interfaces | `sudo ./setup_multi_can.sh 4` |
| `install_deps.sh` | Install ROS2 dependencies | `sudo ./install_deps.sh` |
| `install_xpadneo.sh` | Install Xbox Bluetooth driver | `./install_xpadneo.sh` |
| `start_real_xbox_control.sh` | One-click Xbox real hardware launch | `./start_real_xbox_control.sh can0` |
| `inertia_calibration.py` | Pinocchio inertia calibration | `python3 inertia_calibration.py [--quick\|--high\|--ultra]` |
| `move_to_zero.py` | Move arm to zero position | `python3 move_to_zero.py` |

---

## Launch File Parameters

### `el_a3_teleop/real_teleop.launch.py`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | can0 | CAN interface name |
| `host_can_id` | 253 | Host CAN ID |
| `device` | /dev/input/js0 | Gamepad device path |

### `el_a3_moveit_config/robot.launch.py`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | can0 | CAN interface name |
| `host_can_id` | 253 | Host CAN ID |
| `use_rviz` | true | Whether to launch RViz |

### `el_a3_moveit_config/demo.launch.py`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_mock_hardware` | true | Use simulated hardware |
| `use_rviz` | true | Whether to launch RViz |

---

## License

Apache-2.0

## Contact

For questions, please contact the maintainer.

---

**Last Updated**: 2026-02-10

<br>

# EL-A3 机械臂 ROS2 控制系统

> **EL-A3** 是一款 6 自由度 + 夹爪桌面级机械臂，基于 ROS2 Control 构建，采用 CAN 总线驱动 Robstride 电机。具备 S 曲线轨迹规划、Pinocchio 动力学重力补偿与自动惯性标定能力，支持 Xbox 手柄笛卡尔遥操作、主从遥操作、拖动示教、MoveIt2 运动规划及视觉抓取，兼顾运动平滑性与安全保护。

## 📋 目录

- [简介](#el-a3-机械臂-ros2-控制系统)
- [系统概述](#系统概述)
- [硬件要求](#硬件要求)
- [软件环境](#软件环境)
- [功能包说明](#功能包说明)
- [安装配置](#安装配置)
- [快速开始](#快速开始)
- [控制参数](#控制参数)
- [ROS2 接口](#ros2-接口)
- [电机通信协议](#电机通信协议)
- [故障排除](#故障排除)
- [目录结构](#目录结构)
- [English Version](#el-a3-robotic-arm-ros2-control-system)

---

## 系统概述

### 主要特性

- **S曲线轨迹规划**: 标准7段S曲线速度规划，实现平滑无冲击运动
- **Pinocchio 动力学重力补偿**: 基于完整动力学模型的重力补偿，考虑所有关节级联效应
- **惯性参数自动标定**: 通过多点采样自动拟合各连杆惯性参数，实现精确拖动示教
- **速度前馈**: 位置差分计算速度前馈，配合低通滤波减少运动抖动
- **关节限位保护**: 软限位减速+硬限位停止，保护机械臂安全
- **实时笛卡尔控制**: 50Hz Xbox手柄笛卡尔空间实时遥控
- **MoveIt2集成**: 支持运动规划和避障功能
- **笛卡尔速度映射**: 摇杆输入直接映射为末端执行器速度
- **多级平滑滤波**: 输入平滑 + 关节输出平滑 + 加速度限制，消除抖动
- **奇异点保护**: 自动检测并拒绝导致剧烈跳变的IK解，连续拒绝50帧后自动恢复
- **碰撞检测**: 启动后自动开启自碰撞和环境碰撞检测
- **Home/零点精确回归**: 回到预设位置后从目标点精确开始控制

### 机械臂参数

| 属性 | 规格 |
|------|------|
| 自由度 | 6 DOF + 夹爪 |
| 末端执行器 | 可定制 |
| 通信协议 | 灵足时代私有协议 (CAN 2.0, 扩展帧29位ID) |
| 波特率 | 1Mbps |
| 控制模式 | 运控模式 (MIT-like PD Control) |

### 电机配置

| 关节 | 电机ID | 型号 | 力矩限制 | 速度限制 | 位置限制 | 方向 |
|------|--------|------|----------|----------|----------|------|
| L1_joint | 1 | RS00 | ±14 Nm | ±33 rad/s | ±2.79 rad (±160°) | -1 |
| L2_joint | 2 | RS00 | ±14 Nm | ±33 rad/s | -0.17~3.14 rad (-10°~180°) | +1 |
| L3_joint | 3 | RS00 | ±14 Nm | ±33 rad/s | -2.96~0.17 rad (-170°~10°) | -1 |
| L4_joint | 4 | EL05 | ±6 Nm | ±50 rad/s | ±1.75 rad (±100°) | +1 |
| L5_joint | 5 | EL05 | ±6 Nm | ±50 rad/s | ±1.75 rad (±100°) | -1 |
| L6_joint | 6 | EL05 | ±6 Nm | ±50 rad/s | ±3.14 rad (±180°) | +1 |
| 夹爪 | 7 | EL05 | ±0.4 Nm* | - | - | +1 |

> *夹爪使用力矩控制模式，±0.4Nm 通过方向键上下控制

---

## 硬件要求

### 必需硬件

- **EL-A3 机械臂** (含 6 个 Robstride 电机)
- **CAN 适配器**: CANdle / gs_usb 兼容设备
- **电源**: 24V/48V 直流电源 (根据电机规格)
- **PC**: Ubuntu 22.04 x86_64

### 可选硬件

- **Xbox 手柄**: 支持有线或蓝牙连接
  - Xbox One 控制器
  - Xbox Series X|S 控制器
  - 其他 XInput 兼容手柄

---

## 软件环境

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS 版本**: ROS 2 Humble Hawksbill
- **内核模块**: `gs_usb` (用于 CANdle 适配器)

### 依赖安装

```bash
# 一键安装所有依赖
cd /home/wy/RS/A3/scripts
sudo ./install_deps.sh
```

或手动安装：

```bash
# ROS2 Control 相关包
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-hardware-interface \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-joint-trajectory-controller

# MoveIt2 相关包
sudo apt install ros-humble-moveit \
                 ros-humble-moveit-ros-move-group \
                 ros-humble-moveit-ros-planning-interface \
                 ros-humble-moveit-ros-visualization \
                 ros-humble-moveit-planners-ompl \
                 ros-humble-moveit-kinematics

# 工具和其他依赖
sudo apt install ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2 \
                 ros-humble-joy \
                 can-utils

# Python 依赖
pip3 install python-can scipy
```

### Xbox 手柄蓝牙驱动 (可选)

```bash
# 安装 xpadneo 驱动以获得更好的蓝牙支持
cd /home/wy/RS/A3/scripts
./install_xpadneo.sh
```

---

## 功能包说明

| 功能包 | 说明 |
|--------|------|
| `el_a3_hardware` | ROS2 Control 硬件接口，实现 CAN 通信驱动 |
| `el_a3_description` | URDF 机器人描述、ros2_control 配置、控制器参数 |
| `el_a3_moveit_config` | MoveIt2 运动规划配置 |
| `el_a3_teleop` | Xbox 手柄实时笛卡尔空间控制 |

---

## 安装配置

### 1. 克隆/复制项目

```bash
# 项目已位于 /home/wy/RS/A3
cd /home/wy/RS/A3
```

### 2. 编译工作空间

```bash
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 设置 CAN 接口

```bash
# 连接 CAN 适配器后执行
sudo /home/wy/RS/A3/scripts/setup_can.sh can0 1000000

# 验证接口状态
ip link show can0
candump can0  # 监听 CAN 总线
```

### 4. 配置手柄 (可选)

**有线连接**:
```bash
# 插入 USB 后自动识别
ls /dev/input/js*
```

**蓝牙连接**:
```bash
# 使用配置脚本
./scripts/setup_bluetooth_xbox.sh
```

---

## 快速开始

### 🎮 Xbox 手柄实时控制 (推荐)

**真实硬件模式**:
```bash
# 终端1: 设置 CAN 接口
sudo ./scripts/setup_can.sh can0

# 终端2: 启动控制系统
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0
```

或使用一键脚本:
```bash
./scripts/start_real_xbox_control.sh can0
```

**手柄控制映射**:

| 按键/摇杆 | 功能 |
|-----------|------|
| 左摇杆 Y | X 方向平移 |
| 左摇杆 X | Y 方向平移 |
| LT/RT | Z 方向上下移动 |
| 右摇杆 X | Yaw 旋转 |
| 右摇杆 Y | Pitch 旋转 |
| LB/RB | Roll 旋转 |
| A 键 | 切换速度档位 (5档) |
| B 键 | 回到初始位置 (home) |
| X 键 | 回到零点位置 (所有关节归零) |
| Y 键 | 切换重力补偿模式（拖动示教） |
| Menu 键 | 切换主从遥操作（can0 主 / can1 从） |
| 方向键上 | 夹爪闭合 (+0.4 Nm) |
| 方向键下 | 夹爪张开 (-0.4 Nm) |

**速度档位**:

| 档位 | 名称 | 平移速度 | 旋转速度 |
|------|------|----------|----------|
| 1 | 超慢 | 25 mm/s | 0.25 rad/s |
| 2 | 慢速 | 60 mm/s | 0.6 rad/s |
| 3 | 中速 | 120 mm/s | 1.2 rad/s |
| 4 | 快速 | 240 mm/s | 2.4 rad/s |
| 5 | 极速 | 600 mm/s | 6 rad/s |

### 🤖 主从遥操作模式

通过 Xbox 手柄 Menu 键可在笛卡尔控制与主从遥操作之间切换。主从模式下，can0 上的主臂进入纯零力矩模式（Kp=0, Kd=0），可自由拖动；can1 上的从臂通过直接 CAN MIT 控制实时跟随主臂关节位置，包括 7 号夹爪电机。

**前提条件**:
- can0 和 can1 均已启动
- 主臂（can0）的 `ros2_control_node` 已运行
- **不要**同时运行从臂（can1）的 `ros2_control_node`，从臂由节点直接 CAN 控制

**启动方式**:
```bash
# 1. 设置 CAN 接口
sudo ./scripts/setup_can.sh can0 1000000
sudo ./scripts/setup_can.sh can1 1000000

# 2. 启动控制系统（仅 can0）
cd /home/wy/RS/A3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0

# 3. 按 Menu 键切换主从模式
```

**工作流程**:

1. **按下 Menu 键启用**：
   - 主臂（can0）通过 MoveIt 缓慢回零位
   - 从臂（can1）通过 CAN S 曲线轨迹同步回零位
   - 回零完成后，主臂进入纯零力矩模式（可拖动）
   - 从臂所有 7 个电机开始 MIT 位置跟随

2. **主从跟随运行中**：
   - 主臂 L1-L6：读取 ROS2 `/joint_states` 关节位置
   - 从臂 L1-L6：直接 CAN MIT 命令跟随（自动转换关节方向）
   - 主臂 Motor 7（夹爪）：纯零力矩模式（Kp=0, Kd=0）
   - 从臂 Motor 7（夹爪）：读取主臂 CAN 反馈位置并跟随

3. **再次按 Menu 键关闭**：
   - 两臂同步缓慢回零位
   - 回零完成后从臂电机释放（零力矩）
   - 主臂恢复正常位置控制模式

**从臂 CAN 直接控制参数**:

| 参数 | 值 | 说明 |
|------|-----|------|
| Kp | 80.0 | MIT 位置跟随刚度 |
| Kd | 2.0 | MIT 速度阻尼 |
| 力矩前馈 | 0.0 | 无额外力矩 |
| 控制频率 | 50 Hz | 与手柄控制循环同步 |

**电机方向映射**（关节坐标 → 电机坐标）:

| 电机 | 关节 | 方向系数 |
|------|------|---------|
| Motor 1 | L1_joint | -1.0 |
| Motor 2 | L2_joint | +1.0 |
| Motor 3 | L3_joint | -1.0 |
| Motor 4 | L4_joint | +1.0 |
| Motor 5 | L5_joint | -1.0 |
| Motor 6 | L6_joint | +1.0 |
| Motor 7 | 夹爪 | +1.0 |

### 仿真模式 (MoveIt Demo)

无需真实硬件，使用 mock 硬件进行测试：

```bash
ros2 launch el_a3_moveit_config demo.launch.py
```

### 仿真模式 + Xbox 手柄控制

使用仿真硬件配合 Xbox 手柄进行测试（无需真实机械臂）：

```bash
ros2 launch el_a3_teleop sim_teleop.launch.py
```

### 真实硬件 + MoveIt 控制

```bash
# 设置 CAN 接口
sudo ./scripts/setup_can.sh can0

# 启动 MoveIt 控制系统
ros2 launch el_a3_moveit_config robot.launch.py can_interface:=can0
```

### 仅启动 ros2_control (不带 MoveIt)

```bash
ros2 launch el_a3_description el_a3_control.launch.py use_mock_hardware:=false can_interface:=can0
```

---

## 控制参数

### 硬件接口参数 (`el_a3_ros2_control.xacro`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | can0 | CAN 接口名称 |
| `host_can_id` | 253 (0xFD) | 主机 CAN ID |
| `position_kp` | 80.0 | 位置 PD 控制 Kp 增益 |
| `position_kd` | 4.0 | 位置 PD 控制 Kd 增益 |
| `velocity_limit` | 10.0 | 速度限制 (rad/s) |
| `velocity_filter_alpha` | 0.1 | 速度前馈滤波系数 (0-1，减少起步反冲) |
| `smoothing_alpha` | 0.08 | 低通滤波系数 (0-1) |
| `max_velocity` | 2.0 | 最大速度限制 (rad/s) |
| `max_acceleration` | 8.0 | 最大加速度限制 (rad/s²) |
| `max_jerk` | 50.0 | 最大加加速度限制 (rad/s³) - S曲线规划 |
| `s_curve_enabled` | true | 启用S曲线轨迹规划 |
| `gravity_feedforward_ratio` | 0.5 | 重力补偿前馈比例 (0-1)，50%重力作为前馈力矩 |
| `gravity_comp_L{n}_sin` | 见下表 | 关节n重力补偿sin系数 (Nm) |
| `gravity_comp_L{n}_cos` | 0.0 | 关节n重力补偿cos系数 (Nm) |
| `gravity_comp_L{n}_offset` | 0.0 | 关节n重力补偿偏移量 (Nm) |
| `limit_margin` | 0.15 | 关节限位减速区域 (rad, ~8.6°) |
| `limit_stop_margin` | 0.02 | 关节限位硬停止区域 (rad, ~1.1°) |
| `can_frame_delay_us` | 50 | CAN帧间发送延迟 (μs，防止缓冲区拥塞) |

**重力补偿参数默认值**:

| 关节 | sin_coeff | 说明 |
|------|-----------|------|
| L1 | 0.0 | 基座绕Z轴旋转，无重力影响 |
| L2 | 3.5 | 大臂俯仰，承受主要重力负载 |
| L3 | 2.0 | 小臂俯仰 |
| L4 | 0.0 | 腕部Roll |
| L5 | 0.3 | 腕部Pitch |
| L6 | 0.0 | 腕部Yaw |

重力补偿公式 (简化模型): `τ_ff = (sin_coeff × sin(θ) + cos_coeff × cos(θ) + offset) × gravity_feedforward_ratio`

### 重力补偿系统

系统采用 **双模型架构**，支持简化三角函数模型和基于 Pinocchio 库的完整动力学 RNEA 模型。

#### 架构概览

```
                     ┌──────────────────────────────┐
                     │    el_a3_hardware.cpp         │
                     │    write() @ 200Hz            │
                     └──────────┬───────────────────┘
                                │
                 ┌──────────────┼──────────────────┐
                 │              │                   │
          ┌──────▼──────┐  ┌───▼────────────┐  ┌──▼───────────────┐
          │ 简化模型     │  │ Pinocchio RNEA │  │  零力矩模式       │
          │ τ=A·sin+B·cos│  │ τ=RNEA(q,0,0) │  │  Kp=0, 100%补偿  │
          │ +C (独立关节) │  │ (级联效应)     │  │  拖动示教/遥操作  │
          └──────────────┘  └────────────────┘  └──────────────────┘
```

#### 模型1: 简化三角函数模型

每个关节独立计算，不考虑其他关节姿态影响：

```
τ_ff = (sin_coeff × sin(θ) + cos_coeff × cos(θ) + offset) × gravity_feedforward_ratio
```

参数通过 `el_a3_ros2_control.xacro` 配置：

```xml
<param name="gravity_comp_L2_sin">3.5</param>
<param name="gravity_comp_L2_cos">0.0</param>
<param name="gravity_comp_L2_offset">0.0</param>
```

**标定工具**: `scripts/gravity_calibration.py`（旧版简化标定）

#### 模型2: Pinocchio RNEA 完整动力学模型（推荐）

使用递归牛顿-欧拉算法 (RNEA)，考虑所有关节级联效应：

```
τ_gravity = RNEA(model, data, q, v=0, a=0)
```

**核心原理**：当速度和加速度均为零时，RNEA 逆动力学的输出即为各关节所需的重力补偿力矩。Pinocchio 通过 URDF 模型获取运动学链（DH参数、连杆几何），结合标定后的惯性参数（质量 `m` 和质心位置 `c`），从末端向基座递推计算每个关节承受的重力负载。

**调用链路**：

```
on_init():
  initPinocchioModel(urdf_path)           # 从 URDF 构建 Pinocchio 模型
  loadCalibratedInertia(yaml_path)         # 从 YAML 加载标定的惯性参数
  applyCalibratedInertiaToModel()          # 覆盖 L2-L6 的 mass 和 com

write() @ 200Hz:
  q = hw_positions_                        # 读取当前关节角度
  τ = computePinocchioGravity(q)           # RNEA 计算重力力矩
    → Eigen::VectorXd tau = pinocchio::rnea(model, data, q, 0, 0)
    → gravity_torques[i] = tau[i] × direction[i]
  
  正常模式: cmd_torque = τ[i] × gravity_feedforward_ratio (50%)
  零力矩模式: cmd_torque = τ[i] × 1.0 (100%)
```

**待标定参数（共12个）**：

| 连杆 | 参数 | 说明 |
|------|------|------|
| L2 | mass, com_x | 大臂（2个参数），主要承重 |
| L3 | mass, com_x, com_y | 小臂（3个参数）|
| L4 | mass, com_x, com_y | 腕部 Roll（3个参数）|
| L5 | mass, com_z | 腕部 Pitch（2个参数）|
| L6 | mass, com_z | 末端 Yaw（2个参数）|

> L1 绕 Z 轴旋转，不受重力影响，无需标定。

#### Pinocchio 参数配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_pinocchio_gravity` | true | 启用 Pinocchio 完整动力学重力补偿 |
| `urdf_path` | el_a3.urdf | URDF 文件路径 |
| `inertia_config_path` | inertia_params.yaml | 标定后的惯性参数配置文件 |
| `gravity_feedforward_ratio` | 0.5 | 正常模式重力前馈比例 (0~1) |

#### 控制模式对比

| 模式 | Kp | Kd | 速度 | 重力补偿 | 用途 |
|------|----|----|------|----------|------|
| 正常位置控制 | 80.0 | 4.0 | 差分前馈 | 50% | 精确位置跟踪 |
| 重力补偿零力矩 | 0 | 关节独立 | 0 | 100% | 拖动示教 |
| 纯零力矩 | 0 | 0 | 0 | 0 | 主从遥操作主臂 |

### 零力矩模式（拖动示教）

零力矩模式下，位置控制 Kp=0，仅保留阻尼和重力补偿，允许手动拖动机械臂进行示教。

#### ROS2 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/{ns}/set_zero_torque_mode` | `std_srvs/SetBool` | 重力补偿零力矩模式（拖动示教）|
| `/{ns}/set_pure_zero_torque_mode` | `std_srvs/SetBool` | 纯零力矩模式（主从遥操作主臂）|

```bash
# 启用零力矩模式
ros2 service call /arm1/set_zero_torque_mode std_srvs/srv/SetBool "{data: true}"

# 启用纯零力矩模式（遥操作主臂用）
ros2 service call /arm1/set_pure_zero_torque_mode std_srvs/srv/SetBool "{data: true}"

# 关闭零力矩模式
ros2 service call /arm1/set_zero_torque_mode std_srvs/srv/SetBool "{data: false}"
```

#### 关节独立 Kp/Kd 配置

零力矩模式下各关节可独立配置阻尼，在 `el_a3_ros2_control.xacro` 中设置：

```xml
<param name="zero_torque_kp_L1">0.0</param>
<param name="zero_torque_kd_L1">0.05</param>
<param name="zero_torque_kd_L2">0.125</param>  <!-- 大臂，较高阻尼 -->
<param name="zero_torque_kd_L3">0.15</param>
<param name="zero_torque_kd_L4">0.15</param>
<param name="zero_torque_kd_L5">0.02</param>
<param name="zero_torque_kd_L6">0.02</param>
```

| Kd 范围 | 手感 | 适用场景 |
|---------|------|----------|
| 0.02~0.05 | 轻柔，几乎无阻力 | 精细示教、小关节 |
| 0.1~0.15 | 适中阻尼 | 一般示教 |
| 0.2~0.5 | 明显阻力 | 安全优先、大关节 |

#### 注意事项

1. **启动顺序**: 先启动控制器，等待电机使能后再启用零力矩模式
2. **重力补偿**: 零力矩模式依赖精确的重力补偿，建议先运行惯性参数标定
3. **安全**: 零力矩模式下机械臂可自由移动，注意防止碰撞
4. **关节限位**: 软限位保护仍然生效，接近限位时会有阻力
5. **退出模式**: 关闭零力矩模式后，机械臂会保持当前位置

### 惯性参数标定

系统提供自动惯性参数标定程序 (`scripts/inertia_calibration.py`)，通过 Pinocchio 动力学模型，在多个关节配置下采集力矩数据，使用最小二乘法拟合各连杆的质量和质心位置。

#### 标定算法

```
1. 生成测试配置（以 home 点为基准，在各关节限位范围内组合变化）
2. 逐点移动机械臂，等待稳定后采集 40 次力矩样本取平均
3. 构建优化问题:
   min Σ_点 Σ_关节 (τ_measured - τ_pinocchio(q, params))²
4. 使用 scipy L-BFGS-B 求解最优 mass 和 com 参数（有边界约束）
5. 计算 RMSE 和 R² 评估拟合质量
6. 保存到 YAML 配置文件
```

#### 标定模式

```bash
# 完整标定 L2-L6 (~46个测试点，约10分钟)
python3 scripts/inertia_calibration.py

# 快速标定 (~20个测试点，约3分钟)
python3 scripts/inertia_calibration.py --quick

# 高精度标定 (~80个测试点，约20分钟)
python3 scripts/inertia_calibration.py --high

# 超高精度标定 (~120个测试点，约35分钟)
python3 scripts/inertia_calibration.py --ultra --samples 60

# 腕部标定 L4-L6（保留 L2/L3，负载变化时用）
python3 scripts/inertia_calibration.py --wrist

# L2-L5 联合标定（固定 L6 为 URDF 值）
python3 scripts/inertia_calibration.py --combo --samples 50
```

#### 标定流程

1. **启动控制器**: `ros2 launch el_a3_description el_a3_control.launch.py`
2. **运行标定程序**: 程序自动移动机械臂到各测试点采集数据
3. **等待完成**: 标定完成后自动保存参数并返回 home 位置
4. **重启控制器**: 重启后自动加载新参数

#### 输出文件

标定结果保存在 `el_a3_description/config/inertia_params.yaml`：

```yaml
inertia_params:
  L2:
    mass: 0.9106         # 质量 (kg)
    com: [0.087, 0.0, 0.0]  # 质心位置 (m)
  L3:
    mass: 0.3747
    com: [-0.080, 0.033, 0.003]
  # ... L4, L5, L6

calibration_info:
  date: "2026-02-04 18:59:19"
  num_samples: 115
  rmse: 0.0793            # 拟合误差 (Nm)
  r_squared: 0.9952       # 拟合优度
```

#### 标定参数说明

| 关节 | 说明 | 主要影响 |
|------|------|----------|
| L2 | 大臂 | 整体重力补偿精度，最重要 |
| L3 | 小臂 | 中等负载补偿 |
| L4 | 腕部 Roll | 末端姿态相关 |
| L5 | 腕部 Pitch | 末端姿态相关 |
| L6 | 末端 Yaw | 负载变化敏感 |

### 控制器参数 (`el_a3_controllers.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `update_rate` | 200 Hz | 控制器更新频率 |
| `state_publish_rate` | 200 Hz | 状态发布频率 |
| `interpolation_method` | splines | 轨迹插值方法 |
| `goal` | 0.03 rad | 目标到达位置容差 |

### Xbox 遥控参数 (`xbox_teleop.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `update_rate` | 50.0 Hz | 控制循环频率 |
| `use_fast_ik_mode` | true | 使用快速 IK 模式 |
| `max_linear_velocity` | 0.15 | 最大线速度 (m/s) |
| `max_angular_velocity` | 1.5 | 最大角速度 (rad/s) |
| `joint_smoothing_alpha` | 0.15 | 关节输出平滑系数 (0-1，越小越平滑) |
| `max_joint_velocity` | 1.5 | 单关节最大速度 (rad/s) |
| `max_joint_acceleration` | 5.0 | 单关节最大加速度 (rad/s²) |
| `input_smoothing_factor` | 0.3 | 输入平滑滤波系数 |
| `deadzone` | 0.15 | 摇杆死区阈值 |
| `max_ik_jump_threshold` | 0.5 | 单关节最大允许跳变 (rad) |
| `singularity_warning_count` | 5 | 连续拒绝IK解警告阈值 |
| `enable_collision_check` | true | 启用碰撞检测 |

---

## ROS2 接口

### Topics

#### 发布 (Published)

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | 关节状态 (位置/速度/力矩) |
| `/robot_description` | `std_msgs/String` | latched | URDF 描述 |
| `/target_pose` | `geometry_msgs/PoseStamped` | 50 Hz | 目标末端位姿 |
| `/debug/ik_solution` | `sensor_msgs/JointState` | 50 Hz | IK 解调试信息 |
| `/debug/hw_command` | `sensor_msgs/JointState` | 20 Hz | 控制器发送的命令位置 |
| `/debug/smoothed_command` | `sensor_msgs/JointState` | 20 Hz | 平滑后发送给电机的命令 |
| `/debug/gravity_torque` | `sensor_msgs/JointState` | 20 Hz | 重力补偿力矩 |
| `/debug/motor_temperature` | `sensor_msgs/JointState` | 4 Hz | 电机温度 (°C) |

#### 订阅 (Subscribed)

| Topic | 类型 | 说明 |
|-------|------|------|
| `/joy` | `sensor_msgs/Joy` | Xbox 手柄输入 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 关节轨迹指令 |

### Services

| Service | 类型 | 说明 |
|---------|------|------|
| `/el_a3/set_zero_torque_mode` | `std_srvs/SetBool` | 启用/关闭零力矩模式（拖动示教） |
| `/el_a3/set_pure_zero_torque_mode` | `std_srvs/SetBool` | 启用/关闭纯零力矩模式（主从遥操作主臂） |
| `/compute_ik` | `moveit_msgs/GetPositionIK` | 逆运动学求解 |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` | 笛卡尔路径规划 |

### Actions

| Action | 类型 | 说明 |
|--------|------|------|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 关节轨迹执行 |
| `/move_action` | `moveit_msgs/MoveGroup` | MoveIt 运动规划 |

### TF Frames

```
base_link
├── L1_joint → l1_link_urdf_asm
│   └── L2_joint → l2_l3_urdf_asm
│       └── L3_joint → l3_lnik_urdf_asm
│           └── L4_joint → l4_l5_urdf_asm
│               └── L5_joint → part_9
│                   └── L6_joint → l5_l6_urdf_asm
│                       └── end_effector
```

---

## 电机通信协议

系统使用灵足时代私有协议与电机通信，采用 **运控模式 (MIT-like)** 进行实时控制。

### 运控模式原理

运控模式下，每个控制周期发送以下参数：

```
τ = Kp × (θ_target - θ_actual) + Kd × (ω_target - ω_actual) + τ_ff
```

| 参数 | 范围 | 说明 |
|------|------|------|
| θ_target | ±12.57 rad | 目标位置 |
| ω_target | 见电机规格 | 目标速度 (速度前馈，位置差分计算) |
| Kp | 0~500 (RS00/EL05) | 位置刚度 |
| Kd | 0~5 (RS00/EL05) | 阻尼系数 |
| τ_ff | 见电机规格 | 前馈力矩 (重力补偿) |

### 通信类型

| 类型 | 功能 | 说明 |
|------|------|------|
| 1 | 运控模式控制 | 发送位置/速度/Kp/Kd/力矩 |
| 2 | 电机反馈 | 接收位置/速度/力矩/温度 |
| 3 | 电机使能 | 启动电机 |
| 4 | 电机停止 | 停止电机 |
| 6 | 设置零位 | 设置当前位置为零点 |
| 18 | 参数写入 | 写入运行模式/位置等参数 |

详细协议说明请参考 [`电机通信协议汇总.md`](./电机通信协议汇总.md)。

---

## 故障排除

### CAN 接口问题

```bash
# 检查 USB 设备
lsusb | grep -i can

# 加载内核模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe gs_usb

# 检查接口
ip link show type can

# 重启接口
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 监听 CAN 数据
candump can0
```

### 电机无响应

1. 检查 CAN 接线和终端电阻
2. 确认电机 ID 配置正确 (1-6)
3. 检查电源供电
4. 使用 candump 监听数据
5. 检查主机 CAN ID (默认 253/0xFD)

### Xbox 手柄问题

```bash
# 检查手柄设备
ls -la /dev/input/js*
jstest /dev/input/js0

# 检查 joy 节点
ros2 topic echo /joy
```

### 编译错误

```bash
# 清理并重新编译
cd /home/wy/RS/A3/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 末端抖动问题

1. 调整 `position_kp` 和 `position_kd` 参数
2. 增加 `smoothing_alpha` 值 (更平滑但响应变慢)
3. 检查关节是否有机械间隙
4. 降低控制频率

### 抖动抑制机制

系统采用多级平滑策略消除机械臂运动抖动：

**1. 输入平滑 (Input Smoothing)**
- 参数: `input_smoothing_factor` (默认 0.3)
- 对摇杆原始输入进行一阶低通滤波
- 消除手抖和传感器噪声

**2. 关节输出平滑 (Joint Output Smoothing)**
- 参数: `joint_smoothing_alpha` (默认 0.15)
- 对IK求解结果进行低通滤波
- 公式: `filtered = α × target + (1-α) × previous`

**3. 速度限制 (Velocity Limiting)**
- 参数: `max_joint_velocity` (默认 1.5 rad/s)
- 限制单关节每周期最大位移
- 防止IK解跳变导致的急速运动

**4. 加速度限制 (Acceleration Limiting)**
- 参数: `max_joint_acceleration` (默认 5.0 rad/s²)
- 限制速度变化率，确保平滑加减速
- 避免电机力矩突变

**5. 奇异点保护 (Singularity Protection)**
- 参数: `max_ik_jump_threshold` (默认 0.5 rad)
- 检测并拒绝导致关节大幅跳变的IK解
- 在奇异点附近保持稳定

---

## 目录结构

```
/home/wy/RS/A3/
├── README.md                          # 本文档
├── 电机通信协议汇总.md                  # 电机通信协议详细说明
│
├── el_a3_description/                 # 机器人描述包（URDF、配置、Launch）
│   ├── urdf/
│   │   ├── el_a3.urdf.xacro              # URDF 主文件（宏定义）
│   │   ├── el_a3.urdf                     # 编译后的 URDF
│   │   └── el_a3_ros2_control.xacro       # ros2_control 硬件接口配置
│   ├── config/
│   │   ├── el_a3_controllers.yaml         # 单臂控制器参数
│   │   ├── multi_arm_controllers.yaml     # 多臂控制器参数
│   │   ├── multi_arm_config.yaml          # 多臂 CAN 接口和命名空间配置
│   │   ├── master_slave_config.yaml       # 主从遥操作映射配置
│   │   ├── inertia_params.yaml            # 标定后的惯性参数（arm1/通用）
│   │   ├── inertia_params_arm2.yaml       # arm2 独立标定的惯性参数
│   │   └── el_a3_view.rviz               # RViz 可视化配置
│   ├── launch/
│   │   ├── el_a3_control.launch.py        # 单臂控制系统启动
│   │   └── multi_arm_control.launch.py    # 多臂控制系统启动
│   └── meshes/                            # 3D 模型文件 (STL)
│
├── el_a3_hardware/                    # ROS2 Control 硬件接口包
│   ├── include/el_a3_hardware/
│   │   ├── el_a3_hardware.hpp             # 硬件接口头文件
│   │   ├── robstride_can_driver.hpp       # CAN 驱动头文件
│   │   └── s_curve_generator.hpp          # S曲线轨迹生成器
│   ├── src/
│   │   ├── el_a3_hardware.cpp             # 硬件接口实现（Pinocchio重力补偿、零力矩模式）
│   │   ├── robstride_can_driver.cpp       # CAN 通信驱动实现
│   │   └── s_curve_generator.cpp          # S曲线轨迹生成器实现
│   └── el_a3_hardware_plugin.xml          # 插件描述
│
├── el_a3_moveit_config/               # MoveIt2 运动规划配置包
│   ├── config/
│   │   ├── el_a3.srdf                     # 语义机器人描述
│   │   ├── kinematics.yaml                # 运动学求解器配置
│   │   ├── joint_limits.yaml              # 关节限制
│   │   ├── ompl_planning.yaml             # OMPL 规划器配置
│   │   ├── moveit_controllers.yaml        # MoveIt 控制器配置
│   │   └── servo_config.yaml              # MoveIt Servo 配置
│   └── launch/
│       ├── demo.launch.py                 # 仿真演示
│       └── robot.launch.py                # 真实硬件 + MoveIt
│
├── ros2_ws/src/                       # ROS2 工作空间（Python 包）
│   ├── el_a3_teleop/                  # 遥操作包
│   │   ├── config/
│   │   │   ├── xbox_teleop.yaml           # Xbox 手柄笛卡尔控制参数
│   │   │   └── xbox_servo_teleop.yaml     # Xbox + MoveIt Servo 参数
│   │   ├── launch/
│   │   │   ├── real_teleop.launch.py      # 真实硬件 Xbox 遥控
│   │   │   ├── sim_teleop.launch.py       # 仿真环境遥控
│   │   │   ├── complete_teleop.launch.py  # 完整遥控启动
│   │   │   ├── master_slave.launch.py     # 主从遥操作启动
│   │   │   └── xbox_servo_teleop.launch.py # Xbox Servo 模式
│   │   └── el_a3_teleop/
│   │       ├── xbox_teleop_node.py        # Xbox 笛卡尔控制节点
│   │       ├── master_slave_node.py       # 主从遥操作节点（一对多/多对多）
│   │       └── xbox_servo_node.py         # Xbox Servo 控制节点
│   │
│   ├── el_a3_web_ui/                  # Web 可视化控制界面
│   │   ├── el_a3_web_ui/
│   │   │   ├── web_server.py              # Flask Web 服务器
│   │   │   └── ros2_bridge.py             # ROS2 桥接
│   │   ├── templates/index.html           # Web 页面
│   │   └── static/                        # 前端资源 (JS/CSS/STL)
│
├── scripts/                           # 实用脚本与标定工具
│   ├── 标定工具/
│   │   ├── inertia_calibration.py             # Pinocchio 惯性参数标定（推荐）
│   │   ├── pinocchio_gravity_calibration.py   # Pinocchio 重力标定
│   │   ├── gravity_calibration.py             # 简化三角函数重力标定（旧版）
│   │   ├── gravity_calibration_analyzer.py    # 标定数据分析工具
│   │   └── GRAVITY_CALIBRATION_README.md      # 旧版标定使用说明
│   ├── 遥操作/
│   │   ├── teleop_master_slave.py             # 独立主从遥操作脚本
│   │   ├── test_teleop_can0_can1.py           # 双臂遥操作测试
│   │   └── test_teleop_can1_master.py         # can1 主臂遥操作测试
│   ├── 环境配置/
│   │   ├── setup_can.sh                       # CAN 接口设置
│   │   ├── setup_multi_can.sh                 # 多 CAN 接口批量设置
│   │   ├── install_deps.sh                    # ROS2 依赖安装
│   │   ├── install_xpadneo.sh                 # Xbox 蓝牙驱动安装
│   │   └── setup_bluetooth_xbox.sh            # 蓝牙手柄配置
│   ├── 启动脚本/
│   │   ├── start_real_xbox_control.sh         # 一键启动 Xbox 实机控制
│   │   ├── start_teleop.sh                    # 遥操作启动脚本
│   │   └── start_web_ui.sh                    # Web UI 启动
│   ├── 测试工具/
│   │   ├── move_to_zero.py                    # 移动到零位
│   │   ├── simple_motion_test.py              # 简单运动测试
│   │   ├── zero_test.py                       # 零位测试
│   │   ├── test_kp_values.py                  # Kp 参数测试
│   │   └── test_single_move.py                # 单步运动测试
│   └── foxglove_bridge.service                # Foxglove 远程可视化服务
│
├── hardware/                          # 硬件设计资料
│   ├── mechanical/                       # 机械臂结构模型
│   │   ├── step/                            # STEP 格式 3D 模型
│   │   ├── stl/                             # STL 格式网格文件
│   │   └── drawings/                        # 工程图纸 (DXF/DWG/PDF)
│   └── electronics/                      # 电路板设计文件
│       ├── schematic/                       # 原理图
│       ├── pcb/                             # PCB 布局
│       ├── gerber/                          # Gerber 生产文件
│       ├── bom/                             # 物料清单
│       └── datasheet/                       # 元器件数据手册
│
├── EDULITE-A3/                        # 原始 URDF 导出模型 (PART/STL)
├── EL_A3_urdf/                        # 早期 URDF 版本
│
├── ACTIVATE_XBOX_CONTROLLER.md        # Xbox 手柄激活说明
├── XBOX_CONTROL_SETUP.md              # Xbox 控制详细设置
├── XBOX_HOW_TO_USE.md                 # Xbox 使用指南
├── BLUETOOTH_XBOX_SETUP.md            # 蓝牙设置指南
└── XBOX_QUICK_FIX.md                  # 快速修复指南
```

---

## 脚本使用说明

| 脚本 | 功能 | 使用方法 |
|------|------|----------|
| `setup_can.sh` | 设置单个 CAN 接口 | `sudo ./setup_can.sh can0 1000000` |
| `setup_multi_can.sh` | 批量设置多个 CAN | `sudo ./setup_multi_can.sh 4` |
| `install_deps.sh` | 安装 ROS2 依赖 | `sudo ./install_deps.sh` |
| `install_xpadneo.sh` | 安装 Xbox 蓝牙驱动 | `./install_xpadneo.sh` |
| `setup_bluetooth_xbox.sh` | 配置蓝牙手柄 | `./setup_bluetooth_xbox.sh` |
| `start_real_xbox_control.sh` | 一键启动 Xbox 实机控制 | `./start_real_xbox_control.sh can0` |
| `start_teleop.sh` | 启动遥操作 | `./start_teleop.sh` |
| `inertia_calibration.py` | Pinocchio 惯性参数标定 | `python3 inertia_calibration.py [--quick\|--high\|--ultra\|--wrist\|--combo]` |
| `teleop_master_slave.py` | 独立主从遥操作 | `python3 teleop_master_slave.py` |
| `test_teleop_can0_can1.py` | 双臂遥操作测试 | `python3 test_teleop_can0_can1.py` |
| `move_to_zero.py` | 移动机械臂到零位 | `python3 move_to_zero.py` |
| `simple_motion_test.py` | 简单运动测试 | `python3 simple_motion_test.py` |

---

## Launch 文件参数

### `el_a3_teleop/real_teleop.launch.py`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | can0 | CAN 接口名称 |
| `host_can_id` | 253 | 主机 CAN ID |
| `device` | /dev/input/js0 | 手柄设备路径 |

### `el_a3_moveit_config/robot.launch.py`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | can0 | CAN 接口名称 |
| `host_can_id` | 253 | 主机 CAN ID |
| `use_rviz` | true | 是否启动 RViz |

### `el_a3_moveit_config/demo.launch.py`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_mock_hardware` | true | 使用仿真硬件 |
| `use_rviz` | true | 是否启动 RViz |

---

## 许可证

Apache-2.0

## 联系方式

如有问题，请联系维护者。

---

**最后更新 / Last Updated**: 2026-02-10
