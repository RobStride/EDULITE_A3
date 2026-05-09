/**
 * @file el_a3_hardware.cpp
 * @brief EL-A3 机械臂 ROS2 Control 硬件接口实现
 */

#include "el_a3_hardware/el_a3_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <time.h>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace el_a3_hardware
{

RsA3HardwareInterface::RsA3HardwareInterface()
  : can_interface_("can0")
  , host_can_id_(0xFD)
  , position_kp_(100.0)
  , position_kd_(4.0)
  , velocity_limit_(10.0)
  , control_mode_(ControlMode::POSITION)
  , use_mock_hardware_(false)
  
  , zero_torque_kd_(1.0)
  , gravity_comp_enabled_(false)
  , gravity_feedforward_ratio_(1.0)
  , use_pinocchio_gravity_(false)
  , pinocchio_initialized_(false)
  , use_calibrated_inertia_(false)
  , limit_margin_(0.15)          // 约在 ~15° 处开始减速（≈8.6°）
  , limit_stop_margin_(0.02)     // 约在 ~1° 处硬停止（≈1.1°）
  , limit_decel_factor_(0.3)     // 减速到 30%
  
{
}

RsA3HardwareInterface::~RsA3HardwareInterface()
{
  on_shutdown(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 解析参数
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  }
  if (info_.hardware_parameters.count("host_can_id")) {
    host_can_id_ = std::stoi(info_.hardware_parameters.at("host_can_id"));
  }
  if (info_.hardware_parameters.count("position_kp")) {
    position_kp_ = std::stod(info_.hardware_parameters.at("position_kp"));
  }
  if (info_.hardware_parameters.count("position_kd")) {
    position_kd_ = std::stod(info_.hardware_parameters.at("position_kd"));
  }
  if (info_.hardware_parameters.count("velocity_limit")) {
    velocity_limit_ = std::stod(info_.hardware_parameters.at("velocity_limit"));
  }
  if (info_.hardware_parameters.count("use_mock_hardware")) {
    use_mock_hardware_ = info_.hardware_parameters.at("use_mock_hardware") == "true";
  }

  // 解析关节配置
  if (!parseJointConfig(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 分配状态与指令向量
  size_t num_joints = joint_configs_.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_temperatures_.resize(num_joints, 0.0);
  filtered_torque_feedback_.resize(num_joints, 0.0);
  hw_commands_positions_.resize(num_joints, 0.0);
  hw_commands_velocities_.resize(num_joints, 0.0);
  hw_commands_efforts_.resize(num_joints, 0.0);
  
  // 初始化位置指令平滑滤波（含速度/加速度/加加速度限制 - S 曲线规划）
  smoothed_positions_.resize(num_joints, 0.0);
  smoothed_velocities_.resize(num_joints, 0.0);
  smoothed_accelerations_.resize(num_joints, 0.0);  // 用于 S 曲线规划
  
  // 初始化速度前馈相关变量
  last_cmd_positions_.resize(num_joints, 0.0);         // 上一周期指令位置
  last_hw_commands_positions_.resize(num_joints, 0.0); // 上一帧 hw_commands（用于检测指令更新）
  cmd_velocities_.resize(num_joints, 0.0);             // 计算得到的指令速度
  filtered_cmd_velocities_.resize(num_joints, 0.0);    // 一阶滤波后的指令速度
  velocity_ff_stage2_.resize(num_joints, 0.0);         // 二阶滤波中间量
  vel_ma_buffer_.resize(num_joints, {0.0, 0.0, 0.0, 0.0});  // 4-sample MA circular buffer
  vel_ma_idx_.resize(num_joints, 0);                         // MA buffer write index
  velocity_settle_counter_.resize(num_joints, 0);            // 速度稳定计数器
  velocity_filter_alpha_ = 0.3;                        // 速度滤波系数
  
  // 默认参数
  smoothing_alpha_ = 0.8;       // 平滑系数（0.8 = 近直通，teleop 层已有充分平滑）
  max_velocity_ = 2.0;          // 最大速度 2 rad/s
  max_acceleration_ = 8.0;      // 最大加速度 8 rad/s²
  
  control_period_ = 0.005;      // 默认 200Hz -> 5ms
  first_command_ = true;
  gravity_feedforward_ratio_ = 1.0;  // 默认 100% 重力补偿前馈（与零力矩模式一致）
  
  
  // 从参数读取平滑系数
  if (info_.hardware_parameters.count("smoothing_alpha")) {
    smoothing_alpha_ = std::stod(info_.hardware_parameters.at("smoothing_alpha"));
    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.01, 1.0);
  }
  
  // 从参数读取速度上限
  if (info_.hardware_parameters.count("max_velocity")) {
    max_velocity_ = std::stod(info_.hardware_parameters.at("max_velocity"));
  }
  
  // 从参数读取加速度上限
  if (info_.hardware_parameters.count("max_acceleration")) {
    max_acceleration_ = std::stod(info_.hardware_parameters.at("max_acceleration"));
  }
  
  
  
  
  // 从参数读取重力补偿前馈比例
  if (info_.hardware_parameters.count("gravity_feedforward_ratio")) {
    gravity_feedforward_ratio_ = std::stod(info_.hardware_parameters.at("gravity_feedforward_ratio"));
    gravity_feedforward_ratio_ = std::clamp(gravity_feedforward_ratio_, 0.0, 1.0);
  }
  
  // 从参数读取关节限位保护参数
  if (info_.hardware_parameters.count("limit_margin")) {
    limit_margin_ = std::stod(info_.hardware_parameters.at("limit_margin"));
  }
  if (info_.hardware_parameters.count("limit_stop_margin")) {
    limit_stop_margin_ = std::stod(info_.hardware_parameters.at("limit_stop_margin"));
  }
  if (info_.hardware_parameters.count("limit_decel_factor")) {
    limit_decel_factor_ = std::stod(info_.hardware_parameters.at("limit_decel_factor"));
    limit_decel_factor_ = std::clamp(limit_decel_factor_, 0.0, 1.0);
  }
  
  // 初始化关节限位保护状态
  joint_at_limit_.resize(num_joints, false);
  limit_warn_counter_.resize(num_joints, 0);

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "已初始化：%zu 个关节，CAN 接口：%s", num_joints, can_interface_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  max_vel=%.1f rad/s，max_acc=%.1f rad/s²",
              max_velocity_, max_acceleration_);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  PID：Kp=%.1f，Kd=%.1f，重力补偿前馈比例=%.0f%%",
              position_kp_, position_kd_, gravity_feedforward_ratio_ * 100.0);
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "关节限位保护：margin=%.3f rad，stop_margin=%.3f rad，decel_factor=%.2f",
              limit_margin_, limit_stop_margin_, limit_decel_factor_);

  // 初始化调试发布器节点
  debug_node_ = rclcpp::Node::make_shared("el_a3_hw_debug");
  hw_cmd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/hw_command", 10);
  smoothed_cmd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/smoothed_command", 10);
  gravity_torque_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/gravity_torque", 10);
  velocity_ff_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/velocity_feedforward", 10);
  temperature_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/motor_temperature", 10);
  torque_feedback_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/torque_feedback", 10);
  adaptive_kd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/adaptive_kd", 10);
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "调试发布器已创建：/debug/hw_command, /debug/smoothed_command, /debug/gravity_torque, "
              "/debug/velocity_feedforward, /debug/motor_temperature, /debug/torque_feedback");

  // ============ 初始化重力补偿参数 ============
  gravity_params_.resize(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    gravity_params_[i] = {0.0, 0.0, 0.0};  // Default to 0
  }
  
  // 从参数读取重力补偿（如果存在）
  // 格式：gravity_comp_L1_sin, gravity_comp_L1_cos, gravity_comp_L1_offset 等
  std::vector<std::string> joint_prefixes = {"L1", "L2", "L3", "L4", "L5", "L6"};
  for (size_t i = 0; i < num_joints && i < joint_prefixes.size(); ++i) {
    std::string prefix = "gravity_comp_" + joint_prefixes[i];
    if (info_.hardware_parameters.count(prefix + "_sin")) {
      gravity_params_[i].sin_coeff = std::stod(info_.hardware_parameters.at(prefix + "_sin"));
    }
    if (info_.hardware_parameters.count(prefix + "_cos")) {
      gravity_params_[i].cos_coeff = std::stod(info_.hardware_parameters.at(prefix + "_cos"));
    }
    if (info_.hardware_parameters.count(prefix + "_offset")) {
      gravity_params_[i].offset = std::stod(info_.hardware_parameters.at(prefix + "_offset"));
    }
  }
  
  // 检查是否存在非零重力补偿参数
  for (size_t i = 0; i < num_joints; ++i) {
    if (gravity_params_[i].sin_coeff != 0.0 || 
        gravity_params_[i].cos_coeff != 0.0 || 
        gravity_params_[i].offset != 0.0) {
      gravity_comp_enabled_ = true;
      break;
    }
  }
  
  if (gravity_comp_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "重力补偿已启用，参数如下：");
    for (size_t i = 0; i < num_joints && i < joint_prefixes.size(); ++i) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  %s: sin=%.4f, cos=%.4f, offset=%.4f",
                  joint_prefixes[i].c_str(),
                  gravity_params_[i].sin_coeff,
                  gravity_params_[i].cos_coeff,
                  gravity_params_[i].offset);
    }
  }
  
  // 读取零力矩模式的 Kd 参数
  if (info_.hardware_parameters.count("zero_torque_kd")) {
    zero_torque_kd_ = std::stod(info_.hardware_parameters.at("zero_torque_kd"));
  }
  
  // ============ 速度自适应 Kd 参数 ============
  if (info_.hardware_parameters.count("adaptive_kd_enabled")) {
    adaptive_kd_enabled_ = (info_.hardware_parameters.at("adaptive_kd_enabled") == "true");
  }
  if (info_.hardware_parameters.count("zero_torque_kd_min")) {
    zero_torque_kd_min_ = std::stod(info_.hardware_parameters.at("zero_torque_kd_min"));
  }
  if (info_.hardware_parameters.count("zero_torque_kd_max")) {
    zero_torque_kd_max_ = std::stod(info_.hardware_parameters.at("zero_torque_kd_max"));
  }
  if (info_.hardware_parameters.count("kd_velocity_ref")) {
    kd_velocity_ref_ = std::stod(info_.hardware_parameters.at("kd_velocity_ref"));
  }
  if (info_.hardware_parameters.count("kd_smoothing_alpha")) {
    kd_smoothing_alpha_ = std::stod(info_.hardware_parameters.at("kd_smoothing_alpha"));
  }
  
  // Initialize per-joint adaptive Kd state to kd_max (safe side)
  adaptive_kd_values_.resize(joint_configs_.size(), zero_torque_kd_max_);
  
  if (adaptive_kd_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "速度自适应 Kd 已启用: kd_min=%.4f, kd_max=%.4f, v_ref=%.2f, alpha=%.2f",
                zero_torque_kd_min_, zero_torque_kd_max_, kd_velocity_ref_, kd_smoothing_alpha_);
  }
  
  // ============ Pinocchio 动力学模型初始化 ============
  // 读取 URDF 路径
  if (info_.hardware_parameters.count("urdf_path")) {
    urdf_path_ = info_.hardware_parameters.at("urdf_path");
    
    // 尝试初始化 Pinocchio
    if (initPinocchioModel(urdf_path_)) {
      // 检查是否启用 Pinocchio 重力补偿
      if (info_.hardware_parameters.count("use_pinocchio_gravity")) {
        use_pinocchio_gravity_ = (info_.hardware_parameters.at("use_pinocchio_gravity") == "true");
      }
      
      // 读取惯量缩放因子（用于标定微调）
      inertia_scale_params_.resize(joint_configs_.size());
      for (size_t i = 0; i < joint_configs_.size(); ++i) {
        inertia_scale_params_[i].mass_scale = 1.0;
        inertia_scale_params_[i].com_x_offset = 0.0;
        inertia_scale_params_[i].com_y_offset = 0.0;
        inertia_scale_params_[i].com_z_offset = 0.0;
        
        std::string prefix = "inertia_scale_L" + std::to_string(i + 1);
        if (info_.hardware_parameters.count(prefix + "_mass")) {
          inertia_scale_params_[i].mass_scale = std::stod(info_.hardware_parameters.at(prefix + "_mass"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_x")) {
          inertia_scale_params_[i].com_x_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_x"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_y")) {
          inertia_scale_params_[i].com_y_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_y"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_z")) {
          inertia_scale_params_[i].com_z_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_z"));
        }
      }
      
      // 读取惯量配置文件路径
      if (info_.hardware_parameters.count("inertia_config_path")) {
        inertia_config_path_ = info_.hardware_parameters.at("inertia_config_path");
        
        // 加载标定后的惯量参数
        if (loadCalibratedInertia(inertia_config_path_)) {
          // 应用到 Pinocchio 模型
          applyCalibratedInertiaToModel();
          use_calibrated_inertia_ = true;
          RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                      "已从以下路径加载标定惯量参数：%s", inertia_config_path_.c_str());
        }
      }
      
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Pinocchio 重力补偿：%s（标定惯量：%s）",
                  use_pinocchio_gravity_ ? "启用" : "禁用",
                  use_calibrated_inertia_ ? "是" : "否");
    }
  }
  
  // Dynamic parameters for runtime tuning
  debug_node_->declare_parameter("position_kp", position_kp_);
  debug_node_->declare_parameter("position_kd", position_kd_);
  debug_node_->declare_parameter("gravity_feedforward_ratio", gravity_feedforward_ratio_);
  debug_node_->declare_parameter("zero_torque_kd", zero_torque_kd_);
  debug_node_->declare_parameter("smoothing_alpha", smoothing_alpha_);
  debug_node_->declare_parameter("adaptive_kd_enabled", adaptive_kd_enabled_);
  debug_node_->declare_parameter("zero_torque_kd_min", zero_torque_kd_min_);
  debug_node_->declare_parameter("zero_torque_kd_max", zero_torque_kd_max_);
  debug_node_->declare_parameter("kd_velocity_ref", kd_velocity_ref_);
  debug_node_->declare_parameter("kd_smoothing_alpha", kd_smoothing_alpha_);

  param_callback_handle_ = debug_node_->add_on_set_parameters_callback(
    std::bind(&RsA3HardwareInterface::onParameterChange, this, std::placeholders::_1));

  // Create wall timers on debug_node_ for periodic publishing (off the write() hot path)
  debug_timer_4hz_ = debug_node_->create_wall_timer(
    std::chrono::milliseconds(250),
    std::bind(&RsA3HardwareInterface::debugPublish4Hz, this));
  debug_timer_10hz_ = debug_node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&RsA3HardwareInterface::debugPublish10Hz, this));
  debug_timer_20hz_ = debug_node_->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&RsA3HardwareInterface::debugPublish20Hz, this));

  // Start dedicated spin thread for debug_node_ (timers + parameter callbacks)
  debug_thread_running_ = true;
  debug_spin_thread_ = std::thread([this]() {
    while (debug_thread_running_ && rclcpp::ok()) {
      rclcpp::spin_some(debug_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Debug publishing thread started (4/10/20 Hz timers, off write() hot path)");

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Zero-torque mode: use controller_manager switch_controller "
              "(arm_controller <-> zero_torque_controller). fixed_Kd=%.1f, adaptive=%s",
              zero_torque_kd_, adaptive_kd_enabled_ ? "ON" : "OFF");

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RsA3HardwareInterface::parseJointConfig(const hardware_interface::HardwareInfo& info)
{
  joint_configs_.clear();
  
  for (const auto& joint : info.joints) {
    JointConfig config;
    config.name = joint.name;
    
    // Parse motor_id
    if (joint.parameters.count("motor_id")) {
      config.motor_id = std::stoi(joint.parameters.at("motor_id"));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "关节 %s 缺少 motor_id 参数", joint.name.c_str());
      return false;
    }
    
    // Parse motor_type
    if (joint.parameters.count("motor_type")) {
      std::string type_str = joint.parameters.at("motor_type");
      if (type_str == "RS00") {
        config.motor_type = MotorType::RS00;
      } else if (type_str == "EL05") {
        config.motor_type = MotorType::EL05;
      } else if (type_str == "RS05") {
        config.motor_type = MotorType::RS05;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                     "关节 %s 的 motor_type 未知：%s", joint.name.c_str(), type_str.c_str());
        return false;
      }
    } else {
      // Default based on motor_id: 1-3 = RS00, 4-6 = EL05
      config.motor_type = (config.motor_id <= 3) ? MotorType::RS00 : MotorType::EL05;
    }
    
    // Parse offset
    if (joint.parameters.count("position_offset")) {
      config.position_offset = std::stod(joint.parameters.at("position_offset"));
    } else {
      config.position_offset = 0.0;
    }
    
    // Parse direction
    if (joint.parameters.count("direction")) {
      config.direction = std::stod(joint.parameters.at("direction"));
    } else {
      config.direction = 1.0;
    }
    
    // Parse joint limits (from parameters or command_interface)
    config.lower_limit = -6.28;  // Default ±360°
    config.upper_limit = 6.28;
    
    if (joint.parameters.count("lower_limit")) {
      config.lower_limit = std::stod(joint.parameters.at("lower_limit"));
    }
    if (joint.parameters.count("upper_limit")) {
      config.upper_limit = std::stod(joint.parameters.at("upper_limit"));
    }
    
    // Try to get limits from command_interface (ros2_control standard method)
    for (const auto& cmd_if : joint.command_interfaces) {
      if (cmd_if.name == "position") {
        if (cmd_if.min != cmd_if.max) {  // Valid limits
          config.lower_limit = std::stod(cmd_if.min);
          config.upper_limit = std::stod(cmd_if.max);
        }
        break;
      }
    }
    
    // Parse joint-specific Kp/Kd (0 means use global value)
    config.kp = 0.0;
    config.kd = 0.0;
    if (joint.parameters.count("kp")) {
      config.kp = std::stod(joint.parameters.at("kp"));
    }
    if (joint.parameters.count("kd")) {
      config.kd = std::stod(joint.parameters.at("kd"));
    }
    
    // Parse per-joint adaptive Kd range (0 = use global default)
    config.zero_torque_kd_min = 0.0;
    config.zero_torque_kd_max = 0.0;
    if (joint.parameters.count("zero_torque_kd_min")) {
      config.zero_torque_kd_min = std::stod(joint.parameters.at("zero_torque_kd_min"));
    }
    if (joint.parameters.count("zero_torque_kd_max")) {
      config.zero_torque_kd_max = std::stod(joint.parameters.at("zero_torque_kd_max"));
    }
    
    joint_configs_.push_back(config);
    
    if (config.kp > 0.0 || config.kd > 0.0) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "关节 %s：motor_id=%d，type=%s，dir=%.1f，限位=[%.1f°~%.1f°]，Kp=%.0f，Kd=%.1f",
                  config.name.c_str(), config.motor_id,
                  motorTypeName(config.motor_type),
                  config.direction,
                  config.lower_limit * 180.0 / M_PI, config.upper_limit * 180.0 / M_PI,
                  config.kp, config.kd);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "关节 %s：motor_id=%d，type=%s，dir=%.1f，限位=[%.1f°~%.1f°]",
                  config.name.c_str(), config.motor_id,
                  motorTypeName(config.motor_type),
                  config.direction,
                  config.lower_limit * 180.0 / M_PI, config.upper_limit * 180.0 / M_PI);
    }
  }
  
  return true;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "使用 mock 硬件：跳过 CAN 初始化");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 创建并初始化 CAN 驱动
  can_driver_ = std::make_unique<RobstrideCanDriver>(can_interface_, host_can_id_);
  
  if (!can_driver_->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "CAN 驱动初始化失败：%s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 为每个关节设置电机型号（添加延时避免 CAN 缓冲区溢出）
  for (const auto& config : joint_configs_) {
    can_driver_->setMotorType(config.motor_id, config.motor_type);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Add delay
  }

  // 启动接收线程
  can_driver_->startReceiveThread();

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "硬件已配置完成：%s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (can_driver_) {
    can_driver_->stopReceiveThread();
    can_driver_->close();
    can_driver_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "硬件资源已清理");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Mock 硬件已激活");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ============ 步骤 0：清除所有电机故障 ============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "正在清除电机故障...");
  for (const auto& config : joint_configs_) {
    can_driver_->disableMotor(config.motor_id, true);  // clear_fault=true
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  // ============ 步骤 1：使能所有电机（Kp=0，无位置控制） ============
  // 电机在失能状态不会回传反馈，因此必须先使能
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "正在以软模式使能电机（Kp=0）...");
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    
    // 1) 先停止电机（不清故障，前面已清过）
    can_driver_->disableMotor(config.motor_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
    // 2) 设置为运控模式（run_mode = 0）
    if (!can_driver_->setRunMode(config.motor_id, RunMode::MOTION_CONTROL)) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "电机 %d 设置运控模式失败", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
    // 3) 使能电机
    if (!can_driver_->enableMotor(config.motor_id)) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "电机 %d 使能失败", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 4) [关键] 发送 Kp=0 的指令，使电机进入“软”状态，不跟踪任何位置
    //    这样即使发送任意位置指令，电机也不会运动
    can_driver_->sendMotionControl(
        config.motor_id,
        config.motor_type,
        0.0,          // 位置无关紧要，因为 Kp=0
        0.0,          // velocity = 0
        0.0,          // Kp = 0（不跟踪位置！）
        4.0,          // Kd = 4.0（提高阻尼，防止抖动）
        0.0           // torque = 0
    );
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "电机 %d 已以软模式使能（Kp=0，Kd=4）", config.motor_id);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // ============ 步骤 2：开环控制 - 使用默认位置 (0)，跳过反馈等待 ============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "开环模式：所有关节使用默认位置 0.0，跳过反馈等待");
  
  std::vector<double> initial_positions(joint_configs_.size(), 0.0);
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "电机 %d 初始位置：0.0 rad（开环）", config.motor_id);
  }

  // ============ 步骤 3：发送保持指令（切换到正常控制） ============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "正在切换到位置保持模式...");
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    
    // 初始化状态变量
    hw_positions_[i] = initial_positions[i];
    hw_commands_positions_[i] = initial_positions[i];
    smoothed_positions_[i] = initial_positions[i];
    smoothed_velocities_[i] = 0.0;
    smoothed_accelerations_[i] = 0.0;
    
    // 计算电机坐标系下的位置
    double motor_pos = initial_positions[i] * config.direction + config.position_offset;
    
    // 发送“保持当前位置”指令（正常 Kp/Kd）
    can_driver_->sendMotionControl(
        config.motor_id,
        config.motor_type,
        motor_pos,        // 使用当前位置
        0.0,              // velocity = 0
        position_kp_,     // 正常 Kp
        position_kd_,     // 正常 Kd
        0.0               // torque = 0
    );
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "电机 %d 保持在 %.4f rad（Kp=%.0f，Kd=%.1f）",
                config.motor_id, initial_positions[i], position_kp_, position_kd_);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "全部 %zu 个电机初始化成功", joint_configs_.size());
  
  first_command_ = false;

  gravity_input_positions_.resize(joint_configs_.size(), 0.0);
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    gravity_input_positions_[i] = initial_positions[i];
    last_cmd_positions_[i] = initial_positions[i];
    last_hw_commands_positions_[i] = initial_positions[i];
    filtered_cmd_velocities_[i] = 0.0;
    velocity_ff_stage2_[i] = 0.0;
    velocity_settle_counter_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), 
              "硬件已激活（CSP 位置模式）");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && can_driver_) {
    // 失能所有电机
    for (const auto& config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "硬件已停用");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Stop debug spin thread before tearing down node
  if (debug_thread_running_) {
    debug_thread_running_ = false;
    if (debug_spin_thread_.joinable()) {
      debug_spin_thread_.join();
    }
  }

  on_deactivate(rclcpp_lifecycle::State());
  on_cleanup(rclcpp_lifecycle::State());
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "硬件已关闭");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && can_driver_) {
    // 急停：失能所有电机并清故障
    for (const auto& config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id, true);
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"), "硬件发生错误");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RsA3HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    // Add temperature state interface
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, "temperature", &hw_temperatures_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RsA3HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type RsA3HardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& /*stop_interfaces*/)
{
  bool wants_effort = false;
  for (const auto& iface : start_interfaces) {
    if (iface.find("effort") != std::string::npos) {
      wants_effort = true;
      break;
    }
  }
  if (wants_effort) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Mode switch prepared: effort (zero-torque)");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RsA3HardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  bool starting_effort = false;
  bool stopping_effort = false;

  for (const auto& iface : start_interfaces) {
    if (iface.find("effort") != std::string::npos) {
      starting_effort = true;
      break;
    }
  }
  for (const auto& iface : stop_interfaces) {
    if (iface.find("effort") != std::string::npos) {
      stopping_effort = true;
      break;
    }
  }

  if (starting_effort && !effort_mode_) {
    effort_mode_ = true;
    // Reset adaptive Kd to kd_max (safe, high-damping start)
    for (size_t i = 0; i < adaptive_kd_values_.size(); ++i) {
      const auto& cfg = joint_configs_[i];
      adaptive_kd_values_[i] = (cfg.zero_torque_kd_max > 0.0)
                                ? cfg.zero_torque_kd_max : zero_torque_kd_max_;
    }
    RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                "Switched to EFFORT mode (zero-torque). Kp=0, adaptive_kd=%s, fixed_Kd=%.1f",
                adaptive_kd_enabled_ ? "ON" : "OFF", zero_torque_kd_);
  }
  if (stopping_effort && effort_mode_) {
    effort_mode_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Switched to POSITION mode. Kp=%.0f, Kd=%.1f", position_kp_, position_kd_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RsA3HardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // ============ Read all states from actual motors ============
  if (!use_mock_hardware_ && can_driver_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const auto& config = joint_configs_[i];
      auto feedback = can_driver_->getMotorFeedback(config.motor_id);
      
      if (feedback.is_valid) {
        auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - feedback.last_update).count();
        if (age_ms > 50) {
          static int stale_warn = 0;
          if (stale_warn++ % 200 == 0) {
            RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
              "电机 %d 反馈数据已过期 (%ld ms)，可能通信中断",
              config.motor_id, age_ms);
          }
        }
        // Convert from motor coordinate frame to joint coordinate frame
        // motor_pos = joint_pos * direction + offset
        // joint_pos = (motor_pos - offset) / direction = (motor_pos - offset) * direction
        hw_positions_[i] = (feedback.position - config.position_offset) * config.direction;
        hw_velocities_[i] = feedback.velocity * config.direction;
        hw_efforts_[i] = feedback.torque * config.direction;
        hw_temperatures_[i] = feedback.temperature;  // 电机温度 (°C)
      }
    }
  } else {
    // Mock 模式：使用指令值作为状态
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      hw_positions_[i] = smoothed_positions_[i];
      hw_velocities_[i] = smoothed_velocities_[i];
      hw_efforts_[i] = 0.0;
      hw_temperatures_[i] = 25.0;  // Mock 温度
    }
  }
  
  // EMA filter on torque feedback (alpha=0.15 at 200Hz -> ~5Hz cutoff)
  constexpr double torque_ema_alpha = 0.15;
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    filtered_torque_feedback_[i] =
        torque_ema_alpha * hw_efforts_[i] +
        (1.0 - torque_ema_alpha) * filtered_torque_feedback_[i];
  }

  // 发布温度数据（降频：每 50 次 read 发布一次）
  static int temp_pub_counter = 0;
  if (++temp_pub_counter >= 50) {
    temp_pub_counter = 0;
    sensor_msgs::msg::JointState temp_msg;
    temp_msg.header.stamp = debug_node_->now();
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      temp_msg.name.push_back(joint_configs_[i].name);
      temp_msg.effort.push_back(hw_temperatures_[i]);  // 使用 effort 字段存储温度
    }
    temperature_pub_->publish(temp_msg);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RsA3HardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  // Thread-safety assumption: write() is called exclusively from ros2_control's
  // single-threaded update loop. The static counters in this function are safe
  // under this assumption. Convert to member variables if threading model changes.
  static int write_counter = 0;
  
  if (use_mock_hardware_) {
    return hardware_interface::return_type::OK;
  }

  if (!can_driver_ || !can_driver_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Get actual control period with tighter validation
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = control_period_;  // Use default period
  } else if (dt > control_period_ * 2.0) {
    static int period_warn = 0;
    if (period_warn++ % 200 == 0) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
        "控制周期 %.1f ms 超出 2 倍标称值 (%.1f ms)，限幅处理",
        dt * 1000.0, control_period_ * 1000.0);
    }
    dt = control_period_ * 2.0;
  }

  auto write_t_start = std::chrono::steady_clock::now();

  // [Smoothing filter] Set directly on first command, avoid smoothing from 0
  if (first_command_) {
    gravity_input_positions_.resize(joint_configs_.size(), 0.0);
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      smoothed_positions_[i] = hw_commands_positions_[i];
      smoothed_velocities_[i] = 0.0;
      smoothed_accelerations_[i] = 0.0;
      gravity_input_positions_[i] = hw_commands_positions_[i];
      
      // Initialize velocity feedforward variables, avoid jump on first computation
      last_cmd_positions_[i] = hw_commands_positions_[i];
      last_hw_commands_positions_[i] = hw_commands_positions_[i];
      filtered_cmd_velocities_[i] = 0.0;
      velocity_ff_stage2_[i] = 0.0;  // 2nd-order filter intermediate value
      velocity_settle_counter_[i] = 0;
    }
    first_command_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "收到首条指令，正在初始化位置");
  }
  
  bool global_stop = false;
  
  // ============ Pre-compute Pinocchio gravity compensation vector (if enabled) ============
  // Smooth positions for gravity computation (alpha=0.1 → ~3.4Hz cutoff at 200Hz)
  constexpr double gravity_smooth_alpha = 0.1;
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    gravity_input_positions_[i] = gravity_smooth_alpha * hw_commands_positions_[i]
                                + (1.0 - gravity_smooth_alpha) * gravity_input_positions_[i];
  }
  std::vector<double> pinocchio_gravity_torques;
  if (use_pinocchio_gravity_ && pinocchio_initialized_) {
    pinocchio_gravity_torques = computePinocchioGravity(gravity_input_positions_);
  }
  
  // Send commands using motion control mode
  struct timespec next_frame_ts;
  clock_gettime(CLOCK_MONOTONIC, &next_frame_ts);
  auto write_deadline = write_t_start + std::chrono::microseconds(4500);

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];

    if (std::chrono::steady_clock::now() > write_deadline) {
      static int deadline_warn = 0;
      if (deadline_warn++ % 200 == 0) {
        RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
          "write() 接近 deadline，跳过关节 %zu 及之后", i);
      }
      break;
    }
    
    // ============ Position command processing ============
    double new_position = hw_commands_positions_[i];
    
    // EMA position smoothing (smoothing_alpha_: 0=hold, 1=direct passthrough)
    smoothed_positions_[i] = smoothing_alpha_ * new_position
                           + (1.0 - smoothing_alpha_) * smoothed_positions_[i];

    // Derive velocity from smoothed position to stay phase-aligned with position EMA
    double smoothed_velocity = (smoothed_positions_[i] - last_cmd_positions_[i]) / dt;
    last_cmd_positions_[i] = smoothed_positions_[i];

    smoothed_accelerations_[i] = (smoothed_velocity - smoothed_velocities_[i]) / dt;
    smoothed_velocities_[i] = smoothed_velocity;
    
    if (applyJointLimitProtection(i, smoothed_positions_[i])) {
      global_stop = true;
    }

    // Convert joint coordinates to motor coordinates
    double cmd_position = smoothed_positions_[i] * config.direction + config.position_offset;
    
    // Clamp position to motor valid range
    auto params = getMotorParams(config.motor_type);
    cmd_position = std::clamp(cmd_position, params.p_min, params.p_max);
    
    // ============ Compute velocity feedforward (MA pre-filter + 2nd-order EMA) ============
    double filtered_velocity;
    
    {
      // 4-sample moving average pre-filter (null at 50Hz = teleop segment rate)
      vel_ma_buffer_[i][vel_ma_idx_[i]] = smoothed_velocity;
      vel_ma_idx_[i] = (vel_ma_idx_[i] + 1) % 4;
      double ma_velocity = 0.0;
      for (int k = 0; k < 4; ++k) ma_velocity += vel_ma_buffer_[i][k];
      ma_velocity *= 0.25;

      double cmd_velocity = std::clamp(ma_velocity, -velocity_limit_, velocity_limit_);

      // ==== 轨迹结束/取消检测（双重判据）====
      // 直接监测原始指令位置 hw_commands_positions_ 是否还在变化：
      // arm_controller 一旦进入 hold 模式（轨迹完成/被取消），它会把同一个位置
      // 反复发下来 → cmd_pos_delta 立刻为 0。这是比 cmd_velocity 更快的信号，
      // 因为不依赖位置 EMA 的衰减。检测到后**立即**把两级 EMA 状态归零，
      // 让 filtered_velocity 在 1 个周期内变 0，杜绝电机 Kd·v_ff 残余把关节
      // 继续往原方向推（轨迹结束的"沉一下"根因）。
      double cmd_pos_delta = std::abs(hw_commands_positions_[i] - last_hw_commands_positions_[i]);
      last_hw_commands_positions_[i] = hw_commands_positions_[i];

      constexpr double POS_DELTA_THRESHOLD = 1e-5;  // rad/cycle，<2e-3 rad/s 视为停
      constexpr double VEL_THRESHOLD       = 0.02;  // rad/s
      constexpr int    SETTLE_CYCLES       = 1;     // 1 周期 = 5ms 即触发

      bool stopped = (cmd_pos_delta < POS_DELTA_THRESHOLD)
                  || (std::abs(cmd_velocity) < VEL_THRESHOLD);

      if (stopped) {
        if (++velocity_settle_counter_[i] >= SETTLE_CYCLES) {
          filtered_cmd_velocities_[i] = 0.0;
          velocity_ff_stage2_[i]      = 0.0;
          cmd_velocity                = 0.0;  // 同时把滤波器输入也清零
        }
      } else {
        velocity_settle_counter_[i] = 0;
      }

      // 2nd-order EMA filter
      double alpha1 = 0.18;
      double first_stage = alpha1 * cmd_velocity + (1.0 - alpha1) * filtered_cmd_velocities_[i];
      filtered_cmd_velocities_[i] = first_stage;

      double alpha2 = 0.18;
      filtered_velocity = alpha2 * first_stage + (1.0 - alpha2) * velocity_ff_stage2_[i];

      // Acceleration limiting
      double max_velocity_change = max_acceleration_ * dt;
      double velocity_change = filtered_velocity - velocity_ff_stage2_[i];
      if (std::abs(velocity_change) > max_velocity_change) {
        filtered_velocity = velocity_ff_stage2_[i] +
                            max_velocity_change * (velocity_change > 0 ? 1.0 : -1.0);
      }

      // 兜底：检测到停止时强制 filtered_velocity 也归零，跳过加速度限幅可能造成的尾巴
      if (stopped && velocity_settle_counter_[i] >= SETTLE_CYCLES) {
        filtered_velocity = 0.0;
      }
    }
    
    // Smooth deadzone: cubic fade from 0 to 1 over [0, deadzone_width]
    // Removed deadzone to fix low-speed stick-slip jitter
    // const double dz_width = 0.01;
    // double abs_v = std::abs(filtered_velocity);
    // double fade = (abs_v < dz_width)
    //     ? abs_v * abs_v * (3.0 - 2.0 * abs_v / dz_width) / (dz_width * dz_width)
    //     : 1.0;
    // filtered_velocity *= fade;

    // Soft clamp via tanh instead of hard clamp (smooth near ±max)
    const double max_velocity_ff = 1.2;
    filtered_velocity = max_velocity_ff * std::tanh(filtered_velocity / max_velocity_ff);
    
    velocity_ff_stage2_[i] = filtered_velocity;

    // Convert to motor coordinate frame velocity
    double motor_cmd_velocity = velocity_ff_stage2_[i] * config.direction;
    
    // Debug: periodic log output
    if (write_counter % 1000 == 0 && i == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("RsA3HardwareInterface"),
                  "[速度前馈] 平滑位置差分速度=%.3f，滤波后=%.3f",
                  smoothed_velocity, filtered_velocity);
    }
    
    // ============ Compute gravity compensation torque (as feedforward by ratio) ============
    double gravity_torque = 0.0;
    if (gravity_comp_enabled_) {
      if (use_pinocchio_gravity_ && pinocchio_initialized_ && i < pinocchio_gravity_torques.size()) {
        // Use Pinocchio full dynamics computation (considering all joint cascading effects)
        gravity_torque = pinocchio_gravity_torques[i] * gravity_feedforward_ratio_;
      } else {
        // Use simplified model (per-joint independent), based on command position
        gravity_torque = computeGravityTorque(i, hw_commands_positions_[i]) * gravity_feedforward_ratio_;
      }
    }
    
    // ============ Select control parameters based on active mode ============
    double motor_kp, motor_kd, cmd_torque;
    double final_cmd_position;

    if (effort_mode_) {
      // Effort mode (zero-torque): Kp=0, Kd=damping, torque from effort command interface
      motor_kp = 0.0;
      if (adaptive_kd_enabled_) {
        adaptive_kd_values_[i] = computeAdaptiveKd(i, hw_velocities_[i], adaptive_kd_values_[i]);
        motor_kd = adaptive_kd_values_[i];
      } else {
        motor_kd = std::clamp(zero_torque_kd_, 0.0, 5.0);
      }
      cmd_torque = hw_commands_efforts_[i];
      final_cmd_position = hw_positions_[i] * config.direction + config.position_offset;
    } else {
      double joint_kp = (config.kp > 0.0) ? config.kp : position_kp_;
      double joint_kd = (config.kd > 0.0) ? config.kd : position_kd_;
      motor_kp = std::clamp(joint_kp, 0.0, 500.0);
      motor_kd = std::clamp(joint_kd, 0.0, 5.0);
      cmd_torque = gravity_torque;
      final_cmd_position = cmd_position;
    }

    double final_cmd_velocity = effort_mode_ ? 0.0 : motor_cmd_velocity;
    double motor_cmd_torque = cmd_torque * config.direction;

    if (!can_driver_->sendMotionControl(
          config.motor_id,
          config.motor_type,
          final_cmd_position,
          final_cmd_velocity,
          motor_kp,
          motor_kd,
          motor_cmd_torque
        )) {
      // Use static counter instead of THROTTLE to avoid Clock issues
      static int warn_counter = 0;
      if (warn_counter++ % 1000 == 0) {
        RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
          "向电机 %d 发送运控指令失败", config.motor_id);
      }
    }
    
    // Absolute-time frame scheduling: constant 50us spacing regardless of sendMotionControl latency
    next_frame_ts.tv_nsec += 50000;
    if (next_frame_ts.tv_nsec >= 1000000000L) {
      next_frame_ts.tv_sec++;
      next_frame_ts.tv_nsec -= 1000000000L;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_frame_ts, nullptr);
  }

  // === write() timing monitor ===
  {
    auto write_t_end = std::chrono::steady_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        write_t_end - write_t_start).count();
    write_timing_sum_us_ += elapsed_us;
    write_timing_count_++;
    if (elapsed_us > write_timing_max_us_) {
      write_timing_max_us_ = elapsed_us;
    }
    if (elapsed_us > 4000) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
        "write() 耗时 %ld us 超出预算 (4ms), max=%ld us, avg=%ld us",
        elapsed_us, write_timing_max_us_,
        write_timing_count_ > 0 ? write_timing_sum_us_ / write_timing_count_ : 0);
    }
    // Periodic timing report every 10s (~2000 cycles at 200Hz)
    if (write_counter % 2000 == 0 && write_timing_count_ > 0) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
        "[时序] write() avg=%ld us, max=%ld us, cycles=%ld, CAN retries=%lu, fails=%lu",
        write_timing_sum_us_ / write_timing_count_, write_timing_max_us_,
        write_timing_count_,
        can_driver_ ? can_driver_->getSendRetryCount() : 0UL,
        can_driver_ ? can_driver_->getSendFailCount() : 0UL);
      write_timing_max_us_ = 0;
      write_timing_sum_us_ = 0;
      write_timing_count_ = 0;
    }
  }

  // Debug publishing is handled by dedicated timers in the debug spin thread,
  // keeping this hot path clean for deterministic CAN timing.

  write_counter++;
  return hardware_interface::return_type::OK;
}

// ============ Debug publish timer callbacks (run in debug spin thread) ============

void RsA3HardwareInterface::debugPublish4Hz()
{
  if (!hw_cmd_pub_ || !smoothed_cmd_pub_) return;
  auto now = debug_node_->get_clock()->now();

  sensor_msgs::msg::JointState hw_cmd_msg;
  hw_cmd_msg.header.stamp = now;
  for (const auto& config : joint_configs_) {
    hw_cmd_msg.name.push_back(config.name);
  }
  hw_cmd_msg.position = hw_commands_positions_;
  hw_cmd_pub_->publish(hw_cmd_msg);

  sensor_msgs::msg::JointState smoothed_msg;
  smoothed_msg.header.stamp = now;
  smoothed_msg.name = hw_cmd_msg.name;
  smoothed_msg.position = smoothed_positions_;
  smoothed_msg.velocity = velocity_ff_stage2_;
  smoothed_cmd_pub_->publish(smoothed_msg);
}

void RsA3HardwareInterface::debugPublish10Hz()
{
  if (gravity_comp_enabled_ && gravity_torque_pub_) {
    auto now = debug_node_->get_clock()->now();
    sensor_msgs::msg::JointState gravity_msg;
    gravity_msg.header.stamp = now;
    for (const auto& config : joint_configs_) {
      gravity_msg.name.push_back(config.name);
    }
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      gravity_msg.effort.push_back(computeGravityTorque(i, hw_positions_[i]));
    }
    gravity_torque_pub_->publish(gravity_msg);
  }

  if (torque_feedback_pub_) {
    auto now = debug_node_->get_clock()->now();
    sensor_msgs::msg::JointState torque_msg;
    torque_msg.header.stamp = now;
    for (const auto& config : joint_configs_) {
      torque_msg.name.push_back(config.name);
    }
    torque_msg.effort = filtered_torque_feedback_;
    torque_feedback_pub_->publish(torque_msg);
  }

  if (effort_mode_ && adaptive_kd_enabled_ && adaptive_kd_pub_) {
    auto now = debug_node_->get_clock()->now();
    sensor_msgs::msg::JointState kd_msg;
    kd_msg.header.stamp = now;
    for (const auto& config : joint_configs_) {
      kd_msg.name.push_back(config.name);
    }
    kd_msg.effort = adaptive_kd_values_;
    adaptive_kd_pub_->publish(kd_msg);
  }
}

void RsA3HardwareInterface::debugPublish20Hz()
{
  if (!velocity_ff_pub_) return;
  auto now = debug_node_->get_clock()->now();
  sensor_msgs::msg::JointState velocity_ff_msg;
  velocity_ff_msg.header.stamp = now;
  for (const auto& config : joint_configs_) {
    velocity_ff_msg.name.push_back(config.name);
  }
  velocity_ff_msg.velocity = velocity_ff_stage2_;
  velocity_ff_pub_->publish(velocity_ff_msg);
}

double RsA3HardwareInterface::computeGravityTorque(size_t joint_idx, double position)
{
  if (joint_idx >= gravity_params_.size()) {
    return 0.0;
  }
  
  const auto& params = gravity_params_[joint_idx];
  return params.sin_coeff * std::sin(position)
       + params.cos_coeff * std::cos(position)
       + params.offset;
}

double RsA3HardwareInterface::computeAdaptiveKd(
    size_t joint_idx, double velocity, double prev_kd)
{
  const auto& config = joint_configs_[joint_idx];
  double kd_min = (config.zero_torque_kd_min > 0.0)
                  ? config.zero_torque_kd_min : zero_torque_kd_min_;
  double kd_max = (config.zero_torque_kd_max > 0.0)
                  ? config.zero_torque_kd_max : zero_torque_kd_max_;

  double v = std::abs(velocity);
  double ratio = v / kd_velocity_ref_;
  double kd_raw = kd_min + (kd_max - kd_min) / (1.0 + ratio * ratio);

  double kd = kd_smoothing_alpha_ * kd_raw + (1.0 - kd_smoothing_alpha_) * prev_kd;
  return std::clamp(kd, 0.0, 5.0);
}

// Zero-torque mode is now handled via controller_manager switch_controller
// (arm_controller <-> zero_torque_controller). No service needed.

double RsA3HardwareInterface::computeLimitProtectionFactor(
  size_t joint_idx, double current_pos, double target_pos)
{
  if (joint_idx >= joint_configs_.size()) {
    return 1.0;
  }
  
  const auto& config = joint_configs_[joint_idx];
  double lower = config.lower_limit;
  double upper = config.upper_limit;
  
  // Calculate distance to boundary
  double dist_to_lower = current_pos - lower;
  double dist_to_upper = upper - current_pos;
  
  // Determine motion direction
  double motion_dir = target_pos - current_pos;
  
  // Select relevant boundary distance
  double relevant_dist;
  if (motion_dir < 0) {
    // Moving toward lower limit
    relevant_dist = dist_to_lower;
  } else if (motion_dir > 0) {
    // Moving toward upper limit
    relevant_dist = dist_to_upper;
  } else {
    return 1.0;  // No motion
  }
  
  // If within limit boundary, compute deceleration factor
  if (relevant_dist < limit_margin_) {
    if (relevant_dist <= limit_stop_margin_) {
      // Hard stop zone
      return 0.0;
    }
    // Linear deceleration zone: from limit_decel_factor_ to 1.0
    double ratio = (relevant_dist - limit_stop_margin_) / (limit_margin_ - limit_stop_margin_);
    return limit_decel_factor_ + (1.0 - limit_decel_factor_) * ratio;
  }
  
  return 1.0;
}

bool RsA3HardwareInterface::applyJointLimitProtection(size_t joint_idx, double& target_pos)
{
  if (joint_idx >= joint_configs_.size()) {
    return false;
  }
  
  const auto& config = joint_configs_[joint_idx];
  double lower = config.lower_limit;
  double upper = config.upper_limit;
  
  bool hit_limit = false;
  
  // Check and clamp target position
  if (target_pos < lower + limit_stop_margin_) {
    target_pos = lower + limit_stop_margin_;
    hit_limit = true;
  } else if (target_pos > upper - limit_stop_margin_) {
    target_pos = upper - limit_stop_margin_;
    hit_limit = true;
  }
  
  // Joint limit warning (print every 100 triggers to prevent log flooding)
  if (hit_limit) {
    if (!joint_at_limit_[joint_idx]) {
      joint_at_limit_[joint_idx] = true;
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "⚠️ 关节 %s 达到限位！pos=%.3f rad（%.1f°），limits=[%.2f, %.2f]",
                  config.name.c_str(), target_pos, target_pos * 180.0 / M_PI,
                  lower, upper);
    }
    limit_warn_counter_[joint_idx]++;
    if (limit_warn_counter_[joint_idx] % 500 == 0) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "关节 %s 仍处于限位区（count=%d）",
                  config.name.c_str(), limit_warn_counter_[joint_idx]);
    }
  } else {
    if (joint_at_limit_[joint_idx]) {
      joint_at_limit_[joint_idx] = false;
      limit_warn_counter_[joint_idx] = 0;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "✓ 关节 %s 已离开限位区", config.name.c_str());
    }
  }
  
  return hit_limit;
}

// ============ Pinocchio dynamics function implementation ============

bool RsA3HardwareInterface::initPinocchioModel(const std::string& urdf_path)
{
  try {
    // 从 URDF 文件构建模型
    pinocchio::urdf::buildModel(urdf_path, pinocchio_model_);
    
    // 创建 Data 对象
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    
    pinocchio_initialized_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Pinocchio 模型初始化成功：");
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - 模型名称：%s", pinocchio_model_.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - 关节数量：%d", pinocchio_model_.njoints);
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - 自由度数量：%d", pinocchio_model_.nv);
    
    // 输出关节信息
    for (int i = 1; i < pinocchio_model_.njoints; ++i) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  - 关节 %d：%s", i, pinocchio_model_.names[i].c_str());
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "从 %s 初始化 Pinocchio 模型失败：%s",
                 urdf_path.c_str(), e.what());
    pinocchio_initialized_ = false;
    return false;
  }
}

std::vector<double> RsA3HardwareInterface::computePinocchioGravity(
  const std::vector<double>& positions)
{
  std::vector<double> gravity_torques(joint_configs_.size(), 0.0);
  
  if (!pinocchio_initialized_ || positions.size() != joint_configs_.size()) {
    return gravity_torques;
  }
  
  try {
    // Create position vector q (Pinocchio format)
    // Note: needs to be filled based on actual joint mapping
    Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
    
    // Map hardware joints to Pinocchio model joints
    // Assume same order, adjust based on actual URDF
    for (size_t i = 0; i < std::min(positions.size(), static_cast<size_t>(pinocchio_model_.nq)); ++i) {
      // Apply direction and offset (reverse conversion, because Pinocchio uses URDF coordinate frame)
      q[i] = positions[i];
    }
    
    // Create zero velocity and zero acceleration
    Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    
    // Use RNEA (Recursive Newton-Euler Algorithm) to compute inverse dynamics
    // When v=0, a=0, the result is gravity compensation torque
    Eigen::VectorXd tau = pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, v, a);
    
    // Map results back to hardware joints
    for (size_t i = 0; i < std::min(gravity_torques.size(), static_cast<size_t>(tau.size())); ++i) {
      // If using calibrated inertia parameters, skip scale factor (already applied in model)
      // Only apply scale factor when using URDF defaults
      double scale = 1.0;
      if (!use_calibrated_inertia_ && i < inertia_scale_params_.size()) {
        scale = inertia_scale_params_[i].mass_scale;
      }

      // 返回 URDF/关节坐标系下的重力力矩（不在这里乘 direction）。
      // direction 只在 write() 里把 cmd_torque 转换成电机坐标系时乘一次：
      //   motor_cmd_torque = cmd_torque * direction
      // 之前这里也乘了一次 direction，导致 direction=-1 的关节（L1/L3/L5）
      // 重力前馈方向反向 —— 表现为机械臂在重力下方向上"被前馈拽下来"的下垂。
      gravity_torques[i] = tau[i] * scale;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RsA3HardwareInterface"),
                          *debug_node_->get_clock(), 5000,
                          "Pinocchio 重力计算异常：%s", e.what());
  }
  
  return gravity_torques;
}

bool RsA3HardwareInterface::loadCalibratedInertia(const std::string& config_path)
{
  try {
    // Simple YAML parsing (no external library dependency)
    std::ifstream file(config_path);
    if (!file.is_open()) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "无法打开惯量配置文件：%s", config_path.c_str());
      return false;
    }
    
    // Initialize parameter container
    calibrated_inertia_params_.resize(6);  // L1-L6
    
    // Read defaults from Pinocchio model
    for (size_t i = 0; i < 6 && i + 1 < static_cast<size_t>(pinocchio_model_.nbodies); ++i) {
      const auto& inertia = pinocchio_model_.inertias[i + 1];  // Skip universe
      calibrated_inertia_params_[i].mass = inertia.mass();
      calibrated_inertia_params_[i].com_x = inertia.lever()[0];
      calibrated_inertia_params_[i].com_y = inertia.lever()[1];
      calibrated_inertia_params_[i].com_z = inertia.lever()[2];
    }
    
    // Parse YAML (simplified, line-by-line parsing)
    std::string line;
    std::string current_joint;
    bool in_inertia_params = false;
    bool use_calibrated = false;
    
    while (std::getline(file, line)) {
      // Skip blank lines and comments
      size_t pos = line.find_first_not_of(" \t");
      if (pos == std::string::npos || line[pos] == '#') continue;
      
      // Check use_calibrated_params
      if (line.find("use_calibrated_params:") != std::string::npos) {
        use_calibrated = (line.find("true") != std::string::npos);
        continue;
      }
      
      // Check inertia_params section
      if (line.find("inertia_params:") != std::string::npos) {
        in_inertia_params = true;
        continue;
      }
      
      if (!in_inertia_params) continue;
      
      // Check joint name (L2:, L3:, etc.)
      for (int j = 2; j <= 6; ++j) {
        std::string joint_key = "L" + std::to_string(j) + ":";
        if (line.find(joint_key) != std::string::npos && 
            line.find("mass") == std::string::npos &&
            line.find("com") == std::string::npos) {
          current_joint = "L" + std::to_string(j);
          break;
        }
      }
      
      // Parse mass
      if (!current_joint.empty() && line.find("mass:") != std::string::npos) {
        size_t colon_pos = line.find("mass:");
        std::string value_str = line.substr(colon_pos + 5);
        // Remove comments
        size_t comment_pos = value_str.find('#');
        if (comment_pos != std::string::npos) {
          value_str = value_str.substr(0, comment_pos);
        }
        // Remove whitespace
        value_str.erase(std::remove_if(value_str.begin(), value_str.end(), ::isspace), value_str.end());
        
        int joint_idx = std::stoi(current_joint.substr(1)) - 1;
        if (joint_idx >= 0 && joint_idx < 6) {
          calibrated_inertia_params_[joint_idx].mass = std::stod(value_str);
        }
      }
      
      // Parse com
      if (!current_joint.empty() && line.find("com:") != std::string::npos) {
        size_t bracket_start = line.find('[');
        size_t bracket_end = line.find(']');
        if (bracket_start != std::string::npos && bracket_end != std::string::npos) {
          std::string com_str = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
          // Parse three values
          std::vector<double> com_values;
          std::stringstream ss(com_str);
          std::string token;
          while (std::getline(ss, token, ',')) {
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            if (!token.empty()) {
              com_values.push_back(std::stod(token));
            }
          }
          
          int joint_idx = std::stoi(current_joint.substr(1)) - 1;
          if (joint_idx >= 0 && joint_idx < 6 && com_values.size() >= 3) {
            calibrated_inertia_params_[joint_idx].com_x = com_values[0];
            calibrated_inertia_params_[joint_idx].com_y = com_values[1];
            calibrated_inertia_params_[joint_idx].com_z = com_values[2];
          }
        }
      }
    }
    
    file.close();
    
    if (!use_calibrated) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "配置中未启用标定惯量：将使用 URDF 默认值");
      return false;
    }
    
    // 输出加载到的参数
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "已加载标定惯量参数：");
    for (size_t i = 1; i < 6; ++i) {  // L2-L6
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  L%zu: mass=%.4f kg, com=[%.4f, %.4f, %.4f] m",
                  i + 1,
                  calibrated_inertia_params_[i].mass,
                  calibrated_inertia_params_[i].com_x,
                  calibrated_inertia_params_[i].com_y,
                  calibrated_inertia_params_[i].com_z);
    }
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "加载惯量配置失败：%s", e.what());
    return false;
  }
}

void RsA3HardwareInterface::applyCalibratedInertiaToModel()
{
  if (!pinocchio_initialized_ || calibrated_inertia_params_.size() < 6) {
    return;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "正在将标定惯量应用到 Pinocchio 模型...");
  
  // Update inertia parameters for L2-L6 (Pinocchio model index starts from 1, skip universe)
  for (size_t i = 1; i < 6 && i + 1 < static_cast<size_t>(pinocchio_model_.nbodies); ++i) {
    const auto& params = calibrated_inertia_params_[i];
    
    // Update mass (mass() returns reference)
    pinocchio_model_.inertias[i + 1].mass() = params.mass;
    
    // Update center of mass position (lever() returns vector reference)
    pinocchio_model_.inertias[i + 1].lever()[0] = params.com_x;
    pinocchio_model_.inertias[i + 1].lever()[1] = params.com_y;
    pinocchio_model_.inertias[i + 1].lever()[2] = params.com_z;
    
    RCLCPP_DEBUG(rclcpp::get_logger("RsA3HardwareInterface"),
                 "  已更新 L%zu：mass=%.4f，com=[%.4f, %.4f, %.4f]",
                 i + 1, params.mass, params.com_x, params.com_y, params.com_z);
  }
  
  // Recreate Pinocchio Data object to apply changes
  pinocchio_data_ = pinocchio::Data(pinocchio_model_);
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Pinocchio 模型已更新（已应用标定惯量参数）");
}

rcl_interfaces::msg::SetParametersResult RsA3HardwareInterface::onParameterChange(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto& param : parameters) {
    const auto& name = param.get_name();
    
    if (name == "position_kp" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.0, 500.0);
      position_kp_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 position_kp = %.1f", val);
    }
    else if (name == "position_kd" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.0, 10.0);
      position_kd_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 position_kd = %.2f", val);
    }
    else if (name == "gravity_feedforward_ratio" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.0, 1.0);
      gravity_feedforward_ratio_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 gravity_feedforward_ratio = %.2f", val);
    }
    else if (name == "zero_torque_kd" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.0, 10.0);
      zero_torque_kd_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 zero_torque_kd = %.2f", val);
    }
    else if (name == "smoothing_alpha" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.01, 1.0);
      smoothing_alpha_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 smoothing_alpha = %.3f", val);
    }
    else if (name == "adaptive_kd_enabled" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      adaptive_kd_enabled_ = param.as_bool();
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 adaptive_kd_enabled = %s", adaptive_kd_enabled_ ? "true" : "false");
    }
    else if (name == "zero_torque_kd_min" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.0, 5.0);
      zero_torque_kd_min_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 zero_torque_kd_min = %.4f", val);
    }
    else if (name == "zero_torque_kd_max" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.0, 5.0);
      zero_torque_kd_max_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 zero_torque_kd_max = %.4f", val);
    }
    else if (name == "kd_velocity_ref" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.01, 100.0);
      kd_velocity_ref_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 kd_velocity_ref = %.2f", val);
    }
    else if (name == "kd_smoothing_alpha" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double val = std::clamp(param.as_double(), 0.01, 1.0);
      kd_smoothing_alpha_ = val;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "动态参数更新 kd_smoothing_alpha = %.3f", val);
    }
    else {
      result.successful = false;
      result.reason = "Unknown parameter: " + name;
    }
  }
  
  return result;
}

}  // namespace el_a3_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  el_a3_hardware::RsA3HardwareInterface,
  hardware_interface::SystemInterface)

