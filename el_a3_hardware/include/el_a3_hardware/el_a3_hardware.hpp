/**
 * @file el_a3_hardware.hpp
 * @brief EL-A3 机械臂的 ROS2 Control 硬件接口
 */

#ifndef EL_A3_HARDWARE__EL_A3_HARDWARE_HPP_
#define EL_A3_HARDWARE__EL_A3_HARDWARE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "el_a3_hardware/robstride_can_driver.hpp"
#include "el_a3_hardware/s_curve_generator.hpp"

// Pinocchio：用于动力学计算
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

namespace el_a3_hardware
{

/**
 * @brief 关节配置
 */
struct JointConfig
{
  std::string name;
  uint8_t motor_id;
  MotorType motor_type;
  double position_offset;
  double direction;  // 1.0 or -1.0
  double lower_limit;  // Joint lower limit (rad)
  double upper_limit;  // Joint upper limit (rad)
  double kp;           // 关节独立 Kp（0 表示使用全局默认值）
  double kd;           // 关节独立 Kd（0 表示使用全局默认值）
};

/**
 * @brief EL-A3 的 ROS2 Control 硬件接口
 */
class RsA3HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RsA3HardwareInterface)

  RsA3HardwareInterface();
  ~RsA3HardwareInterface() override;

  // SystemInterface 接口
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;
  
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /**
   * @brief 从硬件信息中解析关节配置
   */
  bool parseJointConfig(const hardware_interface::HardwareInfo& info);
  
  // CAN 驱动
  std::unique_ptr<RobstrideCanDriver> can_driver_;
  std::string can_interface_;
  uint8_t host_can_id_;
  
  // 关节配置
  std::vector<JointConfig> joint_configs_;
  
  // 状态接口数据
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_temperatures_;  // 电机温度 (°C)
  
  // 指令接口数据
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  
  // 控制参数
  double position_kp_;
  double position_kd_;
  double velocity_limit_;
  
  // 位置指令平滑滤波（含速度/加速度/加加速度限制 - S 曲线规划）
  std::vector<double> smoothed_positions_;     // 平滑后位置指令
  std::vector<double> smoothed_velocities_;    // 平滑后速度（用于加速度限制）
  std::vector<double> smoothed_accelerations_; // 平滑后加速度（用于 S 曲线规划）
  
  // 速度前馈计算
  std::vector<double> last_cmd_positions_;          // 上一周期指令位置（用于计算速度前馈）
  std::vector<double> last_hw_commands_positions_;  // 上一帧 hw_commands（用于判断指令是否更新）
  std::vector<double> cmd_velocities_;              // 计算得到的指令速度
  std::vector<double> filtered_cmd_velocities_;     // 一阶滤波后的指令速度
  std::vector<double> velocity_ff_stage2_;          // 二阶滤波中间量（最终发送的速度前馈）
  double velocity_filter_alpha_;                    // 速度滤波系数 (0-1)
  double smoothing_alpha_;                          // 平滑系数 (0-1，越小越平滑)
  double max_velocity_;                             // 最大速度限制 (rad/s)
  double max_acceleration_;                         // 最大加速度限制 (rad/s²)
  bool first_command_;                              // 首条指令标志
  double control_period_;                           // 控制周期 (s)
  
  // Control mode
  enum class ControlMode
  {
    POSITION,    // CSP mode
    VELOCITY,
    EFFORT
  };
  ControlMode control_mode_;
  
  bool use_mock_hardware_;  // 是否使用仿真/假硬件
  
  // S 曲线轨迹生成器（每个关节一个）
  std::vector<std::unique_ptr<SCurveGenerator>> s_curve_generators_;
  bool s_curve_enabled_;                      // 是否启用 S 曲线规划
  
  // ============ 零力矩模式与重力补偿 ============
  // 零力矩模式
  bool zero_torque_mode_;           // 是否启用零力矩模式
  double zero_torque_kd_;           // 零力矩模式阻尼系数
  
  // 重力补偿参数（按关节：τ = sin_coeff * sin(θ) + cos_coeff * cos(θ) + offset）
  struct GravityCompParams {
    double sin_coeff;
    double cos_coeff;
    double offset;
  };
  std::vector<GravityCompParams> gravity_params_;
  bool gravity_comp_enabled_;       // 是否启用重力补偿
  double gravity_feedforward_ratio_;          // 重力补偿前馈比例 (0-1，默认 0.5=50%)
  
  // ============ Pinocchio 动力学模型 ============
  bool use_pinocchio_gravity_;      // 是否使用 Pinocchio 进行重力补偿
  std::string urdf_path_;           // URDF 文件路径
  pinocchio::Model pinocchio_model_;     // Pinocchio 模型
  pinocchio::Data pinocchio_data_;       // Pinocchio 数据
  bool pinocchio_initialized_;           // Pinocchio 是否初始化成功
  
  // 惯量参数（用于标定，完全替换 URDF 默认值）
  struct CalibratedInertiaParams {
    double mass;            // 质量 (kg)
    double com_x;           // 质心 x 坐标 (m)
    double com_y;           // 质心 y 坐标 (m)
    double com_z;           // 质心 z 坐标 (m)
  };
  std::vector<CalibratedInertiaParams> calibrated_inertia_params_;
  std::string inertia_config_path_;   // 惯量配置文件路径
  bool use_calibrated_inertia_;       // 是否使用标定后的惯量参数
  
  // 旧版缩放因子结构体（保留用于向后兼容）
  struct InertiaScaleParams {
    double mass_scale;      // 质量缩放系数
    double com_x_offset;    // 质心 x 偏移
    double com_y_offset;    // 质心 y 偏移
    double com_z_offset;    // 质心 z 偏移
  };
  std::vector<InertiaScaleParams> inertia_scale_params_;
  
  /**
   * @brief 初始化 Pinocchio 模型
   * @param urdf_path URDF 文件路径
   * @return 是否成功
   */
  bool initPinocchioModel(const std::string& urdf_path);
  
  /**
   * @brief 从 YAML 配置文件加载标定后的惯量参数
   * @param config_path 配置文件路径
   * @return 是否成功
   */
  bool loadCalibratedInertia(const std::string& config_path);
  
  /**
   * @brief 将标定惯量参数应用到 Pinocchio 模型
   */
  void applyCalibratedInertiaToModel();
  
  /**
   * @brief 使用 Pinocchio 计算完整的重力补偿向量
   * @param positions 当前关节位置
   * @return 每个关节的重力补偿力矩
   */
  std::vector<double> computePinocchioGravity(const std::vector<double>& positions);
  
  // 计算关节重力补偿力矩（简化模型：各关节相互独立）
  double computeGravityTorque(size_t joint_idx, double position);
  
  // 零力矩模式服务回调
  void zeroTorqueModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  
  // 零力矩模式服务
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr zero_torque_srv_;
  
  // 调试发布器
  rclcpp::Node::SharedPtr debug_node_;
  std::thread spin_thread_;                 // 处理服务回调的线程
  std::atomic<bool> spin_thread_running_;   // 线程运行标志
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hw_cmd_pub_;        // 控制器输出的指令
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr smoothed_cmd_pub_;  // 实际发送给电机的平滑指令
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_torque_pub_; // 重力补偿力矩
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr velocity_ff_pub_;   // 速度前馈（发送给电机）
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr temperature_pub_;   // 电机温度
  
  // ============ 关节限位保护 ============
  double limit_margin_;              // 开始减速的余量 (rad)
  double limit_stop_margin_;         // 硬停止余量 (rad)
  double limit_decel_factor_;        // 接近限位时的减速系数 (0-1)
  double max_jerk_;                  // 最大加加速度限制 (rad/s³) - S 曲线规划
  std::vector<bool> joint_at_limit_; // 每个关节是否处于限位区
  std::vector<int> limit_warn_counter_;  // 限位告警计数（避免刷屏）
  
  /**
   * @brief 计算限位保护系数
   * @param joint_idx 关节索引
   * @param current_pos 当前角度
   * @param target_pos 目标角度
   * @return 速度缩放系数 (0.0-1.0)
   */
  double computeLimitProtectionFactor(size_t joint_idx, double current_pos, double target_pos);
  
  /**
   * @brief 应用限位保护并夹紧目标角度
   * @param joint_idx 关节索引
   * @param target_pos 目标角度（会被修改）
   * @return 是否触发限位保护
   */
  bool applyJointLimitProtection(size_t joint_idx, double& target_pos);
};

}  // namespace el_a3_hardware

#endif  // EL_A3_HARDWARE__EL_A3_HARDWARE_HPP_

