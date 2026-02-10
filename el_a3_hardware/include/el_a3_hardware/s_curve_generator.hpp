/**
 * @file s_curve_generator.hpp
 * @brief 标准 7 段式 S 曲线轨迹生成器（用于平滑运动控制）
 * 
 * 实现标准 7 段式 S 曲线速度剖面：
 * 1. 加加速度上升段（加速度逐渐增大）
 * 2. 匀加速段
 * 3. 加加速度下降段（加速度逐渐降到 0）
 * 4. 匀速巡航段
 * 5. 加加速度下降段（减速度逐渐增大）
 * 6. 匀减速段
 * 7. 加加速度上升段（减速度逐渐降到 0）
 * 
 * 特性：
 * - 支持摇杆等实时增量更新
 * - 支持生成完整轨迹（用于 MoveIt 路径再参数化）
 * - 支持配置 max_velocity / max_acceleration / max_jerk
 */

#ifndef EL_A3_HARDWARE__S_CURVE_GENERATOR_HPP_
#define EL_A3_HARDWARE__S_CURVE_GENERATOR_HPP_

#include <cmath>
#include <algorithm>
#include <vector>

namespace el_a3_hardware
{

/**
 * @brief 单次点到点运动的 S 曲线剖面参数
 */
struct SCurveProfile
{
  // 7 个阶段的持续时间
  double t1;  // 加加速度上升时间（加速阶段）
  double t2;  // 匀加速时间
  double t3;  // 加加速度下降时间（加速阶段结束）
  double t4;  // 匀速时间（巡航）
  double t5;  // 加加速度下降时间（减速阶段）
  double t6;  // 匀减速时间
  double t7;  // 加加速度上升时间（减速阶段结束）
  
  double total_time;
  
  // 运动约束
  double j_max;   // 最大加加速度 (rad/s³)
  double a_max;   // 最大加速度 (rad/s²)
  double v_max;   // 最大速度 (rad/s)
  
  // 运动参数
  double distance;      // 运动总距离
  double direction;     // +1 或 -1
  double v_cruise;      // 实际达到的巡航速度
  double a_limit;       // 实际达到的加速度上限
  
  // 初始条件
  double p0;  // 初始位置
  double v0;  // 初始速度
  double a0;  // 初始加速度
  
  SCurveProfile()
    : t1(0), t2(0), t3(0), t4(0), t5(0), t6(0), t7(0)
    , total_time(0)
    , j_max(50.0), a_max(10.0), v_max(3.0)
    , distance(0), direction(1.0), v_cruise(0), a_limit(0)
    , p0(0), v0(0), a0(0)
  {}
};

/**
 * @brief 实时增量 S 曲线控制的状态量
 */
struct SCurveState
{
  double position;
  double velocity;
  double acceleration;
  double jerk;
  
  // 目标跟踪
  double target_position;
  bool is_moving;
  
  SCurveState()
    : position(0), velocity(0), acceleration(0), jerk(0)
    , target_position(0), is_moving(false)
  {}
};

/**
 * @brief S 曲线轨迹生成器
 * 
 * 提供两种工作模式：
 * 1. 实时模式：根据目标变化持续更新位置
 * 2. 轨迹模式：针对给定位移生成完整轨迹
 */
class SCurveGenerator
{
public:
  /**
   * @brief 构造函数（带运动约束）
   * @param max_velocity 最大速度 (rad/s)
   * @param max_acceleration 最大加速度 (rad/s²)
   * @param max_jerk 最大加加速度 (rad/s³)
   */
  SCurveGenerator(double max_velocity = 3.0, 
                  double max_acceleration = 10.0, 
                  double max_jerk = 50.0);
  
  /**
   * @brief 设置运动约束
   */
  void setConstraints(double max_velocity, double max_acceleration, double max_jerk);
  
  /**
   * @brief 获取当前运动约束
   */
  void getConstraints(double& max_velocity, double& max_acceleration, double& max_jerk) const;
  
  /**
   * @brief 使用当前状态初始化生成器
   * @param position 当前位置
   * @param velocity 当前速度（默认 0）
   * @param acceleration 当前加速度（默认 0）
   */
  void initialize(double position, double velocity = 0.0, double acceleration = 0.0);
  
  /**
   * @brief 重置为初始状态
   */
  void reset();
  
  // ==================== 实时模式 ====================
  
  /**
   * @brief 更新目标位置（实时模式）
   * @param target_position 新目标位置
   */
  void setTarget(double target_position);
  
  /**
   * @brief 使用 S 曲线剖面计算下一步位置（实时模式）
   * @param dt 时间步长（秒）
   * @return 更新后的位置
   * 
   * 本方法通过以下步骤实现实时 S 曲线控制：
   * 1. 根据位置误差计算期望速度
   * 2. 限制加速度变化（加加速度限制）
   * 3. 限制加速度
   * 4. 限制速度
   * 5. 积分得到新的位置
   */
  double update(double dt);
  
  /**
   * @brief 获取当前状态
   */
  const SCurveState& getState() const { return state_; }
  
  /**
   * @brief 获取当前位置
   */
  double getPosition() const { return state_.position; }
  
  /**
   * @brief 获取当前速度
   */
  double getVelocity() const { return state_.velocity; }
  
  /**
   * @brief 获取当前加速度
   */
  double getAcceleration() const { return state_.acceleration; }
  
  /**
   * @brief 是否处于运动状态
   */
  bool isMoving() const { return state_.is_moving; }
  
  // ==================== 轨迹模式 ====================
  
  /**
   * @brief 计算点到点运动的完整 S 曲线剖面
   * @param start_position 起始位置
   * @param end_position 终止位置
   * @param start_velocity 起始速度（默认 0）
   * @param end_velocity 终止速度（默认 0）
   * @return 计算得到的剖面参数
   */
  SCurveProfile calculateProfile(double start_position, double end_position,
                                  double start_velocity = 0.0, double end_velocity = 0.0);
  
  /**
   * @brief 给定剖面下，获取时刻 t 的位置
   * @param profile S 曲线剖面
   * @param t 从起点开始的时间
   * @return 时刻 t 的位置
   */
  double getPositionAtTime(const SCurveProfile& profile, double t) const;
  
  /**
   * @brief 给定剖面下，获取时刻 t 的速度
   * @param profile S 曲线剖面
   * @param t 从起点开始的时间
   * @return 时刻 t 的速度
   */
  double getVelocityAtTime(const SCurveProfile& profile, double t) const;
  
  /**
   * @brief 给定剖面下，获取时刻 t 的加速度
   * @param profile S 曲线剖面
   * @param t 从起点开始的时间
   * @return 时刻 t 的加速度
   */
  double getAccelerationAtTime(const SCurveProfile& profile, double t) const;
  
  /**
   * @brief 按固定时间间隔生成轨迹点
   * @param profile S 曲线剖面
   * @param dt 轨迹点时间间隔
   * @param positions 输出：每个时间点的位置
   * @param velocities 输出：每个时间点的速度
   * @param accelerations 输出：每个时间点的加速度
   */
  void generateTrajectory(const SCurveProfile& profile, double dt,
                          std::vector<double>& positions,
                          std::vector<double>& velocities,
                          std::vector<double>& accelerations) const;

private:
  // 运动约束
  double max_velocity_;
  double max_acceleration_;
  double max_jerk_;
  
  // 实时状态
  SCurveState state_;
  
  // 实时模式的位置跟踪增益
  double position_gain_;
  
  // 小量阈值
  static constexpr double EPSILON = 1e-9;
  static constexpr double VELOCITY_THRESHOLD = 1e-6;
  static constexpr double POSITION_THRESHOLD = 1e-7;
  
  /**
   * @brief 在最大减速度约束下，计算减速到 0 的时间
   */
  double calculateStoppingTime(double velocity, double acceleration) const;
  
  /**
   * @brief 按 S 曲线减速剖面计算制动距离
   */
  double calculateStoppingDistance(double velocity, double acceleration) const;
  
  /**
   * @brief 计算带加加速度限制的加速度更新
   */
  double computeJerkLimitedAcceleration(double current_acc, double desired_acc, double dt) const;
  
  /**
   * @brief 判断所处的段并计算对应加加速度
   */
  double computeSegmentJerk(const SCurveProfile& profile, double t) const;
  
  /**
   * @brief 计算短距离剖面（可能达不到最大速度）
   */
  void calculateShortProfile(SCurveProfile& profile) const;
  
  /**
   * @brief 计算长距离剖面（可达到最大速度）
   */
  void calculateLongProfile(SCurveProfile& profile) const;
  
  /**
   * @brief 将数值夹紧到指定范围
   */
  static double clamp(double value, double min_val, double max_val)
  {
    return std::max(min_val, std::min(max_val, value));
  }
  
  /**
   * @brief 符号函数
   */
  static double sign(double value)
  {
    if (value > EPSILON) return 1.0;
    if (value < -EPSILON) return -1.0;
    return 0.0;
  }
};

}  // namespace el_a3_hardware

#endif  // EL_A3_HARDWARE__S_CURVE_GENERATOR_HPP_
