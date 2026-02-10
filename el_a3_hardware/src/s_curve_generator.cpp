/**
 * @file s_curve_generator.cpp
 * @brief S 曲线轨迹生成器实现
 */

#include "el_a3_hardware/s_curve_generator.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace el_a3_hardware
{

SCurveGenerator::SCurveGenerator(double max_velocity, double max_acceleration, double max_jerk)
  : max_velocity_(max_velocity)
  , max_acceleration_(max_acceleration)
  , max_jerk_(max_jerk)
  , position_gain_(10.0)  // 位置跟踪增益
{
  reset();
}

void SCurveGenerator::setConstraints(double max_velocity, double max_acceleration, double max_jerk)
{
  max_velocity_ = std::abs(max_velocity);
  max_acceleration_ = std::abs(max_acceleration);
  max_jerk_ = std::abs(max_jerk);
}

void SCurveGenerator::getConstraints(double& max_velocity, double& max_acceleration, double& max_jerk) const
{
  max_velocity = max_velocity_;
  max_acceleration = max_acceleration_;
  max_jerk = max_jerk_;
}

void SCurveGenerator::initialize(double position, double velocity, double acceleration)
{
  state_.position = position;
  state_.velocity = velocity;
  state_.acceleration = acceleration;
  state_.jerk = 0.0;
  state_.target_position = position;
  state_.is_moving = false;
}

void SCurveGenerator::reset()
{
  state_ = SCurveState();
}

void SCurveGenerator::setTarget(double target_position)
{
  state_.target_position = target_position;
}

double SCurveGenerator::update(double dt)
{
  if (dt <= 0.0 || dt > 0.1) {
    return state_.position;
  }
  
  // 计算位置误差
  double position_error = state_.target_position - state_.position;
  
  // 判断是否已足够接近目标
  if (std::abs(position_error) < POSITION_THRESHOLD && 
      std::abs(state_.velocity) < VELOCITY_THRESHOLD) {
    state_.velocity = 0.0;
    state_.acceleration = 0.0;
    state_.jerk = 0.0;
    state_.is_moving = false;
    state_.position = state_.target_position;
    return state_.position;
  }
  
  state_.is_moving = true;
  
  // ==================== S 曲线实时算法 ====================
  // 通过加加速度限制实现实时 S 曲线控制
  
  // Step 1：根据位置误差计算期望速度
  // 使用比例控制并施加速度上限
  double desired_velocity = position_gain_ * position_error;
  desired_velocity = clamp(desired_velocity, -max_velocity_, max_velocity_);
  
  // Step 2：根据当前速度估算制动距离（考虑 S 曲线减速剖面）
  double stopping_distance = calculateStoppingDistance(state_.velocity, state_.acceleration);
  
  // Step 3：判断是否需要开始减速
  double direction = sign(position_error);
  double velocity_direction = sign(state_.velocity);
  
  // 如果正在远离目标，或距离不足以刹停，则需要减速
  bool need_decelerate = false;
  if (velocity_direction != 0 && velocity_direction != direction) {
    // 运动方向错误：必须减速
    need_decelerate = true;
  } else if (std::abs(position_error) <= std::abs(stopping_distance) * 1.1) {
    // 接近目标：提前开始减速（留 10% 裕量）
    need_decelerate = true;
  }
  
  // Step 4：计算期望加速度
  double desired_acceleration;
  if (need_decelerate) {
    // 减速：加速度方向应与速度相反
    if (std::abs(state_.velocity) < VELOCITY_THRESHOLD) {
      desired_acceleration = 0.0;
    } else {
      desired_acceleration = -sign(state_.velocity) * max_acceleration_;
    }
  } else {
    // 朝目标方向加速
    double velocity_error = desired_velocity - state_.velocity;
    desired_acceleration = clamp(velocity_error / dt, -max_acceleration_, max_acceleration_);
  }
  
  // Step 5：施加加加速度限制（S 曲线平滑性的关键）
  double new_acceleration = computeJerkLimitedAcceleration(
    state_.acceleration, desired_acceleration, dt);
  
  // Step 6：更新加加速度
  state_.jerk = (new_acceleration - state_.acceleration) / dt;
  
  // Step 7：更新加速度
  state_.acceleration = new_acceleration;
  
  // Step 8：在加速度限制下更新速度
  double new_velocity = state_.velocity + state_.acceleration * dt;
  new_velocity = clamp(new_velocity, -max_velocity_, max_velocity_);
  state_.velocity = new_velocity;
  
  // Step 9：更新位置
  // 使用梯形积分提高精度
  double avg_velocity = (state_.velocity + new_velocity) * 0.5;
  state_.position += avg_velocity * dt;
  
  // Step 10：非常接近目标且速度很小则直接夹紧到目标
  if (std::abs(state_.target_position - state_.position) < POSITION_THRESHOLD * 10 &&
      std::abs(state_.velocity) < VELOCITY_THRESHOLD * 10) {
    state_.position = state_.target_position;
  }
  
  return state_.position;
}

double SCurveGenerator::calculateStoppingTime(double velocity, double acceleration) const
{
  // 将加速度减到 0 所需时间
  double t_jerk = std::abs(acceleration) / max_jerk_;
  
  // 加加速度阶段的速度变化量
  double v_change_jerk = 0.5 * std::abs(acceleration) * t_jerk;
  
  // 加加速度阶段结束后的剩余速度
  double v_remaining = std::abs(velocity) - v_change_jerk;
  
  if (v_remaining <= 0) {
    // 仅靠加加速度阶段即可停止
    return std::sqrt(2.0 * std::abs(velocity) / max_jerk_);
  }
  
  // 需要进入匀减速阶段
  // 匀减速阶段持续时间
  double t_const = v_remaining / max_acceleration_;
  
  // 最后一个加加速度阶段：将加速度带回到 0
  double t_jerk_final = max_acceleration_ / max_jerk_;
  
  return t_jerk + t_const + t_jerk_final;
}

double SCurveGenerator::calculateStoppingDistance(double velocity, double acceleration) const
{
  double v = std::abs(velocity);
  double a = std::abs(acceleration);
  
  if (v < VELOCITY_THRESHOLD) {
    return 0.0;
  }
  
  // 简化版 S 曲线制动距离估算
  // 完整推导需要考虑 7 个阶段；实时控制中使用近似即可获得平滑减速效果
  
  // 将加速度减到 0 的时间（加加速度阶段 1）
  double t1 = a / max_jerk_;
  double d1 = v * t1 + 0.5 * a * t1 * t1 - (1.0/6.0) * max_jerk_ * t1 * t1 * t1;
  double v1 = v + a * t1 - 0.5 * max_jerk_ * t1 * t1;
  
  if (v1 <= 0) {
    // 在第一段加加速度阶段内停止
    return std::abs(d1);
  }
  
  // 匀减速阶段
  double t2 = (v1 - max_acceleration_ * max_acceleration_ / (2.0 * max_jerk_)) / max_acceleration_;
  if (t2 < 0) t2 = 0;
  double d2 = v1 * t2 - 0.5 * max_acceleration_ * t2 * t2;
  double v2 = v1 - max_acceleration_ * t2;
  
  // 最后一个加加速度阶段
  double t3 = max_acceleration_ / max_jerk_;
  double d3 = v2 * t3 - 0.5 * max_acceleration_ * t3 * t3 + (1.0/6.0) * max_jerk_ * t3 * t3 * t3;
  
  double total_distance = d1 + d2 + d3;
  
  // 增加安全裕量
  return std::abs(total_distance) * 1.2;
}

double SCurveGenerator::computeJerkLimitedAcceleration(double current_acc, double desired_acc, double dt) const
{
  double acc_change = desired_acc - current_acc;
  double max_acc_change = max_jerk_ * dt;
  
  // 按加加速度上限夹紧加速度变化量
  acc_change = clamp(acc_change, -max_acc_change, max_acc_change);
  
  double new_acc = current_acc + acc_change;
  
  // 同时夹紧加速度本身
  new_acc = clamp(new_acc, -max_acceleration_, max_acceleration_);
  
  return new_acc;
}

// ==================== 轨迹模式实现 ====================

SCurveProfile SCurveGenerator::calculateProfile(double start_position, double end_position,
                                                 double start_velocity, double end_velocity [[maybe_unused]])
{
  SCurveProfile profile;
  profile.j_max = max_jerk_;
  profile.a_max = max_acceleration_;
  profile.v_max = max_velocity_;
  profile.p0 = start_position;
  profile.v0 = start_velocity;
  profile.a0 = 0.0;  // Assume starting from zero acceleration
  
  double displacement = end_position - start_position;
  profile.distance = std::abs(displacement);
  profile.direction = (displacement >= 0) ? 1.0 : -1.0;
  
  if (profile.distance < EPSILON) {
    // 无需运动
    profile.total_time = 0.0;
    return profile;
  }
  
  // 达到最大加速度所需时间（加加速度阶段）
  double t_j = profile.a_max / profile.j_max;
  
  // 单个加加速度阶段获得的速度增量
  double v_j = 0.5 * profile.j_max * t_j * t_j;
  
  // 加速阶段（含两个加加速度段）的位移
  // 假设能够达到最大加速度
  double d_acc = v_j * t_j + profile.a_max * t_j * t_j + v_j;
  
  // 判断是否能达到最大速度
  double v_acc = 2 * v_j;  // Velocity gained during full acceleration phase
  
  if (v_acc >= profile.v_max) {
    // 达不到最大加速度：三角形剖面
    calculateShortProfile(profile);
  } else {
    // 判断是否能在半程之前达到最大速度
    double d_to_vmax = d_acc + (profile.v_max - v_acc) * (profile.v_max - v_acc) / (2.0 * profile.a_max);
    
    if (2.0 * d_to_vmax > profile.distance) {
      // 达不到最大速度：梯形加速度剖面
      calculateShortProfile(profile);
    } else {
      // 可达到最大速度：完整 S 曲线
      calculateLongProfile(profile);
    }
  }
  
  return profile;
}

void SCurveGenerator::calculateShortProfile(SCurveProfile& profile) const
{
  // 短距离情况下，可能达不到最大速度，甚至达不到最大加速度
  // 使用对称剖面：t1=t3=t5=t7，t2=t6，t4=0
  
  double j = profile.j_max;
  double a = profile.a_max;
  // double v = profile.v_max;  // 短剖面不会达到最大速度，因此不使用
  double d = profile.distance;
  
  // 达到最大加速度所需时间
  double t_j = a / j;
  
  // 判断是否能达到最大加速度
  // 纯加加速度剖面（t1=t3，无匀加速）的位移
  double d_jerk_only = j * t_j * t_j * t_j / 3.0;
  
  if (d < 2.0 * d_jerk_only) {
    // 极短距离：纯加加速度剖面
    double t1 = std::cbrt(d * 1.5 / j);
    profile.t1 = t1;
    profile.t2 = 0.0;
    profile.t3 = t1;
    profile.t4 = 0.0;
    profile.t5 = t1;
    profile.t6 = 0.0;
    profile.t7 = t1;
    profile.v_cruise = j * t1 * t1;  // 峰值速度
    profile.a_limit = j * t1;        // 峰值加速度
  } else {
    // 需要匀加速阶段
    // 在位移约束下求解 t1 与 t2
    // d = 2 * (v_j * t_j + 0.5 * a * t_j^2 + a * t_a * t_j + 0.5 * a * t_a^2)
    
    profile.t1 = t_j;
    profile.t3 = t_j;
    profile.t5 = t_j;
    profile.t7 = t_j;
    
    // t1 结束时速度
    double v1 = 0.5 * j * t_j * t_j;
    
    // 4 个加加速度段覆盖的位移
    double d_jerk = 4.0 * (v1 * t_j / 2.0 + j * t_j * t_j * t_j / 6.0);
    
    // 匀加速/匀减速阶段的剩余位移
    double d_const = d - d_jerk;
    
    // 求解 t2（匀加速时间）
    // 使用一元二次方程
    double A = a;
    double B = 2.0 * v1 + a * t_j;
    double C = -d_const / 2.0;
    
    double discriminant = B * B - 4.0 * A * C;
    if (discriminant < 0) discriminant = 0;
    
    double t2 = (-B + std::sqrt(discriminant)) / (2.0 * A);
    if (t2 < 0) t2 = 0;
    
    profile.t2 = t2;
    profile.t6 = t2;
    profile.t4 = 0.0;  // No cruise phase
    
    profile.v_cruise = v1 + a * (t_j + t2);
    profile.a_limit = a;
  }
  
  profile.total_time = profile.t1 + profile.t2 + profile.t3 + profile.t4 +
                       profile.t5 + profile.t6 + profile.t7;
}

void SCurveGenerator::calculateLongProfile(SCurveProfile& profile) const
{
  // 含巡航段的完整 7 段式 S 曲线
  double j = profile.j_max;
  double a = profile.a_max;
  double v = profile.v_max;
  double d = profile.distance;
  
  // 加加速度段持续时间
  double t_j = a / j;
  
  // 加速阶段加加速度段获得的速度增量
  double v_j = 0.5 * j * t_j * t_j;
  
  // 匀加速达到最大速度所需时间
  double t_a = (v - 2.0 * v_j) / a;
  if (t_a < 0) t_a = 0;
  
  profile.t1 = t_j;
  profile.t2 = t_a;
  profile.t3 = t_j;
  profile.t5 = t_j;
  profile.t6 = t_a;
  profile.t7 = t_j;
  
  // 加速阶段（t1+t2+t3）覆盖的位移
  double d_acc = v_j * t_j + 0.5 * a * t_j * t_j +  // t1
                 (v_j + 0.5 * a * t_j) * t_a + 0.5 * a * t_a * t_a +  // t2
                 (v_j + a * t_j + a * t_a) * t_j + 0.5 * a * t_j * t_j - j * t_j * t_j * t_j / 6.0;  // t3
  
  // 减速阶段对称
  double d_dec = d_acc;
  
  // 巡航段位移
  double d_cruise = d - d_acc - d_dec;
  if (d_cruise < 0) d_cruise = 0;
  
  // 巡航段时间
  profile.t4 = d_cruise / v;
  
  profile.v_cruise = v;
  profile.a_limit = a;
  
  profile.total_time = profile.t1 + profile.t2 + profile.t3 + profile.t4 +
                       profile.t5 + profile.t6 + profile.t7;
}

double SCurveGenerator::computeSegmentJerk(const SCurveProfile& profile, double t) const
{
  double j = profile.j_max * profile.direction;
  
  // 判断所处阶段
  double t_end1 = profile.t1;
  double t_end2 = t_end1 + profile.t2;
  double t_end3 = t_end2 + profile.t3;
  double t_end4 = t_end3 + profile.t4;
  double t_end5 = t_end4 + profile.t5;
  double t_end6 = t_end5 + profile.t6;
  // double t_end7 = t_end6 + profile.t7;  // = total_time
  
  if (t < t_end1) {
    return j;  // 段 1：正加加速度（加速）
  } else if (t < t_end2) {
    return 0.0;  // 段 2：加加速度为 0（匀加速）
  } else if (t < t_end3) {
    return -j;  // 段 3：负加加速度（减小加速度）
  } else if (t < t_end4) {
    return 0.0;  // 段 4：加加速度为 0（巡航）
  } else if (t < t_end5) {
    return -j;  // 段 5：负加加速度（开始减速）
  } else if (t < t_end6) {
    return 0.0;  // 段 6：加加速度为 0（匀减速）
  } else {
    return j;  // 段 7：正加加速度（减速结束）
  }
}

double SCurveGenerator::getPositionAtTime(const SCurveProfile& profile, double t) const
{
  if (t <= 0) return profile.p0;
  if (t >= profile.total_time) return profile.p0 + profile.distance * profile.direction;
  
  double j = profile.j_max * profile.direction;
  double p = profile.p0;
  double v = profile.v0;
  double a = profile.a0;
  
  // 时间边界
  double t_end1 = profile.t1;
  double t_end2 = t_end1 + profile.t2;
  double t_end3 = t_end2 + profile.t3;
  double t_end4 = t_end3 + profile.t4;
  double t_end5 = t_end4 + profile.t5;
  double t_end6 = t_end5 + profile.t6;
  
  // 依次处理各阶段
  auto processSegment = [&](double dt, double jerk) {
    p += v * dt + 0.5 * a * dt * dt + (1.0/6.0) * jerk * dt * dt * dt;
    v += a * dt + 0.5 * jerk * dt * dt;
    a += jerk * dt;
  };
  
  // 段 1
  if (t <= t_end1) {
    processSegment(t, j);
    return p;
  }
  processSegment(profile.t1, j);
  
  // 段 2
  if (t <= t_end2) {
    processSegment(t - t_end1, 0.0);
    return p;
  }
  processSegment(profile.t2, 0.0);
  
  // 段 3
  if (t <= t_end3) {
    processSegment(t - t_end2, -j);
    return p;
  }
  processSegment(profile.t3, -j);
  
  // 段 4（巡航）
  if (t <= t_end4) {
    processSegment(t - t_end3, 0.0);
    return p;
  }
  processSegment(profile.t4, 0.0);
  
  // 段 5
  if (t <= t_end5) {
    processSegment(t - t_end4, -j);
    return p;
  }
  processSegment(profile.t5, -j);
  
  // 段 6
  if (t <= t_end6) {
    processSegment(t - t_end5, 0.0);
    return p;
  }
  processSegment(profile.t6, 0.0);
  
  // 段 7
  processSegment(t - t_end6, j);
  return p;
}

double SCurveGenerator::getVelocityAtTime(const SCurveProfile& profile, double t) const
{
  if (t <= 0) return profile.v0;
  if (t >= profile.total_time) return 0.0;  // 假设末端静止
  
  double j = profile.j_max * profile.direction;
  double v = profile.v0;
  double a = profile.a0;
  
  // 时间边界
  double t_end1 = profile.t1;
  double t_end2 = t_end1 + profile.t2;
  double t_end3 = t_end2 + profile.t3;
  double t_end4 = t_end3 + profile.t4;
  double t_end5 = t_end4 + profile.t5;
  double t_end6 = t_end5 + profile.t6;
  
  auto processSegment = [&](double dt, double jerk) {
    v += a * dt + 0.5 * jerk * dt * dt;
    a += jerk * dt;
  };
  
  if (t <= t_end1) {
    v += a * t + 0.5 * j * t * t;
    return v;
  }
  processSegment(profile.t1, j);
  
  if (t <= t_end2) {
    v += a * (t - t_end1);
    return v;
  }
  processSegment(profile.t2, 0.0);
  
  if (t <= t_end3) {
    double dt = t - t_end2;
    v += a * dt + 0.5 * (-j) * dt * dt;
    return v;
  }
  processSegment(profile.t3, -j);
  
  if (t <= t_end4) {
    v += a * (t - t_end3);
    return v;
  }
  processSegment(profile.t4, 0.0);
  
  if (t <= t_end5) {
    double dt = t - t_end4;
    v += a * dt + 0.5 * (-j) * dt * dt;
    return v;
  }
  processSegment(profile.t5, -j);
  
  if (t <= t_end6) {
    v += a * (t - t_end5);
    return v;
  }
  processSegment(profile.t6, 0.0);
  
  double dt = t - t_end6;
  v += a * dt + 0.5 * j * dt * dt;
  return v;
}

double SCurveGenerator::getAccelerationAtTime(const SCurveProfile& profile, double t) const
{
  if (t <= 0 || t >= profile.total_time) return 0.0;
  
  double j = profile.j_max * profile.direction;
  double a = profile.a0;
  
  // 时间边界
  double t_end1 = profile.t1;
  double t_end2 = t_end1 + profile.t2;
  double t_end3 = t_end2 + profile.t3;
  double t_end4 = t_end3 + profile.t4;
  double t_end5 = t_end4 + profile.t5;
  double t_end6 = t_end5 + profile.t6;
  
  if (t <= t_end1) {
    return a + j * t;
  }
  a += j * profile.t1;
  
  if (t <= t_end2) {
    return a;
  }
  
  if (t <= t_end3) {
    return a + (-j) * (t - t_end2);
  }
  a += (-j) * profile.t3;
  
  if (t <= t_end4) {
    return a;  // Should be ~0
  }
  
  if (t <= t_end5) {
    return a + (-j) * (t - t_end4);
  }
  a += (-j) * profile.t5;
  
  if (t <= t_end6) {
    return a;
  }
  
  return a + j * (t - t_end6);
}

void SCurveGenerator::generateTrajectory(const SCurveProfile& profile, double dt,
                                          std::vector<double>& positions,
                                          std::vector<double>& velocities,
                                          std::vector<double>& accelerations) const
{
  positions.clear();
  velocities.clear();
  accelerations.clear();
  
  if (profile.total_time <= 0 || dt <= 0) {
    positions.push_back(profile.p0);
    velocities.push_back(profile.v0);
    accelerations.push_back(profile.a0);
    return;
  }
  
  int num_points = static_cast<int>(std::ceil(profile.total_time / dt)) + 1;
  positions.reserve(num_points);
  velocities.reserve(num_points);
  accelerations.reserve(num_points);
  
  for (double t = 0; t <= profile.total_time; t += dt) {
    positions.push_back(getPositionAtTime(profile, t));
    velocities.push_back(getVelocityAtTime(profile, t));
    accelerations.push_back(getAccelerationAtTime(profile, t));
  }
  
  // 确保包含最终点
  if (positions.empty() || 
      std::abs(positions.back() - (profile.p0 + profile.distance * profile.direction)) > EPSILON) {
    positions.push_back(profile.p0 + profile.distance * profile.direction);
    velocities.push_back(0.0);
    accelerations.push_back(0.0);
  }
}

}  // namespace el_a3_hardware
