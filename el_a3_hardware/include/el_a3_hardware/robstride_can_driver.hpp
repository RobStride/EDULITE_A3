/**
 * @file robstride_can_driver.hpp
 * @brief Robstride 电机 CAN 通信驱动（基于 candlight/gs_usb）
 * 
 * 支持 RS00 与 EL05 系列电机（私有协议）
 */

#ifndef EL_A3_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_
#define EL_A3_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace el_a3_hardware
{

/**
 * @brief 电机型号枚举
 */
enum class MotorType
{
  RS00,  // 1-3号关节: 力矩±14Nm, 速度±33rad/s
  EL05   // 4-6号关节: 力矩±6Nm, 速度±50rad/s
};

/**
 * @brief 电机运行模式
 */
enum class RunMode : uint8_t
{
  MOTION_CONTROL = 0,  // 运控模式
  POSITION_PP = 1,     // 位置模式(PP)
  VELOCITY = 2,        // 速度模式
  CURRENT = 3,         // 电流模式
  POSITION_CSP = 5     // 位置模式(CSP)
};

/**
 * @brief 电机反馈数据
 */
struct MotorFeedback
{
  uint8_t motor_id;
  double position;      // rad
  double velocity;      // rad/s
  double torque;        // Nm
  double temperature;   // °C
  uint8_t mode_state;   // 0: Reset, 1: Cali, 2: Motor
  uint8_t fault_code;
  bool is_valid;
};

/**
 * @brief 不同电机型号的参数范围
 */
struct MotorParams
{
  double p_min;   // 位置最小值 (rad)
  double p_max;   // 位置最大值 (rad)
  double v_min;   // 速度最小值 (rad/s)
  double v_max;   // 速度最大值 (rad/s)
  double t_min;   // 力矩最小值 (Nm)
  double t_max;   // 力矩最大值 (Nm)
  double kp_min;  // Kp 最小值
  double kp_max;  // Kp 最大值
  double kd_min;  // Kd 最小值
  double kd_max;  // Kd 最大值
};

/**
 * @brief 根据电机型号获取参数范围
 */
inline MotorParams getMotorParams(MotorType type)
{
  MotorParams params;
  params.p_min = -12.57;
  params.p_max = 12.57;
  params.kp_min = 0.0;
  params.kd_min = 0.0;
  
  // 所有电机 Kp/Kd 映射范围相同: Kp 0~500, Kd 0~5
  params.kp_max = 500.0;
  params.kd_max = 5.0;
  
  switch (type)
  {
    case MotorType::RS00:
      params.v_min = -33.0;
      params.v_max = 33.0;
      params.t_min = -14.0;
      params.t_max = 14.0;
      break;
    case MotorType::EL05:
      params.v_min = -50.0;
      params.v_max = 50.0;
      params.t_min = -6.0;
      params.t_max = 6.0;
      break;
  }
  return params;
}

/**
 * @brief Robstride CAN 驱动类
 */
class RobstrideCanDriver
{
public:
  /**
   * @brief 构造 Robstride CAN 驱动
   * @param can_interface CAN 接口名（例如 "can0"）
   * @param host_can_id 主机 CAN ID（默认 0xFD）
   */
  explicit RobstrideCanDriver(const std::string& can_interface, uint8_t host_can_id = 0xFD);
  
  ~RobstrideCanDriver();
  
  /**
   * @brief 初始化 CAN 接口
   * @return 成功返回 true
   */
  bool init();
  
  /**
   * @brief 关闭 CAN 接口
   */
  void close();
  
  /**
   * @brief 检查驱动是否已连接
   */
  bool isConnected() const { return socket_fd_ >= 0; }
  
  /**
   * @brief 使能电机
   * @param motor_id 电机 CAN ID
   * @return 成功返回 true
   */
  bool enableMotor(uint8_t motor_id);
  
  /**
   * @brief 失能电机
   * @param motor_id 电机 CAN ID
   * @param clear_fault 是否清故障标志
   * @return 成功返回 true
   */
  bool disableMotor(uint8_t motor_id, bool clear_fault = false);
  
  /**
   * @brief 设置电机机械零位
   * @param motor_id 电机 CAN ID
   * @return 成功返回 true
   */
  bool setZeroPosition(uint8_t motor_id);
  
  /**
   * @brief 发送运控指令（Type 1）
   * @param motor_id 电机 CAN ID
   * @param motor_type 用于参数映射的电机型号
   * @param position 目标位置 (rad)
   * @param velocity 目标速度 (rad/s)
   * @param kp 位置增益
   * @param kd 速度增益
   * @param torque 前馈力矩 (Nm)
   * @return 成功返回 true
   */
  bool sendMotionControl(
    uint8_t motor_id,
    MotorType motor_type,
    double position,
    double velocity,
    double kp,
    double kd,
    double torque = 0.0);
  
  /**
   * @brief 写入单个参数（Type 18）
   * @param motor_id 电机 CAN ID
   * @param param_index 参数索引
   * @param value 参数值 (float)
   * @return 成功返回 true
   */
  bool writeParameter(uint8_t motor_id, uint16_t param_index, float value);
  
  /**
   * @brief 设置运行模式
   * @param motor_id 电机 CAN ID
   * @param mode 运行模式
   * @return 成功返回 true
   */
  bool setRunMode(uint8_t motor_id, RunMode mode);
  
  /**
   * @brief 设置 CSP 位置指令
   * @param motor_id 电机 CAN ID
   * @param position 目标位置 (rad)
   * @return 成功返回 true
   */
  bool setPositionCSP(uint8_t motor_id, double position);
  
  /**
   * @brief 设置 CSP 模式速度上限
   * @param motor_id 电机 CAN ID
   * @param velocity_limit 最大速度 (rad/s)
   * @return 成功返回 true
   */
  bool setVelocityLimit(uint8_t motor_id, double velocity_limit);
  
  /**
   * @brief 获取电机反馈
   * @param motor_id 电机 CAN ID
   * @return 电机反馈数据
   */
  MotorFeedback getMotorFeedback(uint8_t motor_id);
  
  /**
   * @brief 为指定电机 ID 设置电机型号
   * @param motor_id 电机 CAN ID
   * @param type 电机型号（RS00 或 EL05）
   */
  void setMotorType(uint8_t motor_id, MotorType type);
  
  /**
   * @brief 启动接收线程
   */
  void startReceiveThread();
  
  /**
   * @brief 停止接收线程
   */
  void stopReceiveThread();

private:
  /**
   * @brief 构造私有协议的扩展 CAN ID
   * @param comm_type 通信类型 (0-26)
   * @param data_area2 数据区2（主机 CAN ID 等）
   * @param target_id 目标电机 CAN ID
   * @return 29 位扩展 CAN ID
   */
  uint32_t buildExtendedCanId(uint8_t comm_type, uint16_t data_area2, uint8_t target_id);
  
  /**
   * @brief 发送 CAN 帧
   */
  bool sendFrame(const can_frame& frame);
  
  /**
   * @brief 接收 CAN 帧
   */
  bool receiveFrame(can_frame& frame, int timeout_ms = 100);
  
  /**
   * @brief 接收线程函数
   */
  void receiveThreadFunc();
  
  /**
   * @brief 从 CAN 帧解析电机反馈
   */
  void parseFeedback(const can_frame& frame, MotorType motor_type);
  
  /**
   * @brief 按协议将浮点数编码为 uint16
   */
  static uint16_t floatToUint16(double x, double x_min, double x_max);
  
  /**
   * @brief 按协议将 uint16 解码为浮点数
   */
  static double uint16ToFloat(uint16_t x_int, double x_min, double x_max);
  
  std::string can_interface_;
  uint8_t host_can_id_;
  int socket_fd_;
  
  std::mutex send_mutex_;
  std::mutex feedback_mutex_;
  
  std::atomic<bool> receive_running_;
  std::thread receive_thread_;
  
  // 电机反馈缓存（按 motor_id 索引）
  std::vector<MotorFeedback> motor_feedbacks_;
  std::vector<MotorType> motor_types_;
};

}  // namespace el_a3_hardware

#endif  // EL_A3_HARDWARE__ROBSTRIDE_CAN_DRIVER_HPP_

