/**
 * @file robstride_can_driver.cpp
 * @brief Robstride 电机 CAN 通信驱动实现
 */

#include "el_a3_hardware/robstride_can_driver.hpp"

#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>

namespace el_a3_hardware
{

// 参数索引
constexpr uint16_t PARAM_RUN_MODE = 0x7005;
constexpr uint16_t PARAM_LOC_REF = 0x7016;
constexpr uint16_t PARAM_LIMIT_SPD = 0x7017;
constexpr uint16_t PARAM_LIMIT_CUR = 0x7018;

RobstrideCanDriver::RobstrideCanDriver(const std::string& can_interface, uint8_t host_can_id)
  : can_interface_(can_interface)
  , host_can_id_(host_can_id)
  , socket_fd_(-1)
  , receive_running_(false)
{
  // 初始化反馈缓存（电机 ID 1-6）
  motor_feedbacks_.resize(16);
  motor_types_.resize(16, MotorType::RS00);
  
  for (auto& fb : motor_feedbacks_) {
    fb.is_valid = false;
  }
}

RobstrideCanDriver::~RobstrideCanDriver()
{
  stopReceiveThread();
  close();
}

bool RobstrideCanDriver::init()
{
  // 创建 socket
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "[RobstrideCanDriver] 创建 CAN socket 失败: " << strerror(errno) << std::endl;
    return false;
  }
  
  // 获取接口索引
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "[RobstrideCanDriver] 获取接口索引失败（" 
              << can_interface_ << "）: " << strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }
  
  // 绑定 socket
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  
  if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "[RobstrideCanDriver] 绑定 CAN socket 失败: " << strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }
  
  // 设置接收超时
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;  // 100ms
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  
  std::cout << "[RobstrideCanDriver] 已在 " << can_interface_ << " 上初始化" << std::endl;
  return true;
}

void RobstrideCanDriver::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    std::cout << "[RobstrideCanDriver] 已关闭 CAN 接口" << std::endl;
  }
}

uint32_t RobstrideCanDriver::buildExtendedCanId(uint8_t comm_type, uint16_t data_area2, uint8_t target_id)
{
  // 29位ID结构:
  // Bit28~bit24: 通信类型
  // bit23~8: 数据区2（主机CAN_ID等）
  // bit7~0: 目标地址（电机CAN_ID）
  uint32_t id = 0;
  id |= (static_cast<uint32_t>(comm_type) & 0x1F) << 24;
  id |= (static_cast<uint32_t>(data_area2) & 0xFFFF) << 8;
  id |= target_id & 0xFF;
  return id | CAN_EFF_FLAG;  // 设置扩展帧标志位
}

bool RobstrideCanDriver::sendFrame(const can_frame& frame)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  
  // 添加重试机制处理缓冲区满的情况
  const int max_retries = 10;
  const int retry_delay_us = 1000;  // 1ms
  
  for (int retry = 0; retry < max_retries; ++retry) {
  ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(frame));
    if (nbytes == sizeof(frame)) {
      return true;
    }
    
    // 如果是缓冲区满，等待后重试
    if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK) {
      usleep(retry_delay_us * (retry + 1));  // 递增延迟
      continue;
    }
    
    // 其他错误直接返回失败
    std::cerr << "[RobstrideCanDriver] 发送 CAN 帧失败: " << strerror(errno) << std::endl;
    return false;
  }
  
  std::cerr << "[RobstrideCanDriver] 重试 " << max_retries << " 次后仍发送 CAN 帧失败: " << strerror(errno) << std::endl;
  return false;
}

bool RobstrideCanDriver::receiveFrame(can_frame& frame, int timeout_ms)
{
  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;
  
  int ret = poll(&pfd, 1, timeout_ms);
  if (ret <= 0) {
    return false;
  }
  
  ssize_t nbytes = ::read(socket_fd_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

uint16_t RobstrideCanDriver::floatToUint16(double x, double x_min, double x_max)
{
  if (x > x_max) x = x_max;
  if (x < x_min) x = x_min;
  
  double span = x_max - x_min;
  return static_cast<uint16_t>((x - x_min) * 65535.0 / span);
}

double RobstrideCanDriver::uint16ToFloat(uint16_t x_int, double x_min, double x_max)
{
  double span = x_max - x_min;
  return static_cast<double>(x_int) * span / 65535.0 + x_min;
}

bool RobstrideCanDriver::enableMotor(uint8_t motor_id)
{
  // 注意：不再自动设置运行模式，调用者需要先调用 setRunMode()
  // 通信类型3：电机使能运行
  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  
  frame.can_id = buildExtendedCanId(3, host_can_id_, motor_id);
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  
  if (!sendFrame(frame)) {
    return false;
  }
  
  std::cout << "[RobstrideCanDriver] 电机 " << static_cast<int>(motor_id) << " 已使能" << std::endl;
  return true;
}

bool RobstrideCanDriver::disableMotor(uint8_t motor_id, bool clear_fault)
{
  // 通信类型4：电机停止运行
  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  
  frame.can_id = buildExtendedCanId(4, host_can_id_, motor_id);
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  
  if (clear_fault) {
    frame.data[0] = 1;  // 清故障
  }
  
  if (!sendFrame(frame)) {
    return false;
  }
  
  std::cout << "[RobstrideCanDriver] 电机 " << static_cast<int>(motor_id) << " 已失能" << std::endl;
  return true;
}

bool RobstrideCanDriver::setZeroPosition(uint8_t motor_id)
{
  // 通信类型6：设置电机机械零位
  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  
  frame.can_id = buildExtendedCanId(6, host_can_id_, motor_id);
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  frame.data[0] = 1;  // Byte[0]=1
  
  if (!sendFrame(frame)) {
    return false;
  }
  
  std::cout << "[RobstrideCanDriver] 电机 " << static_cast<int>(motor_id) << " 机械零位已设置" << std::endl;
  return true;
}

bool RobstrideCanDriver::sendMotionControl(
  uint8_t motor_id,
  MotorType motor_type,
  double position,
  double velocity,
  double kp,
  double kd,
  double torque)
{
  auto params = getMotorParams(motor_type);
  
  // 通信类型1：运控模式电机控制指令
  // 根据协议文档，29位CAN ID结构：
  // Bit28~24: 通信类型 (=1)
  // Bit23~8:  前馈力矩 (16位，映射到 T_MIN~T_MAX)
  // Bit7~0:   目标电机CAN_ID
  // 数据区映射：
  // - 角度: 0~65535 对应 (-12.57~12.57) rad
  // - 角速度: 0~65535 对应 (-50~50) rad/s
  // - Kp: 0~65535 对应 (0~500)
  // - Kd: 0~65535 对应 (0~5)
  
  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  
  // 计算前馈力矩的原始值 (0 Nm 映射到 ~32768)
  uint16_t torque_raw = floatToUint16(torque, params.t_min, params.t_max);
  
  // 构建CAN ID：将 torque_raw 放入 Bit23~8 (替代原来的 host_can_id)
  frame.can_id = buildExtendedCanId(1, torque_raw, motor_id);
  frame.can_dlc = 8;
  
  // 转换数据（高字节在前，低字节在后）
  // 运控模式使用统一的映射范围
  constexpr double MOTION_V_MIN = -50.0;
  constexpr double MOTION_V_MAX = 50.0;
  
  uint16_t pos_raw = floatToUint16(position, params.p_min, params.p_max);
  uint16_t vel_raw = floatToUint16(velocity, MOTION_V_MIN, MOTION_V_MAX);
  uint16_t kp_raw = floatToUint16(kp, params.kp_min, params.kp_max);
  uint16_t kd_raw = floatToUint16(kd, params.kd_min, params.kd_max);
  
  // 数据打包（高字节在前）
  frame.data[0] = (pos_raw >> 8) & 0xFF;
  frame.data[1] = pos_raw & 0xFF;
  frame.data[2] = (vel_raw >> 8) & 0xFF;
  frame.data[3] = vel_raw & 0xFF;
  frame.data[4] = (kp_raw >> 8) & 0xFF;
  frame.data[5] = kp_raw & 0xFF;
  frame.data[6] = (kd_raw >> 8) & 0xFF;
  frame.data[7] = kd_raw & 0xFF;
  
  // 调试：检查发送的原始数据（低频率，避免阻塞执行器）
  static int debug_count = 0;
  if (debug_count++ % 60000 == 0) {
    if (motor_id == 1 || motor_id == 2) {
      std::cout << "[CAN TX] Motor " << static_cast<int>(motor_id)
                << ": pos_cmd=" << position
                << ", pos_raw=" << pos_raw
                << ", torque_raw=" << torque_raw
                << "（写入 CAN ID，0Nm≈32768）" << std::endl;
    }
  }
  
  return sendFrame(frame);
}

bool RobstrideCanDriver::writeParameter(uint8_t motor_id, uint16_t param_index, float value)
{
  // 通信类型18：单个参数写入（掉电丢失）
  can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  
  frame.can_id = buildExtendedCanId(18, host_can_id_, motor_id);
  frame.can_dlc = 8;
  
  // 数据低字节在前，高字节在后
  frame.data[0] = param_index & 0xFF;
  frame.data[1] = (param_index >> 8) & 0xFF;
  frame.data[2] = 0;
  
  // 浮点值（小端序）
  uint32_t value_raw;
  std::memcpy(&value_raw, &value, sizeof(float));
  frame.data[4] = value_raw & 0xFF;
  frame.data[5] = (value_raw >> 8) & 0xFF;
  frame.data[6] = (value_raw >> 16) & 0xFF;
  frame.data[7] = (value_raw >> 24) & 0xFF;
  
  return sendFrame(frame);
}

bool RobstrideCanDriver::setRunMode(uint8_t motor_id, RunMode mode)
{
  return writeParameter(motor_id, PARAM_RUN_MODE, static_cast<float>(static_cast<uint8_t>(mode)));
}

bool RobstrideCanDriver::setPositionCSP(uint8_t motor_id, double position)
{
  return writeParameter(motor_id, PARAM_LOC_REF, static_cast<float>(position));
}

bool RobstrideCanDriver::setVelocityLimit(uint8_t motor_id, double velocity_limit)
{
  return writeParameter(motor_id, PARAM_LIMIT_SPD, static_cast<float>(velocity_limit));
}

void RobstrideCanDriver::startReceiveThread()
{
  if (receive_running_) return;
  
  receive_running_ = true;
  receive_thread_ = std::thread(&RobstrideCanDriver::receiveThreadFunc, this);
}

void RobstrideCanDriver::stopReceiveThread()
{
  if (!receive_running_) return;
  
  receive_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

void RobstrideCanDriver::receiveThreadFunc()
{
  can_frame frame;
  static int recv_count = 0;
  static std::array<int, 7> motor_feedback_count = {0};  // 跟踪每个电机的反馈计数
  
  while (receive_running_) {
    if (receiveFrame(frame, 10)) {
      // 检查是否为扩展帧
      if (frame.can_id & CAN_EFF_FLAG) {
        // 从 29 位扩展 ID 中提取通信类型与电机 ID
        // 通信类型2反馈帧结构:
        // Bit28~24: 通信类型 (=2)
        // Bit23~22: 模式状态
        // Bit21~16: 故障信息
        // Bit15~8: 电机CAN ID
        // Bit7~0: 主机CAN ID (目标地址)
        uint32_t can_id = frame.can_id & CAN_EFF_MASK;
        uint8_t comm_type = (can_id >> 24) & 0x1F;
        uint8_t motor_id = (can_id >> 8) & 0xFF;  // 电机ID在bit8~15
        
        // 调试：周期性打印接收帧信息（低频率，避免阻塞执行器）
        if (++recv_count >= 60000) {
          recv_count = 0;
          std::cout << "[RobstrideCanDriver] 接收 CAN ID: 0x" << std::hex << can_id 
                    << "，comm_type: " << std::dec << static_cast<int>(comm_type)
                    << "，motor_id: " << static_cast<int>(motor_id) << std::endl;
          // 打印反馈统计
          std::cout << "[RobstrideCanDriver] 反馈计数 - M1:" << motor_feedback_count[1]
                    << " M2:" << motor_feedback_count[2]
                    << " M3:" << motor_feedback_count[3]
                    << " M4:" << motor_feedback_count[4]
                    << " M5:" << motor_feedback_count[5]
                    << " M6:" << motor_feedback_count[6] << std::endl;
        }
        
        if (comm_type == 2 && motor_id > 0 && motor_id < motor_types_.size()) {
          // 统计反馈
          if (motor_id <= 6) motor_feedback_count[motor_id]++;
          // 通信类型2：电机反馈数据
          parseFeedback(frame, motor_types_[motor_id]);
        }
      }
    }
  }
}

void RobstrideCanDriver::parseFeedback(const can_frame& frame, MotorType motor_type)
{
  auto params = getMotorParams(motor_type);
  
  uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  uint8_t motor_id = (can_id >> 8) & 0xFF;  // 电机ID在bit8~15
  
  if (motor_id == 0 || motor_id >= motor_feedbacks_.size()) return;
  
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  
  MotorFeedback& fb = motor_feedbacks_[motor_id];
  
  // 解析数据（高字节在前）- 通信类型2数据格式
  // Byte0~1: 当前角度 [0~65535] 对应 (P_MIN~P_MAX)
  // Byte2~3: 当前角速度 [0~65535] 对应 (V_MIN~V_MAX)
  // Byte4~5: 当前力矩 [0~65535] 对应 (T_MIN~T_MAX)
  // Byte6~7: 当前温度 Temp*10
  uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  uint16_t vel_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
  uint16_t temp_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
  
  fb.motor_id = motor_id;
  fb.position = uint16ToFloat(pos_raw, params.p_min, params.p_max);
  fb.velocity = uint16ToFloat(vel_raw, params.v_min, params.v_max);
  fb.torque = uint16ToFloat(torque_raw, params.t_min, params.t_max);
  fb.temperature = static_cast<double>(temp_raw) / 10.0;
  
  // 从 CAN ID 提取模式状态与故障信息
  // Bit22~23: 模式状态 (0=Reset, 1=Cali, 2=Motor)
  // Bit16~21: 故障信息
  fb.mode_state = (can_id >> 22) & 0x03;
  fb.fault_code = (can_id >> 16) & 0x3F;
  
  fb.is_valid = true;
}

MotorFeedback RobstrideCanDriver::getMotorFeedback(uint8_t motor_id)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  
  if (motor_id < motor_feedbacks_.size()) {
    return motor_feedbacks_[motor_id];
  }
  
  MotorFeedback fb;
  fb.is_valid = false;
  return fb;
}

void RobstrideCanDriver::setMotorType(uint8_t motor_id, MotorType type)
{
  if (motor_id < motor_types_.size()) {
    motor_types_[motor_id] = type;
    std::cout << "[RobstrideCanDriver] Motor " << static_cast<int>(motor_id) 
              << " 电机型号已设置为 " << (type == MotorType::RS00 ? "RS00" : "EL05") << std::endl;
  }
}

}  // namespace el_a3_hardware

