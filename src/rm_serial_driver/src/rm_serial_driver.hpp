#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// C++ system
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "SolveTrajectory.hpp"
#include "linux_uart.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "tracker_node.hpp"
#include "uart.hpp"

namespace rm_serial_driver
{
class RMSerialDriver
{
 public:
  explicit RMSerialDriver(double timestamp_offset, std::string device_name, int baud_rate,
                          LibXR::UART::Parity parity);

  ~RMSerialDriver();
  float pitch_trans(float originAngle);
  float pitch_re_trans(float originAngle);
  float yaw_trans(float originAngle);
  float yaw_re_trans(float originAngle);

 private:
  // 在 RMSerialDriver 类的头文件中添加成员变量
  std::ofstream csv_file_;

  void receiveData();

  void sendData(rm_auto_aim::ArmorTrackerNode::Send msg);

  void reopenPort();

  void resetTracker();

  // void receive_marker(const auto_aim_interfaces::msg::Receive::SharedPtr msg);

  // Serial port
  std::string device_name_;
  int baud_rate_;
  LibXR::UART::Parity parity_;

  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;

  struct Receive
  {
    double pitch;
    double yaw;
    double roll;
  };

  Receive receive_msg_;

  LibXR::LinuxUART* uart_;

  double timestamp_offset_ = 0;
  LibXR::Topic velocity_topic_ = LibXR::Topic("/current_velocity", sizeof(double));
  LibXR::Topic receive_topic_ = LibXR::Topic("/tracker/receive", sizeof(Receive));

  LibXR::Thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
