#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/receive.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"

namespace rm_serial_driver
{
class RMSerialDriver
{
 public:
  explicit RMSerialDriver(double timestamp_offset, std::string device_name, int baud_rate,
                          std::string parity);

  std::shared_ptr<rclcpp::Node> node_;

  ~RMSerialDriver();
  float pitch_trans(float originAngle);
  float pitch_re_trans(float originAngle);
  float yaw_trans(float originAngle);
  float yaw_re_trans(float originAngle);

 private:
  // 在 RMSerialDriver 类的头文件中添加成员变量
  std::ofstream csv_file_;

  void getParams();

  void receiveData();

  void sendData(const std::shared_ptr<const auto_aim_interfaces::msg::Send> msg);

  void reopenPort();

  void setParam(const rclcpp::Parameter& param);

  void resetTracker();

  // void receive_marker(const auto_aim_interfaces::msg::Receive::SharedPtr msg);

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  int baud_rate_;
  std::string parity_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Param client to set detect_colr
  using ResultFuturePtr =
      std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  auto_aim_interfaces::msg::Receive::SharedPtr receive_msg_;

  double timestamp_offset_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Receive>::SharedPtr receive_pub_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
