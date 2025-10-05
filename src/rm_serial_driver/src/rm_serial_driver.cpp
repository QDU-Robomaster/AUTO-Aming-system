#include "rm_serial_driver.hpp"

// ROS
#include <ostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <math.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "crc.hpp"
#include "logger.hpp"
#include "packet.hpp"

// 串口驱动
namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(double timestamp_offset, std::string device_name,
                               int baud_rate, std::string parity)
    : owned_ctx_{new IoContext(2)},
      device_name_{device_name},
      baud_rate_{baud_rate},
      parity_{parity},
      serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)},
      timestamp_offset_(timestamp_offset)
{
  //! serial_driver  第一部分 参数设置
  // INFO打印
  XR_LOG_INFO("Start RMSerialDriver!");

  // node_ = new rclcpp::Node("rm_serial_driver");
  node_ = rclcpp::Node::make_shared("rm_serial_driver");

  getParams();  // 传参

  //* 创建发布者
  // 时间偏移量
  // /joint_states 发布端,用来发布云台
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  // 发布当前弹速
  velocity_pub_ = node_->create_publisher<auto_aim_interfaces::msg::Velocity>(
      "/current_velocity", 10);

  // // 检查参数客户端
  detector_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(node_, "armor_detector");

  // Tracker重置服务客户端
  reset_tracker_client_ = node_->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  //! serial_driver 第二部分 串口初始化以及收发
  try
  {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open())
    {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  }
  catch (const std::exception& ex)
  {
    XR_LOG_ERROR("Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "gimbal_odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // // 创造接受节点
  // receive_pub_ = this->create_publisher<auto_aim_interfaces::msg::Receive>(
  // "/tracker/receive",  rclcpp::SensorDataQoS());

  // Create Subscription
  send_sub_ = node_->create_subscription<auto_aim_interfaces::msg::Send>(
      "/tracker/send", rclcpp::SensorDataQoS(),
      std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  // target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "/tracker/target", rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

//* 析构
RMSerialDriver::~RMSerialDriver() {}

//! 接受数据 电控 -> 视觉
void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok())
  {
    try
    {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A)
      {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        // CRC校验
        bool crc_ok = LibXR::CRC16::Verify(reinterpret_cast<const uint8_t*>(&packet),
                                           sizeof(packet));

        if (crc_ok)
        {
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_)
          {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker)
          {
            resetTracker();
          }

          // 将 电控来的 [0~2PI] -> [-PI ~ PI]
          packet.pitch = RMSerialDriver::pitch_re_trans(packet.pitch);
          packet.yaw = RMSerialDriver::yaw_re_trans(packet.yaw);
          packet.roll = RMSerialDriver::pitch_trans(packet.roll);

          // auto_aim_interfaces::msg::Receive receive_msg;
          // receive_msg.pitch = packet.pitch;
          // receive_msg.yaw = packet.yaw;
          // receive_msg.roll = packet.roll;
          // receive_pub_->publish(receive_msg);

          // 打印 data 结构体中的 xyz 和 yaw 值
          // std::cout << "xyz: (" << packet.aim_x << ", " << packet.aim_y << ",
          // " << packet.aim_z << ")" << std::endl; std::cout << "pitch: " <<
          // packet.pitch << "yaw: " << packet.yaw << std::endl;
          // XR_LOG_INFO("CRC OK!");

          // //LOG [Receive] aim_xyz

          // XR_LOG_INFO("[Receive] aim_x %f!", packet.aim_x);
          // XR_LOG_INFO("[Receive] aim_y %f!", packet.aim_y);
          // XR_LOG_INFO("[Receive] aim_z %f!", packet.aim_z);

          // // //LOG [Receive] [Receive] rp
          // XR_LOG_INFO("[Receive] roll %f!", packet.roll);
          // XR_LOG_INFO("[Receive] pitch %f!", packet.pitch);
          // XR_LOG_INFO("[Receive] yaw %f!", packet.yaw);

          //* 发布的 joint_state
          sensor_msgs::msg::JointState joint_state;
          joint_state.header.stamp =
              node_->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          joint_state.name.push_back("pitch_joint");
          joint_state.name.push_back("yaw_joint");

          // float temp_pitch = pitch_re_trans(packet.pitch);
          joint_state.position.push_back(packet.pitch);

          // float temp_yaw = yaw_re_trans(packet.yaw);
          joint_state.position.push_back(packet.yaw);
          joint_state_pub_->publish(joint_state);

          // 速度发布
          auto_aim_interfaces::msg::Velocity current_velocity;
          current_velocity.header.stamp =
              node_->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          current_velocity.velocity = packet.current_v;
          velocity_pub_->publish(current_velocity);
        }
        else
        {
          XR_LOG_ERROR("CRC error!");
        }
      }
      else
      {
        XR_LOG_WARN("Invalid header: %02X", header[0]);
      }
    }
    catch (const std::exception& ex)
    {
      XR_LOG_ERROR("Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

//! 发送数据 视觉 -> 电控
void RMSerialDriver::sendData(
    const std::shared_ptr<const auto_aim_interfaces::msg::Send> msg)
{
  // 对齐目标号码
  const static std::map<std::string, uint8_t> id_unit8_map{
      {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
      {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  // 发
  try
  {
    // // 计算差值 pitch_diff, yaw_diff
    // float pitch_diff = 0;
    // float yaw_diff = 0;

    // pitch_diff = msg->pitch - get_parameter("joint_state").as_double();
    // yaw_diff = msg->yaw - ;

    SendPacket packet;
    packet.is_fire = 0;
    packet.x = msg->position.x;
    packet.y = msg->position.y;
    packet.z = msg->position.z;
    packet.v_yaw = msg->v_yaw;
    // std::cout<<"--------------------------------------"<<std::endl;
    // XR_LOG_INFO("[Send] pitch %f!", msg->pitch);
    // XR_LOG_INFO("[Send] yaw %f!", msg->yaw);

    packet.pitch = pitch_trans(msg->pitch);
    // std::cout<<pitch_trans(msg->pitch)<<std::endl;
    packet.yaw = yaw_trans(msg->yaw);
    // packet.pitch = pitch_trans(-0.05);s
    // packet.yaw = 0.3;

    // 关于 pitch 硬补
    // packet.pitch = 0.121;
    // packet.pitch = RMSerialDriver::pitch_trans(msg->pitch);
    // packet.yaw = RMSerialDriver::yaw_trans(msg->yaw);

    // crc对齐
    packet.checksum = LibXR::CRC16::Calculate(reinterpret_cast<uint8_t*>(&packet),
                                              sizeof(packet) - sizeof(uint16_t));

    // 打印 data 结构体中的 xyz 和 yaw 值
    // std::cout << "[Send] is_fire" << packet.is_fire << std::endl;
    // XR_LOG_INFO("[Send] aim_x %f!", packet.x);
    // XR_LOG_INFO("[Send] aim_y %f!", packet.y);
    // XR_LOG_INFO("[Send] aim_z %f!", packet.z);

    // XR_LOG_INFO(
    // "-------------------------------------------------------------");
    // XR_LOG_INFO("[Send] pitch %f!", packet.pitch);
    // XR_LOG_INFO("[Send] yaw %f!", packet.yaw);
    // XR_LOG_INFO(
    // "-------------------------------------------------------------");

    // if(packet.is_fire == true){
    //   XR_LOG_INFO("--------------开火--------------");
    // }

    //* 向串口发送数据
    // packet -> vector<uint8_t>
    std::vector<uint8_t> data = toVector(packet);

    // 串口发送
    serial_driver_->port()->send(data);

    // 延迟
    std_msgs::msg::Float64 latency;
    latency.data = (node_->now() - msg->header.stamp).seconds() * 1000.0;
    XR_LOG_DEBUG("Total latency: %fms", latency.data);

    // 错误处理
  }
  catch (const std::exception& ex)
  {
    XR_LOG_ERROR("Error while sending data: %s", ex.what());
    reopenPort();
  }
}

//! 读取参数以及错误处理
void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try
  {
    const auto pt_string = this->parity_;

    if (pt_string == "none")
    {
      pt = Parity::NONE;
    }
    else if (pt_string == "odd")
    {
      pt = Parity::ODD;
    }
    else if (pt_string == "even")
    {
      pt = Parity::EVEN;
    }
    else
    {
      throw std::invalid_argument{
          "The parity parameter must be one of: none, odd, or even."};
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    XR_LOG_ERROR("The parity provided was invalid");
    throw ex;
  }

  device_config_ =
      std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate_, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  XR_LOG_WARN("Attempting to reopen port");
  try
  {
    if (serial_driver_->port()->is_open())
    {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    XR_LOG_INFO("Successfully reopened port");
  }
  catch (const std::exception& ex)
  {
    XR_LOG_ERROR("Error while reopening port: %s", ex.what());
    if (rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter& param)
{
  // TODO：设置颜色
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready())
  {
    XR_LOG_WARN("Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  XR_LOG_INFO("Reset tracker!");
}

//! 角度换算
// [-PI,PI] -> [0,2PI] 转换
float RMSerialDriver::pitch_trans(float originAngle)
{
  if (originAngle < 0)
  {
    originAngle = originAngle + 2 * M_PI;
  }

  return originAngle;
  // return originAngle;
}

// [0,2PI] -> [-PI,PI] 转换
float RMSerialDriver::pitch_re_trans(float originAngle)
{
  if (originAngle > M_PI)
  {
    originAngle = originAngle - 2 * M_PI;
  }

  return originAngle;
  // return originAngle-M_PI;
}
// [-PI,PI] -> [0,2PI] 转换
float RMSerialDriver::yaw_trans(float originAngle)
{
  if (originAngle < 0)
  {
    originAngle = originAngle + 2 * M_PI;
  }

  return originAngle;

  // [0,2PI] -> [-PI,PI] 转换
}
float RMSerialDriver::yaw_re_trans(float originAngle)
{
  if (originAngle > M_PI)
  {
    originAngle = originAngle - 2 * M_PI;
  }
  return originAngle;
}

}  // namespace rm_serial_driver
