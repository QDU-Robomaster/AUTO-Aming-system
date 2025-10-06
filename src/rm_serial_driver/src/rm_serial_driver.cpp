#include "rm_serial_driver.hpp"

// C++ system
#include <math.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "crc.hpp"
#include "libxr_rw.hpp"
#include "libxr_type.hpp"
#include "linux_uart.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "packet.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "tracker.hpp"
#include "tracker_node.hpp"

// 串口驱动
namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(double timestamp_offset, std::string device_name,
                               int baud_rate, LibXR::UART::Parity parity)
    : device_name_{device_name},
      baud_rate_{baud_rate},
      parity_{parity},
      timestamp_offset_(timestamp_offset)
{
  //! serial_driver  第一部分 参数设置
  // INFO打印
  XR_LOG_INFO("Start RMSerialDriver!");

  //! serial_driver 第二部分 串口初始化以及收发
  uart_ =
      new LibXR::LinuxUART(device_name_.c_str(), baud_rate_, parity_, 8, 1, 128, 8192);

  receive_thread_.Create<RMSerialDriver*>(
      this, [](RMSerialDriver* serial) { serial->receiveData(); }, "RecvThread", 512,
      LibXR::Thread::Priority::HIGH);

  LibXR::Topic send_topic = LibXR::Topic::Find("/tracker/send");

  auto send_cb = LibXR::Topic::Callback::Create(
      [](bool, RMSerialDriver* serial, LibXR::RawData& data)
      {
        XR_LOG_DEBUG("SerialDriver send data");
        auto send_data =
            reinterpret_cast<rm_auto_aim::ArmorTrackerNode::Send*>(data.addr_);
        serial->sendData(*send_data);
      },
      this);

  send_topic.RegisterCallback(send_cb);

  // target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "/tracker/target", rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

//* 析构
RMSerialDriver::~RMSerialDriver() {}

//! 接受数据 电控 -> 视觉
void RMSerialDriver::receiveData()
{
  uint8_t header(1);
  ReceivePacket data;
  LibXR::Semaphore sem;
  LibXR::ReadOperation read_op(sem);

  while (true)
  {
    uart_->Read(header, read_op);

    if (header == 0x5A)
    {
      // data.resize(sizeof(ReceivePacket) - 1);
      // serial_driver_->port()->receive(data);
      uart_->Read({&data.header + 1, sizeof(ReceivePacket) - 1}, read_op);

      ReceivePacket& packet = data;

      // CRC校验
      bool crc_ok =
          LibXR::CRC16::Verify(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));

      if (crc_ok)
      {
        if (!initial_set_param_ || packet.detect_color != previous_receive_color_)
        {
          // TODO: 发布阵营
          previous_receive_color_ = packet.detect_color;
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

        // 速度发布
        double current_velocity;
        current_velocity = packet.current_v;
        velocity_topic_.Publish(current_velocity);
      }
      else
      {
        XR_LOG_ERROR("CRC error!");
      }
    }
    else
    {
      XR_LOG_WARN("Invalid header: %02X", header);
    }
  }
}

//! 发送数据 视觉 -> 电控
void RMSerialDriver::sendData(rm_auto_aim::ArmorTrackerNode::Send msg)
{
  static LibXR::WriteOperation write_op;

  // 对齐目标号码
  const static std::map<std::string, uint8_t> id_unit8_map{
      {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
      {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  // // 计算差值 pitch_diff, yaw_diff
  // float pitch_diff = 0;
  // float yaw_diff = 0;

  // pitch_diff = msg->pitch - get_parameter("joint_state").as_double();
  // yaw_diff = msg->yaw - ;

  SendPacket packet;
  packet.is_fire = 0;
  packet.x = msg.position.x();
  packet.y = msg.position.y();
  packet.z = msg.position.z();
  packet.v_yaw = msg.v_yaw;
  // std::cout<<"--------------------------------------"<<std::endl;
  // XR_LOG_INFO("[Send] pitch %f!", msg->pitch);
  // XR_LOG_INFO("[Send] yaw %f!", msg->yaw);

  packet.pitch = pitch_trans(msg.pitch);
  // std::cout<<pitch_trans(msg->pitch)<<std::endl;
  packet.yaw = yaw_trans(msg.yaw);
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
  uart_->Write(packet, write_op);

  // 错误处理
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
