#include <rclcpp/rclcpp.hpp>

#include "detector_node.hpp"
#include "hik_camera_node.hpp"
#include "libxr.hpp"
#include "libxr_system.hpp"
#include "rm_serial_driver.hpp"
#include "tracker_node.hpp"

int main(int argc, char** argv)
{
  LibXR::PlatformInit();

  // 初始化 ROS 2 节点
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  // Detector
  rclcpp::NodeOptions detector_options;

  auto detector_node = std::make_shared<rm_auto_aim::ArmorDetectorNode>(detector_options);

  executor.add_node(detector_node);

  // Tracker
  rclcpp::NodeOptions tracker_options;

  auto tracker_node = std::make_shared<rm_auto_aim::ArmorTrackerNode>(tracker_options);

  executor.add_node(tracker_node);

// Serial
#if 1
  rm_serial_driver::RMSerialDriver serial_driver(0.0, "/dev/ttyACM0", 460800, "none");
#endif

// Camera
#if 1
  hik_camera::HikCameraNode camera(
      "narrow_stereo", 1440, 1080,
      {{2340.46464112537, 0.0, 713.3224120377864, 0.0, 2336.8745144649124,
        547.4106752074272, 0.0, 0.0, 1.0}},
      {{-0.09558691800515781, 0.3013704144837407, -0.0008218465102445683,
        0.00024582434306615617, 0.0}},
      {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
      {{2323.906982421875, 0.0, 712.9446224841959, 0.0, 0.0, 2324.767578125,
        546.6426169058832, 0.0, 0.0, 0.0, 1.0, 0.0}},
      "plumb_bob", true, 32.0, 500.0);
#endif

  // 运行 ROS 2 节点
  executor.spin();

  // 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}
