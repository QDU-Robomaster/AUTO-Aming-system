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
  rclcpp::NodeOptions serial_options;
  serial_options.parameter_overrides({{"device_name", "/dev/ttyACM0"},
                                      {"baud_rate", 460800},
                                      {"flow_control", "none"},
                                      {"parity", "none"},
                                      {"stop_bits", "1"}});
  auto serial_driver_node =
      std::make_shared<rm_serial_driver::RMSerialDriver>(serial_options);
  executor.add_node(serial_driver_node);
#endif

// Camera
#if 1
  rclcpp::NodeOptions camera_options;

  // 手动设置相机参数（直接在 C++ 代码中传递）
  camera_options.parameter_overrides(
      {{"camera_name", "narrow_stereo"},
       {"exposure_time", 500},  // 曝光时间（单位：微秒）
       {"gain", 32.0},          // 增益
       {"image_width", 1440},   // 图像宽度
       {"image_height", 1080},  // 图像高度
       {"camera_matrix",
        std::vector<double>{2340.46464112537, 0.0, 713.3224120377864, 0.0,
                            2336.8745144649124, 547.4106752074272, 0.0, 0.0, 1.0}},
       {"distortion_model", "plumb_bob"},
       {"distortion_coefficients",
        std::vector<double>{-0.09558691800515781, 0.3013704144837407,
                            -0.0008218465102445683, 0.00024582434306615617, 0.0}},
       {"rectification_matrix",
        std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
       {"projection_matrix",
        std::vector<double>{2323.906982421875, 0.0, 712.9446224841959, 0.0, 0.0,
                            2324.767578125, 546.6426169058832, 0.0, 0.0, 0.0, 1.0,
                            0.0}}});

  auto hik_camera_node = std::make_shared<hik_camera::HikCameraNode>(camera_options);
  executor.add_node(hik_camera_node);
#endif

  // 运行 ROS 2 节点
  executor.spin();

  // 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}
