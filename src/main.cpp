#include <memory>
#include <rclcpp/node.hpp>
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
  rm_auto_aim::ArmorDetectorNode armor_detector_node(
      true,         // debug
      1,            // detect_color
      85,           // binary_thres
      0.1,          // light_min_ratio
      0.4,          // light_max_ratio
      40.0,         // light_max_angle
      0.7,          // armor_min_light_ratio
      0.8,          // armor_min_small_center_distance
      3.2,          // armor_max_small_center_distance
      3.2,          // armor_min_large_center_distance
      5.5,          // armor_max_large_center_distance
      35.0,         // armor_max_angle
      0.6,          // classifier_threshold
      {"negative"}  // ignore_classes
  );

  // Tracker
  rm_auto_aim::ArmorTrackerNode armor_tracker_node(10.0,     // max_armor_distance
                                                   0.5,      // tracker_max_match_distance
                                                   1.0,      // tracker_max_match_yaw_diff
                                                   5,        // tracker_tracking_thres
                                                   1.0,      // tracker_lost_time_thres
                                                   0.038,    // tracker_k
                                                   10,       // tracker_bias_time
                                                   0.18375,  // tracker_s_bias
                                                   0.0,      // tracker_z_bias
                                                   0.05,     // ekf_sigma2_q_xyz
                                                   5.0,      // ekf_sigma2_q_yaw
                                                   80.0,     // ekf_sigma2_q_r
                                                   0.00025,  // ekf_r_xyz_factor
                                                   0.005,    // ekf_r_yaw
                                                   "gimbal_odom"  // target_frame
  );

  executor.add_node(armor_detector_node.node_);
  executor.add_node(armor_tracker_node.node_);

// Serial
#if 1
  rm_serial_driver::RMSerialDriver serial_driver(0.0, "/dev/ttyUSB0", 460800, "none");
  executor.add_node(serial_driver.node_);
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
  executor.add_node(camera.node_);
#endif

  // 运行 ROS 2 节点
  executor.spin();

  // 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}
