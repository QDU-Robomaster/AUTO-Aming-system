#include "camera_base.hpp"
#include "detector_node.hpp"
#include "hik_camera.hpp"
#include "libxr.hpp"
#include "libxr_system.hpp"
#include "logger.hpp"
#include "ramfs.hpp"
#include "rm_serial_driver.hpp"
#include "terminal.hpp"
#include "thread.hpp"
#include "tracker_node.hpp"
#include "uart.hpp"
#include "uvc_camera.hpp"

int main(int, char**)
{
  LibXR::PlatformInit();

  XR_LOG_PASS("Platform initialized");

  LibXR::RamFS ramfs;

  LibXR::Terminal<1024, 64, 16, 128> terminal(ramfs);

  LibXR::Thread term_thread;
  term_thread.Create(&terminal, LibXR::Terminal<1024, 64, 16, 128>::ThreadFun, "terminal",
                     512, LibXR::Thread::Priority::MEDIUM);

  LibXR::Thread::Sleep(100);

  // Camera
#if 0
  // ===== HIK (new API): CameraBase::CameraInfo + RuntimeParam =====
  CameraBase::CameraInfo info{};
  info.width  = 1440;
  info.height = 1080;

  // 内参
  info.camera_matrix = {2340.46464112537,
                        0.0,
                        713.3224120377864,
                        0.0,
                        2336.8745144649124,
                        547.4106752074272,
                        0.0,
                        0.0,
                        1.0};
  info.distortion_coefficients = {-0.09558691800515781, 0.3013704144837407,
                                  -0.0008218465102445683, 0.00024582434306615617, 0.0};
  info.rectification_matrix = {1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
  info.projection_matrix = {2323.906982421875, 0.0, 712.9446224841959, 0.0,
                            0.0, 2324.767578125, 546.6426169058832, 0.0,
                            0.0, 0.0, 1.0, 0.0};
  std::snprintf(info.distortion_model.data(), info.distortion_model.size(), "%s", "plumb_bob");

  hik_camera::HikCameraNode::RuntimeParam runtime{};
  runtime.gain = 32.0f;
  runtime.exposure_time = 500.0f;  // microseconds

  hik_camera::HikCameraNode camera(info, runtime);

#else
  // ===== UVC (new API): CameraBase::CameraInfo + RuntimeParam =====
  CameraBase::CameraInfo info{};
  info.width = 1280;  // 期望分辨率（>0 则作为请求值）
  info.height = 720;

  // 内参（沿用你原值）
  info.camera_matrix = {2340.46464112537,
                        0.0,
                        713.3224120377864,
                        0.0,
                        2336.8745144649124,
                        547.4106752074272,
                        0.0,
                        0.0,
                        1.0};
  info.distortion_coefficients.plumb_bob = {-0.09558691800515781, 0.3013704144837407,
                                            -0.0008218465102445683,
                                            0.00024582434306615617, 0.0};
  info.rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.projection_matrix = {2323.906982421875,
                            0.0,
                            712.9446224841959,
                            0.0,
                            0.0,
                            2324.767578125,
                            546.6426169058832,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0};
  info.distortion_model = CameraBase::DistortionModel::PLUMB_BOB;

  UvcCamera::RuntimeParam run{};
  run.fps = 30;
  run.manual_exposure = true;
  run.exposure_time = 600;  // 某些驱动需用 -5~-1（EV 风格）可自行尝试
  run.gain = 0.0f;

  UvcCamera node(info, run);
#endif

  LibXR::Thread::Sleep(100);

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
  rm_auto_aim::ArmorTrackerNode armor_tracker_node(10.0,  // max_armor_distance
                                                   0.5, 1.0, 5, 1.0, 0.038, 10, 0.18375,
                                                   0.0, 0.05, 5.0, 80.0, 0.00025, 0.005,
                                                   "gimbal_odom");
  LibXR::Thread::Sleep(100);

// Serial
#if 1
  rm_serial_driver::RMSerialDriver serial_driver(0.0, "/dev/ttyACM0", 460800,
                                                 LibXR::UART::Parity::NO_PARITY);
#endif

  //   // 运行 ROS 2 节点
  //   executor.spin();

  //   // 关闭 ROS 2
  //   rclcpp::shutdown();
  while (1)
  {
    LibXR::Thread::Sleep(1000);
  }
  return 0;
}
