#include "CameraBase.hpp"
#include "HikCamera.hpp"
#include "app_framework.hpp"
#include "armor.hpp"
#include "libxr.hpp"
#include "libxr_system.hpp"
#include "logger.hpp"
#include "ramfs.hpp"
#include "terminal.hpp"
#include "thread.hpp"
#include "uart.hpp"
#include "xrobot_main.hpp"

int main(int, char**)
{
  LibXR::PlatformInit();

  XR_LOG_PASS("Platform initialized");

  LibXR::RamFS ramfs;

  LibXR::Terminal<1024, 64, 16, 128> terminal(ramfs);

  LibXR::Thread term_thread;
  term_thread.Create(&terminal, LibXR::Terminal<1024, 64, 16, 128>::ThreadFun, "terminal",
                     512, LibXR::Thread::Priority::MEDIUM);

  LibXR::HardwareContainer hw;
  XRobotMain(hw);

  //   ArmorDetector::Config cfg{{{ArmorNumber::NEGATIVE}, 0.6}};
  //   cfg.detect_color = 1;
  //   cfg.binary_thres = 85;
  //   cfg.light.min_ratio = 0.1;
  //   cfg.light.max_ratio = 0.4;
  //   cfg.light.max_angle = 40.0;
  //   cfg.armor.min_light_ratio = 0.7;
  //   cfg.armor.min_small_center_distance = 0.8;
  //   cfg.armor.max_small_center_distance = 3.2;
  //   cfg.armor.min_large_center_distance = 3.2;
  //   cfg.armor.max_large_center_distance = 5.5;
  //   cfg.armor.max_angle = 35.0;

  //   ArmorDetector armor_detector_node(cfg);

  //   // Tracker
  //   ArmorTrackerNode armor_tracker_node(10.0,  // max_armor_distance
  //                                       0.5, 1.0, 5, 1.0, 0.038, 10, 0.18375, 0.0,
  //                                       0.05, 5.0, 80.0, 0.00025, 0.005);

  // // Serial
  // #if 1
  //   rm_serial_driver::RMSerialDriver serial_driver(0.0, "/dev/ttyACM0", 460800,
  //                                                  LibXR::UART::Parity::NO_PARITY);
  // #endif

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
