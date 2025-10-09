#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "BlinkLED.hpp"
#include "UvcCamera.hpp"
#include "CameraBase.hpp"
#include "HikCamera.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "RMSerialDriver.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static HikCamera HikCamera_0(hw, appmgr, {1440, 1080, 4320, 0, CameraBase::Encoding::RGB8, {2340.46464112537, 0.0, 713.3224120377864, 0.0, 2336.8745144649124, 547.4106752074272, 0.0, 0.0, 1.0}, CameraBase::DistortionModel::PLUMB_BOB, {-0.09558691800515781, 0.3013704144837407, -0.0008218465102445683, 0.00024582434306615617, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, {2323.906982421875, 0.0, 712.9446224841959, 0.0, 0.0, 2324.767578125, 546.6426169058832, 0.0, 0.0, 0.0, 1.0, 0.0}}, {32.0, 600.0});
  static ArmorDetector ArmorDetector_0(hw, appmgr, {{{ArmorNumber::NEGATIVE}, 0.7}, 1, 85, {0.1, 0.4, 40.0}, {0.7, 0.8, 3.2, 3.2, 5.5, 35.0}});
  static ArmorTracker ArmorTracker_0(hw, appmgr, {{10.0}, {0.15, 1.0}, {5, 0.3}, {0.092, 100, 0.19133, 0.21265}, {20.0, 100.0, 800}, {0.05, 0.02}, {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}});
  static RMSerialDriver RMSerialDriver_0(hw, appmgr, 0.0, "/dev/ttyUSB0", 115200, LibXR::UART::Parity::NO_PARITY);

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}