#include "app_framework.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "ramfs.hpp"
#include "terminal.hpp"
#include "thread.hpp"
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
  while (1)
  {
    LibXR::Thread::Sleep(1000);
  }
  return 0;
}
