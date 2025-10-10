#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "app_framework.hpp"
#include "libxr.hpp"
#include "libxr_rw.hpp"
#include "libxr_system.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "terminal.hpp"
#include "thread.hpp"
#include "xrobot_main.hpp"

void (*log_cb_fun)(bool in_isr, LibXR::Topic, LibXR::RawData &log_data) =
    [](bool, LibXR::Topic tp, LibXR::RawData &log_data)
{
  UNUSED(tp);

  auto log = reinterpret_cast<LibXR::LogData *>(log_data.addr_);

  if (LibXR::STDIO::write_ && LibXR::STDIO::write_->Writable())
  {
    using clock = std::chrono::system_clock;

    static std::ofstream f;
    if (!f.is_open())
    {
      auto now = clock::now();
      std::time_t t = clock::to_time_t(now);
      std::tm tm{};
      localtime_r(&t, &tm);

      std::ostringstream oss;
      // 首次打开时按启动时间命名：YYYYMMDD_HHMMSS.log
      oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".log";
      f.open(oss.str(), std::ios::out | std::ios::app);

      LibXR::STDIO::Printf("Log written to %s\n", oss.str().c_str());
    }

    if (f)
    {
      auto now = clock::now();
      auto ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
          1000;
      std::time_t t2 = clock::to_time_t(now);
      std::tm tm2{};
      localtime_r(&t2, &tm2);
      char ts[32];
      (void)std::strftime(ts, sizeof(ts), "%F %T", &tm2);  // "YYYY-MM-DD HH:MM:SS"

      f << '[' << ts << '.' << std::setw(3) << std::setfill('0') << ms.count()
        << std::setfill(' ') << "][" << static_cast<unsigned>(log->level) << "] "
        << (log->file ? log->file : "?") << ':' << log->line << ' ' << log->message
        << '\n';
      f.flush();
    }
  }
};

int main(int, char **)
{
  LibXR::PlatformInit();

  XR_LOG_PASS("Platform initialized");

  LibXR::RamFS ramfs;

  LibXR::Terminal<1024, 64, 16, 128> terminal(ramfs);

  LibXR::Thread term_thread;
  term_thread.Create(&terminal, LibXR::Terminal<1024, 64, 16, 128>::ThreadFun, "terminal",
                     512, LibXR::Thread::Priority::MEDIUM);

  auto log_topic = LibXR::Topic(LibXR::Topic::Find("/xr/log"));
  auto log_cb = LibXR::Topic::Callback::Create(log_cb_fun, log_topic);
  log_topic.RegisterCallback(log_cb);

  LibXR::HardwareContainer peripherals{
      LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
  };
  XRobotMain(peripherals);
  while (1)
  {
    LibXR::Thread::Sleep(1000);
  }
  return 0;
}
