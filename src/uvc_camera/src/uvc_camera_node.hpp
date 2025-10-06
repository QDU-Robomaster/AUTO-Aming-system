#pragma once

#include <opencv2/opencv.hpp>

#include "camera_base.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "timebase.hpp"

// STL
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace uvc_camera
{

class UvcCameraNode
{
  static constexpr int MAX_W = 4096;
  static constexpr int MAX_H = 3072;
  static constexpr int CH = 3;  // RGB8
  static constexpr size_t BUF_BYTES = static_cast<size_t>(MAX_W) * MAX_H * CH;

 public:
  struct InitParam
  {
    std::string camera_name = "narrow_stereo";
    int image_width = 1280;
    int image_height = 720;
    // 与海康保持接口兼容（可选）：
    std::array<double, 9> camera_matrix{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 5> distortion_coefficients{0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 9> rectification_matrix{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 12> projection_matrix{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<char, 32> distortion_model = {"plumb_bob"};

    int camera_index = -1;  // -1 表示自动枚举并选择第一个可用摄像头
  };

  struct RuntimeParam
  {
    // 注意：UVC 驱动对增益/曝光的单位与语义并不完全统一；此处尽量兼容。
    float gain = 0.0f;            // OpenCV CAP_PROP_GAIN（单位依驱动而异）
    float exposure_time = 10000;  // 近似微秒；驱动可能要求对数或负值（见 .cpp 注释）
    bool manual_exposure = true;  // 尝试关闭自动曝光
    int fps = 30;                 // 期望帧率
  };

 public:
  UvcCameraNode();
  explicit UvcCameraNode(const InitParam& init, const RuntimeParam& runtime);
  ~UvcCameraNode();

  void SetRuntimeParam(const RuntimeParam& p);
  RuntimeParam GetRuntimeParam() const { return runtime_; }
  InitParam GetInitParam() const { return init_; }

 private:
  void UpdateParameters();
  bool OpenCamera();
  static std::vector<int> EnumerateCameras(int max_test = 10);

 private:
  // 数据缓冲（与海康示例保持风格一致）
  std::unique_ptr<std::array<uint8_t, BUF_BYTES>> frame_buf_{};

  // 参数
  InitParam init_{};
  RuntimeParam runtime_{};

  // 话题
  LibXR::Topic frame_topic_ = LibXR::Topic("image_raw", sizeof(cv::Mat));
  LibXR::Topic info_topic_ = LibXR::Topic("camera_info", sizeof(CameraBase::CameraInfo));

  // 设备
  std::unique_ptr<cv::VideoCapture> cap_{};
  std::string camera_name_;
  int device_index_ = -1;

  int fail_count_ = 0;
  std::atomic<bool> running_{false};
  std::thread capture_thread_{};
};

}  // namespace uvc_camera