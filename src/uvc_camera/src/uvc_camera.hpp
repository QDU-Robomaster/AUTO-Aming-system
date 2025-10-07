#pragma once

#include <opencv2/core/mat.hpp>

#include "camera_base.hpp"
#include "libxr.hpp"
#include "message.hpp"

// STL
#include <array>
#include <cstdint>
#include <memory>
#include <opencv2/videoio.hpp>

class UvcCamera
{
  static constexpr int MAX_W = 4096;
  static constexpr int MAX_H = 3072;
  static constexpr int CH = 3;  // RGB8
  static constexpr size_t BUF_BYTES = static_cast<size_t>(MAX_W) * MAX_H * CH;

 public:
  struct RuntimeParam
  {
    // Note: UVC semantics vary by driver; we follow OpenCV props as closely as possible.
    float gain = 0.0f;  // cv::CAP_PROP_GAIN (units driver-specific)
    float exposure_time =
        10000;  // Often microseconds; some drivers expect EV/negative values
    bool manual_exposure = true;  // Try to force manual exposure
    int fps = 30;                 // Requested FPS (best-effort)
  };

 public:
  // Convenience default constructor
  UvcCamera();

  // Main constructor: keep API similar to Hik node
  explicit UvcCamera(const CameraBase::CameraInfo& info, const RuntimeParam& runtime);

  ~UvcCamera();

  void SetRuntimeParam(const RuntimeParam& p);
  RuntimeParam GetRuntimeParam() const { return runtime_; }

 private:
  void UpdateParameters();
  bool OpenCamera();
  static std::vector<int> EnumerateCameras(int max_test = 10);
  static void ThreadFun(UvcCamera* self);

 private:
  // Large preallocated RGB buffer (matches Hik style)
  std::unique_ptr<std::array<uint8_t, BUF_BYTES>> frame_buf_{};

  // Parameters/state
  CameraBase::CameraInfo info_{};
  RuntimeParam runtime_{};

  // Topics (same names as Hik)
  LibXR::Topic frame_topic_ = LibXR::Topic("image_raw", sizeof(cv::Mat));
  LibXR::Topic info_topic_ = LibXR::Topic("camera_info", sizeof(CameraBase::CameraInfo));

  // Device
  std::unique_ptr<cv::VideoCapture> cap_{};
  int device_index_ = -1;
  int fail_count_ = 0;
  std::atomic<bool> running_{false};
  LibXR::Thread capture_thread_{};
};
