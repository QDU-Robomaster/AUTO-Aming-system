#pragma once

#include "MvCameraControl.h"
#include "camera_base.hpp"
#include "libxr.hpp"
#include "message.hpp"

// STL
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <opencv2/core/mat.hpp>

namespace hik_camera
{

class HikCameraNode
{
  static constexpr int MAX_W = 4096;
  static constexpr int MAX_H = 3072;
  static constexpr int CH = 3;
  static constexpr size_t BUF_BYTES = static_cast<size_t>(MAX_W) * MAX_H * CH;

 public:
  struct RuntimeParam
  {
    float gain = 32.0f;
    float exposure_time = 500.0f;  // microseconds
  };

 public:
  // Convenience default constructor
  HikCameraNode();

  // Main constructor (definition in .cpp)
  explicit HikCameraNode(const CameraBase::CameraInfo& info, const RuntimeParam& runtime);

  ~HikCameraNode();

  void SetRuntimeParam(const RuntimeParam& p);

 private:
  void UpdateParameters();

  static void ThreadFun(HikCameraNode* self);

  // Runtime state
  std::unique_ptr<std::array<uint8_t, BUF_BYTES>> frame_buf_{};  // large RGB buffer

  // Parameters
  CameraBase::CameraInfo info_;
  RuntimeParam runtime_{};

  // Topics
  LibXR::Topic frame_topic_ = LibXR::Topic("image_raw", sizeof(cv::Mat));
  LibXR::Topic info_topic_ = LibXR::Topic("camera_info", sizeof(CameraBase::CameraInfo));

  // Camera handle & info
  void* camera_handle_{};
  MV_IMAGE_BASIC_INFO img_info_{};
  MV_CC_PIXEL_CONVERT_PARAM convert_param_{};

  int fail_count_ = 0;
  std::atomic<bool> running_{false};
  LibXR::Thread capture_thread_{};
};

}  // namespace hik_camera
