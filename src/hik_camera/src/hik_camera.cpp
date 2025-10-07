#include "hik_camera.hpp"

#include "MvCameraControl.h"
#include "camera_base.hpp"
#include "logger.hpp"
#include "thread.hpp"

namespace hik_camera
{

HikCameraNode::HikCameraNode() : HikCameraNode(CameraBase::CameraInfo{}, RuntimeParam{})
{
}

HikCameraNode::HikCameraNode(const CameraBase::CameraInfo& info,
                             const RuntimeParam& runtime)
    : info_(info), runtime_(runtime)
{
  XR_LOG_INFO("Starting HikCameraNode!");

  MV_CC_DEVICE_INFO_LIST device_list{};
  // enum device
  auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  XR_LOG_INFO("Found camera count = %d", device_list.nDeviceNum);

  while (device_list.nDeviceNum == 0)
  {
    XR_LOG_ERROR("No camera found!");
    XR_LOG_INFO("Enum state: [%x]", ret);
    LibXR::Thread::Sleep(1000);
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }

  XR_LOG_PASS("Hik Camera found!");

  MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
  MV_CC_OpenDevice(camera_handle_);

  // Get camera information
  MV_CC_GetImageInfo(camera_handle_, &img_info_);

  frame_buf_ = std::make_unique<std::array<uint8_t, BUF_BYTES>>();

  // Init convert param
  convert_param_ = {};
  convert_param_.nWidth = img_info_.nWidthValue;
  convert_param_.nHeight = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
  convert_param_.pDstBuffer = frame_buf_->data();
  convert_param_.nDstBufferSize = BUF_BYTES;

  UpdateParameters();

  MV_CC_StartGrabbing(camera_handle_);

  running_.store(true);
  capture_thread_.Create(this, ThreadFun, "HikCameraThread",
                         static_cast<size_t>(1024 * 128),
                         LibXR::Thread::Priority::REALTIME);
}

HikCameraNode::~HikCameraNode()
{
  if (camera_handle_)
  {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(reinterpret_cast<void*>(camera_handle_));
    camera_handle_ = nullptr;
  }
  XR_LOG_INFO("HikCameraNode destroyed!");
}

void HikCameraNode::SetRuntimeParam(const RuntimeParam& p)
{
  runtime_ = p;
  UpdateParameters();
}

void HikCameraNode::UpdateParameters()
{
  // Exposure
  MVCC_FLOATVALUE f_value{};
  MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
  runtime_.exposure_time = std::clamp(runtime_.exposure_time, f_value.fMin, f_value.fMax);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", runtime_.exposure_time);
  XR_LOG_INFO("Exposure time: %f", runtime_.exposure_time);

  // Gain
  MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
  runtime_.gain = std::clamp(runtime_.gain, f_value.fMin, f_value.fMax);
  MV_CC_SetFloatValue(camera_handle_, "Gain", runtime_.gain);
  XR_LOG_INFO("Gain: %f", runtime_.gain);
}

void HikCameraNode::ThreadFun(HikCameraNode* self)
{
  MV_FRAME_OUT out_frame{};

  XR_LOG_INFO("Publishing image!");

  while (self->running_.load())
  {
    auto ret = MV_CC_GetImageBuffer(self->camera_handle_, &out_frame, 1000);

    if (MV_OK == ret)
    {
      const int W = out_frame.stFrameInfo.nWidth;
      const int H = out_frame.stFrameInfo.nHeight;

      self->info_.width = static_cast<uint32_t>(W);
      self->info_.height = static_cast<uint32_t>(H);
      self->info_.step = static_cast<uint32_t>(W * CH);
      self->info_.timestamp = LibXR::Timebase::GetMicroseconds();

      self->convert_param_.pSrcData = out_frame.pBufAddr;
      self->convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
      self->convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
      MV_CC_ConvertPixelType(self->camera_handle_, &self->convert_param_);
      MV_CC_FreeImageBuffer(self->camera_handle_, &out_frame);

      cv::Mat img(H, W, CV_8UC3, self->frame_buf_->data(), static_cast<size_t>(W) * CH);

      self->frame_topic_.Publish(img);
      self->info_topic_.Publish(self->info_);

      self->fail_count_ = 0;
    }
    else
    {
      XR_LOG_WARN("Get buffer failed! nRet: [%x]", ret);
      MV_CC_StopGrabbing(self->camera_handle_);
      MV_CC_StartGrabbing(self->camera_handle_);
      self->fail_count_++;
    }

    if (self->fail_count_ > 5)
    {
      XR_LOG_ERROR("Camera failed!");
    }
  }
}

}  // namespace hik_camera
