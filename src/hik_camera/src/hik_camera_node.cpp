#include "hik_camera_node.hpp"

#include <cstdio>  // std::snprintf

namespace hik_camera
{

HikCameraNode::HikCameraNode() : HikCameraNode(InitParam{}, RuntimeParam{}) {}

HikCameraNode::HikCameraNode(const InitParam& init, const RuntimeParam& runtime)
    : init_(init), runtime_(runtime), camera_name_(init.camera_name)
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
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }

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
  capture_thread_ = std::thread{
      [this]() -> void
      {
        MV_FRAME_OUT out_frame{};

        XR_LOG_INFO("Publishing image!");

        CameraBase::CameraInfo info{0u,
                                    0u,
                                    0u,
                                    0,
                                    {0},
                                    init_.camera_matrix,
                                    init_.distortion_coefficients,
                                    init_.rectification_matrix,
                                    init_.projection_matrix,
                                    init_.distortion_model};
        (void)std::snprintf(info.encoding, sizeof(info.encoding), "%s", "rgb8");

        while (running_.load())
        {
          auto ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);

          if (MV_OK == ret)
          {
            const int W = out_frame.stFrameInfo.nWidth;
            const int H = out_frame.stFrameInfo.nHeight;

            info.width = static_cast<uint32_t>(W);
            info.height = static_cast<uint32_t>(H);
            info.step = static_cast<uint32_t>(W * CH);
            info.timestamp = LibXR::Timebase::GetMicroseconds();

            convert_param_.pSrcData = out_frame.pBufAddr;
            convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
            MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
            MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

            cv::Mat img(H, W, CV_8UC3, frame_buf_->data(), static_cast<size_t>(W) * CH);

            frame_topic_.Publish(img);
            info_topic_.Publish(info);

            fail_count_ = 0;
          }
          else
          {
            XR_LOG_WARN("Get buffer failed! nRet: [%x]", ret);
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_StartGrabbing(camera_handle_);
            fail_count_++;
          }

          if (fail_count_ > 5)
          {
            XR_LOG_ERROR("Camera failed!");
          }
        }
      }};
}

HikCameraNode::~HikCameraNode()
{
  running_.store(false);
  if (capture_thread_.joinable())
  {
    capture_thread_.join();
  }
  if (camera_handle_)
  {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
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

}  // namespace hik_camera
