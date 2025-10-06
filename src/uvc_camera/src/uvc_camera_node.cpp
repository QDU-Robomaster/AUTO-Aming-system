#include "uvc_camera_node.hpp"

#include <cstdio>
#include <opencv2/highgui.hpp>

#include "camera_base.hpp"

namespace uvc_camera
{

UvcCameraNode::UvcCameraNode() : UvcCameraNode(InitParam{}, RuntimeParam{}) {}

std::vector<int> UvcCameraNode::EnumerateCameras(int max_test)
{
  std::vector<int> ids;
  auto try_open = [&](int idx, int backend)
  {
    cv::VideoCapture t(idx, backend);
    if (!t.isOpened()) return false;
    cv::Mat tmp;
    bool ok = t.read(tmp) && !tmp.empty();
    t.release();
    if (ok)
    {
      XR_LOG_INFO("Candidate camera index %d works on backend %d", idx, backend);
    }
    return ok;
  };

  for (int i = 0; i < max_test; ++i)
  {
    if (try_open(i, cv::CAP_V4L2) || try_open(i, cv::CAP_ANY))
    {
      ids.push_back(i);
    }
    else
    {
      XR_LOG_WARN("/dev/video%d is not a capture device or no frames.", i);
    }
  }
  return ids;
}

UvcCameraNode::UvcCameraNode(const InitParam& init, const RuntimeParam& runtime)
    : init_(init), runtime_(runtime), camera_name_(init.camera_name)
{
  XR_LOG_INFO("Starting UvcCameraNode (OpenCV UVC)!");

  // 1) 枚举并选择设备
  auto ids = EnumerateCameras(10);
  XR_LOG_INFO("Found camera count = %d", static_cast<int>(ids.size()));

  while (ids.empty())
  {
    XR_LOG_ERROR("No camera found! Retrying...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ids = EnumerateCameras(10);
  }

  device_index_ = (init_.camera_index >= 0) ? init_.camera_index : ids.front();

  // 2) 打开设备
  if (!OpenCamera())
  {
    XR_LOG_ERROR("Failed to open camera index %d", device_index_);
    throw std::runtime_error("UvcCameraNode: cannot open camera");
  }

  // 3) 预分配最大缓冲
  frame_buf_ = std::make_unique<std::array<uint8_t, BUF_BYTES>>();

  // 4) 参数设定
  UpdateParameters();

  // 5) 启动抓帧线程
  running_.store(true);
  capture_thread_ = std::thread{
      [this]()
      {
        XR_LOG_INFO("Publishing image!");

        CameraBase::CameraInfo info{
            0u,
            0u,
            0u,
            0,
            {0},
            init_.camera_matrix,
            init_.distortion_coefficients,
            init_.rectification_matrix,
            init_.projection_matrix,
            init_.distortion_model,
        };
        (void)std::snprintf(info.encoding, sizeof(info.encoding), "%s", "rgb8");

        while (running_.load())
        {
          cv::Mat frame_bgr;
          if (cap_ && cap_->isOpened() && cap_->read(frame_bgr) && !frame_bgr.empty())
          {
            const int W = frame_bgr.cols;
            const int H = frame_bgr.rows;

            // 目标 RGB 缓冲区（与海康示例一致，使用预分配内存）
            cv::Mat img_rgb(H, W, CV_8UC3, frame_buf_->data(),
                            static_cast<size_t>(W) * CH);
            cv::cvtColor(frame_bgr, img_rgb, cv::COLOR_BGR2RGB);

            info.width = static_cast<uint32_t>(W);
            info.height = static_cast<uint32_t>(H);
            info.step = static_cast<uint32_t>(W * CH);
            info.timestamp = LibXR::Timebase::GetMicroseconds();

            // 发布
            frame_topic_.Publish(img_rgb);
            info_topic_.Publish(info);
            fail_count_ = 0;
          }
          else
          {
            XR_LOG_WARN("Grab frame failed! Try reopen.");
            fail_count_++;

            // 尝试重开设备
            if (cap_) cap_->release();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            OpenCamera();
          }

          if (fail_count_ > 5)
          {
            XR_LOG_ERROR("Camera failed repeatedly (%d)!", fail_count_);
          }
        }
      }};
}

bool UvcCameraNode::OpenCamera()
{
  auto open_with = [&](int backend, int fourcc, const char* tag) -> bool
  {
    cap_ = std::make_unique<cv::VideoCapture>(device_index_, backend);
    if (!cap_ || !cap_->isOpened()) return false;

    if (fourcc) cap_->set(cv::CAP_PROP_FOURCC, fourcc);
    if (init_.image_width > 0) cap_->set(cv::CAP_PROP_FRAME_WIDTH, init_.image_width);
    if (init_.image_height > 0) cap_->set(cv::CAP_PROP_FRAME_HEIGHT, init_.image_height);
    if (runtime_.fps > 0) cap_->set(cv::CAP_PROP_FPS, runtime_.fps);
    cap_->set(cv::CAP_PROP_BUFFERSIZE, 1);

    // 暖机尝试读取若干帧
    cv::Mat fr;
    bool ok = false;
    for (int t = 0; t < 10; ++t)
    {
      if (cap_->read(fr) && !fr.empty())
      {
        ok = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!ok)
    {
      XR_LOG_WARN("Open %s but no frames, closing...", tag);
      cap_->release();
      cap_.reset();
      return false;
    }

    XR_LOG_INFO("Open success via %s | actual WxH=%.0fx%.0f fps=%.1f", tag,
                cap_->get(cv::CAP_PROP_FRAME_WIDTH), cap_->get(cv::CAP_PROP_FRAME_HEIGHT),
                cap_->get(cv::CAP_PROP_FPS));
    return true;
  };

  int MJPG = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  int YUYV = cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');  // 有些驱动叫 YUY2

  if (open_with(cv::CAP_V4L2, MJPG, "V4L2+MJPG")) return true;
  if (open_with(cv::CAP_V4L2, YUYV, "V4L2+YUYV")) return true;
  if (open_with(cv::CAP_V4L2, 0, "V4L2(no FOURCC)")) return true;
  if (open_with(cv::CAP_ANY, MJPG, "ANY+MJPG")) return true;
  if (open_with(cv::CAP_ANY, 0, "ANY(no FOURCC)")) return true;

  // 可选：GStreamer 管线兜底（OpenCV 启用 GStreamer 时有效）
  {
    std::string dev = "/dev/video" + std::to_string(device_index_);
    std::ostringstream pipe;
    int w = init_.image_width > 0 ? init_.image_width : 1280;
    int h = init_.image_height > 0 ? init_.image_height : 720;
    int f = runtime_.fps > 0 ? runtime_.fps : 30;
    pipe << "v4l2src device=" << dev << " ! image/jpeg,framerate=" << f
         << "/1,width=" << w << ",height=" << h
         << " ! jpegdec ! videoconvert ! appsink drop=1 sync=0";

    cap_ = std::make_unique<cv::VideoCapture>(pipe.str(), cv::CAP_GSTREAMER);
    if (cap_ && cap_->isOpened())
    {
      cv::Mat fr;
      if (cap_->read(fr) && !fr.empty())
      {
        XR_LOG_INFO("Open success via GST pipeline: %s", pipe.str().c_str());
        return true;
      }
      cap_->release();
      cap_.reset();
    }
  }

  return false;
}

UvcCameraNode::~UvcCameraNode()
{
  running_.store(false);
  if (capture_thread_.joinable())
  {
    capture_thread_.join();
  }
  if (cap_)
  {
    cap_->release();
    cap_.reset();
  }
  XR_LOG_INFO("UvcCameraNode destroyed!");
}

void UvcCameraNode::SetRuntimeParam(const RuntimeParam& p)
{
  runtime_ = p;
  UpdateParameters();
}

static void try_set(cv::VideoCapture& cap, int prop, double value, const char* name)
{
  bool ok = cap.set(prop, value);
  XR_LOG_INFO("Set %s = %.3f (%s)", name, value, ok ? "ok" : "no");
}

void UvcCameraNode::UpdateParameters()
{
  if (!cap_ || !cap_->isOpened()) return;

  // 分辨率/FPS（再设一次，确保生效）
  if (init_.image_width > 0)
    try_set(*cap_, cv::CAP_PROP_FRAME_WIDTH, init_.image_width, "width");
  if (init_.image_height > 0)
    try_set(*cap_, cv::CAP_PROP_FRAME_HEIGHT, init_.image_height, "height");
  if (runtime_.fps > 0) try_set(*cap_, cv::CAP_PROP_FPS, runtime_.fps, "fps");

  // 曝光控制：不同平台/驱动值域差异很大
  // - V4L2: CAP_PROP_AUTO_EXPOSURE 使用 1 为手动、3 为自动（或 0.25/0.75 的浮点表征）
  // - Windows: 常见要求 CAP_PROP_EXPOSURE 为对数或负值（例如 -4 ~ -1）
  // 这里做最大努力：先尝试手动，再设定曝光值；失败不报错但打印日志。
  if (runtime_.manual_exposure)
  {
    // 两种常见写法都试一下
    try_set(*cap_, cv::CAP_PROP_AUTO_EXPOSURE, 1.0, "auto_exposure=manual(1.0)");
    try_set(*cap_, cv::CAP_PROP_AUTO_EXPOSURE, 0.25, "auto_exposure=manual(0.25)");
  }
  else
  {
    try_set(*cap_, cv::CAP_PROP_AUTO_EXPOSURE, 3.0, "auto_exposure=auto(3.0)");
    try_set(*cap_, cv::CAP_PROP_AUTO_EXPOSURE, 0.75, "auto_exposure=auto(0.75)");
  }

  // 曝光值：如果驱动使用“绝对曝光(微秒)”语义，可以直接设较大的正数；
  // 如果驱动使用“曝光等级/EV”，可能需要负数（如 -6 ~ -1）。
  try_set(*cap_, cv::CAP_PROP_EXPOSURE, static_cast<double>(runtime_.exposure_time),
          "exposure");

  // 增益
  try_set(*cap_, cv::CAP_PROP_GAIN, static_cast<double>(runtime_.gain), "gain");

  XR_LOG_INFO("Parameters updated: exposure=%.3f, gain=%.3f, fps=%d",
              runtime_.exposure_time, runtime_.gain, runtime_.fps);
}

}  // namespace uvc_camera