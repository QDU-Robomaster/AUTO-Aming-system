#include "uvc_camera.hpp"

#include <cstddef>
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sstream>

#include "logger.hpp"
#include "thread.hpp"

// ---- Helpers ---------------------------------------------------------------
static void try_set(cv::VideoCapture& cap, int prop, double value, const char* name)
{
  bool ok = cap.set(prop, value);
  XR_LOG_INFO("Set %s = %.3f (%s)", name, value, ok ? "ok" : "no");
}

// Try opening a camera index with a backend and grabbing a frame
static bool try_open_once(int idx, int backend)
{
  cv::VideoCapture t(idx, backend);
  if (!t.isOpened())
  {
    return false;
  }
  t.set(cv::CAP_PROP_BUFFERSIZE, 1);
  cv::Mat tmp;
  bool ok = false;
  for (int i = 0; i < 5; ++i)
  {
    if (t.read(tmp) && !tmp.empty())
    {
      ok = true;
      break;
    }
    LibXR::Thread::Sleep(100);
  }
  t.release();
  if (ok)
  {
    XR_LOG_INFO("Candidate camera index %d works on backend %d", idx, backend);
  }
  return ok;
}

// ---- Public ----------------------------------------------------------------
UvcCamera::UvcCamera() : UvcCamera(CameraBase::CameraInfo{}, RuntimeParam{}) {}

UvcCamera::UvcCamera(const CameraBase::CameraInfo& info, const RuntimeParam& runtime)
    : info_(info), runtime_(runtime)
{
  XR_LOG_INFO("Starting UvcCamera (OpenCV UVC)!");

  // 1) Enumerate & choose device
  auto ids = EnumerateCameras(10);
  XR_LOG_INFO("Found camera count = %d", static_cast<int>(ids.size()));

  while (ids.empty())
  {
    XR_LOG_ERROR("No camera found!");
    LibXR::Thread::Sleep(100);
    ids = EnumerateCameras(10);
  }

  device_index_ = ids.front();
  XR_LOG_PASS("UVC camera selected: index=%d", device_index_);

  // 2) Open device
  if (!OpenCamera())
  {
    XR_LOG_ERROR("Failed to open camera index %d", device_index_);
    throw std::runtime_error("UvcCamera: cannot open camera");
  }

  // 3) Preallocate max buffer
  frame_buf_ = std::make_unique<std::array<uint8_t, BUF_BYTES>>();

  // 4) Apply parameters
  UpdateParameters();

  // 5) Start capture thread
  running_.store(true);
  capture_thread_.Create(this, ThreadFun, "UvcCameraThread",
                         static_cast<size_t>(1024 * 128),
                         LibXR::Thread::Priority::REALTIME);
}

UvcCamera::~UvcCamera()
{
  // Signal exit & release device
  running_.store(false);

  if (cap_)
  {
    cap_->release();
    cap_.reset();
  }

  XR_LOG_INFO("UvcCamera destroyed!");
}

void UvcCamera::SetRuntimeParam(const RuntimeParam& p)
{
  runtime_ = p;
  UpdateParameters();
}

// ---- Private ---------------------------------------------------------------
std::vector<int> UvcCamera::EnumerateCameras(int max_test)
{
  std::vector<int> ids;
  for (int i = 0; i < max_test; ++i)
  {
    if (try_open_once(i, cv::CAP_V4L2) || try_open_once(i, cv::CAP_ANY))
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

bool UvcCamera::OpenCamera()
{
  auto open_with = [&](int backend, int fourcc, const char* tag) -> bool
  {
    cap_ = std::make_unique<cv::VideoCapture>(device_index_, backend);
    if (!cap_ || !cap_->isOpened())
    {
      return false;
    }

    if (fourcc)
    {
      cap_->set(cv::CAP_PROP_FOURCC, fourcc);
    }

    // Use desired WxH from info_ if provided (else default 1280x720)
    const int REG_W = info_.width > 0 ? static_cast<int>(info_.width) : 1280;
    const int REG_H = info_.height > 0 ? static_cast<int>(info_.height) : 720;

    if (REG_W > 0)
    {
      cap_->set(cv::CAP_PROP_FRAME_WIDTH, REG_W);
    }
    if (REG_H > 0)
    {
      cap_->set(cv::CAP_PROP_FRAME_HEIGHT, REG_H);
    }
    if (runtime_.fps > 0)
    {
      cap_->set(cv::CAP_PROP_FPS, runtime_.fps);
    }

    cap_->set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Warmup frames
    cv::Mat fr;
    bool ok = false;
    for (int t = 0; t < 10; ++t)
    {
      if (cap_->read(fr) && !fr.empty())
      {
        ok = true;
        break;
      }
      LibXR::Thread::Sleep(100);
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

  int mjpg = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  int yuyv = cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');  // sometimes named YUY2

  if (open_with(cv::CAP_V4L2, mjpg, "V4L2+MJPG"))
  {
    return true;
  }
  if (open_with(cv::CAP_V4L2, yuyv, "V4L2+YUYV"))
  {
    return true;
  }
  if (open_with(cv::CAP_V4L2, 0, "V4L2(no FOURCC)"))
  {
    return true;
  }
  if (open_with(cv::CAP_ANY, mjpg, "ANY+MJPG"))
  {
    return true;
  }
  if (open_with(cv::CAP_ANY, 0, "ANY(no FOURCC)"))
  {
    return true;
  }

  // Optional: GStreamer pipeline fallback (if OpenCV built with GStreamer)
  {
    std::string dev = "/dev/video" + std::to_string(device_index_);
    std::ostringstream pipe;
    int w = info_.width > 0 ? static_cast<int>(info_.width) : 1280;
    int h = info_.height > 0 ? static_cast<int>(info_.height) : 720;
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

void UvcCamera::UpdateParameters()
{
  if (!cap_ || !cap_->isOpened())
  {
    return;
  }

  // Resolution/FPS (again, to ensure they stick)
  if (info_.width > 0)
  {
    try_set(*cap_, cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(info_.width), "width");
  }
  if (info_.height > 0)
  {
    try_set(*cap_, cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(info_.height),
            "height");
  }
  if (runtime_.fps > 0)
  {
    try_set(*cap_, cv::CAP_PROP_FPS, runtime_.fps, "fps");
  }

  // Exposure control: best effort across drivers
  if (runtime_.manual_exposure)
  {
    try_set(*cap_, cv::CAP_PROP_AUTO_EXPOSURE, 1.0, "auto_exposure=manual(1.0)");
    try_set(*cap_, cv::CAP_PROP_AUTO_EXPOSURE, 0.25, "auto_exposure=manual(0.25)");
  }
  else
  {
    try_set(*cap_, cv::CAP_PROP_AUTO_EXPOSURE, 3.0, "auto_exposure=auto(3.0)");
    try_set(*cap_, cv::CAP_PROP_AUTO_EXPOSURE, 0.75, "auto_exposure=auto(0.75)");
  }

  try_set(*cap_, cv::CAP_PROP_EXPOSURE, static_cast<double>(runtime_.exposure_time),
          "exposure");

  // Gain
  try_set(*cap_, cv::CAP_PROP_GAIN, static_cast<double>(runtime_.gain), "gain");

  XR_LOG_INFO("Parameters updated: exposure=%.3f, gain=%.3f, fps=%d",
              runtime_.exposure_time, runtime_.gain, runtime_.fps);
}

void UvcCamera::ThreadFun(UvcCamera* self)
{
  XR_LOG_INFO("Publishing image!");

  while (self->running_.load())
  {
    cv::Mat frame_bgr;
    if (self->cap_ && self->cap_->isOpened() && self->cap_->read(frame_bgr) &&
        !frame_bgr.empty())
    {
      const int W = frame_bgr.cols;
      const int H = frame_bgr.rows;

      // Prepare target RGB view on preallocated buffer
      cv::Mat img_rgb(H, W, CV_8UC3, self->frame_buf_->data(),
                      static_cast<size_t>(W) * CH);
      cv::cvtColor(frame_bgr, img_rgb, cv::COLOR_BGR2RGB);

      self->info_.width = static_cast<uint32_t>(W);
      self->info_.height = static_cast<uint32_t>(H);
      self->info_.step = static_cast<uint32_t>(W * CH);
      self->info_.timestamp = LibXR::Timebase::GetMicroseconds();

      self->frame_topic_.Publish(img_rgb);
      self->info_topic_.Publish(self->info_);

      self->fail_count_ = 0;
    }
    else
    {
      XR_LOG_WARN("Grab frame failed! Try reopen.");
      self->fail_count_++;

      // Try reopen device
      if (self->cap_)
      {
        self->cap_->release();
      }
      LibXR::Thread::Sleep(100);
      self->OpenCamera();
    }

    if (self->fail_count_ > 5)
    {
      XR_LOG_ERROR("Camera failed repeatedly (%d)!", self->fail_count_);
    }
  }
}
