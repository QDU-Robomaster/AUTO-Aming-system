#include "armor_detector.hpp"

#include "logger.hpp"

ArmorDetector::ArmorDetector(const Config& cfg)
    : cfg_cache_(cfg),
      binary_thres_(cfg.binary_thres),
      detect_color_(cfg.detect_color),
      l_(cfg.light),
      a_(cfg.armor),
      classifier_threshold_(cfg.classifier.threshold),
      ignore_classes_(cfg.classifier.ignore_classes)
{
  XR_LOG_INFO("Starting ArmorDetector (with Config).");

  // 初始化分类器
  InitClassifier();

  // 订阅 camera_info
  auto info_topic = LibXR::Topic(LibXR::Topic::Find("camera_info"));
  auto info_cb = LibXR::Topic::Callback::Create(
      [](bool, ArmorDetector* self, LibXR::RawData& data)
      {
        auto* camera_info = reinterpret_cast<CameraBase::CameraInfo*>(data.addr_);
        static bool inited = false;
        if (!inited)
        {
#ifdef XR_LOG_PASS
          XR_LOG_PASS("Got camera info!");
#endif
          inited = true;

          self->cam_center_ =
              cv::Point2f(static_cast<float>(camera_info->camera_matrix[2]),
                          static_cast<float>(camera_info->camera_matrix[5]));
          self->cam_info_ = std::make_shared<CameraBase::CameraInfo>(*camera_info);

          ASSERT(camera_info->distortion_model == CameraBase::DistortionModel::PLUMB_BOB);

          auto dist =
              std::array<double, 5>{camera_info->distortion_coefficients.plumb_bob.k1,
                                    camera_info->distortion_coefficients.plumb_bob.k2,
                                    camera_info->distortion_coefficients.plumb_bob.p1,
                                    camera_info->distortion_coefficients.plumb_bob.p2,
                                    camera_info->distortion_coefficients.plumb_bob.k3};

          self->pnp_solver_ =
              std::make_unique<PnPSolver>(camera_info->camera_matrix, dist);
        }
      },
      this);
  info_topic.RegisterCallback(info_cb);

  // 订阅 image_raw
  auto img_topic = LibXR::Topic(LibXR::Topic::Find("image_raw"));
  auto img_cb = LibXR::Topic::Callback::Create(
      [](bool, ArmorDetector* self, LibXR::RawData& data)
      {
        XR_LOG_DEBUG("Got image!");
        auto* img_msg = reinterpret_cast<cv::Mat*>(data.addr_);
        self->ImageCallback(img_msg);
      },
      this);
  img_topic.RegisterCallback(img_cb);
}

void ArmorDetector::SetConfig(const Config& cfg)
{
  cfg_cache_ = cfg;

  binary_thres_ = cfg.binary_thres;
  detect_color_ = cfg.detect_color;
  l_ = cfg.light;
  a_ = cfg.armor;
  classifier_threshold_ = cfg.classifier.threshold;
  ignore_classes_ = cfg.classifier.ignore_classes;

  classifier_.reset();
  InitClassifier();
}

void ArmorDetector::InitClassifier()
{
  classifier_ = std::make_unique<NumberClassifier>(MODEL_PATH, classifier_threshold_,
                                                   ignore_classes_);
}

cv::Mat ArmorDetector::PreprocessImage(const cv::Mat& rgb_img)
{
  cv::Mat gray;
  cv::cvtColor(rgb_img, gray, cv::COLOR_RGB2GRAY);
  cv::threshold(gray, gray, binary_thres_, 255, cv::THRESH_BINARY);
  return gray;
}

std::vector<Light> ArmorDetector::FindLights(const cv::Mat& rgb_img,
                                             const cv::Mat& binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  lights.reserve(contours.size());

  for (size_t i = 0; i < contours.size(); ++i)
  {
    const auto& contour = contours[i];
    if (contour.size() < 5)
    {
      continue;
    }

    auto r_rect = cv::minAreaRect(contour);
    Light light(r_rect);

    if (!IsLight(light))
    {
      continue;
    }

    const auto RECT = light.boundingRect();
    if (!(0 <= RECT.x && 0 <= RECT.width && RECT.x + RECT.width <= rgb_img.cols &&
          0 <= RECT.y && 0 <= RECT.height && RECT.y + RECT.height <= rgb_img.rows))
    {
      continue;
    }

    const cv::Mat ROI = rgb_img(RECT);

    cv::Mat mask(RECT.size(), CV_8UC1, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> one{contour};
    // 将整图坐标的轮廓偏移到 ROI 坐标系
    cv::drawContours(mask, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED,
                     cv::LINE_8, cv::noArray(), INT_MAX, cv::Point(-RECT.x, -RECT.y));

    const cv::Scalar MEAN_RGB = cv::mean(ROI, mask);
    const double MEAN_R = MEAN_RGB[0];
    const double MEAN_B = MEAN_RGB[2];
    light.color = (MEAN_R > MEAN_B) ? RED : BLUE;

    lights.emplace_back(std::move(light));
  }

  return lights;
}

bool ArmorDetector::IsLight(const Light& light)
{
  const double RATIO = light.width / LibXR::max(light.length, 1e-6f);
  const bool RATIO_OK = (l_.min_ratio < RATIO && RATIO < l_.max_ratio);

  const bool ANGLE_OK = (light.tilt_angle < l_.max_angle);

  return RATIO_OK && ANGLE_OK;
}

std::vector<Armor> ArmorDetector::MatchLights(const std::vector<Light>& lights)
{
  std::vector<Armor> armors;
  armors.reserve(lights.size());

  for (auto light_1 = lights.begin(); light_1 != lights.end(); ++light_1)
  {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); ++light_2)
    {
      if (light_1->color != detect_color_ || light_2->color != detect_color_)
      {
        continue;
      }

      if (ContainLight(*light_1, *light_2, lights))
      {
        continue;
      }

      const auto TYPE = IsArmor(*light_1, *light_2);
      if (TYPE != ArmorType::INVALID)
      {
        XR_LOG_DEBUG("Found armor type %d", static_cast<int>(TYPE));
        Armor armor(*light_1, *light_2);
        armor.type = TYPE;
        armors.emplace_back(std::move(armor));
      }
    }
  }
  return armors;
}

bool ArmorDetector::ContainLight(const Light& light_1, const Light& light_2,
                                 const std::vector<Light>& lights)
{
  std::vector<cv::Point2f> points{light_1.top, light_1.bottom, light_2.top,
                                  light_2.bottom};
  const auto BOUNDING_RECT = cv::boundingRect(points);

  for (const auto& test_light : lights)
  {
    if (test_light.center == light_1.center || test_light.center == light_2.center)
    {
      continue;
    }

    if (BOUNDING_RECT.contains(test_light.top) ||
        BOUNDING_RECT.contains(test_light.bottom) ||
        BOUNDING_RECT.contains(test_light.center))
    {
      return true;
    }
  }
  return false;
}

ArmorType ArmorDetector::IsArmor(const Light& light_1, const Light& light_2)
{
  const double LEN1 = light_1.length, LEN2 = light_2.length;
  const double LIGHT_LENGTH_RATION =
      (LEN1 < LEN2) ? (LEN1 / LibXR::max(LEN2, 1e-6f)) : (LEN2 / LibXR::max(LEN1, 1e-6f));
  const bool LIGHT_RATIO_OK = LIGHT_LENGTH_RATION > a_.min_light_ratio;

  const double AVG_LIGHT_LENGTH = (LEN1 + LEN2) / 2.0;
  const double CENTER_DISTANCE =
      cv::norm(light_1.center - light_2.center) / LibXR::max(AVG_LIGHT_LENGTH, 1e-6f);
  const bool CENTER_DISTANCE_OK = (a_.min_small_center_distance <= CENTER_DISTANCE &&
                                   CENTER_DISTANCE < a_.max_small_center_distance) ||
                                  (a_.min_large_center_distance <= CENTER_DISTANCE &&
                                   CENTER_DISTANCE < a_.max_large_center_distance);

  const cv::Point2f DIFF = light_1.center - light_2.center;
  const float ANGLE =
      std::abs(std::atan2(DIFF.y, DIFF.x)) * 180.0f / static_cast<float>(CV_PI);
  const bool ANGLE_OK = ANGLE < a_.max_angle;

  const bool IS_ARMOR = LIGHT_RATIO_OK && CENTER_DISTANCE_OK && ANGLE_OK;
  if (!IS_ARMOR)
  {
    return ArmorType::INVALID;
  }

  return (CENTER_DISTANCE > a_.min_large_center_distance) ? ArmorType::LARGE
                                                          : ArmorType::SMALL;
}

std::vector<Armor> ArmorDetector::Detect(const cv::Mat& input)
{
  binary_img_ = PreprocessImage(input);

  cv::imshow("binary_img", binary_img_);
  cv::waitKey(1);
  lights_ = FindLights(input, binary_img_);
  armors_ = MatchLights(lights_);

  if (!lights_.empty())
  {
    XR_LOG_DEBUG("Found %d lights", static_cast<int>(lights_.size()));
  }
  if (!armors_.empty())
  {
    XR_LOG_DEBUG("Found %d armors", static_cast<int>(armors_.size()));
  }

  if (!armors_.empty() && classifier_)
  {
    classifier_->ExtractNumbers(input, armors_);
    classifier_->Classify(armors_);
  }

  return armors_;
}

// =========== 回调 / 发布 ===========

void ArmorDetector::ImageCallback(cv::Mat* img_msg)
{
  const auto ARMORS = Detect(*img_msg);

  if (pnp_solver_ == nullptr)
  {
    return;
  }

  armors_msg_.clear();

  for (const auto& armor : ARMORS)
  {
    cv::Mat rvec, tvec;
    const bool SUCCESS = pnp_solver_->SolvePnP(armor, rvec, tvec);
    if (!SUCCESS)
    {
      XR_LOG_WARN("PnP failed!");
      continue;
    }

    XR_LOG_DEBUG("Got armor pose!");

    ArmorDetectorResult armor_msg;
    armor_msg.type = armor.type;
    armor_msg.number = armor.number;

    armor_msg.pose.translation.x() = tvec.at<double>(0);
    armor_msg.pose.translation.y() = tvec.at<double>(1);
    armor_msg.pose.translation.z() = tvec.at<double>(2);

    cv::Mat r;
    cv::Rodrigues(rvec, r);
    armor_msg.pose.rotation = LibXR::RotationMatrix<double>(
        r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2), r.at<double>(1, 0),
        r.at<double>(1, 1), r.at<double>(1, 2), r.at<double>(2, 0), r.at<double>(2, 1),
        r.at<double>(2, 2));

    const auto EULR = armor_msg.pose.rotation.ToEulerAngle();
    XR_LOG_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", EULR[0], EULR[1], EULR[2]);
    XR_LOG_DEBUG("x: %f, y: %f, z: %f", armor_msg.pose.translation.x(),
                 armor_msg.pose.translation.y(), armor_msg.pose.translation.z());

    armor_msg.distance_to_image_center =
        pnp_solver_->CalculateDistanceToCenter(armor.center);

    armors_msg_.emplace_back(std::move(armor_msg));
  }

  // 发布
  armors_topic_.Publish(armors_msg_);
}
