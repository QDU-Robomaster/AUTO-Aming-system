#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// STL
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor.hpp"
#include "camera_base.hpp"
#include "detector.hpp"
#include "detector_node.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "transform.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(bool debug, int detect_color, int binary_thr,
                                     double light_min_ratio, double light_max_ratio,
                                     double light_max_angle, double armor_min_light_ratio,
                                     double armor_min_small_center_distance,
                                     double armor_max_small_center_distance,
                                     double armor_min_large_center_distance,
                                     double armor_max_large_center_distance,
                                     double armor_max_angle, double classifier_threshold,
                                     std::vector<std::string> ignore_classes)
    : binary_thres_(binary_thr),
      detect_color_(detect_color),
      light_min_ratio_(light_min_ratio),
      light_max_ratio_(light_max_ratio),
      light_max_angle_(light_max_angle),
      armor_min_light_ratio_(armor_min_light_ratio),
      armor_min_small_center_distance_(armor_min_small_center_distance),
      armor_max_small_center_distance_(armor_max_small_center_distance),
      armor_min_large_center_distance_(armor_min_large_center_distance),
      armor_max_large_center_distance_(armor_max_large_center_distance),
      armor_max_angle_(armor_max_angle),
      classifier_threshold_(classifier_threshold),
      ignore_classes_(ignore_classes)
{
  XR_LOG_INFO("Starting DetectorNode!");

  // Detector
  detector_ = initDetector();

  auto info_topic = LibXR::Topic(LibXR::Topic::Find("camera_info"));
  auto info_cb = LibXR::Topic::Callback::Create(
      [](bool, ArmorDetectorNode* detector_node, LibXR::RawData& data)
      {
        CameraBase::CameraInfo* camera_info =
            reinterpret_cast<CameraBase::CameraInfo*>(data.addr_);

        static bool inited = false;
        if (!inited)
        {
#ifdef XR_LOG_PASS
          XR_LOG_PASS("Got camera info!");
#endif
          inited = true;

          // cx=K(0,2)=index2, cy=K(1,2)=index5（行优先）
          detector_node->cam_center_ =
              cv::Point2f(static_cast<float>(camera_info->camera_matrix[2]),
                          static_cast<float>(camera_info->camera_matrix[5]));

          detector_node->cam_info_ =
              std::make_shared<CameraBase::CameraInfo>(*camera_info);

          // 新版：按模型提取合适长度的畸变参数
          std::vector<double> distortion_coefficients =
              CameraBase::CameraInfo::ToPnPDistCoeffs(
                  camera_info->distortion_model, camera_info->distortion_coefficients);

          // 创建 PnP 求解器
          detector_node->pnp_solver_ = std::make_unique<PnPSolver>(
              camera_info->camera_matrix, distortion_coefficients);

          // TODO: 若输入为“已矫正/立体矫正”图像，推荐：
          // 1) 使用 projection_matrix 的前 3×3 作为内参传入；
          // 2) 畸变向量置空（NONE）。
          // 以避免误把原始畸变用于矫正后的图像。
        }
      },
      this);

  info_topic.RegisterCallback(info_cb);

  auto img_topic = LibXR::Topic(LibXR::Topic::Find("image_raw"));
  auto img_cb = LibXR::Topic::Callback::Create(
      [](bool, ArmorDetectorNode* detector_node, LibXR::RawData& data)
      {
        XR_LOG_DEBUG("Got image!");
        cv::Mat* img_msg = reinterpret_cast<cv::Mat*>(data.addr_);
        detector_node->imageCallback(img_msg);
      },
      this);

  img_topic.RegisterCallback(img_cb);
}

void ArmorDetectorNode::imageCallback(cv::Mat* img_msg)
{
  auto armors = detectArmors(img_msg);

  if (pnp_solver_ != nullptr)
  {
    armors_msg_.clear();

    ArmorDetectorResult armor_msg;
    for (const auto& armor : armors)
    {
      cv::Mat rvec, tvec;
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
      if (success)
      {
        XR_LOG_DEBUG("Got armor pose!");
        // Fill basic info
        armor_msg.type = armor.type;
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.translation.x() = tvec.at<double>(0);
        armor_msg.pose.translation.y() = tvec.at<double>(1);
        armor_msg.pose.translation.z() = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        armor_msg.pose.rotation = LibXR::RotationMatrix<double>(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
            rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
            rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
            rotation_matrix.at<double>(2, 2));

        auto eulr = armor_msg.pose.rotation.ToEulerAngle();
        XR_LOG_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", eulr[0], eulr[1], eulr[2]);
        XR_LOG_DEBUG("x: %f, y: %f, z: %f", armor_msg.pose.translation.x(),
                     armor_msg.pose.translation.y(), armor_msg.pose.translation.z());

        // Fill the distance to image center
        armor_msg.distance_to_image_center =
            pnp_solver_->calculateDistanceToCenter(armor.center);
        armors_msg_.emplace_back(armor_msg);
      }
      else
      {
        XR_LOG_WARN("PnP failed!");
      }
    }

    // Publishing detected armors
    armors_topic_.Publish(armors_msg_);
  }
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  Detector::LightParams l_params = {.min_ratio = light_min_ratio_,
                                    .max_ratio = light_max_ratio_,
                                    .max_angle = light_max_angle_};

  Detector::ArmorParams a_params = {
      .min_light_ratio = armor_min_light_ratio_,
      .min_small_center_distance = armor_min_small_center_distance_,
      .max_small_center_distance = armor_max_small_center_distance_,
      .min_large_center_distance = armor_min_large_center_distance_,
      .max_large_center_distance = armor_max_large_center_distance_,
      .max_angle = armor_max_angle_};

  auto detector =
      std::make_unique<Detector>(binary_thres_, detect_color_, l_params, a_params);

  // Init classifier
  std::string pkg_path = MODEL_PATH;
  auto model_path = pkg_path + "/mlp.onnx";
  auto label_path = pkg_path + "/label.txt";
  double threshold = classifier_threshold_;
  std::vector<std::string> ignore_classes = ignore_classes_;
  detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path,
                                                            threshold, ignore_classes);

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(cv::Mat* img_msg)
{
  auto shared_img = std::make_shared<cv::Mat>(*img_msg);
  auto& img = *shared_img.get();

  // Update params
  detector_->binary_thres = binary_thres_;
  detector_->detect_color = detect_color_;
  detector_->classifier->threshold = classifier_threshold_;

  auto armors = detector_->detect(img);

  return armors;
}

}  // namespace rm_auto_aim
