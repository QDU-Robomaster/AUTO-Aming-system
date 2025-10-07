#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// STD
#include <memory>
#include <string>
#include <vector>

#include "camera_base.hpp"
#include "detector.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "number_classifier.hpp"
#include "pnp_solver.hpp"

class ArmorDetectorNode
{
 public:
  ArmorDetectorNode(bool debug = true, int detect_color = 1, int binary_thres = 85,
                    double light_min_ratio = 0.1, double light_max_ratio = 0.4,
                    double light_max_angle = 40.0, double armor_min_light_ratio = 0.7,
                    double armor_min_small_center_distance = 0.8,
                    double armor_max_small_center_distance = 3.2,
                    double armor_min_large_center_distance = 3.2,
                    double armor_max_large_center_distance = 5.5,
                    double armor_max_angle = 35.0, double classifier_threshold = 0.7,
                    std::vector<std::string> ignore_classes = {"negative"});

 private:
  void imageCallback(cv::Mat* img_msg);

  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(cv::Mat* img_msg);

  // Armor Detector
  std::unique_ptr<Detector> detector_;

  int binary_thres_;
  int detect_color_;
  double light_min_ratio_;
  double light_max_ratio_;
  double light_max_angle_;
  double armor_min_light_ratio_;
  double armor_min_small_center_distance_;
  double armor_max_small_center_distance_;
  double armor_min_large_center_distance_;
  double armor_max_large_center_distance_;
  double armor_max_angle_;
  double classifier_threshold_;
  std::vector<std::string> ignore_classes_;

  // Detected armors publisher
  ArmorDetectorResults armors_msg_;
  LibXR::Topic armors_topic_ = LibXR::Topic("/detector/armors", sizeof(armors_msg_));

  // Camera info part
  cv::Point2f cam_center_;
  std::shared_ptr<CameraBase::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;
};

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
