#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"

namespace rm_auto_aim
{
class Detector
{
 public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle与垂直方向的最大差角
    double max_angle;
  };

  struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  Detector(const int& bin_thres, const int& color, const LightParams& l,
           const ArmorParams& a);

  std::vector<Armor> detect(const cv::Mat& input);

  cv::Mat preprocessImage(const cv::Mat& input);
  std::vector<Light> findLights(const cv::Mat& rbg_img, const cv::Mat& binary_img);
  std::vector<Armor> matchLights(const std::vector<Light>& lights);

  // For debug usage
  cv::Mat getAllNumbersImage();

  int binary_thres;
  int detect_color;
  LightParams l;
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;

  // Debug msgs
  cv::Mat binary_img;

 private:
  bool isLight(const Light& possible_light);
  bool containLight(const Light& light_1, const Light& light_2,
                    const std::vector<Light>& lights);
  ArmorType isArmor(const Light& light_1, const Light& light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
