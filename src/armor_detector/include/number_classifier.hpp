#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <string>
#include <vector>

#include "armor.hpp"

class NumberClassifier
{
 public:
  NumberClassifier(const std::string& model_path, double threshold,
                   const std::initializer_list<ArmorNumber>& ignore_classes = {});

  void ExtractNumbers(const cv::Mat& src, std::vector<Armor>& armors);

  void Classify(std::vector<Armor>& armors);

  void SetThreshold(double threshold) { threshold_ = threshold; }

 private:
  cv::dnn::Net net_;
  std::initializer_list<ArmorNumber> ignore_classes_;
  double threshold_;
};

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
