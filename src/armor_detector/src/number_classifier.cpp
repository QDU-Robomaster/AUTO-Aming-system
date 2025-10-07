// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"

NumberClassifier::NumberClassifier(const std::string& model_path,
                                   const std::string& label_path, const double thre,
                                   const std::vector<std::string>& ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes)
{
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line))
  {
    class_names_.push_back(line);
  }
}

void NumberClassifier::extractNumbers(const cv::Mat& src, std::vector<Armor>& armors)
{
  // Light length in image
  const int light_length = 12;
  // Image size after warp（透视变换）
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);

  for (auto& armor : armors)
  {
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {armor.left_light.bottom, armor.left_light.top,
                                      armor.right_light.top, armor.right_light.bottom};

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width =
        armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix,
                        cv::Size(warp_width, warp_height));

    // Get ROI
    number_image =
        number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(
        number_image, number_image, 0, 255,
        cv::THRESH_BINARY | cv::THRESH_OTSU);  // cv::THRESH_OTSU自动计算最优阈值

    armor.number_img = number_image;
  }
}

void NumberClassifier::classify(std::vector<Armor>& armors)
{
  for (auto& armor : armors)
  {
    cv::Mat image = armor.number_img.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    net_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax
    float max_prob = *std::max_element(
        outputs.begin<float>(),
        outputs.end<
            float>());  // max_element函数返回的是一个迭代器，要获取实际值，需要解引用*
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    armor.number = class_names_[label_id];

    (void)snprintf(armor.classification_result.data(), armor.classification_result.size(),
                   "%s: %.1f%%", class_names_[label_id].c_str(), confidence * 100.0);
  }

  armors.erase(
      std::
          remove_if(  // remove_if会把向量中满足特定条件的元素移动到向量末尾，返回一个指向新的逻辑末尾的迭代器
              armors.begin(), armors.end(),
              [this](const Armor& armor)
              {
                if (armor.confidence < threshold)
                {
                  return true;
                }

                for (const auto& ignore_class : ignore_classes_)
                {
                  if (armor.number == ignore_class)
                  {
                    return true;
                  }
                }

                bool mismatch_armor_type = false;
                if (armor.type == ArmorType::LARGE)
                {
                  mismatch_armor_type = armor.number == "outpost" ||
                                        armor.number == "2" || armor.number == "guard";
                }
                else if (armor.type == ArmorType::SMALL)
                {
                  mismatch_armor_type = armor.number == "1" || armor.number == "base";
                }
                return mismatch_armor_type;
              }),
      armors.end());
}
