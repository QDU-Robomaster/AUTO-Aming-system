// OpenCV
#include <initializer_list>
#include <opencv2/opencv.hpp>

// STL
#include <math.h>

#include <algorithm>
#include <string>
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"

NumberClassifier::NumberClassifier(
    const std::string& model_path, double thre,
    const std::initializer_list<ArmorNumber>& ignore_classes)
    : ignore_classes_(ignore_classes), threshold_(thre)
{
  net_ = cv::dnn::readNetFromONNX(model_path);
}

void NumberClassifier::ExtractNumbers(const cv::Mat& src, std::vector<Armor>& armors)
{
  // Light length in image
  const int LIGHT_LENGTH = 12;
  // Image size after warp（透视变换）
  const int WARP_HEIGHT = 28;
  const int SMALL_ARMOR_WIDTH = 32;
  const int LARGE_ARMOR_WIDTH = 54;
  // Number ROI size
  const cv::Size ROI_SIZE(20, 28);

  for (auto& armor : armors)
  {
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {armor.left_light.bottom, armor.left_light.top,
                                      armor.right_light.top, armor.right_light.bottom};

    const int TOP_LIGHT_Y = (WARP_HEIGHT - LIGHT_LENGTH) / 2 - 1;
    const int BOTTOM_LIGHT_Y = TOP_LIGHT_Y + LIGHT_LENGTH;
    const int WARP_WIDTH =
        armor.type == ArmorType::SMALL ? SMALL_ARMOR_WIDTH : LARGE_ARMOR_WIDTH;
    cv::Point2f target_vertices[4] = {
        cv::Point(0, BOTTOM_LIGHT_Y),
        cv::Point(0, TOP_LIGHT_Y),
        cv::Point(WARP_WIDTH - 1, TOP_LIGHT_Y),
        cv::Point(WARP_WIDTH - 1, BOTTOM_LIGHT_Y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix,
                        cv::Size(WARP_WIDTH, WARP_HEIGHT));

    // Get ROI
    number_image =
        number_image(cv::Rect(cv::Point((WARP_WIDTH - ROI_SIZE.width) / 2, 0), ROI_SIZE));

    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(
        number_image, number_image, 0, 255,
        cv::THRESH_BINARY | cv::THRESH_OTSU);  // cv::THRESH_OTSU自动计算最优阈值

    armor.number_img = number_image;
  }
}

void NumberClassifier::Classify(std::vector<Armor>& armors)
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

    double confidence = 0;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = static_cast<float>(confidence);
    armor.number = static_cast<typeof(armor.number)>(label_id);
  }

  armors.erase(
      std::
          remove_if(  // remove_if会把向量中满足特定条件的元素移动到向量末尾，返回一个指向新的逻辑末尾的迭代器
              armors.begin(), armors.end(),
              [this](const Armor& armor)
              {
                if (armor.confidence < threshold_)
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
                  mismatch_armor_type = armor.number == ArmorNumber::OUTPOST ||
                                        armor.number == ArmorNumber::TWO ||
                                        armor.number == ArmorNumber::GUARD;
                }
                else if (armor.type == ArmorType::SMALL)
                {
                  mismatch_armor_type = armor.number == ArmorNumber::ONE ||
                                        armor.number == ArmorNumber::BASE;
                }
                return mismatch_armor_type;
              }),
      armors.end());
}
