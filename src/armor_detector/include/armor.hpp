#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <array>
#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "transform.hpp"

const int RED = 0;
const int BLUE = 1;

enum class ArmorType
{
  SMALL,
  LARGE,
  INVALID
};
inline const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Light : public cv::RotatedRect
{
  Light() = default;

  // Light继承自cv::RotatedRect，可以直接使用其成员函数
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    // 灯条四个点（按y从小到大排序）
    cv::Point2f p[4];
    box.points(p);  // RotatedRect::points 获取矩形四个顶点
    std::sort(p, p + 4,
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

    // 顶部与底部中心点
    top = (p[0] + p[1]) * 0.5f;
    bottom = (p[2] + p[3]) * 0.5f;

    // 长度 = 顶/底中心距离；宽度 = 顶部两点的距离
    length = static_cast<double>(cv::norm(top - bottom));
    width = static_cast<double>(cv::norm(p[0] - p[1]));

    // 与“竖直方向”的倾斜角（单位：度）
    // 这里用atan2(|Δx|, |Δy|)；结果转换为度
    tilt_angle = static_cast<float>(
        std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) * 180.0 /
        CV_PI);
  }

  int color = -1;
  cv::Point2f top{}, bottom{};
  double length = 0.0;
  double width = 0.0;
  float tilt_angle = 0.0f;  // degrees
};

struct Armor
{
  Armor() = default;

  Armor(const Light& l1, const Light& l2)
  {
    if (l1.center.x < l2.center.x)
    {
      left_light = l1;
      right_light = l2;
    }
    else
    {
      left_light = l2;
      right_light = l1;
    }
    center = (left_light.center + right_light.center) * 0.5f;
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center{};
  ArmorType type = ArmorType::INVALID;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence = 0.0f;
  std::array<char, 32> classification_result;  // 保持原拼写以兼容现有代码
};

struct ArmorDetectorResult
{
  std::string number;
  ArmorType type;
  float distance_to_image_center;
  LibXR::Transform<double> pose;
};

typedef std::vector<ArmorDetectorResult> ArmorDetectorResults;

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
