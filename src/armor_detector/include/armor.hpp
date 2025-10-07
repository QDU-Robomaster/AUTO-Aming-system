#pragma once

/** @file armor.hpp
 *  @brief 装甲板相关基础类型与数据结构。
 */

#include <array>
#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <cmath>
#include <string_view>
#include <vector>

#include "transform.hpp"

/** @name 颜色标签
 *  用于灯条/装甲的目标颜色判定。
 */
///@{
const int RED = 0;   ///< 红色目标
const int BLUE = 1;  ///< 蓝色目标
///@}

/** @brief 装甲板类型。 */
enum class ArmorType : uint8_t
{
  SMALL,   ///< 小装甲
  LARGE,   ///< 大装甲
  INVALID  ///< 非法/不成立
};

/** @brief 装甲板编号。 */
enum class ArmorNumber : uint8_t
{
  INVALID = 0,
  ONE = 1,
  TWO = 2,
  THREE = 3,
  FOUR = 4,
  FIVE = 5,
  OUTPOST = 6,
  GUARD = 7,
  BASE = 8,
  NEGATIVE = 9,
};

/** @brief 装甲类型到字符串的映射（与 ArmorType 顺序一致）。 */
static constexpr std::string_view ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

/** @brief 灯条（继承自 cv::RotatedRect）。
 *
 *  根据最小外接旋转矩形的四点，计算灯条上下端点、长度/宽度与相对竖直方向的倾角。
 */
struct Light : public cv::RotatedRect
{
  Light() = default;

  /** @brief 由旋转矩形构造灯条。
   *  @param box OpenCV 旋转矩形（像素坐标）
   */
  explicit Light(cv::RotatedRect& box)
      : cv::RotatedRect(box),
        tilt_angle(static_cast<float>(
            std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) * 180.0 /
            CV_PI))
  {
    // 旋转矩形四点（按 y 从小到大排序）
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4,
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

    // 顶部与底部中心点
    top = (p[0] + p[1]) * 0.5f;
    bottom = (p[2] + p[3]) * 0.5f;

    // 几何量
    length = static_cast<double>(cv::norm(top - bottom));
    width = static_cast<double>(cv::norm(p[0] - p[1]));
  }

  int color = -1;               ///< 颜色标签（RED/BLUE），-1 表示未知
  cv::Point2f top{}, bottom{};  ///< 顶/底中心
  double length = 0.0;          ///< 灯条长度（像素）
  double width = 0.0;           ///< 灯条宽度（像素）
  float tilt_angle = 0.0f;      ///< 倾角（度）
};

/** @brief 装甲板（由两根灯条组成）。 */
struct Armor
{
  Armor() = default;

  /** @brief 由两根灯条构造装甲（自动按左右排序）。
   *  @param l1 灯条 1
   *  @param l2 灯条 2
   */
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
  Light left_light, right_light;        ///< 左/右灯条
  cv::Point2f center{};                 ///< 装甲中心
  ArmorType type = ArmorType::INVALID;  ///< 估计装甲类型

  // Number part
  cv::Mat number_img;       ///< 识别裁剪图
  ArmorNumber number;       ///< 识别结果字符串
  float confidence = 0.0f;  ///< 识别置信度
};

/** @brief 单个装甲的发布结果。 */
struct ArmorDetectorResult
{
  ArmorNumber number;              ///< 编号字符串
  ArmorType type;                  ///< 装甲类型
  float distance_to_image_center;  ///< 中心点到图像中心的距离
  LibXR::Transform<double> pose;   ///< 相机坐标系下的位姿
};

/** @brief 装甲发布结果数组类型别名。 */
using ArmorDetectorResults = std::vector<ArmorDetectorResult>;