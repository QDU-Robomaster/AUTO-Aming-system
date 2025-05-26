// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace rm_auto_aim
{
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), detect_color(color), l(l), a(a)
{
}

std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img);
  armors_ = matchLights(lights_);

  if (!armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img) //图像预处理
{
  cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;  //// 定义一个向量，用于存储图像中检测到的所有轮廓
  vector<cv::Vec4i> hierarchy;  // 定义一个向量，用于存储图像中检测到的所有轮廓的层级信息
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);//仅检索最外层轮廓（忽略嵌套轮廓），仅保留水平、垂直和对角方向的端点（矩形仅需4个点）

  vector<Light> lights;
  this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;//跳过轮廓点数太少的，避免误判，减小干扰

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto rect = light.boundingRect();
      if (  // Avoid assertion failed 确保矩形区域完全在图像范围内，避免越界访问
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {//函数用于判断一个点是否在给定的轮廓（多边形）内(>0)，或者在轮廓上(=0)，亦或是在轮廓外(<0)。
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ? 判断红蓝灯条
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool Detector::isLight(const Light & light)
{
  // The ratio of light (short side / long side) 通关宽高比判断是否是灯条
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  // 1. 创建包围盒：用两个灯条的顶端和底端点构建最小外接矩形
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points); // 生成整数坐标的矩形

  // 2. 遍历所有灯条进行碰撞检测
  for (const auto & test_light : lights) {
    // 跳过当前正在配对的两个灯条（通过中心点坐标比较）
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    // 3. 检查其他灯条的关键点是否在包围盒内
    if (
      bounding_rect.contains(test_light.top) ||    // 顶点在区域内
      bounding_rect.contains(test_light.bottom) || // 底点在区域内
      bounding_rect.contains(test_light.center)) { // 中心点在区域内
      return true; // 发现干扰灯条立即返回
    }
  }

  return false; // 遍历完成未发现干扰灯条
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side) 灯条长度比例检查
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                            : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length) 灯条中心距离检查
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                            center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                            center_distance < a.max_large_center_distance);

  // Angle of light center connection  灯条中心连线角度检查
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

cv::Mat Detector::getAllNumbersImage()//将检测到的所有装甲板上的数字图像垂直拼接成一个单独的图像并返回
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::drawResults(cv::Mat & img)
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = (light.color == RED) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : armors_) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace rm_auto_aim
