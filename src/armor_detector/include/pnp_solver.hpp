#pragma once

#include <array>
#include <opencv2/core.hpp>
#include <vector>

#include "armor.hpp"

/**
 * @class PnPSolver
 * @brief 使用 SolvePnP 估计小装甲板/大装甲板的位姿。
 */
class PnPSolver
{
 public:
  /**
   * @brief 构造函数
   * @param camera_matrix 相机内参矩阵（3x3，行优先存储），格式为
   *        (fx, 0, cx, 0, fy, cy, 0, 0, 1)。
   * @param distortion_coefficients 相机畸变系数 (k1, k2, p1, p2, k3)。
   * @warning 装甲板的物理尺寸单位为毫米，构造时会转换为米。
   */
  explicit PnPSolver(std::array<double, 9>& camera_matrix,
                     std::array<double, 5>& distortion_coefficients);

  /**
   * @brief 估计装甲板的三维位姿。
   * @param armor 输入装甲板信息（由两条灯条推导出的四个角点）。
   * @param rvec 输出旋转向量 (Rodrigues)，从模型坐标到相机坐标。
   * @param tvec 输出平移向量 (米)，从模型原点到相机原点。
   * @return true 计算成功。
   * @return false 计算失败（如点退化）。
   *
   * @note 使用 `cv::SOLVEPNP_IPPE` 算法，适合平面目标。
   */
  [[nodiscard]] bool SolvePnP(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec);

  /**
   * @brief 计算图像点到图像中心（主点）的像素距离。
   * @param image_point 输入图像点（像素坐标）。
   * @return 到主点 (cx, cy) 的像素距离。
   */
  [[nodiscard]] double CalculateDistanceToCenter(const cv::Point2f& image_point);

 private:
  cv::Mat camera_matrix_;  ///< 相机内参矩阵 (3x3, CV_64F)
  cv::Mat dist_coeffs_;    ///< 相机畸变参数 (1x5, CV_64F)

  // 装甲板尺寸（毫米）
  inline static constexpr float SMALL_ARMOR_WIDTH = 135.f;
  inline static constexpr float SMALL_ARMOR_HEIGHT = 55.f;
  inline static constexpr float LARGE_ARMOR_WIDTH = 225.f;
  inline static constexpr float LARGE_ARMOR_HEIGHT = 55.f;

  // 装甲板四个角点（单位：米），按顺时针顺序排列
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};