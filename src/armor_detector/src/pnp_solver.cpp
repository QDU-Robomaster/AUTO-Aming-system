#include "pnp_solver.hpp"

#include <opencv2/calib3d.hpp>

PnPSolver::PnPSolver(std::array<double, 9>& camera_matrix,
                     std::array<double, 5>& dist_coeffs)
    : camera_matrix_(cv::Mat(3, 3, CV_64F, camera_matrix.data()).clone()),
      dist_coeffs_(cv::Mat(1, 5, CV_64F, dist_coeffs.data()).clone())
{
  // 将毫米转换为米，并构造模型点
  constexpr double SMALL_HALF_T = SMALL_ARMOR_WIDTH * 0.5 / 1000.0;
  constexpr double SMALL_HALF_Z = SMALL_ARMOR_HEIGHT * 0.5 / 1000.0;
  constexpr double LARGE_HALF_Y = LARGE_ARMOR_WIDTH * 0.5 / 1000.0;
  constexpr double LARGE_HALF_Z = LARGE_ARMOR_HEIGHT * 0.5 / 1000.0;

  // 点顺序：左下、左上、右上、右下（顺时针）
  // 模型坐标：x 前，y 左，z 上
  small_armor_points_ = {
      {0.0f, static_cast<float>(SMALL_HALF_T), static_cast<float>(-SMALL_HALF_Z)},
      {0.0f, static_cast<float>(SMALL_HALF_T), static_cast<float>(SMALL_HALF_Z)},
      {0.0f, static_cast<float>(-SMALL_HALF_T), static_cast<float>(SMALL_HALF_Z)},
      {0.0f, static_cast<float>(-SMALL_HALF_T), static_cast<float>(-SMALL_HALF_Z)}};

  large_armor_points_ = {
      {0.0f, static_cast<float>(LARGE_HALF_Y), static_cast<float>(-LARGE_HALF_Z)},
      {0.0f, static_cast<float>(LARGE_HALF_Y), static_cast<float>(LARGE_HALF_Z)},
      {0.0f, static_cast<float>(-LARGE_HALF_Y), static_cast<float>(LARGE_HALF_Z)},
      {0.0f, static_cast<float>(-LARGE_HALF_Y), static_cast<float>(-LARGE_HALF_Z)}};
}

bool PnPSolver::SolvePnP(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec)
{
  // 填充图像点，顺序与模型点一致：左下、左上、右上、右下
  std::vector<cv::Point2f> image_armor_points = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

  const auto& object_points =
      (armor.type == ArmorType::SMALL) ? small_armor_points_ : large_armor_points_;

  return cv::solvePnP(object_points, image_armor_points, camera_matrix_, dist_coeffs_,
                      rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

double PnPSolver::CalculateDistanceToCenter(const cv::Point2f& image_point)
{
  float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
  float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
  return cv::norm(image_point - cv::Point2f(cx, cy));
}
