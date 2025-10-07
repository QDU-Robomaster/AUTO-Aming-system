#pragma once

/** @file armor_detector.hpp
 *  @brief 装甲板检测器（头文件）。
 *
 *  提供 ArmorDetector 的配置结构与对外接口声明；
 *  仅包含声明与轻量内联实现，不引入行为差异。
 */

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// STL
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// Project
#include "armor.hpp"
#include "camera_base.hpp"
#include "libxr.hpp"
#include "message.hpp"
#include "number_classifier.hpp"
#include "pnp_solver.hpp"

#ifndef MODEL_PATH
#define MODEL_PATH "."
#endif

/** @brief 装甲板检测器。
 *
 * 基于二值化与几何匹配从图像中提取“灯条”，组合为“装甲板”，
 * 并结合分类器识别装甲编号；在相机参数可用时，可调用 PnP 解算位姿并发布结果。
 */
class ArmorDetector
{
 public:
  /** @brief 灯条几何筛选参数。*/
  struct LightParams
  {
    double min_ratio{0.1};   ///< 宽高比下限 (width/height)
    double max_ratio{0.4};   ///< 宽高比上限
    double max_angle{40.0};  ///< 允许的最大倾角(度)
  };

  /** @brief 装甲匹配与姿态相关参数。*/
  struct ArmorParams
  {
    double min_light_ratio{0.7};            ///< 两灯条长度比下限 (短/长)
    double min_small_center_distance{0.8};  ///< 小装甲中心距下限(相对灯条长)
    double max_small_center_distance{3.2};  ///< 小装甲中心距上限
    double min_large_center_distance{3.2};  ///< 大装甲中心距下限
    double max_large_center_distance{5.5};  ///< 大装甲中心距上限
    double max_angle{35.0};                 ///< 两灯条连线与水平夹角上限(度)
  };

  /** @brief 数字分类器参数。*/
  struct ClassifierParams
  {
    std::initializer_list<ArmorNumber> ignore_classes{
        ArmorNumber::NEGATIVE};  ///< 忽略的类别标签
    double threshold{0.7};       ///< 置信度阈值
  };

  /** @brief 检测器整体配置。*/
  struct Config
  {
    ClassifierParams classifier{};  ///< 分类器参数
    int detect_color{1};            ///< 要检测的目标颜色 (RED/BLUE)
    int binary_thres{85};           ///< 二值化阈值
    LightParams light{};            ///< 灯条筛选参数
    ArmorParams armor{};            ///< 装甲匹配参数
  };

  /** @brief 使用配置构造检测器。
   *  @param cfg 初始化配置
   */
  explicit ArmorDetector(const Config& cfg);

  /** @brief 在单帧图像上执行检测。
   *  @param input RGB 图像 (CV_8UC3, 通道顺序假定为 R,G,B)
   *  @return 检出的装甲板列表（若启用分类器，包含编号信息）
   */
  std::vector<Armor> Detect(const cv::Mat& input);

  /** @brief 设置二值化阈值。 */
  void SetBinaryThres(int t) { binary_thres_ = t; }

  /** @brief 设置目标检测颜色（RED/BLUE）。 */
  void SetDetectColor(int c) { detect_color_ = c; }

  /** @brief 设置分类阈值并同步到已构造的分类器。
   *  @param t 新的阈值
   */
  void SetClassifierThreshold(double t)
  {
    classifier_threshold_ = t;
    if (classifier_)
    {
      classifier_->SetThreshold(t);
    }
  }

  /** @brief 以整体配置更新检测器（会重建分类器）。
   *  @param cfg 新配置
   */
  void SetConfig(const Config& cfg);

  /** @brief 获取当前配置快照。 */
  const Config& GetConfig() const { return cfg_cache_; }

 private:
  // —— 图像管线 ——
  /** @brief 预处理：RGB→灰度→二值化。
   *  @param rgb_img 输入 RGB 图像
   *  @return 二值图
   */
  cv::Mat PreprocessImage(const cv::Mat& rgb_img);

  /** @brief 从二值图轮廓中提取灯条并判定颜色。
   *  @param rgb_img 原始 RGB 图
   *  @param binary_img 二值图
   *  @return 通过几何与颜色判定的灯条列表
   */
  std::vector<Light> FindLights(const cv::Mat& rgb_img, const cv::Mat& binary_img);

  /** @brief 将灯条两两匹配为装甲候选。
   *  @param lights 灯条列表
   *  @return 通过几何约束的装甲列表
   */
  std::vector<Armor> MatchLights(const std::vector<Light>& lights);

  /** @brief 判断灯条是否满足几何约束。 */
  bool IsLight(const Light& possible_light);

  /** @brief 判断两灯条包围区域内是否包含其它灯条，避免误配。 */
  bool ContainLight(const Light& light_1, const Light& light_2,
                    const std::vector<Light>& lights);

  /** @brief 判断一对灯条组成的装甲类型（小/大/无效）。 */
  ArmorType IsArmor(const Light& light_1, const Light& light_2);

  // —— 回调与初始化 ——
  /** @brief 图像订阅回调：检测、解算 PnP 并发布结果。 */
  void ImageCallback(cv::Mat* img_msg);

  /** @brief 初始化/重建数字分类器。 */
  void InitClassifier();

 private:
  // 配置缓存（来源：SetConfig/构造）
  Config cfg_cache_{};  ///< 当前配置快照

  // 便捷缓存（与配置字段对应）
  int binary_thres_{85};              ///< 二值阈值
  int detect_color_{1};               ///< 目标颜色
  LightParams l_{};                   ///< 灯条参数缓存
  ArmorParams a_{};                   ///< 装甲参数缓存
  double classifier_threshold_{0.7};  ///< 分类置信度阈值
  std::initializer_list<ArmorNumber> ignore_classes_{
      ArmorNumber::NEGATIVE};  ///< 忽略类别

  // 组件
  std::unique_ptr<NumberClassifier> classifier_{};  ///< 数字分类器
  std::unique_ptr<PnPSolver> pnp_solver_{};         ///< PnP 解算器

  // 运行时缓存
  std::vector<Light> lights_{};  ///< 最新帧灯条
  std::vector<Armor> armors_{};  ///< 最新帧装甲
  cv::Mat binary_img_{};         ///< 最新帧二值图

  // 发布/订阅
  ArmorDetectorResults armors_msg_{};  ///< 发布消息缓存
  LibXR::Topic armors_topic_{
      LibXR::Topic("/detector/armors", sizeof(ArmorDetectorResults))};  ///< 发布 Topic

  // 相机信息
  cv::Point2f cam_center_{0.f, 0.f};                    ///< 成像中心（cx, cy）
  std::shared_ptr<CameraBase::CameraInfo> cam_info_{};  ///< 相机内参/畸变
};