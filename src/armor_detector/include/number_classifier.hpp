#pragma once

/**
 * @file number_classifier.hpp
 * @brief 数字装甲板分类器（头文件）。提供从装甲 ROI 提取数字图像并使用 ONNX
 * 模型分类的接口。
 *
 * 典型流程：
 * 1. 使用 ExtractNumbers() 根据灯条透视展开装甲区域，并裁剪出数字 ROI。
 * 2. 使用 Classify() 调用 DNN 模型进行前向推理，写回 Armor 的 number 与 confidence。
 */

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <initializer_list>
#include <string>
#include <vector>

#include "armor.hpp"  // 定义 Armor、ArmorType、ArmorNumber

/**
 * @class NumberClassifier
 * @brief 数字分类器：对装甲板中的数字进行识别与筛选。
 */
class NumberClassifier
{
 public:
  /**
   * @brief 构造函数
   * @param model_path ONNX 模型文件路径
   * @param threshold  置信度阈值（低于此阈值的结果会被丢弃）
   * @param ignore_classes 可选；需要忽略的类别列表（例如：哨兵、基地等按需过滤）
   */
  NumberClassifier(const std::string& model_path, double threshold,
                   const std::initializer_list<ArmorNumber>& ignore_classes = {});

  /**
   * @brief 从原图中为每个装甲提取数字 ROI（透视展开 + 二值化）
   * @param src    输入图（BGR）
   * @param armors 装甲集合，函数会为每个 Armor 写入 number_img 成员
   */
  void ExtractNumbers(const cv::Mat& src, std::vector<Armor>& armors);

  /**
   * @brief 对已提取的数字图进行分类，并基于阈值/忽略类别/大小匹配进行过滤
   * @param armors 输入/输出的装甲集合；函数会写回 Armor::number 与
   * Armor::confidence，并在不满足条件时从容器中移除
   */
  void Classify(std::vector<Armor>& armors);

  /**
   * @brief 设置置信度阈值
   */
  void SetThreshold(double threshold) { threshold_ = threshold; }

 private:
  cv::dnn::Net net_;  ///< DNN 模型
  std::initializer_list<ArmorNumber>
      ignore_classes_;      ///< 需要忽略的类别（安全起见使用 vector 持有）
  double threshold_ = 0.0;  ///< 置信度阈值
};
