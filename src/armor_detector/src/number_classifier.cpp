// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <string>
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"

namespace
{
// 透视展开后的图像与 ROI 参数
constexpr int K_LIGHT_LENGTH = 12;  // 图中灯条长度（像素）
constexpr int K_WARP_HEIGHT = 28;   // 透视展开后的高度
constexpr int K_SMALL_WIDTH = 32;   // 小装甲展开宽度
constexpr int K_LARGE_WIDTH = 54;   // 大装甲展开宽度
const cv::Size K_ROI_SIZE(20, 28);  // 数字 ROI 大小
}  // namespace

NumberClassifier::NumberClassifier(
    const std::string& model_path, double threshold,
    const std::initializer_list<ArmorNumber>& ignore_classes)
    : ignore_classes_(ignore_classes), threshold_(threshold)
{
  // 读取 ONNX 模型
  net_ = cv::dnn::readNetFromONNX(model_path);
}

void NumberClassifier::ExtractNumbers(const cv::Mat& src, std::vector<Armor>& armors)
{
  for (auto& armor : armors)
  {
    // 以灯条四个顶点构建源四边形（左下、左上、右上、右下）
    cv::Point2f src_vertices[4] = {armor.left_light.bottom, armor.left_light.top,
                                   armor.right_light.top, armor.right_light.bottom};

    // 目标四边形（展开后），将灯条高度放在中间，便于截取数字区域
    const int TOP_Y = (K_WARP_HEIGHT - K_LIGHT_LENGTH) / 2 - 1;
    const int BOTTOM_Y = TOP_Y + K_LIGHT_LENGTH;
    const int WARP_W = (armor.type == ArmorType::SMALL) ? K_SMALL_WIDTH : K_LARGE_WIDTH;

    cv::Point2f dst_vertices[4] = {
        cv::Point2f(0.f, static_cast<float>(BOTTOM_Y)),
        cv::Point2f(0.f, static_cast<float>(TOP_Y)),
        cv::Point2f(static_cast<float>(WARP_W - 1), static_cast<float>(TOP_Y)),
        cv::Point2f(static_cast<float>(WARP_W - 1), static_cast<float>(BOTTOM_Y)),
    };

    // 透视变换
    cv::Mat number_image;
    const cv::Mat M = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::warpPerspective(src, number_image, M, cv::Size(WARP_W, K_WARP_HEIGHT));

    // 截取位于中间的数字 ROI
    const int X = (WARP_W - K_ROI_SIZE.width) / 2;
    const int Y = 0;
    number_image = number_image(cv::Rect(cv::Point(X, Y), K_ROI_SIZE)).clone();

    // 灰度 + Otsu 二值化（输入通常为 BGR）
    cv::cvtColor(number_image, number_image, cv::COLOR_BGR2GRAY);
    cv::threshold(number_image, number_image, 0, 255,
                  cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = number_image;  // 挂到 Armor 上，供后续分类
  }
}

void NumberClassifier::Classify(std::vector<Armor>& armors)
{
  for (auto& armor : armors)
  {
    cv::Mat image = armor.number_img;  // 单通道 8U

    // 归一化到 [0,1]
    image.convertTo(image, CV_32F, 1.0 / 255.0);

    // 构造输入 blob（自动添加 batch 维度与通道维度）
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // 前向推理
    net_.setInput(blob);
    cv::Mat logits = net_.forward();  // 形状一般为 (1, num_classes)

    // Softmax 计算
    const float MAX_LOGIT = *std::max_element(logits.begin<float>(), logits.end<float>());
    cv::Mat exps;
    cv::exp(logits - MAX_LOGIT, exps);
    const float SUM = static_cast<float>(cv::sum(exps)[0]);
    cv::Mat prob = exps / SUM;  // (1, num_classes)

    // 取最大概率与类别
    double confidence = 0.0;
    cv::Point class_id;
    cv::minMaxLoc(prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id);
    const int LABEL_ID = class_id.x;

    armor.confidence = static_cast<float>(confidence);
    armor.number = static_cast<ArmorNumber>(LABEL_ID);
  }

  // 末端过滤：阈值、忽略类别、大小-数字匹配规则
  armors.erase(std::remove_if(armors.begin(), armors.end(),
                              [this](const Armor& a)
                              {
                                // 置信度过滤
                                if (a.confidence < threshold_)
                                {
                                  return true;
                                }

                                // 忽略指定类别
                                for (const auto& ig : ignore_classes_)
                                {
                                  if (a.number == ig)
                                  {
                                    return true;
                                  }
                                }

                                // 大/小装甲与数字的先验匹配关系（根据项目规则可调整）
                                bool mismatch = false;
                                if (a.type == ArmorType::LARGE)
                                {
                                  mismatch = (a.number == ArmorNumber::OUTPOST ||
                                              a.number == ArmorNumber::TWO ||
                                              a.number == ArmorNumber::GUARD);
                                }
                                else if (a.type == ArmorType::SMALL)
                                {
                                  mismatch = (a.number == ArmorNumber::ONE ||
                                              a.number == ArmorNumber::BASE);
                                }
                                return mismatch;
                              }),
               armors.end());
}