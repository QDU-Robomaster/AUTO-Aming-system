#include "tracker.hpp"

#include "armor.hpp"
#include "cycle_value.hpp"
#include "libxr.hpp"
#include "transform.hpp"

// STD
#include <cfloat>

Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
    : tracker_state_(LOST),
      tracked_id_(ArmorNumber::INVALID),
      measurement_(Eigen::VectorXd::Zero(4)),
      target_state_(Eigen::VectorXd::Zero(9)),
      max_match_distance_(max_match_distance),
      max_match_yaw_diff_(max_match_yaw_diff)
{
}

void Tracker::Init(const ArmorDetectorResults& armors_msg)
{
  if (armors_msg.empty())
  {
    return;
  }

  double min_distance = DBL_MAX;  // 定义最大初始值
  tracked_armor_ = armors_msg[0];
  for (const auto& armor : armors_msg)
  {
    if (armor.distance_to_image_center < min_distance)
    {
      min_distance = armor.distance_to_image_center;
      tracked_armor_ = armor;
    }  // 选择距离屏幕中心最近的装甲板
       // 为被追踪的装甲板，一旦选定跟踪目标后，在LOST之前不会切换目标，即使新目标在中心
  }

  InitEKF(tracked_armor_);
  XR_LOG_DEBUG("Init EKF!");

  tracked_id_ = tracked_armor_.number;
  tracker_state_ = DETECTING;

  UpdateArmorsNum(tracked_armor_);
}

void Tracker::Update(const ArmorDetectorResults& armors_msg)
{
  Eigen::VectorXd ekf_prediction = ekf_.Predict();  // 根据整车c的预测，得出装甲板的位置
  XR_LOG_DEBUG("EKF predict");
  bool matched = false;  // 对预测的装甲板和观测的装甲板进行匹配
  target_state_ = ekf_prediction;  // 整车c的预测向量

  if (!armors_msg.empty())
  {
    ArmorDetectorResult same_id_armor;
    int same_id_armors_count = 0;
    auto predicted_position =
        GetArmorPositionFromState(ekf_prediction);  // 计算,根据原装甲板得到预测装甲板位置
    double min_position_diff = DBL_MAX;  // 最小位置差值,最大初始值
    double yaw_diff = DBL_MAX;  // 定义yaw差值,预测装甲板和真实装甲板

    for (const auto& armor : armors_msg)
    {  // 遍历当前装甲板
      // 只考虑具有相同 ID 的装甲,忽略其余装甲
      if (armor.number == tracked_id_)
      {
        same_id_armor = armor;
        same_id_armors_count++;
        // 误差分析,计算预测位置与当前装甲位置之间的差异
        auto p = armor.pose.translation;  // p是真正的观察到的 装甲板的 position
        Eigen::Vector3d position_vec(p.x(), p.y(), p.z());
        double position_diff = (predicted_position - position_vec).norm();

        if (position_diff < min_position_diff)
        {  // 位置小于最小匹配距离，表明这就是预测的那一块装甲板
          min_position_diff = position_diff;
          yaw_diff = abs(OrientationToYaw(armor.pose.rotation) - ekf_prediction(6));
          tracked_armor_ = armor;
        }
      }
    }

    // 存储tracker信息
    info_position_diff_ = min_position_diff;
    info_yaw_diff_ = yaw_diff;

    // 检查最近装甲的距离和偏航角差是否在阈值范围内
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_)
    {  // 最近装甲板距离与yaw差值比阈值小
      // 找到匹配的装甲板
      matched = true;  // 注意之前的 matched = false
      auto p = tracked_armor_.pose.translation;
      // 更新 EKF
      double measured_yaw =
          OrientationToYaw(tracked_armor_.pose.rotation);  // 测量的yaw值
      measurement_ = Eigen::Vector4d(p.x(), p.y(), p.z(), measured_yaw);
      target_state_ = ekf_.Update(measurement_);
      XR_LOG_DEBUG(
          "EKF update");  // 更新ekf [DEBUG] [timestamp] [armor_tracker]: EKF update
    }
    else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_)
    {
      // 未找到匹配的装甲，但仅有一个具有相同 ID 的装甲
      // 且偏航角发生了跳变，将此情况视为目标正在旋转并且装甲发生了 **跳变**
      HandleArmorJump(same_id_armor);  // 跳变处理
    }
    else
    {
      // 没找到匹配的装甲板
      XR_LOG_WARN("No matched armor found!");  //[DEBUG] [timestamp] [armor_tracker]: No
                                               // matched armor found!
    }
  }

  // 防止半径扩散
  if (target_state_(8) < 0.12)
  {
    target_state_(8) = 0.12;
    ekf_.SetState(target_state_);
  }
  else if (target_state_(8) > 0.4)
  {
    target_state_(8) = 0.4;
    ekf_.SetState(target_state_);
  }

  // 跟踪状态机制处理
  if (tracker_state_ == DETECTING)
  {
    if (matched)
    {
      detect_count_++;
      if (detect_count_ > tracking_thres_)
      {
        detect_count_ = 0;
        tracker_state_ = TRACKING;
      }
    }
    else
    {
      detect_count_ = 0;
      tracker_state_ = LOST;
    }
  }
  else if (tracker_state_ == TRACKING)
  {
    if (!matched)
    {
      tracker_state_ = TEMP_LOST;
      lost_count_++;
    }
  }
  else if (tracker_state_ == TEMP_LOST)
  {
    if (!matched)
    {
      lost_count_++;
      if (lost_count_ > lost_thres_)
      {
        lost_count_ = 0;
        tracker_state_ = LOST;
      }
    }
    else
    {
      tracker_state_ = TRACKING;
      lost_count_ = 0;
    }
  }
}

// 初始化ekf
void Tracker::InitEKF(const ArmorDetectorResult& a)
{
  double xa = a.pose.translation.x();
  double ya = a.pose.translation.y();
  double za = a.pose.translation.z();
  last_yaw_ = 0;
  double yaw = OrientationToYaw(a.pose.rotation);

  // 设置初始位置在目标后面0.2米
  target_state_ = Eigen::VectorXd::Zero(9);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  dz_ = 0, another_r_ = r;
  target_state_ << xc, 0, yc, 0, za, 0, yaw, 0, r;

  ekf_.SetState(target_state_);
}

void Tracker::UpdateArmorsNum(const ArmorDetectorResult&)
{
  if (tracked_id_ == ArmorNumber::OUTPOST)
  {
    tracked_armors_num_ = ArmorsNum::OUTPOST_3;  // 前哨站
  }
  else
  {
    tracked_armors_num_ = ArmorsNum::NORMAL_4;
  }
}

// 处理装甲板 **跳变**
void Tracker::HandleArmorJump(const ArmorDetectorResult& current_armor)
{
  double yaw = OrientationToYaw(current_armor.pose.rotation);
  target_state_(6) = yaw;
  UpdateArmorsNum(current_armor);
  // Only 4 armors has 2 radius and height
  if (tracked_armors_num_ == ArmorsNum::NORMAL_4)
  {
    dz_ = target_state_(4) - current_armor.pose.translation.z();
    target_state_(4) = current_armor.pose.translation.z();
    std::swap(target_state_(8), another_r_);
  }
  XR_LOG_WARN("Armor jump!");

  // 如果位置差大于 max_match_distance_，
  // 将此情况视为 EKF 发散，重置状态。
  auto p = current_armor.pose.translation;
  Eigen::Vector3d current_p(p.x(), p.y(), p.z());
  Eigen::Vector3d infer_p = GetArmorPositionFromState(target_state_);
  if ((current_p - infer_p).norm() > max_match_distance_)
  {
    double r = target_state_(8);
    target_state_(0) = p.x() + r * cos(yaw);  // xc
    target_state_(1) = 0;                     // vxc
    target_state_(2) = p.y() + r * sin(yaw);  // yc
    target_state_(3) = 0;                     // vyc
    target_state_(4) = p.z();                 // za
    target_state_(5) = 0;                     // vza
    XR_LOG_ERROR("Reset State!");
  }

  ekf_.SetState(target_state_);
}

// 姿态转换成偏航角(yaw)
double Tracker::OrientationToYaw(const LibXR::Quaternion<double>& q)
{
  // Get armor yaw
  LibXR::EulerAngle<double> eulr =
      LibXR::RotationMatrix<double>(q.ToRotationMatrix()).ToEulerAngle();
  auto yaw = eulr.Yaw();
  // Make yaw change continuous (-pi~pi to -inf~inf)
  const double DELTA =
      LibXR::CycleValue<double>(yaw) -
      LibXR::CycleValue<double>(last_yaw_);  // = wrapToPi(yaw - last_yaw_)
  yaw = last_yaw_ + DELTA;
  last_yaw_ = yaw;
  return yaw;
}

// 三维计算,根据我们一开始的装甲板解算得到的预测装甲板位置
Eigen::Vector3d Tracker::GetArmorPositionFromState(const Eigen::VectorXd& x)
{
  // 计算当前装甲板的预测位置
  double xc = x(0), yc = x(2), za = x(4);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}
