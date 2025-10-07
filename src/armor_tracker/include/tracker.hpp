#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

#include "armor.hpp"
#include "extended_kalman_filter.hpp"
#include "transform.hpp"

// 装甲板数量 正常的4块 前哨站三块
enum class ArmorsNum
{
  NORMAL_4 = 4,
  OUTPOST_3 = 3
};

class Tracker  // 整车观测
{
 public:
  Tracker(double max_match_distance, double max_match_yaw_diff);

  void Init(const ArmorDetectorResults& armors_msg);

  void Update(const ArmorDetectorResults& armors_msg);

  enum State
  {             // 四个状态
    LOST,       // 丢失
    DETECTING,  // 观测中
    TRACKING,   // 跟踪中
    TEMP_LOST,  // 临时丢失
  } tracker_state_;

  ExtendedKalmanFilter ekf_;

  int tracking_thres_;
  int lost_thres_;

  // 装甲板情况
  ArmorNumber tracked_id_;             // 装甲板号
  ArmorDetectorResult tracked_armor_;  // 被跟踪的装甲板
  ArmorsNum tracked_armors_num_;       // 被跟踪装甲版数

  double info_position_diff_;
  double info_yaw_diff_;

  Eigen::VectorXd measurement_;  // 测量

  Eigen::VectorXd target_state_;  // 目标状态

  //? 储存另一片装甲板信息
  double dz_, another_r_;

  void InitEKF(const ArmorDetectorResult& a);

  void UpdateArmorsNum(const ArmorDetectorResult& a);

  void HandleArmorJump(const ArmorDetectorResult& a);

  double OrientationToYaw(const LibXR::Quaternion<double>& q);

  Eigen::Vector3d GetArmorPositionFromState(const Eigen::VectorXd& x);

  double max_match_distance_;
  double max_match_yaw_diff_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;
};

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
