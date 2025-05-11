#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

// 装甲板数量 正常的4块 前哨站三块
enum class ArmorsNum { NORMAL_4 = 4,OUTPOST_3 = 3 };

class Tracker //整车观测
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg);

  void update(const Armors::SharedPtr & armors_msg);

  ExtendedKalmanFilter ekf;

  int tracking_thres;
  int lost_thres;

  enum State {  //四个状态
    LOST, //丢失
    DETECTING,  //观测中
    TRACKING, //跟踪中
    TEMP_LOST,  //临时丢失
  } tracker_state;

  // 装甲板情况
  std::string tracked_id; //装甲板号
  Armor tracked_armor;  //被跟踪的装甲板
  ArmorsNum tracked_armors_num; //被跟踪装甲版数

  double info_position_diff;
  double info_yaw_diff;

  Eigen::VectorXd measurement;  //测量

  Eigen::VectorXd target_state; //目标状态

  //? 储存另一片装甲板信息
  double dz, another_r;

private:
  void initEKF(const Armor & a);

  void updateArmorsNum(const Armor & a);

  void handleArmorJump(const Armor & a);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;
  double max_match_yaw_diff_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
