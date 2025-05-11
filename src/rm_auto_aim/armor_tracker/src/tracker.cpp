#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_auto_aim

{

Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST),
  tracked_id(std::string("")),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  max_match_distance_(max_match_distance),  // 对成员对象 max_match_distance 初始化
  max_match_yaw_diff_(max_match_yaw_diff)
{
}

void Tracker::init(const Armors::SharedPtr & armors_msg)  //tracker初始化
{
  // 初始条件判定
  if (armors_msg->armors.empty()) { //判断装甲板库是不是空的
    return;
  }

  // 简单地选择最接近图像中心的装甲板
  double min_distance = DBL_MAX;  //定义最大初始值
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    } //* 选择距离屏幕中心最近的装甲板 为被追踪的装甲板
  }

  initEKF(tracked_armor); //初始化卡尔曼
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!"); // 正在ekf初始化 [DEBUG] [timestamp] [armor_tracker]: Init EKF!

  tracked_id = tracked_armor.number;  //这块装甲板id被标记
  tracker_state = DETECTING;  //此时正在追踪

  updateArmorsNum(tracked_armor); //更新装甲板
} // Tracker::init


void Tracker::update(const Armors::SharedPtr & armors_msg)  //更新tracker
{
  // KF 预测
  Eigen::VectorXd ekf_prediction = ekf.predict();
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict"); // 正在ekf [DEBUG] [timestamp] [armor_tracker]: EKF predict

  bool matched = false; //?判断匹配 默认没找到的
  // 如果未找到匹配的装甲板，则使用 KF 预测作为默认目标状态 就是matched = false
  target_state = ekf_prediction;

  // 在找到之后
  if (!armors_msg->armors.empty())  //并非空
  {
    // “找到具有相同 ID 的最近装甲。”
    Armor same_id_armor;  //在找到装甲板之后,这是同时找到的另外的装甲版
    int same_id_armors_count = 0; //同时发现的数量
    auto predicted_position = getArmorPositionFromState(ekf_prediction);  //计算,根据原装甲板得到预测装甲板位置
    double min_position_diff = DBL_MAX; //最小位置差值,最大初始值
    double yaw_diff = DBL_MAX;  //定义yaw差值,预测装甲板和真实装甲板

    for (const auto & armor : armors_msg->armors) { //遍历当前装甲板
      // 只考虑具有相同 ID 的装甲,忽略其余装甲
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        // 误差分析,计算预测位置与当前装甲位置之间的差异
        auto p = armor.pose.position; // p是真正的观察到的 装甲板的 position
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        //* 计算预测位置与当前装甲位置之间的欧几里得距离

        // 找到离主装甲板最近预测装甲板
        if (position_diff < min_position_diff) {
          min_position_diff = position_diff;
          yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
          tracked_armor = armor;  //此时标记这个装甲板是被标记的装甲板
        }
      }
    }
    // 在同名装甲板之中 找到预测差值最小的一个,并且储存预测差值,?我感觉,在车小陀螺之中,靠近相机的一块装甲板动的快,预测的差值应该会大.所以一般来说这里用的装甲板是边缘的装甲板

    // 存储tracker信息
    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    // 检查最近装甲的距离和偏航角差是否在阈值范围内
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {  //最近装甲板距离与yaw差值比阈值小
      // 找到匹配的装甲板
      matched = true; //注意之前的 matched = false
      auto p = tracked_armor.pose.position;
      // 更新 EKF
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation); //测量的yaw值
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      target_state = ekf.update(measurement);
      RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update");  //更新ekf [DEBUG] [timestamp] [armor_tracker]: EKF update
    }
    else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      // 未找到匹配的装甲，但仅有一个具有相同 ID 的装甲
      // 且偏航角发生了跳变，将此情况视为目标正在旋转并且装甲发生了 **跳变**
      handleArmorJump(same_id_armor); //跳变处理
    }
    else {
      // 没找到匹配的装甲板
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");  //[DEBUG] [timestamp] [armor_tracker]: No matched armor found!
    }
  }

  // 防止半径扩散
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    ekf.setState(target_state);
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    ekf.setState(target_state);
  }

  // 跟踪状态机制处理
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}


// 初始化ekf
void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  //设置初始位置在目标后面0.2米
  target_state = Eigen::VectorXd::Zero(9);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  dz = 0, another_r = r;
  target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

  ekf.setState(target_state);
}


// 大改
void Tracker::updateArmorsNum(const Armor & armor)
{
  if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;  //前哨站
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

// 处理装甲板 **跳变**
void Tracker::handleArmorJump(const Armor & current_armor)
{
  double yaw = orientationToYaw(current_armor.pose.orientation);
  target_state(6) = yaw;
  updateArmorsNum(current_armor);
  // Only 4 armors has 2 radius and height
  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    dz = target_state(4) - current_armor.pose.position.z;
    target_state(4) = current_armor.pose.position.z;
    std::swap(target_state(8), another_r);
  }
  RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

  // 如果位置差大于 max_match_distance_，
  // 将此情况视为 EKF 发散，重置状态。
  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // za
    target_state(5) = 0;                   // vza
    RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
  }

  ekf.setState(target_state);
}

//姿态转换成偏航角(yaw)
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

// 三维计算,根据我们一开始的装甲板解算得到的预测装甲板位置
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  // 计算当前装甲板的预测位置
  double xc = x(0), yc = x(2), za = x(4);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace rm_auto_aim
