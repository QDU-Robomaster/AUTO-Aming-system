#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_executor/SolveTrajectory.hpp"
#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"

namespace rm_auto_aim
{
using armors_tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
using velocity_tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Velocity>;
class ArmorTrackerNode
{
 public:
  explicit ArmorTrackerNode(
      double max_armor_distance = 10.0, double tracker_max_match_distance = 0.15,
      double tracker_max_match_yaw_diff = 1.0, int tracker_tracking_thres = 5,
      double tracker_lost_time_thres = 0.3, double tracker_k = 0.092,
      int tracker_bias_time = 100, double tracker_s_bias = 0.19133,
      double tracker_z_bias = 0.21265, double ekf_sigma2_q_xyz = 20.0,
      double ekf_sigma2_q_yaw = 100.0, double ekf_sigma2_q_r = 800,
      double ekf_r_xyz_factor = 0.05, double ekf_r_yaw = 0.02,
      std::string target_frame = "odom");

 private:
  void velocityCallback(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg);

  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  void publishMarkers(const auto_aim_interfaces::msg::Target& target_msg);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;

  rclcpp::Node* node_;

  // Armor tracker
  double s2qxyz_, s2qyaw_, s2qr_;
  double r_xyz_factor, r_yaw;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<SolveTrajectory> gaf_solver;

  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<armors_tf2_filter> armors_filter_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_sub_;
  std::shared_ptr<velocity_tf2_filter> velocity_filter_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
