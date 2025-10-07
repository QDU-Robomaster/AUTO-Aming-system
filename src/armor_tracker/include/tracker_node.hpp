#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// STD
#include <memory>
#include <string>
#include <vector>

#include "SolveTrajectory.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "tracker.hpp"
#include "transform.hpp"

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
      std::string target_frame = "odom",
      LibXR::Transform<double> base_transform_static = {});

  struct TrackerInfo
  {
    double position_diff;
    double yaw_diff;
    LibXR::Position<double> position;
    double yaw;
  };

  struct Send
  {
    bool is_fire;
    LibXR::Position<double> position;
    double v_yaw;
    double pitch;
    double yaw;
    Eigen::Matrix<double, 3, 1> cmd_vel_linear;
    Eigen::Matrix<double, 3, 1> cmd_vel_angular;
  };

 private:
  void velocityCallback(double velocity_msg);

  void armorsCallback(ArmorDetectorResults& armors_ptr);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  LibXR::MicrosecondTimestamp last_time_;
  double dt_;

  // Armor tracker
  double s2qxyz_, s2qyaw_, s2qr_;
  double r_xyz_factor, r_yaw;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<SolveTrajectory> gaf_solver;

  // Subscriber with tf2 message_filter
  LibXR::Transform<double> base_transform_static_;
  LibXR::Quaternion<double> base_rotation_;
  LibXR::Mutex base_rotation_lock_;
  std::string target_frame_;
  LibXR::Topic tf_topic_ = LibXR::Topic("/tf", sizeof(LibXR::Quaternion<double>));

  // Tracker info publisher
  LibXR::Topic info_topic_ = LibXR::Topic("/tracker/info", sizeof(TrackerInfo));

  // Publisher
  LibXR::Topic target_topic_ =
      LibXR::Topic("/tracker/target", sizeof(SolveTrajectory::Target));
  LibXR::Topic send_topic_ = LibXR::Topic("/tracker/send", sizeof(Send));
};

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
