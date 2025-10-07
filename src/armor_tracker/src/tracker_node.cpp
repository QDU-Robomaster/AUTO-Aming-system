#include "tracker_node.hpp"

#include <cstring>
#include <memory>
#include <vector>

#include "SolveTrajectory.hpp"
#include "armor.hpp"
#include "libxr.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "timebase.hpp"
#include "transform.hpp"

ArmorTrackerNode::ArmorTrackerNode(
    double max_armor_distance, double tracker_max_match_distance,
    double tracker_max_match_yaw_diff, int tracker_tracking_thres,
    double tracker_lost_time_thres, double tracker_k, int tracker_bias_time,
    double tracker_s_bias, double tracker_z_bias, double ekf_sigma2_q_xyz,
    double ekf_sigma2_q_yaw, double ekf_sigma2_q_r, double ekf_r_xyz_factor,
    double ekf_r_yaw, std::string target_frame,
    LibXR::Transform<double> base_transform_static)
    : max_armor_distance_(max_armor_distance),
      s2qxyz_(ekf_sigma2_q_xyz),
      s2qyaw_(ekf_sigma2_q_yaw),
      s2qr_(ekf_sigma2_q_r),
      r_xyz_factor(ekf_r_xyz_factor),
      r_yaw(ekf_r_yaw),
      lost_time_thres_(tracker_lost_time_thres),
      base_transform_static_(base_transform_static),
      target_frame_(target_frame)
{
  XR_LOG_INFO("Starting TrackerNode!");
  // Tracker
  double max_match_distance = tracker_max_match_distance;
  double max_match_yaw_diff = tracker_max_match_yaw_diff;
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = tracker_tracking_thres;

  double k = tracker_k;
  int bias_time = tracker_bias_time;
  double s_bias = tracker_s_bias;
  double z_bias = tracker_z_bias;
  gaf_solver = std::make_unique<SolveTrajectory>(k, bias_time, s_bias, z_bias);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function 过程函数对状态进行更新
  auto f = [this](const Eigen::VectorXd& x)
  {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    x_new(4) += x(5) * dt_;
    x_new(6) += x(7) * dt_;
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd&)
  {
    Eigen::MatrixXd f(9, 9);
    // clang-format off 临时禁用格式化工具，确保矩阵按指定格式排列
    f << 1, dt_, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, dt_, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, dt_, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, dt_, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1;
    // clang-format on
    return f;
  };
  // h - Observation function 观测函数对状态进行测量
  auto h = [](const Eigen::VectorXd& x)
  {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(4);               // za
    z(3) = x(6);               // yaw
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {  //状体量到观测量的一个转换矩阵，将整车c的状态转换为装甲板a的状态，用预测之后的c推出预测之后的a
    Eigen::MatrixXd h(4, 9);
    double yaw = x(6), r = x(8);
    // clang-format off
    //              xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
    h <<  /*xa*/    1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
          /*ya*/    0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
          /*za*/    0,   0,   0,   0,   1,   0,   0,          0,   0,
          /*yaw*/   0,   0,   0,   0,   0,   0,   1,          0,   0;
    // clang-format on
    return h;
  };
  auto u_q = [this]()
  {
    Eigen::MatrixXd q(9, 9);
    double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    // clang-format on
    return q;
  };

  auto u_r = [this](const Eigen::VectorXd& z)
  {
    Eigen::DiagonalMatrix<double, 4> r;
    double x = r_xyz_factor;
    r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  LibXR::Topic armors_topic = LibXR::Topic::Find("/detector/armors");

  auto armors_cb = LibXR::Topic::Callback::Create(
      [](bool, ArmorTrackerNode* tracker_node, LibXR::RawData& data)
      {
        auto armors_msg = reinterpret_cast<ArmorDetectorResults*>(data.addr_);
        tracker_node->armorsCallback(*armors_msg);
      },
      this);

  armors_topic.RegisterCallback(armors_cb);

  LibXR::Topic velocity_topic = LibXR::Topic::FindOrCreate<double>(
      "/current_velocity", nullptr, false, false, false);

  auto velocity_cb = LibXR::Topic::Callback::Create(
      [](bool, ArmorTrackerNode* tracker_node, LibXR::RawData& data)
      {
        auto velocity_msg = reinterpret_cast<double*>(data.addr_);
        tracker_node->velocityCallback(*velocity_msg);
      },
      this);

  velocity_topic.RegisterCallback(velocity_cb);

  LibXR::Topic base_rotation_topic =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<double>>("/base_rotation");

  auto base_rotation_cb = LibXR::Topic::Callback::Create(
      [](bool, ArmorTrackerNode* tracker_node, LibXR::RawData& data)
      {
        LibXR::Mutex::LockGuard(tracker_node->base_rotation_lock_);
        auto base_rotation_msg = reinterpret_cast<LibXR::Quaternion<double>*>(data.addr_);
        tracker_node->base_rotation_ = *base_rotation_msg;
      },
      this);

  base_rotation_topic.RegisterCallback(base_rotation_cb);
}

void ArmorTrackerNode::velocityCallback(double velocity_msg)
{
  gaf_solver->init(velocity_msg);
}

void ArmorTrackerNode::armorsCallback(ArmorDetectorResults& armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  base_rotation_lock_.Lock();
  for (auto& armor : armors_msg)
  {
    LibXR::Transform<double> tf = armor.pose;

    armor.pose = LibXR::Transform<double>(base_rotation_, {0.0, 0.0, 0.0}) +
                 base_transform_static_ + tf;
  }

  base_rotation_lock_.Unlock();

  // Filter abnormal armors
  armors_msg.erase(std::remove_if(armors_msg.begin(), armors_msg.end(),
                                  [this](const ArmorDetectorResult& armor)
                                  {
                                    return abs(armor.pose.translation.z()) > 1.2 ||
                                           Eigen::Vector2d(armor.pose.translation.x(),
                                                           armor.pose.translation.y())
                                                   .norm() > max_armor_distance_;
                                  }),
                   armors_msg.end());

  // Init message
  TrackerInfo info_msg;
  SolveTrajectory::Target target_msg;
  Send send_msg;
  strncpy(target_msg.id.data(), target_frame_.c_str(), target_frame_.size());

  auto time = LibXR::Timebase::GetMicroseconds();

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST)
  {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  }
  else
  {
    // 求时间差
    dt_ = (time - last_time_).ToSecond();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
    tracker_->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x() = tracker_->measurement(0);
    info_msg.position.y() = tracker_->measurement(1);
    info_msg.position.z() = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_topic_.Publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING)
    {
      target_msg.tracking = false;
    }
    else if (tracker_->tracker_state == Tracker::TRACKING ||
             tracker_->tracker_state == Tracker::TEMP_LOST)
    {
      target_msg.tracking = true;
      // Fill target message
      const auto& state = tracker_->target_state;
      strncpy(target_msg.id.data(), tracker_->tracked_id.c_str(), target_frame_.size());
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x() = state(0);
      target_msg.velocity.x() = state(1);
      target_msg.position.y() = state(2);
      target_msg.velocity.y() = state(3);
      target_msg.position.z() = state(4);
      target_msg.velocity.z() = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.dz = tracker_->dz;

      float pitch = 0, yaw = 0, aim_x = 0, aim_y = 0, aim_z = 0;
      gaf_solver->autoSolveTrajectory(pitch, yaw, aim_x, aim_y, aim_z, &target_msg);

      gaf_solver->setFireCallback([&](bool is_fire) { send_msg.is_fire = is_fire; });
      send_msg.position.x() = aim_x;
      send_msg.position.y() = aim_y;
      send_msg.position.z() = aim_z;
      send_msg.v_yaw = target_msg.v_yaw;
      send_msg.pitch = pitch;
      send_msg.yaw = yaw;

      // std::cout << "aim_x: " << aim_x << " aim_y: " << aim_y << " aim_z: " << aim_z <<
      // " pitch: " << pitch << " yaw: " << yaw << std::endl;
    }
  }

  last_time_ = time;

  send_topic_.Publish(send_msg);
  target_topic_.Publish(target_msg);
}
