#include "armor_tracker/tracker_node.hpp"

#include <memory>
#include <vector>

#include "libxr.hpp"

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(
    double max_armor_distance, double tracker_max_match_distance,
    double tracker_max_match_yaw_diff, int tracker_tracking_thres,
    double tracker_lost_time_thres, double tracker_k, int tracker_bias_time,
    double tracker_s_bias, double tracker_z_bias, double ekf_sigma2_q_xyz,
    double ekf_sigma2_q_yaw, double ekf_sigma2_q_r, double ekf_r_xyz_factor,
    double ekf_r_yaw, std::string target_frame)
    : max_armor_distance_(max_armor_distance),
      s2qxyz_(ekf_sigma2_q_xyz),
      s2qyaw_(ekf_sigma2_q_yaw),
      s2qr_(ekf_sigma2_q_r),
      r_xyz_factor(ekf_r_xyz_factor),
      r_yaw(ekf_r_yaw),
      lost_time_thres_(tracker_lost_time_thres),
      target_frame_(target_frame)
{
  XR_LOG_INFO("Starting TrackerNode!");

  node_ = rclcpp::Node::make_shared("armor_tracker");

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

  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/tracker/reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        tracker_->tracker_state = Tracker::LOST;
        response->success = true;
        XR_LOG_INFO("Tracker reset!");
        return;
      });

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(), node_->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(node_, "/detector/armors", rmw_qos_profile_sensor_data);

  armors_filter_ = std::make_shared<armors_tf2_filter>(
      armors_sub_, *tf2_buffer_, target_frame_, 10, node_->get_node_logging_interface(),
      node_->get_node_clock_interface(), std::chrono::duration<int>(1));

  // Register a callback with tf2_ros::MessageFilter to be called when transforms are
  // available
  armors_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  velocity_sub_ = node_->create_subscription<auto_aim_interfaces::msg::Velocity>(
      "/current_velocity",
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
      std::bind(&ArmorTrackerNode::velocityCallback, this, std::placeholders::_1));

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
}

void ArmorTrackerNode::velocityCallback(
    const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
{
  gaf_solver->init(velocity_msg);
}

void ArmorTrackerNode::armorsCallback(
    const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  for (auto& armor : armors_msg->armors)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try
    {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    }
    catch (const tf2::ExtrapolationException& ex)
    {
      XR_LOG_ERROR("Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
      std::remove_if(
          armors_msg->armors.begin(), armors_msg->armors.end(),
          [this](const auto_aim_interfaces::msg::Armor& armor)
          {
            return abs(armor.pose.position.z) > 1.2 ||
                   Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                       max_armor_distance_;
          }),
      armors_msg->armors.end());

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  auto_aim_interfaces::msg::Send send_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST)
  {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  }
  else
  {
    // 求时间差
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
    tracker_->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
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
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.dz = tracker_->dz;

      float pitch = 0, yaw = 0, aim_x = 0, aim_y = 0, aim_z = 0;
      auto msg = std::make_shared<auto_aim_interfaces::msg::Target>(target_msg);
      gaf_solver->autoSolveTrajectory(pitch, yaw, aim_x, aim_y, aim_z, msg);

      gaf_solver->setFireCallback([&](bool is_fire) { send_msg.is_fire = is_fire; });
      send_msg.position.x = aim_x;
      send_msg.position.y = aim_y;
      send_msg.position.z = aim_z;
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

  publishMarkers(target_msg);
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target& target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking)
  {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y,
           za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y,
           vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++)
    {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4)
      {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      }
      else
      {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
  }
  else
  {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(armor_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_topic_.Publish(marker_array);
}

}  // namespace rm_auto_aim
