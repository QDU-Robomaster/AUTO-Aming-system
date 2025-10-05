#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STL
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"
#include "libxr.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(bool debug, int detect_color, int binary_thr,
                                     double light_min_ratio, double light_max_ratio,
                                     double light_max_angle, double armor_min_light_ratio,
                                     double armor_min_small_center_distance,
                                     double armor_max_small_center_distance,
                                     double armor_min_large_center_distance,
                                     double armor_max_large_center_distance,
                                     double armor_max_angle, double classifier_threshold,
                                     std::vector<std::string> ignore_classes)
    : binary_thres_(binary_thr),
      detect_color_(detect_color),
      light_min_ratio_(light_min_ratio),
      light_max_ratio_(light_max_ratio),
      light_max_angle_(light_max_angle),
      armor_min_light_ratio_(armor_min_light_ratio),
      armor_min_small_center_distance_(armor_min_small_center_distance),
      armor_max_small_center_distance_(armor_max_small_center_distance),
      armor_min_large_center_distance_(armor_min_large_center_distance),
      armor_max_large_center_distance_(armor_max_large_center_distance),
      armor_max_angle_(armor_max_angle),
      classifier_threshold_(classifier_threshold),
      ignore_classes_(ignore_classes),
      debug_(debug)
{
  XR_LOG_INFO("Starting DetectorNode!");

  // Detector
  detector_ = initDetector();

  node_ = new rclcpp::Node("armor_detector");

  // Armors Publisher
  armors_pub_ = node_->create_publisher<auto_aim_interfaces::msg::Armors>(
      "/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detector/marker", 10);

  // Debug Publishers
  if (debug_)
  {
    createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
      "debug",
      [this](const rclcpp::Parameter& p)
      {
        debug_ = p.as_bool();
        debug_ ? createDebugPublishers() : destroyDebugPublishers();
      });

  cam_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
      {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
        cam_info_sub_.reset();
      });

  img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
}

void ArmorDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  auto armors = detectArmors(img_msg);

  if (pnp_solver_ != nullptr)
  {
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto& armor : armors)
    {
      cv::Mat rvec, tvec;
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
      if (success)
      {
        // Fill basic info
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        tf2::Matrix3x3 tf2_rotation_matrix(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
            rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
            rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
            rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // Fill the distance to image center
        armor_msg.distance_to_image_center =
            pnp_solver_->calculateDistanceToCenter(armor.center);

        // Fill the markers
        armor_marker_.id++;
        armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
        armor_marker_.pose = armor_msg.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        armors_msg_.armors.emplace_back(armor_msg);
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      }
      else
      {
        XR_LOG_WARN("PnP failed!");
      }
    }

    // Publishing detected armors
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    publishMarkers();
  }
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = binary_thres_;

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = detect_color_;

  Detector::LightParams l_params = {.min_ratio = light_min_ratio_,
                                    .max_ratio = light_max_ratio_,
                                    .max_angle = light_max_angle_};

  Detector::ArmorParams a_params = {
      .min_light_ratio = armor_min_light_ratio_,
      .min_small_center_distance = armor_min_small_center_distance_,
      .max_small_center_distance = armor_max_small_center_distance_,
      .min_large_center_distance = armor_min_large_center_distance_,
      .max_large_center_distance = armor_max_large_center_distance_,
      .max_angle = armor_max_angle_};

  auto detector =
      std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // Init classifier
  std::string pkg_path = MODEL_PATH;
  auto model_path = pkg_path + "/mlp.onnx";
  auto label_path = pkg_path + "/label.txt";
  double threshold = classifier_threshold_;
  std::vector<std::string> ignore_classes = ignore_classes_;
  detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path,
                                                            threshold, ignore_classes);

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // Update params
  detector_->binary_thres = node_->get_parameter("binary_thres").as_int();
  detector_->detect_color = node_->get_parameter("detect_color").as_int();
  detector_->classifier->threshold =
      node_->get_parameter("classifier_threshold").as_double();

  auto armors = detector_->detect(img);

  auto final_time = node_->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  XR_LOG_DEBUG("Latency: %fms", latency);

  // Publish debug info
  if (debug_)
  {
    binary_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
              [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
    std::sort(detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
              [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty())
    {
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
          *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    detector_->drawResults(img);
    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

void ArmorDetectorNode::createDebugPublishers()
{
  lights_data_pub_ = node_->create_publisher<auto_aim_interfaces::msg::DebugLights>(
      "/detector/debug_lights", 10);
  armors_data_pub_ = node_->create_publisher<auto_aim_interfaces::msg::DebugArmors>(
      "/detector/debug_armors", 10);

  binary_img_pub_ = image_transport::create_publisher(node_, "/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(node_, "/detector/number_img");
  result_img_pub_ = image_transport::create_publisher(node_, "/detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();  // 释放调试发布者所占用的资源，并将其置为无效状态
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();  // 关闭调试发布者，使其不再发布任何消息
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim
