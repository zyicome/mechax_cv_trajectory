// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.hpp>
#include <opencv2/core/hal/interface.h>
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

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

#include <chrono>

namespace rm_left_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("left_armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting LeftDetectorNode!");

  // Detector
  detector_ = initDetector();

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/left_camera/detector/armors", rclcpp::SensorDataQoS());

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

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/left_camera/detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  std::cout << "debug_" << debug_<<std::endl;
  if (debug_) {
    createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  /*cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    // "/camera/camera_info", rclcpp::SensorDataQoS(),
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      //cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      //cam_center_ = cv::Point2f(320, 240);
      cam_center_ = cv::Point2f(camera_info->width / 2, camera_info->height / 2);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      camera_matrix_.at<double>(0,0) = camera_info->k[0];
    camera_matrix_.at<double>(0,2) = camera_info->k[2];
    camera_matrix_.at<double>(1,1) = camera_info->k[4];
    camera_matrix_.at<double>(1,2) = camera_info->k[5];
    camera_matrix_.at<double>(2,2) = 1.0;
      cam_info_sub_.reset();
    });*/

    std::array<double,9> camera_matrix_data = {1760.81949, 0.0, 645.42614, 0.0, 1755.71066, 605.24265, 0.0, 0.0, 1.0};
    std::vector<double> distortion_coefficients_data = {-0.089674, -0.010603, 0.003439, -0.007395, 0.0};
    camera_matrix_.at<double>(0,0) = 1760.81949;
    camera_matrix_.at<double>(0,2) = 645.42614;
    camera_matrix_.at<double>(1,1) = 1755.71066;
    camera_matrix_.at<double>(1,2) = 605.24265;
    camera_matrix_.at<double>(2,2) = 1.0;
    distortion_coefficients_.at<double>(0,0) = -0.089674;
    distortion_coefficients_.at<double>(0,1) = -0.010603;
    distortion_coefficients_.at<double>(0,2) = 0.003439;
    distortion_coefficients_.at<double>(0,3) = -0.007395;
    distortion_coefficients_.at<double>(0,4) = 0.0;
    cam_center_ = cv::Point2f(1440 / 2, 1080 / 2);
    pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix_data, distortion_coefficients_data);

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    // "/camera/image_color", rclcpp::SensorDataQoS(),
    "/image_left", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  decision_sub_ = this->create_subscription<std_msgs::msg::Int8>("/serial/decision", 10,
    std::bind(&ArmorDetectorNode::decisionCallback, this, std::placeholders::_1));

    decision = -1;

    detector_->classifier->ignore_classes_ = {"negative","guard","base"};

    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    //----------------------------------------------------------------------------------
    start = std::chrono::steady_clock::now();
    end = std::chrono::steady_clock::now();
    fps = 0;
    now_fps = 0;
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = end - start;

  if(diff.count() >= 1)
  {
    std::cout << diff.count() << "s and detector receive fps: " << fps<< std::endl;
    now_fps = fps;
    start = std::chrono::steady_clock::now();
    fps = 0;
  }

  fps++;

  auto armors = detectArmors(img_msg);

  if (pnp_solver_ != nullptr) {
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      cv::Mat rvec, tvec;
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
      if (success) {
        // Fill basic info
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // Fill pose
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);

      armorpose_mat.at<double>(0,0) = armor_msg.pose.position.x;
      armorpose_mat.at<double>(1,0) = armor_msg.pose.position.y;
      armorpose_mat.at<double>(2,0) = armor_msg.pose.position.z;

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
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

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
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }

    // Publishing detected armors
    auto time_point = std::chrono::high_resolution_clock::now();
    auto epoch = time_point.time_since_epoch();
    double time = std::chrono::duration<double>(epoch).count();
    armors_msg_.time = time;
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    publishMarkers();
  }
}

void ArmorDetectorNode::decisionCallback(const std_msgs::msg::Int8 msg)
{
  //std::cout << "msg.data" << msg.data << std::endl;
  if(decision == msg.data)
  {
    return;
  }
  geometry_msgs::msg::TransformStamped static_transformStamped;
  decision = msg.data;
  if(decision == 0)
  {
    detector_->classifier->ignore_classes_.clear();
    detector_->classifier->ignore_classes_.push_back("negative");
    detector_->classifier->ignore_classes_.push_back("guard");
    detector_->classifier->ignore_classes_.push_back("base");
    /*static_transformStamped.header.stamp = this->now();
    static_transformStamped.header.frame_id = "left_gimbal_link";
    static_transformStamped.child_frame_id = "left_camera_link";
    static_transformStamped.transform.translation.x = 0.110;
    static_transformStamped.transform.translation.y = 0.045;
    static_transformStamped.transform.translation.z = 0.030;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 3.1415 / 120.0, 3.1415 / 300);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    tf_publisher_->sendTransform(static_transformStamped);*/
  }
  else if(decision == 1)
  {
    detector_->classifier->ignore_classes_.clear();
    detector_->classifier->ignore_classes_.push_back("negative");
    detector_->classifier->ignore_classes_.push_back("base");
    /*static_transformStamped.header.stamp = this->now();
    static_transformStamped.header.frame_id = "left_gimbal_link";
    static_transformStamped.child_frame_id = "left_camera_link";
    static_transformStamped.transform.translation.x = 0.110;
    static_transformStamped.transform.translation.y = 0.045;
    static_transformStamped.transform.translation.z = 0.030;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 3.1415 / 120, 3.1415 / 300);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    tf_publisher_->sendTransform(static_transformStamped);*/
  }
  else if(decision == 2)
  {
    detector_->classifier->ignore_classes_.clear();
    detector_->classifier->ignore_classes_.push_back("negative");
    detector_->classifier->ignore_classes_.push_back("1");
    detector_->classifier->ignore_classes_.push_back("2");
    detector_->classifier->ignore_classes_.push_back("3");
    detector_->classifier->ignore_classes_.push_back("4");
    detector_->classifier->ignore_classes_.push_back("guard");
    detector_->classifier->ignore_classes_.push_back("base");
    /*static_transformStamped.header.stamp = this->now();
    static_transformStamped.header.frame_id = "left_gimbal_link";
    static_transformStamped.child_frame_id = "left_camera_link";
    static_transformStamped.transform.translation.x = 0.105;
    static_transformStamped.transform.translation.y = 0.00;
    static_transformStamped.transform.translation.z = 0.030;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    tf_publisher_->sendTransform(static_transformStamped);*/
  }
}

//}  // namespace rm_auto_aim

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 80, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.8),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("left_armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.8);
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // Update params
  detector_->binary_thres = get_parameter("binary_thres").as_int();
  detector_->detect_color = get_parameter("detect_color").as_int();
  detector_->detect_color = 1;
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  auto armors = detector_->detect(img);

  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  // Publish debug info
  if (debug_) {
    binary_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) {
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
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

void ArmorDetectorNode::createDebugPublishers()
{
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/left_camera/detector/debug_lights", 10);
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/left_camera/detector/debug_armors", 10);

  binary_img_pub_ = image_transport::create_publisher(this, "/left_camera/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "/left_camera/detector/number_img");
  result_img_pub_ = image_transport::create_publisher(this, "/left_camera/detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_left_auto_aim::ArmorDetectorNode)
