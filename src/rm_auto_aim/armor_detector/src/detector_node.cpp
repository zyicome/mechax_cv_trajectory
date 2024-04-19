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

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Detector
  detector_ = initDetector();

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
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

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  changeyaw_pub = this->create_publisher<auto_aim_interfaces::msg::Bias>("/trajectory/changeyaw", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", false);
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

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
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
    });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    // "/camera/image_color", rclcpp::SensorDataQoS(),
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  needpose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/tracker/needpose", 10,
    std::bind(&ArmorDetectorNode::needposeCallback, this, std::placeholders::_1));

  armorpose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/tracker/armorpose", 10,
    std::bind(&ArmorDetectorNode::armorposeCallback, this, std::placeholders::_1));
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
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
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    publishMarkers();
  }
}

void ArmorDetectorNode::needposeCallback(const geometry_msgs::msg::PointStamped::SharedPtr needpose_ptr)
{
  needpose = cv::Point3f(needpose_ptr->point.x,needpose_ptr->point.y,needpose_ptr->point.z);
  cv::Mat needpose_mat = cv::Mat::zeros(3,1,CV_64FC1);
  needpose_mat.at<double>(0,0) = needpose_ptr->point.x;
  needpose_mat.at<double>(1,0) = needpose_ptr->point.y;
  needpose_mat.at<double>(2,0) = needpose_ptr->point.z;
  cv::Mat pose_mat = cv::Mat::zeros(3,1,CV_64FC1);
  pose_mat = camera_matrix_ * needpose_mat;
  //std::cout << "camera_matrix: " << camera_matrix_<<std::endl;
  pose_mat.at<double>(0,0) = pose_mat.at<double>(0,0) / fabs(pose_mat.at<double>(2,0));
  pose_mat.at<double>(1,0) = pose_mat.at<double>(1,0) / fabs(pose_mat.at<double>(2,0));
  pose_mat.at<double>(2,0) = 1.0;
  needpose_img = cv::Point2f(pose_mat.at<double>(0,0), pose_mat.at<double>(1,0));
  /*std::cout << "needpose_ptr->pose.position.x : " << needpose_ptr->point.x << std::endl;
  std::cout << "needpose_ptr->pose.position.y : " << needpose_ptr->point.y << std::endl;
  std::cout << "needpose_ptr->pose.position.z : " << needpose_ptr->point.z << std::endl;
  std::cout << "needpose_img.x : " << needpose_img.x << std::endl;
  std::cout << "needpose_img.y : " << needpose_img.y << std::endl;*/
}

void ArmorDetectorNode::armorposeCallback(const geometry_msgs::msg::PointStamped::SharedPtr armorpose_ptr)
{
  cv::Mat prediction_armorpose_mat = cv::Mat::zeros(3,1,CV_64FC1);
  prediction_armorpose_mat.at<double>(0,0) = armorpose_ptr->point.x;
  prediction_armorpose_mat.at<double>(1,0) = armorpose_ptr->point.y;
  prediction_armorpose_mat.at<double>(2,0) = armorpose_ptr->point.z;
  prediction_armorpose = cv::Point3f(prediction_armorpose_mat.at<double>(0,0),prediction_armorpose_mat.at<double>(1,0),prediction_armorpose_mat.at<double>(2,0));
  if(prediction_armorpose_mat.at<double>(0,0)==0 && prediction_armorpose_mat.at<double>(1,0)==0 && prediction_armorpose_mat.at<double>(2,0)==0)
  {
    return;
  }
  cv::Mat pose_mat = cv::Mat::zeros(3,1,CV_64FC1);
  pose_mat = camera_matrix_ * prediction_armorpose_mat;
  //std::cout << "camera_matrix: " << camera_matrix_<<std::endl;
  pose_mat.at<double>(0,0) = pose_mat.at<double>(0,0) / fabs(pose_mat.at<double>(2,0));
  pose_mat.at<double>(1,0) = pose_mat.at<double>(1,0) / fabs(pose_mat.at<double>(2,0));
  pose_mat.at<double>(2,0) = 1.0;
  prediction_armorpose_img = cv::Point2f(pose_mat.at<double>(0,0), pose_mat.at<double>(1,0));
  /*std::cout << "armor_msg_->pose.position.x : " << armorpose_mat.at<double>(0,0) << std::endl;
  std::cout << "armor_msg_->pose.position.y : " << armorpose_mat.at<double>(1,0) << std::endl;
  std::cout << "armor_msg_->pose.position.z : " << armorpose_mat.at<double>(2,0) << std::endl;
  std::cout << "needpose_img.x : " << needpose_img.x << std::endl;
  std::cout << "needpose_img.y : " << needpose_img.y << std::endl;*/
}

void ArmorDetectorNode::getarmorpose()
{
  /*cv::Mat armorpose_mat = cv::Mat::zeros(3,1,CV_64FC1);
  armorpose_mat.at<double>(0,0) = armorpose_ptr->point.x;
  armorpose_mat.at<double>(1,0) = armorpose_ptr->point.y;
  armorpose_mat.at<double>(2,0) = armorpose_ptr->point.z;*/
  armorpose = cv::Point3f(armorpose_mat.at<double>(0,0),armorpose_mat.at<double>(1,0),armorpose_mat.at<double>(2,0));
  if(armorpose_mat.at<double>(0,0)==0 && armorpose_mat.at<double>(1,0)==0 && armorpose_mat.at<double>(2,0)==0)
  {
    return;
  }
  cv::Mat arpose_mat = cv::Mat::zeros(3,1,CV_64FC1);
  arpose_mat = camera_matrix_ * armorpose_mat;
  //std::cout << "camera_matrix: " << camera_matrix_<<std::endl;
  arpose_mat.at<double>(0,0) = arpose_mat.at<double>(0,0) / fabs(arpose_mat.at<double>(2,0));
  arpose_mat.at<double>(1,0) = arpose_mat.at<double>(1,0) / fabs(arpose_mat.at<double>(2,0));
  arpose_mat.at<double>(2,0) = 1.0;
  armorpose_img = cv::Point2f(arpose_mat.at<double>(0,0), arpose_mat.at<double>(1,0));
  /*std::cout << "armor_msg_->pose.position.x : " << armorpose_mat.at<double>(0,0) << std::endl;
  std::cout << "armor_msg_->pose.position.y : " << armorpose_mat.at<double>(1,0) << std::endl;
  std::cout << "armor_msg_->pose.position.z : " << armorpose_mat.at<double>(2,0) << std::endl;
  std::cout << "needpose_img.x : " << needpose_img.x << std::endl;
  std::cout << "needpose_img.y : " << needpose_img.y << std::endl;*/
}

void ArmorDetectorNode::is_need_change()
{
  if(armorpose.x !=0 && armorpose.y !=0 && needpose.x !=0 && needpose.y !=0)
  {
    /*if(fabs(armorpose_img.x - needpose_img.x) > 0.001 && fabs(armorpose_img.x - needpose_img.x) < 20)
    {
        float needchangeyaw = acos(fabs(needpose.x* armorpose.x + armorpose.z * needpose.z) / (sqrtf(pow(needpose.x,2) + pow(needpose.z,2)) * sqrtf(pow(armorpose.x,2) + pow(armorpose.z,2))));
        //std::cout << "needchangeyaw: " << needchangeyaw << std::endl;
        std_msgs::msg::Float64 msg;
        if(armorpose_img.x > needpose_img.x)
        {
          msg.data = -needchangeyaw;
        }
        else{
          msg.data = needchangeyaw;
        }
        if(distance < 0.10)
      {
       msg.is_can_hit = true;
      }
      else
      {
        msg.is_can_hit = false;
        }
        changeyaw_pub->publish(msg);
    }*/

    // 三维距离测算
  // 得到装甲板位置到预测击打位置的距离
  float distance = fabs(powf((armorpose.x - needpose.x),2) + powf((armorpose.y - needpose.y),2) + powf((armorpose.z - needpose.z),2));
  //float distance = fabs(powf((armorpose.x - needpose.x),2) + powf((armorpose.z - needpose.z),2));
  std::cout << "distance: " << distance << std::endl;
  if(distance < 0.15)
  {
    float needchangeyaw = acos(fabs(needpose.x* armorpose.x + armorpose.z * needpose.z) / (sqrtf(pow(needpose.x,2) + pow(needpose.z,2)) * sqrtf(pow(armorpose.x,2) + pow(armorpose.z,2)))) * 57.3f;
    //std::cout << "needchangeyaw: " << needchangeyaw << std::endl;
    auto_aim_interfaces::msg::Bias msg;
    if(armorpose_img.x > needpose_img.x)
    {
      msg.needchangeyaw = -needchangeyaw;
    }
    else{
      msg.needchangeyaw = needchangeyaw;
    }
    if(distance < 0.10)
    {
      msg.is_can_hit = true;
    }
    else
    {
      msg.is_can_hit = false;
    }
    changeyaw_pub->publish(msg);
    //std::cout << "needchangeyaw: " << needchangeyaw << std::endl;
  }

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
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
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
    // Draw need armor center
    getarmorpose();
    if(armorpose_img.x != 0 && armorpose_img.y != 0)
    {
      cv::circle(img, armorpose_img, 5, cv::Scalar(0, 0, 255), 2);
    }
    if(prediction_armorpose_img.x != 0 && prediction_armorpose_img.y != 0)
    {
      cv::circle(img, prediction_armorpose_img, 5, cv::Scalar(255, 255, 255), 2);
    }
    if(needpose_img.x != 0 && needpose_img.y != 0)
    {
      is_need_change();
      cv::circle(img, needpose_img, 5, cv::Scalar(0, 255, 0), 2);
    }
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
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
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
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
