#include <iostream>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "image_processer.hpp"
#include "power_rune.hpp"
#include "blade.hpp"
#include "prediction.hpp"

namespace qianli_rm_rune
{

class RuneNode : public rclcpp::Node
{
public:
    RuneNode(const rclcpp::NodeOptions & options);

    void rune_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rune_pose_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rune_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    cv::Mat camera_matrix_;

    Configuration cfg_;
    ImageProcesser image_processer_;
    PowerRune power_rune_;
    ContourInfo contour_info_;
    Prediction predictor;
    vector<ContourInfo> contours_info_;

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

} // namespace qianli_rm_rune