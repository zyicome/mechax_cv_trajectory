#include <iostream>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace qianli_rm_rune
{

class RuneNode : public rclcpp::Node
{
public:
    RuneNode(const rclcpp::NodeOptions & options);

    void rune_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rune_pose_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rune_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    cv::Mat camera_matrix_;
};

} // namespace qianli_rm_rune