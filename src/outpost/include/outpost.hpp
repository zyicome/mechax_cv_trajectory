#include <iostream>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "auto_aim_interfaces/msg/receive_serial.hpp"
#include "auto_aim_interfaces/msg/send_serial.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/bias.hpp"
#include "auto_aim_interfaces/msg/inter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/points.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>

using namespace std;

struct Mypoint
{
  double x;
  double y;
  double z;
  double yaw;
  double timetolast;
};

class Outpost : public::rclcpp::Node
{
public:
    // hanshu
    Outpost();

    void tf2_init();

    void left_armors_callback(const auto_aim_interfaces::msg::Armors msg);

    double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

    double get_distance(cv::Point3d point_one,cv::Point3d point_two);

    double get_distance_2f(cv::Point2f point_one,cv::Point2f point_two);

    void updateCallback(const std_msgs::msg::Int8 msg);

    bool fittingToCircle_needtoupdate(std::vector<Mypoint> &new_armor_points,std::vector<Mypoint> &save_armor_points);

    //------------------
    std::vector<Mypoint> armor_points;
    std::vector<Mypoint> save_armor_points;
    //------------------
    //std::chrono::time_point<std::chrono::system_clock,chrono::seconds> start_time;
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point end_time;
    double start;
    double end;
    double lastyaw;
    // Subsciption
    //------------------
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr left_armors_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr update_sub_;
    //------------------
    // Publisher
    //------------------
    visualization_msgs::msg::Marker point_marker_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Points>::SharedPtr outpost_points_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Point>::SharedPtr outpost_point_pub_;
    //------------------
    visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    //------------------
    // Broadcast tf from odom to gimbal_link
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    double timestamp_offset_ = 0;
};
