#include <iostream>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "auto_aim_interfaces/msg/receive_serial.hpp"
#include "auto_aim_interfaces/msg/send_serial.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/bias.hpp"
#include "auto_aim_interfaces/msg/inter.hpp"
#include "auto_aim_interfaces/msg/points.hpp"
#include "auto_aim_interfaces/msg/point.hpp"
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

#include <opencv2/opencv.hpp>

#include <chrono>

using namespace std;

const float g = 9.8;
#define ARMOR_NUM_BALANCE 2;
#define ARMOR_NUM_OUTPOST 3;

struct result
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
};

struct Mypoint
{
  double x;
  double y;
  double z;
  double yaw;
  double timetolast;
};

struct Circle
{
    float radius;
    double z;
    cv::Point2f circle_center;
    std::vector<Mypoint> armor_points;
};

class Trajectoryer : public::rclcpp::Node
{
public:
    // hanshu
    Trajectoryer();

    void parameters_init();

    int no_resistance_model(float &angle_pitch,const float &object_x,const float &object_y,const float &object_z,const float &v0);

    int single_resistance_model(float &angle_pitch,const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    int single_resistance_model_two(float &angle_pitch,const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    int two_resistance_model(float &angle_pitch, const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    bool is_solvable(const float &object_x,const float &object_y,const float &object_z,const float &v0,float &alpha);

    int solve_trajectory();

    void test();

    void left_camera_target_callback(const auto_aim_interfaces::msg::Target msg);

    void right_result_callback(const auto_aim_interfaces::msg::Inter msg);

    void front_assist_target_callback(const auto_aim_interfaces::msg::Target msg);

    void back_assist_target_callback(const auto_aim_interfaces::msg::Target msg);

    void angle_callback(const auto_aim_interfaces::msg::ReceiveSerial msg);

    void outpostPointsCallback(const auto_aim_interfaces::msg::Points msg);

    void outpostPointCallback(const auto_aim_interfaces::msg::Point msg);

    void usingcircle_outpostPointsCallback(const auto_aim_interfaces::msg::Points msg);

    void usingcircle_outpostPointCallback(const auto_aim_interfaces::msg::Point msg);

    void fittingToCircle(std::vector<Mypoint> &armor_points,Circle circle);

    void get_circle_xy(double &time, cv::Point3d &start_point,cv::Point3d &hit_point);
    
    void tf2_init();

    void get_bigyaw(const geometry_msgs::msg::PointStamped smallpose);
    
    double get_distance(cv::Point3d point_one,cv::Point3d point_two);

    void assist_get_yaw_bigyaw(float &left_angle_pitch, float &left_angle_yaw,float &right_angle_pitch,float &right_angle_yaw,float &angle_bigyaw,const geometry_msgs::msg::PointStamped &pose);

    // parameters
    //------------------
    float v0; // m/s
    float left_angle_pitch;
    float left_angle_yaw;
    float right_angle_pitch;
    float right_angle_yaw;
    float angle_bigyaw;
    float fly_t; // m
    float y_bias;
    float z_bias;
    //------------------
    float now_pitch;
    float now_yaw;
    //------------------
    geometry_msgs::msg::PointStamped pose;
    geometry_msgs::msg::PointStamped armorpose;
    geometry_msgs::msg::PointStamped needpose;
    //------------------
    int armor_num;
    // 敌方云台中心在我方云台中心坐标系下的坐标
    float yaw;
    float v_yaw;
    float vx;
    float ros_x;
    float vy;
    float ros_y;
    float vz;
    float ros_z;
    float r_1;
    float r_2;
    float dz;
    bool is_left_tracking;
    bool is_right_tracking;
    bool is_assist_tracking;
    float distance;
    string id;
    //------------------
    float assist_x;
    float assist_y;
    float assist_z;
    float stay_x;
    float stay_y;
    float stay_z;
    //------------------
    float randa;
    bool is_hero;
    //------------------
    bool is_left_can_hit;
    bool is_right_can_hit;
    //------------------
    int latency_count;
    float all_latency;
    //------------------
    //------------------
    float motor_speed;
    float motor_bias_time;
    float serial_bias_time;
    float latency_bias_time;
    auto_aim_interfaces::msg::Bias bias_time_msg;
    //------------------
    //------------------
    bool is_matched;
    bool is_start;
    float outpost_bias_t;
    std::vector<Mypoint> armor_points;
    int point_number;
    Circle circle;
    float outpost_radius;
    float outpost_fly_time;
    //------------------
    bool is_assisting;
    double start;
    double end;
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point update_time;
    std::chrono::steady_clock::time_point end_time;
    std::chrono::steady_clock::time_point next_time;
    std::chrono::steady_clock::time_point stay_start_time;
    std::chrono::steady_clock::time_point assist_time;
    // Subsciption
    //------------------
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr left_camera_target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr front_assist_target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr back_assist_target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Inter>::SharedPtr right_camera_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::ReceiveSerial>::SharedPtr angle_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Points>::SharedPtr outpost_points_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Point>::SharedPtr outpost_point_sub_;
    //------------------
    // Publisher
    //------------------
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr maker_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SendSerial>::SharedPtr result_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Inter>::SharedPtr left_camera_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr update_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Bias>::SharedPtr bias_time_pub_;
    //------------------
    //timer
    //------------------
    rclcpp::TimerBase::SharedPtr timer_;
    //------------------
    // Broadcast tf from odom to gimbal_link
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    string needpose_target_frame_;
    string armorpose_target_frame_;
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
