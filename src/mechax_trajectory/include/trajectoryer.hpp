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
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/float64.hpp>

using namespace std;

#define g 9.8
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

    void target_callback(const auto_aim_interfaces::msg::Target msg);

    void right_result_callback(const auto_aim_interfaces::msg::Inter msg);

    void front_assist_target_callback(const auto_aim_interfaces::msg::Target msg);

    void back_assist_target_callback(const auto_aim_interfaces::msg::Target msg);

    void angle_callback(const auto_aim_interfaces::msg::ReceiveSerial msg);

    void changeyaw_callback(const auto_aim_interfaces::msg::Bias msg);

    void needposeCallback(const geometry_msgs::msg::PointStamped needpose_ptr);

    void tf2_init();

    void get_need_pose(const geometry_msgs::msg::PointStamped pose);

    void get_needpose(const geometry_msgs::msg::PointStamped needpose);

    void get_armorpose(const geometry_msgs::msg::PointStamped armorpose);

    void get_bigyaw(const geometry_msgs::msg::PointStamped smallpose);

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
    float bias_t;
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
    //------------------
    float randa;
    bool is_hero;
    //------------------
    float needchangeyaw;
    bool is_left_can_hit;
    bool is_right_can_hit;
    //------------------
    // Subsciption
    //------------------
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr front_assist_target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr back_assist_target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Inter>::SharedPtr right_camera_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::ReceiveSerial>::SharedPtr angle_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Bias>::SharedPtr changeyaw_sub_;
    //------------------
    // Publisher
    //------------------
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr maker_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SendSerial>::SharedPtr result_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr needpose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr armorpose_pub_;
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
