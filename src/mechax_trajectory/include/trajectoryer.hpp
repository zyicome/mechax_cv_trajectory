#include <iostream>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "auto_aim_interfaces/msg/receive_serial.hpp"
#include "auto_aim_interfaces/msg/send_serial.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/bias.hpp"
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

const float g  = 9.8;
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

    int no_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0);

    int single_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    int single_resistance_model_two(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    int two_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    bool is_solvable(const float &object_x,const float &object_y,const float &object_z,const float &v0,float &alpha);

    int solve_trajectory();

    void test();

    void target_callback(const auto_aim_interfaces::msg::Target msg);

    void angle_callback(const auto_aim_interfaces::msg::ReceiveSerial msg);

    void power_rune_callback(geometry_msgs::msg::PointStamped msg);

    // parameters
    //------------------
    float v0; // m/s
    float angle_pitch;
    float angle_yaw;
    float distance;
    float fly_t; // m
    float y_bias;
    float z_bias;
    //------------------
    float now_pitch;
    float now_yaw;
    //------------------
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
    bool is_tracking;
    bool is_can_hit;
    string id;
    //------------------
    int latency_count;
    float all_latency;
    //------------------
    float motor_speed;
    float motor_bias_time;
    float serial_bias_time;
    float latency_bias_time;
    auto_aim_interfaces::msg::Bias bias_time_msg;
    //------------------
    float randa;
    bool is_hero;
    bool is_rune;
    //------------------
    // Subsciption
    //------------------
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::ReceiveSerial>::SharedPtr angle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr power_rune_sub_;
    //------------------
    // Publisher
    //------------------
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr maker_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Bias>::SharedPtr bias_time_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SendSerial>::SharedPtr result_pub_;
    //------------------
    //timer
    //------------------
    rclcpp::TimerBase::SharedPtr timer_;
    //------------------
};
