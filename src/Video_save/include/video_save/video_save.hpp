#include <opencv4/opencv2/opencv.hpp>
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

#include "std_msgs/msg/int8.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>


class VideoSave : public rclcpp::Node
{
    public:
        VideoSave();
        ~VideoSave();
        
        cv::Mat sub_video_frame;
        void video_callback(sensor_msgs::msg::CompressedImage msg);

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_sub_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr is_begin_to_save_sub_;
        cv::VideoWriter out1_;
        rclcpp::Time last_time1;
        bool out1_is_opened = false;
        int count1;

        bool is_begin_to_save;
        std::string video_path;
};