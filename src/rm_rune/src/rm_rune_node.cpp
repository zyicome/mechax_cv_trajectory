#include "rm_rune_node.hpp"

namespace qianli_rm_rune
{
    RuneNode::RuneNode(const rclcpp::NodeOptions & options) : Node("rm_rune_node", options)
    {
        RCLCPP_INFO(get_logger(), "Hello, QianLi RM Rune!");

        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

        rune_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rune/prediction", 10);

        rune_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(), std::bind(&RuneNode::rune_image_callback, this, std::placeholders::_1));

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            camera_matrix_.at<double>(0,0) = camera_info->k[0];
            camera_matrix_.at<double>(0,2) = camera_info->k[2];
            camera_matrix_.at<double>(1,1) = camera_info->k[4];
            camera_matrix_.at<double>(1,2) = camera_info->k[5];
            camera_matrix_.at<double>(2,2) = 1.0;
            cam_info_sub_.reset();
        });
    }

    //处理图像消息，进行图像处理和预测，并发布3D点位信息。

    //参数:
    //- msg: sensor_msgs::msg::Image类型，表示接收到的图像消息。
    //- publisher: Publisher类型，用于发布处理后的3D点位信息。
    void RuneNode::rune_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat rune_image;
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            rune_image = cv_ptr->image;
        }
        catch (cv_bridge::Exception & e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat rune_gray_image;
        // 将图像转换为灰度图像
        cv::cvtColor(rune_image, rune_gray_image, cv::COLOR_BGR2GRAY);

        cv::Mat rune_binary_image;
        // 对灰度图像进行二值化处理
        cv::threshold(rune_gray_image, rune_binary_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // 查找图像中的轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(rune_binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 计算轮廓信息
    }
} // namespace qianli_rm_rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(qianli_rm_rune::RuneNode)