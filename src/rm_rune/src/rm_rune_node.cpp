#include "rm_rune_node.hpp"

namespace qianli_rm_rune
{
    RuneNode::RuneNode(const rclcpp::NodeOptions & options) : Node("rm_rune_node", options)
    {
        RCLCPP_INFO(get_logger(), "Hello, QianLi RM Rune!");

        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

        rune_pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rune/prediction", 10);

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

        // Subscriber with tf2 message_filter
        // tf2 relevant
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // Create the timer interface before call to waitForTransform,
        // to avoid a tf2_ros::CreateTimerInterfaceException exception
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
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
        rune_gray_image = image_processer_.to_gray(rune_image, cfg_.kernel_size);

        cv::Mat rune_binary_image;
        // 将灰度图像转换为二值图像
        rune_binary_image = image_processer_.to_binary(rune_gray_image, cfg_.binary_threshold);

        // 查找图像中的轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(rune_binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 计算轮廓信息
        for(auto &contour : contours)
        {
            contour_info_.setContour(contour);
            contours_info_.push_back(contour_info_);
        }

        // 根据面积和Hu矩过滤轮廓信息
        contours_info_ = power_rune_.filterByArea(contours_info_, cfg_.min_area);
        contours_info_ = power_rune_.filterByHu(contours_info_, cfg_.ref_hu, cfg_.hu_dev_threshold);
        if (contours_info_.empty()) {
            RCLCPP_DEBUG(get_logger(), "未检测到能量机关");
            return;
        }
        if (contours_info_.size() > 1) {
            RCLCPP_WARN(get_logger(), "检测到 %ld 个能量机关，仅使用最形状接近的一个", contours_info_.size());
        }

        Blade blade(contours_info_[0],cfg_);
        predictor.update(blade.vector);
        auto radian = predictor.predict();
        auto predicted_vector = power_rune_.predict(blade.vector, radian);

        if (cam_info_->k.empty()) {
            RCLCPP_ERROR(get_logger(), "没有相机信息，无法计算3D点位信息");
            return;
        }

        // Assuming cam_info_.k is a std::vector<float> with at least 6 elements
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.frame_id = "camera_link";
        point_msg.point.x = 1;
        point_msg.point.y = -(predicted_vector.x + blade.center.x - cam_info_->k[2]) / cam_info_->k[0];
        point_msg.point.z = -(predicted_vector.y + blade.center.y - cam_info_->k[5]) / cam_info_->k[4];

        float distance = (cam_info_->k[0] + cam_info_->k[4]) / 2 / std::sqrt(predicted_vector.x * predicted_vector.x + predicted_vector.y * predicted_vector.y) * 0.7 * cfg_.distance_correction_ratio;

        point_msg.point.x *= distance;
        point_msg.point.y *= distance;
        point_msg.point.z *= distance;

        geometry_msgs::msg::PointStamped transformed_msg;
        try {
            transformed_msg.point = tf2_buffer_->transform(point_msg, "odom").point;
            transformed_msg.header.frame_id = "odom";
            transformed_msg.header.stamp = point_msg.header.stamp;
            rune_pose_pub_->publish(transformed_msg);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "无法将坐标从 camera_link 转换到 odom：%s", ex.what());
        }
    }
} // namespace qianli_rm_rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(qianli_rm_rune::RuneNode)