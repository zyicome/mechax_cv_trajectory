// rm_rune_node.cpp

#include "rm_rune_node.hpp"


namespace qianli_rm_rune
{
    RuneNode::RuneNode(const rclcpp::NodeOptions & options) : Node("rm_rune_node", options)
    {
        // 在控制台输出节点启动信息
        RCLCPP_INFO(get_logger(), "Hello, QianLi RM Rune!");

        // 调用神经网络识别
        const std::string& modelPath = "/home/qianli/buff25/mechax_cv_trajectory/src/rm_rune/model/rm_buff.onnx"; // 确保路径正确
        const std::string& onnx_provider = OnnxProviders::CPU; // "cpu";CPUExecutionProvider
        const std::string& onnx_logid = "yolov8_inference2";

        // 初始化模型
        model = std::make_unique<AutoBackendOnnx>(modelPath.c_str(), onnx_logid.c_str(), onnx_provider.c_str());
        RCLCPP_INFO(get_logger(), "model loaded");

        // 初始化相机内参矩阵为全零矩阵
        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

        // 创建发布者，用于发布3D预测位置（/rune/prediction）
        rune_pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rune/prediction", 10);

        // 创建订阅者，订阅图像原始数据（/image_raw）
        rune_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&RuneNode::rune_image_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅相机内参数据（/camera_info）
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
                cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                // 将相机参数存入相机矩阵
                camera_matrix_.at<double>(0,0) = camera_info->k[0];
                camera_matrix_.at<double>(0,2) = camera_info->k[2];
                camera_matrix_.at<double>(1,1) = camera_info->k[4];
                camera_matrix_.at<double>(1,2) = camera_info->k[5];
                camera_matrix_.at<double>(2,2) = 1.0;
                // 完成相机内参获取后，取消订阅
                cam_info_sub_.reset();
            });

        // 初始化tf2缓存和监听器，用于将预测的3D坐标转换到不同的坐标系
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // 创建一次性定时器，用于延迟初始化 image_transport
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 延迟时间，可以根据需要调整
            [this]() {
                try {
                    // 初始化 image_transport::ImageTransport，传递 shared_ptr<Node>
                    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
                    result_image_pub_ = it_->advertise("rune/result_image", 10);
                    RCLCPP_INFO(get_logger(), "Initialized image_transport publisher for /rune/result_image");

                    // 取消定时器，因为只需要初始化一次
                    init_timer_->cancel();
                }
                catch (const std::bad_weak_ptr & e) {
                    RCLCPP_ERROR(get_logger(), "Failed to initialize ImageTransport: %s", e.what());
                }
            }
        );
    }

    /*
    图像处理的回调函数，处理接收到的图像信息，进行图像处理、预测并发布3D点位信息。
    参数:
    - msg: sensor_msgs::msg::Image类型，表示接收到的图像消息。
    */
    void RuneNode::rune_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat rune_image;
        try
        {
            // 使用cv_bridge将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            rune_image = cv_ptr->image;
        }
        catch (cv_bridge::Exception & e)
        {
            // 如果图像转换失败，输出错误信息
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        float mask_threshold = 0.80f;
        float conf_threshold = 0.80f;
        float iou_threshold = 0.80f;
        int conversion_code = cv::COLOR_RGB2BGR;

        std::vector<cv::Scalar> posePalette = generateRandomColors(model->getNc(), model->getCh());
        std::unordered_map<int, std::string> names = model->getNames();

        // 转换颜色空间
        cv::cvtColor(rune_image, rune_image, conversion_code);

        // 进行推理
        std::vector<YoloResults> objs = model->predict_once(rune_image, conf_threshold, iou_threshold, mask_threshold, conversion_code);

        std::vector<std::vector<cv::Point>> contours;
        cv::Mat result_image; // 声明用于存储处理后图像的变量

        // 调用 plot_results 并传入 result_image
        contour_info_.plot_results(rune_image, objs, posePalette, names, rune_image.size(), contours, result_image);


        // 将处理后的图像转换为 ROS 消息并发布
        if (it_ && result_image_pub_)
        {
            auto result_msg = cv_bridge::CvImage(msg->header, "rgb8", result_image).toImageMsg();
            result_image_pub_.publish(result_msg); // 使用 image_transport 发布
            // RCLCPP_INFO(get_logger(), "Published result_image to /rune/result_image");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "ImageTransport not initialized yet. Skipping image publish.");
        }



        if (contours.empty()) {
            RCLCPP_INFO(get_logger(), "未检测到能量机关");
            return;
        }
        else {
            // contour_info_.setContour(contours[0]);
            // std::cout<< contours[0]<<std::endl;
            RCLCPP_INFO(get_logger(), "检测到rune" );
        }


        contours_info_.clear();// 清空上一帧的数据
        for(auto &contour : contours){
            contour_info_.setContour(contour);
            // std::cout<< contour << std::endl;
            if (contour_info_.index == 0 || contour_info_.index == 2){
                contours_info_.push_back(contour_info_);

            }
        }

        // 如果没有检测到合适的能量机关，输出调试信息
        if (contours_info_.empty()) {
            RCLCPP_INFO(get_logger(), "未检测到未击打的扇叶");
            return;
        }


        // 对 contours 按照 conf 字段进行从大到小排序
        std::sort(contours_info_.begin(), contours_info_.end(), [](const ContourInfo& a, const ContourInfo& b) {
            return a.conf > b.conf;  // 从大到小排序
        });


        // 使用Blade类对象对检测到的轮廓信息进行处理
        Blade blade(contours_info_[0], cfg_);

        // 更新预测器并进行预测
        
        predictor.update(blade.vector); // 计算目标与中心之间的向量
        // std::cout<< blade.center<<";"<<blade.circle_center <<std::endl; // left_up : 0,0
        // std::cout << blade.vector << std::endl;


        auto radian = predictor.predict(); // 返回预测的角度
        auto predicted_vector = power_rune_.predict(blade.vector, radian); // 返回预测的x，y坐标  1440;1080
        RCLCPP_INFO(get_logger(), "Predicted vector: x = %f, y = %f", predicted_vector.x, predicted_vector.y);

        // 如果没有相机信息，无法计算3D点位，输出错误信息
        if (cam_info_->k.empty()) {
            RCLCPP_ERROR(get_logger(), "没有相机信息，无法计算3D点位信息");
            return;
        }
        
        // 创建消息并填充预测的3D点位
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.frame_id = "camera_link";
        point_msg.header.stamp = msg->header.stamp;
        point_msg.point.x = 1;
        point_msg.point.y = -(static_cast<double>(predicted_vector.x) + static_cast<double>(blade.circle_center.x) - camera_matrix_.at<double>(0,2)) / camera_matrix_.at<double>(0,0);
        point_msg.point.z = -(static_cast<double>(predicted_vector.y) + static_cast<double>(blade.circle_center.y) - camera_matrix_.at<double>(1,2)) / camera_matrix_.at<double>(1,1);


        float z_constant = std::sqrt(predicted_vector.x * predicted_vector.x + predicted_vector.y * predicted_vector.y) * 0.7 * cfg_.distance_correction_ratio;
        // 根据相机参数和预测向量，计算3D距离
        float distance = (camera_matrix_.at<double>(0,0) + camera_matrix_.at<double>(1,1)) / 2 / z_constant;

        // std::cout << "z_constant:" << z_constant << std::endl;
        std::cout << "distance:" << distance << std::endl;


        // 将计算后的距离信息应用到3D点位
        point_msg.point.x *= distance;
        point_msg.point.y *= distance;
        point_msg.point.z *= distance;

        geometry_msgs::msg::PointStamped transformed_msg;
        try {
            transformed_msg.point = tf2_buffer_->transform(point_msg, "odom").point;
            transformed_msg.header.frame_id = "odom";
            transformed_msg.header.stamp = point_msg.header.stamp;
            rune_pose_pub_->publish(transformed_msg);
            RCLCPP_INFO(get_logger(), "Published rune prediction: x = %f, y = %f, z = %f", point_msg.point.x, point_msg.point.y, point_msg.point.z);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "无法将坐标从 camera_link 转换到 odom：%s", ex.what());
        }
    }
} // namespace qianli_rm_rune

#include "rclcpp_components/register_node_macro.hpp"

// 注册组件，确保该节点在库加载时可以被发现并使用"CPUExecutionProvider"
RCLCPP_COMPONENTS_REGISTER_NODE(qianli_rm_rune::RuneNode)