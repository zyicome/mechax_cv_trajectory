// rm_rune_node.cpp

#include "rm_rune_node.hpp"


namespace qianli_rm_rune
{
    RuneNode::RuneNode(const rclcpp::NodeOptions & options) : Node("rm_rune_node", options),
    frame_count_(0),
    last_time_(this->now()),
    inference(  // ✅ 在初始化列表中构造对象
        "/home/fyk/fyk/mechax_cv_trajectroy_rune_openvino/src/rm_rune/model/buff480.onnx", // 模型路径
        cv::Size(480, 480),          // 输入尺寸
        0.5f,   // 置信度阈值
        0.5f   // NMS阈值
    ) 
    
    {
        // 在控制台输出节点启动信息
        RCLCPP_INFO(get_logger(), "Hello, QianLi RM Rune!");


                // 初始化相机内参矩阵
        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

        // 订阅相机内参
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
                RCLCPP_INFO(this->get_logger(), "Received camera info!");
                RCLCPP_INFO(this->get_logger(), "K matrix: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                        camera_info->k[0], camera_info->k[1], camera_info->k[2],
                        camera_info->k[3], camera_info->k[4], camera_info->k[5],
                        camera_info->k[6], camera_info->k[7], camera_info->k[8]);

                camera_matrix_.at<double>(0, 0) = camera_info->k[0];  // fx
                camera_matrix_.at<double>(0, 2) = camera_info->k[2];   // cx
                camera_matrix_.at<double>(1, 1) = camera_info->k[4];   // fy
                camera_matrix_.at<double>(1, 2) = camera_info->k[5];    // cy
                camera_matrix_.at<double>(2, 2) = 1.0;
                cam_info_sub_.reset();
            });

        // 调用神经网络识别
        // const std::string& onnx_provider = OnnxProviders::CPU; // "cpu";CPUExecutionProvider
        // const std::string& onnx_logid = "yolov8_inference2";

        // // 初始化模型
        // model = std::make_unique<AutoBackendOnnx>(modelPath.c_str(), onnx_logid.c_str(), onnx_provider.c_str());
        // const std::string& modelPath = "/home/fyk/fyk/mechax_cv_trajectory_rune/src/rm_rune/model/buff320.onnx"; // 确保路径正确
        // const float confidence_threshold = 0.5;
        // const float NMS_threshold = 0.5;
        // inference(modelPath, cv::Size(640, 640), confidence_threshold, NMS_threshold);
        RCLCPP_INFO(get_logger(), "model loaded");


        // 创建发布者，用于发布3D预测位置（/rune/prediction）
        rune_pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rune/prediction", 10);

        // 创建订阅者，订阅图像原始数据（/image_raw）
        rune_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&RuneNode::rune_image_callback, this, std::placeholders::_1));



        // 初始化tf2缓存和监听器，用于将预测的3D坐标转换到不同的坐标系
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);




        //debug
        // 创建一次性定时器，用于延迟初始化 image_transport
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 延迟时间，可以根据需要调整
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
        // 新增帧率计算逻辑
        auto current_time = this->now();
        frame_count_++;
        double elapsed = (current_time - last_time_).seconds();
        
        if (elapsed >= 1.0) {
            double fps = frame_count_ / elapsed;
            RCLCPP_INFO(get_logger(), "[FPS] Current: %.2f", fps);
            frame_count_ = 0;
            last_time_ = current_time;
        }


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


        inference.RunInference(rune_image);


        std::vector<std::vector<cv::Point2f>> contours;
        cv::Mat result_image; // 声明用于存储处理后图像的变量

        // result_image = rune_image.clone(); // 克隆原始图像以便后续处理
        contours = inference.contours; // 获取检测到的轮廓




        if (contours.empty()) {
            RCLCPP_INFO(get_logger(), "未检测到能量机关");
            return;
        }
        else {
            // RCLCPP_INFO(get_logger(), "检测到rune" );
            // std::cout << contours[0] << std::endl;
        }


        std::vector<cv::Point2f> rune_imagePoints = {
            {contours[0][1].x,contours[0][1].y },  // p1
            {contours[0][2].x,contours[0][2].y},  // p2
            {contours[0][4].x,contours[0][4].y},  // p4
            {contours[0][5].x,contours[0][5].y}   // p5
        };

        // std::cout << rune_imagePoints <<std::endl;


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


        //debug predict
        auto radian = predictor.predict(); // 返回预测的角度
        auto predicted_vector = power_rune_.predict(blade.vector, radian); // 返回预测的x，y坐标  1440;1080
        auto predicted_vectorP1 = power_rune_.predict(rune_imagePoints[0]-blade.circle_center, radian); 
        auto predicted_vectorP2 = power_rune_.predict(rune_imagePoints[1]-blade.circle_center, radian);
        auto predicted_vectorP3 = power_rune_.predict(rune_imagePoints[2]-blade.circle_center, radian);
        auto predicted_vectorP4 = power_rune_.predict(rune_imagePoints[3]-blade.circle_center, radian);
        // std::cout << "radian: " << radian << std::endl;
        // RCLCPP_INFO(get_logger(), "Predicted vector: x = %f, y = %f", predicted_vector.x, predicted_vector.y);

        cv::Point2f predicted_point;
        predicted_point.x = predicted_vector.x + blade.circle_center.x;
        predicted_point.y = predicted_vector.y + blade.circle_center.y;
        rune_imagePoints[0].x = predicted_vectorP1.x + blade.circle_center.x;
        rune_imagePoints[0].y = predicted_vectorP1.y + blade.circle_center.y;
        rune_imagePoints[1].x = predicted_vectorP2.x + blade.circle_center.x;
        rune_imagePoints[1].y = predicted_vectorP2.y + blade.circle_center.y;
        rune_imagePoints[2].x = predicted_vectorP3.x + blade.circle_center.x;
        rune_imagePoints[2].y = predicted_vectorP3.y + blade.circle_center.y;
        rune_imagePoints[3].x = predicted_vectorP4.x + blade.circle_center.x;
        rune_imagePoints[3].y = predicted_vectorP4.y + blade.circle_center.y;
        

        // 将处理后的图像转换为 ROS 消息并发布
        if (it_ && result_image_pub_)
        {   
            // debug
            cv::circle(rune_image, predicted_point, 10, cv::Scalar(0, 255, 0), -1); // 绘制预测点
            cv::circle(rune_image, rune_imagePoints[0], 5, cv::Scalar(255, 0, 0), -1); 
            cv::circle(rune_image, rune_imagePoints[1], 5, cv::Scalar(255, 0, 0), -1);
            cv::circle(rune_image, rune_imagePoints[2], 5, cv::Scalar(255, 0, 0), -1);
            cv::circle(rune_image, rune_imagePoints[3], 5, cv::Scalar(255, 0, 0), -1);
            auto result_msg = cv_bridge::CvImage(msg->header, "rgb8", rune_image).toImageMsg();
            result_image_pub_.publish(result_msg); // 使用 image_transport 发布
            // RCLCPP_INFO(get_logger(), "Published result_image to /rune/result_image");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "ImageTransport not initialized yet. Skipping image publish.");
        }



        // 如果没有相机信息，无法计算3D点位，输出错误信息
        if (cam_info_->k.empty()) {
            RCLCPP_ERROR(get_logger(), "没有相机信息，无法计算3D点位信息");
            return;
        }
        

        std::vector<cv::Point3f> rune_objectPoints = {
            {0, 16, 5},
            {0, 16, -5},
            {0, -16, -5},
            {0, -16, 5}
        };


        cv::Mat rvec, tvec;
        cv::Mat distCoeffs;  // 空的
        // Solve PnP
        bool success = cv::solvePnP(rune_objectPoints, rune_imagePoints, camera_matrix_, distCoeffs, rvec, tvec);

        if (success) {
            // std::cout << "Rotation Vector (rvec): " << rvec << std::endl;
            // std::cout << "Translation Vector (tvec): " << tvec << std::endl;
        } else {
            std::cerr << "PnP solving failed!" << std::endl;
        }

        // 创建消息并填充预测的3D点位
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.frame_id = "camera_optical_frame";
        point_msg.header.stamp = msg->header.stamp;
        point_msg.point.x = tvec.at<double>(0, 0)/100;
        point_msg.point.y = tvec.at<double>(1, 0)/100;
        point_msg.point.z = tvec.at<double>(2, 0)/100;



        geometry_msgs::msg::PointStamped transformed_msg;
        try {
            transformed_msg.point = tf2_buffer_->transform(point_msg, "odom").point;
            transformed_msg.header.frame_id = "odom";
            transformed_msg.header.stamp = point_msg.header.stamp;
            rune_pose_pub_->publish(transformed_msg);
            // RCLCPP_INFO(get_logger(), "Published rune prediction: x = %f, y = %f, z = %f", point_msg.point.x, point_msg.point.y, point_msg.point.z);
            // RCLCPP_INFO(get_logger(), "Published rune prediction: x = %f, y = %f, z = %f", transformed_msg.point.x, transformed_msg.point.y, transformed_msg.point.z);
        } catch (tf2::TransformException& ex) {
            // RCLCPP_WARN(get_logger(), "无法将坐标从 camera_link 转换到 odom：%s", ex.what());
        }
    }
} // namespace qianli_rm_rune

#include "rclcpp_components/register_node_macro.hpp"

// 注册组件，确保该节点在库加载时可以被发现并使用"CPUExecutionProvider"
RCLCPP_COMPONENTS_REGISTER_NODE(qianli_rm_rune::RuneNode)