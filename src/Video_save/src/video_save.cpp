#include "video_save/video_save.hpp"


VideoSave::VideoSave() : Node("video_save")
{
    RCLCPP_INFO(get_logger(), "Start video_save node sucessfully!");
    is_begin_to_save = false;
    last_time1 = this->now();
    count1 = 0;
    img_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/detector/result_img/compressed",1,std::bind(&VideoSave::video_callback,this,std::placeholders::_1));

    std::string pkg_path = "/home/qianli/apex/src/Video_save";
    video_path = pkg_path + "/video_/";
}
VideoSave::~VideoSave()
{
    out1_.release();
}

void VideoSave::video_callback(sensor_msgs::msg::CompressedImage msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    this->sub_video_frame = cv_ptr->image;
    rclcpp::Time now = this->now();
    double fps = 1.0 / (now - this->last_time1).seconds();
    last_time1 = now;
    if(count1 < 100)
    {
        count1++;
        return;
    }
    RCLCPP_INFO(this->get_logger(), "fps: %f", fps);
    if (!out1_is_opened)
    {
        RCLCPP_INFO(get_logger(), "began to save video!");
        std::time_t now = std::time(0);
        char buffer[100];
        std::strftime(buffer, 100, "%Y-%m-%d-%H-%M-%S", std::localtime(&now));
        std::string buffer_str(buffer); // 将字符数组转换为字符串
        out1_.open(video_path+buffer_str+".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 24, this->sub_video_frame.size());
        out1_is_opened = true;
    }
    out1_.set(cv::CAP_PROP_FPS, fps);
    out1_.write(this->sub_video_frame);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSave>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include "video_save/video_save.hpp"


VideoSave::VideoSave() : Node("video_save")
{
    RCLCPP_INFO(get_logger(), "Start video_save node sucessfully!");
    is_begin_to_save = false;
    last_time1 = this->now();
    count1 = 0;
    img_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/detector/result_img/compressed",1,std::bind(&VideoSave::video_callback,this,std::placeholders::_1));

    std::string pkg_path = "/home/qianli/mechax/src/Video_save";
    video_path = pkg_path + "/video_/";
}
VideoSave::~VideoSave()
{
    out1_.release();
}

void VideoSave::video_callback(sensor_msgs::msg::CompressedImage msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    this->sub_video_frame = cv_ptr->image;
    rclcpp::Time now = this->now();
    double fps = 1.0 / (now - this->last_time1).seconds();
    last_time1 = now;
    if(count1 < 100)
    {
        count1++;
        return;
    }
    RCLCPP_INFO(this->get_logger(), "fps: %f", fps);
    if (!out1_is_opened)
    {
        RCLCPP_INFO(get_logger(), "began to save video!");
        std::time_t now = std::time(0);
        char buffer[100];
        std::strftime(buffer, 100, "%Y-%m-%d-%H-%M-%S", std::localtime(&now));
        std::string buffer_str(buffer); // 将字符数组转换为字符串
        out1_.open(video_path+buffer_str+".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 24, this->sub_video_frame.size());
        out1_is_opened = true;
    }
    out1_.set(cv::CAP_PROP_FPS, fps);
    out1_.write(this->sub_video_frame);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSave>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}