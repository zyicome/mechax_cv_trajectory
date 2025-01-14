// 略
// 图片处理函数 灰度化二值化卷积核滤波 


#include "image_processer.hpp"

// ImageProcesser 构造函数，默认构造函数，无初始化操作
ImageProcesser::ImageProcesser()
{
    // 空的构造函数，用于创建一个空的 ImageProcesser 对象
}

// 生成一个自定义的圆形卷积核
// 参数：kernel_size - 卷积核的大小（边长）
// 返回值：返回一个大小为 kernel_size x kernel_size 的二值圆形卷积核
cv::Mat ImageProcesser::kernel(int kernel_size)
{
    // 创建一个大小为 kernel_size x kernel_size 的全零矩阵
    cv::Mat n = cv::Mat::zeros(kernel_size, kernel_size, CV_8UC1);
    
    // 在矩阵中心位置绘制一个圆形，圆的半径为 kernel_size / 2，像素值为 1
    cv::circle(n, cv::Point(kernel_size / 2, kernel_size / 2), kernel_size / 2, cv::Scalar(1), -1);
    
    // 返回生成的圆形卷积核
    return n;
}

// 将图像转换为灰度图，处理过程中提取图像的亮度通道并进行膨胀操作
// 参数：
//   image - 输入的 BGR 图像
//   kernel_size - 用于膨胀操作的卷积核大小
// 返回值：返回处理后的灰度图像
cv::Mat ImageProcesser::to_gray(cv::Mat image, int kernel_size)
{
    cv::Mat hsv, gray;
    
    // 将 BGR 图像转换为 HSV 颜色空间
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    
    // 将 HSV 图像中的 V（亮度）通道提取为灰度图
    gray = hsv.clone();  // 克隆 HSV 图像到 gray
    int channels[] = {2};  // 提取 V 通道，通道索引为 2
    cv::mixChannels(&hsv, 1, &gray, 1, channels, 1);  // 将 V 通道复制到 gray 图像
    
    // 对灰度图像进行膨胀操作，以增强亮度区域
    cv::Mat kernel = this->kernel(kernel_size);  // 生成卷积核
    cv::dilate(gray, gray, kernel);  // 使用卷积核对图像进行膨胀
    
    // 返回处理后的灰度图像
    return gray;
}

// 将图像转换为二值图像
// 参数：
//   image - 输入的灰度图像
//   binary_threshold - 二值化阈值
// 返回值：返回二值图像
cv::Mat ImageProcesser::to_binary(cv::Mat image, int binary_threshold)
{
    cv::Mat binary_image;
    
    // 使用给定的阈值对灰度图进行二值化处理
    cv::threshold(image, binary_image, binary_threshold, 255, cv::THRESH_BINARY);
    
    // 返回二值图像
    return binary_image;
}
