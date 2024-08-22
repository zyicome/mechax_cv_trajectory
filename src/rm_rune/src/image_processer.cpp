#include "image_processer.hpp"

ImageProcesser::ImageProcesser()
{

}

cv::Mat ImageProcesser::kernel(int kernel_size)
{
    cv::Mat n = cv::Mat::zeros(kernel_size, kernel_size, CV_8UC1);
    cv::circle(n, cv::Point(kernel_size / 2, kernel_size / 2), kernel_size / 2, cv::Scalar(1), -1);
    return n;
}

cv::Mat ImageProcesser::to_gray(cv::Mat image, int kernel_size)
{
    cv::Mat hsv, gray;
    // Convert BGR to HSV
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // Extract the V channel
    gray = hsv.clone();
    int channels[] = {2};
    cv::mixChannels(&hsv, 1, &gray, 1, channels, 1);
    // Dilate the V channel
    cv::Mat kernel = this->kernel(kernel_size);
    cv::dilate(gray, gray, kernel);
    return gray;
}

cv::Mat ImageProcesser::to_binary(cv::Mat image,int binary_threshold)
{
    cv::Mat binary_image;
    cv::threshold(image, binary_image, binary_threshold, 255, cv::THRESH_BINARY);
    return binary_image;
}

