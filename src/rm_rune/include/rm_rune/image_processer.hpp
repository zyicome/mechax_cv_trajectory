#pragma once
#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class ImageProcesser
{
public:
    ImageProcesser();

    cv::Mat kernel(int kernel_size);
    cv::Mat to_gray(cv::Mat image, int kernel_size);
    cv::Mat to_binary(cv::Mat image ,int binary_threshold);
};