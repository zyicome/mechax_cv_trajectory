#pragma once
#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

// #include "contour_info.hpp"
#include "detect.hpp"
#include "configuration.hpp"

/*@ivar contour: 输入的轮廓信息。
@ivar target: 根据特征向量和配置中的目标相对位置计算的目标位置。
@ivar center: 根据特征向量和配置中的中心相对位置计算的中心位置。
@ivar vector: 根据目标位置和中心位置计算出的向量。
@param contour: ContourInfo类型的实例。*/
class Blade
{
public:
    Blade(ContourInfo& contour,Configuration& cfg);

    ContourInfo contour;
    cv::Point2f target;
    cv::Point2f center;
    cv::Point2f circle_center;
    cv::Point2f vector;
};