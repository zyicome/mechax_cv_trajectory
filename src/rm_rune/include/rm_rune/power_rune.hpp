#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "contour_info.hpp" // Assume this is the header file for ContourInfo class
#include "configuration.hpp" // Assume this contains cfg with configurations like ref_hu, hu_dev_threshold, min_area

class PowerRune
{
public:
    PowerRune();

    float huCondition(ContourInfo& contour,const cv::Mat& ref_hu);
    bool areaCondition(ContourInfo& contour, const float& min_area);
    std::vector<ContourInfo> filterByArea(std::vector<ContourInfo>& contours, float minArea);
    std::vector<ContourInfo> filterByHu(std::vector<ContourInfo>& contours, const cv::Mat& refHu, float huDevThreshold);
    cv::Point2f predict(const cv::Point2f& vector, float radian);
};