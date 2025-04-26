#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "detect.hpp" // Assume this is the header file for ContourInfo class
#include "configuration.hpp" // Assume this contains cfg with configurations like ref_hu, hu_dev_threshold, min_area

class PowerRune
{
public:
    PowerRune();
    std::vector<ContourInfo> sortByconf(std::vector<ContourInfo>& contours);
    cv::Point2f predict(const cv::Point2f& vector, float radian);
};