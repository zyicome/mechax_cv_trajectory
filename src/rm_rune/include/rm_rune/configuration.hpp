#pragma once
#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class Configuration
{
public:
    bool enable; // 是否启用节点
    float binary_threshold;
    int kernel_size; // 形态学的核大小
    float min_area; // 轮廓最小面积
    cv::Mat ref_hu; // 参考HU值
    float hu_dev_threshold; // HU值误差阈值
    float target_relative_position; // 目标相对位置
    float center_relative_position; // 中心相对位置
    float refit_delay_sec; // 重新拟合延迟
    float hit_delay_sec; // 命中延迟
    float distance_correction_ratio; // 距离校正比例

    Configuration();

    void init();
};