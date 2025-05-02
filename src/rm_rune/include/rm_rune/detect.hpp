#ifndef DETECT_H
#define DETECT_H

#pragma once

// 标准库包含
#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iomanip>
#include <random>
#include <filesystem>

// OpenCV库包含
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"


// 命名空间别名
namespace fs = std::filesystem;

// 定义骨架和颜色映射
extern std::vector<std::vector<int>> skeleton;
extern std::vector<cv::Scalar> posePalette;
extern std::vector<int> limbColorIndices;
extern std::vector<int> kptColorIndices;

// ContourInfo类声明
class ContourInfo
{
public:
    ContourInfo();
    ContourInfo(const std::vector<cv::Point2f>& contour);
    void setContour(const std::vector<cv::Point2f>& contour);


    // 成员变量
    std::vector<cv::Point2f> contour;
    cv::Moments moments;
    cv::Mat huMoments;
    cv::Point2f center;        
    cv::Point2f circle_center; 
    int index;              
    int conf;                

};

#endif // DETECT_H
