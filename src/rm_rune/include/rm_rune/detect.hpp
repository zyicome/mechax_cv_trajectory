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

// rm_rune项目包含
#include "rm_rune/onnx_model_base.h"
#include "rm_rune/autobackend.h"
#include "rm_rune/augment.h"
#include "rm_rune/constants.h"
#include "rm_rune/common.h"

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
    ContourInfo(const std::vector<cv::Point>& contour);
    void setContour(const std::vector<cv::Point>& contour);

    // 计算面积
    double getArea();
    // 计算质心
    cv::Point2f getCentroid();
    // 获取主特征方向和特征值
    std::pair<cv::Vec2f, double> getEigenMax();
    // 获取偏度
    cv::Vec2f getSkewness();
    // 计算Hu矩的偏差
    float hu_moments_deviation(cv::Mat hu_moments, cv::Mat reference);

    // 绘制检测到的掩膜
    void plot_masks(cv::Mat img, std::vector<YoloResults>& result, std::vector<cv::Scalar> color,
                    std::unordered_map<int, std::string>& names);

    // 绘制检测到的关键点
    void plot_keypoints(cv::Mat& image, const std::vector<YoloResults>& results, const cv::Size& shape);

    // 绘制检测结果，包括边界框、关键点和掩膜
    void plot_results(cv::Mat img, std::vector<YoloResults>& results,
                    std::vector<cv::Scalar> color, std::unordered_map<int, std::string>& names,
                    const cv::Size& shape, std::vector<std::vector<cv::Point>>& contours,cv::Mat& result_image);


    // 成员变量
    std::vector<cv::Point> contour;
    cv::Moments moments;
    cv::Mat huMoments;
    cv::Point center;        // 正确的单一 Point 类型
    cv::Point circle_center; // 正确的单一 Point 类型
    int index;               // 正确的单一 int 类型
    int conf;                // 正确的单一 int 类型

};

// 函数声明（若需要可以移除，如果函数已内嵌到类中）
cv::Scalar generateRandomColor(int numChannels);
std::vector<cv::Scalar> generateRandomColors(int class_names_num, int numChannels);

#endif // DETECT_H
