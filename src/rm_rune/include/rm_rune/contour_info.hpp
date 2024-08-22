#pragma once
#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class ContourInfo
{
public:
    ContourInfo();
    ContourInfo(const std::vector<cv::Point>& contour);
    void setContour(const std::vector<cv::Point>& contour);

    double getArea();
    cv::Point2f getCentroid();
    std::pair<cv::Vec2f, double> getEigenMax();
    cv::Vec2f getSkewness();
    float hu_moments_deviation(cv::Mat hu_moments, cv::Mat reference);

    std::vector<cv::Point> contour;
    cv::Moments moments;
    cv::Mat huMoments;
};