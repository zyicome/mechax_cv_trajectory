//预测主函数

#include <random>

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>
#include "rm_rune/detect.hpp"


namespace fs = std::filesystem;

ContourInfo::ContourInfo()
{
    // 空的构造函数，用于创建一个空对象
}

ContourInfo::ContourInfo(const std::vector<cv::Point2f>& contour)//contour改为存储标签置信度+五个关键点
{
    // 分别累加 x 和 y 坐标
    int sum_x = contour[1].x + contour[2].x + contour[5].x + contour[4].x;
    int sum_y = contour[1].y + contour[2].y + contour[5].y + contour[4].y;

    // 计算平均值
    this->center = cv::Point2f(sum_x / 4, sum_y / 4);  // 使用 cv::Point 初始化中心点
    this->circle_center = cv::Point2f(contour[3].x, contour[3].y);  // 使用 cv::Point 初始化圆心
    this->index = contour[0].x;  // 初始化索引
    this->conf = contour[0].y;  // 初始化置信度
}

// 设置轮廓并更新轮廓相关信息
void ContourInfo::setContour(const std::vector<cv::Point2f>& contour)
{
    // 分别累加 x 和 y 坐标
    int sum_x = contour[1].x + contour[2].x + contour[5].x + contour[4].x;
    int sum_y = contour[1].y + contour[2].y + contour[5].y + contour[4].y;

    // 计算平均值
    this->center = cv::Point2f(sum_x / 4, sum_y / 4);  // 使用 cv::Point 更新中心点
    this->circle_center = cv::Point2f(contour[3].x, contour[3].y);  // 使用 cv::Point 更新圆心
    this->index = contour[0].x;  // 更新索引
    this->conf = contour[0].y;  // 更新置信度
}
