// 好像可以略
// 决断函数 进行判断和执行



#include "power_rune.hpp"

// PowerRune 构造函数
PowerRune::PowerRune()
{
    // 空的构造函数，没有初始化操作
}


// 根据给定的旋转角度和向量，预测旋转后的向量
// 参数：
//   vector - 输入的二维向量
//   radian - 旋转的角度（以弧度为单位）
// 返回值：返回旋转后的向量
cv::Point2f PowerRune::predict(const cv::Point2f& vector, float radian) {
    // 构建二维旋转矩阵
    cv::Matx22f rotationMatrix(std::cos(radian), -std::sin(radian),
                               std::sin(radian), std::cos(radian));
    
    // 使用旋转矩阵对输入向量进行旋转
    cv::Point2f rotatedVector = rotationMatrix * cv::Vec2f(vector.x, vector.y);
    
    // 返回旋转后的向量
    return rotatedVector;
}

std::vector<ContourInfo> PowerRune::sortByconf(std::vector<ContourInfo>& contours)
{
    // 对 contours 按照 conf 字段进行从大到小排序
    std::sort(contours.begin(), contours.end(), [](const ContourInfo& a, const ContourInfo& b) {
        return a.conf > b.conf;  // 从大到小排序
    });

    // 直接返回排序后的 contours 向量
    return contours;
}