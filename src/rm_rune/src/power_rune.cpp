// 好像可以略
// 决断函数 进行判断和执行



#include "power_rune.hpp"

// PowerRune 构造函数
PowerRune::PowerRune()
{
    // 空的构造函数，没有初始化操作
}

// 计算 Hu 矩的偏差条件
// 参数：
//   contour - 轮廓信息对象，包含轮廓的 Hu 矩
//   ref_hu - 参考 Hu 矩
// 返回值：返回 Hu 矩的偏差值
float PowerRune::huCondition(ContourInfo& contour, const cv::Mat& ref_hu)
{
    // 获取轮廓的 Hu 矩
    cv::Mat hu_moments = contour.huMoments;
    
    // 计算 Hu 矩与参考 Hu 矩之间的偏差
    float hu_dev = contour.hu_moments_deviation(contour.huMoments, ref_hu);
    
    // 返回偏差值
    return hu_dev;
}

// 判断轮廓的面积是否满足最小面积条件
// 参数：
//   contour - 轮廓信息对象
//   min_area - 最小允许面积
// 返回值：返回布尔值，表示轮廓面积是否大于最小面积
bool PowerRune::areaCondition(ContourInfo& contour, const float& min_area) 
{
    // 获取轮廓的面积
    float area = contour.getArea();
    
    // 判断面积是否大于最小面积条件
    bool result = area > min_area;
    
    // 返回判断结果
    return result;
}

// 根据面积过滤轮廓
// 参数：
//   contours - 轮廓信息的列表
//   minArea - 最小面积条件
// 返回值：返回过滤后的轮廓列表（面积大于 minArea 的轮廓）
std::vector<ContourInfo> PowerRune::filterByArea(std::vector<ContourInfo>& contours, float minArea) {
    std::vector<ContourInfo> filtered;

    // 使用 std::copy_if 复制符合面积条件的轮廓
    std::copy_if(contours.begin(), contours.end(), std::back_inserter(filtered), [this, minArea](ContourInfo& contour) {
        bool result = this->areaCondition(contour, minArea);  // 判断面积条件
        return result;  // 返回符合条件的轮廓
    });

    // 返回过滤后的轮廓
    return filtered;
}

// 根据 Hu 矩的偏差值过滤轮廓
// 参数：
//   contours - 轮廓信息的列表
//   refHu - 参考 Hu 矩
//   huDevThreshold - Hu 矩偏差阈值
// 返回值：返回按照偏差值排序的轮廓列表
std::vector<ContourInfo> PowerRune::filterByHu(std::vector<ContourInfo>& contours, const cv::Mat& refHu, float huDevThreshold) {
    std::vector<std::pair<ContourInfo, double>> filteredWithDeviation;

    // 遍历所有轮廓
    for (auto& contour : contours) {
        cv::Mat hu_moments = contour.huMoments;
        
        // 计算每个轮廓的 Hu 矩偏差
        double deviation = contour.hu_moments_deviation(hu_moments, refHu);
        
        // 如果偏差小于阈值，将轮廓及其偏差存储
        if (deviation <= huDevThreshold) {
            filteredWithDeviation.emplace_back(contour, deviation);
        }
    }

    // 按照 Hu 偏差值从小到大对轮廓进行排序
    std::sort(filteredWithDeviation.begin(), filteredWithDeviation.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

    // 将排序后的轮廓提取出来
    std::vector<ContourInfo> sortedContours;
    for (const auto& pair : filteredWithDeviation) {
        sortedContours.push_back(pair.first);  // 提取轮廓对象
    }

    // 返回排序后的轮廓列表
    return sortedContours;
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