#include "power_rune.hpp"

PowerRune::PowerRune()
{
    // Constructor
}

float PowerRune::huCondition(ContourInfo& contour,const cv::Mat& ref_hu)
{
    cv::Mat hu_moments = contour.huMoments;
    float hu_dev = contour.hu_moments_deviation(contour.huMoments, ref_hu);
    return hu_dev;
}


bool PowerRune::areaCondition(ContourInfo& contour, const float& min_area) 
{
    float area = contour.getArea();
    bool result = area > min_area;
    return result;
}

std::vector<ContourInfo> PowerRune::filterByArea(std::vector<ContourInfo>& contours, float minArea) {
    std::vector<ContourInfo> filtered;
    std::copy_if(contours.begin(), contours.end(), std::back_inserter(filtered), [this,minArea](ContourInfo& contour) {
        bool result = this->areaCondition(contour, minArea);
        return result;
    });
    return filtered;
}

std::vector<ContourInfo> PowerRune::filterByHu(std::vector<ContourInfo>& contours, const cv::Mat& refHu, float huDevThreshold) {
    std::vector<std::pair<ContourInfo, double>> filteredWithDeviation;

    for (auto& contour : contours) {
        cv::Mat hu_moments = contour.huMoments;
        double deviation = contour.hu_moments_deviation(hu_moments, refHu);
        if (deviation <= huDevThreshold) {
            filteredWithDeviation.emplace_back(contour, deviation);
        }
    }

    std::sort(filteredWithDeviation.begin(), filteredWithDeviation.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

    std::vector<ContourInfo> sortedContours;
    for (const auto& pair : filteredWithDeviation) {
        sortedContours.push_back(pair.first);
    }

    return sortedContours;
}

cv::Point2f PowerRune::predict(const cv::Point2f& vector, float radian) {
    cv::Matx22f rotationMatrix(std::cos(radian), -std::sin(radian),
                               std::sin(radian), std::cos(radian));
    cv::Point2f rotatedVector = rotationMatrix * cv::Vec2f(vector.x, vector.y);
    return rotatedVector;
}