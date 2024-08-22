#include "blade.hpp"

Blade::Blade(ContourInfo& contour,Configuration& cfg)
{
    cv::Point2f centroid = contour.getCentroid();
    cv::Point2f axis = contour.getEigenMax().first;
    float value = contour.getEigenMax().second;
    cv::Point2f skewness = contour.getSkewness();

    // Calculate orientation based on axis and skewness
    cv::Point2f orientation = (axis.dot(skewness) < 0) ? axis : -axis;
    cv::Point2f trait = orientation * std::sqrt(value);

    this->contour = contour;
    this->target = centroid + trait * cfg.target_relative_position;
    this->center = centroid + trait * cfg.center_relative_position;
    this->vector = this->target - this->center;
}