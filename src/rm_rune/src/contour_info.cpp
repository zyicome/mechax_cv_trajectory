#include "contour_info.hpp"

ContourInfo::ContourInfo()
{

}

ContourInfo::ContourInfo(const std::vector<cv::Point>& contour)
{
    this->contour = contour;
    this->moments = cv::moments(contour);
    cv::HuMoments(this->moments, this->huMoments);
}

void ContourInfo::setContour(const std::vector<cv::Point>& contour)
{
    this->contour = contour;
    this->moments = cv::moments(contour);
    cv::HuMoments(this->moments, this->huMoments);
}

double ContourInfo::getArea()
{
    return moments.m00;
}

cv::Point2f ContourInfo::getCentroid()
{
        return cv::Point2f(static_cast<float>(this->moments.m10 / this->moments.m00),
                           static_cast<float>(this->moments.m01 / this->moments.m00));
}

std::pair<cv::Vec2f, double> ContourInfo::getEigenMax()
{
    double nu_xx = this->moments.mu20 / this->moments.m00;
    double nu_yy = this->moments.mu02 / this->moments.m00;
    double nu_xy = this->moments.mu11 / this->moments.m00;

    cv::Matx22f covMatrix(nu_xx, nu_xy, nu_xy, nu_yy);
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(covMatrix, eigenvalues, eigenvectors);

    int maxEigenvalueIndex = 0; // Assuming the first eigenvalue is the largest
    cv::Vec2f majorAxisVector(eigenvectors.at<float>(maxEigenvalueIndex, 0),
                                  eigenvectors.at<float>(maxEigenvalueIndex, 1));
    double variance = eigenvalues.at<float>(maxEigenvalueIndex);

    return std::make_pair(majorAxisVector, variance);
}

cv::Vec2f ContourInfo::getSkewness()
{
        return cv::Vec2f(static_cast<float>(this->moments.mu30 / this->moments.m00),
                         static_cast<float>(this->moments.mu03 / this->moments.m00));
}

cv::Mat signMat(const cv::Mat& mat) {
    cv::Mat signMat = cv::Mat::zeros(mat.size(), mat.type());
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            double val = mat.at<double>(i, j);
            if (val > 0) signMat.at<double>(i, j) = 1;
            else if (val < 0) signMat.at<double>(i, j) = -1;
            // If val == 0, signMat.at<double>(i, j) remains 0
        }
    }
    return signMat;
}

//计算Hu矩偏差。
//@param hu_moments: 图像的Hu矩。
//@param reference: 参考Hu矩。
//@return: 计算得到的Hu矩偏差值。
float ContourInfo::hu_moments_deviation(cv::Mat hu_moments, cv::Mat reference)
{
    // Validation of input parameters
    if (hu_moments.rows * hu_moments.cols != 7 || reference.rows * reference.cols != 7) {
        throw std::invalid_argument("Both hu_moments and reference must have 7 elements.");
    }
    if (hu_moments.type() != CV_64F || reference.type() != CV_64F) {
        throw std::invalid_argument("Input arrays must be of type double (CV_64F).");
    }

    cv::Mat log_hu_moments, log_reference;
    // Logarithmic transformation
    cv::log(cv::abs(hu_moments), log_hu_moments);
    log_hu_moments = -log_hu_moments.mul(signMat(hu_moments));
    cv::log(cv::abs(reference), log_reference);
    log_reference = -log_reference.mul(signMat(reference));

    float deviation = 0.0;
    for (int i = 0; i < 6; ++i) {
        deviation += std::abs(1.0 / log_hu_moments.at<double>(i) - 1.0 / log_reference.at<double>(i));
    }

    return deviation;
}
