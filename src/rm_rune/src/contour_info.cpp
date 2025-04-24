// // 略
// // 模板匹配函数 用于识别buff



// #include "contour_info.hpp"

// // ContourInfo 默认构造函数，未做任何初始化
// ContourInfo::ContourInfo()
// {
//     // 空的构造函数，用于创建一个空对象
// }

// // ContourInfo 带参构造函数，用于使用传入的轮廓数据初始化对象
// ContourInfo::ContourInfo(const std::vector<cv::Point>& contour)
// {
//     // 初始化轮廓信息
//     this->contour = contour;
    
//     // 计算轮廓的矩（moments），包括空间矩和中心矩
//     this->moments = cv::moments(contour);
    
//     // 使用矩计算Hu不变矩，用于形状特征分析
//     cv::HuMoments(this->moments, this->huMoments);
// }

// // 设置轮廓并更新轮廓相关信息
// void ContourInfo::setContour(const std::vector<cv::Point>& contour)
// {
//     // 更新轮廓信息
//     this->contour = contour;
    
//     // 重新计算该轮廓的矩
//     this->moments = cv::moments(contour);
    
//     // 重新计算Hu不变矩
//     cv::HuMoments(this->moments, this->huMoments);
// }

// // 获取轮廓的面积，通过 `m00` 矩获取
// double ContourInfo::getArea()
// {
//     // 矩中的 m00 表示轮廓的面积
//     return moments.m00;
// }

// // 获取轮廓的质心，通过空间矩计算质心坐标
// cv::Point2f ContourInfo::getCentroid()
// {
//     // 质心计算：m10/m00 和 m01/m00 分别是 x 和 y 坐标
//     return cv::Point2f(static_cast<float>(this->moments.m10 / this->moments.m00),
//                        static_cast<float>(this->moments.m01 / this->moments.m00));
// }

// // 获取轮廓的主轴方向（最大特征向量）和方差
// std::pair<cv::Vec2f, double> ContourInfo::getEigenMax()
// {
//     // 归一化二阶矩（中心矩），用于构建协方差矩阵
//     double nu_xx = this->moments.mu20 / this->moments.m00;
//     double nu_yy = this->moments.mu02 / this->moments.m00;
//     double nu_xy = this->moments.mu11 / this->moments.m00;

//     // 构建协方差矩阵
//     cv::Matx22f covMatrix(nu_xx, nu_xy, nu_xy, nu_yy);
    
//     // 计算协方差矩阵的特征值和特征向量
//     cv::Mat eigenvalues, eigenvectors;
//     cv::eigen(covMatrix, eigenvalues, eigenvectors);

//     // 假设第一个特征值是最大的
//     int maxEigenvalueIndex = 0;
    
//     // 获取主轴方向的特征向量
//     cv::Vec2f majorAxisVector(eigenvectors.at<float>(maxEigenvalueIndex, 0),
//                               eigenvectors.at<float>(maxEigenvalueIndex, 1));
    
//     // 获取最大特征值（主轴方向的方差）
//     double variance = eigenvalues.at<float>(maxEigenvalueIndex);

//     // 返回主轴方向和方差
//     return std::make_pair(majorAxisVector, variance);
// }

// // 获取轮廓的偏度（Skewness），通过三阶中心矩计算
// cv::Vec2f ContourInfo::getSkewness()
// {
//     // 偏度：通过三阶矩 mu30 和 mu03 计算
//     return cv::Vec2f(static_cast<float>(this->moments.mu30 / this->moments.m00),
//                      static_cast<float>(this->moments.mu03 / this->moments.m00));
// }

// // signMat 函数，用于计算矩阵的符号
// // 输入矩阵：mat
// // 返回：同维度矩阵，元素为 1（正数），-1（负数），或 0（零值）
// cv::Mat signMat(const cv::Mat& mat) {
//     cv::Mat signMat = cv::Mat::zeros(mat.size(), mat.type());
//     for (int i = 0; i < mat.rows; ++i) {
//         for (int j = 0; j < mat.cols; ++j) {
//             double val = mat.at<double>(i, j);
//             if (val > 0) signMat.at<double>(i, j) = 1;
//             else if (val < 0) signMat.at<double>(i, j) = -1;
//             // 如果值为 0，则保持为 0
//         }
//     }
//     return signMat;
// }

// // 计算 Hu 矩的偏差
// // 参数：
// //   hu_moments - 计算得到的 Hu 矩
// //   reference - 参考的 Hu 矩
// // 返回值：Hu 矩与参考矩的偏差值
// float ContourInfo::hu_moments_deviation(cv::Mat hu_moments, cv::Mat reference)
// {
//     // 验证输入矩阵必须具有 7 个元素
//     if (hu_moments.rows * hu_moments.cols != 7 || reference.rows * reference.cols != 7) {
//         throw std::invalid_argument("Both hu_moments and reference must have 7 elements.");
//     }
//     // 验证输入矩阵的类型为 CV_64F
//     if (hu_moments.type() != CV_64F || reference.type() != CV_64F) {
//         throw std::invalid_argument("Input arrays must be of type double (CV_64F).");
//     }

//     cv::Mat log_hu_moments, log_reference;
    
//     // 对 Hu 矩进行对数变换
//     cv::log(cv::abs(hu_moments), log_hu_moments);
//     log_hu_moments = -log_hu_moments.mul(signMat(hu_moments));
//     cv::log(cv::abs(reference), log_reference);
//     log_reference = -log_reference.mul(signMat(reference));

//     // 计算 Hu 矩的偏差
//     float deviation = 0.0;
//     for (int i = 0; i < 6; ++i) {
//         deviation += std::abs(1.0 / log_hu_moments.at<double>(i) - 1.0 / log_reference.at<double>(i));
//     }

//     return deviation;  // 返回计算的偏差值
// }
