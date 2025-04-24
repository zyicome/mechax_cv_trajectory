// 需修改
// 整合 计算buff位置/预测位置



#include "blade.hpp"

// Blade 构造函数，初始化 Blade 对象
// 参数：
//   contour - 传递的轮廓信息对象，包含几何信息（如质心、主轴等）
//   cfg - 配置信息，用于控制相对目标和中心的计算
Blade::Blade(ContourInfo& contour, Configuration& cfg)
{   
    /*
    // 获取轮廓的质心（几何中心）
    cv::Point2f centroid = contour.getCentroid();
    
    // 获取轮廓的最大特征值对应的特征向量（主轴方向）
    cv::Point2f axis = contour.getEigenMax().first;
    
    // 获取轮廓的最大特征值
    float value = contour.getEigenMax().second;
    
    // 获取轮廓的偏度（用于反映轮廓的不对称性）
    cv::Point2f skewness = contour.getSkewness();

    // 计算方向，基于轴和偏度
    // 如果轴和偏度的点积为负，表示两者方向相反，取反方向的轴，否则取原方向
    cv::Point2f orientation = (axis.dot(skewness) < 0) ? axis : -axis;
    
    // 计算特征向量，使用主轴方向和特征值的平方根来放缩特征向量
    cv::Point2f trait = orientation * std::sqrt(value);

    // 初始化当前 Blade 对象的轮廓
    this->contour = contour;
    
    // 计算目标点位置，相对于质心的位置，由 cfg 配置中的比例决定
    this->target = centroid + trait * cfg.target_relative_position;//需击打目标中心
    
    // 计算中心点位置，相对于质心的位置，由 cfg 配置中的比例决定
    this->center = centroid + trait * cfg.center_relative_position;//buff圆心
    
    // 计算目标与中心之间的向量，用于表示目标与中心的相对方向和距离
    this->vector = this->target - this->center;//目标与中心之间的平移向量
    */

    // 如果 contour.center 是一个 cv::Point 类型的数组，取其第一个元素并转换为 cv::Point2f 类型
    cv::Point2f center = cv::Point2f(contour.center);

    cv::Point2f circle_center = cv::Point2f(contour.circle_center);
    
    center.y = center.y;
    circle_center.y = circle_center.y;

    this->contour = contour;
    this->center = center;//扇叶中心
    this->circle_center = circle_center;//圆心中心
    this->vector = center - circle_center;//扇叶中心与圆心之间的平移向量
}
